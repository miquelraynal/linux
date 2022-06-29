// SPDX-License-Identifier: GPL-2.0-only
/*
 * IEEE 802.15.4 scanning management
 *
 * Copyright (C) 2021 Qorvo US, Inc
 * Authors:
 *   - David Girault <david.girault@qorvo.com>
 *   - Miquel Raynal <miquel.raynal@bootlin.com>
 */

#include <linux/module.h>
#include <linux/rtnetlink.h>
#include <net/mac802154.h>

#include "ieee802154_i.h"
#include "driver-ops.h"
#include "../ieee802154/nl802154.h"

#define IEEE802154_BEACON_MHR_SZ 13
#define IEEE802154_BEACON_PL_SZ 4
#define IEEE802154_MAC_CMD_MHR_SZ 23
#define IEEE802154_MAC_CMD_PL_SZ 1
#define IEEE802154_BEACON_SKB_SZ (IEEE802154_BEACON_MHR_SZ + \
				  IEEE802154_BEACON_PL_SZ)
#define IEEE802154_MAC_CMD_SKB_SZ (IEEE802154_MAC_CMD_MHR_SZ + \
				   IEEE802154_MAC_CMD_PL_SZ)

static int mac802154_send_scan_done(struct ieee802154_local *local, u8 cmd)
{
	struct cfg802154_scan_request *scan_req;
	struct wpan_phy *wpan_phy;
	struct wpan_dev *wpan_dev;

	scan_req = rcu_dereference_protected(local->scan_req,
					     lockdep_is_held(&local->scan_lock));
	wpan_phy = scan_req->wpan_phy;
	wpan_dev = scan_req->wpan_dev;

	cfg802154_flush_known_coordinators(wpan_dev);

	return nl802154_send_scan_done(wpan_phy, wpan_dev, scan_req, cmd);
}

static int mac802154_end_of_scan(struct ieee802154_local *local,
				 struct ieee802154_sub_if_data *sdata,
				 bool aborted)
{
	u8 cmd;

	drv_set_channel(local, local->phy->current_page,
			local->phy->current_channel);
	ieee802154_configure_durations(local->phy, local->phy->current_page,
				       local->phy->current_channel);

	clear_bit(IEEE802154_IS_SCANNING, &local->ongoing);
	drv_stop(local);
	synchronize_net();
	sdata->required_filtering = sdata->iface_default_filtering;
	drv_start(local, sdata->required_filtering, &local->addr_filt);
	ieee802154_mlme_op_post(local);
	module_put(local->hw.parent->driver->owner);

	cmd = aborted ? NL802154_CMD_ABORT_SCAN : NL802154_CMD_SCAN_DONE;

	return mac802154_send_scan_done(local, cmd);
}

int mac802154_abort_scan_locked(struct ieee802154_sub_if_data *sdata)
{
	struct ieee802154_local *local = sdata->local;

	lockdep_assert_held(&local->scan_lock);

	if (!mac802154_is_scanning(local))
		return -ESRCH;

	cancel_delayed_work(&local->scan_work);

	return mac802154_end_of_scan(local, sdata, true);
}

static unsigned int mac802154_scan_get_channel_time(u8 duration_order,
						    u8 symbol_duration)
{
	u64 base_super_frame_duration = (u64)symbol_duration *
		IEEE802154_SUPERFRAME_PERIOD * IEEE802154_SLOT_PERIOD;

	return usecs_to_jiffies(base_super_frame_duration *
				(BIT(duration_order) + 1));
}

void mac802154_flush_queued_beacons(struct ieee802154_local *local)
{
	struct cfg802154_mac_pkt *beacon, *tmp;

	lockdep_assert_held(&local->scan_lock);

	list_for_each_entry_safe(beacon, tmp, &local->rx_beacon_list, node) {
		list_del(&beacon->node);
		kfree_skb(beacon->skb);
		kfree(beacon);
	}
}

static int mac802154_scan_prepare_beacon_req(struct ieee802154_local *local)
{
	memset(&local->scan_beacon_req, 0, sizeof(local->scan_beacon_req));
	local->scan_beacon_req.mhr.fc.type = IEEE802154_FC_TYPE_MAC_CMD;
	local->scan_beacon_req.mhr.fc.dest_addr_mode = IEEE802154_SHORT_ADDRESSING;
	local->scan_beacon_req.mhr.fc.version = IEEE802154_2003_STD;
	local->scan_beacon_req.mhr.fc.source_addr_mode = IEEE802154_NO_ADDRESSING;
	local->scan_beacon_req.mhr.dest.mode = IEEE802154_ADDR_SHORT;
	local->scan_beacon_req.mhr.dest.pan_id = cpu_to_le16(IEEE802154_PANID_BROADCAST);
	local->scan_beacon_req.mhr.dest.short_addr = cpu_to_le16(IEEE802154_ADDR_BROADCAST);
	local->scan_beacon_req.mac_pl.cmd_id = IEEE802154_CMD_BEACON_REQ;

	return 0;
}

static int mac802154_transmit_beacon_req_locked(struct ieee802154_local *local,
						struct ieee802154_sub_if_data *sdata)
{
	struct sk_buff *skb;
	int ret;

	lockdep_assert_held(&local->scan_lock);

	skb = alloc_skb(IEEE802154_MAC_CMD_SKB_SZ, GFP_KERNEL);
	if (!skb)
		return -ENOBUFS;

	skb->dev = sdata->dev;

	ret = ieee802154_mac_cmd_push(skb, &local->scan_beacon_req, NULL, 0);
	if (ret) {
		kfree_skb(skb);
		return ret;
	}

	return ieee802154_mlme_tx(local, sdata, skb);
}

void mac802154_scan_worker(struct work_struct *work)
{
	struct ieee802154_local *local =
		container_of(work, struct ieee802154_local, scan_work.work);
	struct cfg802154_scan_request *scan_req;
	struct ieee802154_sub_if_data *sdata;
	unsigned int scan_duration;
	unsigned long chan;
	int ret;

	/* In practice we don't really need the rtnl here, besides for the
	 * drv_set_channel() operation. Unfortunately, as the rtnl is always
	 * taken before any other lock, we must acquire it before scan_lock() to
	 * avoid circular dependencies.
	 */
	rtnl_lock();
	mutex_lock(&local->scan_lock);

	if (!mac802154_is_scanning(local))
		goto unlock_mutex;

	scan_req = rcu_dereference_protected(local->scan_req,
					     lockdep_is_held(&local->scan_lock));
	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(scan_req->wpan_dev);

	if (local->suspended || !ieee802154_sdata_running(sdata))
		goto queue_work;

	do {
		chan = find_next_bit((const unsigned long *)&scan_req->channels,
				     IEEE802154_MAX_CHANNEL + 1,
				     local->scan_channel_idx + 1);

		/* If there are no more channels left, complete the scan */
		if (chan > IEEE802154_MAX_CHANNEL) {
			mac802154_end_of_scan(local, sdata, false);
			goto unlock_mutex;
		}

		/* Bypass the stack on purpose. As the channel change cannot be
		 * made atomic with regard to the incoming beacon flow, we flush
		 * the beacons list after changing the channel and before
		 * releasing the scan lock, to avoid processing beacons which
		 * have been received during this time frame.
		 */
		ret = drv_set_channel(local, scan_req->page, chan);
		local->scan_channel_idx = chan;
		ieee802154_configure_durations(local->phy, scan_req->page, chan);
		mac802154_flush_queued_beacons(local);
	} while (ret);

	if (scan_req->type == NL802154_SCAN_ACTIVE) {
		ret = mac802154_transmit_beacon_req_locked(local, sdata);
		if (ret)
			dev_err(&sdata->dev->dev,
				"Error when transmitting beacon request (%d)\n", ret);
	}

queue_work:
	scan_duration = mac802154_scan_get_channel_time(scan_req->duration,
							local->phy->symbol_duration);
	dev_dbg(&sdata->dev->dev,
		"Scan channel %lu of page %u for %ums\n",
		chan, scan_req->page, jiffies_to_msecs(scan_duration));
	queue_delayed_work(local->mac_wq, &local->scan_work, scan_duration);

unlock_mutex:
	mutex_unlock(&local->scan_lock);
	rtnl_unlock();
}

int mac802154_trigger_scan_locked(struct ieee802154_sub_if_data *sdata,
				  struct cfg802154_scan_request *request)
{
	struct ieee802154_local *local = sdata->local;
	int ret;

	lockdep_assert_held(&local->scan_lock);

	if (mac802154_is_scanning(local))
		return -EBUSY;

	if (request->type != NL802154_SCAN_PASSIVE &&
	    request->type != NL802154_SCAN_ACTIVE)
		return -EOPNOTSUPP;

	/* Store scanning parameters */
	rcu_assign_pointer(local->scan_req, request);

	/* Starting a background job, ensure the module cannot be removed */
	if (!try_module_get(local->hw.parent->driver->owner))
		return -ENODEV;

	/* Software scanning requires to set promiscuous mode, so we need to
	 * pause the Tx queue during the entire operation.
	 */
	ieee802154_mlme_op_pre(local);

	drv_stop(local);
	synchronize_net();
	sdata->required_filtering = IEEE802154_FILTERING_3_SCAN;
	ret = drv_start(local, sdata->required_filtering, &local->addr_filt);
	if (ret) {
		module_put(local->hw.parent->driver->owner);
		ieee802154_mlme_op_post(local);
		return ret;
	}

	local->scan_channel_idx = -1;
	set_bit(IEEE802154_IS_SCANNING, &local->ongoing);
	if (request->type == NL802154_SCAN_ACTIVE)
		mac802154_scan_prepare_beacon_req(local);

	queue_delayed_work(local->mac_wq, &local->scan_work, 0);

	nl802154_send_start_scan(local->scan_req->wpan_phy,
				 local->scan_req->wpan_dev);

	return 0;
}

int mac802154_process_beacon(struct ieee802154_local *local,
			     struct sk_buff *skb)
{
	struct ieee802154_beacon_hdr *bh = (void *)skb->data;
	struct ieee802154_addr *src = &mac_cb(skb)->source;
	struct cfg802154_scan_request *scan_req;
	struct ieee802154_coord_desc *desc;

	if (skb->len != sizeof(*bh))
		return -EINVAL;

	if (unlikely(src->mode == IEEE802154_ADDR_NONE))
		return -EINVAL;

	scan_req = rcu_dereference_protected(local->scan_req,
					     &local->scan_lock);
	if (unlikely(!scan_req))
		return -EINVAL;

	dev_dbg(&skb->dev->dev,
		"BEACON received on channel %d of page %d\n",
		local->scan_channel_idx, scan_req->page);

	/* Parse beacon, create PAN information and forward to upper layers */
	desc = cfg802154_alloc_coordinator(src);
	if (!desc)
		return -ENOMEM;

	desc->page = scan_req->page;
	desc->channel = local->scan_channel_idx;
	desc->link_quality = mac_cb(skb)->lqi;
	desc->superframe_spec = get_unaligned_le16(skb->data);
	desc->gts_permit = bh->gts_permit;
	cfg802154_record_coordinator(scan_req->wpan_phy, scan_req->wpan_dev, desc);

	return 0;
}

static int mac802154_transmit_beacon_locked(struct ieee802154_local *local,
					    struct wpan_dev *wpan_dev)
{
	struct cfg802154_beacon_request *beacon_req;
	struct ieee802154_sub_if_data *sdata;
	struct sk_buff *skb;
	int ret;

	lockdep_assert_held(&local->beacon_lock);

	/* Update the sequence number */
	local->beacon.mhr.seq = atomic_inc_return(&wpan_dev->bsn) & 0xFF;

	skb = alloc_skb(IEEE802154_BEACON_SKB_SZ, GFP_KERNEL);
	if (!skb)
		return -ENOBUFS;

	beacon_req = rcu_dereference_protected(local->beacon_req,
					       &local->beacon_lock);
	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(beacon_req->wpan_dev);
	skb->dev = sdata->dev;

	ret = ieee802154_beacon_push(skb, &local->beacon);
	if (ret) {
		kfree_skb(skb);
		return ret;
	}

	return ieee802154_mlme_tx_one(local, sdata, skb);
}

void mac802154_beacon_worker(struct work_struct *work)
{
	struct ieee802154_local *local =
		container_of(work, struct ieee802154_local, beacon_work.work);
	struct cfg802154_beacon_request *beacon_req;
	struct ieee802154_sub_if_data *sdata;
	int ret;

	mutex_lock(&local->beacon_lock);

	if (!mac802154_is_beaconing(local))
		goto unlock_mutex;

	if (local->suspended)
		goto queue_work;

	beacon_req = rcu_dereference_protected(local->beacon_req,
					       &local->beacon_lock);
	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(beacon_req->wpan_dev);

	dev_dbg(&sdata->dev->dev, "Sending beacon\n");
	ret = mac802154_transmit_beacon_locked(local, beacon_req->wpan_dev);
	if (ret)
		dev_err(&sdata->dev->dev,
			"Beacon could not be transmitted (%d)\n", ret);

queue_work:
	if (beacon_req->interval < IEEE802154_ACTIVE_SCAN_DURATION)
		queue_delayed_work(local->mac_wq, &local->beacon_work,
				   local->beacon_interval);

unlock_mutex:
	mutex_unlock(&local->beacon_lock);
}

static void mac802154_end_beaconing(struct ieee802154_local *local)
{
	struct cfg802154_beacon_request *beacon_req;
	struct ieee802154_sub_if_data *sdata;

	beacon_req = rcu_dereference_protected(local->beacon_req,
					       &local->beacon_lock);
	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(beacon_req->wpan_dev);

	if (beacon_req->interval < IEEE802154_ACTIVE_SCAN_DURATION)
		cancel_delayed_work(&local->beacon_work);

	clear_bit(IEEE802154_IS_BEACONING, &local->ongoing);
	nl802154_end_beaconing(beacon_req->wpan_dev, beacon_req);
}

int mac802154_stop_beacons_locked(struct ieee802154_local *local)
{
	lockdep_assert_held(&local->beacon_lock);

	if (!mac802154_is_beaconing(local))
		return -ESRCH;

	mac802154_end_beaconing(local);

	return 0;
}

int mac802154_send_beacons_locked(struct ieee802154_sub_if_data *sdata,
				  struct cfg802154_beacon_request *request)
{
	struct ieee802154_local *local = sdata->local;
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;

	lockdep_assert_held(&local->beacon_lock);

	if (mac802154_is_beaconing(local))
		mac802154_stop_beacons_locked(local);

	/* Store scanning parameters */
	rcu_assign_pointer(local->beacon_req, request);

	set_bit(IEEE802154_IS_BEACONING, &local->ongoing);

	memset(&local->beacon, 0, sizeof(local->beacon));
	local->beacon.mhr.fc.type = IEEE802154_FC_TYPE_BEACON;
	local->beacon.mhr.fc.security_enabled = 0;
	local->beacon.mhr.fc.frame_pending = 0;
	local->beacon.mhr.fc.ack_request = 0;
	local->beacon.mhr.fc.intra_pan = 0;
	local->beacon.mhr.fc.dest_addr_mode = IEEE802154_NO_ADDRESSING;
	local->beacon.mhr.fc.version = IEEE802154_2003_STD;
	local->beacon.mhr.fc.source_addr_mode = IEEE802154_EXTENDED_ADDRESSING;
	atomic_set(&request->wpan_dev->bsn, -1);
	local->beacon.mhr.source.mode = IEEE802154_ADDR_LONG;
	local->beacon.mhr.source.pan_id = cpu_to_le16(request->wpan_dev->pan_id);
	local->beacon.mhr.source.extended_addr = cpu_to_le64(request->wpan_dev->extended_addr);
	local->beacon.mac_pl.beacon_order = request->interval;
	if (request->interval <= IEEE802154_MAX_SCAN_DURATION)
		local->beacon.mac_pl.superframe_order = request->interval;
	local->beacon.mac_pl.final_cap_slot = 0xf;
	local->beacon.mac_pl.battery_life_ext = 0;
	local->beacon.mac_pl.pan_coordinator = !wpan_dev->parent;
	local->beacon.mac_pl.assoc_permit = 1;

	if (request->interval == IEEE802154_ACTIVE_SCAN_DURATION)
		return 0;

	/* Start the beacon work */
	local->beacon_interval =
		mac802154_scan_get_channel_time(request->interval,
						request->wpan_phy->symbol_duration);
	queue_delayed_work(local->mac_wq, &local->beacon_work, 0);

	return 0;
}

int mac802154_perform_association(struct ieee802154_sub_if_data *sdata,
				  struct ieee802154_pan_device *coord,
				  __le16 *short_addr)
{
	u64 ceaddr = swab64((__force u64)coord->extended_addr);
	struct ieee802154_association_req_frame frame = {};
	struct ieee802154_local *local = sdata->local;
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;
	struct sk_buff *skb;
	int ret;

	frame.mhr.fc.type = IEEE802154_FC_TYPE_MAC_CMD;
	frame.mhr.fc.security_enabled = 0;
	frame.mhr.fc.frame_pending = 0;
	frame.mhr.fc.ack_request = 1; /* We always expect an ack here */
	frame.mhr.fc.intra_pan = 0;
	frame.mhr.fc.dest_addr_mode = (coord->mode == IEEE802154_ADDR_LONG) ?
		IEEE802154_EXTENDED_ADDRESSING : IEEE802154_SHORT_ADDRESSING;
	frame.mhr.fc.version = IEEE802154_2003_STD;
	frame.mhr.fc.source_addr_mode = IEEE802154_EXTENDED_ADDRESSING;
	frame.mhr.source.mode = IEEE802154_ADDR_LONG;
	frame.mhr.source.pan_id = cpu_to_le16(IEEE802154_PANID_BROADCAST);
	frame.mhr.source.extended_addr = cpu_to_le64(wpan_dev->extended_addr);
	frame.mhr.dest.mode = coord->mode;
	frame.mhr.dest.pan_id = cpu_to_le16(coord->pan_id);
	if (coord->mode == IEEE802154_ADDR_LONG)
		frame.mhr.dest.extended_addr = cpu_to_le64(coord->extended_addr);
	else
		frame.mhr.dest.short_addr = cpu_to_le16(coord->short_addr);
	frame.mhr.seq = atomic_inc_return(&wpan_dev->dsn) & 0xFF;
	frame.mac_pl.cmd_id = IEEE802154_CMD_ASSOCIATION_REQ;
	frame.assoc_req_pl.device_type = 1;
	frame.assoc_req_pl.power_source = 1;
	frame.assoc_req_pl.rx_on_when_idle = 1;
	frame.assoc_req_pl.alloc_addr = 1;

	skb = alloc_skb(IEEE802154_MAC_CMD_SKB_SZ + sizeof(frame.assoc_req_pl),
			GFP_KERNEL);
	if (!skb)
		return -ENOBUFS;

	skb->dev = sdata->dev;

	ret = ieee802154_mac_cmd_push(skb, &frame, &frame.assoc_req_pl,
				      sizeof(frame.assoc_req_pl));
	if (ret) {
		kfree_skb(skb);
		return ret;
	}

	local->assoc_dev = coord;
	reinit_completion(&local->assoc_done);
	set_bit(IEEE802154_IS_ASSOCIATING, &local->ongoing);

	ret = ieee802154_mlme_tx_one(local, sdata, skb);
	if (ret) {
		if (ret > 0)
			ret = (ret == IEEE802154_NO_ACK) ? -EREMOTEIO : -EIO;
		dev_warn(&sdata->dev->dev,
			 "No ASSOC REQ ACK received from %8phC\n", &ceaddr);
		goto clear_assoc;
	}

	ret = wait_for_completion_killable_timeout(&local->assoc_done, 10 * HZ);
	if (ret <= 0) {
		dev_warn(&sdata->dev->dev,
			 "No ASSOC RESP received from %8phC\n", &ceaddr);
		ret = -ETIMEDOUT;
		goto clear_assoc;
	}

	if (local->assoc_status != IEEE802154_ASSOCIATION_SUCCESSFUL) {
		if (local->assoc_status == IEEE802154_PAN_AT_CAPACITY)
			ret = -ERANGE;
		else
			ret = -EPERM;

		dev_warn(&sdata->dev->dev,
			 "Negative ASSOC RESP received from %8phC: %s\n", &ceaddr,
			 local->assoc_status == IEEE802154_PAN_AT_CAPACITY ?
			 "PAN at capacity" : "access denied");
	}

	ret = 0;
	*short_addr = local->assoc_addr;

clear_assoc:
	clear_bit(IEEE802154_IS_ASSOCIATING, &local->ongoing);
	local->assoc_dev = NULL;

	return ret;
}

int mac802154_process_association_resp(struct ieee802154_sub_if_data *sdata,
				       struct sk_buff *skb)
{
	struct ieee802154_addr *src = &mac_cb(skb)->source;
	struct ieee802154_addr *dest = &mac_cb(skb)->dest;
	u64 deaddr = swab64((__force u64)dest->extended_addr);
	struct ieee802154_local *local = sdata->local;
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;
	struct ieee802154_assoc_resp_pl resp_pl = {};

	if (skb->len != sizeof(resp_pl))
		return -EINVAL;

	if (unlikely(src->mode != IEEE802154_EXTENDED_ADDRESSING ||
		     dest->mode != IEEE802154_EXTENDED_ADDRESSING))
		return -EINVAL;

	if (unlikely(dest->extended_addr != wpan_dev->extended_addr ||
		     src->extended_addr != local->assoc_dev->extended_addr))
		return -ENODEV;

	memcpy(&resp_pl, skb->data, sizeof(resp_pl));
	local->assoc_addr = le16_to_cpu(resp_pl.short_addr);
	local->assoc_status = resp_pl.status;

	dev_dbg(&skb->dev->dev,
		"ASSOC RESP 0x%x received from %8phC, getting short address %04x\n",
		local->assoc_status, &deaddr, le16_to_cpu(local->assoc_addr));

	complete(&local->assoc_done);

	return 0;
}

int mac802154_send_disassociation_notif(struct ieee802154_sub_if_data *sdata,
					struct ieee802154_pan_device *target,
					u8 reason)
{
	struct ieee802154_disassociation_notif_frame frame = {};
	u64 teaddr = swab64((__force u64)target->extended_addr);
	struct ieee802154_local *local = sdata->local;
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;
	struct sk_buff *skb;
	int ret;

	frame.mhr.fc.type = IEEE802154_FC_TYPE_MAC_CMD;
	frame.mhr.fc.security_enabled = 0;
	frame.mhr.fc.frame_pending = 0;
	frame.mhr.fc.ack_request = 1;
	frame.mhr.fc.intra_pan = 1;
	frame.mhr.fc.dest_addr_mode = (target->mode == IEEE802154_ADDR_LONG) ?
		IEEE802154_EXTENDED_ADDRESSING : IEEE802154_SHORT_ADDRESSING;
	frame.mhr.fc.version = IEEE802154_2003_STD;
	frame.mhr.fc.source_addr_mode = IEEE802154_EXTENDED_ADDRESSING;
	frame.mhr.source.mode = IEEE802154_ADDR_LONG;
	frame.mhr.source.pan_id = cpu_to_le16(wpan_dev->pan_id);
	frame.mhr.source.extended_addr = cpu_to_le64(wpan_dev->extended_addr);
	frame.mhr.dest.mode = target->mode;
	frame.mhr.dest.pan_id = cpu_to_le16(wpan_dev->pan_id);
	if (target->mode == IEEE802154_ADDR_LONG)
		frame.mhr.dest.extended_addr = cpu_to_le64(target->extended_addr);
	else
		frame.mhr.dest.short_addr = cpu_to_le16(target->short_addr);
	frame.mhr.seq = atomic_inc_return(&wpan_dev->dsn) & 0xFF;
	frame.mac_pl.cmd_id = IEEE802154_CMD_DISASSOCIATION_NOTIFY;
	frame.disassoc_pl = reason;

	skb = alloc_skb(IEEE802154_MAC_CMD_SKB_SZ + sizeof(frame.disassoc_pl),
			GFP_KERNEL);
	if (!skb)
		return -ENOBUFS;

	skb->dev = sdata->dev;

	ret = ieee802154_mac_cmd_push(skb, &frame, &frame.disassoc_pl,
				      sizeof(frame.disassoc_pl));
	if (ret) {
		kfree_skb(skb);
		return ret;
	}

	ret = ieee802154_mlme_tx_one(local, sdata, skb);
	if (ret) {
		dev_warn(&sdata->dev->dev,
			 "No DISASSOC ACK received from %8phC\n", &teaddr);
		if (ret > 0)
			ret = (ret == IEEE802154_NO_ACK) ? -EREMOTEIO : -EIO;
		return ret;
	}

	dev_dbg(&sdata->dev->dev, "DISASSOC ACK received from %8phC\n", &teaddr);
	return 0;
}

static int
mac802154_send_association_resp(struct ieee802154_sub_if_data *sdata,
				struct ieee802154_pan_device *target,
				struct ieee802154_assoc_resp_pl *assoc_resp_pl)
{
	u64 teaddr = swab64((__force u64)target->extended_addr);
	struct ieee802154_association_resp_frame frame = {};
	struct ieee802154_local *local = sdata->local;
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;
	struct sk_buff *skb;
	int ret;

	frame.mhr.fc.type = IEEE802154_FC_TYPE_MAC_CMD;
	frame.mhr.fc.security_enabled = 0;
	frame.mhr.fc.frame_pending = 0;
	frame.mhr.fc.ack_request = 1; /* We always expect an ack here */
	frame.mhr.fc.intra_pan = 1;
	frame.mhr.fc.dest_addr_mode = IEEE802154_EXTENDED_ADDRESSING;
	frame.mhr.fc.version = IEEE802154_2003_STD;
	frame.mhr.fc.source_addr_mode = IEEE802154_EXTENDED_ADDRESSING;
	frame.mhr.seq = 10;
	frame.mhr.source.mode = IEEE802154_ADDR_LONG;
	frame.mhr.source.extended_addr = cpu_to_le64(wpan_dev->extended_addr);
	frame.mhr.dest.mode = IEEE802154_ADDR_LONG;
	frame.mhr.dest.pan_id = cpu_to_le16(wpan_dev->pan_id);
	frame.mhr.dest.extended_addr = cpu_to_le64(target->extended_addr);
	frame.mhr.seq = atomic_inc_return(&wpan_dev->dsn) & 0xFF;
	frame.mac_pl.cmd_id = IEEE802154_CMD_ASSOCIATION_RESP;

	skb = alloc_skb(IEEE802154_MAC_CMD_SKB_SZ + sizeof(*assoc_resp_pl),
			GFP_KERNEL);
	if (!skb)
		return -ENOBUFS;

	skb->dev = sdata->dev;

	ret = ieee802154_mac_cmd_push(skb, &frame, assoc_resp_pl,
				      sizeof(*assoc_resp_pl));
	if (ret) {
		kfree_skb(skb);
		return ret;
	}

	ret = ieee802154_mlme_tx(local, sdata, skb);
	if (ret) {
		dev_warn(&sdata->dev->dev,
			 "No ASSOC RESP ACK received from %8phC\n", &teaddr);
		if (ret > 0)
			ret = (ret == IEEE802154_NO_ACK) ? -EREMOTEIO : -EIO;
		return ret;
	}

	return 0;
}

int mac802154_process_association_req(struct ieee802154_sub_if_data *sdata,
				      struct sk_buff *skb)
{
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;
	struct ieee802154_addr *src = &mac_cb(skb)->source;
	struct ieee802154_addr *dest = &mac_cb(skb)->dest;
	struct ieee802154_assoc_resp_pl assoc_resp_pl = {};
	struct ieee802154_assoc_req_pl assoc_req_pl;
	struct ieee802154_pan_device *child, *exchild;
	struct ieee802154_addr tmp = {};
	u64 ceaddr;
	int ret;

	if (skb->len != sizeof(assoc_req_pl))
		return -EINVAL;

	if (unlikely(src->mode != IEEE802154_EXTENDED_ADDRESSING))
		return -EINVAL;

	if (unlikely(dest->pan_id != wpan_dev->pan_id))
		return -ENODEV;

	if (dest->mode == IEEE802154_EXTENDED_ADDRESSING &&
	    unlikely(dest->extended_addr != wpan_dev->extended_addr))
		return -ENODEV;
	else if (dest->mode == IEEE802154_SHORT_ADDRESSING &&
		 unlikely(dest->short_addr != wpan_dev->short_addr))
		return -ENODEV;

	mutex_lock(&wpan_dev->association_lock);

	memcpy(&assoc_req_pl, skb->data, sizeof(assoc_req_pl));
	if (assoc_req_pl.assoc_type) {
		dev_err(&skb->dev->dev, "Fast associations not supported yet\n");
		ret = -ENOTSUPP;
		goto unlock;
	}

	child = kzalloc(sizeof(*child), GFP_KERNEL);
	if (!child) {
		ret = -ENOMEM;
		goto unlock;
	}

	child->extended_addr = src->extended_addr;
	child->mode = IEEE802154_EXTENDED_ADDRESSING;
	ceaddr = swab64(child->extended_addr);

	if (wpan_dev->nchildren >= wpan_dev->max_associations) {
		if (!wpan_dev->max_associations)
			assoc_resp_pl.status = IEEE802154_PAN_ACCESS_DENIED;
		else
			assoc_resp_pl.status = IEEE802154_PAN_AT_CAPACITY;
		assoc_resp_pl.short_addr = cpu_to_le16(IEEE802154_ADDR_SHORT_BROADCAST);
		dev_dbg(&sdata->dev->dev,
			"Refusing ASSOC REQ from child %8phC, %s\n", &ceaddr,
			assoc_resp_pl.status == IEEE802154_PAN_ACCESS_DENIED ?
			"access denied" : "too many children");
	} else {
		assoc_resp_pl.status = IEEE802154_ASSOCIATION_SUCCESSFUL;
		if (assoc_req_pl.alloc_addr) {
			assoc_resp_pl.short_addr = cfg802154_get_free_short_addr(wpan_dev);
			child->mode = IEEE802154_SHORT_ADDRESSING;
		} else {
			assoc_resp_pl.short_addr = cpu_to_le16(IEEE802154_ADDR_SHORT_UNSPEC);
		}
		child->short_addr = assoc_resp_pl.short_addr;
		dev_dbg(&sdata->dev->dev,
			"Accepting ASSOC REQ from child %8phC, providing short address 0x%04x\n",
			&ceaddr, le16_to_cpu(child->short_addr));
	}

	ret = mac802154_send_association_resp(sdata, child, &assoc_resp_pl);
	if (ret || assoc_resp_pl.status != IEEE802154_ASSOCIATION_SUCCESSFUL) {
		kfree(child);
		goto unlock;
	}

	dev_dbg(&sdata->dev->dev,
		"Successful association with new child %8phC\n", &ceaddr);

	/* Ensure this child is not already associated (might happen due to
	 * retransmissions), in this case drop the ex structure.
	 */
	tmp.mode = child->mode;
	if (tmp.mode == IEEE802154_SHORT_ADDRESSING)
		tmp.short_addr = child->short_addr;
	else
		tmp.extended_addr = child->extended_addr;
	exchild = cfg802154_device_is_child(wpan_dev, &tmp);
	if (exchild) {
		dev_dbg(&sdata->dev->dev,
			"Child %8phC was already known\n", &ceaddr);
		list_del(&exchild->node);
	}

	list_add(&child->node, &wpan_dev->children);
	wpan_dev->nchildren++;
	wpan_dev->association_generation++;

unlock:
	mutex_unlock(&wpan_dev->association_lock);
	return ret;
}

int mac802154_process_disassociation_notif(struct ieee802154_sub_if_data *sdata,
					   struct sk_buff *skb)
{
	struct ieee802154_addr *src = &mac_cb(skb)->source;
	struct ieee802154_addr *dest = &mac_cb(skb)->dest;
	struct wpan_dev *wpan_dev = &sdata->wpan_dev;
	struct ieee802154_pan_device *child;
	struct ieee802154_addr target;
	bool parent;
	u64 teaddr;

	if (skb->len != sizeof(u8))
		return -EINVAL;

	if (unlikely(src->mode != IEEE802154_EXTENDED_ADDRESSING))
		return -EINVAL;

	if (dest->mode == IEEE802154_EXTENDED_ADDRESSING &&
	    unlikely(dest->extended_addr != wpan_dev->extended_addr))
		return -ENODEV;
	else if (dest->mode == IEEE802154_SHORT_ADDRESSING &&
		 unlikely(dest->short_addr != wpan_dev->short_addr))
		return -ENODEV;

	target.mode = IEEE802154_EXTENDED_ADDRESSING;
	target.extended_addr = src->extended_addr;
	teaddr = swab64((__force u64)target.extended_addr);
	dev_dbg(&skb->dev->dev, "Processing DISASSOC NOTIF from %8phC\n", &teaddr);

	mutex_lock(&wpan_dev->association_lock);
	parent = cfg802154_device_is_parent(wpan_dev, &target);
	if (!parent)
		child = cfg802154_device_is_child(wpan_dev, &target);
	if (!parent && !child) {
		mutex_unlock(&wpan_dev->association_lock);
		return -EINVAL;
	}

	if (parent) {
		kfree(wpan_dev->parent);
		wpan_dev->parent = NULL;
	} else {
		list_del(&child->node);
		kfree(child);
		wpan_dev->nchildren--;
	}
	wpan_dev->association_generation++;

	mutex_unlock(&wpan_dev->association_lock);

	return 0;
}
