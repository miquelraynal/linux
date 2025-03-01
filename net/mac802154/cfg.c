// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Authors:
 * Alexander Aring <aar@pengutronix.de>
 *
 * Based on: net/mac80211/cfg.c
 */

#include <net/rtnetlink.h>
#include <net/cfg802154.h>

#include "ieee802154_i.h"
#include "driver-ops.h"
#include "cfg.h"

static struct net_device *
ieee802154_add_iface_deprecated(struct wpan_phy *wpan_phy,
				const char *name,
				unsigned char name_assign_type, int type)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	struct net_device *dev;

	rtnl_lock();
	dev = ieee802154_if_add(local, name, name_assign_type, type,
				cpu_to_le64(0x0000000000000000ULL));
	rtnl_unlock();

	return dev;
}

static void ieee802154_del_iface_deprecated(struct wpan_phy *wpan_phy,
					    struct net_device *dev)
{
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);

	ieee802154_if_remove(sdata);
}

#ifdef CONFIG_PM
static int ieee802154_suspend(struct wpan_phy *wpan_phy)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);

	if (!local->open_count)
		goto suspend;

	ieee802154_sync_and_hold_queue(local);
	synchronize_net();

	/* stop hardware - this must stop RX */
	ieee802154_stop_device(local);

suspend:
	local->suspended = true;
	return 0;
}

static int ieee802154_resume(struct wpan_phy *wpan_phy)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	int ret;

	/* nothing to do if HW shouldn't run */
	if (!local->open_count)
		goto wake_up;

	/* restart hardware */
	ret = drv_start(local, local->phy->filtering, &local->addr_filt);
	if (ret)
		return ret;

wake_up:
	ieee802154_release_queue(local);
	local->suspended = false;
	return 0;
}
#else
#define ieee802154_suspend NULL
#define ieee802154_resume NULL
#endif

static int
ieee802154_add_iface(struct wpan_phy *phy, const char *name,
		     unsigned char name_assign_type,
		     enum nl802154_iftype type, __le64 extended_addr)
{
	struct ieee802154_local *local = wpan_phy_priv(phy);
	struct net_device *err;

	err = ieee802154_if_add(local, name, name_assign_type, type,
				extended_addr);
	return PTR_ERR_OR_ZERO(err);
}

static int
ieee802154_del_iface(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev)
{
	ieee802154_if_remove(IEEE802154_WPAN_DEV_TO_SUB_IF(wpan_dev));

	return 0;
}

static int
ieee802154_set_channel(struct wpan_phy *wpan_phy, u8 page, u8 channel)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	int ret;

	ASSERT_RTNL();

	if (wpan_phy->current_page == page &&
	    wpan_phy->current_channel == channel)
		return 0;

	/* Refuse to change channels during scanning or beaconing */
	if (mac802154_is_scanning(local) || mac802154_is_beaconing(local))
		return -EBUSY;

	ret = drv_set_channel(local, page, channel);
	if (!ret) {
		wpan_phy->current_page = page;
		wpan_phy->current_channel = channel;
		ieee802154_configure_durations(wpan_phy, page, channel);
	}

	return ret;
}

static int
ieee802154_set_cca_mode(struct wpan_phy *wpan_phy,
			const struct wpan_phy_cca *cca)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	int ret;

	ASSERT_RTNL();

	if (wpan_phy_cca_cmp(&wpan_phy->cca, cca))
		return 0;

	ret = drv_set_cca_mode(local, cca);
	if (!ret)
		wpan_phy->cca = *cca;

	return ret;
}

static int
ieee802154_set_cca_ed_level(struct wpan_phy *wpan_phy, s32 ed_level)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	int ret;

	ASSERT_RTNL();

	if (wpan_phy->cca_ed_level == ed_level)
		return 0;

	ret = drv_set_cca_ed_level(local, ed_level);
	if (!ret)
		wpan_phy->cca_ed_level = ed_level;

	return ret;
}

static int
ieee802154_set_tx_power(struct wpan_phy *wpan_phy, s32 power)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	int ret;

	ASSERT_RTNL();

	if (wpan_phy->transmit_power == power)
		return 0;

	ret = drv_set_tx_power(local, power);
	if (!ret)
		wpan_phy->transmit_power = power;

	return ret;
}

static int
ieee802154_set_pan_id(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
		      __le16 pan_id)
{
	int ret;

	ASSERT_RTNL();

	if (wpan_dev->pan_id == pan_id)
		return 0;

	ret = mac802154_wpan_update_llsec(wpan_dev->netdev);
	if (!ret)
		wpan_dev->pan_id = pan_id;

	return ret;
}

static int
ieee802154_set_backoff_exponent(struct wpan_phy *wpan_phy,
				struct wpan_dev *wpan_dev,
				u8 min_be, u8 max_be)
{
	ASSERT_RTNL();

	wpan_dev->min_be = min_be;
	wpan_dev->max_be = max_be;
	return 0;
}

static int
ieee802154_set_short_addr(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			  __le16 short_addr)
{
	ASSERT_RTNL();

	wpan_dev->short_addr = short_addr;
	return 0;
}

static int
ieee802154_set_max_csma_backoffs(struct wpan_phy *wpan_phy,
				 struct wpan_dev *wpan_dev,
				 u8 max_csma_backoffs)
{
	ASSERT_RTNL();

	wpan_dev->csma_retries = max_csma_backoffs;
	return 0;
}

static int
ieee802154_set_max_frame_retries(struct wpan_phy *wpan_phy,
				 struct wpan_dev *wpan_dev,
				 s8 max_frame_retries)
{
	ASSERT_RTNL();

	wpan_dev->frame_retries = max_frame_retries;
	return 0;
}

static int
ieee802154_set_lbt_mode(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			bool mode)
{
	ASSERT_RTNL();

	wpan_dev->lbt = mode;
	return 0;
}

static int
ieee802154_set_ackreq_default(struct wpan_phy *wpan_phy,
			      struct wpan_dev *wpan_dev, bool ackreq)
{
	ASSERT_RTNL();

	wpan_dev->ackreq = ackreq;
	return 0;
}

static int mac802154_trigger_scan(struct wpan_phy *wpan_phy,
				  struct cfg802154_scan_request *request)
{
	struct ieee802154_sub_if_data *sdata;

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(request->wpan_dev);

	ASSERT_RTNL();

	return mac802154_trigger_scan_locked(sdata, request);
}

static int mac802154_abort_scan(struct wpan_phy *wpan_phy,
				struct wpan_dev *wpan_dev)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	struct ieee802154_sub_if_data *sdata;

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(wpan_dev);

	ASSERT_RTNL();

	return mac802154_abort_scan_locked(local, sdata);
}

static int mac802154_send_beacons(struct wpan_phy *wpan_phy,
				  struct cfg802154_beacon_request *request)
{
	struct ieee802154_sub_if_data *sdata;

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(request->wpan_dev);

	ASSERT_RTNL();

	return mac802154_send_beacons_locked(sdata, request);
}

static int mac802154_stop_beacons(struct wpan_phy *wpan_phy,
				  struct wpan_dev *wpan_dev)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	struct ieee802154_sub_if_data *sdata;

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(wpan_dev);

	ASSERT_RTNL();

	return mac802154_stop_beacons_locked(local, sdata);
}

static int mac802154_associate(struct wpan_phy *wpan_phy,
			       struct wpan_dev *wpan_dev,
			       struct ieee802154_addr *coord)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	u64 ceaddr = swab64((__force u64)coord->extended_addr);
	struct ieee802154_sub_if_data *sdata;
	struct ieee802154_pan_device *parent;
	__le16 short_addr;
	int ret;

	ASSERT_RTNL();

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(wpan_dev);

	if (wpan_dev->parent) {
		dev_err(&sdata->dev->dev,
			"Device %8phC is already associated\n", &ceaddr);
		return -EPERM;
	}

	if (coord->mode == IEEE802154_SHORT_ADDRESSING)
		return -EINVAL;

	parent = kzalloc(sizeof(*parent), GFP_KERNEL);
	if (!parent)
		return -ENOMEM;

	parent->pan_id = coord->pan_id;
	parent->mode = coord->mode;
	parent->extended_addr = coord->extended_addr;
	parent->short_addr = cpu_to_le16(IEEE802154_ADDR_SHORT_BROADCAST);

	/* Set the PAN ID hardware address filter beforehand to avoid dropping
	 * the association response with a destination PAN ID field set to the
	 * "new" PAN ID.
	 */
	if (local->hw.flags & IEEE802154_HW_AFILT) {
		ret = drv_set_pan_id(local, coord->pan_id);
		if (ret < 0)
			goto free_parent;
	}

	ret = mac802154_perform_association(sdata, parent, &short_addr);
	if (ret)
		goto reset_panid;

	if (local->hw.flags & IEEE802154_HW_AFILT) {
		ret = drv_set_short_addr(local, short_addr);
		if (ret < 0)
			goto reset_panid;
	}

	wpan_dev->pan_id = coord->pan_id;
	wpan_dev->short_addr = short_addr;
	wpan_dev->parent = parent;
	wpan_dev->association_generation++;

	return 0;

reset_panid:
	if (local->hw.flags & IEEE802154_HW_AFILT)
		drv_set_pan_id(local, IEEE802154_PAN_ID_BROADCAST);

free_parent:
	kfree(parent);
	return ret;
}

static int mac802154_disassociate_from_parent(struct wpan_phy *wpan_phy,
					      struct wpan_dev *wpan_dev)
{
	struct ieee802154_local *local = wpan_phy_priv(wpan_phy);
	struct ieee802154_pan_device *child, *tmp;
	struct ieee802154_sub_if_data *sdata;
	unsigned int max_assoc;
	u64 eaddr;
	int ret;

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(wpan_dev);

	/* Start by disassociating all the children and preventing new ones to
	 * attempt associations.
	 */
	max_assoc = cfg802154_set_max_associations(wpan_dev, 0);
	list_for_each_entry_safe(child, tmp, &wpan_dev->children, node) {
		ret = mac802154_send_disassociation_notif(sdata, child,
							  IEEE802154_COORD_WISHES_DEVICE_TO_LEAVE);
		if (ret) {
			eaddr = swab64((__force u64)child->extended_addr);
			dev_err(&sdata->dev->dev,
				"Disassociation with %8phC may have failed (%d)\n",
				&eaddr, ret);
		}

		list_del(&child->node);
	}

	ret = mac802154_send_disassociation_notif(sdata, wpan_dev->parent,
						  IEEE802154_DEVICE_WISHES_TO_LEAVE);
	if (ret) {
		eaddr = swab64((__force u64)wpan_dev->parent->extended_addr);
		dev_err(&sdata->dev->dev,
			"Disassociation from %8phC may have failed (%d)\n",
			&eaddr, ret);
	}

	ret = 0;

	kfree(wpan_dev->parent);
	wpan_dev->parent = NULL;
	wpan_dev->association_generation++;
	wpan_dev->pan_id = cpu_to_le16(IEEE802154_PAN_ID_BROADCAST);
	wpan_dev->short_addr = cpu_to_le16(IEEE802154_ADDR_SHORT_BROADCAST);

	if (local->hw.flags & IEEE802154_HW_AFILT) {
		ret = drv_set_pan_id(local, wpan_dev->pan_id);
		if (ret < 0)
			goto reset_mac_assoc;

		ret = drv_set_short_addr(local, wpan_dev->short_addr);
		if (ret < 0)
			goto reset_mac_assoc;
	}

reset_mac_assoc:
	cfg802154_set_max_associations(wpan_dev, max_assoc);

	return ret;
}

static int mac802154_disassociate_child(struct wpan_phy *wpan_phy,
					struct wpan_dev *wpan_dev,
					struct ieee802154_pan_device *child)
{
	struct ieee802154_sub_if_data *sdata;
	int ret;

	sdata = IEEE802154_WPAN_DEV_TO_SUB_IF(wpan_dev);

	ret = mac802154_send_disassociation_notif(sdata, child,
						  IEEE802154_COORD_WISHES_DEVICE_TO_LEAVE);
	if (ret)
		return ret;

	list_del(&child->node);
	wpan_dev->nchildren--;
	wpan_dev->association_generation++;
	kfree(child);

	return 0;
}

static int mac802154_disassociate(struct wpan_phy *wpan_phy,
				  struct wpan_dev *wpan_dev,
				  struct ieee802154_addr *target)
{
	u64 teaddr = swab64((__force u64)target->extended_addr);
	struct ieee802154_pan_device *pan_device;

	ASSERT_RTNL();

	if (cfg802154_device_is_parent(wpan_dev, target))
		return mac802154_disassociate_from_parent(wpan_phy, wpan_dev);

	pan_device = cfg802154_device_is_child(wpan_dev, target);
	if (pan_device)
		return mac802154_disassociate_child(wpan_phy, wpan_dev,
						    pan_device);

	dev_err(&wpan_dev->netdev->dev,
		"Device %8phC is not associated with us\n", &teaddr);

	return -EINVAL;
}

#ifdef CONFIG_IEEE802154_NL802154_EXPERIMENTAL
static void
ieee802154_get_llsec_table(struct wpan_phy *wpan_phy,
			   struct wpan_dev *wpan_dev,
			   struct ieee802154_llsec_table **table)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);

	*table = &sdata->sec.table;
}

static void
ieee802154_lock_llsec_table(struct wpan_phy *wpan_phy,
			    struct wpan_dev *wpan_dev)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);

	mutex_lock(&sdata->sec_mtx);
}

static void
ieee802154_unlock_llsec_table(struct wpan_phy *wpan_phy,
			      struct wpan_dev *wpan_dev)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);

	mutex_unlock(&sdata->sec_mtx);
}

static int
ieee802154_set_llsec_params(struct wpan_phy *wpan_phy,
			    struct wpan_dev *wpan_dev,
			    const struct ieee802154_llsec_params *params,
			    int changed)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_set_params(&sdata->sec, params, changed);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_get_llsec_params(struct wpan_phy *wpan_phy,
			    struct wpan_dev *wpan_dev,
			    struct ieee802154_llsec_params *params)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_get_params(&sdata->sec, params);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_add_llsec_key(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			 const struct ieee802154_llsec_key_id *id,
			 const struct ieee802154_llsec_key *key)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_key_add(&sdata->sec, id, key);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_del_llsec_key(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			 const struct ieee802154_llsec_key_id *id)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_key_del(&sdata->sec, id);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_add_seclevel(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			const struct ieee802154_llsec_seclevel *sl)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_seclevel_add(&sdata->sec, sl);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_del_seclevel(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			const struct ieee802154_llsec_seclevel *sl)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_seclevel_del(&sdata->sec, sl);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_add_device(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
		      const struct ieee802154_llsec_device *dev_desc)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_dev_add(&sdata->sec, dev_desc);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_del_device(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
		      __le64 extended_addr)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_dev_del(&sdata->sec, extended_addr);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_add_devkey(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
		      __le64 extended_addr,
		      const struct ieee802154_llsec_device_key *key)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_devkey_add(&sdata->sec, extended_addr, key);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}

static int
ieee802154_del_devkey(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
		      __le64 extended_addr,
		      const struct ieee802154_llsec_device_key *key)
{
	struct net_device *dev = wpan_dev->netdev;
	struct ieee802154_sub_if_data *sdata = IEEE802154_DEV_TO_SUB_IF(dev);
	int res;

	mutex_lock(&sdata->sec_mtx);
	res = mac802154_llsec_devkey_del(&sdata->sec, extended_addr, key);
	mutex_unlock(&sdata->sec_mtx);

	return res;
}
#endif /* CONFIG_IEEE802154_NL802154_EXPERIMENTAL */

const struct cfg802154_ops mac802154_config_ops = {
	.add_virtual_intf_deprecated = ieee802154_add_iface_deprecated,
	.del_virtual_intf_deprecated = ieee802154_del_iface_deprecated,
	.suspend = ieee802154_suspend,
	.resume = ieee802154_resume,
	.add_virtual_intf = ieee802154_add_iface,
	.del_virtual_intf = ieee802154_del_iface,
	.set_channel = ieee802154_set_channel,
	.set_cca_mode = ieee802154_set_cca_mode,
	.set_cca_ed_level = ieee802154_set_cca_ed_level,
	.set_tx_power = ieee802154_set_tx_power,
	.set_pan_id = ieee802154_set_pan_id,
	.set_short_addr = ieee802154_set_short_addr,
	.set_backoff_exponent = ieee802154_set_backoff_exponent,
	.set_max_csma_backoffs = ieee802154_set_max_csma_backoffs,
	.set_max_frame_retries = ieee802154_set_max_frame_retries,
	.set_lbt_mode = ieee802154_set_lbt_mode,
	.set_ackreq_default = ieee802154_set_ackreq_default,
	.trigger_scan = mac802154_trigger_scan,
	.abort_scan = mac802154_abort_scan,
	.send_beacons = mac802154_send_beacons,
	.stop_beacons = mac802154_stop_beacons,
	.associate = mac802154_associate,
	.disassociate = mac802154_disassociate,
#ifdef CONFIG_IEEE802154_NL802154_EXPERIMENTAL
	.get_llsec_table = ieee802154_get_llsec_table,
	.lock_llsec_table = ieee802154_lock_llsec_table,
	.unlock_llsec_table = ieee802154_unlock_llsec_table,
	/* TODO above */
	.set_llsec_params = ieee802154_set_llsec_params,
	.get_llsec_params = ieee802154_get_llsec_params,
	.add_llsec_key = ieee802154_add_llsec_key,
	.del_llsec_key = ieee802154_del_llsec_key,
	.add_seclevel = ieee802154_add_seclevel,
	.del_seclevel = ieee802154_del_seclevel,
	.add_device = ieee802154_add_device,
	.del_device = ieee802154_del_device,
	.add_devkey = ieee802154_add_devkey,
	.del_devkey = ieee802154_del_devkey,
#endif /* CONFIG_IEEE802154_NL802154_EXPERIMENTAL */
};
