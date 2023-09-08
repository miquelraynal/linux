// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2007, 2008, 2009 Siemens AG
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

#include <net/cfg802154.h>
#include <net/rtnetlink.h>

#include "ieee802154.h"
#include "nl802154.h"
#include "sysfs.h"
#include "core.h"

/* name for sysfs, %d is appended */
#define PHY_NAME "phy"

/* RCU-protected (and RTNL for writers) */
LIST_HEAD(cfg802154_rdev_list);
int cfg802154_rdev_list_generation;

struct wpan_phy *wpan_phy_find(const char *str)
{
	struct device *dev;

	if (WARN_ON(!str))
		return NULL;

	dev = class_find_device_by_name(&wpan_phy_class, str);
	if (!dev)
		return NULL;

	return container_of(dev, struct wpan_phy, dev);
}
EXPORT_SYMBOL(wpan_phy_find);

struct wpan_phy_iter_data {
	int (*fn)(struct wpan_phy *phy, void *data);
	void *data;
};

static int wpan_phy_iter(struct device *dev, void *_data)
{
	struct wpan_phy_iter_data *wpid = _data;
	struct wpan_phy *phy = container_of(dev, struct wpan_phy, dev);

	return wpid->fn(phy, wpid->data);
}

int wpan_phy_for_each(int (*fn)(struct wpan_phy *phy, void *data),
		      void *data)
{
	struct wpan_phy_iter_data wpid = {
		.fn = fn,
		.data = data,
	};

	return class_for_each_device(&wpan_phy_class, NULL,
			&wpid, wpan_phy_iter);
}
EXPORT_SYMBOL(wpan_phy_for_each);

struct cfg802154_registered_device *
cfg802154_rdev_by_wpan_phy_idx(int wpan_phy_idx)
{
	struct cfg802154_registered_device *result = NULL, *rdev;

	ASSERT_RTNL();

	list_for_each_entry(rdev, &cfg802154_rdev_list, list) {
		if (rdev->wpan_phy_idx == wpan_phy_idx) {
			result = rdev;
			break;
		}
	}

	return result;
}

struct wpan_phy *wpan_phy_idx_to_wpan_phy(int wpan_phy_idx)
{
	struct cfg802154_registered_device *rdev;

	ASSERT_RTNL();

	rdev = cfg802154_rdev_by_wpan_phy_idx(wpan_phy_idx);
	if (!rdev)
		return NULL;
	return &rdev->wpan_phy;
}

struct wpan_phy *
wpan_phy_new(const struct cfg802154_ops *ops, size_t priv_size)
{
	static atomic_t wpan_phy_counter = ATOMIC_INIT(0);
	struct cfg802154_registered_device *rdev;
	size_t alloc_size;

	alloc_size = sizeof(*rdev) + priv_size;
	rdev = kzalloc(alloc_size, GFP_KERNEL);
	if (!rdev)
		return NULL;

	rdev->ops = ops;

	rdev->wpan_phy_idx = atomic_inc_return(&wpan_phy_counter);

	if (unlikely(rdev->wpan_phy_idx < 0)) {
		/* ugh, wrapped! */
		atomic_dec(&wpan_phy_counter);
		kfree(rdev);
		return NULL;
	}

	/* atomic_inc_return makes it start at 1, make it start at 0 */
	rdev->wpan_phy_idx--;

	INIT_LIST_HEAD(&rdev->wpan_dev_list);
	device_initialize(&rdev->wpan_phy.dev);
	dev_set_name(&rdev->wpan_phy.dev, PHY_NAME "%d", rdev->wpan_phy_idx);

	rdev->wpan_phy.dev.class = &wpan_phy_class;
	rdev->wpan_phy.dev.platform_data = rdev;

	wpan_phy_net_set(&rdev->wpan_phy, &init_net);

	init_waitqueue_head(&rdev->dev_wait);
	init_waitqueue_head(&rdev->wpan_phy.sync_txq);

	spin_lock_init(&rdev->wpan_phy.queue_lock);

	return &rdev->wpan_phy;
}
EXPORT_SYMBOL(wpan_phy_new);

static u64 ieee802154_uwb_default_codes(struct wpan_phy *phy, int channel)
{
	u64 supported = 0;

	if (phy->supported.prfs & NL802154_MEAN_PRF_4030KHZ ||
	    phy->supported.prfs	& NL802154_MEAN_PRF_16100KHZ) {
		if (channel == 0 || channel == 1 || channel == 8 || channel == 12)
			supported |= GENMASK(2, 1);

		if (channel == 2 || channel == 5 || channel == 9 || channel == 13)
			supported |= GENMASK(4, 3);

		if (channel == 3 || channel == 6 || channel == 10 || channel == 14)
			supported |= GENMASK(6, 5);

		if (channel == 4 || channel == 7 || channel == 11 || channel == 15)
			supported |= GENMASK(8, 1);
	}

	if (phy->supported.prfs & NL802154_MEAN_PRF_62890KHZ) {
		if ((channel >= 0 && channel <= 3) ||
		    (channel == 5 || channel == 6) ||
		    (channel >= 8 && channel <= 10) ||
		    (channel >= 12 && channel <= 14))
			supported |= GENMASK(12, 9);

		if (channel >= 0 && channel <= 15 && phy->supported.dps)
			supported |= GENMASK(16, 13) | GENMASK(24, 21);

		if (channel == 4 || channel == 7 || channel == 11 || channel == 15)
			supported |= GENMASK(13, 9) | GENMASK(20, 17);
	}

	if (phy->supported.prfs & NL802154_MEAN_PRF_111090KHZ)
		supported |= GENMASK(32, 25);

	return supported;
}

static int wpan_phy_sanity_checks(struct wpan_phy *phy)
{
	unsigned long uwb_chans = phy->supported.channels[4];
	u64 uwb_pcodes;
	unsigned int chan, pcode;

	if (!uwb_chans)
		return 0;

	for_each_set_bit(chan, &uwb_chans, IEEE802154_MAX_UWB_CHANNEL) {
		if (!ieee802154_uwb_supported_codes(phy, chan)) {
			dev_err(&phy->dev,
				"Missing UWB channel %d preamble code list\n", chan);
			return -EINVAL;
		}

		if ((ieee802154_uwb_supported_codes(phy, chan) &
		     ieee802154_uwb_default_codes(phy, chan)) !=
		    ieee802154_uwb_supported_codes(phy, chan)) {
			dev_err(&phy->dev,
				"Wrong UWB channel %d preamble code list\n", chan);
			return -EINVAL;
		}

		uwb_pcodes = ieee802154_uwb_supported_codes(phy, chan);
		for_each_set_bit(pcode, (unsigned long *)&uwb_pcodes,
				 IEEE802154_MAX_PREAMBLE_CODE) {
			if (!ieee802154_uwb_supported_prfs(phy)) {
				dev_err(&phy->dev,
					"Missing UWB channel %d preamble code %d PRF list\n",
					chan, pcode);
				return -EINVAL;
			}

			if (!(ieee802154_uwb_supported_prfs(phy) &
			      ieee802154_uwb_default_prfs(pcode))) {
				dev_err(&phy->dev,
					"Wrong UWB channel %d preamble code %d PRF list\n",
					chan, pcode);
				return -EINVAL;
			}
		}
	}

	return 0;
}

int wpan_phy_register(struct wpan_phy *phy)
{
	struct cfg802154_registered_device *rdev = wpan_phy_to_rdev(phy);
	int ret;

	ret = wpan_phy_sanity_checks(phy);
	if (ret)
		return ret;

	rtnl_lock();
	ret = device_add(&phy->dev);
	if (ret) {
		rtnl_unlock();
		return ret;
	}

	list_add_rcu(&rdev->list, &cfg802154_rdev_list);
	cfg802154_rdev_list_generation++;

	/* TODO phy registered lock */
	rtnl_unlock();

	/* TODO nl802154 phy notify */

	return 0;
}
EXPORT_SYMBOL(wpan_phy_register);

void wpan_phy_unregister(struct wpan_phy *phy)
{
	struct cfg802154_registered_device *rdev = wpan_phy_to_rdev(phy);

	wait_event(rdev->dev_wait, ({
		int __count;
		rtnl_lock();
		__count = rdev->opencount;
		rtnl_unlock();
		__count == 0; }));

	rtnl_lock();
	/* TODO nl802154 phy notify */
	/* TODO phy registered lock */

	WARN_ON(!list_empty(&rdev->wpan_dev_list));

	/* First remove the hardware from everywhere, this makes
	 * it impossible to find from userspace.
	 */
	list_del_rcu(&rdev->list);
	synchronize_rcu();

	cfg802154_rdev_list_generation++;

	device_del(&phy->dev);

	rtnl_unlock();
}
EXPORT_SYMBOL(wpan_phy_unregister);

void wpan_phy_free(struct wpan_phy *phy)
{
	put_device(&phy->dev);
}
EXPORT_SYMBOL(wpan_phy_free);

static void cfg802154_free_peer_structures(struct wpan_dev *wpan_dev)
{
	struct ieee802154_pan_device *child, *tmp;

	mutex_lock(&wpan_dev->association_lock);

	kfree(wpan_dev->parent);
	wpan_dev->parent = NULL;

	list_for_each_entry_safe(child, tmp, &wpan_dev->children, node) {
		list_del(&child->node);
		kfree(child);
	}

	wpan_dev->nchildren = 0;
	wpan_dev->association_generation++;

	mutex_unlock(&wpan_dev->association_lock);
}

int cfg802154_switch_netns(struct cfg802154_registered_device *rdev,
			   struct net *net)
{
	struct wpan_dev *wpan_dev;
	int err = 0;

	list_for_each_entry(wpan_dev, &rdev->wpan_dev_list, list) {
		if (!wpan_dev->netdev)
			continue;
		wpan_dev->netdev->features &= ~NETIF_F_NETNS_LOCAL;
		err = dev_change_net_namespace(wpan_dev->netdev, net, "wpan%d");
		if (err)
			break;
		wpan_dev->netdev->features |= NETIF_F_NETNS_LOCAL;
	}

	if (err) {
		/* failed -- clean up to old netns */
		net = wpan_phy_net(&rdev->wpan_phy);

		list_for_each_entry_continue_reverse(wpan_dev,
						     &rdev->wpan_dev_list,
						     list) {
			if (!wpan_dev->netdev)
				continue;
			wpan_dev->netdev->features &= ~NETIF_F_NETNS_LOCAL;
			err = dev_change_net_namespace(wpan_dev->netdev, net,
						       "wpan%d");
			WARN_ON(err);
			wpan_dev->netdev->features |= NETIF_F_NETNS_LOCAL;
		}

		return err;
	}

	wpan_phy_net_set(&rdev->wpan_phy, net);

	err = device_rename(&rdev->wpan_phy.dev, dev_name(&rdev->wpan_phy.dev));
	WARN_ON(err);

	return 0;
}

void cfg802154_dev_free(struct cfg802154_registered_device *rdev)
{
	kfree(rdev);
}

static void
cfg802154_update_iface_num(struct cfg802154_registered_device *rdev,
			   int iftype, int num)
{
	ASSERT_RTNL();

	rdev->num_running_ifaces += num;
}

static int cfg802154_netdev_notifier_call(struct notifier_block *nb,
					  unsigned long state, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct wpan_dev *wpan_dev = dev->ieee802154_ptr;
	struct cfg802154_registered_device *rdev;

	if (!wpan_dev)
		return NOTIFY_DONE;

	rdev = wpan_phy_to_rdev(wpan_dev->wpan_phy);

	/* TODO WARN_ON unspec type */

	switch (state) {
		/* TODO NETDEV_DEVTYPE */
	case NETDEV_REGISTER:
		dev->features |= NETIF_F_NETNS_LOCAL;
		wpan_dev->identifier = ++rdev->wpan_dev_id;
		list_add_rcu(&wpan_dev->list, &rdev->wpan_dev_list);
		rdev->devlist_generation++;
		mutex_init(&wpan_dev->association_lock);
		INIT_LIST_HEAD(&wpan_dev->children);
		wpan_dev->max_associations = SZ_16K;

		wpan_dev->netdev = dev;
		break;
	case NETDEV_DOWN:
		cfg802154_update_iface_num(rdev, wpan_dev->iftype, -1);

		rdev->opencount--;
		wake_up(&rdev->dev_wait);
		break;
	case NETDEV_UP:
		cfg802154_update_iface_num(rdev, wpan_dev->iftype, 1);

		rdev->opencount++;
		break;
	case NETDEV_UNREGISTER:
		cfg802154_free_peer_structures(wpan_dev);

		/* It is possible to get NETDEV_UNREGISTER
		 * multiple times. To detect that, check
		 * that the interface is still on the list
		 * of registered interfaces, and only then
		 * remove and clean it up.
		 */
		if (!list_empty(&wpan_dev->list)) {
			list_del_rcu(&wpan_dev->list);
			rdev->devlist_generation++;
		}
		/* synchronize (so that we won't find this netdev
		 * from other code any more) and then clear the list
		 * head so that the above code can safely check for
		 * !list_empty() to avoid double-cleanup.
		 */
		synchronize_rcu();
		INIT_LIST_HEAD(&wpan_dev->list);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block cfg802154_netdev_notifier = {
	.notifier_call = cfg802154_netdev_notifier_call,
};

static void __net_exit cfg802154_pernet_exit(struct net *net)
{
	struct cfg802154_registered_device *rdev;

	rtnl_lock();
	list_for_each_entry(rdev, &cfg802154_rdev_list, list) {
		if (net_eq(wpan_phy_net(&rdev->wpan_phy), net))
			WARN_ON(cfg802154_switch_netns(rdev, &init_net));
	}
	rtnl_unlock();
}

static struct pernet_operations cfg802154_pernet_ops = {
	.exit = cfg802154_pernet_exit,
};

static int __init wpan_phy_class_init(void)
{
	int rc;

	rc = register_pernet_device(&cfg802154_pernet_ops);
	if (rc)
		goto err;

	rc = wpan_phy_sysfs_init();
	if (rc)
		goto err_sysfs;

	rc = register_netdevice_notifier(&cfg802154_netdev_notifier);
	if (rc)
		goto err_nl;

	rc = ieee802154_nl_init();
	if (rc)
		goto err_notifier;

	rc = nl802154_init();
	if (rc)
		goto err_ieee802154_nl;

	return 0;

err_ieee802154_nl:
	ieee802154_nl_exit();

err_notifier:
	unregister_netdevice_notifier(&cfg802154_netdev_notifier);
err_nl:
	wpan_phy_sysfs_exit();
err_sysfs:
	unregister_pernet_device(&cfg802154_pernet_ops);
err:
	return rc;
}
subsys_initcall(wpan_phy_class_init);

static void __exit wpan_phy_class_exit(void)
{
	nl802154_exit();
	ieee802154_nl_exit();
	unregister_netdevice_notifier(&cfg802154_netdev_notifier);
	wpan_phy_sysfs_exit();
	unregister_pernet_device(&cfg802154_pernet_ops);
}
module_exit(wpan_phy_class_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IEEE 802.15.4 configuration interface");
MODULE_AUTHOR("Dmitry Eremin-Solenikov");
