// SPDX-License-Identifier: GPL-2.0
/*
 * IEEE 802.15.4 PAN management
 *
 * Copyright (C) 2021 Qorvo US, Inc
 * Authors:
 *   - David Girault <david.girault@qorvo.com>
 *   - Miquel Raynal <miquel.raynal@bootlin.com>
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

#include <net/cfg802154.h>
#include <net/af_ieee802154.h>

#include "ieee802154.h"
#include "../ieee802154/nl802154.h"
#include "trace.h"

struct ieee802154_coord_desc *
cfg802154_alloc_coordinator(struct ieee802154_addr *coord)
{
	struct ieee802154_coord_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_ATOMIC);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	desc->addr = kzalloc(sizeof(*coord), GFP_ATOMIC);
	if (!desc->addr) {
		kfree(desc);
		return ERR_PTR(-ENOMEM);
	}

	memcpy(desc->addr, coord, sizeof(*coord));

	return desc;
}
EXPORT_SYMBOL_GPL(cfg802154_alloc_coordinator);

static void cfg802154_free_coordinator_desc(struct ieee802154_coord_desc *desc)
{
	kfree(desc->addr);
	kfree(desc);
}

static bool
cfg802154_is_same_coordinator(struct ieee802154_coord_desc *a,
			      struct ieee802154_coord_desc *b)
{
	if (a->addr->pan_id != b->addr->pan_id)
		return false;

	if (a->addr->mode != b->addr->mode)
		return false;

	if (a->addr->mode == IEEE802154_ADDR_SHORT &&
	    a->addr->short_addr == b->addr->short_addr)
		return true;
	else if (a->addr->mode == IEEE802154_ADDR_LONG &&
		 a->addr->extended_addr == b->addr->extended_addr)
		return true;

	return false;
}

static bool
cfg802154_coordinator_is_known(struct wpan_dev *wpan_dev,
			       struct ieee802154_coord_desc *desc)
{
	struct ieee802154_coord_desc *item;

	list_for_each_entry(item, &wpan_dev->coord_list, node)
		if (cfg802154_is_same_coordinator(item, desc))
			return true;

	return false;
}

void cfg802154_record_coordinator(struct wpan_phy *wpan_phy,
				  struct wpan_dev *wpan_dev,
				  struct ieee802154_coord_desc *desc)
{
	spin_lock_bh(&wpan_dev->coord_list_lock);

	if (cfg802154_coordinator_is_known(wpan_dev, desc)) {
		nl802154_advertise_known_coordinator(wpan_phy, wpan_dev, desc);
		cfg802154_free_coordinator_desc(desc);
	} else {
		list_add_tail(&desc->node, &wpan_dev->coord_list);
		trace_802154_new_coordinator(desc);
		nl802154_advertise_new_coordinator(wpan_phy, wpan_dev, desc);
	}

	spin_unlock_bh(&wpan_dev->coord_list_lock);
}
EXPORT_SYMBOL_GPL(cfg802154_record_coordinator);

void cfg802154_flush_known_coordinators(struct wpan_dev *wpan_dev)
{
	struct ieee802154_coord_desc *desc, *tmp;

	spin_lock_bh(&wpan_dev->coord_list_lock);

	list_for_each_entry_safe(desc, tmp, &wpan_dev->coord_list, node) {
		list_del(&desc->node);
		cfg802154_free_coordinator_desc(desc);
	}

	spin_unlock_bh(&wpan_dev->coord_list_lock);
}
EXPORT_SYMBOL_GPL(cfg802154_flush_known_coordinators);

static bool cfg802154_same_addr(struct ieee802154_pan_device *a,
				struct ieee802154_addr *b)
{
	if (!a || !b)
		return false;

	switch (b->mode) {
	case IEEE802154_ADDR_SHORT:
		return a->short_addr == b->short_addr;
	case IEEE802154_ADDR_LONG:
		return a->extended_addr == b->extended_addr;
	default:
		return false;
	}
}

bool cfg802154_device_is_associated(struct wpan_dev *wpan_dev)
{
	bool is_assoc;

	mutex_lock(&wpan_dev->association_lock);
	is_assoc = !list_empty(&wpan_dev->children) || wpan_dev->parent;
	mutex_unlock(&wpan_dev->association_lock);

	return is_assoc;
}

bool cfg802154_device_is_parent(struct wpan_dev *wpan_dev,
				struct ieee802154_addr *target)
{
	lockdep_assert_held(&wpan_dev->association_lock);

	if (cfg802154_same_addr(wpan_dev->parent, target))
		return true;

	return false;
}

struct ieee802154_pan_device *
cfg802154_device_is_child(struct wpan_dev *wpan_dev,
			  struct ieee802154_addr *target)
{
	struct ieee802154_pan_device *child;

	lockdep_assert_held(&wpan_dev->association_lock);

	list_for_each_entry(child, &wpan_dev->children, node)
		if (cfg802154_same_addr(child, target))
			return child;

	return NULL;
}
