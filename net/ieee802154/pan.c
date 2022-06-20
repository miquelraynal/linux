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
