/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __IEEE802154_NL802154_H
#define __IEEE802154_NL802154_H

int nl802154_init(void);
void nl802154_exit(void);
int nl802154_advertise_new_coordinator(struct wpan_phy *wpan_phy,
				       struct wpan_dev *wpan_dev,
				       struct ieee802154_coord_desc *desc);
int nl802154_advertise_known_coordinator(struct wpan_phy *wpan_phy,
					 struct wpan_dev *wpan_dev,
					 struct ieee802154_coord_desc *desc);
int nl802154_send_start_scan(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev);
int nl802154_send_scan_done(struct wpan_phy *wpan_phy, struct wpan_dev *wpan_dev,
			    struct cfg802154_scan_request *request, u8 cmd);

#endif /* __IEEE802154_NL802154_H */
