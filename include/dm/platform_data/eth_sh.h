/*
 * Copyright (C) 2017 Nobuhiro Iwamatsu <iwamatsu@nigauir.org>
 *
 * SPDX-License-Identifier:     GPL-2.0
 */

#ifndef _SH_ETH_H
#define _SH_ETH_H

#include <net.h>

#ifdef CONFIG_DM_ETH

struct sh_eth_pdata {
	struct eth_pdata eth_pdata;
	u32 device_type;
	u8 phy_addr;
	u8 phy_interface;
};

#endif

#endif /* _SH_ETH_H */

