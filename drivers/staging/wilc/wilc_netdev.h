/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#ifndef WILC_NETDEV_H
#define WILC_NETDEV_H
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "wilc_wfi_netdevice.h"
#include "wilc_wlan_if.h"

extern int wait_for_recovery;

struct net_device *wilc_get_if_netdev(struct wilc *wilc, uint8_t ifc);
struct host_if_drv *get_drv_hndl_by_ifc(struct wilc *wilc, uint8_t ifc);

#if KERNEL_VERSION(3, 14, 0) > LINUX_VERSION_CODE
static inline void ether_addr_copy(u8 *dst, const u8 *src)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	*(u32 *)dst = *(const u32 *)src;
	*(u16 *)(dst + 4) = *(const u16 *)(src + 4);
#else
	u16 *a = (u16 *)dst;
	const u16 *b = (const u16 *)src;

	a[0] = b[0];
	a[1] = b[1];
	a[2] = b[2];
#endif
}

static inline bool ether_addr_equal_unaligned(const u8 *addr1, const u8 *addr2)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	return ether_addr_equal(addr1, addr2);
#else
	return memcmp(addr1, addr2, ETH_ALEN) == 0;
#endif
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0) */

int wilc_bt_power_up(struct wilc *wilc, int source);
int wilc_bt_power_down(struct wilc *wilc, int source);
void wilc_wfi_monitor_rx(struct net_device *mon_dev, u8 *buff, u32 size);
struct wilc_vif *
wilc_netdev_ifc_init(struct wilc *wl, const char *name, int iftype,
		     enum nl80211_iftype type, bool rtnl_locked);

#endif /* WILC_NETDEV_H */
