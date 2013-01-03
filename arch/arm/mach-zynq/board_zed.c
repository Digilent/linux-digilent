/*
 *  Copyright (C) 2011 Xilinx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * New portions Copyright (c) 2012 Digilent Inc.
 * ZED board definition file based on the board_zc702.c file.
 * Additional functionality provided by the ZED board definition:
 *  - Recognition of ZED board in the device tree to support a single Zynq
 *    kernel that runs on several Zynq based boards.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/of_platform.h>

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/xilinx_devices.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <asm/hardware/gic.h>
#include "common.h"

//#define ZED_REV_B

extern struct sys_timer xttcpss_sys_timer;

static void __init board_zed_init(void)
{

	/* initialize the xilinx common code before the board
	 * specific
	 */
	xilinx_init_machine();

	printk("\n###############################################\n");
	printk  ("#                                             #\n");
	printk  ("#                Board ZED Init               #\n");
	printk  ("#                                             #\n");
	printk  ("###############################################\n\n");
}

static const char *xilinx_dt_match[] = {
	"xlnx,zynq-zed",
	NULL
};

MACHINE_START(XILINX_EP107, "Xilinx Zynq Platform")
	.map_io		= xilinx_map_io,
	.init_irq	= xilinx_irq_init,
#ifdef	GIC_CPU_CTRL
	.handle_irq	= gic_handle_irq,
#endif
	.init_machine	= board_zed_init,
	.timer		= &xttcpss_sys_timer,
	.dt_compat	= xilinx_dt_match,
	.reserve	= xilinx_memory_init,
	.restart	= xilinx_system_reset,
MACHINE_END
