# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(src)/ -DWILC_ASIC_A0 -DWILC_DEBUGFS

wilc-objs := wilc_wfi_cfgoperations.o wilc_netdev.o wilc_mon.o \
			wilc_hif.o wilc_wlan_cfg.o wilc_debugfs.o \
			wilc_wlan.o sysfs.o wilc_bt.o

obj-$(CONFIG_WILC_SDIO) += wilc-sdio.o
wilc-sdio-objs += $(wilc-objs)
wilc-sdio-objs += wilc_sdio.o

obj-$(CONFIG_WILC_SPI) += wilc-spi.o
wilc-spi-objs += $(wilc-objs)
wilc-spi-objs += wilc_spi.o 
