// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/debugfs.h>

#include "wilc_debugfs.h"

atomic_t WILC_DEBUG_REGION = ATOMIC_INIT(INIT_DBG | GENERIC_DBG |
					 CFG80211_DBG | HOSTAPD_DBG |
					 PWRDEV_DBG);

#if defined(WILC_DEBUGFS)
static struct dentry *wilc_dir;

static ssize_t wilc_debug_region_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	char buf[128];
	int res = 0;

	/* only allow read from start */
	if (*ppos > 0)
		return 0;

	res = scnprintf(buf, sizeof(buf), "Debug Region: (0x%08x)\n",
				    atomic_read(&WILC_DEBUG_REGION));

	return simple_read_from_buffer(userbuf, count, ppos, buf, res);
}

static ssize_t wilc_debug_region_write(struct file *filp,
				      const char __user *buf, size_t count,
				      loff_t *ppos)
{
	int flag = 0;
	int ret;

	ret = kstrtouint_from_user(buf, count, 16, &flag);
	if (ret)
		return ret;

	if (flag > DBG_REGION_ALL) {
		pr_err("%s, value (0x%08x) is out of range, stay previous flag (0x%08x)\n",
			   __func__, flag, atomic_read(&WILC_DEBUG_REGION));
		pr_err("allowed bits are 0 to 15\n");
		return -EINVAL;
	}

	atomic_set(&WILC_DEBUG_REGION, (int)flag);

	pr_info("Debug region set to %x\n", atomic_read(&WILC_DEBUG_REGION));

	return count;
}

#define FOPS(_open, _read, _write, _poll) { \
		.owner	= THIS_MODULE, \
		.open	= (_open), \
		.read	= (_read), \
		.write	= (_write), \
		.poll		= (_poll), \
}

struct wilc_debugfs_info_t {
	const char *name;
	int perm;
	unsigned int data;
	const struct file_operations fops;
};

static struct wilc_debugfs_info_t debugfs_info[] = {
	{
		"wilc_debug_region",
		0666,
		0,
		FOPS(NULL, wilc_debug_region_read, wilc_debug_region_write,
		     NULL),
	},
};

int wilc_debugfs_init(void)
{
	int i;
	struct wilc_debugfs_info_t *info;

	wilc_dir = debugfs_create_dir("wilc", NULL);
	if (wilc_dir == NULL) {
		pr_err("Error creating debugfs\n");
		return -EFAULT;
	}
	for (i = 0; i < ARRAY_SIZE(debugfs_info); i++) {
		info = &debugfs_info[i];
		debugfs_create_file(info->name,
				    info->perm,
				    wilc_dir,
				    &info->data,
				    &info->fops);
	}
	return 0;
}

void wilc_debugfs_remove(void)
{
	debugfs_remove_recursive(wilc_dir);
}

#endif
