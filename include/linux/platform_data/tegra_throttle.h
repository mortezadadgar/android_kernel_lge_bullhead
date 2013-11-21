/*
 * Copyright (c) 2010-2013 NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_THROTTLE_H
#define __TEGRA_THROTTLE_H

#include <linux/thermal.h>
#include <linux/err.h>
#include <linux/platform_device.h>

struct dentry;

struct tegra_cooling_device {
	char *cdev_type;
	int *trip_temperatures;
	int trip_temperatures_num;
};

#define MAX_THROT_TABLE_SIZE	(64)
#define NO_CAP			(ULONG_MAX) /* no cap */
#define CPU_THROT_LOW		0 /* lowest throttle freq. only used for CPU */

enum throttle_cap_freqs {
	CAP_CPU = 0,
	CAP_GPU = 1,
	CAP_C2BUS = 1,
	CAP_C3BUS,
	CAP_SCLK,
	CAP_EMC,
	NUM_OF_CAP_FREQS,
};

struct throttle_table {
	unsigned long cap_freqs[NUM_OF_CAP_FREQS];
};

struct balanced_throttle {
	struct thermal_cooling_device *cdev;
	struct list_head node;
	unsigned long cur_state;
	int throttle_count;
	int throt_tab_size;
	struct throttle_table *throt_tab;
#ifdef CONFIG_DEBUG_FS
	struct dentry *d_tab;
#endif
};

#endif
