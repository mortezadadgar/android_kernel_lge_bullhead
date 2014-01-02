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

#define MAX_THROT_TABLE_SIZE	(64)
#define NO_CAP			(ULONG_MAX) /* no cap */

enum throttle_cap_freqs {
	CAP_CPU = 0,
	CAP_GPU,
	CAP_C3BUS,
	CAP_SCLK,
	CAP_EMC,
	NUM_OF_CAP_FREQS,
};

struct throttle_table {
	unsigned long cap_freqs[NUM_OF_CAP_FREQS];
};

struct balanced_throttle_instance {
	struct thermal_cooling_device *cdev;
	struct device_node *np;
	struct list_head node;
	unsigned long cur_state;
	int throttle_count;
	int throt_tab_size;
	struct throttle_table *throt_tab;
#ifdef CONFIG_DEBUG_FS
	struct dentry *d_tab;
#endif
};

struct tegra_balanced_throttle {
	int num_throt;
	const struct cpufreq_frequency_table *cpu_freq_table;
	unsigned long cpu_throttle_lowest_speed;
	unsigned long bthrot_speed;
};

struct tegra_throttle_cap_data {
	const char *cap_name;
	struct clk *cap_clk;
	unsigned long cap_freq;
	unsigned long max_freq;
};

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
struct thermal_cooling_device *balanced_throttle_register(
		struct balanced_throttle_instance *bthrot, char *type);
void balanced_throttle_unregister(struct balanced_throttle_instance *bthrot);
void tegra_throttling_enable(bool enable);
int tegra_throttle_init(struct platform_device *pdev,
			struct tegra_balanced_throttle *b_throt,
			struct tegra_throttle_cap_data *cap_freqs_table,
			int cap_freqs_table_size);
#else
static inline struct thermal_cooling_device *balanced_throttle_register(
		struct balanced_throttle_instance *bthrot, char *type)
{ return ERR_PTR(-ENODEV); }
static inline void balanced_throttle_unregister(
		struct balanced_throttle_instance *bthrot)
{ return; }
static inline void tegra_throttling_enable(bool enable)
{ return; }
static inline int tegra_throttle_init(
		struct platform_device *pdev,
		struct tegra_balanced_throttle *b_throt,
		struct tegra_throttle_cap_data *cap_freqs_table,
		int cap_freqs_table_size)
{ return 0; }
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */

#endif	/* __TEGRA_THROTTLE_H */
