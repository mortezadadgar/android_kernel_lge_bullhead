/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#ifndef __DRIVERS_MISC_TEGRA_FUSE_H
#define __DRIVERS_MISC_TEGRA_FUSE_H

struct tegra_sku_info {
	int sku_id;
	int cpu_process_id;
	int cpu_speedo_id;
	int cpu_speedo_value;
	int cpu_iddq_value;
	int core_process_id;
	int soc_speedo_id;
	int gpu_speedo_id;
	int gpu_process_id;
	int gpu_speedo_value;
	enum tegra_revision revision;
};

int tegra_fuse_create_sysfs(struct device *dev, int size,
	u32 (*readl)(const unsigned int offset),
	u32 (*write)(const unsigned int offset, const char *buf, u32 size),
	struct tegra_sku_info *sku_info);

bool tegra30_spare_fuse(int bit);
u32 tegra30_fuse_readl(const unsigned int offset);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void tegra20_init_speedo_data(struct tegra_sku_info *sku_info,
			      struct device *dev);
bool tegra20_spare_fuse(int bit);
bool tegra20_spare_fuse_early(int spare_bit, void *fuse_base);
#else
static inline void tegra20_init_speedo_data(struct tegra_sku_info *sku_info,
					    struct device *dev) {}
static inline bool tegra20_spare_fuse(int bit) { return false; }
static inline bool tegra20_spare_fuse_early(int spare_bit, void *fuse_base)
{
			return false;
}
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
void tegra30_init_speedo_data(struct tegra_sku_info *sku_info,
			      struct device *dev);
#else
static inline void tegra30_init_speedo_data(struct tegra_sku_info *sku_info,
					    struct device *dev) {}
#endif

#ifdef CONFIG_ARCH_TEGRA_114_SOC
void tegra114_init_speedo_data(struct tegra_sku_info *sku_info,
			       struct device *dev);
#else
static inline void tegra114_init_speedo_data(struct tegra_sku_info *sku_info,
					     struct device *dev) {}
#endif

#ifdef CONFIG_ARCH_TEGRA_124_SOC
void tegra124_init_speedo_data(struct tegra_sku_info *sku_info,
			       struct device *dev);
#else
static inline void tegra124_init_speedo_data(struct tegra_sku_info *sku_info,
					     struct device *dev) {}
#endif

#endif
