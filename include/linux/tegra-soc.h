/*
 * Copyright (c) 2012,2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_TEGRA_SOC_H_
#define __LINUX_TEGRA_SOC_H_

#include <asm/cputype.h>

extern int tegra_sku_id;

u32 tegra_read_chipid(void);
int tegra_get_cpu_process_id(void);
int tegra_get_core_process_id(void);
int tegra_get_gpu_process_id(void);
int tegra_get_cpu_speedo_id(void);
int tegra_get_soc_speedo_id(void);
int tegra_get_gpu_speedo_id(void);
int tegra_get_cpu_speedo_value(void);
int tegra_get_gpu_speedo_value(void);
int tegra_get_cpu_iddq_value(void);
u32 tegra_get_vp8_enable(void);

enum {
	TEGRA_CLUSTER_G = 0,
	TEGRA_CLUSTER_LP = 1,
};

static inline bool is_lp_cluster(void)
{
	return MPIDR_AFFINITY_LEVEL(read_cpuid_mpidr(), 1) == TEGRA_CLUSTER_LP;
}

#ifdef CONFIG_ARCH_TEGRA_124_SOC
int tegra_switch_cluster(int new_cluster);
int tegra_cluster_control_init(void);
int tegra124_get_core_speedo_mv(void);
struct tegra_cooling_device *tegra_get_gpu_vmin_cdev(void);
struct tegra_cooling_device *tegra_get_gpu_vts_cdev(void);
#else
static inline int tegra_switch_cluster(int new_cluster)
{
	return -EINVAL;
}
static inline int tegra_cluster_control_init(void)
{
	return -EINVAL;
}
static inline int tegra124_get_core_speedo_mv(void)
{ return -EINVAL; }
static inline struct tegra_cooling_device *tegra_get_gpu_vmin_cdev(void)
{ return ERR_PTR(-EINVAL); }
static inline struct tegra_cooling_device *tegra_get_gpu_vts_cdev(void)
{ return ERR_PTR(-EINVAL); }
#endif

#ifdef CONFIG_ARM_TEGRA_CPUFREQ
int tegra_cpufreq_init(void);
#else
static inline int tegra_cpufreq_init(void)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_TEGRA_CPUQUIET
int tegra_cpuquiet_init(void);
#else
static inline int tegra_cpuquiet_init(void)
{
	return -EINVAL;
}
#endif

u32 tegra_fuse_readl(unsigned long offset);

struct gpu_info {
	int num_pixel_pipes;
	int num_alus_per_pixel_pipe;
};

void tegra_gpu_get_info(struct gpu_info *pinfo);

#if defined(CONFIG_TEGRA_GK20A) && defined(CONFIG_ARCH_TEGRA_124_SOC)
int tegra_gpu_set_speed_cap(unsigned long *speed_cap);
#else
static inline int tegra_gpu_set_speed_cap(unsigned long *speed_cap)
{ return -EINVAL; }
#endif

#endif /* __LINUX_TEGRA_SOC_H_ */
