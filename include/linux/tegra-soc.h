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

#define TEGRA20         0x20
#define TEGRA30         0x30
#define TEGRA114        0x35
#define TEGRA124        0x40

#ifndef __ASSEMBLY__

#include <asm/cputype.h>
#include <linux/err.h>

enum tegra_revision {
	TEGRA_REVISION_UNKNOWN = 0,
	TEGRA_REVISION_A01,
	TEGRA_REVISION_A02,
	TEGRA_REVISION_A03,
	TEGRA_REVISION_A03p,
	TEGRA_REVISION_A04,
	TEGRA_REVISION_MAX,
};

u32 tegra_read_straps(void);
void tegra_init_fuse(void);

extern int tegra_chip_id;
extern enum tegra_revision tegra_revision;
extern int tegra_bct_strapping;
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
#endif

#ifdef CONFIG_TEGRA_GK20A
struct tegra_cooling_device *tegra_get_gpu_vmin_cdev(void);
struct tegra_cooling_device *tegra_get_gpu_vts_cdev(void);
#else
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

u32 tegra30_fuse_readl(const unsigned int offset);

#ifdef CONFIG_TEGRA_KFUSE
/* there are 144 32-bit values in total */
#define TEGRA_KFUSE_DATA_SZ (144 * 4)

int tegra_kfuse_read(void *dest, size_t len);
#endif

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


#if defined(CONFIG_TEGRA20_APB_DMA)
int tegra_apb_readl_using_dma(unsigned long offset, u32 *value);
int tegra_apb_writel_using_dma(u32 value, unsigned long offset);
#else
static inline int tegra_apb_readl_using_dma(unsigned long offset, u32 *value)
{
	return -EINVAL;
}
static inline int tegra_apb_writel_using_dma(u32 value, unsigned long offset)
{
	return -EINVAL;
}
#endif

void tegra_pmc_remove_dpd_req(void);
void tegra_pmc_clear_dpd_sample(void);

#endif /* __ASSEMBLY__ */

#endif /* __LINUX_TEGRA_SOC_H_ */
