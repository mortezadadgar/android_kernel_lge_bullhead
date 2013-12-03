/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/tegra-soc.h>

#include "fuse.h"

#define CPU_PROCESS_CORNERS_NUM	2
#define GPU_PROCESS_CORNERS_NUM	2
#define CORE_PROCESS_CORNERS_NUM	2

#define FUSE_CPU_SPEEDO_0	0x114
#define FUSE_CPU_SPEEDO_1	0x12c
#define FUSE_CPU_SPEEDO_2	0x130
#define FUSE_SOC_SPEEDO_0	0x134
#define FUSE_SOC_SPEEDO_1	0x138
#define FUSE_SOC_SPEEDO_2	0x13c
#define FUSE_CPU_IDDQ		0x118
#define FUSE_SOC_IDDQ		0x140
#define FUSE_GPU_IDDQ		0x228
#define FUSE_FT_REV		0x128

enum {
	THRESHOLD_INDEX_0,
	THRESHOLD_INDEX_1,
	THRESHOLD_INDEX_COUNT,
};

static int cpu_speedo_0_value;
static int cpu_speedo_1_value;
static int soc_speedo_0_value;
static int soc_speedo_1_value;
static int soc_speedo_2_value;
static int cpu_iddq_value;
static int gpu_iddq_value;
static int soc_iddq_value;

static const u32 cpu_process_speedos[][CPU_PROCESS_CORNERS_NUM] = {
	{2190,	UINT_MAX},
	{0,	UINT_MAX},
};

static const u32 gpu_process_speedos[][GPU_PROCESS_CORNERS_NUM] = {
	{1965,	UINT_MAX},
	{0,	UINT_MAX},
};

static const u32 core_process_speedos[][CORE_PROCESS_CORNERS_NUM] = {
	{2101,	UINT_MAX},
	{0,	UINT_MAX},
};

static void rev_sku_to_speedo_ids(int sku, int *threshold)
{
	/* Assign to default */
	tegra_cpu_speedo_id = 0;
	tegra_soc_speedo_id = 0;
	tegra_gpu_speedo_id = 0;
	*threshold = THRESHOLD_INDEX_0;

	switch (sku) {
	case 0x00: /* Eng sku */
	case 0x0F:
		/* Using the default */
		break;

	case 0x81:
	case 0x83:
		tegra_cpu_speedo_id = 2;
		break;

	case 0x07:
		tegra_cpu_speedo_id = 1;
		tegra_soc_speedo_id = 1;
		tegra_gpu_speedo_id = 1;
		*threshold = THRESHOLD_INDEX_1;
		break;

	default:
		pr_err("Tegra124 Unknown SKU %d\n", sku);
		/* Using the default for the error case */
		break;
	}
}

void tegra124_init_speedo_data(void)
{
	int i;
	int threshold;

	BUILD_BUG_ON(ARRAY_SIZE(cpu_process_speedos) !=
			THRESHOLD_INDEX_COUNT);
	BUILD_BUG_ON(ARRAY_SIZE(gpu_process_speedos) !=
			THRESHOLD_INDEX_COUNT);
	BUILD_BUG_ON(ARRAY_SIZE(core_process_speedos) !=
			THRESHOLD_INDEX_COUNT);

	cpu_speedo_0_value = tegra_fuse_readl(FUSE_CPU_SPEEDO_0);
	cpu_speedo_1_value = tegra_fuse_readl(FUSE_CPU_SPEEDO_1);

	/* GPU Speedo is stored in CPU_SPEEDO_2 */
	tegra_gpu_speedo_value = tegra_fuse_readl(FUSE_CPU_SPEEDO_2);

	soc_speedo_0_value = tegra_fuse_readl(FUSE_SOC_SPEEDO_0);
	soc_speedo_1_value = tegra_fuse_readl(FUSE_SOC_SPEEDO_1);
	soc_speedo_2_value = tegra_fuse_readl(FUSE_SOC_SPEEDO_2);

	cpu_iddq_value = tegra_fuse_readl(FUSE_CPU_IDDQ);
	soc_iddq_value = tegra_fuse_readl(FUSE_SOC_IDDQ);
	gpu_iddq_value = tegra_fuse_readl(FUSE_GPU_IDDQ);

	tegra_cpu_speedo_value = cpu_speedo_0_value;

	if (tegra_cpu_speedo_value == 0) {
		pr_warn("Tegra124: Warning: Speedo value not fused. PLEASE FIX!!!!!!!!!!!\n");
		pr_warn("Tegra124: Warning: PLEASE USE BOARD WITH FUSED SPEEDO VALUE !!!!\n");
		BUG();
	}

	rev_sku_to_speedo_ids(tegra_sku_id, &threshold);

	tegra_cpu_iddq_value = tegra_fuse_readl(FUSE_CPU_IDDQ);

	for (i = 0; i < GPU_PROCESS_CORNERS_NUM; i++)
		if (tegra_gpu_speedo_value <
			gpu_process_speedos[threshold][i])
			break;
	tegra_gpu_process_id = i;

	for (i = 0; i < CPU_PROCESS_CORNERS_NUM; i++)
		if (tegra_cpu_speedo_value <
			cpu_process_speedos[threshold][i])
				break;
	tegra_cpu_process_id = i;

	for (i = 0; i < CORE_PROCESS_CORNERS_NUM; i++)
		if (soc_speedo_0_value <
			core_process_speedos[threshold][i])
			break;
	tegra_core_process_id = i;

	pr_info("Tegra124: GPU Speedo ID=%d, Speedo Value=%d\n",
			tegra_gpu_speedo_id, tegra_gpu_speedo_value);
}

int tegra124_get_core_speedo_mv(void)
{
	switch (tegra_soc_speedo_id) {
	case 0:
	case 1:
		return 1150;
	default:
		BUG();
	}
}
