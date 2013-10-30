/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/spinlock.h>
#include <linux/tegra-powergate.h>

static struct powergate_partition_info tegra114_powergate_partition_info[] = {
	[TEGRA_POWERGATE_CPU]	= { .name =  "cpu0" },
	[TEGRA_POWERGATE_3D]	= { .name =    "3d" },
	[TEGRA_POWERGATE_VENC]	= { .name =  "venc" },
	[TEGRA_POWERGATE_VDEC]	= { .name =  "vdec" },
	[TEGRA_POWERGATE_MPE]	= { .name =   "mpe" },
	[TEGRA_POWERGATE_HEG]	= { .name =   "heg" },
	[TEGRA_POWERGATE_CPU1]	= { .name =  "cpu1" },
	[TEGRA_POWERGATE_CPU2]	= { .name =  "cpu2" },
	[TEGRA_POWERGATE_CPU3]	= { .name =  "cpu3" },
	[TEGRA_POWERGATE_CELP]	= { .name =  "celp" },
	[TEGRA_POWERGATE_CPU0]	= { .name =  "cpu0" },
	[TEGRA_POWERGATE_C0NC]	= { .name =  "c0nc" },
	[TEGRA_POWERGATE_C1NC]	= { .name =  "c1nc" },
	[TEGRA_POWERGATE_DIS]	= { .name =   "dis" },
	[TEGRA_POWERGATE_DISB]	= { .name =  "disb" },
	[TEGRA_POWERGATE_XUSBA]	= { .name = "xusba" },
	[TEGRA_POWERGATE_XUSBB]	= { .name = "xusbb" },
	[TEGRA_POWERGATE_XUSBC]	= { .name = "xusbc" },
};

static const u8 tegra114_cpu_domains[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const char *tegra114_get_powerdomain_name(int id)
{
	return tegra114_powergate_partition_info[id].name;
}

static struct powergate_ops tegra114_powergate_ops = {
	.get_powergate_domain_name = tegra114_get_powerdomain_name,
	.powergate_partition = NULL,
	.unpowergate_partition = NULL,
};

static struct powergate t114_powergate = {
	.soc_name = "tegra114",
	.num_powerdomains = 23,
	.num_cpu_domains = 4,
	.cpu_domain_map = tegra114_cpu_domains,
	.ops = &tegra114_powergate_ops,
};

struct powergate * __init tegra114_powergate_init(void)
{
	return &t114_powergate;
}

