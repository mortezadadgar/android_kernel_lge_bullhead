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

static struct powergate_partition_info tegra20_powergate_partition_info[] = {
	[TEGRA_POWERGATE_CPU]  = { .name =  "cpu" },
	[TEGRA_POWERGATE_3D]   = { .name =   "3d" },
	[TEGRA_POWERGATE_VENC] = { .name = "venc" },
	[TEGRA_POWERGATE_VDEC] = { .name = "vdec" },
	[TEGRA_POWERGATE_PCIE] = { .name = "pcie" },
	[TEGRA_POWERGATE_L2]   = { .name =   "l2" },
	[TEGRA_POWERGATE_MPE]  = { .name =  "mpe" },
};

static const char *tegra20_get_powerdomain_name(int id)
{
	return tegra20_powergate_partition_info[id].name;
}

static struct powergate_ops tegra20_powergate_ops = {
	.get_powergate_domain_name = tegra20_get_powerdomain_name,
	.powergate_partition = NULL,
	.unpowergate_partition = NULL,
};

static struct powergate t20_powergate = {
	.soc_name = "tegra20",
	.num_powerdomains = 7,
	.num_cpu_domains = 0,
	.cpu_domain_map = NULL,
	.ops = &tegra20_powergate_ops,
};

struct powergate * __init tegra20_powergate_init(void)
{
	return &t20_powergate;
}

