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

#ifndef __TEGRA_MC_H_
#define __TEGRA_MC_H_

#include <linux/of.h>

#ifdef CONFIG_TEGRA114_MC
u32 tegra114_mc_readl(u32 offs);
void tegra114_mc_writel(u32 val, u32 offs);
#else
static inline u32 tegra114_mc_readl(u32 offs) { return -ENODEV; }
static inline void tegra114_mc_writel(u32 val, u32 offs) {}
#endif

#ifdef CONFIG_TEGRA124_MC
u32 tegra124_mc_readl(u32 offs);
void tegra124_mc_writel(u32 val, u32 offs);
int tegra124_mc_check_vpr(void);
#else
static inline u32 tegra124_mc_readl(u32 offs) { return -ENODEV; }
static inline void tegra124_mc_writel(u32 val, u32 offs) {}
static int tegra124_mc_check_vpr(void) { return -ENODEV; }
#endif

#define TEGRA_MC_CLIENT_AFI		0
#define TEGRA_MC_CLIENT_DC		2
#define TEGRA_MC_CLIENT_DCB		3
#define TEGRA_MC_CLIENT_EPP		4
#define TEGRA_MC_CLIENT_G2		5
#define TEGRA_MC_CLIENT_ISP		8
#define TEGRA_MC_CLIENT_MSENC		11
#define TEGRA_MC_CLIENT_MPE		11
#define TEGRA_MC_CLIENT_NV		12
#define TEGRA_MC_CLIENT_SATA		15
#define TEGRA_MC_CLIENT_VDE		16
#define TEGRA_MC_CLIENT_VI		17
#define TEGRA_MC_CLIENT_VIC		18
#define TEGRA_MC_CLIENT_XUSB_HOST	19
#define TEGRA_MC_CLIENT_XUSB_DEV	20
#define TEGRA_MC_CLIENT_TSEC		22
#define TEGRA_MC_CLIENT_ISPB		33
#define TEGRA_MC_CLIENT_GPU		34

#ifdef CONFIG_TEGRA124_MC
int tegra_mc_flush(int id);
int tegra_mc_flush_done(int id);
#else
static inline int tegra_mc_flush(int id)
{ return 0; }
static inline int tegra_mc_flush_done(int id)
{ return 0; }
#endif

int tegra124_mc_register_notify(struct notifier_block *nb);

static __maybe_unused inline int tegra_mc_get_effective_bytes_width(void)
{
	if (of_machine_is_compatible("nvidia,tegra124"))
		return 8;
	else
		return 4;
}

#endif
