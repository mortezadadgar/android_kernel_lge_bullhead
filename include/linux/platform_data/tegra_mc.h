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
bool tegra124_mc_is_ready(void);
#else
static inline u32 tegra124_mc_readl(u32 offs) { return -ENODEV; }
static inline void tegra124_mc_writel(u32 val, u32 offs) {}
static inline bool tegra124_mc_is_ready(void) { return false; }
#endif

#endif
