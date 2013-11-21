/*
 * Copyright (c) 2012, 2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __CLK_PLLG_H
#define __CLK_PLLG_H

#include <linux/clk/tegra.h>

struct tegra_clk_pllg_params {
	u32 min_freq, max_freq; /* MHz */
	u32 min_vco, max_vco;   /* MHz */
	u32 min_u, max_u;       /* MHz */
	u32 min_m, max_m;
	u32 min_n, max_n;
	u32 min_pl, max_pl;
};

struct clk *tegra_clk_register_pllg(const char *name, const char *parent_name,
				void __iomem *clk_base, unsigned long flags,
				struct tegra_clk_pllg_params *params,
				spinlock_t *lock);

#endif
