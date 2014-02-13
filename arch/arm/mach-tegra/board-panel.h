/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __MACH_TEGRA_BOARD_PANEL_H
#define __MACH_TEGRA_BOARD_PANEL_H

#include <linux/platform_device.h>
#include <linux/platform_data/tegra_dc.h>

#ifdef CONFIG_TEGRA_DC
extern atomic_t sd_brightness;
extern struct platform_pwm_backlight_data venice_bl_data;
extern struct tegra_dc_platform_data venice_disp1_pdata;
extern struct tegra_dc_platform_data venice_disp2_pdata;

extern int venice_panel_init(void);
#else
static inline int venice_panel_init(void)
{
	return -EINVAL;
}
#endif

#endif /* __MACH_TEGRA_BOARD_PANEL_H */
