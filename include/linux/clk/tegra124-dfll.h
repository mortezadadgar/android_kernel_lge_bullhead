/*
 * tegra124-dfll.h - prototypes and macros for the Tegra T124 DFLL clock source
 *
 * Copyright (C) 2012-2013 NVIDIA Corporation.
 * Aleksandr Frid <afrid@nvidia.com>
 * Paul Walmsley <pwalmsley@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_CLK_TEGRA124_DFLL_H_
#define _LINUX_CLK_TEGRA124_DFLL_H_

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

enum tegra_dfll_therm_type {
	TEGRA_DFLL_THERM_FLOOR = 0,
	TEGRA_DFLL_THERM_CAP,
};

/* Function prototypes */

#ifdef CONFIG_CLK_TEGRA_T124_DFLL
extern int tegra124_dfll_suspend(struct platform_device *pdev);
extern void tegra124_dfll_resume(struct platform_device *pdev);
extern int tegra124_dfll_lock_loop(void);
extern int tegra124_dfll_unlock_loop(void);
extern int tegra124_dfll_get_fv_table(int *num_freqs, unsigned long **freqs,
		int **millivolts);

/* Thermal interface */
extern int tegra124_dfll_update_thermal_index(struct platform_device *pdev,
					      enum tegra_dfll_therm_type type,
					      unsigned long new_idx);
extern int tegra124_dfll_get_thermal_index(struct platform_device *pdev,
					   enum tegra_dfll_therm_type type);
extern int tegra124_dfll_count_therm_states(struct platform_device *pdev,
					    enum tegra_dfll_therm_type type);
extern int tegra124_dfll_get_therm_state_temp(struct platform_device *pdev,
					      enum tegra_dfll_therm_type type,
					      unsigned long index);
extern int tegra124_dfll_attach_thermal(struct platform_device *pdev,
					enum tegra_dfll_therm_type type,
					struct thermal_cooling_device *cdev);
extern int tegra124_dfll_detach_thermal(struct platform_device *pdev,
					enum tegra_dfll_therm_type type,
					struct thermal_cooling_device *cdev);
#else
static inline int tegra124_dfll_suspend(struct platform_device *pdev)
{ return 0; }
static inline void tegra124_dfll_resume(struct platform_device *pdev)
{ }
static inline int tegra124_dfll_lock_loop(void)
{ return -EPERM; }
static inline int tegra124_dfll_unlock_loop(void)
{ return -EPERM; }
static inline int tegra124_dfll_get_fv_table(int *num_freqs,
		unsigned long **freqs, int **millivolts)
{return -EPERM; }
static inline int tegra124_dfll_update_thermal_index(
						struct platform_device *pdev,
						enum tegra_dfll_therm_type type,
						unsigned long new_idx)
{ return 0; }
static inline int tegra124_dfll_get_thermal_index(struct platform_device *pdev,
						enum tegra_dfll_therm_type type)
{ return 0; }
static inline int tegra124_dfll_count_therm_states(struct platform_device *pdev,
						enum tegra_dfll_therm_type type)
{ return 0; }
static inline int tegra124_dfll_get_therm_state_temp(
						struct platform_device *pdev,
						enum tegra_dfll_therm_type type,
						unsigned long index)
{ return -EINVAL; }
static inline int tegra124_dfll_attach_thermal(struct platform_device *pdev,
					enum tegra_dfll_therm_type type,
					struct thermal_cooling_device *cdev)
{ return 0; }
static inline int tegra124_dfll_detach_thermal(struct platform_device *pdev,
					enum tegra_dfll_therm_type type,
					struct thermal_cooling_device *cdev)
{ return 0; }
#endif
#endif
