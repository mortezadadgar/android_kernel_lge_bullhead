/*
 * gk20a DVFS header
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
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

#ifndef _GK20A_DVFS_H_
#define _GK20A_DVFS_H_

#define MAX_DVFS_FREQS		40
#define MAX_DVFS_TABLES		80
#define MAX_THERMAL_FLOORS	8
#define MAX_THERMAL_LIMITS	8
#define MAX_THERMAL_RANGES	(MAX_THERMAL_LIMITS + 1)

struct gk20a_cvb_parameters {
	int	c0;
	int	c1;
	int	c2;
	int	c3;
	int	c4;
	int	c5;
};

struct gk20a_cvb_table {
	unsigned long freq;
	struct gk20a_cvb_parameters cvb_param;
};

struct gk20a_cvb_info {
	int speedo_id;
	int process_id;

	int max_mv;
	int min_mv;
	int speedo_scale;
	int voltage_scale;
	int thermal_scale;

	struct gk20a_cvb_table cvb_table[MAX_DVFS_FREQS];

	int vmin_trips_table[MAX_THERMAL_LIMITS];
	int therm_floors_table[MAX_THERMAL_LIMITS];
	int vts_trips_table[MAX_THERMAL_LIMITS];
};

struct gk20a_dvfs_rail {
	const char *supply_name;
	struct device *dev;
	struct regulator *reg;

	u32 max_uv;
	u32 min_uv;
	u32 nominal_uv;
	int step_uv;

	const int *therm_mv_floors;
	int therm_mv_floors_num;

	int therm_floor_idx;
	int therm_scale_idx;

	struct tegra_cooling_device *vmin_cdev;
	struct tegra_cooling_device *vts_cdev;
};

/**
 * gk20a_dvfs - gk20a dvfs instance
 *
 * @g: the gk20a instance
 * @rail: the instance to the connected rail
 * @num_freqs: the number of the available frequencies for gk20a
 * @freqs: the available frequencies for gk20a
 * @millivolts: the required voltages for the frequency with the same index
 * @therm_dvfs: true if the dvfs table depends on the thermal
 */
struct gk20a_dvfs {
	struct gk20a *g;
	struct gk20a_dvfs_rail *rail;
	int speedo_id;
	int process_id;
	int freqs_mult;

	int num_freqs;
	unsigned long freqs[MAX_DVFS_FREQS];
	int *millivolts;
	int max_millivolts;
	int cur_millivolts;
	unsigned long int cur_rate;

	bool therm_dvfs;
};

int gk20a_dvfs_init(struct platform_device *pdev, struct gk20a *g);
int gk20a_dvfs_adjust_voltage(struct gk20a *g, unsigned long rate);
int gk20a_dvfs_get_freqs(struct gk20a *g, unsigned long **freqs,
	int *num_freqs);
unsigned long gk20a_dvfs_get_min_freq(struct gk20a *g);
unsigned long gk20a_dvfs_get_max_freq(struct gk20a *g);
int gk20a_dvfs_enable_rail(struct platform_device *pdev, struct gk20a *g);
int gk20a_dvfs_disable_rail(struct platform_device *pdev, struct gk20a *g);
int gk20a_dvfs_init_dvfs_table(struct platform_device *pdev, struct gk20a *g);

#endif
