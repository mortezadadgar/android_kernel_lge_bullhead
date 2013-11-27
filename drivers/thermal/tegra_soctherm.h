/*
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __TEGRA_SOCTHERM_H
#define __TEGRA_SOCTHERM_H

/* This order must match the soc_therm HW register spec */
enum soctherm_sense {
	TSENSE_CPU0 = 0,
	TSENSE_CPU1,
	TSENSE_CPU2,
	TSENSE_CPU3,
	TSENSE_MEM0,
	TSENSE_MEM1,
	TSENSE_GPU,
	TSENSE_PLLX,
	TSENSE_SIZE,
};

/* This order must match the soc_therm HW register spec */
enum soctherm_therm_id {
	THERM_CPU = 0,
	THERM_GPU,
	THERM_MEM,
	THERM_PLL,
	THERM_SIZE,
};

enum soctherm_throttle_id {
	THROTTLE_LIGHT = 0,
	THROTTLE_HEAVY,
	THROTTLE_SIZE,
};

enum soctherm_throttle_dev_id {
	THROTTLE_DEV_CPU = 0,
	THROTTLE_DEV_GPU,
	THROTTLE_DEV_SIZE,
	THROTTLE_DEV_NONE,
};

struct soctherm_sensor {
	bool sensor_enable;
	bool zone_enable;
	int tall;
	int tiddq;
	int ten_count;
	int tsample;
	int tsamp_ate;
	u8 pdiv;
	u8 pdiv_ate;
};

struct soctherm_therm {
	bool zone_enable;
	int passive_delay;
	int num_trips;
	int hotspot_offset;
	struct thermal_trip_info trips[THERMAL_MAX_TRIPS];
	struct thermal_zone_params *tzp;
};

struct soctherm_throttle_dev {
	bool enable;
	u8 dividend;
	u8 divisor;
	u16 duration;
	u8 step;
};

struct soctherm_throttle {
	u8 priority;
	struct soctherm_throttle_dev devs[THROTTLE_DEV_SIZE];
};

struct soctherm_tsensor_pmu_data {
	u8 poweroff_reg_data;
	u8 poweroff_reg_addr;
	u8 controller_type;
	u8 i2c_controller_id;
	u8 pinmux;
	u8 pmu_16bit_ops;
	u8 pmu_i2c_addr;
};

struct soctherm_sensor2tsensorcalib {
	int tsensor;
	u32 calib_val;
};

struct soctherm_fuse_calib_data {
	u32 fuse_calib_base_cp;
	u32 fuse_calib_base_ft;
	u32 actual_temp_cp;
	u32 actual_temp_ft;
	struct soctherm_sensor2tsensorcalib *tsensor_calib;
	int *fuse_corr_alpha;
	int *fuse_corr_beta;
};

struct soctherm_platform_data {
	struct soctherm_sensor sensor_data[TSENSE_SIZE];
	struct soctherm_therm therm[THERM_SIZE];
	struct soctherm_throttle throttle[THROTTLE_SIZE];
	struct soctherm_tsensor_pmu_data *tshut_pmu_trip_data;
	struct soctherm_fuse_calib_data *fuse_calib_data;

	unsigned int thermal_irq;

	void __iomem *soctherm_base;
	void __iomem *pmc_base;
	void __iomem *clk_reset_base;

	struct clk *soctherm_clk;
	struct clk *tsensor_clk;
	unsigned long soctherm_clk_rate;
	unsigned long tsensor_clk_rate;

	bool read_hw_temp;
};

int soctherm_parse_pmu_dt(struct platform_device *pdev);
void soctherm_init_sensor(struct platform_device *pdev,
			  const struct soctherm_sensor *sensor);
void soctherm_init_clk_rate(struct platform_device *pdev,
			    unsigned long scotherm_clk_rate,
			    unsigned long tsensor_clk_rate);
void soctherm_add_trip_points(struct platform_device *pdev,
			      struct thermal_trip_info *trips,
			      int *num_trips,
			      struct tegra_cooling_device *cdev_data);

#ifdef CONFIG_ARCH_TEGRA_114_SOC
int tegra114_soctherm_init(struct device_node *soctherm_dn);
#else
static inline int tegra114_soctherm_init(struct device_node *soctherm_dn)
{ return 0; }
#endif

#ifdef CONFIG_ARCH_TEGRA_124_SOC
int tegra124_soctherm_init(struct device_node *soctherm_dn);
#else
static inline int tegra124_soctherm_init(struct device_node *soctherm_dn)
{ return 0; }
#endif

#endif /* __TEGRA_SOCTHERM_H */
