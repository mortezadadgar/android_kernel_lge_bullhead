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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/thermal.h>
#include <linux/platform_data/tegra_thermal.h>
#include <linux/tegra-soc.h>

#include "tegra_soctherm.h"

static const unsigned long t114_default_soctherm_clk_rate = 51000000;
static const unsigned long t114_default_tsensor_clk_rate = 500000;

static const s32 t114_nominal_calib_cp = 25;
static const s32 t114_nominal_calib_ft = 90;

static const struct soctherm_sensor t114_sensor_defaults = {
	.tall      = 16300,
	.tiddq     = 1,
	.ten_count = 1,
	.tsample   = 163,
	.tsamp_ate = 655,
	.pdiv      = 10,
	.pdiv_ate  = 10,
};

static struct soctherm_tsensor_pmu_data t114_pmu_data;

static struct thermal_zone_params t114_soctherm_therm_cpu_tzp = {
	.governor_name = "pid_thermal_gov",
};

static struct soctherm_platform_data t114_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &t114_soctherm_therm_cpu_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &t114_soctherm_therm_cpu_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
		},
	},
	.throttle = {
		[THROTTLE_HEAVY] = {
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = 1,
				},
			},
		},
	},
	.tshut_pmu_trip_data = &t114_pmu_data,
};

static struct soctherm_sensor2tsensorcalib t114_tsensor_calib[] = {
	[TSENSE_CPU0] = {
		.tsensor = 0,
	},
	[TSENSE_CPU1] = {
		.tsensor = 1,
	},
	[TSENSE_CPU2] = {
		.tsensor = 2,
	},
	[TSENSE_CPU3] = {
		.tsensor = 3,
	},
	[TSENSE_MEM0] = {
		.tsensor = 5,
	},
	[TSENSE_MEM1] = {
		.tsensor = 6,
	},
	[TSENSE_GPU] = {
		.tsensor  = 4,
	},
	[TSENSE_PLLX] = {
		.tsensor = 7,
	},
};

static int t114_fuse_corr_alpha[] = { /* scaled *1000000 */
	[TSENSE_CPU0] = 1196400,
	[TSENSE_CPU1] = 1196400,
	[TSENSE_CPU2] = 1196400,
	[TSENSE_CPU3] = 1196400,
	[TSENSE_GPU]  = 1124500,
	[TSENSE_PLLX] = 1224200,
};

static int t114_fuse_corr_beta[] = { /* scaled *1000000 */
	[TSENSE_CPU0] = -13600000,
	[TSENSE_CPU1] = -13600000,
	[TSENSE_CPU2] = -13600000,
	[TSENSE_CPU3] = -13600000,
	[TSENSE_GPU]  =  -9793100,
	[TSENSE_PLLX] = -14665000,
};

static struct soctherm_fuse_calib_data t114_calib_data = {
	.tsensor_calib = t114_tsensor_calib,
	.fuse_corr_alpha = t114_fuse_corr_alpha,
	.fuse_corr_beta = t114_fuse_corr_beta,
};

#define T114_FUSE_VSENSOR_CALIB_0	0x08c
#define T114_FUSE_BASE_CP_SHIFT		0
#define T114_FUSE_BASE_CP_MASK		0x3ff
#define T114_FUSE_BASE_FT_SHIFT		16
#define T114_FUSE_BASE_FT_MASK		0x7ff
#define T114_FUSE_SHIFT_CP_SHIFT	10
#define T114_FUSE_SHIFT_CP_MASK		0x3f
#define T114_FUSE_SHIFT_CP_BITS		6
#define T114_FUSE_SHIFT_FT_SHIFT	27
#define T114_FUSE_SHIFT_FT_MASK		0x1f
#define T114_FUSE_SHIFT_FT_BITS		5

static int t114_tsensor_calib_offset[] = {
	[0] = 0x098,
	[1] = 0x084,
	[2] = 0x088,
	[3] = 0x12c,
	[4] = 0x154,
	[5] = 0x158,
	[6] = 0x15c,
	[7] = 0x160,
};

static int tegra114_fuse_get_tsensor_calib(int index, u32 *calib)
{
	if (index < 0 || index >= ARRAY_SIZE(t114_tsensor_calib_offset))
		return -EINVAL;
	*calib = tegra30_fuse_readl(t114_tsensor_calib_offset[index]);
	return 0;
}

static int tegra114_fuse_calib_base_get_cp(u32 *base_cp, s32 *shifted_cp)
{
	s32 cp;
	u32 val = tegra30_fuse_readl(T114_FUSE_VSENSOR_CALIB_0);

	if (base_cp && shifted_cp) {
		*base_cp = (((val) & (T114_FUSE_BASE_CP_MASK
				<< T114_FUSE_BASE_CP_SHIFT))
				>> T114_FUSE_BASE_CP_SHIFT);
		if (!(*base_cp))
			return -EINVAL;

		cp = (((val) & (T114_FUSE_SHIFT_CP_MASK
				<< T114_FUSE_SHIFT_CP_SHIFT))
				>> T114_FUSE_SHIFT_CP_SHIFT);

		*shifted_cp = ((s32)(cp)
				<< (32 - T114_FUSE_SHIFT_CP_BITS)
				>> (32 - T114_FUSE_SHIFT_CP_BITS));
	}

	return 0;
}

static int tegra114_fuse_calib_base_get_ft(u32 *base_ft, s32 *shifted_ft)
{
	s32 ft;
	u32 val = tegra30_fuse_readl(T114_FUSE_VSENSOR_CALIB_0);

	if (base_ft && shifted_ft) {
		*base_ft = (((val) & (T114_FUSE_BASE_FT_MASK
				<< T114_FUSE_BASE_FT_SHIFT))
				>> T114_FUSE_BASE_FT_SHIFT);
		if (!(*base_ft))
			return -EINVAL;

		ft = (((val) & (T114_FUSE_SHIFT_FT_MASK
				<< T114_FUSE_SHIFT_FT_SHIFT))
				>> T114_FUSE_SHIFT_FT_SHIFT);

		*shifted_ft = ((s32)(ft)
				<< (32 - T114_FUSE_SHIFT_FT_BITS)
				>> (32 - T114_FUSE_SHIFT_FT_BITS));
	}

	return 0;
}

static int tegra114_soctherm_fuse_read_calib_base(void)
{
	s32 calib_cp, calib_ft;
	u32 fuse_calib_base_cp, fuse_calib_base_ft;

	if (tegra114_fuse_calib_base_get_cp(&fuse_calib_base_cp, &calib_cp) ||
	    tegra114_fuse_calib_base_get_ft(&fuse_calib_base_ft, &calib_ft)) {
		return -EINVAL;
	}
	t114_calib_data.fuse_calib_base_cp = fuse_calib_base_cp;
	t114_calib_data.fuse_calib_base_ft = fuse_calib_base_ft;

	/* use HI precision to calculate: use fuse_temp in 0.5C */
	t114_calib_data.actual_temp_cp = 2 * t114_nominal_calib_cp + calib_cp;
	t114_calib_data.actual_temp_ft = 2 * t114_nominal_calib_ft + calib_ft;

	return 0;
}

static int tegra114_soctherm_fuse_get_tsensor_calib(void)
{
	u32 value;
	int i, sensor, ret;

	for (i = 0; i < TSENSE_SIZE; i++) {
		sensor = t114_calib_data.tsensor_calib[i].tsensor;
		ret = tegra114_fuse_get_tsensor_calib(sensor, &value);
		if (ret < 0)
			return ret;
		t114_calib_data.tsensor_calib[i].calib_val = value;
	}

	return 0;
}

int tegra114_soctherm_init(struct device_node *soctherm_dn)
{
	struct platform_device *pdev;
	struct soctherm_platform_data *pdata;
	struct device_node *dn;
	void __iomem *base;
	struct thermal_trip_info *trips;
	int *num_trips;

	pdev = of_find_device_by_node(soctherm_dn);
	if (!pdev) {
		dev_err(&pdev->dev, "Failed to get soctherm device\n");
		return -EINVAL;
	}

	pdata = &t114_soctherm_data;
	platform_set_drvdata(pdev, pdata);

	if (!t114_calib_data.fuse_calib_base_cp) {
		if (tegra114_soctherm_fuse_read_calib_base() < 0) {
			dev_err(&pdev->dev,
				"ERROR: Improper CP or FT calib fuse.\n");
			return -EINVAL;
		}
		if (tegra114_soctherm_fuse_get_tsensor_calib() < 0) {
			dev_err(&pdev->dev,
				"ERROR: Improper tsensor calib fuse.\n");
			return -EINVAL;
		}
		pdata->fuse_calib_data = &t114_calib_data;
	}

	trips = pdata->therm[THERM_CPU].trips;
	num_trips = &pdata->therm[THERM_CPU].num_trips;

	/* get soctherm base address */
	base = of_iomap(soctherm_dn, 0);
	if (!base) {
		dev_err(&pdev->dev, "Can't map soctherm registers\n");
		WARN_ON(1);
		return -EINVAL;
	}
	pdata->soctherm_base = base;

	/* get pmc base address */
	dn = of_find_compatible_node(NULL, NULL, "nvidia,tegra114-pmc");
	if (!dn) {
		dev_err(&pdev->dev, "Failed to find pmc node\n");
		WARN_ON(1);
		return -EINVAL;
	}
	base = of_iomap(dn, 0);
	if (!base) {
		dev_err(&pdev->dev, "Can't map pmc registers\n");
		WARN_ON(1);
		return -EINVAL;
	}
	pdata->pmc_base = base;

	/* get clk base address */
	dn = of_find_compatible_node(NULL, NULL, "nvidia,tegra114-car");
	if (!dn) {
		dev_err(&pdev->dev, "Failed to find clk node\n");
		WARN_ON(1);
		return -EINVAL;
	}
	base = of_iomap(dn, 0);
	if (!base) {
		dev_err(&pdev->dev, "Can't map clk registers\n");
		WARN_ON(1);
		return -EINVAL;
	}
	pdata->clk_reset_base = base;

	soctherm_parse_pmu_dt(pdev);

	/* initialize default sensor value */
	soctherm_init_sensor(pdev, &t114_sensor_defaults);

	/* initialize default clock rates */
	soctherm_init_clk_rate(pdev,
			       t114_default_soctherm_clk_rate,
			       t114_default_tsensor_clk_rate);

	return 0;
}
