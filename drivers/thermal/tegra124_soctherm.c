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
#include <linux/platform_data/tegra_edp.h>
#include <linux/tegra-soc.h>

#include "tegra_soctherm.h"

static const unsigned long t124_default_soctherm_clk_rate = 51000000;
static const unsigned long t124_default_tsensor_clk_rate = 400000;

static const s32 t124_nominal_calib_cp = 25;
static const s32 t124_nominal_calib_ft = 105;

static const struct soctherm_sensor t124_sensor_defaults = {
	.tall      = 16300,
	.tiddq     = 1,
	.ten_count = 1,
	.tsample   = 120,
	.tsamp_ate = 481,
	.pdiv      = 8,
	.pdiv_ate  = 8,
};

static struct soctherm_tsensor_pmu_data t124_pmu_data;

static struct thermal_zone_params t124_soctherm_therm_cpu_tzp = {
	.governor_name = "pid_thermal_gov",
};

static bool t124_cpu_edp_trips_done;

static struct soctherm_platform_data t124_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 86000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 96000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 98000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &t124_soctherm_therm_cpu_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 88000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 98000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &t124_soctherm_therm_cpu_tzp,
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
	.tshut_pmu_trip_data = &t124_pmu_data,
};

static struct soctherm_sensor2tsensorcalib t124_tsensor_calib[] = {
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

static int t124_fuse_corr_alpha[] = { /* scaled *1000000 */
	[TSENSE_CPU0] = 1148300,
	[TSENSE_CPU1] = 1126100,
	[TSENSE_CPU2] = 1155800,
	[TSENSE_CPU3] = 1134900,
	[TSENSE_MEM0] = 1062700,
	[TSENSE_MEM1] = 1084700,
	[TSENSE_GPU]  = 1084300,
	[TSENSE_PLLX] = 1134500,
};

static int t124_fuse_corr_beta[] = { /* scaled *1000000 */
	[TSENSE_CPU0] =  -6572300,
	[TSENSE_CPU1] =  -5794600,
	[TSENSE_CPU2] =  -7462800,
	[TSENSE_CPU3] =  -6810800,
	[TSENSE_MEM0] =  -4463200,
	[TSENSE_MEM1] =  -5603400,
	[TSENSE_GPU]  =  -5111900,
	[TSENSE_PLLX] =  -7410700,
};

static struct soctherm_fuse_calib_data t124_calib_data = {
	.tsensor_calib = t124_tsensor_calib,
	.fuse_corr_alpha = t124_fuse_corr_alpha,
	.fuse_corr_beta = t124_fuse_corr_beta,
};

/* fuse registers used in public fuse data read API */
#define T124_FUSE_FT_REV		0x128
#define T124_FUSE_CP_REV		0x190
/* sparse realignment register */
#define T124_FUSE_SPARE_REALIGNMENT_REG_0	0x2fc
/* tsensor8_calib */
#define T124_FUSE_TSENSOR_CALIB_8	0x280

#define T124_FUSE_BASE_CP_SHIFT		0
#define T124_FUSE_BASE_CP_MASK		0x3ff
#define T124_FUSE_BASE_FT_SHIFT		10
#define T124_FUSE_BASE_FT_MASK		0x7ff
#define T124_FUSE_SHIFT_CP_SHIFT	0
#define T124_FUSE_SHIFT_CP_MASK		0x3f
#define T124_FUSE_SHIFT_CP_BITS		6
#define T124_FUSE_SHIFT_FT_SHIFT	21
#define T124_FUSE_SHIFT_FT_MASK		0x1f
#define T124_FUSE_SHIFT_FT_BITS		5

static int t124_tsensor_calib_offset[] = {
	[0] = 0x198,
	[1] = 0x184,
	[2] = 0x188,
	[3] = 0x22c,
	[4] = 0x254,
	[5] = 0x258,
	[6] = 0x25c,
	[7] = 0x260,
};

static int tegra124_fuse_get_tsensor_calib(int index, u32 *calib)
{
	if (index < 0 || index >= ARRAY_SIZE(t124_tsensor_calib_offset))
		return -EINVAL;
	*calib = tegra_fuse_readl(t124_tsensor_calib_offset[index]);
	return 0;
}

static int fuse_cp_rev_check(void)
{
	u32 rev, rev_major, rev_minor;

	rev = tegra_fuse_readl(T124_FUSE_CP_REV);
	rev_minor = rev & 0x1f;
	rev_major = (rev >> 5) & 0x3f;
	/* CP rev < 00.4 is unsupported */
	if ((rev_major == 0) && (rev_minor < 4))
		return -EINVAL;

	return 0;
}

static inline int fuse_ft_rev_check(void)
{
	u32 rev, rev_major, rev_minor;

	rev = tegra_fuse_readl(T124_FUSE_FT_REV);
	rev_minor = rev & 0x1f;
	rev_major = (rev >> 5) & 0x3f;
	/* FT rev < 00.5 is unsupported */
	if ((rev_major == 0) && (rev_minor < 5))
		return -EINVAL;

	return 0;
}

static int tegra124_fuse_calib_base_get_cp(u32 *base_cp, s32 *shifted_cp)
{
	s32 cp;
	u32 val = tegra_fuse_readl(T124_FUSE_TSENSOR_CALIB_8);

	if (fuse_cp_rev_check() || !val)
		return -EINVAL;

	if (base_cp && shifted_cp) {
		*base_cp = (((val) & (T124_FUSE_BASE_CP_MASK
				<< T124_FUSE_BASE_CP_SHIFT))
				>> T124_FUSE_BASE_CP_SHIFT);

		val = tegra_fuse_readl(T124_FUSE_SPARE_REALIGNMENT_REG_0);
		cp = (((val) & (T124_FUSE_SHIFT_CP_MASK
				<< T124_FUSE_SHIFT_CP_SHIFT))
				>> T124_FUSE_SHIFT_CP_SHIFT);

		*shifted_cp = ((s32)(cp)
				<< (32 - T124_FUSE_SHIFT_CP_BITS)
				>> (32 - T124_FUSE_SHIFT_CP_BITS));
	}

	return 0;
}

static int tegra124_fuse_calib_base_get_ft(u32 *base_ft, s32 *shifted_ft)
{
	s32 ft;
	u32 val = tegra_fuse_readl(T124_FUSE_TSENSOR_CALIB_8);

	if (fuse_ft_rev_check() || !val)
		return -EINVAL;

	if (base_ft && shifted_ft) {
		*base_ft = (((val) & (T124_FUSE_BASE_FT_MASK
				<< T124_FUSE_BASE_FT_SHIFT))
				>> T124_FUSE_BASE_FT_SHIFT);

		ft = (((val) & (T124_FUSE_SHIFT_FT_MASK
				<< T124_FUSE_SHIFT_FT_SHIFT))
				>> T124_FUSE_SHIFT_FT_SHIFT);

		*shifted_ft = ((s32)(ft)
				<< (32 - T124_FUSE_SHIFT_FT_BITS)
				>> (32 - T124_FUSE_SHIFT_FT_BITS));
	}

	return 0;
}

static int tegra124_soctherm_fuse_read_calib_base(void)
{
	s32 calib_cp, calib_ft;
	u32 fuse_calib_base_cp, fuse_calib_base_ft;

	if (tegra124_fuse_calib_base_get_cp(&fuse_calib_base_cp, &calib_cp) ||
	    tegra124_fuse_calib_base_get_ft(&fuse_calib_base_ft, &calib_ft)) {
		return -EINVAL;
	}
	t124_calib_data.fuse_calib_base_cp = fuse_calib_base_cp;
	t124_calib_data.fuse_calib_base_ft = fuse_calib_base_ft;

	/* use HI precision to calculate: use fuse_temp in 0.5C */
	t124_calib_data.actual_temp_cp = 2 * t124_nominal_calib_cp + calib_cp;
	t124_calib_data.actual_temp_ft = 2 * t124_nominal_calib_ft + calib_ft;

	return 0;
}

static int tegra124_soctherm_fuse_get_tsensor_calib(void)
{
	u32 value;
	int i, sensor, ret;

	for (i = 0; i < TSENSE_SIZE; i++) {
		sensor = t124_calib_data.tsensor_calib[i].tsensor;
		ret = tegra124_fuse_get_tsensor_calib(sensor, &value);
		if (ret < 0)
			return ret;
		t124_calib_data.tsensor_calib[i].calib_val = value;
	}

	return 0;
}

static int tegra124_soctherm_parse_cdev(struct device_node *soctherm_dn,
					char *property_name,
					const char **cdev_cap_type,
					const char **cdev_floor_type,
					struct platform_device **pdev)
{
	struct device_node *dn_cdev, *dn;
	const __be32 *prop;
	int ret = 0;

	dn_cdev = NULL;
	dn = NULL;

	prop = of_get_property(soctherm_dn, property_name, NULL);
	if (!prop) {
		ret = -EINVAL;
		goto error;
	}
	dn_cdev = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!dn_cdev) {
		ret = -ENOENT;
		goto error;
	}

	*cdev_cap_type = of_get_property(dn_cdev, "cdev-cap-type", NULL);
	*cdev_floor_type = of_get_property(dn_cdev, "cdev-floor-type", NULL);
	if (IS_ERR_OR_NULL(*cdev_floor_type) &&
	    IS_ERR_OR_NULL(*cdev_cap_type)) {
		ret = -EINVAL;
		goto error;
	}

	prop = of_get_property(dn_cdev, "act-dev", NULL);
	if (!prop) {
		ret = -EINVAL;
		goto error;
	}
	dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!dn) {
		ret = -ENOENT;
		goto error;
	}

	*pdev = of_find_device_by_node(dn);
	if (!*pdev) {
		ret = -ENOENT;
		goto error;
	}

error:
	if (dn)
		of_node_put(dn);
	if (dn_cdev)
		of_node_put(dn_cdev);

	return ret;
}

int tegra124_soctherm_init(struct device_node *soctherm_dn)
{
	struct platform_device *pdev, *pdev_cdev;
	struct soctherm_platform_data *pdata;
	struct device_node *dn;
	void __iomem *base;
	struct thermal_trip_info *trips;
	int *num_trips;
	const char *cdev_cap_type, *cdev_floor_type;
	int err;

	pdev = of_find_device_by_node(soctherm_dn);
	if (!pdev) {
		dev_err(&pdev->dev, "Failed to get soctherm device\n");
		return -EINVAL;
	}

	pdata = &t124_soctherm_data;
	platform_set_drvdata(pdev, pdata);

	if (!t124_calib_data.fuse_calib_base_cp) {
		if (tegra124_soctherm_fuse_read_calib_base() < 0) {
			dev_err(&pdev->dev,
				"ERROR: Improper CP or FT calib fuse.\n");
			return -EINVAL;
		}
		if (tegra124_soctherm_fuse_get_tsensor_calib() < 0) {
			dev_err(&pdev->dev,
				"ERROR: Improper tsensor calib fuse.\n");
			return -EINVAL;
		}
		pdata->fuse_calib_data = &t124_calib_data;
	}

	trips = pdata->therm[THERM_CPU].trips;
	num_trips = &pdata->therm[THERM_CPU].num_trips;

	/* add trips of cpu edp */
	if (!t124_cpu_edp_trips_done) {
		err = tegra124_soctherm_parse_cdev(soctherm_dn, "cpu-edp-cdev",
						   &cdev_cap_type,
						   &cdev_floor_type,
						   &pdev_cdev);
		if (err || IS_ERR_OR_NULL(cdev_cap_type)) {
			dev_warn(&pdev->dev, "Can't find cpu-edp node\n");
			goto ignore_cpu_edp;
		}

		/* edp temperature margin is 8000 */
		err = tegra_cpu_edp_init_trips(pdev_cdev,
					       trips,
					       num_trips,
					       (char *)cdev_cap_type,
					       8000);
		if (err == -EPROBE_DEFER)
			return err;
		else if (err)
			goto ignore_cpu_edp;
		else
			t124_cpu_edp_trips_done = true;
	}

ignore_cpu_edp:

	/* get soctherm base address */
	base = of_iomap(soctherm_dn, 0);
	if (!base) {
		dev_err(&pdev->dev, "Can't map soctherm registers\n");
		WARN_ON(1);
		return -EINVAL;
	}
	pdata->soctherm_base = base;

	/* get pmc base address */
	dn = of_find_compatible_node(NULL, NULL, "nvidia,tegra124-pmc");
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
	dn = of_find_compatible_node(NULL, NULL, "nvidia,tegra124-car");
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
	soctherm_init_sensor(pdev, &t124_sensor_defaults);

	/* initialize default clock rates */
	soctherm_init_clk_rate(pdev,
			       t124_default_soctherm_clk_rate,
			       t124_default_tsensor_clk_rate);

	return 0;
}
