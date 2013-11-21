/*
 * Copyright (c) 2012-2013 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/tegra-dvfs.h>
#include <linux/platform_data/tegra_thermal.h>
#include <linux/platform_data/tegra_emc.h>
#include <linux/platform_device.h>

#include "tegra_throttle.h"

static struct tegra_balanced_throttle b_throt;
static struct balanced_throttle_instance *cpu_throt;

static struct tegra_throttle_cap_data tegra124_cap_freqs_table[] = {
	{ .cap_name = "cap.throttle.c3bus" },
	{ .cap_name = "cap.throttle.sclk" },
	{ .cap_name = "cap.throttle.emc" },
};

static int tegra124_throt_parse_dt(struct platform_device *pdev,
				   char *property_name,
				   struct balanced_throttle_instance *throttle)
{
	struct throttle_table *throttle_table;
	struct device_node *t_dn, *tc_dn;
	const __be32 *prop;
	int num;

	prop = of_get_property(pdev->dev.of_node, property_name, NULL);
	if (!prop) {
		dev_err(&pdev->dev, "No throttle table in DT data\n");
		return -EINVAL;
	}

	t_dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!t_dn) {
		dev_err(&pdev->dev, "No throttle table in DT data\n");
		return -EINVAL;
	}

	num = of_get_child_count(t_dn);
	if (!num) {
		dev_err(&pdev->dev, "No throttle table in DT data\n");
		of_node_put(t_dn);
		return -EINVAL;
	}

	throttle_table = devm_kzalloc(&pdev->dev,
				      sizeof(struct throttle_table) * num,
				      GFP_KERNEL);
	if (!throttle_table) {
		dev_err(&pdev->dev, "allocate throttle table failed\n");
		of_node_put(t_dn);
		return -ENOMEM;
	}

	num = 0;
	for_each_child_of_node(t_dn, tc_dn) {
		u32 val;

		if (of_property_read_u32(tc_dn, "cpu-freq", &val)) {
			dev_err(&pdev->dev, "missing cpu-freq, ignore this state\n");
			continue;
		} else {
			throttle_table[num].cap_freqs[CAP_CPU] = val;
		}

		if (of_property_read_u32(tc_dn, "gpu-freq", &val))
			throttle_table[num].cap_freqs[CAP_GPU] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_GPU] = val;

		if (of_property_read_u32(tc_dn, "c3bus-freq", &val))
			throttle_table[num].cap_freqs[CAP_C3BUS] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_C3BUS] = val;

		if (of_property_read_u32(tc_dn, "sclk-freq", &val))
			throttle_table[num].cap_freqs[CAP_SCLK] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_SCLK] = val;

		if (of_property_read_u32(tc_dn, "emc-freq", &val))
			throttle_table[num].cap_freqs[CAP_EMC] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_EMC] = val;

		num++;
	}

	throttle->throt_tab_size = num;
	throttle->throt_tab = throttle_table;

	return 0;
}

static bool tegra_throttle_cap_clk_is_ready(struct platform_device *pdev)
{
	/* Check if emc scaling and dvfs are ready. */
	if (!tegra124_emc_is_ready()) {
		dev_warn(&pdev->dev, "Emc scalling is not ready.\n");
		return false;
	}

	if (IS_ERR(tegra_dvfs_get_core_vmin_cdev())) {
		dev_warn(&pdev->dev, "dvfs for cap_clk is not ready.\n");
		return false;
	}

	return true;
}

static int tegra124_throttle_probe(struct platform_device *pdev)
{
	int ret;

	if (!tegra_throttle_cap_clk_is_ready(pdev)) {
		dev_warn(&pdev->dev,
			 "Throttle cap clks are not ready.\n");
		return -EPROBE_DEFER;
	}

	ret = tegra_throttle_init(pdev, &b_throt, tegra124_cap_freqs_table,
				  ARRAY_SIZE(tegra124_cap_freqs_table));
	if (ret)
		return ret;

	/* register cpu-balanced */
	cpu_throt = devm_kzalloc(&pdev->dev,
				 sizeof(*cpu_throt), GFP_KERNEL);
	if (!cpu_throt)
		return -ENOMEM;

	if (tegra124_throt_parse_dt(pdev, "cpu-balanced-states", cpu_throt)) {
		dev_err(&pdev->dev, "Parse cpu throttle table failed.\n");
		return -EINVAL;
	}

	balanced_throttle_register(cpu_throt, "cpu-balanced");

	dev_info(&pdev->dev,
		 "Tegra junction temperature throttling initialized\n");

	return 0;
}

static int tegra124_throttle_remove(struct platform_device *pdev)
{
	balanced_throttle_unregister(cpu_throt);

	return 0;
}

static const struct of_device_id tegra124_throttle_match[] = {
	{ .compatible = "nvidia,tegra124-tj-throttle", },
	{},
};

static struct platform_driver tegra124_throttle_driver = {
	.probe	= tegra124_throttle_probe,
	.remove	= tegra124_throttle_remove,
	.driver = {
		.name	= "tegra124-throttle",
		.owner	= THIS_MODULE,
		.of_match_table = tegra124_throttle_match,
	},
};

static int __init tegra124_throttle_init(void)
{
	return platform_driver_register(&tegra124_throttle_driver);
}
module_init(tegra124_throttle_init);

static void __exit tegra124_throttle_exit(void)
{
	platform_driver_unregister(&tegra124_throttle_driver);
}
module_exit(tegra124_throttle_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Junction throttle driver");
