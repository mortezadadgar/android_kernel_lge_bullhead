/*
 * Clock driver for ams AS3722 device.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/mfd/as3722.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct as3722_clks {
	struct device *dev;
	struct as3722 *as3722;
	struct clk_hw hw;
	struct clk *clk;
	struct clk_onecell_data clk_data;
};

static inline struct as3722_clks *to_as3722_clks(struct clk_hw *hw)
{
	return container_of(hw, struct as3722_clks, hw);
}

static unsigned long as3722_clks_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	return 32768;
}

static int as3722_clks_prepare(struct clk_hw *hw)
{
	struct as3722_clks *as3722_clks = to_as3722_clks(hw);
	int ret;

	ret = as3722_update_bits(as3722_clks->as3722, AS3722_RTC_CONTROL_REG,
			AS3722_RTC_CLK32K_OUT_EN, AS3722_RTC_CLK32K_OUT_EN);
	if (ret < 0)
		dev_err(as3722_clks->dev, "RTC_CONTROL_REG update failed, %d\n",
			ret);

	return ret;
}

static void as3722_clks_unprepare(struct clk_hw *hw)
{
	struct as3722_clks *as3722_clks = to_as3722_clks(hw);
	int ret;

	ret = as3722_update_bits(as3722_clks->as3722, AS3722_RTC_CONTROL_REG,
			AS3722_RTC_CLK32K_OUT_EN, 0);
	if (ret < 0)
		dev_err(as3722_clks->dev, "RTC_CONTROL_REG update failed, %d\n",
			ret);
}

static int as3722_clks_is_prepared(struct clk_hw *hw)
{
	struct as3722_clks *as3722_clks = to_as3722_clks(hw);
	int ret;
	u32 val;

	ret = as3722_read(as3722_clks->as3722, AS3722_RTC_CONTROL_REG, &val);
	if (ret < 0) {
		dev_err(as3722_clks->dev, "RTC_CONTROL_REG read failed, %d\n",
			ret);
		return ret;
	}

	return !!(val & AS3722_RTC_CLK32K_OUT_EN);
}

static struct clk_ops as3722_clks_ops = {
	.prepare	= as3722_clks_prepare,
	.unprepare	= as3722_clks_unprepare,
	.is_prepared	= as3722_clks_is_prepared,
	.recalc_rate	= as3722_clks_recalc_rate,
};

static struct clk_init_data as3722_clks_hw_init = {
	.name = "clk32k",
	.ops = &as3722_clks_ops,
	.flags = CLK_IS_ROOT | CLK_IGNORE_UNUSED,
};

static int as3722_clks_probe(struct platform_device *pdev)
{
	struct as3722_clks *as3722_clks;
	struct clk *clk;
	int ret;

	as3722_clks = devm_kzalloc(&pdev->dev, sizeof(*as3722_clks),
				GFP_KERNEL);
	if (!as3722_clks)
		return -ENOMEM;

	as3722_clks->clk_data.clks = devm_kzalloc(&pdev->dev,
			sizeof(*as3722_clks->clk_data.clks), GFP_KERNEL);
	if (!as3722_clks->clk_data.clks)
		return -ENOMEM;

	platform_set_drvdata(pdev, as3722_clks);

	as3722_clks->as3722 = dev_get_drvdata(pdev->dev.parent);
	as3722_clks->dev = &pdev->dev;
	as3722_clks->hw.init = &as3722_clks_hw_init;

	clk = devm_clk_register(&pdev->dev, &as3722_clks->hw);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev, "Fail to register clock %s, %d\n",
			as3722_clks_hw_init.name, ret);
		return ret;
	}
	as3722_clks->clk = clk;
	as3722_clks->clk_data.clks[0] = clk;
	as3722_clks->clk_data.clk_num = 1;
	ret = of_clk_add_provider(pdev->dev.parent->of_node,
			of_clk_src_simple_get, &as3722_clks->clk_data);
	if (ret < 0)
		dev_err(&pdev->dev, "Fail to add clock driver, %d\n", ret);
	return ret;
}

static int as3722_clks_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.parent->of_node);
	return 0;
}

static struct platform_driver as3722_clks_driver = {
	.driver = {
		.name = "as3722-clk",
		.owner = THIS_MODULE,
	},
	.probe = as3722_clks_probe,
	.remove = as3722_clks_remove,
};

module_platform_driver(as3722_clks_driver);

MODULE_DESCRIPTION("Clock driver for ams AS3722 PMIC Device");
MODULE_ALIAS("platform:as3722-clk");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
