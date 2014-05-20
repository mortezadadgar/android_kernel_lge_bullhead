/*
 * gk20a rail gating driver
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/platform_data/tegra_mc.h>
#include <linux/types.h>
#include <linux/clk/tegra.h>

#include "dev.h"
#include "gk20a.h"
#include "gk20a_dvfs.h"
#include "gk20a_power.h"

#define MC_CLIENT_GPU 34

#define TEGRA_PMC_BASE 0x7000E400

#define PMC_GPU_RG_CNTRL_0		0x2d4

#define HOTRESET_READ_COUNT	5

static struct clk *ref_clk, *pwr_clk;

static void __iomem *pmc_base;

static int pmc_init(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "nvidia,tegra124-pmc");
	if (!node) {
		WARN(1, "failed to find pmc node\n");
		return -EPERM;
	}

	pmc_base = of_iomap(node, 0);
	if (!pmc_base) {
		WARN(1, "failed to map pmc base\n");
		return -EPERM;
	}

	return 0;
}

static void pmc_write(u32 val, unsigned long reg)
{
	int ret = 0;

	if (!pmc_base)
		ret = pmc_init();

	if (!ret)
		writel(val, pmc_base + reg);
}

static inline int get_clks(struct platform_device *pdev)
{
	if (!ref_clk) {
		ref_clk = devm_clk_get(&pdev->dev, "gpu");
		if (IS_ERR(ref_clk)) {
			WARN(1, "failed to get reference clock\n");
			return -EPERM;
		}
	}

	/* pllp_out_5 */
	if (!pwr_clk) {
		pwr_clk = devm_clk_get(&pdev->dev, "pwr");
		if (IS_ERR(pwr_clk)) {
			WARN(1, "failed to get pwr clock\n");
			return -EPERM;
		}
	}

	return 0;
}

/*
 * Public interfaces
 */

/**
 * gk20a_power_on - do the power on sequence for gk20a
 *
 * @pdev: the platform device instance of gk20a
 * @g: the gk20a instance
 */
int gk20a_power_on(struct platform_device *pdev, struct gk20a *g)
{
	int ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	ret = tegra124_mc_check_vpr();
	if (ret) {
		dev_err(&pdev->dev, "%s aborting due to incorrect vpr setting\n",
				__func__);
		return ret;
	}

	ret = gk20a_dvfs_enable_rail(pdev, g);
	if (ret) {
		WARN(1, "failed to power on gk20a rail\n");
		return ret;
	}

	ret = get_clks(pdev);
	if (ret)
		return ret;

	clk_prepare_enable(ref_clk);
	clk_prepare_enable(pwr_clk);
	udelay(10);

	tegra_periph_reset_assert(ref_clk);
	udelay(10);

	/* disable clamp */
	pmc_write(0, PMC_GPU_RG_CNTRL_0);
	udelay(10);

	tegra_periph_reset_deassert(ref_clk);
	udelay(10);

	/* mc flush done */
	tegra_mc_flush_done(MC_CLIENT_GPU);
	udelay(10);

	return 0;
}

/**
 * gk20a_power_off - do the power off sequence for gk20a
 *
 * @pdev: the platform device instance of gk20a
 * @g: the gk20a instance
 */
int gk20a_power_off(struct platform_device *pdev, struct gk20a *g)
{
	int ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	ret = get_clks(pdev);
	if (ret)
		return ret;

	tegra_mc_flush(MC_CLIENT_GPU);
	udelay(10);

	/* enable clamp */
	pmc_write(0x1, PMC_GPU_RG_CNTRL_0);
	udelay(10);

	tegra_periph_reset_assert(ref_clk);
	udelay(10);

	clk_disable_unprepare(ref_clk);
	clk_disable_unprepare(pwr_clk);
	udelay(10);

	ret = gk20a_dvfs_disable_rail(pdev, g);
	if (ret)
		return ret;

	return 0;
}

