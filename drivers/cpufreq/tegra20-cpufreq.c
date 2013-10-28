/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/suspend.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "tegra-cpufreq.h"

static struct cpufreq_frequency_table freq_table[] = {
	{ .frequency = 216000 },
	{ .frequency = 312000 },
	{ .frequency = 456000 },
	{ .frequency = 608000 },
	{ .frequency = 760000 },
	{ .frequency = 816000 },
	{ .frequency = 912000 },
	{ .frequency = 1000000 },
	{ .frequency = CPUFREQ_TABLE_END },
};

static struct clk *cpu_clk;
static struct clk *pll_x_clk;
static struct clk *pll_p_clk;
static struct clk *emc_clk;

static void tegra20_emc_clk_set_rate(unsigned long rate)
{
	if (rate >= 816000)
		clk_set_rate(emc_clk, 600000000); /* cpu 816 MHz, emc max */
	else if (rate >= 456000)
		clk_set_rate(emc_clk, 300000000); /* cpu 456 MHz, emc 150Mhz */
	else
		clk_set_rate(emc_clk, 100000000);  /* emc 50Mhz */
}

static int tegra20_cpu_clk_set_rate(unsigned long rate)
{
	int ret;

	/*
	 * Take an extra reference to the main pll so it doesn't turn
	 * off when we move the cpu off of it
	 */
	clk_prepare_enable(pll_x_clk);

	ret = clk_set_parent(cpu_clk, pll_p_clk);
	if (ret) {
		pr_err("Failed to switch cpu to clock pll_p\n");
		goto out;
	}

	if (rate == clk_get_rate(pll_p_clk))
		goto out;

	ret = clk_set_rate(pll_x_clk, rate);
	if (ret) {
		pr_err("Failed to change pll_x to %lu\n", rate);
		goto out;
	}

	ret = clk_set_parent(cpu_clk, pll_x_clk);
	if (ret) {
		pr_err("Failed to switch cpu to clock pll_x\n");
		goto out;
	}

out:
	clk_disable_unprepare(pll_x_clk);
	return ret;
}

static void tegra20_cpufreq_clk_init(void)
{
	clk_prepare_enable(emc_clk);
	clk_prepare_enable(cpu_clk);
}

static void tegra20_cpufreq_clk_exit(void)
{
	clk_disable_unprepare(cpu_clk);
	clk_disable_unprepare(emc_clk);
}

static const struct tegra_cpufreq_config tegra20_cpufreq_config = {
	.emc_clk_set_rate = tegra20_emc_clk_set_rate,
	.cpu_clk_set_rate = tegra20_cpu_clk_set_rate,
	.cpufreq_clk_init = tegra20_cpufreq_clk_init,
	.cpufreq_clk_exit = tegra20_cpufreq_clk_exit,
};

int tegra20_cpufreq_init(struct tegra_cpufreq_data *data,
		const struct tegra_cpufreq_config **soc_config)
{
	cpu_clk = clk_get_sys(NULL, "cclk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	pll_x_clk = clk_get_sys(NULL, "pll_x");
	if (IS_ERR(pll_x_clk))
		return PTR_ERR(pll_x_clk);

	pll_p_clk = clk_get_sys(NULL, "pll_p");
	if (IS_ERR(pll_p_clk))
		return PTR_ERR(pll_p_clk);

	emc_clk = clk_get_sys("cpu", "emc");
	if (IS_ERR(emc_clk)) {
		clk_put(cpu_clk);
		return PTR_ERR(emc_clk);
	}

	data->cpu_clk = cpu_clk;
	data->freq_table = freq_table;
	*soc_config = &tegra20_cpufreq_config;

	return 0;
}
EXPORT_SYMBOL(tegra20_cpufreq_init);
