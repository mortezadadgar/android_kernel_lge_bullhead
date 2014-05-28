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
#include <linux/clk-provider.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-dvfs.h>
#include <linux/clk/tegra124-dfll.h>

#include "tegra-cpufreq.h"

#define MAX_DVFS_FREQS		40
#define LP_BACKUP_FREQ		204000
#define CPU_FREQ_STEP		102000 /* 102MHz cpu_g table step */
#define CPU_FREQ_TABLE_MAX_SIZE (2 * MAX_DVFS_FREQS + 1)
#define CLUSTER_SWITCH_THRESHOLD 696000000

static struct clk *cpu_clk;
static struct clk *fcpu_clk;
static struct clk *lpcpu_clk;
static struct clk *pll_main_clk;
static struct clk *pll_backup_clk;
static struct clk *cpu_dfll_clk;
static struct clk *emc_clk;

static unsigned long dfll_mode_threshold;
static struct cpufreq_frequency_table
		tegra124_cpufreq_tables[CPU_FREQ_TABLE_MAX_SIZE];
static cpumask_t online_cpus;

static DEFINE_MUTEX(tegra124_cpu_lock);

static int tegra124_switch_cluster(unsigned int cluster)
{
	int ret;

	pr_debug("Switching to cluster %u\n", cluster);

	ret = tegra_switch_cluster(cluster);
	if (is_lp_cluster())
		cpu_clk = lpcpu_clk;
	else
		cpu_clk = fcpu_clk;

	return ret;
}

static int tegra124_cpu_clk_dfll_on(struct clk *cpu_clk,
		unsigned long new_rate, unsigned long old_rate)
{
	int ret;

	ret = clk_set_rate(cpu_dfll_clk, new_rate);
	if (ret) {
		pr_err("Failed to set cpu rate %lu on source %s\n",
				new_rate, __clk_get_name(cpu_dfll_clk));
		goto out;
	}

	if (clk_get_parent(cpu_clk) == cpu_dfll_clk)
		return 0;

	ret = clk_set_parent(cpu_clk, cpu_dfll_clk);
	if (ret) {
		pr_err("Failed to switch cpu to %s with error: %d\n",
				__clk_get_name(cpu_dfll_clk), ret);
		goto out;
	}
	ret = tegra124_dfll_lock_loop();
	WARN(ret, "Failed to lock %s at rate %lu\n",
			__clk_get_name(cpu_dfll_clk), new_rate);

	tegra_dvfs_dfll_mode_set(cpu_clk, new_rate);

	pr_debug("CPU DFLL Mode ----> ON\n");

	return 0;
out:
	return ret;
}

int tegra124_cpu_clk_dfll_off(struct clk *cpu_clk,
		unsigned long new_rate, unsigned long old_rate)
{
	int ret;
	struct clk *pll;

	pll = (new_rate <= LP_BACKUP_FREQ * 1000) ?
		pll_backup_clk : pll_main_clk;

	if (old_rate != dfll_mode_threshold) {
		ret = tegra_dvfs_set_rate(cpu_clk, dfll_mode_threshold);
		if (!ret)
			ret = clk_set_rate(cpu_dfll_clk, dfll_mode_threshold);

		if (ret) {
			pr_err("Failed to set cpu rate %lu on source %s\n",
			       dfll_mode_threshold,
			       __clk_get_name(cpu_dfll_clk));
			return ret;
		}
	}

	ret = tegra124_dfll_unlock_loop();
	if (ret) {
		pr_err("Failed to unlock %s\n", __clk_get_name(cpu_dfll_clk));
		goto back_to_dfll;
	}

	ret = tegra_dvfs_dfll_mode_clear(cpu_clk, new_rate);
	if (ret) {
		pr_err("Failed to set cpu rail for rate %lu\n", new_rate);
		goto back_to_dfll;
	}

	ret = clk_set_rate(pll, new_rate);
	if (ret) {
		pr_err("Failed to set cpu rate %lu on source %s\n",
				new_rate, __clk_get_name(pll_main_clk));
		goto back_to_dfll;
	}

	ret = clk_set_parent(cpu_clk, pll);
	if (ret) {
		pr_err("Failed to switch cpu to %s\n", __clk_get_name(pll));
		goto back_to_dfll;
	}

	tegra_dvfs_set_rate(cpu_clk, new_rate);

	pr_debug("CPU DFLL Mode ----> OFF\n");

	return 0;

back_to_dfll:
	tegra124_dfll_lock_loop();
	tegra_dvfs_dfll_mode_set(cpu_clk, new_rate);

	return ret;
}

static void tegra124_emc_clk_set_rate(unsigned long cpu_rate)
{
	unsigned long emc_max_rate = 0;

	if (IS_ERR(emc_clk))
		return;

	emc_max_rate = clk_round_rate(emc_clk, ULONG_MAX);

	/* Vote on memory bus frequency based on cpu frequency */
	if (cpu_rate >= 1400000)
		clk_set_rate(emc_clk, emc_max_rate);
	else if (cpu_rate >= 1200000)
		clk_set_rate(emc_clk, 750000000);
	else if (cpu_rate >= 1100000)
		clk_set_rate(emc_clk, 600000000);
	else if (cpu_rate >= 1000000)
		clk_set_rate(emc_clk, 500000000);
	else if (cpu_rate >= 800000)
		clk_set_rate(emc_clk, 375000000);
	else if (cpu_rate >= 500000)
		clk_set_rate(emc_clk, 200000000);
	else if (cpu_rate >= 250000)
		clk_set_rate(emc_clk, 100000000);
	else
		clk_set_rate(emc_clk, 0);
}

static int tegra124_set_rate_pll(unsigned long new_rate, unsigned long old_rate)
{
	int ret;

	clk_prepare_enable(pll_main_clk);

	if (cpu_clk == fcpu_clk && new_rate > old_rate)
		tegra_dvfs_set_rate(cpu_clk, new_rate);

	ret = clk_set_parent(cpu_clk, pll_backup_clk);
	if (ret) {
		pr_err("Tegra124 cpufreq: Failed to reparent cpu to pll_p\n");
		goto out;
	}

	if (new_rate == clk_get_rate(pll_backup_clk))
		goto out;

	ret = clk_set_rate(pll_main_clk, new_rate);
	if (ret) {
		pr_err("Tegra124 cpufreq: Failed to change pll_x to %lu\n",
				new_rate);
		goto out;
	}

	ret = clk_set_parent(cpu_clk, pll_main_clk);
	if (ret) {
		pr_err("Tegra124 cpufreq: Failed to reparent cpu to pll_x\n");
		goto out;
	}

out:
	if (cpu_clk == fcpu_clk && new_rate <= old_rate)
		tegra_dvfs_set_rate(cpu_clk, new_rate);

	clk_disable_unprepare(pll_main_clk);

	return ret;
}

static int tegra124_cpu_clk_set_rate(unsigned long new_rate)
{
	int *dfll_mv;
	unsigned long *freqs;
	int num_dfll_freqs = 0;
	unsigned long old_rate = clk_get_rate(cpu_clk);
	bool has_dfll = IS_ERR(cpu_dfll_clk) ? false : true;
	int ret = 0;

	if (tegra124_dfll_get_fv_table(&num_dfll_freqs, &freqs, &dfll_mv))
		pr_err("Tegra124 cpufreq: Failed to get dfll fv_table\n");

	mutex_lock(&tegra124_cpu_lock);

	/* Switch back to G cluster if we're in G range. */
	if (is_lp_cluster() && new_rate >= CLUSTER_SWITCH_THRESHOLD) {
		ret = tegra124_switch_cluster(TEGRA_CLUSTER_G);
		if (ret)
			goto out;
	}

	if (!has_dfll || !num_dfll_freqs || is_lp_cluster()) {
		/* DFLL is not supported so we set CPU rate on PLL. */
		ret = tegra124_set_rate_pll(new_rate, old_rate);
	} else {
		/* DFLL is supported. */
		if (new_rate >= dfll_mode_threshold)
			ret = tegra124_cpu_clk_dfll_on(cpu_clk, new_rate,
							old_rate);
		else if (clk_get_parent(cpu_clk) == cpu_dfll_clk)
			ret = tegra124_cpu_clk_dfll_off(cpu_clk, new_rate,
							old_rate);
		else
			ret = tegra124_set_rate_pll(new_rate, old_rate);
	}
	if (ret < 0)
		goto out;

	/* If only CPU0 is up and we're lower than G range, switch. */
	if (cpumask_weight(&online_cpus) == 1 &&
	    cpumask_first(&online_cpus) == 0 &&
	    new_rate < CLUSTER_SWITCH_THRESHOLD)
		ret = tegra124_switch_cluster(TEGRA_CLUSTER_LP);

out:
	mutex_unlock(&tegra124_cpu_lock);

	return ret;
}

static void tegra124_cpufreq_clk_init(void)
{
	if (!IS_ERR(emc_clk))
		clk_prepare_enable(emc_clk);
	BUG_ON(!__clk_is_enabled(cpu_clk) && !__clk_is_prepared(cpu_clk));
}

static void tegra124_cpufreq_clk_exit(void)
{
	if (!IS_ERR(emc_clk))
		clk_disable_unprepare(emc_clk);
}

static const struct tegra_cpufreq_config tegra124_cpufreq_config = {
	.emc_clk_set_rate = tegra124_emc_clk_set_rate,
	.cpu_clk_set_rate = tegra124_cpu_clk_set_rate,
	.cpufreq_clk_init = tegra124_cpufreq_clk_init,
	.cpufreq_clk_exit = tegra124_cpufreq_clk_exit,
};

static int tegra124_prepare_tables(struct tegra_cpufreq_data *data)
{
	int i, j, ret, num_lp_freqs, num_g_freqs;
	int *g_millivolts;
	bool g_vmin_done = false;
	unsigned long *freqs_lp, *freqs_g;
	unsigned long g_vmin_freq, g_start_freq, max_freq, freq;

	ret = tegra_dvfs_get_freqs(lpcpu_clk, &freqs_lp, &num_lp_freqs);
	if (ret || !num_lp_freqs) {
		pr_err("Tegra124 cpufreq: Failed to get LP CPU freq-table\n");
		goto out;
	}

	ret = tegra124_dfll_get_fv_table(&num_g_freqs, &freqs_g, &g_millivolts);
	if (ret || !num_g_freqs) {
		pr_err("Tegra124 cpufreq: Failed to get G CPU freq-table\n");
		goto out;
	}

	g_vmin_freq = freqs_g[0];
	if (g_vmin_freq < LP_BACKUP_FREQ * 1000) {
		WARN(1, "%s: LP CPU backup rate exceeds G CPU rate at Vmin\n",
				__func__);
		return -EINVAL;
	}

	/* Start with singular frequency */
	i = 0;
	tegra124_cpufreq_tables[i++].frequency = LP_BACKUP_FREQ;

	/*
	 * Next, set table steps along LP CPU dvfs ladder, but make sure G CPU
	 * dvfs rate at minimum voltage is not missed (if it happens to be below
	 * LP maximum rate)
	 */
	max_freq = freqs_lp[num_lp_freqs - 1];
	for (j = 0; j < num_lp_freqs; j++) {
		freq = freqs_lp[j];

		if (freq <= LP_BACKUP_FREQ)
			continue;

		if (!g_vmin_done && freq >= g_vmin_freq) {
			g_vmin_done = true;
			if (freq > g_vmin_freq && g_vmin_freq / 1000 >
			    tegra124_cpufreq_tables[i - 1].frequency)
				tegra124_cpufreq_tables[i++].frequency
					= g_vmin_freq / 1000;
		}
		tegra124_cpufreq_tables[i++].frequency = freq / 1000;

		if (freq == max_freq)
			break;
	}

	/*  Fill in the non-DFLL CPU G frequencies above LP CPU maximum rate */
	if (freq < g_vmin_freq) {
		int n = (g_vmin_freq - freq) / CPU_FREQ_STEP;
		for (j = 0; j <= n; j++) {
			freq = g_vmin_freq - CPU_FREQ_STEP * (n - j);
			tegra124_cpufreq_tables[i++].frequency = freq / 1000;
		}
	}

	/* Now, step along the rest of G CPU dvfs ladder */
	g_start_freq = freq;

	for (j = 0; j < num_g_freqs; j++) {
		freq = freqs_g[j];
		if (freq > g_start_freq)
			tegra124_cpufreq_tables[i++].frequency = freq / 1000;
	}
	tegra124_cpufreq_tables[i].frequency = CPUFREQ_TABLE_END;

	data->suspend_index = i - 1;

	return 0;
out:
	return ret;
}

static int tegra124_cpu_notifier(struct notifier_block *nb,
					unsigned long action, void *cpu)
{
	int ret = NOTIFY_OK;

	if (action != CPU_UP_PREPARE && action != CPU_DEAD)
		return NOTIFY_OK;

	mutex_lock(&tegra124_cpu_lock);
	if (action == CPU_DEAD)
		cpumask_clear_cpu((unsigned int)cpu, &online_cpus);
	else {
		cpumask_set_cpu((unsigned int)cpu, &online_cpus);
		if (is_lp_cluster() &&
		    tegra124_switch_cluster(TEGRA_CLUSTER_G) != 0)
			ret = NOTIFY_BAD;
	}
	mutex_unlock(&tegra124_cpu_lock);

	return ret;
}

static struct notifier_block tegra124_cpu_nb = {
	.notifier_call = tegra124_cpu_notifier,
};

int tegra124_cpufreq_init(struct tegra_cpufreq_data *data,
		const struct tegra_cpufreq_config **soc_config)
{
	int ret;

	fcpu_clk = devm_clk_get(data->dev, "cclk_g");
	if (IS_ERR(fcpu_clk))
		return PTR_ERR(fcpu_clk);

	lpcpu_clk = devm_clk_get(data->dev, "cclk_lp");
	if (IS_ERR(lpcpu_clk))
		return PTR_ERR(lpcpu_clk);

	pll_main_clk = devm_clk_get(data->dev, "pll_x");
	if (IS_ERR(pll_main_clk))
		return PTR_ERR(pll_main_clk);

	pll_backup_clk = devm_clk_get(data->dev, "pll_p_out4");
	if (IS_ERR(pll_backup_clk))
		return PTR_ERR(pll_backup_clk);

	emc_clk = devm_clk_get(data->dev, "cpu.emc");
	if (IS_ERR(emc_clk))
		pr_info("Tegra124 cpufreq: no cpu.emc shared clock found\n");

	cpu_dfll_clk = devm_clk_get(data->dev, "dfllCPU_out");
	if (IS_ERR(cpu_dfll_clk))
		pr_info("Tegra124 cpufreq: no DFLL clock found\n");

	/* Start with fast CPU */
	cpu_clk = fcpu_clk;

	ret = tegra_dvfs_get_dfll_threshold(cpu_clk, &dfll_mode_threshold);
	if (ret) {
		ret = -EPROBE_DEFER;
		pr_err("Tegra124 cpufreq: failed to get dfll_mode_threshold\n");
		goto out;
	}

	ret = tegra124_prepare_tables(data);
	if (ret)
		goto out;

	ret = tegra_cluster_control_init();
	if (ret)
		goto out;

	cpumask_copy(&online_cpus, cpu_online_mask);
	register_cpu_notifier(&tegra124_cpu_nb);

	data->cpu_clk = cpu_clk;
	data->freq_table = tegra124_cpufreq_tables;
	*soc_config = &tegra124_cpufreq_config;

	return 0;
out:
	return ret;
}
EXPORT_SYMBOL(tegra124_cpufreq_init);
