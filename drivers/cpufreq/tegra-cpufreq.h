/*
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

#ifndef __LINUX_TEGRA_CPUFREQ_H
#define __LINUX_TEGRA_CPUFREQ_H

struct tegra_cpufreq_config {
	void (*emc_clk_set_rate)(unsigned long);
	int (*cpu_clk_set_rate)(unsigned long);
	void (*cpufreq_clk_init)(void);
	void (*cpufreq_clk_exit)(void);
};

struct tegra_cpufreq_data {
	struct device	*dev;
	struct clk	*cpu_clk;
	struct cpufreq_frequency_table	*freq_table;
	int suspend_index;
};

#ifdef CONFIG_ARM_TEGRA20_CPUFREQ
int tegra20_cpufreq_init(struct tegra_cpufreq_data *data,
		const struct tegra_cpufreq_config **config);
#else
static inline int tegra20_cpufreq_init(struct tegra_cpufreq_data *data,
		const struct tegra_cpufreq_config **config)
{ return -EINVAL; }
#endif

#ifdef CONFIG_ARM_TEGRA124_CPUFREQ
int tegra124_cpufreq_init(struct tegra_cpufreq_data *data,
		const struct tegra_cpufreq_config **config);
#else
static inline int tegra124_cpufreq_init(struct tegra_cpufreq_data *data,
		const struct tegra_cpufreq_config **config)
{ return -EINVAL; }
#endif

#endif /* __LINUX_TEGRA_CPUFREQ_H_ */
