/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Based on arch/arm/plat-omap/cpu-omap.c, (C) 2005 Nokia Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
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

#define MAX_CPUS	4

static struct tegra_cpufreq_data *tegra_data;
static const struct tegra_cpufreq_config *soc_config;

static unsigned long target_cpu_speed[MAX_CPUS];
static DEFINE_MUTEX(tegra_cpu_lock);
static bool is_suspended;

static int tegra_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, tegra_data->freq_table);
}

static unsigned int tegra_getspeed(unsigned int cpu)
{
	unsigned long rate;
	int i;

	rate = clk_get_rate(tegra_data->cpu_clk) / 1000;
	/*
	 * Round to the neareset frequency.  The actual frequency generated
	 * by the CPU clock may differ slightly from the rate in the table.
	 */
	for (i = 0;
	     tegra_data->freq_table[i + 1].frequency != CPUFREQ_TABLE_END;
	     i++) {
		if (rate < (tegra_data->freq_table[i].frequency +
			    tegra_data->freq_table[i + 1].frequency) / 2)
			return tegra_data->freq_table[i].frequency;
	}

	return tegra_data->freq_table[i].frequency;
}

static int tegra_update_cpu_speed(struct cpufreq_policy *policy,
		unsigned long rate)
{
	int ret = 0;
	struct cpufreq_freqs freqs;

	freqs.old = tegra_getspeed(0);
	freqs.new = rate;

	if (freqs.old == freqs.new)
		return ret;

	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * If cpu frequency is increasing, boost memory bandwidth first
	 */
	if (soc_config->emc_clk_set_rate && freqs.new > freqs.old)
		soc_config->emc_clk_set_rate(rate);

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-tegra: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif

	ret = soc_config->cpu_clk_set_rate(freqs.new * 1000);
	if (ret) {
		pr_err("cpu-tegra: Failed to set cpu frequency to %d kHz\n",
			freqs.new);
		freqs.new = freqs.old;
	}

	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * If cpu frequency is decreasing, lower memory bandwidth later
	 */
	if (soc_config->emc_clk_set_rate && freqs.new < freqs.old)
		soc_config->emc_clk_set_rate(rate);

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static unsigned long tegra_cpu_highest_speed(void)
{
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i)
		rate = max(rate, target_cpu_speed[i]);
	return rate;
}

static int tegra_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	struct cpufreq_frequency_table *freq_table = tegra_data->freq_table;
	unsigned int idx;
	unsigned int freq;
	int ret = 0;

	mutex_lock(&tegra_cpu_lock);

	if (is_suspended) {
		ret = -EBUSY;
		goto out;
	}

	cpufreq_frequency_table_target(policy, freq_table, target_freq,
		relation, &idx);

	freq = freq_table[idx].frequency;

	target_cpu_speed[policy->cpu] = freq;

	ret = tegra_update_cpu_speed(policy, tegra_cpu_highest_speed());

out:
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int tegra_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	struct cpufreq_frequency_table *freq_table = tegra_data->freq_table;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);

	mutex_lock(&tegra_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("Tegra cpufreq suspend: setting frequency to %d kHz\n",
			freq_table[tegra_data->suspend_index].frequency);
		tegra_update_cpu_speed(policy,
			freq_table[tegra_data->suspend_index].frequency);
		cpufreq_cpu_put(policy);
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;
		tegra_update_cpu_speed(policy, policy->max);
		pr_info("Tegra cpufreq resume: restore frequency to %u kHz\n",
				policy->max);
		cpufreq_cpu_put(policy);
	}
	mutex_unlock(&tegra_cpu_lock);

	return NOTIFY_OK;
}

static struct notifier_block tegra_cpu_pm_notifier = {
	.notifier_call = tegra_pm_notify,
};

static int tegra_cpu_init(struct cpufreq_policy *policy)
{
	if (soc_config->cpufreq_clk_init)
		soc_config->cpufreq_clk_init();

	cpufreq_frequency_table_cpuinfo(policy, tegra_data->freq_table);
	cpufreq_frequency_table_get_attr(tegra_data->freq_table, policy->cpu);
	policy->cur = tegra_getspeed(policy->cpu);
	target_cpu_speed[policy->cpu] = policy->cur;

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	cpumask_copy(policy->cpus, cpu_possible_mask);

	if (policy->cpu == 0)
		register_pm_notifier(&tegra_cpu_pm_notifier);

	return 0;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, tegra_data->freq_table);
	if (soc_config->cpufreq_clk_exit)
		soc_config->cpufreq_clk_exit();
	return 0;
}

static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver tegra_cpufreq_driver = {
	.verify		= tegra_verify_speed,
	.target		= tegra_target,
	.get		= tegra_getspeed,
	.init		= tegra_cpu_init,
	.exit		= tegra_cpu_exit,
	.name		= "tegra",
	.attr		= tegra_cpufreq_attr,
};

static struct {
	char *compat;
	int (*init)(struct tegra_cpufreq_data *,
			const struct tegra_cpufreq_config **);
} tegra_init_funcs[] = {
	{ "nvidia,tegra20", tegra20_cpufreq_init },
	{ "nvidia,tegra124", tegra124_cpufreq_init },
};

static int tegra_cpufreq_probe(struct platform_device *pdev)
{
	int i;
	int ret = -EINVAL;

	tegra_data = devm_kzalloc(&pdev->dev,
			sizeof(*tegra_data), GFP_KERNEL);
	if (!tegra_data) {
		ret = -ENOMEM;
		goto out;
	}

	tegra_data->dev = &pdev->dev;

	for (i = 0; i < ARRAY_SIZE(tegra_init_funcs); i++) {
		if (of_machine_is_compatible(tegra_init_funcs[i].compat)) {
			ret = tegra_init_funcs[i].init(tegra_data, &soc_config);
			if (!ret)
				break;
			else
				goto out;
		}
	}
	if (i == ARRAY_SIZE(tegra_init_funcs))
		goto out;

	ret = cpufreq_register_driver(&tegra_cpufreq_driver);
	if (ret) {
		dev_err(tegra_data->dev,
			"%s: failed to register cpufreq driver\n", __func__);
		goto out;
	}

	return 0;
out:
	return ret;
}

static int tegra_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&tegra_cpufreq_driver);
	return 0;
}

static struct platform_driver tegra_cpufreq_platdrv = {
	.driver = {
		.name	= "tegra-cpufreq",
		.owner	= THIS_MODULE,
	},
	.probe		= tegra_cpufreq_probe,
	.remove		= tegra_cpufreq_remove,
};
module_platform_driver(tegra_cpufreq_platdrv);

int __init tegra_cpufreq_init(void)
{
	struct platform_device_info devinfo = { .name = "tegra-cpufreq", };

	platform_device_register_full(&devinfo);

	return 0;
}
EXPORT_SYMBOL(tegra_cpufreq_init);


MODULE_AUTHOR("Colin Cross <ccross@android.com>");
MODULE_DESCRIPTION("cpufreq driver for Nvidia Tegra2");
MODULE_LICENSE("GPL");
