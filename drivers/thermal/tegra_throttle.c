/*
 * tegra_throttle.c
 *
 * Copyright (c) 2012, 2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/tegra-soc.h>
#include <linux/platform_data/tegra_emc.h>
#include <linux/platform_device.h>

#include "tegra_throttle.h"

static DEFINE_MUTEX(bthrot_list_lock);
static LIST_HEAD(bthrot_list);

static struct tegra_balanced_throttle *tegra_b_throt;
static struct tegra_throttle_cap_data *tegra_cap_freqs_table;
static int tegra_cap_freqs_table_size;

static unsigned long clip_to_table(unsigned long cpu_freq)
{
	int i;

	if (!tegra_b_throt->cpu_freq_table)
		return -EINVAL;

	for (i = 0;
	     tegra_b_throt->cpu_freq_table[i].frequency != CPUFREQ_TABLE_END;
	     i++) {
		if (tegra_b_throt->cpu_freq_table[i].frequency > cpu_freq)
			break;
	}
	i = (i == 0) ? 0 : i-1;

	return tegra_b_throt->cpu_freq_table[i].frequency;
}

static int
tegra_throttle_get_max_state(struct thermal_cooling_device *cdev,
			     unsigned long *max_state)
{
	struct balanced_throttle_instance *bthrot = cdev->devdata;

	*max_state = bthrot->throt_tab_size;

	return 0;
}

static int
tegra_throttle_get_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long *cur_state)
{
	struct balanced_throttle_instance *bthrot = cdev->devdata;

	*cur_state = bthrot->cur_state;

	return 0;
}

static void tegra_throttle_set_cap_clk(struct throttle_table *throt_tab,
					int cap_clk_index)
{
	unsigned long cap_rate, clk_rate;
	int cap_offset = cap_clk_index - CAP_CLK_START;

	cap_rate = throt_tab->cap_freqs[cap_clk_index];
	if (cap_rate == NO_CAP)
		clk_rate = tegra_cap_freqs_table[cap_offset].max_freq;
	else
		clk_rate = cap_rate * 1000UL;

	if (tegra_cap_freqs_table[cap_offset].cap_freq != clk_rate) {
		clk_set_rate(tegra_cap_freqs_table[cap_offset].cap_clk,
			     clk_rate);
		tegra_cap_freqs_table[cap_offset].cap_freq = clk_rate;
	}
}

static void
tegra_throttle_cap_freqs_update(struct throttle_table *throt_tab,
				int direction)
{
	int i;
	int max_cap_clock = CAP_CLK_START + tegra_cap_freqs_table_size;

	if (direction == 1) {
		/* performance up : throttle less */
		for (i = max_cap_clock - 1; i >= CAP_CLK_START; i--)
			tegra_throttle_set_cap_clk(throt_tab, i);
	} else {
		/* performance down : throotle more */
		for (i = CAP_CLK_START; i < max_cap_clock; i++)
			tegra_throttle_set_cap_clk(throt_tab, i);
	}
}

static int
tegra_throttle_set_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long cur_state)
{
	struct balanced_throttle_instance *bthrot = cdev->devdata;
	int direction;
	int i;
	int max_cap_clock = CAP_CLK_START + tegra_cap_freqs_table_size;
	unsigned long bthrot_speed;
	struct throttle_table *throt_entry;
	struct throttle_table cur_throt_freq = {
		{ NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP}
	};

	if (tegra_b_throt->cpu_freq_table == NULL)
		return 0;

	if (bthrot->cur_state == cur_state)
		return 0;

	direction = bthrot->cur_state >= cur_state;
	bthrot->cur_state = cur_state;

	if (cur_state == 1 && direction == 0)
		bthrot->throttle_count++;

	mutex_lock(&bthrot_list_lock);
	list_for_each_entry(bthrot, &bthrot_list, node) {
		if (!bthrot->cur_state)
			continue;

		throt_entry = &bthrot->throt_tab[bthrot->cur_state-1];
		for (i = 0; i < max_cap_clock; i++) {
			cur_throt_freq.cap_freqs[i] = min(
					cur_throt_freq.cap_freqs[i],
					throt_entry->cap_freqs[i]);
		}
	}

	tegra_throttle_cap_freqs_update(&cur_throt_freq, direction);

	bthrot_speed = clip_to_table(cur_throt_freq.cap_freqs[CAP_CPU]);
	tegra_b_throt->bthrot_speed = bthrot_speed;
	cpufreq_update_policy(0);

	bthrot_speed = cur_throt_freq.cap_freqs[CAP_GPU];
	if (bthrot_speed != NO_CAP)
		tegra_gpu_set_speed_cap(&bthrot_speed);

	mutex_unlock(&bthrot_list_lock);

	return 0;
}

static struct thermal_cooling_device_ops tegra_throttle_cooling_ops = {
	.get_max_state = tegra_throttle_get_max_state,
	.get_cur_state = tegra_throttle_get_cur_state,
	.set_cur_state = tegra_throttle_set_cur_state,
};

/**
 * tegra_throttle_cpufreq_policy_notifier - Notifier callback for cpufreq policy
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to highjack the notification on cpufreq policy transition.
 * Every time there is a change in policy, we will intercept and
 * update the cpufreq policy with thermal throttling constraints.
 *
 * Return: 0 (success)
 */
static int tegra_throttle_cpufreq_policy_notifier(struct notifier_block *nb,
						  unsigned long event,
						  void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	if (tegra_b_throt->bthrot_speed == NO_CAP)
		return 0;

	/* Limit max freq to be within tegra_throttle limit. */

	if (policy->max != tegra_b_throt->bthrot_speed)
		cpufreq_verify_within_limits(policy, 0,
					     tegra_b_throt->bthrot_speed);

	return 0;
}

/* Notifier for cpufreq policy change */
static struct notifier_block tegra_throttle_cpufreq_notifier_block = {
	.notifier_call = tegra_throttle_cpufreq_policy_notifier,
};

#ifdef CONFIG_DEBUG_FS
static int table_show(struct seq_file *s, void *data)
{
	struct balanced_throttle_instance *bthrot = s->private;
	int max_cap_clock = CAP_CLK_START + tegra_cap_freqs_table_size;
	int i, j;

	for (i = 0; i < bthrot->throt_tab_size; i++) {
		/* CPU FREQ */
		seq_printf(s, "[%d] = %7lu",
			   i, bthrot->throt_tab[i].cap_freqs[CAP_CPU]);

		/* GPU FREQ and other DVFS module FREQS */
		for (j = CAP_GPU; j < max_cap_clock; j++)
			if (bthrot->throt_tab[i].cap_freqs[j] == NO_CAP)
				seq_printf(s, " %7s", "NO_CAP");
			else
				seq_printf(s, " %7lu",
					   bthrot->throt_tab[i].cap_freqs[j]);
		seq_puts(s, "\n");
	}

	return 0;
}

static int table_open(struct inode *inode, struct file *file)
{
	return single_open(file, table_show, inode->i_private);
}

static const struct file_operations table_fops = {
	.open		= table_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *throttle_debugfs_root;
#endif /* CONFIG_DEBUG_FS */


struct thermal_cooling_device *balanced_throttle_register(
		struct balanced_throttle_instance *bthrot,
		char *type)
{
#ifdef CONFIG_DEBUG_FS
	char name[32];
#endif
	mutex_lock(&bthrot_list_lock);
	tegra_b_throt->num_throt++;
	list_add(&bthrot->node, &bthrot_list);
	mutex_unlock(&bthrot_list_lock);

	bthrot->cdev = thermal_of_cooling_device_register(
						bthrot->np,
						type,
						bthrot,
						&tegra_throttle_cooling_ops);

	if (IS_ERR(bthrot->cdev)) {
		bthrot->cdev = NULL;
		return ERR_PTR(-ENODEV);
	}

#ifdef CONFIG_DEBUG_FS
	sprintf(name, "throttle_table%d", tegra_b_throt->num_throt);
	bthrot->d_tab = debugfs_create_file(name, 0644, throttle_debugfs_root,
					   bthrot, &table_fops);
#endif

	return bthrot->cdev;
}
EXPORT_SYMBOL(balanced_throttle_register);

void balanced_throttle_unregister(struct balanced_throttle_instance *bthrot)
{
	thermal_cooling_device_unregister(bthrot->cdev);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(bthrot->d_tab);
#endif

	mutex_lock(&bthrot_list_lock);
	tegra_b_throt->num_throt--;
	list_del(&bthrot->node);
	mutex_unlock(&bthrot_list_lock);
}
EXPORT_SYMBOL(balanced_throttle_unregister);

int tegra_throttle_init(struct platform_device *pdev,
			struct tegra_balanced_throttle *b_throt,
			struct tegra_throttle_cap_data *cap_freqs_table,
			int cap_freqs_table_size)
{
	int i;

	b_throt->cpu_freq_table = cpufreq_frequency_get_table(0);
	if (!b_throt->cpu_freq_table) {
		dev_warn(&pdev->dev,
			 "tegra_throttle: cannot get cpufreq table data\n");
		return -EPROBE_DEFER;
	}
	b_throt->cpu_throttle_lowest_speed =
			b_throt->cpu_freq_table[0].frequency;
	b_throt->bthrot_speed = NO_CAP;

#ifdef CONFIG_DEBUG_FS
	throttle_debugfs_root = debugfs_create_dir("tegra_throttle", NULL);
#endif

	for (i = 0; i < cap_freqs_table_size; i++) {
		struct clk *c;
		dev_info(&pdev->dev,
			 "setting clock %s\n", cap_freqs_table[i].cap_name);
		c = devm_clk_get(&pdev->dev, cap_freqs_table[i].cap_name);
		if (IS_ERR(c)) {
			dev_warn(&pdev->dev,
				 "tegra_throttle: cannot get clock %s\n",
				 cap_freqs_table[i].cap_name);
			return -EPROBE_DEFER;
		}

		cap_freqs_table[i].cap_clk = c;
		cap_freqs_table[i].max_freq = clk_round_rate(c, ULONG_MAX);
		cap_freqs_table[i].cap_freq = cap_freqs_table[i].max_freq;
	}

	tegra_b_throt = b_throt;
	tegra_cap_freqs_table = cap_freqs_table;
	tegra_cap_freqs_table_size = cap_freqs_table_size;

	cpufreq_register_notifier(&tegra_throttle_cpufreq_notifier_block,
				  CPUFREQ_POLICY_NOTIFIER);

	dev_info(&pdev->dev, "tegra_throttle : init done\n");

	return 0;
}
EXPORT_SYMBOL(tegra_throttle_init);

void tegra_throttle_deinit(void)
{
	cpufreq_unregister_notifier(&tegra_throttle_cpufreq_notifier_block,
				    CPUFREQ_POLICY_NOTIFIER);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(throttle_debugfs_root);
#endif
}
EXPORT_SYMBOL(tegra_throttle_deinit);
