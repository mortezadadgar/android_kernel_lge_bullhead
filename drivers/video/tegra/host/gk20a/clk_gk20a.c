/*
 * drivers/video/tegra/host/gk20a/clk_gk20a.c
 *
 * GK20A Clocks
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>	/* for mdelay */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/tegra-soc.h>

#include "../dev.h"

#include "clk_pllg.h"
#include "gk20a.h"
#include "gk20a_dvfs.h"
#include "hw_trim_gk20a.h"

#define nvhost_dbg_clk(fmt, arg...) \
	nvhost_dbg(dbg_clk, fmt, ##arg)

static int num_gpu_cooling_freq;
static struct gpufreq_table_data *gpu_cooling_freq;
static struct gk20a *gk20a;

struct gpufreq_table_data *tegra_gpufreq_table_get(void)
{
	return gpu_cooling_freq;
}

unsigned int tegra_gpufreq_table_size_get(void)
{
	return num_gpu_cooling_freq;
}

static u8 pl_to_div[] = {
/* PL:   0, 1, 2, 3, 4, 5, 6,  7,  8,  9, 10, 11, 12, 13, 14 */
/* p: */ 1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 12, 16, 20, 24, 32 };

struct tegra_clk_pllg_params pllg_params = {
	.min_freq = 144, .max_freq = 2064,
	.min_vco = 1000, .max_vco = 2064,
	.min_u = 12, .max_u = 38,
	.min_m = 1, .max_m = 255,
	.min_n = 8, .max_n = 255,
	.min_pl = 1, .max_pl = 32,
};
/* Needed by GK20A PMU module */
EXPORT_SYMBOL(pllg_params);

/**
 * gk20a_init_clk_gpcpll - initialize the internal gpu clock
 *
 * @pdev: instance of gk20a platform device
 *
 * This function should be called during the probe phase
 */
int gk20a_init_clk_gpcpll(struct gk20a *g)
{
	void __iomem *clk_base;
	struct clk *clk, *parent;
	const char *parent_name;
	struct platform_device *pdev = g->dev;
	struct device_node *np = pdev->dev.of_node;

	if (!g->regs) {
		nvhost_err(dev_from_gk20a(g),
				"tegra124 gk20 bar0 not initialized yet");
		return -EPERM;
	}
	clk_base = g->regs;

	/*
	 * The reference clock of gk20a should always be the first item of the
	 * clocks in DT.
	 */
	parent = clk_get(&pdev->dev, NULL);
	if (IS_ERR(parent)) {
		nvhost_err(dev_from_gk20a(g), "pllg parent not initialized");
		return -EPERM;
	}

	parent_name = __clk_get_name(parent);

	clk = tegra_clk_register_pllg("gpcpll", parent_name, clk_base, 0,
					&pllg_params, NULL);

	clk_register_clkdev(clk, "gpcpll", "tegra_gk20a");

	of_clk_add_provider(np, of_clk_src_simple_get, clk);

	gk20a = g;

	return 0;
}

static int gk20a_init_clk_reset_enable_hw(struct gk20a *g)
{
	nvhost_dbg_fn("");
	return 0;
}

struct clk *gk20a_clk_get(struct gk20a *g)
{
	if (!g->clk.tegra_clk) {
		struct clk *clk;

		clk = clk_get_sys("tegra_gk20a", "gpcpll");
		if (IS_ERR(clk)) {
			nvhost_err(dev_from_gk20a(g),
				"fail to get tegra gpu clk tegra_gk20a/gpcpll");
			return NULL;
		}
		g->clk.tegra_clk = clk;
	}

	return g->clk.tegra_clk;
}

static int gk20a_init_clk_setup_sw(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	unsigned long *freqs;
	int err, num_freqs;

	nvhost_dbg_fn("");

	if (clk->sw_ready) {
		nvhost_dbg_fn("skip init");
		return 0;
	}

	if (!gk20a_clk_get(g))
		return -EINVAL;

	err = gk20a_dvfs_get_freqs(g, &freqs, &num_freqs);
	if (!err) {
		int i, j;

		/* init j for inverse traversal of frequencies */
		j = num_freqs - 1;

		gpu_cooling_freq = kzalloc(
				(1 + num_freqs) * sizeof(*gpu_cooling_freq),
				GFP_KERNEL);

		/* store frequencies in inverse order */
		for (i = 0; i < num_freqs; ++i, --j) {
			gpu_cooling_freq[i].index = i;
			gpu_cooling_freq[i].frequency = freqs[j];
		}

		/* add 'end of table' marker */
		gpu_cooling_freq[i].index = i;
		gpu_cooling_freq[i].frequency = GPUFREQ_TABLE_END;

		/* store number of frequencies */
		num_gpu_cooling_freq = num_freqs + 1;
	}

	mutex_init(&clk->clk_mutex);

	clk->sw_ready = true;

	nvhost_dbg_fn("done");
	return 0;
}

static int gk20a_init_clk_setup_hw(struct gk20a *g)
{
	u32 data;

	nvhost_dbg_fn("");

	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	data = set_field(data,
			trim_sys_gpc2clk_out_sdiv14_m() |
			trim_sys_gpc2clk_out_vcodiv_m() |
			trim_sys_gpc2clk_out_bypdiv_m(),
			trim_sys_gpc2clk_out_sdiv14_indiv4_mode_f() |
			trim_sys_gpc2clk_out_vcodiv_by1_f() |
			trim_sys_gpc2clk_out_bypdiv_f(0));
	gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);

	return 0;
}

int gk20a_init_clk_support(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	u32 err;

	nvhost_dbg_fn("");

	clk->g = g;

	err = gk20a_init_clk_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_clk_setup_sw(g);
	if (err)
		return err;

	mutex_lock(&clk->clk_mutex);
	clk->clk_hw_on = true;

	err = gk20a_init_clk_setup_hw(g);
	mutex_unlock(&clk->clk_mutex);
	if (err)
		return err;

	/* FIXME: this effectively prevents host level clock gating */
	err = clk_prepare_enable(g->clk.tegra_clk);
	if (err)
		return err;

	gk20a_writel(g, 0x9080, 0x80000200);

	return err;
}

u32 gk20a_clk_get_rate(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;

	return clk_get_rate(clk->tegra_clk) / MHZ;
}

/**
 * gk20a_clk_round_rate - round rate for gpcpll
 *
 * @g: gk20a instance
 * @rate: target rate in MHz
 */
int gk20a_clk_round_rate(struct gk20a *g, u32 rate)
{
	unsigned long speed_cap;

	/* make sure the clock is available */
	if (!gk20a_clk_get(g))
		return rate;

	speed_cap = g->clk.speed_cap / KHZ;
	if (speed_cap && rate > speed_cap)
		rate = speed_cap;

	return clk_round_rate(g->clk.tegra_clk, rate * MHZ) / MHZ;
}

/**
 * gk20a_clk_set_rate - set rate for gpcpll
 *
 * @g: gk20a instance
 * @rate: target rate in MHz
 */
int gk20a_clk_set_rate(struct gk20a *g, u32 rate)
{
	struct clk_gk20a *clk = &g->clk;
	unsigned long speed_cap;

	speed_cap = clk->speed_cap / KHZ;
	if (speed_cap && rate > speed_cap)
		rate = speed_cap;

	return clk_set_rate(clk->tegra_clk, rate * MHZ);
}

int gk20a_suspend_clk_support(struct gk20a *g)
{
	clk_disable_unprepare(g->clk.tegra_clk);

	return 0;
}

/**
 * tegra_gpu_set_speed_cap - set the speed cap for gk20a
 *
 * @speed_cap: the cap value in KHz
 *
 * If @speed_cap is a zero, that means frequency capping is disabled.
 */
int tegra_gpu_set_speed_cap(unsigned long *speed_cap)
{
	struct clk_gk20a *clk;

	if (!gk20a) {
		WARN(1, "failed to find gpu instance\n");
		return -EPERM;
	}

	if (!speed_cap)
		return -EINVAL;

	clk = &gk20a->clk;

	if (*speed_cap)
		*speed_cap = clk_round_rate(clk->tegra_clk,
					    *speed_cap * KHZ) / KHZ;

	mutex_lock(&clk->clk_mutex);
	clk->speed_cap = *speed_cap;
	mutex_unlock(&clk->clk_mutex);

	if (*speed_cap)
		return gk20a_clk_set_rate(gk20a, *speed_cap * KHZ);
	else
		return 0;
}

#ifdef CONFIG_DEBUG_FS

static int rate_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	*val = (u64)gk20a_clk_get_rate(g);
	return 0;
}
static int rate_set(void *data, u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	return gk20a_clk_set_rate(g, (u32)val);
}
DEFINE_SIMPLE_ATTRIBUTE(rate_fops, rate_get, rate_set, "%llu\n");

static int cap_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	*val = g->clk.speed_cap / KHZ;
	return 0;
}
static int cap_set(void *data, u64 val)
{
	unsigned long rate = val * KHZ;
	return tegra_gpu_set_speed_cap(&rate);
}
DEFINE_SIMPLE_ATTRIBUTE(cap_fops, cap_get, cap_set, "%llu\n");

static int pll_reg_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	u32 reg, m, n, pl, f;

	mutex_lock(&g->clk.clk_mutex);
	if (!g->clk.clk_hw_on) {
		seq_printf(s, "gk20a powered down - no access to registers\n");
		mutex_unlock(&g->clk.clk_mutex);
		return 0;
	}

	reg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	seq_printf(s, "cfg  = 0x%x : %s : %s\n", reg,
		   trim_sys_gpcpll_cfg_enable_v(reg) ? "enabled" : "disabled",
		   trim_sys_gpcpll_cfg_pll_lock_v(reg) ? "locked" : "unlocked");

	reg = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	m = trim_sys_gpcpll_coeff_mdiv_v(reg);
	n = trim_sys_gpcpll_coeff_ndiv_v(reg);
	pl = trim_sys_gpcpll_coeff_pldiv_v(reg);
	f = g->clk.gpc_pll.clk_in * n / (m * pl_to_div[pl]);
	seq_printf(s, "coef = 0x%x : m = %u : n = %u : pl = %u", reg, m, n, pl);
	seq_printf(s, " : pll_f(gpu_f) = %u(%u) MHz\n", f, f/2);
	mutex_unlock(&g->clk.clk_mutex);
	return 0;
}

static int pll_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_reg_show, inode->i_private);
}

static const struct file_operations pll_reg_fops = {
	.open		= pll_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int monitor_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	struct clk_gk20a *clk = &g->clk;

	u32 ncycle = 100; /* count GPCCLK for ncycle of clkin */
	u32 clkin = clk->gpc_pll.clk_in;
	u32 count1, count2;

	if (!__clk_is_enabled(clk->tegra_clk)) {
		*val = 0;
		return 0;
	}

	gk20a_writel(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0),
		     trim_gpc_clk_cntr_ncgpcclk_cfg_reset_asserted_f());
	gk20a_writel(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0),
		     trim_gpc_clk_cntr_ncgpcclk_cfg_enable_asserted_f() |
		     trim_gpc_clk_cntr_ncgpcclk_cfg_write_en_asserted_f() |
		     trim_gpc_clk_cntr_ncgpcclk_cfg_noofipclks_f(ncycle));
	/* start */

	/* It should take about 8us to finish 100 cycle of 12MHz.
	   But longer than 100us delay is required here. */
	gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0));
	udelay(2000);

	count1 = gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cnt_r(0));
	udelay(100);
	count2 = gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cnt_r(0));
	*val = (u64)(trim_gpc_clk_cntr_ncgpcclk_cnt_value_v(count2) *
		     clkin / ncycle);

	if (count1 != count2)
		return -EBUSY;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(monitor_fops, monitor_get, NULL, "%llu\n");

int clk_gk20a_debugfs_init(struct platform_device *dev)
{
	struct dentry *d;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct gk20a *g = get_gk20a(dev);

	d = debugfs_create_file(
		"rate", S_IRUGO|S_IWUSR, pdata->debugfs, g, &rate_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"cap", S_IRUGO|S_IWUSR, pdata->debugfs, g, &cap_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"pll_reg", S_IRUGO, pdata->debugfs, g, &pll_reg_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"monitor", S_IRUGO, pdata->debugfs, g, &monitor_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	pr_err("%s: Failed to make debugfs node\n", __func__);
	debugfs_remove_recursive(pdata->debugfs);
	return -ENOMEM;
}

#endif /* CONFIG_DEBUG_FS */
