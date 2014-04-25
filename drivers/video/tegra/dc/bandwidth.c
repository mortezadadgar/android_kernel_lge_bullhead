/*
 * drivers/video/tegra/dc/bandwidth.c
 *
 * Copyright (c) 2010-2013, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Jon Mayo <jmayo@nvidia.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <trace/events/display.h>
#include <linux/platform_data/tegra_emc.h>

#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"

/* Need to set higher EMC than normal usage in case of latency */
#define EMC_BW_USAGE_CUTOFF		2041 /* (100 / 49) * 1000 */
#define EMC_FREQ_CUTOFF_USE_130_PERCENT	100000000
#define EMC_FREQ_CUTOFF_USE_140_PERCENT	50000000

static int use_dynamic_emc = 1;

module_param_named(use_dynamic_emc, use_dynamic_emc, int, S_IRUGO | S_IWUSR);

static int tegra_dc_windows_is_overlapped(struct tegra_dc_win *a,
	struct tegra_dc_win *b)
{
	if (a == b)
		return 0;

	if (!WIN_IS_ENABLED(a) || !WIN_IS_ENABLED(b))
		return 0;

	/* because memory access to load the fifo can overlap, only care
	 * if windows overlap vertically */
	return ((a->out_y + a->out_h > b->out_y) && (a->out_y <= b->out_y)) ||
		((b->out_y + b->out_h > a->out_y) && (b->out_y <= a->out_y));
}

/* check overlapping window combinations to find the max bandwidth. */
static unsigned long tegra_dc_find_max_bandwidth(struct tegra_dc_win *wins[],
						 unsigned n)
{
	unsigned i;
	unsigned j;
	unsigned long bw;
	unsigned long max = 0;

	for (i = 0; i < n; i++) {
		bw = wins[i]->new_bandwidth;
		for (j = 0; j < n; j++)
			if (tegra_dc_windows_is_overlapped(wins[i], wins[j]))
				bw += wins[j]->new_bandwidth;
		if (max < bw)
			max = bw;
	}
	return max;
}

/*
 * Calculate peak EMC bandwidth for each enabled window =
 * pixel_clock * win_bpp * (use_v_filter ? 2 : 1)) * H_scale_factor *
 * (windows_tiling ? 2 : 1)
 *
 * note:
 * (*) We use 2 tap V filter on T2x/T3x, so need double BW if use V filter
 * (*) Tiling mode on T30 and DDR3 requires double BW
 *
 * return:
 * bandwidth in kBps
 */
static unsigned long tegra_dc_calc_win_bandwidth(struct tegra_dc *dc,
	struct tegra_dc_win *w)
{
	unsigned long ret;
	int tiled_windows_bw_multiplier;
	unsigned long bpp;
	unsigned in_w;

	if (!WIN_IS_ENABLED(w))
		return 0;

	if (dfixed_trunc(w->w) == 0 || dfixed_trunc(w->h) == 0 ||
	    w->out_w == 0 || w->out_h == 0)
		return 0;
	if (w->flags & TEGRA_WIN_FLAG_SCAN_COLUMN)
		/* rotated: PRESCALE_SIZE swapped, but WIN_SIZE is unchanged */
		in_w = dfixed_trunc(w->h);
	else
		in_w = dfixed_trunc(w->w); /* normal output, not rotated */

	/* FIXME: Do this when MC driver is ready. */
	tiled_windows_bw_multiplier = 2;

	/* all of tegra's YUV formats(420 and 422) fetch 2 bytes per pixel,
	 * but the size reported by tegra_dc_fmt_bpp for the planar version
	 * is of the luma plane's size only. */
	bpp = tegra_dc_is_yuv_planar(w->fmt) ?
		2 * tegra_dc_fmt_bpp(w->fmt) : tegra_dc_fmt_bpp(w->fmt);
	ret = dc->mode.pclk / 1000UL * bpp / 8 *
		in_w / w->out_w * (WIN_IS_TILED(w) ?
		tiled_windows_bw_multiplier : 1);

	return ret;
}

static unsigned long tegra_dc_get_bandwidth(
	struct tegra_dc_win *windows[], int n)
{
	int i;

	BUG_ON(n > get_dc_n_windows());

	/* emc rate and latency allowance both need to know per window
	 * bandwidths */
	for (i = 0; i < n; i++) {
		struct tegra_dc_win *w = windows[i];

		if (w)
			w->new_bandwidth =
				tegra_dc_calc_win_bandwidth(w->dc, w);
	}

	return tegra_dc_find_max_bandwidth(windows, n);
}

/* to save power, call when display memory clients would be idle */
void tegra_dc_clear_bandwidth(struct tegra_dc *dc)
{
	trace_clear_bandwidth(dc);
	if (__clk_get_enable_count(dc->emc_clk))
		clk_disable_unprepare(dc->emc_clk);
	dc->bw_kbps = 0;
}

/* bw in kByte/second. returns Hz for EMC frequency */
static inline unsigned long tegra_dc_kbps_to_emc(struct tegra_dc *dc,
						 unsigned long bw)
{
	struct clk *emc_master;
	unsigned long freq, old_freq;

	emc_master = clk_get_parent(dc->emc_clk);
	if (bw == ULONG_MAX)
		return clk_round_rate(emc_master, ULONG_MAX);

	freq = tegra_emc_bw_to_freq_req(bw);
	/* freq too big - clamp at max */
	if (freq >= (ULONG_MAX / 1000))
		return clk_round_rate(emc_master, ULONG_MAX);

	/* should never occur because of above */
	if (WARN_ONCE((freq * 1000) < freq, "Bandwidth Overflow"))
		return clk_round_rate(emc_master, ULONG_MAX);

	freq *= 1000;
	freq = clk_round_rate(emc_master, freq);
	/*
	 * Ensure that the normal bw used by the display is no more than 49%
	 * of the total bandwidth set since we will need enough in our
	 * display FIFOs to survive any latency (due to DVFS freq change, etc.)
	 * and we need to fill up the FIFOs before the next latency.
	 */
	while (tegra_emc_freq_req_to_bw(freq) < EMC_BW_USAGE_CUTOFF * bw) {
		old_freq = freq;
		freq = clk_round_rate(emc_master, freq + 1);
		if (old_freq == freq)
			return freq;
	}
	/* Depending on frequency value, the amount of bandwidth usage % of
	 * total we should use is different. Thus we should request a multiple of
	 * original bandwidth on this.  Use 1.4 for < 50MHz, 1.3 for < 100MHz,
	 * else 1.1 */
	if (freq < EMC_FREQ_CUTOFF_USE_140_PERCENT)
		bw += 4 * bw / 10; /* 1.4 */
	else if (freq < EMC_FREQ_CUTOFF_USE_130_PERCENT)
		bw += 3 * bw / 10; /* 1.3 */
	else
		bw += bw / 10; /* 1.1 */
	freq = tegra_emc_bw_to_freq_req(bw);
	freq *= 1000;
	freq = clk_round_rate(emc_master, freq);
	/* Again ensure the bw used is no more than the cutoff */
	while (tegra_emc_freq_req_to_bw(freq) < EMC_BW_USAGE_CUTOFF * bw) {
		freq = clk_round_rate(emc_master, freq + 1);
	}
	return freq;
}

/* use the larger of dc->bw_kbps or dc->new_bw_kbps, and copies
 * dc->new_bw_kbps into dc->bw_kbps.
 * calling this function both before and after a flip is sufficient to select
 * the best possible frequency and latency allowance.
 * set use_new to true to force dc->new_bw_kbps programming.
 */
void tegra_dc_program_bandwidth(struct tegra_dc *dc, bool use_new)
{
	unsigned i;

	if (use_new || dc->bw_kbps != dc->new_bw_kbps) {
		unsigned long bw = max(dc->bw_kbps, dc->new_bw_kbps);
		unsigned long emc_freq;

		/* going from 0 to non-zero */
		if (!dc->bw_kbps && dc->new_bw_kbps &&
			!__clk_get_enable_count(dc->emc_clk))
			clk_prepare_enable(dc->emc_clk);

		emc_freq = tegra_dc_kbps_to_emc(dc, bw);
		clk_set_rate(dc->emc_clk, emc_freq);

		/* going from non-zero to 0 */
		if (dc->bw_kbps && !dc->new_bw_kbps &&
			__clk_get_enable_count(dc->emc_clk))
			clk_disable_unprepare(dc->emc_clk);

		dc->bw_kbps = dc->new_bw_kbps;
	}

	for (i = 0; i < get_dc_n_windows(); i++) {
		struct tegra_dc_win *w = &dc->windows[i];

		/* TODO: Notify MC our new latency allowance. */

		trace_program_bandwidth(dc);
		w->bandwidth = w->new_bandwidth;
	}
}

int tegra_dc_set_dynamic_emc(struct tegra_dc_win *windows[], int n)
{
	unsigned long new_rate;
	struct tegra_dc *dc;

	if (!use_dynamic_emc)
		return 0;

	dc = windows[0]->dc;

	if (tegra_dc_has_multiple_dc())
		new_rate = ULONG_MAX;
	else
		new_rate = tegra_dc_get_bandwidth(windows, n);

	dc->new_bw_kbps = new_rate;
	trace_set_dynamic_emc(dc);

	return 0;
}

/* return the minimum bandwidth in kbps for display to function */
long tegra_dc_calc_min_bandwidth(struct tegra_dc *dc)
{
	unsigned pclk = tegra_dc_get_out_max_pixclock(dc);

	if (WARN_ONCE(!dc, "dc is NULL") ||
		WARN_ONCE(!dc->out, "dc->out is NULL!"))
		return 0;

	if (!pclk && dc->out->type == TEGRA_DC_OUT_HDMI) {
		pclk = tegra_dc_get_out_max_pixclock(dc);
		if (!pclk) {
			if (is_tegra114())
				pclk = 300000000; /* 300MHz max */
			else
				pclk = 150000000; /* 150MHz max */
		}
	} else {
		pclk = dc->mode.pclk;
	}
	return pclk / 1000 * 4; /* support a single 32bpp window */
}
