/*
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/tegra-powergate.h>

#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>

#include "dc.h"
#include "drm.h"
#include "sor.h"

struct tegra_sor {
	struct host1x_client client;
	struct tegra_output output;
	struct device *dev;

	void __iomem *regs;

	struct reset_control *rst;
	struct clk *clk_parent;
	struct clk *clk_safe;
	struct clk *clk_dp;
	struct clk *clk;

	struct tegra_dpaux *dpaux;
	struct drm_dp_link link;

	struct mutex lock;
	bool enabled;

	struct drm_minor *minor;
	struct drm_info_list *debugfs_files;
	struct dentry *debugfs;
};

struct tegra_sor_config {
	u32 bits_per_pixel;

	u32 active_polarity;
	u32 active_count;
	u32 tu_size;
	u32 active_frac;
	u32 watermark;

	u32 hblank_symbols;
	u32 vblank_symbols;
};

/* Tegra124 and Tegra132 have lanes 0 and 2 swapped. */
static const u8 tegra124_sor_lane_map[4] = {
	2, 1, 0, 3,
};

static const u8 tegra124_sor_xbar_cfg[5] = {
	0, 1, 2, 3, 4
};

static const u8 tegra124_sor_voltage_swing[4][4][4] = {
	{
		{ 0x13, 0x19, 0x1e, 0x28 },
		{ 0x1e, 0x25, 0x2d, },
		{ 0x28, 0x32, },
		{ 0x3c, },
	}, {
		{ 0x12, 0x17, 0x1b, 0x25 },
		{ 0x1c, 0x23, 0x2a, },
		{ 0x25, 0x2f, },
		{ 0x39, }
	}, {
		{ 0x12, 0x16, 0x1a, 0x22 },
		{ 0x1b, 0x20, 0x27, },
		{ 0x24, 0x2d, },
		{ 0x36, },
	}, {
		{ 0x11, 0x14, 0x17, 0x1f },
		{ 0x19, 0x1e, 0x24, },
		{ 0x22, 0x2a, },
		{ 0x32, },
	},
};

static const u8 tegra124_sor_pre_emphasis[4][4][4] = {
	{
		{ 0x00, 0x09, 0x13, 0x25 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00, },
	}, {
		{ 0x00, 0x0a, 0x14, 0x28 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00 },
	}, {
		{ 0x00, 0x0a, 0x14, 0x28 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00, },
	}, {
		{ 0x00, 0x0a, 0x14, 0x28 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00, },
	},
};

static const u8 tegra124_sor_post_cursor[4][4][4] = {
	{
		{ 0x00, 0x00, 0x00, 0x00 },
		{ 0x00, 0x00, 0x00, },
		{ 0x00, 0x00, },
		{ 0x00, },
	}, {
		{ 0x02, 0x02, 0x04, 0x05 },
		{ 0x02, 0x04, 0x05, },
		{ 0x04, 0x05, },
		{ 0x05, },
	}, {
		{ 0x04, 0x05, 0x08, 0x0b },
		{ 0x05, 0x09, 0x0b, },
		{ 0x08, 0x0a, },
		{ 0x0b, },
	}, {
		{ 0x05, 0x09, 0x0b, 0x12 },
		{ 0x09, 0x0d, 0x12, },
		{ 0x0b, 0x0f, },
		{ 0x12, },
	},
};

static const u8 tegra124_sor_tx_pu[4][4][4] = {
	{
		{ 0x20, 0x30, 0x40, 0x60 },
		{ 0x30, 0x40, 0x60, },
		{ 0x40, 0x60, },
		{ 0x60, },
	}, {
		{ 0x20, 0x20, 0x30, 0x50 },
		{ 0x30, 0x40, 0x50, },
		{ 0x40, 0x50, },
		{ 0x60, },
	}, {
		{ 0x20, 0x20, 0x30, 0x40, },
		{ 0x30, 0x30, 0x40, },
		{ 0x40, 0x50, },
		{ 0x60, },
	}, {
		{ 0x20, 0x20, 0x20, 0x40, },
		{ 0x30, 0x30, 0x40, },
		{ 0x40, 0x40, },
		{ 0x60, },
	},
};

static inline struct tegra_sor *
host1x_client_to_sor(struct host1x_client *client)
{
	return container_of(client, struct tegra_sor, client);
}

static inline struct tegra_sor *to_sor(struct tegra_output *output)
{
	return container_of(output, struct tegra_sor, output);
}

static inline unsigned long tegra_sor_readl(struct tegra_sor *sor,
					    unsigned long offset)
{
	return readl(sor->regs + (offset << 2));
}

static inline void tegra_sor_writel(struct tegra_sor *sor, unsigned long value,
				    unsigned long offset)
{
	writel(value, sor->regs + (offset << 2));
}

static void tegra_sor_super_update(struct tegra_sor *sor)
{
	tegra_sor_writel(sor, 0, SOR_SUPER_STATE_0);
	tegra_sor_writel(sor, 1, SOR_SUPER_STATE_0);
	tegra_sor_writel(sor, 0, SOR_SUPER_STATE_0);
}

static void tegra_sor_update(struct tegra_sor *sor)
{
	tegra_sor_writel(sor, 0, SOR_STATE_0);
	tegra_sor_writel(sor, 1, SOR_STATE_0);
	tegra_sor_writel(sor, 0, SOR_STATE_0);
}

static int tegra_sor_setup_pwm(struct tegra_sor *sor, unsigned long timeout)
{
	unsigned long value;

	value = tegra_sor_readl(sor, SOR_PWM_DIV);
	value &= ~SOR_PWM_DIV_MASK;
	value |= 0x400; /* period */
	tegra_sor_writel(sor, value, SOR_PWM_DIV);

	value = tegra_sor_readl(sor, SOR_PWM_CTL);
	value &= ~SOR_PWM_CTL_DUTY_CYCLE_MASK;
	value |= 0x400; /* duty cycle */
	value &= ~SOR_PWM_CTL_CLK_SEL; /* clock source: PCLK */
	value |= SOR_PWM_CTL_TRIGGER;
	tegra_sor_writel(sor, value, SOR_PWM_CTL);

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_PWM_CTL);
		if ((value & SOR_PWM_CTL_TRIGGER) == 0)
			return 0;

		usleep_range(25, 100);
	}

	return -ETIMEDOUT;
}

static int tegra_sor_attach(struct tegra_sor *sor)
{
	unsigned long value, timeout;

	/* wake up in normal mode */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE_1);
	value |= SOR_SUPER_STATE_HEAD_MODE_AWAKE;
	value |= SOR_SUPER_STATE_MODE_NORMAL;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE_1);
	tegra_sor_super_update(sor);

	/* attach */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE_1);
	value |= SOR_SUPER_STATE_ATTACHED;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE_1);
	tegra_sor_super_update(sor);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_TEST);
		if ((value & SOR_TEST_ATTACHED) != 0)
			return 0;

		usleep_range(25, 100);
	}

	return -ETIMEDOUT;
}

static int tegra_sor_wakeup(struct tegra_sor *sor)
{
	struct tegra_dc *dc = to_tegra_dc(sor->output.encoder.crtc);
	unsigned long value, timeout;

	/* enable display controller outputs */
	value = tegra_dc_readl(dc, DC_CMD_DISPLAY_POWER_CONTROL);
	value |= PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
		 PW4_ENABLE | PM0_ENABLE | PM1_ENABLE;
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	timeout = jiffies + msecs_to_jiffies(250);

	/* wait for head to wake up */
	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_TEST);
		value &= SOR_TEST_HEAD_MODE_MASK;

		if (value == SOR_TEST_HEAD_MODE_AWAKE)
			return 0;

		usleep_range(25, 100);
	}

	return -ETIMEDOUT;
}

static int tegra_sor_power_up(struct tegra_sor *sor, unsigned long timeout)
{
	unsigned long value;

	value = tegra_sor_readl(sor, SOR_PWR);
	value |= SOR_PWR_TRIGGER | SOR_PWR_NORMAL_STATE_PU;
	tegra_sor_writel(sor, value, SOR_PWR);

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_PWR);
		if ((value & SOR_PWR_TRIGGER) == 0)
			return 0;

		usleep_range(25, 100);
	}

	return -ETIMEDOUT;
}

struct tegra_sor_params {
	/* number of link clocks per line */
	unsigned int num_clocks;
	/* ratio between input and output */
	u64 ratio;
	/* precision factor */
	u64 precision;

	unsigned int active_polarity;
	unsigned int active_count;
	unsigned int active_frac;
	unsigned int tu_size;
	unsigned int error;
};

static int tegra_sor_compute_params(struct tegra_sor *sor,
				    struct tegra_sor_params *params,
				    unsigned int tu_size)
{
	u64 active_sym, active_count, frac, approx;
	u32 active_polarity, active_frac = 0;
	const u64 f = params->precision;
	s64 error;

	active_sym = params->ratio * tu_size;
	active_count = div_u64(active_sym, f) * f;
	frac = active_sym - active_count;

	/* fraction < 0.5 */
	if (frac >= (f / 2)) {
		active_polarity = 1;
		frac = f - frac;
	} else {
		active_polarity = 0;
	}

	if (frac != 0) {
		frac = div_u64(f * f,  frac); /* 1/fraction */
		if (frac <= (15 * f)) {
			active_frac = div_u64(frac, f);

			/* round up */
			if (active_polarity)
				active_frac++;
		} else {
			active_frac = active_polarity ? 1 : 15;
		}
	}

	if (active_frac == 1)
		active_polarity = 0;

	if (active_polarity == 1) {
		if (active_frac) {
			approx = active_count + (active_frac * (f - 1)) * f;
			approx = div_u64(approx, active_frac * f);
		} else {
			approx = active_count + f;
		}
	} else {
		if (active_frac)
			approx = active_count + div_u64(f, active_frac);
		else
			approx = active_count;
	}

	error = div_s64(active_sym - approx, tu_size);
	error *= params->num_clocks;

	if (error <= 0 && abs64(error) < params->error) {
		params->active_count = div_u64(active_count, f);
		params->active_polarity = active_polarity;
		params->active_frac = active_frac;
		params->error = abs64(error);
		params->tu_size = tu_size;

		if (error == 0)
			return true;
	}

	return false;
}

static int tegra_sor_calc_config(struct tegra_sor *sor,
				 struct drm_display_mode *mode,
				 struct tegra_sor_config *config,
				 struct drm_dp_link *link)
{
	const u64 f = 100000, link_rate = link->rate * 1000;
	const u64 pclk = mode->clock * 1000;
	u64 input, output, watermark, num;
	struct tegra_sor_params params;
	u32 num_syms_per_line;
	unsigned int i;

	if (!link_rate || !link->num_lanes || !pclk || !config->bits_per_pixel)
		return -EINVAL;

	input = pclk * config->bits_per_pixel;
	output = link_rate * 8 * link->num_lanes;

	if (input >= output)
		return -ERANGE;

	memset(&params, 0, sizeof(params));
	params.ratio = div64_u64(input * f, output);
	params.num_clocks = div_u64(link_rate * mode->hdisplay, pclk);
	params.precision = f;
	params.error = 64 * f;
	params.tu_size = 64;

	for (i = params.tu_size; i >= 32; i--)
		if (tegra_sor_compute_params(sor, &params, i))
			break;

	if (params.active_frac == 0) {
		config->active_polarity = 0;
		config->active_count = params.active_count;

		if (!params.active_polarity)
			config->active_count--;

		config->tu_size = params.tu_size;
		config->active_frac = 1;
	} else {
		config->active_polarity = params.active_polarity;
		config->active_count = params.active_count;
		config->active_frac = params.active_frac;
		config->tu_size = params.tu_size;
	}

	dev_dbg(sor->dev,
		"polarity: %d active count: %d tu size: %d active frac: %d\n",
		config->active_polarity, config->active_count,
		config->tu_size, config->active_frac);

	watermark = params.ratio * config->tu_size * (f - params.ratio);
	watermark = div_u64(watermark, f);

	watermark = div_u64(watermark + params.error, f);
	config->watermark = watermark + (config->bits_per_pixel / 8) + 2;
	num_syms_per_line = (mode->hdisplay * config->bits_per_pixel) *
			    (link->num_lanes * 8);

	if (config->watermark > 30) {
		config->watermark = 30;
		dev_err(sor->dev,
			"unable to compute TU size, forcing watermark to %u\n",
			config->watermark);
	} else if (config->watermark > num_syms_per_line) {
		config->watermark = num_syms_per_line;
		dev_err(sor->dev, "watermark too high, forcing to %u\n",
			config->watermark);
	}

	/* compute the number of symbols per horizontal blanking interval */
	num = ((mode->htotal - mode->hdisplay) - 7) * link_rate;
	config->hblank_symbols = div_u64(num, pclk);

	if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		config->hblank_symbols -= 3;

	config->hblank_symbols -= 12 / link->num_lanes;

	/* compute the number of symbols per vertical blanking interval */
	num = (mode->hdisplay - 25) * link_rate;
	config->vblank_symbols = div_u64(num, pclk);
	config->vblank_symbols -= 36 / link->num_lanes + 4;

	dev_dbg(sor->dev, "blank symbols: H:%u V:%u\n", config->hblank_symbols,
		config->vblank_symbols);

	return 0;
}

static int tegra_sor_power_up_lanes(struct tegra_sor *sor, unsigned int lanes)
{
	unsigned long timeout;
	u32 value;

	/*
	 * Clear or set the PD_TXD bit corresponding to each lane, depending
	 * on whether it is used or not.
	 */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);

	if (sor->link.num_lanes <= 2)
		value &= ~(SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_2);
	else
		value |= SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_2;

	if (sor->link.num_lanes <= 1)
		value &= ~SOR_DP_PADCTL_PD_TXD_1;
	else
		value |= SOR_DP_PADCTL_PD_TXD_1;

	if (sor->link.num_lanes == 0)
		value &= ~SOR_DP_PADCTL_PD_TXD_0;
	else
		value |= SOR_DP_PADCTL_PD_TXD_0;

	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	/* start lane sequencer */
	value = SOR_LANE_SEQ_CTL_TRIGGER | SOR_LANE_SEQ_CTL_SEQUENCE_DOWN |
		SOR_LANE_SEQ_CTL_POWER_STATE_UP;
	tegra_sor_writel(sor, value, SOR_LANE_SEQ_CTL);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_TRIGGER) == 0)
			break;

		usleep_range(250, 1000);
	}

	if ((value & SOR_LANE_SEQ_CTL_TRIGGER) != 0)
		return -ETIMEDOUT;

	return 0;
}

static int tegra_sor_power_down_lanes(struct tegra_sor *sor)
{
	unsigned long timeout;
	u32 value;

	/* power down all lanes */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value &= ~(SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_0 |
		   SOR_DP_PADCTL_PD_TXD_1 | SOR_DP_PADCTL_PD_TXD_2);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	/* start lane sequencer */
	value = SOR_LANE_SEQ_CTL_TRIGGER | SOR_LANE_SEQ_CTL_SEQUENCE_UP |
		SOR_LANE_SEQ_CTL_POWER_STATE_DOWN;
	tegra_sor_writel(sor, value, SOR_LANE_SEQ_CTL);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_TRIGGER) == 0)
			break;

		usleep_range(25, 100);
	}

	if ((value & SOR_LANE_SEQ_CTL_TRIGGER) != 0)
		return -ETIMEDOUT;

	return 0;
}

static void tegra_sor_dp_precharge(struct tegra_sor *sor, unsigned int lanes)
{
	u32 value;

	/* pre-charge all used lanes */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);

	if (sor->link.num_lanes <= 2)
		value &= ~(SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_2);
	else
		value |= SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_2;

	if (sor->link.num_lanes <= 1)
		value &= ~SOR_DP_PADCTL_PD_TXD_1;
	else
		value |= SOR_DP_PADCTL_PD_TXD_1;

	if (sor->link.num_lanes == 0)
		value &= ~SOR_DP_PADCTL_PD_TXD_0;
	else
		value |= SOR_DP_PADCTL_PD_TXD_0;

	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	usleep_range(15, 100);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value &= ~(SOR_DP_PADCTL_CM_TXD_3 | SOR_DP_PADCTL_CM_TXD_2 |
		   SOR_DP_PADCTL_CM_TXD_1 | SOR_DP_PADCTL_CM_TXD_0);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);
}

static void tegra_sor_dp_term_calibrate(struct tegra_sor *sor)
{
	u32 mask = 0x08, adj = 0, value;

	/* enable pad calibration logic */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value &= ~SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	value = tegra_sor_readl(sor, SOR_PLL_1);
	value |= SOR_PLL_1_TMDS_TERM;
	tegra_sor_writel(sor, value, SOR_PLL_1);

	while (mask) {
		adj |= mask;

		value = tegra_sor_readl(sor, SOR_PLL_1);
		value &= ~SOR_PLL_1_TMDS_TERMADJ_MASK;
		value |= SOR_PLL_1_TMDS_TERMADJ(adj);
		tegra_sor_writel(sor, value, SOR_PLL_1);

		usleep_range(100, 200);

		value = tegra_sor_readl(sor, SOR_PLL_1);
		if (value & SOR_PLL_1_TERM_COMPOUT)
			adj &= ~mask;

		mask >>= 1;
	}

	value = tegra_sor_readl(sor, SOR_PLL_1);
	value &= ~SOR_PLL_1_TMDS_TERMADJ_MASK;
	value |= SOR_PLL_1_TMDS_TERMADJ(adj);
	tegra_sor_writel(sor, value, SOR_PLL_1);

	/* disable pad calibration logic */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value |= SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);
}

static int tegra_sor_dp_link_apply_training(struct drm_dp_link *link)
{
	struct tegra_sor *sor = container_of(link, struct tegra_sor, link);
	u32 voltage_swing = 0, pre_emphasis = 0, post_cursor = 0;
	u32 pattern = 0, tx_pu = 0, value;
	unsigned int i;

	for (value = 0, i = 0; i < link->num_lanes; i++) {
		u8 vs = link->train.request.voltage_swing[i];
		u8 pe = link->train.request.pre_emphasis[i];
		u8 pc = link->train.request.post_cursor[i];
		u8 shift = tegra124_sor_lane_map[i] << 3;

		voltage_swing |= tegra124_sor_voltage_swing[pc][vs][pe] << shift;
		pre_emphasis |= tegra124_sor_pre_emphasis[pc][vs][pe] << shift;
		post_cursor |= tegra124_sor_post_cursor[pc][vs][pe] << shift;

		if (tegra124_sor_tx_pu[pc][vs][pe] > tx_pu)
			tx_pu = tegra124_sor_tx_pu[pc][vs][pe];

		switch (link->train.pattern) {
		case DP_TRAINING_PATTERN_DISABLE:
			value = SOR_DP_TPG_SCRAMBLER_GALIOS |
				SOR_DP_TPG_PATTERN_NONE;
			break;

		case DP_TRAINING_PATTERN_1:
			value = SOR_DP_TPG_SCRAMBLER_NONE |
				SOR_DP_TPG_PATTERN_TRAIN1;
			break;

		case DP_TRAINING_PATTERN_2:
			value = SOR_DP_TPG_SCRAMBLER_NONE |
				SOR_DP_TPG_PATTERN_TRAIN2;
			break;

		case DP_TRAINING_PATTERN_3:
			value = SOR_DP_TPG_SCRAMBLER_NONE |
				SOR_DP_TPG_PATTERN_TRAIN3;
			break;

		default:
			return -EINVAL;
		}

		if (link->capabilities & DP_LINK_CAP_ANSI_8B10B)
			value |= SOR_DP_TPG_CHANNEL_CODING;
		pattern = pattern << 8 | value;
	}

	tegra_sor_writel(sor, voltage_swing, SOR_LANE_DRIVE_CURRENT_0);
	tegra_sor_writel(sor, pre_emphasis, SOR_LANE_PREEMPHASIS_0);

	if (link->capabilities & DP_LINK_CAP_TPS3)
		tegra_sor_writel(sor, post_cursor, SOR_LANE_POST_CURSOR_0);

	tegra_sor_writel(sor, pattern, SOR_DP_TPG);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value &= ~SOR_DP_PADCTL_TX_PU_MASK;
	value |= SOR_DP_PADCTL_TX_PU_ENABLE;
	value |= SOR_DP_PADCTL_TX_PU(tx_pu);

	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	usleep_range(20, 100);

	return 0;
}

static int tegra_sor_dp_link_configure(struct drm_dp_link *link)
{
	struct tegra_sor *sor = container_of(link, struct tegra_sor, link);
	u8 rate, lanes;
	u32 value;
	int err;

	rate = drm_dp_link_rate_to_bw_code(link->rate);
	lanes = link->num_lanes;

	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_LINK_SPEED_MASK;
	value |= SOR_CLK_CNTRL_DP_LINK_SPEED(rate);
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	value = tegra_sor_readl(sor, SOR_DP_LINKCTL_0);
	value &= ~SOR_DP_LINKCTL_LANE_COUNT_MASK;
	value |= SOR_DP_LINKCTL_LANE_COUNT(lanes);

	if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		value |= SOR_DP_LINKCTL_ENHANCED_FRAME;

	tegra_sor_writel(sor, value, SOR_DP_LINKCTL_0);

	usleep_range(400, 1000);

	/* configure load pulse position adjustment */
	value = tegra_sor_readl(sor, SOR_PLL_1);
	value &= ~SOR_PLL1_LOADADJ_MASK;

	switch (rate) {
	case DP_LINK_BW_1_62:
		value |= SOR_PLL1_LOADADJ(0x3);
		break;

	case DP_LINK_BW_2_7:
		value |= SOR_PLL1_LOADADJ(0x4);
		break;

	case DP_LINK_BW_5_4:
		value |= SOR_PLL1_LOADADJ(0x6);
		break;
	}

	tegra_sor_writel(sor, value, SOR_PLL_1);

	err = tegra_sor_power_down_lanes(sor);
	if (err < 0) {
		dev_err(sor->dev, "failed to power down lanes: %d\n", err);
		return err;
	}

	/* power up and pre-charge lanes */
	err = tegra_sor_power_up_lanes(sor, lanes);
	if (err < 0) {
		dev_err(sor->dev, "failed to power up %u lane%s: %d\n",
			lanes, (lanes != 1) ? "s" : "", err);
		return err;
	}

	tegra_sor_dp_precharge(sor, lanes);

	return 0;
}

static const struct drm_dp_link_ops tegra_sor_dp_link_ops = {
	.apply_training = tegra_sor_dp_link_apply_training,
	.configure = tegra_sor_dp_link_configure,
};

static void tegra_sor_apply_config(struct tegra_sor *sor,
				   const struct tegra_sor_config *config)
{
	u32 value;

	/* set linkctl */
	value = tegra_sor_readl(sor, SOR_DP_LINKCTL_0);
	value &= ~SOR_DP_LINKCTL_TU_SIZE_MASK;
	value |= SOR_DP_LINKCTL_TU_SIZE(config->tu_size);
	tegra_sor_writel(sor, value, SOR_DP_LINKCTL_0);

	value = tegra_sor_readl(sor, SOR_DP_CONFIG_0);
	value &= ~SOR_DP_CONFIG_WATERMARK_MASK;
	value |= SOR_DP_CONFIG_WATERMARK(config->watermark);

	value &= ~SOR_DP_CONFIG_ACTIVE_SYM_COUNT_MASK;
	value |= SOR_DP_CONFIG_ACTIVE_SYM_COUNT(config->active_count);

	value &= ~SOR_DP_CONFIG_ACTIVE_SYM_FRAC_MASK;
	value |= SOR_DP_CONFIG_ACTIVE_SYM_FRAC(config->active_frac);

	if (config->active_polarity)
		value |= SOR_DP_CONFIG_ACTIVE_SYM_POLARITY;
	else
		value &= ~SOR_DP_CONFIG_ACTIVE_SYM_POLARITY;

	value |= SOR_DP_CONFIG_ACTIVE_SYM_ENABLE;
	value |= SOR_DP_CONFIG_DISPARITY_NEGATIVE;
	tegra_sor_writel(sor, value, SOR_DP_CONFIG_0);

	value = tegra_sor_readl(sor, SOR_DP_AUDIO_HBLANK_SYMBOLS);
	value &= ~SOR_DP_AUDIO_HBLANK_SYMBOLS_MASK;
	value |= config->hblank_symbols & 0xffff;
	tegra_sor_writel(sor, value, SOR_DP_AUDIO_HBLANK_SYMBOLS);

	value = tegra_sor_readl(sor, SOR_DP_AUDIO_VBLANK_SYMBOLS);
	value &= ~SOR_DP_AUDIO_VBLANK_SYMBOLS_MASK;
	value |= config->vblank_symbols & 0xffff;
	tegra_sor_writel(sor, value, SOR_DP_AUDIO_VBLANK_SYMBOLS);
}

static void tegra_sor_mode_set(struct tegra_sor *sor,
			       const struct drm_display_mode *mode,
			       struct drm_display_info *info)
{
	struct tegra_dc *dc = to_tegra_dc(sor->output.encoder.crtc);
	unsigned int vbe, vse, hbe, hse, vbs, hbs;
	u32 value;

	value = tegra_sor_readl(sor, SOR_STATE_1);
	value &= ~SOR_STATE_ASY_PIXELDEPTH_MASK;
	value &= ~SOR_STATE_ASY_CRC_MODE_MASK;
	value &= ~SOR_STATE_ASY_OWNER_MASK;

	value |= SOR_STATE_ASY_CRC_MODE_COMPLETE |
		 SOR_STATE_ASY_OWNER(dc->pipe + 1);

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		value &= ~SOR_STATE_ASY_HSYNCPOL;

	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		value |= SOR_STATE_ASY_HSYNCPOL;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		value &= ~SOR_STATE_ASY_VSYNCPOL;

	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		value |= SOR_STATE_ASY_VSYNCPOL;

	switch (info->bpc) {
	case 16:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_48_444;
		break;

	case 12:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_36_444;
		break;

	case 10:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_30_444;
		break;

	case 8:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_24_444;
		break;

	case 6:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_18_444;
		break;

	default:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_24_444;
		break;
	}

	tegra_sor_writel(sor, value, SOR_STATE_1);

	/*
	 * TODO: The video timing programming below doesn't seem to match the
	 * register definitions.
	 */

	value = ((mode->vtotal & 0x7fff) << 16) | (mode->htotal & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE_1(dc->pipe));

	/* sync end = sync width - 1 */
	vse = mode->vsync_end - mode->vsync_start - 1;
	hse = mode->hsync_end - mode->hsync_start - 1;

	value = ((vse & 0x7fff) << 16) | (hse & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE_2(dc->pipe));

	/* blank end = sync end + back porch */
	vbe = vse + (mode->vtotal - mode->vsync_end);
	hbe = hse + (mode->htotal - mode->hsync_end);

	value = ((vbe & 0x7fff) << 16) | (hbe & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE_3(dc->pipe));

	/* blank start = blank end + active */
	vbs = vbe + mode->vdisplay;
	hbs = hbe + mode->hdisplay;

	value = ((vbs & 0x7fff) << 16) | (hbs & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE_4(dc->pipe));

	/* XXX interlacing support */
	tegra_sor_writel(sor, 0x001, SOR_HEAD_STATE_5(dc->pipe));
}

static int tegra_output_sor_enable(struct tegra_output *output)
{
	struct tegra_dc *dc = to_tegra_dc(output->encoder.crtc);
	struct drm_display_mode *mode = &dc->base.mode;
	struct drm_display_info info;
	unsigned int i;
	struct tegra_sor *sor = to_sor(output);
	struct tegra_sor_config config;
	struct drm_dp_aux *aux;
	unsigned long value;
	int err = 0;
	bool write_active = false;
	u8 config_cap = 0, config_set = 0;

	mutex_lock(&sor->lock);

	if (sor->enabled)
		goto unlock;

	err = clk_prepare_enable(sor->clk);
	if (err < 0)
		goto unlock;

	/* Make a copy of the display info and sub in our hardcoded bpc */
	memcpy(&info, &output->connector.display_info, sizeof(info));
	info.bpc = 8; /* FIXME: don't hardcode */

	reset_control_deassert(sor->rst);

	/* FIXME: properly convert to struct drm_dp_aux */
	aux = (struct drm_dp_aux *)sor->dpaux;

	if (sor->dpaux) {
		err = tegra_dpaux_enable(sor->dpaux);
		if (err < 0)
			dev_err(sor->dev, "failed to enable DP: %d\n", err);
	}

	err = clk_set_parent(sor->clk, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe parent clock: %d\n", err);

	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_CLK_SEL_MASK;
	value |= SOR_CLK_CNTRL_DP_CLK_SEL_SINGLE_DPCLK;
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	value = tegra_sor_readl(sor, SOR_PLL_2);
	value &= ~SOR_PLL_2_BANDGAP_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL_2);
	usleep_range(20, 100);

	value = tegra_sor_readl(sor, SOR_PLL_3);
	value |= SOR_PLL_3_PLL_VDD_MODE_V3_3;
	tegra_sor_writel(sor, value, SOR_PLL_3);

	if (sor->dpaux) {
		err = drm_dp_dpcd_readb(aux, DP_EDP_CONFIGURATION_CAP,
				       &config_cap);
		if (err < 0)
			dev_err(sor->dev,
				"failed to read eDP capabilites: %d\n", err);

		err = drm_dp_dpcd_readb(aux, DP_EDP_CONFIGURATION_SET,
				       &config_set);
		if (err < 0)
			dev_err(sor->dev,
				"failed to read eDP capabilites: %d\n", err);
	}

	/*
	 * Use Alternative Scrambler Seed Reset (ASSR) iff the panel supports it
	 * and is currently configured to use it.
	 */
	value = tegra_sor_readl(sor, SOR_DP_SPARE_0);
	value |= SOR_DP_SPARE_SEQ_ENABLE;
	if ((config_cap & DP_ALTERNATE_SCRAMBLER_RESET_CAPABLE) &&
	    (config_set & DP_ALTERNATE_SCRAMBLER_RESET_ENABLE))
		value |= SOR_DP_SPARE_PANEL_INTERNAL;
	else
		value &= ~SOR_DP_SPARE_PANEL_INTERNAL;
	value |= SOR_DP_SPARE_MACRO_SOR_CLK;
	tegra_sor_writel(sor, value, SOR_DP_SPARE_0);

	/* disable LVDS mode */
	tegra_sor_writel(sor, 0, SOR_LVDS);

	value = SOR_PLL_0_ICHPMP(0xf) | SOR_PLL_0_VCOCAP_RST |
		SOR_PLL_0_PLLREG_LEVEL_V45 | SOR_PLL_0_RESISTOR_EXT;
	tegra_sor_writel(sor, value, SOR_PLL_0);

	value = tegra_sor_readl(sor, SOR_PLL_2);
	value |= SOR_PLL_2_SEQ_PLLCAPPD;
	value &= ~SOR_PLL_2_SEQ_PLLCAPPD_ENFORCE;
	value |= SOR_PLL_2_LVDS_ENABLE;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	value = SOR_PLL_1_TERM_COMPOUT | SOR_PLL_1_TMDS_TERM;
	tegra_sor_writel(sor, value, SOR_PLL_1);

	while (true) {
		value = tegra_sor_readl(sor, SOR_PLL_2);
		if ((value & SOR_PLL_2_SEQ_PLLCAPPD_ENFORCE) == 0)
			break;

		usleep_range(250, 1000);
	}

	value = tegra_sor_readl(sor, SOR_PLL_2);
	value &= ~SOR_PLL_2_POWERDOWN_OVERRIDE;
	value &= ~SOR_PLL_2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	/*
	 * power up
	 */

	/* set safe link bandwidth (1.62 Gbps) */
	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_LINK_SPEED_MASK;
	value |= SOR_CLK_CNTRL_DP_LINK_SPEED_G1_62;
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	/* step 1 */
	value = tegra_sor_readl(sor, SOR_PLL_2);
	value |= SOR_PLL_2_SEQ_PLLCAPPD_ENFORCE | SOR_PLL_2_PORT_POWERDOWN |
		 SOR_PLL_2_BANDGAP_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	value = tegra_sor_readl(sor, SOR_PLL_0);
	value |= SOR_PLL_0_VCOPD | SOR_PLL_0_POWER_OFF;
	tegra_sor_writel(sor, value, SOR_PLL_0);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value &= ~SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	/* step 2 */
	err = tegra_io_rail_power_on(TEGRA_IO_RAIL_LVDS);
	if (err < 0) {
		dev_err(sor->dev, "failed to power on I/O rail: %d\n", err);
		goto unlock;
	}

	usleep_range(5, 100);

	/* step 3 */
	value = tegra_sor_readl(sor, SOR_PLL_2);
	value &= ~SOR_PLL_2_BANDGAP_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	usleep_range(20, 100);

	/* step 4 */
	value = tegra_sor_readl(sor, SOR_PLL_0);
	value &= ~SOR_PLL_0_POWER_OFF;
	value &= ~SOR_PLL_0_VCOPD;
	tegra_sor_writel(sor, value, SOR_PLL_0);

	value = tegra_sor_readl(sor, SOR_PLL_2);
	value &= ~SOR_PLL_2_SEQ_PLLCAPPD_ENFORCE;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	usleep_range(200, 1000);

	/* step 5 */
	value = tegra_sor_readl(sor, SOR_PLL_2);
	value &= ~SOR_PLL_2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	/* switch to DP clock */
	err = clk_set_parent(sor->clk, sor->clk_dp);
	if (err < 0)
		dev_err(sor->dev, "failed to set DP parent clock: %d\n", err);

	err = tegra_sor_power_up_lanes(sor, sor->link.num_lanes);
	if (err < 0)
		dev_err(sor->dev, "failed to power up lanes %d\n", err);

	/* set link bandwidth */
	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_LINK_SPEED_MASK;
	value |= drm_dp_link_rate_to_bw_code(sor->link.rate) << 2;
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	for (i = 0, value = 0; i < 4; i++) {
		unsigned long lane = SOR_DP_TPG_CHANNEL_CODING |
				     SOR_DP_TPG_SCRAMBLER_GALIOS |
				     SOR_DP_TPG_PATTERN_NONE;
		value = (value << 8) | lane;
	}

	tegra_sor_writel(sor, value, SOR_DP_TPG);

	/* enable pad calibration logic */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value |= SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	/* use DP-A protocol */
	value = tegra_sor_readl(sor, SOR_STATE_1);
	value &= ~SOR_STATE_ASY_PROTOCOL_MASK;
	value |= SOR_STATE_ASY_PROTOCOL_DP_A;
	tegra_sor_writel(sor, value, SOR_STATE_1);

	/* enable port */
	value = tegra_sor_readl(sor, SOR_DP_LINKCTL_0);
	value |= SOR_DP_LINKCTL_ENABLE;
	tegra_sor_writel(sor, value, SOR_DP_LINKCTL_0);

	if (sor->dpaux) {
		err = drm_dp_link_probe(aux, &sor->link);
		if (err < 0) {
			dev_err(sor->dev, "failed to probe eDP link: %d\n",
				err);
			goto unlock;
		}
		sor->link.capabilities |= DP_LINK_CAP_ANSI_8B10B;

		err = drm_dp_link_choose(&sor->link, mode, &info);
		if (err < 0) {
			dev_err(sor->dev, "failed to choose link: %d\n", err);
			goto unlock;
		}

		err = drm_dp_link_power_up(aux, &sor->link);
		if (err < 0) {
			dev_err(sor->dev, "failed to power up eDP link: %d\n",
				err);
			goto unlock;
		}

		tegra_sor_dp_term_calibrate(sor);
		drm_dp_link_train_init(&sor->link.train);

		/*
		 * Workaround for some Nyan panels that train just
		 * fine, but don't report successful status. The fbdev
		 * driver had a bug where it didn't check status for
		 * single lane configs, which is how they used to
		 * work.
		 *
		 * https://bugs.chromium.org/p/chromium/issues/detail?id=737618
		 */
		sor->link.train.clock_recovered = true;
		sor->link.train.channel_equalized = true;

		err = drm_dp_link_train(&sor->link);
		if (err < 0) {
			dev_err(sor->dev, "DP link training failed: %d\n", err);
			goto unlock;
		} else {
			dev_dbg(sor->dev, "DP link training succeeded\n");
		}
	}

	/* compute configuration */
	memset(&config, 0, sizeof(config));
	config.bits_per_pixel = info.bpc * 3;

	err = tegra_sor_calc_config(sor, mode, &config, &sor->link);
	if (err < 0)
		dev_err(sor->dev, "failed to compute link configuration: %d\n",
			err);

	tegra_sor_apply_config(sor, &config);
	tegra_sor_mode_set(sor, mode, &info);

	err = tegra_sor_power_up(sor, 250);
	if (err < 0) {
		dev_err(sor->dev, "failed to power up SOR: %d\n", err);
		goto unlock;
	}

	/* start display controller in continuous mode */
	value = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	value |= WRITE_MUX;
	tegra_dc_writel(dc, value, DC_CMD_STATE_ACCESS);
	write_active = true;

	tegra_dc_writel(dc, VSYNC_H_POSITION(1), DC_DISP_DISP_TIMING_OPTIONS);
	tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY, DC_CMD_DISPLAY_COMMAND);

	/* CSTM (LVDS, link A/B, upper) */
	value = SOR_CSTM_LVDS | SOR_CSTM_LINK_ACT_A | SOR_CSTM_LINK_ACT_B |
		SOR_CSTM_UPPER;
	tegra_sor_writel(sor, value, SOR_CSTM);

	/* PWM setup */
	err = tegra_sor_setup_pwm(sor, 250);
	if (err < 0) {
		dev_err(sor->dev, "failed to setup PWM: %d\n", err);
		goto unlock;
	}

	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value |= SOR_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	tegra_sor_update(sor);

	err = tegra_sor_attach(sor);
	if (err < 0) {
		dev_err(sor->dev, "failed to attach SOR: %d\n", err);
		goto unlock;
	}

	err = tegra_sor_wakeup(sor);
	if (err < 0) {
		dev_err(sor->dev, "failed to enable DC: %d\n", err);
		goto unlock;
	}

	sor->enabled = true;

unlock:
	if (write_active) {
		value = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
		value &= ~WRITE_MUX;
		tegra_dc_writel(dc, value, DC_CMD_STATE_ACCESS);
	}
	mutex_unlock(&sor->lock);
	return err;
}

static int tegra_sor_detach(struct tegra_sor *sor)
{
	unsigned long value, timeout;

	/* switch to safe mode */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE_1);
	value &= ~SOR_SUPER_STATE_MODE_NORMAL;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE_1);
	tegra_sor_super_update(sor);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_PWR);
		if (value & SOR_PWR_MODE_SAFE)
			break;
	}

	if ((value & SOR_PWR_MODE_SAFE) == 0)
		return -ETIMEDOUT;

	/* go to sleep */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE_1);
	value &= ~SOR_SUPER_STATE_HEAD_MODE_MASK;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE_1);
	tegra_sor_super_update(sor);

	/* detach */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE_1);
	value &= ~SOR_SUPER_STATE_ATTACHED;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE_1);
	tegra_sor_super_update(sor);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_TEST);
		if ((value & SOR_TEST_ATTACHED) == 0)
			break;

		usleep_range(25, 100);
	}

	if ((value & SOR_TEST_ATTACHED) != 0)
		return -ETIMEDOUT;

	return 0;
}

static int tegra_sor_power_down(struct tegra_sor *sor)
{
	unsigned long value, timeout;
	int err;

	value = tegra_sor_readl(sor, SOR_PWR);
	value &= ~SOR_PWR_NORMAL_STATE_PU;
	value |= SOR_PWR_TRIGGER;
	tegra_sor_writel(sor, value, SOR_PWR);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_PWR);
		if ((value & SOR_PWR_TRIGGER) == 0)
			return 0;

		usleep_range(25, 100);
	}

	if ((value & SOR_PWR_TRIGGER) != 0)
		return -ETIMEDOUT;

	err = clk_set_parent(sor->clk, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe parent clock: %d\n", err);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL_0);
	value &= ~(SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_0 |
		   SOR_DP_PADCTL_PD_TXD_1 | SOR_DP_PADCTL_PD_TXD_2);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL_0);

	/* stop lane sequencer */
	value = SOR_LANE_SEQ_CTL_TRIGGER | SOR_LANE_SEQ_CTL_SEQUENCE_UP |
		SOR_LANE_SEQ_CTL_POWER_STATE_DOWN;
	tegra_sor_writel(sor, value, SOR_LANE_SEQ_CTL);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_TRIGGER) == 0)
			break;

		usleep_range(25, 100);
	}

	if ((value & SOR_LANE_SEQ_CTL_TRIGGER) != 0)
		return -ETIMEDOUT;

	value = tegra_sor_readl(sor, SOR_PLL_2);
	value |= SOR_PLL_2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	usleep_range(20, 100);

	value = tegra_sor_readl(sor, SOR_PLL_0);
	value |= SOR_PLL_0_POWER_OFF;
	value |= SOR_PLL_0_VCOPD;
	tegra_sor_writel(sor, value, SOR_PLL_0);

	value = tegra_sor_readl(sor, SOR_PLL_2);
	value |= SOR_PLL_2_SEQ_PLLCAPPD;
	value |= SOR_PLL_2_SEQ_PLLCAPPD_ENFORCE;
	tegra_sor_writel(sor, value, SOR_PLL_2);

	usleep_range(20, 100);

	return 0;
}

static int tegra_output_sor_disable(struct tegra_output *output)
{
	struct tegra_dc *dc = to_tegra_dc(output->encoder.crtc);
	struct tegra_sor *sor = to_sor(output);
	unsigned long value;
	int err = 0;

	mutex_lock(&sor->lock);

	if (!sor->enabled)
		goto unlock;

	err = tegra_sor_detach(sor);
	if (err < 0) {
		dev_err(sor->dev, "failed to detach SOR: %d\n", err);
		goto unlock;
	}

	tegra_sor_writel(sor, 0, SOR_STATE_1);
	tegra_sor_update(sor);

	/*
	 * The following accesses registers of the display controller, so make
	 * sure it's only executed when the output is attached to one.
	 */
	if (dc) {
		/*
		 * XXX: We can't do this here because it causes the SOR to go
		 * into an erroneous state and the output will look scrambled
		 * the next time it is enabled. Presumably this is because we
		 * should be doing this only on the next VBLANK. A possible
		 * solution would be to queue a "power-off" event to trigger
		 * this code to be run during the next VBLANK.
		 */
		/*
		value = tegra_dc_readl(dc, DC_CMD_DISPLAY_POWER_CONTROL);
		value &= ~(PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
			   PW4_ENABLE | PM0_ENABLE | PM1_ENABLE);
		tegra_dc_writel(dc, value, DC_CMD_DISPLAY_POWER_CONTROL);
		*/

		value = tegra_dc_readl(dc, DC_CMD_DISPLAY_COMMAND);
		value &= ~DISP_CTRL_MODE_MASK;
		tegra_dc_writel(dc, value, DC_CMD_DISPLAY_COMMAND);

		value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
		value &= ~SOR_ENABLE;
		tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

		tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
		tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	}

	err = tegra_sor_power_down(sor);
	if (err < 0) {
		dev_err(sor->dev, "failed to power down SOR: %d\n", err);
		goto unlock;
	}

	if (sor->dpaux) {
		err = tegra_dpaux_disable(sor->dpaux);
		if (err < 0) {
			dev_err(sor->dev, "failed to disable DP: %d\n", err);
			goto unlock;
		}
	}

	err = tegra_io_rail_power_off(TEGRA_IO_RAIL_LVDS);
	if (err < 0) {
		dev_err(sor->dev, "failed to power off I/O rail: %d\n", err);
		goto unlock;
	}

	reset_control_assert(sor->rst);
	clk_disable_unprepare(sor->clk);

	sor->enabled = false;

unlock:
	mutex_unlock(&sor->lock);
	return err;
}

static int tegra_output_sor_setup_clock(struct tegra_output *output,
					struct clk *clk, unsigned long pclk,
					unsigned int *div)
{
	struct tegra_sor *sor = to_sor(output);
	int err;

	err = clk_set_parent(clk, sor->clk_parent);
	if (err < 0) {
		dev_err(sor->dev, "failed to set parent clock: %d\n", err);
		return err;
	}

	err = clk_set_rate(sor->clk_parent, pclk);
	if (err < 0) {
		dev_err(sor->dev, "failed to set clock rate to %lu Hz\n", pclk);
		return err;
	}

	*div = 0;

	return 0;
}

static int tegra_output_sor_check_mode(struct tegra_output *output,
				       struct drm_display_mode *mode,
				       enum drm_mode_status *status)
{
	/*
	 * FIXME: For now, always assume that the mode is okay.
	 */

	*status = MODE_OK;

	return 0;
}

static enum drm_connector_status
tegra_output_sor_detect(struct tegra_output *output)
{
	struct tegra_sor *sor = to_sor(output);
	enum drm_connector_status ret;

	mutex_lock(&sor->lock);
	if (sor->dpaux)
		ret = tegra_dpaux_detect(sor->dpaux);
	else
		ret = connector_status_unknown;
	mutex_unlock(&sor->lock);

	return ret;
}

static const struct tegra_output_ops sor_ops = {
	.enable = tegra_output_sor_enable,
	.disable = tegra_output_sor_disable,
	.setup_clock = tegra_output_sor_setup_clock,
	.check_mode = tegra_output_sor_check_mode,
	.detect = tegra_output_sor_detect,
};

static int tegra_sor_crc_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static int tegra_sor_crc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int tegra_sor_crc_wait(struct tegra_sor *sor, unsigned long timeout)
{
	u32 value;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_CRC_A);
		if (value & SOR_CRC_A_VALID)
			return 0;

		usleep_range(100, 200);
	}

	return -ETIMEDOUT;
}

static ssize_t tegra_sor_crc_read(struct file *file, char __user *buffer,
				  size_t size, loff_t *ppos)
{
	struct tegra_sor *sor = file->private_data;
	ssize_t num, err;
	char buf[10];
	u32 value;

	mutex_lock(&sor->lock);

	if (!sor->enabled) {
		err = -EAGAIN;
		goto unlock;
	}

	value = tegra_sor_readl(sor, SOR_STATE_1);
	value &= ~SOR_STATE_ASY_CRC_MODE_MASK;
	tegra_sor_writel(sor, value, SOR_STATE_1);

	value = tegra_sor_readl(sor, SOR_CRC_CNTRL);
	value |= SOR_CRC_CNTRL_ENABLE;
	tegra_sor_writel(sor, value, SOR_CRC_CNTRL);

	value = tegra_sor_readl(sor, SOR_TEST);
	value &= ~SOR_TEST_CRC_POST_SERIALIZE;
	tegra_sor_writel(sor, value, SOR_TEST);

	err = tegra_sor_crc_wait(sor, 100);
	if (err < 0)
		goto unlock;

	tegra_sor_writel(sor, SOR_CRC_A_RESET, SOR_CRC_A);
	value = tegra_sor_readl(sor, SOR_CRC_B);

	num = scnprintf(buf, sizeof(buf), "%08x\n", value);

	err = simple_read_from_buffer(buffer, size, ppos, buf, num);

unlock:
	mutex_unlock(&sor->lock);
	return err;
}

static const struct file_operations tegra_sor_crc_fops = {
	.owner = THIS_MODULE,
	.open = tegra_sor_crc_open,
	.read = tegra_sor_crc_read,
	.release = tegra_sor_crc_release,
};

static int tegra_sor_show_regs(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_sor *sor = node->info_ent->data;

#define DUMP_REG(name)						\
	seq_printf(s, "%-40s %#05x %08lx\n", #name, name,	\
		   tegra_sor_readl(sor, name))

	DUMP_REG(SOR_CTXSW);
	DUMP_REG(SOR_SUPER_STATE_0);
	DUMP_REG(SOR_SUPER_STATE_1);
	DUMP_REG(SOR_STATE_0);
	DUMP_REG(SOR_STATE_1);
	DUMP_REG(SOR_HEAD_STATE_0(0));
	DUMP_REG(SOR_HEAD_STATE_0(1));
	DUMP_REG(SOR_HEAD_STATE_1(0));
	DUMP_REG(SOR_HEAD_STATE_1(1));
	DUMP_REG(SOR_HEAD_STATE_2(0));
	DUMP_REG(SOR_HEAD_STATE_2(1));
	DUMP_REG(SOR_HEAD_STATE_3(0));
	DUMP_REG(SOR_HEAD_STATE_3(1));
	DUMP_REG(SOR_HEAD_STATE_4(0));
	DUMP_REG(SOR_HEAD_STATE_4(1));
	DUMP_REG(SOR_HEAD_STATE_5(0));
	DUMP_REG(SOR_HEAD_STATE_5(1));
	DUMP_REG(SOR_CRC_CNTRL);
	DUMP_REG(SOR_DP_DEBUG_MVID);
	DUMP_REG(SOR_CLK_CNTRL);
	DUMP_REG(SOR_CAP);
	DUMP_REG(SOR_PWR);
	DUMP_REG(SOR_TEST);
	DUMP_REG(SOR_PLL_0);
	DUMP_REG(SOR_PLL_1);
	DUMP_REG(SOR_PLL_2);
	DUMP_REG(SOR_PLL_3);
	DUMP_REG(SOR_CSTM);
	DUMP_REG(SOR_LVDS);
	DUMP_REG(SOR_CRC_A);
	DUMP_REG(SOR_CRC_B);
	DUMP_REG(SOR_BLANK);
	DUMP_REG(SOR_SEQ_CTL);
	DUMP_REG(SOR_LANE_SEQ_CTL);
	DUMP_REG(SOR_SEQ_INST(0x0));
	DUMP_REG(SOR_SEQ_INST(0x1));
	DUMP_REG(SOR_SEQ_INST(0x2));
	DUMP_REG(SOR_SEQ_INST(0x3));
	DUMP_REG(SOR_SEQ_INST(0x4));
	DUMP_REG(SOR_SEQ_INST(0x5));
	DUMP_REG(SOR_SEQ_INST(0x6));
	DUMP_REG(SOR_SEQ_INST(0x7));
	DUMP_REG(SOR_SEQ_INST(0x8));
	DUMP_REG(SOR_SEQ_INST(0x9));
	DUMP_REG(SOR_SEQ_INST(0xa));
	DUMP_REG(SOR_SEQ_INST(0xb));
	DUMP_REG(SOR_SEQ_INST(0xc));
	DUMP_REG(SOR_SEQ_INST(0xd));
	DUMP_REG(SOR_SEQ_INST(0xe));
	DUMP_REG(SOR_SEQ_INST(0xf));
	DUMP_REG(SOR_PWM_DIV);
	DUMP_REG(SOR_PWM_CTL);
	DUMP_REG(SOR_VCRC_A_0);
	DUMP_REG(SOR_VCRC_A_1);
	DUMP_REG(SOR_VCRC_B_0);
	DUMP_REG(SOR_VCRC_B_1);
	DUMP_REG(SOR_CCRC_A_0);
	DUMP_REG(SOR_CCRC_A_1);
	DUMP_REG(SOR_CCRC_B_0);
	DUMP_REG(SOR_CCRC_B_1);
	DUMP_REG(SOR_EDATA_A_0);
	DUMP_REG(SOR_EDATA_A_1);
	DUMP_REG(SOR_EDATA_B_0);
	DUMP_REG(SOR_EDATA_B_1);
	DUMP_REG(SOR_COUNT_A_0);
	DUMP_REG(SOR_COUNT_A_1);
	DUMP_REG(SOR_COUNT_B_0);
	DUMP_REG(SOR_COUNT_B_1);
	DUMP_REG(SOR_DEBUG_A_0);
	DUMP_REG(SOR_DEBUG_A_1);
	DUMP_REG(SOR_DEBUG_B_0);
	DUMP_REG(SOR_DEBUG_B_1);
	DUMP_REG(SOR_TRIG);
	DUMP_REG(SOR_MSCHECK);
	DUMP_REG(SOR_XBAR_CTRL);
	DUMP_REG(SOR_XBAR_POL);
	DUMP_REG(SOR_DP_LINKCTL_0);
	DUMP_REG(SOR_DP_LINKCTL_1);
	DUMP_REG(SOR_LANE_DRIVE_CURRENT_0);
	DUMP_REG(SOR_LANE_DRIVE_CURRENT_1);
	DUMP_REG(SOR_LANE4_DRIVE_CURRENT_0);
	DUMP_REG(SOR_LANE4_DRIVE_CURRENT_1);
	DUMP_REG(SOR_LANE_PREEMPHASIS_0);
	DUMP_REG(SOR_LANE_PREEMPHASIS_1);
	DUMP_REG(SOR_LANE4_PREEMPHASIS_0);
	DUMP_REG(SOR_LANE4_PREEMPHASIS_1);
	DUMP_REG(SOR_LANE_POST_CURSOR_0);
	DUMP_REG(SOR_LANE_POST_CURSOR_1);
	DUMP_REG(SOR_DP_CONFIG_0);
	DUMP_REG(SOR_DP_CONFIG_1);
	DUMP_REG(SOR_DP_MN_0);
	DUMP_REG(SOR_DP_MN_1);
	DUMP_REG(SOR_DP_PADCTL_0);
	DUMP_REG(SOR_DP_PADCTL_1);
	DUMP_REG(SOR_DP_DEBUG_0);
	DUMP_REG(SOR_DP_DEBUG_1);
	DUMP_REG(SOR_DP_SPARE_0);
	DUMP_REG(SOR_DP_SPARE_1);
	DUMP_REG(SOR_DP_AUDIO_CTRL);
	DUMP_REG(SOR_DP_AUDIO_HBLANK_SYMBOLS);
	DUMP_REG(SOR_DP_AUDIO_VBLANK_SYMBOLS);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_HEADER);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_0);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_1);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_2);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_3);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_4);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_5);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK_6);
	DUMP_REG(SOR_DP_TPG);
	DUMP_REG(SOR_DP_TPG_CONFIG);
	DUMP_REG(SOR_DP_LQ_CSTM_0);
	DUMP_REG(SOR_DP_LQ_CSTM_1);
	DUMP_REG(SOR_DP_LQ_CSTM_2);

#undef DUMP_REG

	return 0;
}

static struct drm_info_list debugfs_files[] = {
	{ "regs", tegra_sor_show_regs, 0, NULL },
};

static int tegra_sor_debugfs_init(struct tegra_sor *sor,
				  struct drm_minor *minor)
{
	struct dentry *entry;
	int err = 0;
	int i;

	sor->debugfs = debugfs_create_dir("sor", minor->debugfs_root);
	if (!sor->debugfs)
		return -ENOMEM;

	entry = debugfs_create_file("crc", 0644, sor->debugfs, sor,
				    &tegra_sor_crc_fops);
	if (!entry) {
		dev_err(sor->dev,
			"cannot create /sys/kernel/debug/dri/%s/sor/crc\n",
			minor->debugfs_root->d_name.name);
		err = -ENOMEM;
		goto remove;
	}

	sor->debugfs_files = kmemdup(debugfs_files, sizeof(debugfs_files),
				    GFP_KERNEL);
	if (!sor->debugfs_files) {
		err = -ENOMEM;
		goto remove;
	}

	for (i = 0; i < ARRAY_SIZE(debugfs_files); i++)
		sor->debugfs_files[i].data = sor;

	err = drm_debugfs_create_files(sor->debugfs_files,
				       ARRAY_SIZE(debugfs_files),
				       sor->debugfs, minor);
	if (err < 0)
		goto free;

	sor->minor = minor;

	return err;

free:
	kfree(sor->debugfs_files);
	sor->debugfs_files = NULL;
remove:
	debugfs_remove_recursive(sor->debugfs);
	sor->debugfs = NULL;
	return err;
}

static int tegra_sor_debugfs_exit(struct tegra_sor *sor)
{
	sor->minor = NULL;

	kfree(sor->debugfs_files);
	sor->debugfs_files = NULL;

	debugfs_remove_recursive(sor->debugfs);
	sor->debugfs = NULL;

	return 0;
}

static int tegra_sor_init(struct host1x_client *client)
{
	struct tegra_drm *tegra = dev_get_drvdata(client->parent);
	struct tegra_sor *sor = host1x_client_to_sor(client);
	int err;

	if (!sor->dpaux)
		return -ENODEV;

	sor->output.type = TEGRA_OUTPUT_EDP;

	sor->output.dev = sor->dev;
	sor->output.ops = &sor_ops;

	err = tegra_output_init(tegra->drm, &sor->output);
	if (err < 0) {
		dev_err(sor->dev, "output setup failed: %d\n", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_sor_debugfs_init(sor, tegra->drm->primary);
		if (err < 0)
			dev_err(sor->dev, "debugfs setup failed: %d\n", err);
	}

	if (sor->dpaux) {
		err = tegra_dpaux_attach(sor->dpaux, &sor->output);
		if (err < 0) {
			dev_err(sor->dev, "failed to attach DP: %d\n", err);
			return err;
		}
	}

	sor->link.ops = &tegra_sor_dp_link_ops;
	sor->link.aux = (struct drm_dp_aux *)sor->dpaux;

	return 0;
}

static int tegra_sor_exit(struct host1x_client *client)
{
	struct tegra_sor *sor = host1x_client_to_sor(client);
	int err;

	err = tegra_output_disable(&sor->output);
	if (err < 0) {
		dev_err(sor->dev, "output failed to disable: %d\n", err);
		return err;
	}

	if (sor->dpaux) {
		err = tegra_dpaux_detach(sor->dpaux);
		if (err < 0) {
			dev_err(sor->dev, "failed to detach DP: %d\n", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_sor_debugfs_exit(sor);
		if (err < 0)
			dev_err(sor->dev, "debugfs cleanup failed: %d\n", err);
	}

	err = tegra_output_exit(&sor->output);
	if (err < 0) {
		dev_err(sor->dev, "output cleanup failed: %d\n", err);
		return err;
	}

	return 0;
}

static const struct host1x_client_ops sor_client_ops = {
	.init = tegra_sor_init,
	.exit = tegra_sor_exit,
};

static int tegra_sor_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct tegra_sor *sor;
	struct host1x_client *client;
	struct resource *regs;
	int err;

	client = drm_host1x_get_client(&pdev->dev);
	if (client) {
		sor = host1x_client_to_sor(client);
	} else {
		sor = kzalloc(sizeof(*sor), GFP_KERNEL);
		if (!sor)
			return -ENOMEM;

		INIT_LIST_HEAD(&sor->client.list);
		sor->client.ops = &sor_client_ops;
		sor->client.dev = &pdev->dev;
	}

	err = drm_host1x_register(&sor->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register host1x client: %d\n",
			err);
		return err;
	}

	sor->output.dev = sor->dev = &pdev->dev;

	np = of_parse_phandle(pdev->dev.of_node, "nvidia,dpaux", 0);
	if (np) {
		sor->dpaux = tegra_dpaux_find_by_of_node(np);
		of_node_put(np);

		if (!sor->dpaux)
			return -EPROBE_DEFER;
	}

	err = tegra_output_probe(&sor->output);
	if (err < 0)
		return err;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sor->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(sor->regs))
		return PTR_ERR(sor->regs);

	sor->rst = devm_reset_control_get(&pdev->dev, "sor");
	if (IS_ERR(sor->rst))
		return PTR_ERR(sor->rst);

	sor->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sor->clk))
		return PTR_ERR(sor->clk);

	sor->clk_parent = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(sor->clk_parent))
		return PTR_ERR(sor->clk_parent);

	err = clk_prepare_enable(sor->clk_parent);
	if (err < 0)
		return err;

	sor->clk_safe = devm_clk_get(&pdev->dev, "safe");
	if (IS_ERR(sor->clk_safe))
		return PTR_ERR(sor->clk_safe);

	err = clk_prepare_enable(sor->clk_safe);
	if (err < 0)
		return err;

	sor->clk_dp = devm_clk_get(&pdev->dev, "dp");
	if (IS_ERR(sor->clk_dp))
		return PTR_ERR(sor->clk_dp);

	err = clk_prepare_enable(sor->clk_dp);
	if (err < 0)
		return err;

	mutex_init(&sor->lock);

	platform_set_drvdata(pdev, sor);
	sor->client.driver_probed = 1;
	dev_info(&pdev->dev, "initialized\n");
	return 0;
}

static int tegra_sor_remove(struct platform_device *pdev)
{
	struct tegra_sor *sor = platform_get_drvdata(pdev);
	int err;

	err = drm_host1x_unregister(&sor->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);
		return err;
	}

	clk_disable_unprepare(sor->clk_parent);
	clk_disable_unprepare(sor->clk_safe);
	clk_disable_unprepare(sor->clk_dp);
	clk_disable_unprepare(sor->clk);

	kfree(sor);
	return 0;
}

static const struct of_device_id tegra_sor_of_match[] = {
	{ .compatible = "nvidia,tegra124-sor", },
	{ },
};

struct platform_driver tegra_sor_driver = {
	.driver = {
		.name = "tegra-sor",
		.of_match_table = tegra_sor_of_match,
	},
	.probe = tegra_sor_probe,
	.remove = tegra_sor_remove,
};
