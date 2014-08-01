/*
 * arch/arm/mach-tegra/board-venice-panel.c
 *
 * Copyright (c) 2011-2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/ioport.h>
#include <linux/fb.h>
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ktime.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/tegra_dc.h>
#include <dt-bindings/gpio/tegra-gpio.h>

#include "irq.h"
#include "iomap.h"
#include "board.h"
#include "board-panel.h"
#include "common.h"

atomic_t sd_brightness = ATOMIC_INIT(255);
EXPORT_SYMBOL(sd_brightness);

static int power_off_time;
static int pwm_to_bl_on;
static int bl_off_to_pwm;
static bool bl_enabled;
static ktime_t last_power_off;

struct platform_device * __init venice_host1x_init(void)
{
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;

	node = of_find_compatible_node(NULL, NULL, "nvidia,tegra124-host1x");
	if (!node)
		return NULL;

	pdev = of_find_device_by_node(node);
	of_node_put(node);
	return pdev;
}

static struct regulator *avdd_lcd_3v3;
static struct regulator *vdd_lcd_bl;
static struct regulator *lcd_bl_en;
static bool edp_probed, edp_enabled;

static int venice_edp_regulator_probe(struct device *dev)
{
	int ret;

	avdd_lcd_3v3 = devm_regulator_get_optional(dev, "avdd-lcd");
	if (IS_ERR(avdd_lcd_3v3)) {
		dev_err(dev, "edp: avdd_lcd regulator get failed: %ld\n",
				PTR_ERR(avdd_lcd_3v3));
		return PTR_ERR(avdd_lcd_3v3);
	}

	vdd_lcd_bl = devm_regulator_get_optional(dev, "vdd-lcd-bl");
	if (IS_ERR(vdd_lcd_bl)) {
		dev_err(dev, "edp: vdd_bl regulator get failed: %ld\n",
				PTR_ERR(vdd_lcd_bl));
		return PTR_ERR(vdd_lcd_bl);
	}

	lcd_bl_en = devm_regulator_get_optional(dev, "lcd-bl-en");
	if (IS_ERR(lcd_bl_en)) {
		dev_err(dev, "edp: bl_en regulator get failed: %ld\n",
				PTR_ERR(lcd_bl_en));
		ret = PTR_ERR(lcd_bl_en);
		lcd_bl_en = NULL;
		return ret;
	}

	edp_probed = true;

	return 0;
}

static int venice_edp_enable(struct device *dev)
{
	int err = 0;
	int diff = ktime_to_ms(ktime_sub(ktime_get_real(), last_power_off));

	if (edp_enabled)
		return 0;

	if (ktime_to_ms(last_power_off) > 0 && diff < power_off_time)
		msleep(power_off_time - diff);

	err = regulator_enable(avdd_lcd_3v3);
	if (err) {
		dev_err(dev, "Enable regulator avdd_lcd failed: %d\n", err);
		goto fail;
	}
	err = regulator_enable(vdd_lcd_bl);
	if (err) {
		dev_err(dev, "Enable regulator vdd_lcd_bl failed: %d\n", err);
		regulator_disable(avdd_lcd_3v3);
		goto fail;
	}

	edp_enabled = 1;
	return 0;
fail:
	return err;
}

static int venice_edp_disable(void)
{
	if (!edp_enabled)
		return 1;

	if (vdd_lcd_bl)
		regulator_disable(vdd_lcd_bl);

	if (avdd_lcd_3v3)
		regulator_disable(avdd_lcd_3v3);

	last_power_off = ktime_get_real();

	edp_enabled = 0;
	return 0;
}

static int venice_edp_prepoweroff(void)
{
	int ret;

	if (lcd_bl_en && bl_enabled) {
		ret = regulator_disable(lcd_bl_en);
		if (ret)
			pr_err("Disable lcd_bl_en failed: %d\n", ret);
		else
			bl_enabled = false;

		msleep(bl_off_to_pwm);
	}

	return ret;
}

static struct tegra_dc_sd_settings venice_sd_settings = {
	.enable = 0, /* disabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 5,
	.use_vid_luma = false,
	.phase_in_adjustments = 0,
	.k_limit_enable = true,
	.k_limit = 200,
	.sd_window_enable = false,
	.soft_clipping_enable = true,
	/* Low soft clipping threshold to compensate for aggressive k_limit */
	.soft_clipping_threshold = 128,
	.smooth_k_enable = false,
	.smooth_k_incr = 64,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 73, 82},
				{92, 103, 114, 125},
				{138, 150, 164, 178},
				{193, 208, 224, 241},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{255, 255, 255},
				{199, 199, 199},
				{153, 153, 153},
				{116, 116, 116},
				{85, 85, 85},
				{59, 59, 59},
				{36, 36, 36},
				{17, 17, 17},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.use_vpulse2 = true,
	.bl_device_name = "backlight",
};

static struct tegra_dc_out_pin venice_edp_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_DATA_ENABLE,
		.pol	= TEGRA_DC_OUT_PIN_POL_HIGH,
	},
};

static struct tegra_dc_mode venice_mode;
static struct tegra_dp_out venice_dp;

static struct tegra_dc_out venice_disp1_out = {
	.type		 = TEGRA_DC_OUT_DP,
	.flags		 = TEGRA_DC_OUT_CONTINUOUS_MODE,
	.parent_clk	 = "pll_d_out0",
	.align		 = TEGRA_DC_ALIGN_MSB,
	.order		 = TEGRA_DC_ORDER_RED_BLUE,
	.dither		 = TEGRA_DC_TEMPORAL_DITHER,
	.sd_settings	 = &venice_sd_settings,
	.out_pins	 = venice_edp_out_pins,
	.n_out_pins	 = ARRAY_SIZE(venice_edp_out_pins),
	.enable		 = venice_edp_enable,
	.disable	 = venice_edp_disable,
	.prepoweroff	 = venice_edp_prepoweroff,
	.regulator_probe = venice_edp_regulator_probe,
	.dp		 = &venice_dp,
};

static struct tegra_fb_data venice_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

struct tegra_dc_platform_data venice_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &venice_disp1_out,
	.fb		= &venice_disp1_fb_data,
	.emc_clk_rate	= 204000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 1,
#endif
};
EXPORT_SYMBOL(venice_disp1_pdata);

static struct regulator *avdd_hdmi_pex;
static struct regulator *avdd_hdmi_pll;
static struct regulator *vdd_hdmi_5v0;

static int venice_hdmi_regulator_probe(struct device *dev)
{
	struct tegra_dc_platform_data *pdata = dev_get_platdata(dev);
	int gpio_hdmi_hpd;

	if (!edp_probed)
		return -EPROBE_DEFER;

	avdd_hdmi_pex = devm_regulator_get_optional(dev, "avdd-hdmi-pex");
	if (IS_ERR(avdd_hdmi_pex)) {
		dev_err(dev, "HDMI: avdd-hdmi-pex regulator get failed: %ld\n",
				PTR_ERR(avdd_hdmi_pex));
		return PTR_ERR(avdd_hdmi_pex);
	}

	avdd_hdmi_pll = devm_regulator_get_optional(dev, "avdd-hdmi-pll");
	if (IS_ERR(avdd_hdmi_pll)) {
		dev_err(dev, "HDMI: avdd-hdmi-pll regulator get failed: %ld\n",
				PTR_ERR(avdd_hdmi_pll));
		return PTR_ERR(avdd_hdmi_pll);
	}

	vdd_hdmi_5v0 = devm_regulator_get_optional(dev, "vdd-hdmi-5v0");
	if (IS_ERR(vdd_hdmi_5v0)) {
		dev_err(dev, "HDMI: vdd-hdmi-5v0 regulator get failed: %ld\n",
				PTR_ERR(vdd_hdmi_5v0));
		return PTR_ERR(vdd_hdmi_5v0);
	}

	gpio_hdmi_hpd = of_get_named_gpio(dev->of_node, "hdmi-hpd-gpios", 0);
	if (!gpio_is_valid(gpio_hdmi_hpd)) {
		dev_err(dev, "HDMI: hdmi-hpd gpio get failed: %d\n",
				gpio_hdmi_hpd);
		return gpio_hdmi_hpd;
	}
	pdata->default_out->hotplug_gpio = gpio_hdmi_hpd;

	return 0;
}

static int venice_hdmi_enable(struct device *dev)
{
	int ret;

	ret = regulator_enable(avdd_hdmi_pex);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator pex\n");
		return ret;
	}
	ret = regulator_enable(avdd_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator pll\n");
		regulator_disable(avdd_hdmi_pex);
		return ret;
	}

	return 0;
}

static int venice_hdmi_disable(void)
{
	if (avdd_hdmi_pex)
		regulator_disable(avdd_hdmi_pex);

	if (avdd_hdmi_pll)
		regulator_disable(avdd_hdmi_pll);

	return 0;
}

static int venice_hdmi_hotplug_init(struct device *dev)
{
	int ret;

	ret = regulator_enable(vdd_hdmi_5v0);
	if (ret < 0)
		pr_err("hdmi: couldn't enable regulator vdd\n");

	return ret;
}

static int venice_hdmi_postsuspend(void)
{
	if (vdd_hdmi_5v0)
		regulator_disable(vdd_hdmi_5v0);

	return 0;
}

struct tmds_config nyan_tmds_config[] = {
	{ /* 480p/576p / 25.2MHz/27MHz modes */
	.pclk = 27000000,
	.pll0 = 0x01003010,
	.pll1 = 0x00301B00,
	.pe_current = 0x00000000,
	.drive_current = 0x1F1F1F1F,
	.peak_current = 0x03030303,
	.bg_vref_level = 4,
	},
	{ /* 720p / 74.25MHz modes */
	.pclk = 74250000,
	.pll0 = 0x01003110,
	.pll1 = 0x00301500,
	.pe_current = 0x00000000,
	.drive_current = 0x2C2C2C2C,
	.peak_current = 0x07070707,
	.bg_vref_level = 4,
	},
	{ /* 1080p / 148.5MHz modes */
	.pclk = 148500000,
	.pll0 = 0x01003310,
	.pll1 = 0x00301500,
	.pe_current = 0x00000000,
	.drive_current = 0x33333333,
	.peak_current = 0x0C0C0C0C,
	.bg_vref_level = 4,
	},
	{
	.pclk = INT_MAX,
	.pll0 = 0x01003F10,
	.pll1 = 0x00300F00,
	.pe_current = 0x00000000,
	.drive_current = 0x37373737, /* lane3 needs a slightly lower current */
	.peak_current = 0x17171717,
	.bg_vref_level = 6,
	},
};

struct tegra_hdmi_out nyan_hdmi_out = {
	.tmds_config = nyan_tmds_config,
	.n_tmds_config = ARRAY_SIZE(nyan_tmds_config),
};

static struct tegra_dc_out venice_disp2_out = {
	.type		 = TEGRA_DC_OUT_HDMI,
	.flags		 = TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	 = "pll_d2_out0",
	.dcc_bus	 = 3,
	.max_pixclock	 = KHZ2PICOS(297000),
	.align		 = TEGRA_DC_ALIGN_MSB,
	.order		 = TEGRA_DC_ORDER_RED_BLUE,
	.enable		 = venice_hdmi_enable,
	.disable	 = venice_hdmi_disable,
	.hotplug_init	 = venice_hdmi_hotplug_init,
	.regulator_probe = venice_hdmi_regulator_probe,
	.postsuspend	 = venice_hdmi_postsuspend,
	.hdmi_out	 = &nyan_hdmi_out,
};

static struct tegra_fb_data venice_disp2_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 720,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

struct tegra_dc_platform_data venice_disp2_pdata = {
	.default_out	= &venice_disp2_out,
	.fb		= &venice_disp2_fb_data,
	.emc_clk_rate	= 300000000,
};
EXPORT_SYMBOL(venice_disp2_pdata);

static struct platform_device *disp1_device;

static int venice_bl_init(struct device *dev)
{
	struct platform_pwm_backlight_data *data = dev_get_platdata(dev);
	struct platform_pwm_backlight_data defdata;
	int ret;

	ret = pwm_backlight_parse_dt(dev, &defdata);
	if (ret < 0) {
		dev_err(dev, "failed to find platform data\n");
		return ret;
	}

	data->max_brightness = defdata.max_brightness;
	data->dft_brightness = defdata.dft_brightness;
	data->levels = defdata.levels;

	return 0;
}

static int venice_bl_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);
	int ret;

	ret = (brightness * cur_sd_brightness) / 255;

	if (lcd_bl_en) {
		if (!brightness && bl_enabled) {
			ret = regulator_disable(lcd_bl_en);
			if (ret)
				pr_err("Disable lcd_bl_en failed: %d\n", ret);
			else
				bl_enabled = false;
			msleep(bl_off_to_pwm);
		}
	}
	/* SD brightness is a percentage, max SD brightness is 255. */
	return ret;
}

static void venice_bl_notify_after(struct device *unused, int brightness)
{
	int ret;

	if (lcd_bl_en) {
		if (brightness && !bl_enabled) {
			msleep(pwm_to_bl_on);
			ret = regulator_enable(lcd_bl_en);
			if (ret)
				pr_err("Enable lcd_bl_en failed: %d\n", ret);
			else
				bl_enabled = true;
		}
	}
}

static int venice_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp1_device->dev;
}

struct platform_pwm_backlight_data venice_bl_data = {
	.init		= venice_bl_init,
	.notify         = venice_bl_notify,
	.notify_after	= venice_bl_notify_after,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb       = venice_check_fb,
	.enable_gpio	= -1,
};
EXPORT_SYMBOL(venice_bl_data);

static int venice_panel_mode_init(struct platform_device *dcs)
{
	struct device *dev = &dcs->dev;
	u32 val;
	int ret;

	/* Get mode information from dc0 */
	ret = of_property_read_u32(dev->of_node, "depth", &val);
	if (ret < 0) {
		dev_err(dev, "Could not find depth property\n");
		return ret;
	}
	venice_disp1_out.depth = val;

	ret = of_property_read_u32(dev->of_node, "width", &val);
	if (ret < 0) {
		dev_err(dev, "Could not find width property\n");
		return ret;
	}
	venice_disp1_out.width = val;

	ret = of_property_read_u32(dev->of_node, "height", &val);
	if (ret < 0) {
		dev_err(dev, "Could not find height property\n");
		return ret;
	}
	venice_disp1_out.height = val;

	ret = of_property_read_u32(dev->of_node, "power-off-time", &val);
	if (ret < 0)
		power_off_time = 0;
	else
		power_off_time = val;

	ret = of_property_read_u32(dev->of_node, "pwm-to-bl-on", &val);
	if (ret < 0)
		pwm_to_bl_on = 0;
	else
		pwm_to_bl_on = val;

	ret = of_property_read_u32(dev->of_node, "bl-off-to-pwm", &val);
	if (ret < 0)
		bl_off_to_pwm = 0;
	else
		bl_off_to_pwm = val;

	ret = of_property_read_u32(dev->of_node, "drive-current",
		&venice_dp.drive_current);
	if (ret < 0)
		venice_dp.drive_current = 0;

	ret = of_property_read_u32(dev->of_node, "preemphasis",
		&venice_dp.preemphasis);
	if (ret < 0)
		venice_dp.preemphasis = 0;

	/* Optional mode definitions follows */
	ret = of_property_read_u32(dev->of_node, "pclk", &val);
	if (ret == 0) {
		venice_mode.pclk = val;

		ret = of_property_read_u32(dev->of_node, "h-ref-to-sync",
					   &val);
		if (ret < 0) {
			dev_err(dev, "Could not find h-ref-to-sync property\n");
			return ret;
		}
		venice_mode.h_ref_to_sync = val;

		ret = of_property_read_u32(dev->of_node, "v-ref-to-sync",
					   &val);
		if (ret < 0) {
			dev_err(dev, "Could not find v-ref-to-sync property\n");
			return ret;
		}
		venice_mode.v_ref_to_sync = val;

		ret = of_property_read_u32(dev->of_node, "h-sync-width", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find h-sync-width property\n");
			return ret;
		}
		venice_mode.h_sync_width = val;

		ret = of_property_read_u32(dev->of_node, "v-sync-width", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find v-sync-width property\n");
			return ret;
		}
		venice_mode.v_sync_width = val;

		ret = of_property_read_u32(dev->of_node, "h-back-porch", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find h-back-porch property\n");
			return ret;
		}
		venice_mode.h_back_porch = val;

		ret = of_property_read_u32(dev->of_node, "v-back-porch", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find v-back-porch property\n");
			return ret;
		}
		venice_mode.v_back_porch = val;

		ret = of_property_read_u32(dev->of_node, "h-active", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find h-active property\n");
			return ret;
		}
		venice_mode.h_active = val;

		ret = of_property_read_u32(dev->of_node, "v-active", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find v-active property\n");
			return ret;
		}
		venice_mode.v_active = val;

		ret = of_property_read_u32(dev->of_node, "h-front-porch", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find h-front-porch property\n");
			return ret;
		}
		venice_mode.h_front_porch = val;

		ret = of_property_read_u32(dev->of_node, "v-front-porch", &val);
		if (ret < 0) {
			dev_err(dev, "Could not find v-front-porch property\n");
			return ret;
		}
		venice_mode.v_front_porch = val;

		venice_disp1_out.modes = &venice_mode;
		venice_disp1_out.n_modes = 1;

		venice_disp1_fb_data.xres = venice_mode.h_active;
		venice_disp1_fb_data.yres = venice_mode.v_active;
	}
	return 0;
}

void find_dc_dev(struct platform_device **dcs)
{
	struct platform_device *pdev = NULL;

	pdev = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "tegradc.0"));
	dcs[0] = pdev;

	pdev = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "tegradc.1"));
	dcs[1] = pdev;
}
EXPORT_SYMBOL(find_dc_dev);

int __init venice_panel_init(void)
{
	struct platform_device *phost1x = NULL;
	struct platform_device *dc_devs[2];
	int ret;

	phost1x = venice_host1x_init();
	if (!phost1x) {
		pr_err("Could not find host1x device.\n");
		return -EINVAL;
	}
	phost1x->dev.parent = NULL;

	find_dc_dev(dc_devs);
	if (!dc_devs[0] || !dc_devs[1]) {
		pr_err("Could not find dc devices.\n");
		return -EINVAL;
	}
	disp1_device = dc_devs[0];

	ret = venice_panel_mode_init(disp1_device);
	if (ret < 0)
		return ret;

	return 0;
}
