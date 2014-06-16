/*
 * drivers/video/tegra/dc/dc_priv.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (c) 2010-2014, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_DEFS_H
#define __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_DEFS_H
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/nvhost.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/platform_data/tegra_dc.h>
#include <linux/platform_data/tegra_dc_ext.h>
#include <linux/platform_data/tegra_fb.h>
#include <linux/platform_data/tegra_hdmi_audio.h>

#include "dc_reg.h"

/* Pad pitch to 256-byte boundary. */
#define TEGRA_LINEAR_PITCH_ALIGNMENT 256

#define NEED_UPDATE_EMC_ON_EVERY_FRAME (windows_idle_detection_time == 0)

/* 28 bit offset for window clip number */
#define CURSOR_CLIP_SHIFT_BITS(win)	(win << 28)
#define CURSOR_CLIP_GET_WINDOW(reg)	((reg >> 28) & 3)

/* Use runtime check to replace the "ifdef"s. */
static inline int is_tegra114(void)
{
	return of_machine_is_compatible("nvidia,tegra114");
}

static inline int is_tegra124(void)
{
	return of_machine_is_compatible("nvidia,tegra124");
}

static inline u32 ALL_UF_INT(void)
{
#if defined(CONFIG_ARCH_TEGRA_124_SOC)
	if (is_tegra124())
		return WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT | HC_UF_INT |
			WIN_D_UF_INT | WIN_T_UF_INT;
#endif
	return WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT;
}

struct tegra_dc_blend {
	unsigned *z;
	unsigned *flags;
	u8 *alpha;
};

struct tegra_dc;

struct tegra_dc_out_ops {
	/* initialize output.  dc clocks are not on at this point */
	int (*init)(struct tegra_dc *dc);
	/* destroy output.  dc clocks are not on at this point */
	void (*destroy)(struct tegra_dc *dc);
	/* detect connected display.  can sleep.*/
	bool (*detect)(struct tegra_dc *dc);
	/* enable output.  dc clocks are on at this point */
	void (*enable)(struct tegra_dc *dc);
	/* enable dc client.  Panel is enable at this point */
	void (*postpoweron)(struct tegra_dc *dc);
	/* disable output.  dc clocks are on at this point */
	void (*disable)(struct tegra_dc *dc);
	/* dc client is disabled.  dc clocks are on at this point */
	void (*postpoweroff) (struct tegra_dc *dc);
	/* hold output.  keeps dc clocks on. */
	void (*hold)(struct tegra_dc *dc);
	/* release output.  dc clocks may turn off after this. */
	void (*release)(struct tegra_dc *dc);
	/* idle routine of output.  dc clocks may turn off after this. */
	void (*idle)(struct tegra_dc *dc);
	/* suspend output.  dc clocks are on at this point */
	void (*suspend)(struct tegra_dc *dc);
	/* resume output.  dc clocks are on at this point */
	void (*resume)(struct tegra_dc *dc);
	/* mode filter. to provide a list of supported modes*/
	bool (*mode_filter)(const struct tegra_dc *dc,
			struct fb_videomode *mode);
	/* setup pixel clock and parent clock programming */
	long (*setup_clk)(struct tegra_dc *dc, struct clk *clk);
	/* enable output before dc is fully enabled in order to get
	 * info such as panel mode for dc enablement.
	 */
	bool (*early_enable)(struct tegra_dc *dc);
};

struct tegra_dc_shift_clk_div {
	unsigned long mul; /* numerator */
	unsigned long div; /* denominator */
};

struct tegra_fb_info {
	struct tegra_dc_win	*win;
	struct platform_device	*ndev;
	struct fb_info		*info;
	bool			valid;

	struct resource		*fb_mem;

	int			xres;
	int			yres;
	int			curr_xoffset;
	int			curr_yoffset;

	struct fb_videomode	mode;
	phys_addr_t		phys_start;
};

struct tegra_dc {
	struct platform_device		*ndev;
	struct tegra_dc_platform_data	*pdata;

	void __iomem			*base;
	int				irq;

	struct clk			*clk;
	struct clk			*emc_clk;
	unsigned long			bw_kbps; /* bandwidth in KBps */
	unsigned long			new_bw_kbps;
	struct tegra_dc_shift_clk_div	shift_clk_div;

	u32				powergate_id;

	bool				connected;
	bool				enabled;
	bool				suspended;

	struct tegra_dc_out		*out;
	struct tegra_dc_out_ops		*out_ops;
	void				*out_data;

	struct tegra_dc_mode		mode;
	s64				frametime_ns;

	struct tegra_dc_win		*windows;
	struct tegra_dc_blend		blend;
	int				n_windows;
#ifdef CONFIG_TEGRA_DC_CMU
	struct tegra_dc_cmu		cmu;
#endif
	wait_queue_head_t		wq;
	wait_queue_head_t		timestamp_wq;

	struct mutex			lock;
	struct mutex			one_shot_lock;
	struct mutex			one_shot_lp_lock;

	struct resource			*fb_mem;
	struct tegra_fb_info		*fb;
	struct completion		fb_ready;

	struct {
		u32			id;
		u32			min;
		u32			max;
	} *syncpt;
	u32				vblank_syncpt;
	u32				*win_syncpt;

	u32				valid_windows;

	unsigned long			underflow_mask;
	struct work_struct		reset_work;

	struct completion		frame_end_complete;
	struct completion		crc_complete;
	bool				crc_pending;

	struct work_struct		vblank_work;
	long				vblank_ref_count;
	struct work_struct		vpulse2_work;
	long				vpulse2_ref_count;

	struct {
		u64			underflows;
		u64			underflows_a;
		u64			underflows_b;
		u64			underflows_c;
#if defined(CONFIG_ARCH_TEGRA_124_SOC)
		u64			underflows_d;
		u64			underflows_h;
		u64			underflows_t;
#endif
	} stats;

	struct tegra_dc_ext		*ext;

	struct tegra_dc_feature		*feature;
	int				gen1_blend_num;

#ifdef CONFIG_DEBUG_FS
	struct dentry			*debugdir;
#endif
	struct tegra_dc_lut		fb_lut;
	struct delayed_work		underflow_work;
	u32				one_shot_delay_ms;
	struct delayed_work		one_shot_work;
	s64				frame_end_timestamp;
	atomic_t			frame_end_ref;

	bool				mode_dirty;

	struct tegra_edid		*edid;

	struct backlight_device		*bl_device;
};

static inline int get_dc_n_windows(struct tegra_dc *dc)
{
	if (is_tegra124()) {
		if (strcmp(dev_name(&dc->ndev->dev), "tegradc.0") == 0)
			return 4;
		else
			return 3;
	}

	if (is_tegra114())
		return 3;

	pr_err("Get DC windows number: Unsupported Tegra SOC.\n");
	BUG();
	return 0;
}

#endif
