/*
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/platform_data/tegra_thermal.h>
#include <linux/platform_device.h>
#include <linux/tegra-soc.h>

#include "tegra_soctherm.h"

/* Min temp granularity specified as X in 2^X.
 * -1: Hi precision option: 2^-1 = 0.5C
 *  0: Lo precision option: 2^0  = 1.0C
 *  NB: We must use lower precision (0) due to cp_fuse corrections
 */
static const int precision; /* default 0 -> low precision */
#define LOWER_PRECISION_FOR_CONV(val)	((!precision) ? ((val)*2) : (val))
#define LOWER_PRECISION_FOR_TEMP(val)	((!precision) ? ((val)/2) : (val))
#define PRECISION_IS_LOWER()		((!precision))
#define PRECISION_TO_STR()		((!precision) ? "Lo" : "Hi")

#define TS_TSENSE_REGS_SIZE		0x20
#define TS_TSENSE_REG_OFFSET(reg, ts)	((reg) + ((ts) * TS_TSENSE_REGS_SIZE))

#define TS_THERM_LVL_REGS_SIZE		0x20
#define TS_THERM_GRP_REGS_SIZE		0x04
#define TS_THERM_REG_OFFSET(rg, lv, gr)	((rg) + ((lv) * TS_THERM_LVL_REGS_SIZE)\
					+ ((gr) * TS_THERM_GRP_REGS_SIZE))

#define CTL_LVL0_CPU0			0x0
#define CTL_LVL0_CPU0_UP_THRESH_SHIFT	17
#define CTL_LVL0_CPU0_UP_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_DN_THRESH_SHIFT	9
#define CTL_LVL0_CPU0_DN_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_EN_SHIFT		8
#define CTL_LVL0_CPU0_EN_MASK		0x1
#define CTL_LVL0_CPU0_DEV_THROT_LIGHT	0x1
#define CTL_LVL0_CPU0_DEV_THROT_HEAVY	0x2
#define CTL_LVL0_CPU0_CPU_THROT_SHIFT	5
#define CTL_LVL0_CPU0_CPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_GPU_THROT_SHIFT	3
#define CTL_LVL0_CPU0_GPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_STATUS_SHIFT	0
#define CTL_LVL0_CPU0_STATUS_MASK	0x3

#define THERMTRIP			0x80
#define THERMTRIP_ANY_EN_SHIFT		28
#define THERMTRIP_ANY_EN_MASK		0x1
#define THERMTRIP_MEM_EN_SHIFT		27
#define THERMTRIP_MEM_EN_MASK		0x1
#define THERMTRIP_GPU_EN_SHIFT		26
#define THERMTRIP_GPU_EN_MASK		0x1
#define THERMTRIP_CPU_EN_SHIFT		25
#define THERMTRIP_CPU_EN_MASK		0x1
#define THERMTRIP_TSENSE_EN_SHIFT	24
#define THERMTRIP_TSENSE_EN_MASK	0x1
#define THERMTRIP_GPUMEM_THRESH_SHIFT	16
#define THERMTRIP_GPUMEM_THRESH_MASK	0xff
#define THERMTRIP_CPU_THRESH_SHIFT	8
#define THERMTRIP_CPU_THRESH_MASK	0xff
#define THERMTRIP_TSENSE_THRESH_SHIFT	0
#define THERMTRIP_TSENSE_THRESH_MASK	0xff

#define TS_CPU0_CONFIG0				0xc0
#define TS_CPU0_CONFIG0_TALL_SHIFT		8
#define TS_CPU0_CONFIG0_TALL_MASK		0xfffff
#define TS_CPU0_CONFIG0_TCALC_OVER_SHIFT	4
#define TS_CPU0_CONFIG0_TCALC_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_OVER_SHIFT		3
#define TS_CPU0_CONFIG0_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_CPTR_OVER_SHIFT		2
#define TS_CPU0_CONFIG0_CPTR_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_STOP_SHIFT		0
#define TS_CPU0_CONFIG0_STOP_MASK		0x1

#define TS_CPU0_CONFIG1			0xc4
#define TS_CPU0_CONFIG1_EN_SHIFT	31
#define TS_CPU0_CONFIG1_EN_MASK		0x1
#define TS_CPU0_CONFIG1_TIDDQ_SHIFT	15
#define TS_CPU0_CONFIG1_TIDDQ_MASK	0x3f
#define TS_CPU0_CONFIG1_TEN_COUNT_SHIFT	24
#define TS_CPU0_CONFIG1_TEN_COUNT_MASK	0x3f
#define TS_CPU0_CONFIG1_TSAMPLE_SHIFT	0
#define TS_CPU0_CONFIG1_TSAMPLE_MASK	0x3ff

#define TS_CPU0_CONFIG2			0xc8
#define TS_CPU0_CONFIG2_THERM_A_SHIFT	16
#define TS_CPU0_CONFIG2_THERM_A_MASK	0xffff
#define TS_CPU0_CONFIG2_THERM_B_SHIFT	0
#define TS_CPU0_CONFIG2_THERM_B_MASK	0xffff

#define TS_CPU0_STATUS0			0xcc
#define TS_CPU0_STATUS0_VALID_SHIFT	31
#define TS_CPU0_STATUS0_VALID_MASK	0x1
#define TS_CPU0_STATUS0_CAPTURE_SHIFT	0
#define TS_CPU0_STATUS0_CAPTURE_MASK	0xffff

#define TS_CPU0_STATUS1				0xd0
#define TS_CPU0_STATUS1_TEMP_VALID_SHIFT	31
#define TS_CPU0_STATUS1_TEMP_VALID_MASK		0x1
#define TS_CPU0_STATUS1_TEMP_SHIFT		0
#define TS_CPU0_STATUS1_TEMP_MASK		0xffff

#define TS_CPU0_STATUS2			0xd4

#define TS_PDIV				0x1c0
#define TS_PDIV_CPU_SHIFT		12
#define TS_PDIV_CPU_MASK		0xf
#define TS_PDIV_GPU_SHIFT		8
#define TS_PDIV_GPU_MASK		0xf
#define TS_PDIV_MEM_SHIFT		4
#define TS_PDIV_MEM_MASK		0xf
#define TS_PDIV_PLLX_SHIFT		0
#define TS_PDIV_PLLX_MASK		0xf

#define TS_HOTSPOT_OFF			0x1c4
#define TS_HOTSPOT_OFF_CPU_SHIFT	16
#define TS_HOTSPOT_OFF_CPU_MASK		0xff
#define TS_HOTSPOT_OFF_GPU_SHIFT	8
#define TS_HOTSPOT_OFF_GPU_MASK		0xff
#define TS_HOTSPOT_OFF_MEM_SHIFT	0
#define TS_HOTSPOT_OFF_MEM_MASK		0xff

#define TS_TEMP1			0x1c8
#define TS_TEMP1_CPU_TEMP_SHIFT		16
#define TS_TEMP1_CPU_TEMP_MASK		0xffff
#define TS_TEMP1_GPU_TEMP_SHIFT		0
#define TS_TEMP1_GPU_TEMP_MASK		0xffff

#define TS_TEMP2			0x1cc
#define TS_TEMP2_MEM_TEMP_SHIFT		16
#define TS_TEMP2_MEM_TEMP_MASK		0xffff
#define TS_TEMP2_PLLX_TEMP_SHIFT	0
#define TS_TEMP2_PLLX_TEMP_MASK		0xffff

#define INTR_STATUS			0x84
#define INTR_EN				0x88
#define INTR_DIS			0x8c

#define INTR_POS_MD3_SHIFT		31
#define INTR_POS_MD3_MASK		0x1
#define INTR_POS_MU3_SHIFT		30
#define INTR_POS_MU3_MASK		0x1
#define INTR_POS_MD2_SHIFT		29
#define INTR_POS_MD2_MASK		0x1
#define INTR_POS_MU2_SHIFT		28
#define INTR_POS_MU2_MASK		0x1
#define INTR_POS_MD1_SHIFT		27
#define INTR_POS_MD1_MASK		0x1
#define INTR_POS_MU1_SHIFT		26
#define INTR_POS_MU1_MASK		0x1
#define INTR_POS_MD0_SHIFT		25
#define INTR_POS_MD0_MASK		0x1
#define INTR_POS_MU0_SHIFT		24
#define INTR_POS_MU0_MASK		0x1
#define INTR_POS_GD3_SHIFT		23
#define INTR_POS_GD3_MASK		0x1
#define INTR_POS_GU3_SHIFT		22
#define INTR_POS_GU3_MASK		0x1
#define INTR_POS_GD2_SHIFT		21
#define INTR_POS_GD2_MASK		0x1
#define INTR_POS_GU2_SHIFT		20
#define INTR_POS_GU2_MASK		0x1
#define INTR_POS_GD1_SHIFT		19
#define INTR_POS_GD1_MASK		0x1
#define INTR_POS_GU1_SHIFT		18
#define INTR_POS_GU1_MASK		0x1
#define INTR_POS_GD0_SHIFT		17
#define INTR_POS_GD0_MASK		0x1
#define INTR_POS_GU0_SHIFT		16
#define INTR_POS_GU0_MASK		0x1
#define INTR_POS_CD3_SHIFT		15
#define INTR_POS_CD3_MASK		0x1
#define INTR_POS_CU3_SHIFT		14
#define INTR_POS_CU3_MASK		0x1
#define INTR_POS_CD2_SHIFT		13
#define INTR_POS_CD2_MASK		0x1
#define INTR_POS_CU2_SHIFT		12
#define INTR_POS_CU2_MASK		0x1
#define INTR_POS_CD1_SHIFT		11
#define INTR_POS_CD1_MASK		0x1
#define INTR_POS_CU1_SHIFT		10
#define INTR_POS_CU1_MASK		0x1
#define INTR_POS_CD0_SHIFT		9
#define INTR_POS_CD0_MASK		0x1
#define INTR_POS_CU0_SHIFT		8
#define INTR_POS_CU0_MASK		0x1
#define INTR_POS_PD3_SHIFT		7
#define INTR_POS_PD3_MASK		0x1
#define INTR_POS_PU3_SHIFT		6
#define INTR_POS_PU3_MASK		0x1
#define INTR_POS_PD2_SHIFT		5
#define INTR_POS_PD2_MASK		0x1
#define INTR_POS_PU2_SHIFT		4
#define INTR_POS_PU2_MASK		0x1
#define INTR_POS_PD1_SHIFT		3
#define INTR_POS_PD1_MASK		0x1
#define INTR_POS_PU1_SHIFT		2
#define INTR_POS_PU1_MASK		0x1
#define INTR_POS_PD0_SHIFT		1
#define INTR_POS_PD0_MASK		0x1
#define INTR_POS_PU0_SHIFT		0
#define INTR_POS_PU0_MASK		0x1


#define UP_STATS_L0		0x10
#define DN_STATS_L0		0x14

#define STATS_CTL		0x94
#define STATS_CTL_CLR_DN	0x8
#define STATS_CTL_EN_DN		0x4
#define STATS_CTL_CLR_UP	0x2
#define STATS_CTL_EN_UP		0x1

#define THROT_GLOBAL_CFG	0x400

#define CPU_PSKIP_STATUS			0x418
#define CPU_PSKIP_STATUS_M_SHIFT		12
#define CPU_PSKIP_STATUS_M_MASK			0xff
#define CPU_PSKIP_STATUS_N_SHIFT		4
#define CPU_PSKIP_STATUS_N_MASK			0xff
#define CPU_PSKIP_STATUS_ENABLED_SHIFT		0
#define CPU_PSKIP_STATUS_ENABLED_MASK		0x1

#define THROT_PRIORITY_LOCK			0x424
#define THROT_PRIORITY_LOCK_PRIORITY_SHIFT	0
#define THROT_PRIORITY_LOCK_PRIORITY_MASK	0xff

#define THROT_STATUS				0x428
#define THROT_STATUS_BREACH_SHIFT		12
#define THROT_STATUS_BREACH_MASK		0x1
#define THROT_STATUS_STATE_SHIFT		4
#define THROT_STATUS_STATE_MASK			0xff
#define THROT_STATUS_ENABLED_SHIFT		0
#define THROT_STATUS_ENABLED_MASK		0x1

#define THROT_PSKIP_CTRL_LITE_CPU		0x430
#define THROT_PSKIP_CTRL_ENABLE_SHIFT		31
#define THROT_PSKIP_CTRL_ENABLE_MASK		0x1
#define THROT_PSKIP_CTRL_DIVIDEND_SHIFT	8
#define THROT_PSKIP_CTRL_DIVIDEND_MASK		0xff
#define THROT_PSKIP_CTRL_DIVISOR_SHIFT		0
#define THROT_PSKIP_CTRL_DIVISOR_MASK		0xff

#define THROT_PSKIP_RAMP_LITE_CPU		0x434
#define THROT_PSKIP_RAMP_DURATION_SHIFT		8
#define THROT_PSKIP_RAMP_DURATION_MASK		0xffff
#define THROT_PSKIP_RAMP_STEP_SHIFT		0
#define THROT_PSKIP_RAMP_STEP_MASK		0xff

#define THROT_LITE_PRIORITY			0x444
#define THROT_LITE_PRIORITY_PRIORITY_SHIFT	0
#define THROT_LITE_PRIORITY_PRIORITY_MASK	0xff

#define THROT_OFFSET				0x30

#define FUSE_BASE_CP_SHIFT	0
#define FUSE_BASE_CP_MASK	0x3ff
#define FUSE_BASE_FT_SHIFT	16
#define FUSE_BASE_FT_MASK	0x7ff
#define FUSE_SHIFT_CP_SHIFT	10
#define FUSE_SHIFT_CP_MASK	0x3f
#define FUSE_SHIFT_CP_BITS	6
#define FUSE_SHIFT_FT_SHIFT	27
#define FUSE_SHIFT_FT_MASK	0x1f
#define FUSE_SHIFT_FT_BITS	5

#define FUSE_TSENSOR_CALIB_FT_SHIFT	13
#define FUSE_TSENSOR_CALIB_FT_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_CP_SHIFT	0
#define FUSE_TSENSOR_CALIB_CP_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_BITS		13

/* car register offsets needed for enabling HW throttling */
#define CAR_SUPER_CCLKG_DIVIDER		0x36c
#define CDIVG_USE_THERM_CONTROLS_SHIFT	30
#define CDIVG_USE_THERM_CONTROLS_MASK	0x1

/* pmc register offsets needed for powering off PMU */
#define PMC_SCRATCH_WRITE_SHIFT			2
#define PMC_SCRATCH_WRITE_MASK			0x1
#define PMC_ENABLE_RST_SHIFT			1
#define PMC_ENABLE_RST_MASK			0x1
#define PMC_SENSOR_CTRL				0x1B0
#define PMC_SCRATCH54				0x258
#define PMC_SCRATCH55				0x25C

/* scratch54 register bit fields */
#define PMU_OFF_DATA_SHIFT			8
#define PMU_OFF_DATA_MASK			0xff
#define PMU_OFF_ADDR_SHIFT			0
#define PMU_OFF_ADDR_MASK			0xff

/* scratch55 register bit fields */
#define RESET_TEGRA_SHIFT			31
#define RESET_TEGRA_MASK			0x1
#define CONTROLLER_TYPE_SHIFT			30
#define CONTROLLER_TYPE_MASK			0x1
#define I2C_CONTROLLER_ID_SHIFT			27
#define I2C_CONTROLLER_ID_MASK			0x7
#define PINMUX_SHIFT				24
#define PINMUX_MASK				0x7
#define CHECKSUM_SHIFT				16
#define CHECKSUM_MASK				0xff
#define PMU_16BIT_SUPPORT_SHIFT			15
#define PMU_16BIT_SUPPORT_MASK			0x1
#define PMU_I2C_ADDRESS_SHIFT			0
#define PMU_I2C_ADDRESS_MASK			0x7f

#define THROT_PSKIP_CTRL(throt, dev)		(THROT_PSKIP_CTRL_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define THROT_PSKIP_RAMP(throt, dev)		(THROT_PSKIP_RAMP_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))

#define REG_SET(r, _name, val)	(((r) & ~(_name##_MASK << _name##_SHIFT)) | \
				 (((val) & _name##_MASK) << _name##_SHIFT))
#define REG_GET_BIT(r, _name)	((r) & (_name##_MASK << _name##_SHIFT))
#define REG_GET(r, _name)	(REG_GET_BIT(r, _name) >> _name##_SHIFT)
#define MAKE_SIGNED32(val, nb)	((s32)(val) << (32 - (nb)) >> (32 - (nb)))

static DEFINE_MUTEX(soctherm_suspend_resume_lock);

static int soctherm_suspend(struct device *dev);
static int soctherm_resume(struct device *dev);

static struct soctherm_platform_data *plat_data;

static inline void soctherm_writel(u32 value, u32 reg)
{
	__raw_writel(value, (plat_data->soctherm_base + reg));
}

static inline u32 soctherm_readl(u32 reg)
{
	return __raw_readl(plat_data->soctherm_base + reg);
}

static inline void pmc_writel(u32 value, u32 reg)
{
	__raw_writel(value, (plat_data->pmc_base + reg));
}

static inline u32 pmc_readl(u32 reg)
{
	return __raw_readl(plat_data->pmc_base + reg);
}

static inline void clk_reset_writel(u32 value, u32 reg)
{
	__raw_writel(value, (plat_data->clk_reset_base + reg));
}

static inline u32 clk_reset_readl(u32 reg)
{
	return __raw_readl(plat_data->clk_reset_base + reg);
}

/* Convert raw count read from TSOSC to temperature, same as HW arithmetic */
static inline long temp_convert(int cap, int therm_a, int therm_b)
{
	cap *= therm_a;
	cap >>= 10;
	cap += (therm_b << 3);
	cap *= LOWER_PRECISION_FOR_CONV(500);
	cap /= 8;
	return cap;
}

#ifdef CONFIG_THERMAL
static struct thermal_zone_device *thz[THERM_SIZE];
#endif

static u32 fuse_calib_base_cp;
static u32 fuse_calib_base_ft;
static s32 actual_temp_cp;
static s32 actual_temp_ft;

static const char *const therm_names[] = {
	[THERM_CPU] = "CPU",
	[THERM_MEM] = "MEM",
	[THERM_GPU] = "GPU",
	[THERM_PLL] = "PLL",
};

static const char *const sensor_names[] = {
	[TSENSE_CPU0] = "cpu0",
	[TSENSE_CPU1] = "cpu1",
	[TSENSE_CPU2] = "cpu2",
	[TSENSE_CPU3] = "cpu3",
	[TSENSE_MEM0] = "mem0",
	[TSENSE_MEM1] = "mem1",
	[TSENSE_GPU]  = "gpu0",
	[TSENSE_PLLX] = "pllx",
};

static const int sensor2tsensorcalib[] = {
	[TSENSE_CPU0] = 0,
	[TSENSE_CPU1] = 1,
	[TSENSE_CPU2] = 2,
	[TSENSE_CPU3] = 3,
	[TSENSE_MEM0] = 5,
	[TSENSE_MEM1] = 6,
	[TSENSE_GPU]  = 4,
	[TSENSE_PLLX] = 7,
};

static const int tsensor2therm_map[] = {
	[TSENSE_CPU0] = THERM_CPU,
	[TSENSE_CPU1] = THERM_CPU,
	[TSENSE_CPU2] = THERM_CPU,
	[TSENSE_CPU3] = THERM_CPU,
	[TSENSE_GPU]  = THERM_GPU,
	[TSENSE_MEM0] = THERM_MEM,
	[TSENSE_MEM1] = THERM_MEM,
	[TSENSE_PLLX] = THERM_PLL,
};

static const enum soctherm_throttle_dev_id therm2dev[] = {
	[THERM_CPU] = THROTTLE_DEV_CPU,
	[THERM_MEM] = THROTTLE_DEV_NONE,
	[THERM_GPU] = THROTTLE_DEV_GPU,
	[THERM_PLL] = THROTTLE_DEV_NONE,
};

static int sensor2therm_a[TSENSE_SIZE];
static int sensor2therm_b[TSENSE_SIZE];

static const struct soctherm_throttle_dev throttle_defaults[] = {
	[THROTTLE_LIGHT] = {
		.dividend = 229,	/* 20% throttling */
		.divisor  = 255,
		.duration = 0xff,
		.step     = 0xf,
	},
	[THROTTLE_HEAVY] = {
		.dividend = 51,		/* 80% throttling */
		.divisor  = 255,
		.duration = 0xff,
		.step     = 0xf,
	},
};

static inline bool is_thermal_dev_throttle_enabled(
		const struct soctherm_platform_data *pdata,
		size_t throttle_id,
		enum soctherm_therm_id therm_id)
{
	size_t idx;

	if ((throttle_id >= ARRAY_SIZE(pdata->throttle)) ||
	    (therm_id >= ARRAY_SIZE(therm2dev)))
		return false;

	idx = therm2dev[therm_id];
	if (idx == THROTTLE_DEV_NONE)
		return false;

	return pdata->throttle[throttle_id].devs[idx].enable;
}

static inline s64 div64_s64_precise(s64 a, s32 b)
{
	s64 r, al;

	/* scale up for increased precision in division */
	al = a << 16;

	r = div64_s64((al * 2) + 1, 2 * b);
	return r >> 16;
}

static inline long temp_translate(int readback)
{
	int abs = readback >> 8;
	int lsb = (readback & 0x80) >> 7;
	int sign = readback & 0x01 ? -1 : 1;

	return (abs * LOWER_PRECISION_FOR_CONV(1000) +
		lsb * LOWER_PRECISION_FOR_CONV(500)) * sign;
}

#ifdef CONFIG_THERMAL
static inline void prog_hw_shutdown(struct thermal_trip_info *trip_state,
				    int therm)
{
	int trip_temp;
	u32 r;

	trip_temp = LOWER_PRECISION_FOR_TEMP(trip_state->trip_temp / 1000);

	r = soctherm_readl(THERMTRIP);
	if (therm == THERM_CPU) {
		r = REG_SET(r, THERMTRIP_CPU_EN, 1);
		r = REG_SET(r, THERMTRIP_CPU_THRESH, trip_temp);
	} else if (therm == THERM_GPU) {
		r = REG_SET(r, THERMTRIP_GPU_EN, 1);
		r = REG_SET(r, THERMTRIP_GPUMEM_THRESH, trip_temp);
	} else if (therm == THERM_PLL) {
		r = REG_SET(r, THERMTRIP_TSENSE_EN, 1);
		r = REG_SET(r, THERMTRIP_TSENSE_THRESH, trip_temp);
	} else if (therm == THERM_MEM) {
		r = REG_SET(r, THERMTRIP_MEM_EN, 1);
		r = REG_SET(r, THERMTRIP_GPUMEM_THRESH, trip_temp);
	}
	r = REG_SET(r, THERMTRIP_ANY_EN, 0);
	soctherm_writel(r, THERMTRIP);
}

static inline void prog_hw_threshold(struct thermal_trip_info *trip_state,
				     int therm, int throt)
{
	int trip_temp;
	u32 r, reg_off;

	trip_temp = LOWER_PRECISION_FOR_TEMP(trip_state->trip_temp / 1000);

	/* Hardcode LITE on level-1 and HEAVY on level-2 */
	reg_off = TS_THERM_REG_OFFSET(CTL_LVL0_CPU0, throt + 1, therm);

	r = soctherm_readl(reg_off);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, trip_temp);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, trip_temp);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	r = REG_SET(r, CTL_LVL0_CPU0_CPU_THROT,
		    throt == THROTTLE_HEAVY ?
		    CTL_LVL0_CPU0_DEV_THROT_HEAVY :
		    CTL_LVL0_CPU0_DEV_THROT_LIGHT);
	r = REG_SET(r, CTL_LVL0_CPU0_GPU_THROT,
		    throt == THROTTLE_HEAVY ?
		    CTL_LVL0_CPU0_DEV_THROT_HEAVY :
		    CTL_LVL0_CPU0_DEV_THROT_LIGHT);

	soctherm_writel(r, reg_off);
}

static int soctherm_set_limits(enum soctherm_therm_id therm,
				long lo_limit, long hi_limit)
{
	u32 r, reg_off;

	reg_off = TS_THERM_REG_OFFSET(CTL_LVL0_CPU0, 0, therm);
	r = soctherm_readl(reg_off);

	lo_limit = LOWER_PRECISION_FOR_TEMP(lo_limit);
	hi_limit = LOWER_PRECISION_FOR_TEMP(hi_limit);

	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, lo_limit);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, hi_limit);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	soctherm_writel(r, reg_off);

	r = soctherm_readl(INTR_EN);
	if (therm == THERM_CPU) {
		r = REG_SET(r, INTR_POS_CD0, 1);
		r = REG_SET(r, INTR_POS_CU0, 1);
	} else if (therm == THERM_GPU) {
		r = REG_SET(r, INTR_POS_GD0, 1);
		r = REG_SET(r, INTR_POS_GU0, 1);
	}
	soctherm_writel(r, INTR_EN);
	return 0;
}

static void soctherm_update_zone(int zn)
{
	const int MAX_HIGH_TEMP = 128000;
	long low_temp = 0, high_temp = MAX_HIGH_TEMP;
	long trip_temp, passive_low_temp = MAX_HIGH_TEMP, zone_temp;
	enum thermal_trip_type trip_type;
	struct thermal_trip_info *trip_state;
	struct thermal_zone_device *cur_thz = thz[zn];
	int count, trips;

	thermal_zone_device_update(cur_thz);

	trips = cur_thz->trips;
	for (count = 0; count < trips; count++) {
		cur_thz->ops->get_trip_type(cur_thz, count, &trip_type);
		if ((trip_type == THERMAL_TRIP_HOT) ||
		    (trip_type == THERMAL_TRIP_CRITICAL))
			continue; /* handled in HW */

		cur_thz->ops->get_trip_temp(cur_thz, count, &trip_temp);

		trip_state = &plat_data->therm[zn].trips[count];
		zone_temp = cur_thz->temperature;

		if (!trip_state->tripped) { /* not tripped? update high */
			if (trip_temp < high_temp)
				high_temp = trip_temp;
		} else { /* tripped? update low */
			if (trip_type != THERMAL_TRIP_PASSIVE) {
				/* get highest ACTIVE */
				if (trip_temp > low_temp)
					low_temp = trip_temp;
			} else {
				/* get lowest PASSIVE */
				if (trip_temp < passive_low_temp)
					passive_low_temp = trip_temp;
			}
		}
	}

	if (passive_low_temp != MAX_HIGH_TEMP)
		low_temp = max(low_temp, passive_low_temp);

	soctherm_set_limits(zn, low_temp/1000, high_temp/1000);
}

static void soctherm_update(void)
{
	int i;

	for (i = 0; i < THERM_SIZE; i++) {
		if (thz[i] && thz[i]->trips)
			soctherm_update_zone(i);
	}
}

static int soctherm_hw_action_get_max_state(struct thermal_cooling_device *cdev,
					    unsigned long *max_state)
{
	struct thermal_trip_info *trip_state = cdev->devdata;

	if (!trip_state)
		return 0;

	*max_state = 1;
	return 0;
}

static int soctherm_hw_action_get_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long *cur_state)
{
	struct thermal_trip_info *trip_state = cdev->devdata;
	u32 pskip_m;

	if (!trip_state)
		return 0;

	*cur_state = 0;
	if (trip_state->trip_type != THERMAL_TRIP_HOT)
		return 0;

	pskip_m = REG_GET(soctherm_readl(CPU_PSKIP_STATUS), CPU_PSKIP_STATUS_M);

	if (strnstr(trip_state->cdev_type, "heavy", THERMAL_NAME_LENGTH) &&
	    pskip_m == throttle_defaults[THROTTLE_LIGHT].dividend)
		return 0;

	if (strnstr(trip_state->cdev_type, "light", THERMAL_NAME_LENGTH) &&
	    pskip_m == throttle_defaults[THROTTLE_HEAVY].dividend)
		return 0;

	*cur_state =
		!!REG_GET(soctherm_readl(THROT_STATUS), THROT_STATUS_STATE);
	return 0;
}

static int soctherm_hw_action_set_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long cur_state)
{
	return 0; /* hw sets this state */
}

static struct thermal_cooling_device_ops soctherm_hw_action_ops = {
	.get_max_state = soctherm_hw_action_get_max_state,
	.get_cur_state = soctherm_hw_action_get_cur_state,
	.set_cur_state = soctherm_hw_action_set_cur_state,
};

static int soctherm_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int i, index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;

	if (index < 0)
		return 0;

	for (i = 0; i < plat_data->therm[index].num_trips; i++) {
		trip_state = &plat_data->therm[index].trips[i];
		if (trip_state->bound)
			continue;
		if (trip_state->cdev_type &&
		    !strncmp(trip_state->cdev_type, cdev->type,
						THERMAL_NAME_LENGTH)) {
			thermal_zone_bind_cooling_device(thz, i, cdev,
							 trip_state->upper,
							 trip_state->lower);
			trip_state->bound = true;
		}
	}

	thermal_zone_device_update(thz);

	return 0;
}

static int soctherm_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int i, index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;

	if (index < 0)
		return 0;

	for (i = 0; i < plat_data->therm[index].num_trips; i++) {
		trip_state = &plat_data->therm[index].trips[i];
		if (!trip_state->bound)
			continue;
		if (trip_state->cdev_type &&
		    !strncmp(trip_state->cdev_type, cdev->type,
						THERMAL_NAME_LENGTH)) {
			thermal_zone_unbind_cooling_device(thz, 0, cdev);
			trip_state->bound = false;
		}
	}

	return 0;
}

static int soctherm_get_temp(struct thermal_zone_device *thz,
					unsigned long *temp)
{
	int index = (int)thz->devdata;
	u32 r, regv, shft, mask;
	enum soctherm_sense i, j;
	int tt, ti;

	if (index < TSENSE_SIZE) { /* 'TSENSE_XXX' thermal zone */
		regv = TS_CPU0_STATUS1;
		shft = TS_CPU0_STATUS1_TEMP_SHIFT;
		mask = TS_CPU0_STATUS1_TEMP_MASK;
		i = index;
		j = index;
	} else {
		index -= TSENSE_SIZE; /* 'THERM_XXX' thermal zone */

		switch (index) {
		case THERM_CPU:
			regv = TS_TEMP1;
			shft = TS_TEMP1_CPU_TEMP_SHIFT;
			mask = TS_TEMP1_CPU_TEMP_MASK;
			i = TSENSE_CPU0;
			j = TSENSE_CPU3;
			break;

		case THERM_GPU:
			regv = TS_TEMP1;
			shft = TS_TEMP1_GPU_TEMP_SHIFT;
			mask = TS_TEMP1_GPU_TEMP_MASK;
			i = TSENSE_GPU;
			j = TSENSE_GPU;
			break;

		case THERM_PLL:
			regv = TS_TEMP2;
			shft = TS_TEMP2_PLLX_TEMP_SHIFT;
			mask = TS_TEMP2_PLLX_TEMP_MASK;
			i = TSENSE_PLLX;
			j = TSENSE_PLLX;
			break;

		case THERM_MEM:
			regv = TS_TEMP2;
			shft = TS_TEMP2_MEM_TEMP_SHIFT;
			mask = TS_TEMP2_MEM_TEMP_MASK;
			i = TSENSE_MEM0;
			j = TSENSE_MEM1;
			break;

		default:
			return 0; /* error really */
		}
	}

	if (plat_data->read_hw_temp) {
		r = soctherm_readl(regv);
		*temp = temp_translate((r & (mask << shft)) >> shft);
	} else {
		for (tt = 0; i <= j; i++) {
			r = soctherm_readl(TS_TSENSE_REG_OFFSET(
						TS_CPU0_STATUS0, i));
			ti = temp_convert(REG_GET(r, TS_CPU0_STATUS0_CAPTURE),
						sensor2therm_a[i],
						sensor2therm_b[i]);
			*temp = tt = max(tt, ti);
		}
	}
	return 0;
}

static int soctherm_get_trip_type(struct thermal_zone_device *thz,
				int trip, enum thermal_trip_type *type)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;

	if (index < 0)
		return -EINVAL;

	trip_state = &plat_data->therm[index].trips[trip];
	*type = trip_state->trip_type;
	return 0;
}

static int soctherm_get_trip_temp(struct thermal_zone_device *thz,
				int trip, unsigned long *temp)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;
	unsigned long trip_temp, zone_temp;

	if (index < 0)
		return -EINVAL;

	trip_state = &plat_data->therm[index].trips[trip];
	trip_temp = trip_state->trip_temp;
	zone_temp = thz->temperature;

	if (zone_temp >= trip_temp) {
		trip_temp -= trip_state->hysteresis;
		trip_state->tripped = true;
	} else if (trip_state->tripped) {
		trip_temp -= trip_state->hysteresis;
		if (zone_temp < trip_temp)
			trip_state->tripped = false;
	}

	*temp = trip_temp;

	return 0;
}

static int soctherm_set_trip_temp(struct thermal_zone_device *thz,
				int trip, unsigned long temp)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;
	long rem;

	if (index < 0)
		return -EINVAL;

	trip_state = &plat_data->therm[index].trips[trip];

	trip_state->trip_temp = temp;

	rem = trip_state->trip_temp % LOWER_PRECISION_FOR_CONV(1000);
	if (rem) {
		pr_warn("soctherm: zone%d/trip_point%d %ld mC rounded down\n",
			index, trip, trip_state->trip_temp);
		trip_state->trip_temp -= rem;
	}

	if (trip_state->trip_type == THERMAL_TRIP_HOT) {
		if (strnstr(trip_state->cdev_type,
			    "heavy", THERMAL_NAME_LENGTH))
			prog_hw_threshold(trip_state, index, THROTTLE_HEAVY);
		else if (strnstr(trip_state->cdev_type,
				"light", THERMAL_NAME_LENGTH))
			prog_hw_threshold(trip_state, index, THROTTLE_LIGHT);
	}

	/* Allow SW to shutdown at 'Critical temperature reached' */
	soctherm_update_zone(index);

	/* Reprogram HW thermtrip */
	if (trip_state->trip_type == THERMAL_TRIP_CRITICAL)
		prog_hw_shutdown(trip_state, index);

	return 0;
}

static int soctherm_get_crit_temp(struct thermal_zone_device *thz,
				  unsigned long *temp)
{
	int i, index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;

	if (index < 0)
		return -EINVAL;

	for (i = 0; i < plat_data->therm[index].num_trips; i++) {
		trip_state = &plat_data->therm[index].trips[i];
		if (trip_state->trip_type != THERMAL_TRIP_CRITICAL)
			continue;
		*temp = trip_state->trip_temp;
		return 0;
	}

	return -EINVAL;
}

static int soctherm_get_trend(struct thermal_zone_device *thz,
				int trip,
				enum thermal_trend *trend)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;
	struct thermal_trip_info *trip_state;
	long trip_temp;

	if (index < 0)
		return -EINVAL;

	trip_state = &plat_data->therm[index].trips[trip];
	thz->ops->get_trip_temp(thz, trip, &trip_temp);

	switch (trip_state->trip_type) {
	case THERMAL_TRIP_ACTIVE:
		/* aggressive active cooling */
		*trend = THERMAL_TREND_RAISING;
		break;
	case THERMAL_TRIP_PASSIVE:
		if (thz->temperature > trip_state->trip_temp)
			*trend = THERMAL_TREND_RAISING;
		else if (thz->temperature < trip_temp)
			*trend = THERMAL_TREND_DROPPING;
		else
			*trend = THERMAL_TREND_STABLE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct thermal_zone_device_ops soctherm_ops = {
	.bind = soctherm_bind,
	.unbind = soctherm_unbind,
	.get_temp = soctherm_get_temp,
	.get_trip_type = soctherm_get_trip_type,
	.get_trip_temp = soctherm_get_trip_temp,
	.set_trip_temp = soctherm_set_trip_temp,
	.get_crit_temp = soctherm_get_crit_temp,
	.get_trend = soctherm_get_trend,
};

static void soctherm_create_cooling_device(struct soctherm_therm *therm,
					   int therm_id,
					   int trip_index)
{
	int k;
	char *type;

	type = therm->trips[trip_index].cdev_type;

	switch (therm->trips[trip_index].trip_type) {
	case THERMAL_TRIP_CRITICAL:
		thermal_cooling_device_register(type,
						&therm->trips[trip_index],
						&soctherm_hw_action_ops);
		break;

	case THERMAL_TRIP_HOT:
		for (k = 0; k < THROTTLE_SIZE; k++) {
			bool enabled;

			enabled = is_thermal_dev_throttle_enabled(plat_data,
								  k,
								  therm_id);
			if (!enabled)
				continue;

			if ((strnstr(type, "heavy", THERMAL_NAME_LENGTH) &&
			     k == THROTTLE_LIGHT) ||
			    (strnstr(type, "light", THERMAL_NAME_LENGTH) &&
			     k == THROTTLE_HEAVY))
				continue;

			thermal_cooling_device_register(
						type,
						&therm->trips[trip_index],
						&soctherm_hw_action_ops);
		}
		break;
	case THERMAL_TRIP_PASSIVE:
	case THERMAL_TRIP_ACTIVE:
	break; /* done elsewhere */
	}
}

static int soctherm_thermal_sys_init(struct platform_device *pdev)
{
	char name[THERMAL_NAME_LENGTH];
	struct soctherm_therm *therm;
	int i, j;

	for (i = 0; i < TSENSE_SIZE; i++) {
		if (!plat_data->sensor_data[i].zone_enable)
			continue;

		snprintf(name, THERMAL_NAME_LENGTH,
			 "%s-tsensor", sensor_names[i]);
		/* Create a thermal zone device for each sensor */
		thermal_zone_device_register(name,
					     0,
					     0,
					     (void *)i,
					     &soctherm_ops,
					     NULL,
					     0,
					     0);
	}

	for (i = 0; i < THERM_SIZE; i++) {
		therm = &plat_data->therm[i];
		if (!therm->zone_enable)
			continue;

		for (j = 0; j < therm->num_trips; j++)
			soctherm_create_cooling_device(therm, i, j);

		snprintf(name, THERMAL_NAME_LENGTH,
			 "%s-therm", therm_names[i]);
		thz[i] = thermal_zone_device_register(
					name,
					therm->num_trips,
					(1 << therm->num_trips) - 1,
					(void *)TSENSE_SIZE + i,
					&soctherm_ops,
					therm->tzp,
					therm->passive_delay,
					0);
	}

	soctherm_update();
	return 0;
}

#else
static inline void soctherm_update_zone(int zn)
{
}
static inline void soctherm_update(void)
{
}
#endif

static irqreturn_t soctherm_thermal_work_func(int irq, void *arg)
{
	struct platform_device *pdev = arg;
	u32 st, ex = 0, cp = 0, gp = 0;

	st = soctherm_readl(INTR_STATUS);

	/* deliberately clear expected interrupts handled in SW */
	cp |= REG_GET_BIT(st, INTR_POS_CD0);
	cp |= REG_GET_BIT(st, INTR_POS_CU0);
	ex |= cp;

	gp |= REG_GET_BIT(st, INTR_POS_GD0);
	gp |= REG_GET_BIT(st, INTR_POS_GU0);
	ex |= gp;

	if (ex) {
		soctherm_writel(ex, INTR_STATUS);
		st &= ~ex;
		if (cp)
			soctherm_update_zone(THERM_CPU);
		if (gp)
			soctherm_update_zone(THERM_GPU);
	}

	/* deliberately ignore expected interrupts NOT handled in SW */
	ex |= REG_GET_BIT(st, INTR_POS_PD0);
	ex |= REG_GET_BIT(st, INTR_POS_PU0);
	ex |= REG_GET_BIT(st, INTR_POS_MD0);
	ex |= REG_GET_BIT(st, INTR_POS_MU0);

	ex |= REG_GET_BIT(st, INTR_POS_CD1);
	ex |= REG_GET_BIT(st, INTR_POS_CU1);
	ex |= REG_GET_BIT(st, INTR_POS_CD2);
	ex |= REG_GET_BIT(st, INTR_POS_CU2);
	ex |= REG_GET_BIT(st, INTR_POS_CD3);
	ex |= REG_GET_BIT(st, INTR_POS_CU3);

	ex |= REG_GET_BIT(st, INTR_POS_GD1);
	ex |= REG_GET_BIT(st, INTR_POS_GU1);
	ex |= REG_GET_BIT(st, INTR_POS_GD2);
	ex |= REG_GET_BIT(st, INTR_POS_GU2);
	ex |= REG_GET_BIT(st, INTR_POS_GD3);
	ex |= REG_GET_BIT(st, INTR_POS_GU3);

	ex |= REG_GET_BIT(st, INTR_POS_PD1);
	ex |= REG_GET_BIT(st, INTR_POS_PU1);
	ex |= REG_GET_BIT(st, INTR_POS_PD2);
	ex |= REG_GET_BIT(st, INTR_POS_PU2);
	ex |= REG_GET_BIT(st, INTR_POS_PD3);
	ex |= REG_GET_BIT(st, INTR_POS_PU3);

	ex |= REG_GET_BIT(st, INTR_POS_MD1);
	ex |= REG_GET_BIT(st, INTR_POS_MU1);
	ex |= REG_GET_BIT(st, INTR_POS_MD2);
	ex |= REG_GET_BIT(st, INTR_POS_MU2);
	ex |= REG_GET_BIT(st, INTR_POS_MD3);
	ex |= REG_GET_BIT(st, INTR_POS_MU3);
	st &= ~ex;

	if (st) {
		/* Whine about any other unexpected INTR bits still set */
		dev_err(&pdev->dev,
			"soctherm: Ignored unexpected INTRs 0x%08x\n", st);
		soctherm_writel(st, INTR_STATUS);
	}

	return IRQ_HANDLED;
}

static irqreturn_t soctherm_thermal_isr(int irq, void *arg)
{
	u32 r;

	r = soctherm_readl(INTR_STATUS);
	soctherm_writel(r, INTR_DIS);

	return IRQ_WAKE_THREAD;
}

static void tegra_soctherm_throttle_program(enum soctherm_throttle_id throttle,
					struct soctherm_throttle *data)
{
	u32 r;
	int i;
	struct soctherm_throttle_dev *dev;

	for (i = 0; i < THROTTLE_DEV_SIZE; i++) {
		dev = &data->devs[i];

		r = soctherm_readl(THROT_PSKIP_CTRL(throttle, i));
		r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
		if (!dev->enable)
			continue;

		r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND,
			dev->dividend ?: throttle_defaults[throttle].dividend);
		r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR,
			dev->divisor ?: throttle_defaults[throttle].divisor);
		soctherm_writel(r, THROT_PSKIP_CTRL(throttle, i));

		r = soctherm_readl(THROT_PSKIP_RAMP(throttle, i));
		r = REG_SET(r, THROT_PSKIP_RAMP_DURATION,
			dev->duration ?: throttle_defaults[throttle].duration);
		r = REG_SET(r, THROT_PSKIP_RAMP_STEP,
			dev->step ?: throttle_defaults[throttle].step);
		soctherm_writel(r, THROT_PSKIP_RAMP(throttle, i));
	}

	r = soctherm_readl(THROT_PRIORITY_LOCK);
	if (r < data->priority) {
		r = REG_SET(0, THROT_PRIORITY_LOCK_PRIORITY, data->priority);
		soctherm_writel(r, THROT_PRIORITY_LOCK);
	}

	r = REG_SET(0, THROT_LITE_PRIORITY_PRIORITY, data->priority);
	soctherm_writel(r, THROT_LITE_PRIORITY + THROT_OFFSET * throttle);

	/* initialize stats collection */
	r = STATS_CTL_CLR_DN | STATS_CTL_EN_DN |
		STATS_CTL_CLR_UP | STATS_CTL_EN_UP;
	soctherm_writel(r, STATS_CTL);
}

static void soctherm_tsense_program(enum soctherm_sense sensor,
						struct soctherm_sensor *data)
{
	u32 r;

	r = REG_SET(0, TS_CPU0_CONFIG0_TALL, data->tall);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, sensor));

	r = REG_SET(0, TS_CPU0_CONFIG1_TIDDQ, data->tiddq);
	r = REG_SET(r, TS_CPU0_CONFIG1_EN, 1);
	r = REG_SET(r, TS_CPU0_CONFIG1_TEN_COUNT, data->ten_count);
	r = REG_SET(r, TS_CPU0_CONFIG1_TSAMPLE, data->tsample - 1);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, sensor));
}

static int soctherm_clk_init(struct platform_device *pdev)
{
	int err = -EINVAL;
	struct clk *clk_pllp = clk_get_sys(NULL, "pll_p");
	struct clk *clk_clkm = clk_get_sys(NULL, "clk_m");

	if (IS_ERR(clk_pllp) || IS_ERR(clk_clkm))
		return -EINVAL;

	plat_data->soctherm_clk = devm_clk_get(&pdev->dev, "soc_therm");
	plat_data->tsensor_clk = devm_clk_get(&pdev->dev, "tsensor");

	if (IS_ERR(plat_data->tsensor_clk) || IS_ERR(plat_data->soctherm_clk))
		goto error;

	err = clk_set_parent(plat_data->soctherm_clk, clk_pllp);
	if (err) {
		dev_err(&pdev->dev,
			"soctherm: failed to set parent of soc_therm: %d\n",
			err);
		goto error;
	}
	err = clk_set_rate(plat_data->soctherm_clk,
			   plat_data->soctherm_clk_rate);
	if (err) {
		dev_err(&pdev->dev,
			"soctherm: failed to set soc_therm rate to %lu: %d\n",
			plat_data->soctherm_clk_rate, err);
		goto error;
	}

	err = clk_set_parent(plat_data->tsensor_clk, clk_clkm);
	if (err) {
		dev_err(&pdev->dev,
			"soctherm: failed to set parent of tegra-tsensor: %d\n",
			err);
		goto error;
	}
	err = clk_set_rate(plat_data->tsensor_clk,
			   plat_data->tsensor_clk_rate);
	if (err) {
		dev_err(&pdev->dev,
			"soctherm: failed to set tegra-tsensor rate to %lu: %d\n",
			plat_data->tsensor_clk_rate, err);
		goto error;
	}

	return err;

error:
	plat_data->soctherm_clk = NULL;
	plat_data->tsensor_clk = NULL;
	return err;
}

static int soctherm_clk_enable(bool enable)
{
	int ret;

	if (plat_data->soctherm_clk == NULL || plat_data->tsensor_clk == NULL)
		return -EINVAL;

	if (enable) {
		ret = clk_prepare_enable(plat_data->soctherm_clk);
		if (ret)
			return ret;
		ret = clk_prepare_enable(plat_data->tsensor_clk);
		if (ret)
			return ret;
	} else {
		clk_disable_unprepare(plat_data->soctherm_clk);
		clk_disable_unprepare(plat_data->tsensor_clk);
	}

	return 0;
}

static void soctherm_fuse_read_calib_base(void)
{
	struct soctherm_fuse_calib_data *calib_data;

	calib_data = plat_data->fuse_calib_data;

	fuse_calib_base_cp = calib_data->fuse_calib_base_cp;
	fuse_calib_base_ft = calib_data->fuse_calib_base_ft;

	/* use HI precision to calculate: use fuse_temp in 0.5C */
	actual_temp_cp = calib_data->actual_temp_cp;
	actual_temp_ft = calib_data->actual_temp_ft;
}

static int soctherm_fuse_read_tsensor(enum soctherm_sense sensor)
{
	u32 r, value;
	s32 calib, delta_sens, delta_temp;
	s16 therm_a, therm_b;
	s32 div, mult, actual_tsensor_ft, actual_tsensor_cp;

	value = plat_data->fuse_calib_data->tsensor_calib[sensor].calib_val;

	/* Extract bits and convert to signed 2's complement */
	calib = REG_GET(value, FUSE_TSENSOR_CALIB_FT);
	calib = MAKE_SIGNED32(calib, FUSE_TSENSOR_CALIB_BITS);
	actual_tsensor_ft = (fuse_calib_base_ft * 32) + calib;

	calib = REG_GET(value, FUSE_TSENSOR_CALIB_CP);
	calib = MAKE_SIGNED32(calib, FUSE_TSENSOR_CALIB_BITS);
	actual_tsensor_cp = (fuse_calib_base_cp * 64) + calib;

	mult = plat_data->sensor_data[sensor].pdiv *
		plat_data->sensor_data[sensor].tsamp_ate;
	div = plat_data->sensor_data[sensor].tsample *
		plat_data->sensor_data[sensor].pdiv_ate;

	delta_sens = actual_tsensor_ft - actual_tsensor_cp;
	delta_temp = actual_temp_ft - actual_temp_cp;

	therm_a = div64_s64_precise((s64)delta_temp * (1LL << 13) * mult,
				    (s64)delta_sens * div);

	therm_b = div64_s64_precise((((s64)actual_tsensor_ft * actual_temp_cp) -
				     ((s64)actual_tsensor_cp * actual_temp_ft)),
				    (s64)delta_sens);

	if (PRECISION_IS_LOWER()) {
		int alpha, beta;

		/* cp_fuse corrections */
		alpha = plat_data->fuse_calib_data->fuse_corr_alpha[sensor];
		beta = plat_data->fuse_calib_data->fuse_corr_beta[sensor];

		therm_a = div64_s64_precise(
				(s64)therm_a * alpha,
				(s64)1000000LL);
		therm_b = div64_s64_precise(
				(s64)therm_b * alpha + beta,
				(s64)1000000LL);
	}

	therm_a = LOWER_PRECISION_FOR_TEMP(therm_a);
	therm_b = LOWER_PRECISION_FOR_TEMP(therm_b);

	sensor2therm_a[sensor] = (s16)therm_a;
	sensor2therm_b[sensor] = (s16)therm_b;

	r = REG_SET(0, TS_CPU0_CONFIG2_THERM_A, therm_a);
	r = REG_SET(r, TS_CPU0_CONFIG2_THERM_B, therm_b);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, sensor));

	return 0;
}

static void soctherm_therm_trip_init(struct soctherm_tsensor_pmu_data *data)
{
	u32 val, checksum;

	if (!data)
		return;

	val = pmc_readl(PMC_SENSOR_CTRL);
	val = REG_SET(val, PMC_SCRATCH_WRITE, 1);
	val = REG_SET(val, PMC_ENABLE_RST, 1);
	pmc_writel(val, PMC_SENSOR_CTRL);

	/* Fill scratch registers to shutdown device on therm TRIP */
	val = REG_SET(0, PMU_OFF_DATA, data->poweroff_reg_data);
	val = REG_SET(val, PMU_OFF_ADDR, data->poweroff_reg_addr);
	pmc_writel(val, PMC_SCRATCH54);

	val = REG_SET(0, RESET_TEGRA, 1);
	val = REG_SET(val, CONTROLLER_TYPE, data->controller_type);
	val = REG_SET(val, I2C_CONTROLLER_ID, data->i2c_controller_id);
	val = REG_SET(val, PINMUX, data->pinmux);
	val = REG_SET(val, PMU_16BIT_SUPPORT, data->pmu_16bit_ops);
	val = REG_SET(val, PMU_I2C_ADDRESS, data->pmu_i2c_addr);

	checksum = data->poweroff_reg_addr +
		data->poweroff_reg_data +
		(val & 0xFF) +
		((val >> 8) & 0xFF) +
		((val >> 24) & 0xFF);
	checksum &= 0xFF;
	checksum = 0x100 - checksum;

	val = REG_SET(val, CHECKSUM, checksum);
	pmc_writel(val, PMC_SCRATCH55);
}

static int soctherm_init_platform_data(struct platform_device *pdev)
{
	struct soctherm_therm *therm;
	struct soctherm_sensor *s;
	int i, j, k;
	long rem;
	u32 r;

	/* initialize default values for unspecified params */
	for (i = 0; i < TSENSE_SIZE; i++) {
		therm = &plat_data->therm[tsensor2therm_map[i]];
		s = &plat_data->sensor_data[i];
		s->sensor_enable = s->zone_enable;
		s->sensor_enable = s->sensor_enable ?: therm->zone_enable;
	}

	/* Pdiv */
	r = soctherm_readl(TS_PDIV);
	r = REG_SET(r, TS_PDIV_CPU, plat_data->sensor_data[TSENSE_CPU0].pdiv);
	r = REG_SET(r, TS_PDIV_GPU, plat_data->sensor_data[TSENSE_GPU].pdiv);
	r = REG_SET(r, TS_PDIV_MEM, plat_data->sensor_data[TSENSE_MEM0].pdiv);
	r = REG_SET(r, TS_PDIV_PLLX, plat_data->sensor_data[TSENSE_PLLX].pdiv);
	soctherm_writel(r, TS_PDIV);

	/* Thermal Sensing programming */
	soctherm_fuse_read_calib_base();
	for (i = 0; i < TSENSE_SIZE; i++) {
		if (plat_data->sensor_data[i].sensor_enable) {
			soctherm_tsense_program(i, &plat_data->sensor_data[i]);
			if (soctherm_fuse_read_tsensor(i) < 0)
				return -EINVAL;
		}
	}

	/* Sanitize therm trips */
	for (i = 0; i < THERM_SIZE; i++) {
		therm = &plat_data->therm[i];
		if (!therm->zone_enable)
			continue;

		for (j = 0; j < therm->num_trips; j++) {
			rem = therm->trips[j].trip_temp %
				LOWER_PRECISION_FOR_CONV(1000);
			if (rem) {
				dev_warn(&pdev->dev,
					 "soctherm: zone%d/trip_point%d %ld mC rounded down\n",
					 i, j, therm->trips[j].trip_temp);
				therm->trips[j].trip_temp -= rem;
			}
		}
	}

	/* Program hotspot offsets per THERM */
	r = REG_SET(0, TS_HOTSPOT_OFF_CPU,
		    plat_data->therm[THERM_CPU].hotspot_offset / 1000);
	r = REG_SET(r, TS_HOTSPOT_OFF_GPU,
		    plat_data->therm[THERM_GPU].hotspot_offset / 1000);
	r = REG_SET(r, TS_HOTSPOT_OFF_MEM,
		    plat_data->therm[THERM_MEM].hotspot_offset / 1000);
	soctherm_writel(r, TS_HOTSPOT_OFF);

	/* Sanitize HW throttle priority */
	for (i = 0; i < THROTTLE_SIZE; i++)
		if (!plat_data->throttle[i].priority)
			plat_data->throttle[i].priority = 0xE + i;
	if (plat_data->throttle[THROTTLE_HEAVY].priority <
	    plat_data->throttle[THROTTLE_LIGHT].priority)
		dev_err(&pdev->dev,
			"ERROR: Priority of HEAVY less than LIGHT\n");

	/* Thermal HW throttle programming */
	for (i = 0; i < THROTTLE_SIZE; i++) {
		/* Setup PSKIP parameters */
		tegra_soctherm_throttle_program(i, &plat_data->throttle[i]);

		/* Setup throttle thresholds per THERM */
		for (j = 0; j < THERM_SIZE; j++) {
			/* Check if PSKIP params are enabled */
			if (!is_thermal_dev_throttle_enabled(plat_data, i, j))
				continue;

			therm = &plat_data->therm[j];
			for (k = 0; k < therm->num_trips; k++)
				if ((therm->trips[k].trip_type ==
				     THERMAL_TRIP_HOT) &&
				    strnstr(therm->trips[k].cdev_type,
					    i == THROTTLE_HEAVY ? "heavy" :
					    "light", THERMAL_NAME_LENGTH))
					break;
			if (k < therm->num_trips)
				prog_hw_threshold(&therm->trips[k], j, i);
		}
	}

	/* Enable PMC to shutdown */
	soctherm_therm_trip_init(plat_data->tshut_pmu_trip_data);

	r = clk_reset_readl(CAR_SUPER_CCLKG_DIVIDER);
	r = REG_SET(r, CDIVG_USE_THERM_CONTROLS, 1);
	clk_reset_writel(r, CAR_SUPER_CCLKG_DIVIDER);

	/* Thermtrip */
	for (i = 0; i < THERM_SIZE; i++) {
		therm = &plat_data->therm[i];
		if (!therm->zone_enable)
			continue;

		for (j = 0; j < therm->num_trips; j++)
			if (therm->trips[j].trip_type == THERMAL_TRIP_CRITICAL)
				prog_hw_shutdown(&therm->trips[j], i);
	}

	return 0;
}

static int soctherm_suspend(struct device *dev)
{
	soctherm_writel((u32)-1, INTR_DIS);
	disable_irq(plat_data->thermal_irq);
	soctherm_clk_enable(false);
	return 0;
}

static int soctherm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	soctherm_clk_enable(true);
	soctherm_init_platform_data(pdev);
	soctherm_update();
	enable_irq(plat_data->thermal_irq);

	return 0;
}

static int soctherm_platform_init(struct platform_device *pdev)
{
	struct soctherm_platform_data *data = platform_get_drvdata(pdev);
	int err;

	if (!data)
		return -EINVAL;
	plat_data = data;

	plat_data->read_hw_temp = true;

	if (soctherm_clk_init(pdev) < 0)
		return -EINVAL;

	if (soctherm_clk_enable(true) < 0)
		return -EINVAL;

	if (soctherm_init_platform_data(pdev) < 0)
		return -EINVAL;

	plat_data->thermal_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (plat_data->thermal_irq <= 0) {
		dev_err(&pdev->dev, "Failed to map thermal IRQ\n");
		return -EINVAL;
	}

	/* enable threaded interrupts */
	err = devm_request_threaded_irq(&pdev->dev, plat_data->thermal_irq,
					soctherm_thermal_isr,
					soctherm_thermal_work_func,
					IRQF_ONESHOT,
					"soctherm_thermal", pdev);
	if (err < 0)
		return -EINVAL;

	return 0;
}


#ifdef CONFIG_DEBUG_FS
static int regs_show(struct seq_file *s, void *data)
{
	u32 r;
	u32 state;
	int tcpu[TSENSE_SIZE];
	int i, level;

	seq_printf(s, "-----TSENSE (precision %s  convert %s)-----\n",
		   PRECISION_TO_STR(), plat_data->read_hw_temp ? "HW" : "SW");
	for (i = 0; i < TSENSE_SIZE; i++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		seq_printf(s, "%s: ", sensor_names[i]);

		seq_printf(s, "En(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TIDDQ);
		seq_printf(s, "tiddq(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TEN_COUNT);
		seq_printf(s, "ten_count(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TSAMPLE);
		seq_printf(s, "tsample(%d) ", state + 1);

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS1, i));
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID);
		seq_printf(s, "Temp(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
		seq_printf(s, "%d) ", tcpu[i] = temp_translate(state));

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS0, i));
		state = REG_GET(r, TS_CPU0_STATUS0_VALID);
		seq_printf(s, "Capture(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
		seq_printf(s, "%d) (Converted-temp(%ld) ", state,
			   temp_convert(state, sensor2therm_a[i],
					sensor2therm_b[i]));

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, i));
		state = REG_GET(r, TS_CPU0_CONFIG0_TALL);
		seq_printf(s, "Tall(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TCALC_OVER);
		seq_printf(s, "Over(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_OVER);
		seq_printf(s, "%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_CPTR_OVER);
		seq_printf(s, "%d) ", state);

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, i));
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_A);
		seq_printf(s, "Therm_A/B(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_B);
		seq_printf(s, "%d)\n", (s16)state);
	}

	r = soctherm_readl(TS_PDIV);
	seq_printf(s, "PDIV: 0x%x\n", r);

	seq_puts(s, "\n");
	seq_puts(s, "-----SOC_THERM-----\n");

	r = soctherm_readl(TS_TEMP1);
	state = REG_GET(r, TS_TEMP1_CPU_TEMP);
	seq_printf(s, "Temperatures: CPU(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP1_GPU_TEMP);
	seq_printf(s, " GPU(%ld) ", temp_translate(state));
	r = soctherm_readl(TS_TEMP2);
	state = REG_GET(r, TS_TEMP2_PLLX_TEMP);
	seq_printf(s, " PLLX(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP2_MEM_TEMP);
	seq_printf(s, " MEM(%ld)\n", temp_translate(state));

	for (i = 0; i < THERM_SIZE; i++) {
		if (i == THERM_MEM)
			continue;
		seq_printf(s, "%s:\n", therm_names[i]);

		for (level = 0; level < 4; level++) {
			r = soctherm_readl(TS_THERM_REG_OFFSET(CTL_LVL0_CPU0,
								level, i));
			state = REG_GET(r, CTL_LVL0_CPU0_UP_THRESH);
			seq_printf(s, "   %d: Up/Dn(%d/", level,
				   LOWER_PRECISION_FOR_CONV(state));
			state = REG_GET(r, CTL_LVL0_CPU0_DN_THRESH);
			seq_printf(s, "%d) ", LOWER_PRECISION_FOR_CONV(state));
			state = REG_GET(r, CTL_LVL0_CPU0_EN);
			seq_printf(s, "En(%d) ", state);
			state = REG_GET(r, CTL_LVL0_CPU0_CPU_THROT);
			seq_puts(s, "Throt-CPU");
			seq_printf(s, "(%s) ", state ?
				state == CTL_LVL0_CPU0_DEV_THROT_LIGHT ? "L" :
				state == CTL_LVL0_CPU0_DEV_THROT_HEAVY ? "H" :
				"H+L" : "none");
			state = REG_GET(r, CTL_LVL0_CPU0_GPU_THROT);
			seq_puts(s, "Throt-GPU");
			seq_printf(s, "(%s) ", state ?
				state == CTL_LVL0_CPU0_DEV_THROT_LIGHT ? "L" :
				state == CTL_LVL0_CPU0_DEV_THROT_HEAVY ? "H" :
				"H+L" : "none");
			state = REG_GET(r, CTL_LVL0_CPU0_STATUS);
			seq_printf(s, "Status(%s)\n",
				   state == 0 ? "LO" :
				   state == 1 ? "in" :
				   state == 2 ? "??" : "HI");
		}
	}

	r = soctherm_readl(STATS_CTL);
	seq_printf(s, "STATS: Up(%s) Dn(%s)\n",
		   r & STATS_CTL_EN_UP ? "En" : "--",
		   r & STATS_CTL_EN_DN ? "En" : "--");
	for (level = 0; level < 4; level++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(UP_STATS_L0, level));
		seq_printf(s, "  Level_%d Up(%d) ", level, r);
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(DN_STATS_L0, level));
		seq_printf(s, "Dn(%d)\n", r);
	}

	r = soctherm_readl(THERMTRIP);
	state = REG_GET(r, THERMTRIP_ANY_EN);
	seq_printf(s, "ThermTRIP ANY En(%d)\n", state);
	state = REG_GET(r, THERMTRIP_CPU_EN);
	seq_printf(s, "     CPU En(%d) ", state);
	state = REG_GET(r, THERMTRIP_CPU_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));
	state = REG_GET(r, THERMTRIP_GPU_EN);
	seq_printf(s, "     GPU En(%d) ", state);
	state = REG_GET(r, THERMTRIP_GPUMEM_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));
	state = REG_GET(r, THERMTRIP_TSENSE_EN);
	seq_printf(s, "    PLLX En(%d) ", state);
	state = REG_GET(r, THERMTRIP_TSENSE_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));

	r = soctherm_readl(THROT_GLOBAL_CFG);
	seq_printf(s, "GLOBAL THROTTLE CONFIG: 0x%x\n", r);

	r = soctherm_readl(THROT_STATUS);
	state = REG_GET(r, THROT_STATUS_BREACH);
	seq_printf(s, "THROT STATUS: breach(%d) ", state);
	state = REG_GET(r, THROT_STATUS_STATE);
	seq_printf(s, "state(%d) ", state);
	state = REG_GET(r, THROT_STATUS_ENABLED);
	seq_printf(s, "enabled(%d)\n", state);

	r = soctherm_readl(CPU_PSKIP_STATUS);
	state = REG_GET(r, CPU_PSKIP_STATUS_M);
	seq_printf(s, "CPU PSKIP: M(%d) ", state);
	state = REG_GET(r, CPU_PSKIP_STATUS_N);
	seq_printf(s, "N(%d) ", state);
	state = REG_GET(r, CPU_PSKIP_STATUS_ENABLED);
	seq_printf(s, "enabled(%d)\n", state);

	r = soctherm_readl(THROT_PSKIP_CTRL(THROTTLE_LIGHT, THROTTLE_DEV_CPU));
	state = REG_GET(r, THROT_PSKIP_CTRL_ENABLE);
	seq_printf(s, "CPU PSKIP LIGHT: enabled(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_CTRL_DIVIDEND);
	seq_printf(s, "dividend(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_CTRL_DIVISOR);
	seq_printf(s, "divisor(%d) ", state);
	r = soctherm_readl(THROT_PSKIP_RAMP(THROTTLE_LIGHT, THROTTLE_DEV_CPU));
	state = REG_GET(r, THROT_PSKIP_RAMP_DURATION);
	seq_printf(s, "duration(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_RAMP_STEP);
	seq_printf(s, "step(%d)\n", state);

	r = soctherm_readl(THROT_PSKIP_CTRL(THROTTLE_HEAVY, THROTTLE_DEV_CPU));
	state = REG_GET(r, THROT_PSKIP_CTRL_ENABLE);
	seq_printf(s, "CPU PSKIP HEAVY: enabled(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_CTRL_DIVIDEND);
	seq_printf(s, "dividend(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_CTRL_DIVISOR);
	seq_printf(s, "divisor(%d) ", state);
	r = soctherm_readl(THROT_PSKIP_RAMP(THROTTLE_HEAVY, THROTTLE_DEV_CPU));
	state = REG_GET(r, THROT_PSKIP_RAMP_DURATION);
	seq_printf(s, "duration(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_RAMP_STEP);
	seq_printf(s, "step(%d)\n", state);
	return 0;
}

static int temp_log_show(struct seq_file *s, void *data)
{
	u32 r, state;
	int i;
	u64 ts;
	u_long ns;

	ts = cpu_clock(0);
	ns = do_div(ts, 1000000000);
	seq_printf(s, "%6lu.%06lu", (u_long) ts, ns / 1000);

	for (i = 0; i < TSENSE_SIZE; i++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(
					TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(
					TS_CPU0_STATUS1, i));
		if (!REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID)) {
			seq_puts(s, "\tINVALID");
			continue;
		}

		if (plat_data->read_hw_temp) {
			state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
			seq_printf(s, "\t%ld", temp_translate(state));
		} else {
			r = soctherm_readl(TS_TSENSE_REG_OFFSET(
						TS_CPU0_STATUS0, i));
			state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
			seq_printf(s, "\t%ld",
				   temp_convert(state,
						sensor2therm_a[i],
						sensor2therm_b[i]));
		}
	}
	seq_puts(s, "\n");

	return 0;
}

static int regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, regs_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int convert_get(void *data, u64 *val)
{
	*val = !plat_data->read_hw_temp;
	return 0;
}
static int convert_set(void *data, u64 val)
{
	plat_data->read_hw_temp = !val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(convert_fops, convert_get, convert_set, "%llu\n");

static int temp_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, temp_log_show, inode->i_private);
}
static const struct file_operations temp_log_fops = {
	.open		= temp_log_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int soctherm_debug_init(struct platform_device *pdev)
{
	struct dentry *tegra_soctherm_root;

	tegra_soctherm_root = debugfs_create_dir("tegra_soctherm", NULL);
	debugfs_create_file("regs", 0644, tegra_soctherm_root,
			    NULL, &regs_fops);
	debugfs_create_file("convert", 0644, tegra_soctherm_root,
			    NULL, &convert_fops);
	debugfs_create_file("temp_log", 0644, tegra_soctherm_root,
			    pdev, &temp_log_fops);
	return 0;
}
#endif

void soctherm_add_trip_points(struct platform_device *pdev,
			      struct thermal_trip_info *trips,
			      int *num_trips,
			      struct tegra_cooling_device *cdev_data)
{
	int i, num;
	struct thermal_trip_info *trip_state;

	if (!trips || !num_trips || !cdev_data)
		return;

	num = cdev_data->trip_temperatures_num;
	if (*num_trips + num > THERMAL_MAX_TRIPS) {
		dev_warn(&pdev->dev, "cdev %s has too many trips\n",
			 cdev_data->cdev_type);
		return;
	}

	for (i = 0; i < (*num_trips); i++) {
		trip_state = &trips[i];
		if (!strcmp(trip_state->cdev_type, cdev_data->cdev_type)) {
			dev_warn(&pdev->dev, "cdev %s trips already added\n",
				 cdev_data->cdev_type);
			return;
		}
	}

	for (i = 0; i < num; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = cdev_data->cdev_type;
		trip_state->trip_temp = cdev_data->trip_temperatures[i] * 1000;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = trip_state->lower = i + 1;
		trip_state->hysteresis = 1000;

		(*num_trips)++;
	}
}

int soctherm_parse_pmu_dt(struct platform_device *pdev)
{
	struct soctherm_platform_data *pdata = platform_get_drvdata(pdev);
	struct device_node *soctherm_dn = pdev->dev.of_node;
	u32 val;
	int ret;

	ret = of_property_read_u32(soctherm_dn, "pmu-16bit-ops", &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read pmu-16bit-ops\n");
		return ret;
	}
	pdata->tshut_pmu_trip_data->pmu_16bit_ops = val;

	/* use I2C mode */
	pdata->tshut_pmu_trip_data->controller_type = 0;

	ret = of_property_read_u32(soctherm_dn, "pmu-i2c-addr", &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read pmu-i2c-addr\n");
		return ret;
	}
	pdata->tshut_pmu_trip_data->pmu_i2c_addr = val;

	ret = of_property_read_u32(soctherm_dn, "i2c-controller-id", &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read i2c-controller-id\n");
		return ret;
	}
	pdata->tshut_pmu_trip_data->i2c_controller_id = val;

	ret = of_property_read_u32(soctherm_dn, "poweroff-reg-addr", &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read poweroff-reg-addr\n");
		return ret;
	}
	pdata->tshut_pmu_trip_data->poweroff_reg_addr = val;

	ret = of_property_read_u32(soctherm_dn, "poweroff-reg-data", &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read poweroff-reg-data\n");
		return ret;
	}
	pdata->tshut_pmu_trip_data->poweroff_reg_data = val;

	return 0;
}

void soctherm_init_sensor(struct platform_device *pdev,
			  const struct soctherm_sensor *sensor)
{
	int i;
	struct soctherm_platform_data *pdata = platform_get_drvdata(pdev);

	for (i = 0; i < TSENSE_SIZE; i++) {
		struct soctherm_sensor *s;
		s = &pdata->sensor_data[i];
		s->tall      = s->tall      ?: sensor->tall;
		s->tiddq     = s->tiddq     ?: sensor->tiddq;
		s->ten_count = s->ten_count ?: sensor->ten_count;
		s->tsample   = s->tsample   ?: sensor->tsample;
		s->tsamp_ate = s->tsamp_ate ?: sensor->tsamp_ate;
		s->pdiv      = s->pdiv      ?: sensor->pdiv;
		s->pdiv_ate  = s->pdiv_ate  ?: sensor->pdiv_ate;
	}
}

void soctherm_init_clk_rate(struct platform_device *pdev,
			     unsigned long soctherm_clk_rate,
			     unsigned long tsensor_clk_rate)
{
	struct soctherm_platform_data *pdata = platform_get_drvdata(pdev);

	if (!pdata->soctherm_clk_rate)
		pdata->soctherm_clk_rate = soctherm_clk_rate;

	if (!pdata->tsensor_clk_rate)
		pdata->tsensor_clk_rate = tsensor_clk_rate;
}

static const struct of_device_id tegra_soctherm_match[] = {
	{ .compatible = "nvidia,tegra114-soctherm",
	  .data = &tegra114_soctherm_init, },
	{ .compatible = "nvidia,tegra124-soctherm",
	  .data = &tegra124_soctherm_init, },
	{},
};

typedef int (*of_soctherm_init)(struct device_node *);

static int tegra_soctherm_probe(struct platform_device *pdev)
{
	const struct of_device_id *matches;
	struct device_node *np;
	int ret;

	matches = tegra_soctherm_match;
	for_each_matching_node(np, matches) {
		const struct of_device_id *match = of_match_node(matches, np);
		of_soctherm_init soctherm_init = (of_soctherm_init)match->data;

		if (!soctherm_init)
			return -EINVAL;
		ret = soctherm_init(np);
		if (ret)
			return ret;
	}

	ret = soctherm_platform_init(pdev);
	if (ret)
		return -EINVAL;

	soctherm_thermal_sys_init(pdev);

#ifdef CONFIG_DEBUG_FS
	soctherm_debug_init(pdev);
#endif

	return 0;
}

static int tegra_soctherm_remove(struct platform_device *pdev)
{
	plat_data->soctherm_clk = NULL;
	plat_data->tsensor_clk = NULL;

	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(tegra_soctherm_pm,
			 soctherm_suspend, soctherm_resume);
#define TEGRA_SOC_THERM_PM	(&tegra_soctherm_pm)
#else
#define TEGRA_SOC_THERM_PM	NULL
#endif

static struct platform_driver tegra_soctherm_driver = {
	.driver = {
		.name   = "tegra_soctherm",
		.owner  = THIS_MODULE,
		.pm	= TEGRA_SOC_THERM_PM,
		.of_match_table = tegra_soctherm_match,
	},
	.probe = tegra_soctherm_probe,
	.remove = tegra_soctherm_remove,
};

static int __init tegra_soctherm_init(void)
{
	return platform_driver_register(&tegra_soctherm_driver);
}

static void __exit tegra_soctherm_exit(void)
{
	platform_driver_unregister(&tegra_soctherm_driver);
}

module_init(tegra_soctherm_init);
module_exit(tegra_soctherm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tegra1x soc thermal");
