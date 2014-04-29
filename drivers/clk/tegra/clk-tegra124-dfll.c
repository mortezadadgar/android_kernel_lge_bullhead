/*
 * clk-t124-dfll.c - clock provider support for the Tegra124 DFLL clock source
 *
 * Copyright (C) 2012-2013 NVIDIA Corporation.  All rights reserved.
 *
 * Aleksandr Frid <afrid@nvidia.com>
 * Paul Walmsley <pwalmsley@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ...
 *
 * DFLL states:
 *
 * - DISABLED: control logic mode - DISABLED, output interface disabled,
 *   dfll in reset
 * - OPEN_LOOP: control logic mode - OPEN_LOOP, output interface disabled,
 *   dfll is running "unlocked"
 * - CLOSED_LOOP: control logic mode - CLOSED_LOOP, output interface enabled,
 *   dfll is running "locked"
 *
 * In the following code, 'ol' is used to abbreviate 'open loop', and
 * 'cl' is used to abbreviate 'closed loop'.
 *
 * The IP block controlled by this driver is also known as "CL_DVFS".
 */

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>

#include <linux/clk/tegra124-dfll.h>
/*
 * XXX The fuse reading functions should be moved into a driver underneath
 * drivers/, and their symbols exported via EXPORT_SYMBOL(), rather than
 * using functions in arch/arm/mach-tegra/.
 */
#include <linux/tegra-soc.h>

#include "clk.h"

#define DRIVER_NAME			"t124_dfll"

#define to_tegra_dfll_clk_hw(_hw) container_of(_hw, struct tegra_dfll_clk_hw, \
					       hw)

/*
 * DFLL register offset & bitfield macros
 */

/* DFLL_CTRL: DFLL control register */
#define DFLL_CTRL			0x00
#define DFLL_CTRL_MODE_MASK		0x03

/* DFLL_CONFIG: DFLL sample rate control */
#define DFLL_CONFIG			0x04
#define DFLL_CONFIG_DIV_MASK		0xff
#define DFLL_CONFIG_DIV_PRESCALE	32

/* DFLL_PARAMS: Loop control register */
#define DFLL_PARAMS			0x08
#define DFLL_PARAMS_CG_SCALE		(0x1 << 24)
#define DFLL_PARAMS_CG_SCALE_SHIFT	24
#define DFLL_PARAMS_FORCE_MODE_SHIFT	22
#define DFLL_PARAMS_FORCE_MODE_MASK	(0x3 << DFLL_PARAMS_FORCE_MODE_SHIFT)
#define DFLL_PARAMS_CF_PARAM_SHIFT	16
#define DFLL_PARAMS_CF_PARAM_MASK	(0x3f << DFLL_PARAMS_CF_PARAM_SHIFT)
#define DFLL_PARAMS_CI_PARAM_SHIFT	8
#define DFLL_PARAMS_CI_PARAM_MASK	(0x7 << DFLL_PARAMS_CI_PARAM_SHIFT)
#define DFLL_PARAMS_CG_PARAM_SHIFT	0
#define DFLL_PARAMS_CG_PARAM_MASK	(0xff << DFLL_PARAMS_CG_PARAM_SHIFT)

/* DFLL_TUNE0: delay line configuration register 0 */
#define DFLL_TUNE0			0x0c
#define DFLL_TUNE0_DLY_STK_SHIFT	16
#define DFLL_TUNE0_DLY_STK_MASK		(0xff << DFLL_TUNE0_DLY_STK_SHIFT)
#define DFLL_TUNE0_DLY_SRAM_SHIFT	8
#define DFLL_TUNE0_DLY_SRAM_MASK	(0xff << DFLL_TUNE0_DLY_SRAM_SHIFT)
#define DFLL_TUNE0_DLY_INV_SHIFT	0
#define DFLL_TUNE0_DLY_INV_MASK		(0xff << DFLL_TUNE0_DLY_INV_SHIFT)

/* DFLL_TUNE1: delay line configuration register 1 */
#define DFLL_TUNE1			0x10
#define DFLL_TUNE1_DLY_FINE_SHIFT	11
#define DFLL_TUNE1_DLY_FINE_MASK	(0x1ff << DFLL_TUNE1_DLY_FINE_SHIFT)
#define DFLL_TUNE1_DLY_WIRE_SHIFT	0
#define DFLL_TUNE1_DLY_WIRE_MASK	(0x7ff << DFLL_TUNE1_DLY_WIRE_SHIFT)

/* DFLL_FREQ_REQ: target DFLL frequency control */
#define DFLL_FREQ_REQ			0x14
#define DFLL_FREQ_REQ_FORCE_ENABLE	(0x1 << 28)
#define DFLL_FREQ_REQ_FORCE_SHIFT	16
#define DFLL_FREQ_REQ_FORCE_MASK	(0xfff << DFLL_FREQ_REQ_FORCE_SHIFT)
#define FORCE_MAX			2047
#define FORCE_MIN			-2048
#define DFLL_FREQ_REQ_SCALE_SHIFT	8
#define DFLL_FREQ_REQ_SCALE_MASK	(0xff << DFLL_FREQ_REQ_SCALE_SHIFT)
#define SCALE_MAX			256
#define DFLL_FREQ_REQ_FREQ_VALID	(0x1 << 7)
#define DFLL_FREQ_REQ_FREQ_SHIFT	0
#define DFLL_FREQ_REQ_FREQ_MASK		(0x7f << DFLL_FREQ_REQ_FREQ_SHIFT)
#define FREQ_MAX			127

/* DFLL_SCALE_RAMP: slope control for output frequency ramp */
#define DFLL_SCALE_RAMP			0x18
#define DFLL_SCALE_RAMP_RATE_MASK	0xf

/* DFLL_DROOP_CTRL: droop prevention control */
#define DFLL_DROOP_CTRL			0x1c
#define DFLL_DROOP_CTRL_MIN_FREQ_SHIFT	16
#define DFLL_DROOP_CTRL_MIN_FREQ_MASK	(0xff << DFLL_DROOP_CTRL_MIN_FREQ_SHIFT)
#define DFLL_DROOP_CTRL_CUT_SHIFT	8
#define DFLL_DROOP_CTRL_CUT_MASK	(0xf << DFLL_DROOP_CTRL_CUT_SHIFT)
#define DFLL_DROOP_CTRL_RAMP_SHIFT	0
#define DFLL_DROOP_CTRL_RAMP_MASK	(0xff << DFLL_DROOP_CTRL_RAMP_SHIFT)

/* DFLL_OUTPUT_CFG: PMIC interface control */
#define DFLL_OUTPUT_CFG			0x20
#define DFLL_OUTPUT_CFG_I2C_ENABLE	(0x1 << 30)
#define OUT_MASK			0x3f
#define DFLL_OUTPUT_CFG_SAFE_SHIFT	24
#define DFLL_OUTPUT_CFG_SAFE_MASK	(OUT_MASK << DFLL_OUTPUT_CFG_SAFE_SHIFT)
#define DFLL_OUTPUT_CFG_MAX_SHIFT	16
#define DFLL_OUTPUT_CFG_MAX_MASK	(OUT_MASK << DFLL_OUTPUT_CFG_MAX_SHIFT)
#define DFLL_OUTPUT_CFG_MIN_SHIFT	8
#define DFLL_OUTPUT_CFG_MIN_MASK	(OUT_MASK << DFLL_OUTPUT_CFG_MIN_SHIFT)

/* DFLL_OUTPUT_FORCE: output forcing control */
#define DFLL_OUTPUT_FORCE		0x24
#define DFLL_OUTPUT_FORCE_ENABLE_SHIFT	6
#define DFLL_OUTPUT_FORCE_ENABLE_MASK	(0x1 << DFLL_OUTPUT_FORCE_ENABLE_SHIFT)
#define DFLL_OUTPUT_FORCE_DEBOUNCE_CNT_SHIFT	0
#define DFLL_OUTPUT_FORCE_DEBOUNCE_CNT_MASK	\
	(0x3f << DFLL_OUTPUT_FORCE_DEBOUNCE_CNT_SHIFT)

/* DFLL_MONITOR_CTRL: loop data source control */
#define DFLL_MONITOR_CTRL		0x28
#define DFLL_MONITOR_CTRL_SRC_SHIFT	0
#define DFLL_MONITOR_CTRL_SRC_MASK	(0x7 << DFLL_MONITOR_CTRL_SRC_SHIFT)
#define DFLL_MONITOR_CTRL_DISABLE	0
#define DFLL_MONITOR_CTRL_FREQ		6

/* DFLL_MONITOR_DATA: internal monitoring */
#define DFLL_MONITOR_DATA		0x2c
#define DFLL_MONITOR_DATA_NEW_MASK	(0x1 << 16)
#define DFLL_MONITOR_DATA_VAL_MASK	0xFFFF

/* DFLL_I2C_CFG: I2C controller configuration register */
#define DFLL_I2C_CFG			0x40
#define DFLL_I2C_CFG_ARB_ENABLE		(0x1 << 20)
#define DFLL_I2C_CFG_HS_CODE_SHIFT	16
#define DFLL_I2C_CFG_HS_CODE_MASK	(0x7 << DFLL_I2C_CFG_HS_CODE_SHIFT)
#define DFLL_I2C_CFG_PACKET_ENABLE	(0x1 << 15)
#define DFLL_I2C_CFG_SIZE_SHIFT		12
#define DFLL_I2C_CFG_SIZE_MASK		(0x7 << DFLL_I2C_CFG_SIZE_SHIFT)
#define DFLL_I2C_CFG_SLAVE_ADDR_10	(0x1 << 10)
#define DFLL_I2C_CFG_SLAVE_ADDR_SHIFT	0
#define DFLL_I2C_CFG_SLAVE_ADDR_MASK	(0x3ff << DFLL_I2C_CFG_SLAVE_ADDR_SHIFT)

/* DFLL_I2C_VDD_REG_ADDR: PMIC internal register controlling VDD */
#define DFLL_I2C_VDD_REG_ADDR		0x44
#define DFLL_I2C_DEFAULT_DATA_SHIFT	8
#define DFLL_I2C_DEFAULT_DATA_MASK	(0xff << DFLL_I2C_DEFAULT_DATA_SHIFT)
#define DFLL_I2C_ADDR_DATA_SHIFT	0
#define DFLL_I2C_ADDR_DATA_MASK		(0xff << DFLL_I2C_ADDR_DATA_SHIFT)

/* DFLL_I2C_STS: I2C controller status */
#define DFLL_I2C_STS			0x48
#define DFLL_I2C_STS_I2C_LAST_SHIFT	1
#define DFLL_I2C_STS_I2C_REQ_PENDING	0x1

/* DFLL_INTR_STS: DFLL interrupt status register */
#define DFLL_INTR_STS			0x5c

/* DFLL_INTR_EN: DFLL interrupt enable register */
#define DFLL_INTR_EN			0x60
#define DFLL_INTR_MIN_MASK		0x1
#define DFLL_INTR_MAX_MASK		0x2

/* DFLL_I2C_CLK_DIVISOR: I2C controller clock divisor */
#define DFLL_I2C_CLK_DIVISOR		0x16c
#define DFLL_I2C_CLK_DIVISOR_MASK	0xffff
#define DFLL_I2C_CLK_DIVISOR_FS_SHIFT	16
#define DFLL_I2C_CLK_DIVISOR_HS_SHIFT	0
#define DFLL_I2C_CLK_DIVISOR_PREDIV	8
#define DFLL_I2C_CLK_DIVISOR_HSMODE_PREDIV	12

/*
 * DFLL_OUTPUT_LUT: Offset to the start of the 33x8-bit voltage lookup
 *     table. 32-bit aligned.  Used in I2C mode only.
 */
#define DFLL_OUTPUT_LUT			0x200


/*
 * Other constants
 */

/*
 * DFLL_CALIBR_TIME: minimum time interval between DFLL calibration
 * attempts, in microseconds
 */
#define DFLL_CALIBR_TIME		40000

/*
 * DFLL_OUTPUT_PENDING_TIMEOUT: if the code waits for longer than
 * DFLL_OUTPUT_PENDING_TIMEOUT microseconds for in-progress I2C
 * commands to the PMIC to complete, then indicate a timeout error.
 */
#define DFLL_OUTPUT_PENDING_TIMEOUT	1000

/*
 * DFLL_OUTPUT_RAMP_DELAY: after the DFLL's I2C PMIC output starts
 * requesting the minimum TUNE_HIGH voltage, the minimum number of
 * microseconds to wait for the PMIC's voltage output to finish
 * slewing
 */
#define DFLL_OUTPUT_RAMP_DELAY		100

/*
 * DFLL_TUNE_HIGH_DELAY: number of microseconds to wait between tests
 * to see if the high voltage has been reached yet, during a
 * transition from the low-voltage range to the high-voltage range
 */
#define DFLL_TUNE_HIGH_DELAY		2000

/*
 * DFLL_TUNE_HIGH_MARGIN_STEPS: attempt to initially program the DFLL
 * voltage target to a (DFLL_TUNE_HIGH_MARGIN_STEPS * 10 millivolt)
 * margin above the high voltage floor, in closed-loop mode in the
 * high-voltage range
 */
#define DFLL_TUNE_HIGH_MARGIN_STEPS	2

/* MAX_DFLL_VOLTAGES: number of LUT entries */
#define MAX_DFLL_VOLTAGES		33

/*
 * MAX_THERMAL_CAPS: maximum number of thermal cap (temperature,
 * voltage) tuples that can be specified in the driver data.
 */
#define MAX_THERMAL_CAPS		8

/*
 * MAX_THERMAL_FLOORS: maximum number of thermal floor (temperature,
 * voltage) tuples that can be specified in the driver data.
 */
#define MAX_THERMAL_FLOORS		8

/*
 * MAX_DVFS_FREQS: maximum number of DFLL output frequencies that this
 * driver supports.
 */
#define MAX_DVFS_FREQS			40

/*
 * CVB_ROW_WIDTH: in the DT data, this represents the number of cells
 * in each CVB table entry
 */
#define CVB_ROW_WIDTH			4

/*
 * THERM_CAPS_ROW_WIDTH: in the DT data, this represents the number
 * of cells in each thermal cap table entry
 */
#define THERM_CAPS_ROW_WIDTH		2

/*
 * THERM_FLOORS_ROW_WIDTH: in the DT data, this represents the number
 * of cells in each thermal floor table entry
 */
#define THERM_FLOORS_ROW_WIDTH		2

/*
 * DFLL_OUTPUT_CLOCK_NAME: the output clock name representing the DVCO
 * output rate -- registered with the clock framework.
 */
#define DFLL_OUTPUT_CLOCK_NAME		"dfllCPU_out"

/*
 * I2C_OUTPUT_ACTIVE_TEST_US: mandatory minimum interval (in
 * microseconds) between testing whether the I2C controller is
 * currently sending a voltage-set command.  Some comments list this
 * as being a worst-case margin for "disable propagation."
 */
#define I2C_OUTPUT_ACTIVE_TEST_US	2

/**
 * enum tegra_dfll_force_mode - output voltage forcing mode during freq change
 * %TEGRA_DFLL_FORCE_NONE: no I2C output forcing
 * %TEGRA_DFLL_FORCE_FIXED: force for a fixed number of sample periods
 * %TEGRA_DFLL_FORCE_AUTO: force for a calculated number of sample periods
 *
 * Controls whether the output voltage index that the DFLL
 * communicates to the PMIC is forced to a certain value during
 * frequency changes, and if so, how long the forced value persists.
 * Only applies to I2C control.
 */
enum tegra_dfll_force_mode {
	TEGRA_DFLL_FORCE_NONE = 0,
	TEGRA_DFLL_FORCE_FIXED = 1,
	TEGRA_DFLL_FORCE_AUTO = 2,
};

/**
 * struct tegra_dfll_voltage_reg_map - map of voltages to PMIC register values
 * @reg_value: PMIC voltage control register value that produces @reg_uV
 * @reg_uv: microvolts DC that the PMIC will generate when @reg_value is used
 *
 * Maps a voltage @reg_uv to the PMIC voltage-set register value @reg_value.
 */
struct tegra_dfll_voltage_reg_map {
	u8		reg_value;
	int		reg_uv;
};

/**
 * enum tegra_dfll_ctrl_mode - DFLL hardware operating mode
 * @TEGRA_DFLL_UNINITIALIZED: (uninitialized state - not in hardware bitfield)
 * @TEGRA_DFLL_DISABLED: DFLL not generating an output clock
 * @TEGRA_DFLL_OPEN_LOOP: DVCO running, but DFLL not adjusting voltage
 * @TEGRA_DFLL_CLOSED_LOOP: DVCO running and DFLL adjusting PMIC voltage
 *
 * The integer corresponding to the last three states, minus one, is
 * written to the DFLL hardware to change operating modes.
 */
enum tegra_dfll_ctrl_mode {
	TEGRA_DFLL_UNINITIALIZED = 0,
	TEGRA_DFLL_DISABLED = 1,
	TEGRA_DFLL_OPEN_LOOP = 2,
	TEGRA_DFLL_CLOSED_LOOP = 3,
};

/**
 * enum tegra_dfll_tune_state - state of the voltage-regime switching code
 * @TEGRA_DFLL_TUNE_LOW: DFLL in the low-voltage range (or open-loop mode)
 * @TEGRA_DFLL_TUNE_WAIT_DFLL: waiting for DFLL voltage output to reach high
 * @TEGRA_DFLL_TUNE_WAIT_PMIC: waiting for PMIC to react to DFLL output
 * @TEGRA_DFLL_TUNE_HIGH: DFLL in the high-voltage range
 *
 * These are software states; these values are never written into
 * registers.
 */
enum tegra_dfll_tune_state {
	TEGRA_DFLL_TUNE_LOW = 0,
	TEGRA_DFLL_TUNE_WAIT_DFLL,
	TEGRA_DFLL_TUNE_WAIT_PMIC,
	TEGRA_DFLL_TUNE_HIGH,
};

/**
 * struct dfll_rate_req - target DFLL rate request data
 * @freq: value to program to the FREQ bitfield of the DFLL_FREQ_REQ register
 * @scale: value to program to the SCALE bitfield of DFLL_FREQ_REQ
 * @output: desired voltage to force the PMIC voltage output to (LUT index)
 * @cap: unconditionally safe voltage for this frequency
 * @rate: frequency in Hz corresponding to @freq
 *
 * When in closed-loop mode, there is guaranteed to be some voltage
 * margin below @cap for the DFLL to adjust down to.
 */
struct dfll_rate_req {
	u8	freq;
	u8	scale;
	u8	output;
	u8	cap;
	unsigned long rate;
};

/*
 * Possible values for struct tegra_dfll.flags:
 *
 * TEGRA_DFLL_FLAGS_I2C_FORCE_QUIET: disable the DFLL's PMIC voltage
 *   control output before disabling the DFLL IP block.  Set when the
 *   'i2c-quiet-output-workaround' is present in the DT data for the
 *   DFLL IP block.
 */
#define TEGRA_DFLL_FLAGS_I2C_FORCE_QUIET	BIT(0)

/**
 * struct tegra_dfll_cvb - CVB table
 * @freq: (DT) target DVCO frequency
 * @c0: (DT) DFLL calibration constant 0
 * @c1: (DT) DFLL calibration constant 1
 * @c2: (DT) DFLL calibration constant 2
 *
 * Voltage curve data, indexed by @freq.  This data comes from DT.
 */
struct tegra_dfll_cvb {
	unsigned long freq;
	int c0;
	int c1;
	int c2;
};

/**
 * struct tegra_dfll_therm - thermally-sensitive voltage caps
 * @temp: maximum trip temperature
 * @output: PMIC voltage output at @temp or below
 * @mv: minimum voltage output, in millivolts, at @temp or below
 */
struct tegra_dfll_therm {
	u8 temp;
	u8 output;
	u16 mv;
};

/**
 * struct tegra_dfll_clk_hw - DFLL clk_hw wrapper for the clock framework
 * @pdev: DFLL instance
 * @hw: struct clk_hw - for use by the clock framework
 *
 * The @pdev is used by the DFLL driver's clock framework interface
 * functions, to retrieve the DFLL context.
 */
struct tegra_dfll_clk_hw {
	struct platform_device		*pdev;
	struct clk_hw			hw;
};

/**
 * struct tegra_dfll_dvfs_info - all frequency and voltage related table
 *
 * @num_freqs: number of total entries in @freqs
 * @num_voltages: number of entries in out_map
 * @cvb_table_len: number of CVB table entries (see @cvb_table)
 * @freqs: map from CVB table index to target output clock frequency
 * @cpu_dfll_millivolts: map from CVB table index to output voltage (in mV)
 * @clk_dvfs_map: map from CVB table index to output LUT index
 * @cvb_table: CVB table data, from DT
 *
 * @clk_dvfs_map: output voltage mapping: legacy dvfs table index -to-
 * cl_dvfs output LUT index;  cl_dvfs output LUT index -to- PMU
 * value/voltage pair ptr
 */
struct tegra_dfll_dvfs_info {
	u8				num_freqs;
	u8				num_voltages;
	u8				cvb_table_len;
	unsigned long	freqs[MAX_DVFS_FREQS];
	int				cpu_dfll_millivolts[MAX_DVFS_FREQS];
	u8				clk_dvfs_map[MAX_DVFS_FREQS];
	struct tegra_dfll_cvb		*cvb_table;
};

/**
 * struct tegra_dfll - context for a DFLL instance
 * @base: virtual address that the DFLL IP block MMIO space is mapped to
 * @soc_clk: DFLL logic clock input - 51MHz
 * @ref_clk: Reference clock input - 51MHz
 * @i2c_clk: I2C5 controller clock input
 * @dfll_clk: our output clock - registered in dfll_init()
 * @ref_rate: clock rate of @ref_clk.  Does not change once set
 * @vdd_map_size: number of entries in @vdd_map
 * @vdd_map: list of all possible voltages & corresponding PMIC VSELs
 * @out_map: map from LUT index to voltage & PMIC VSEL - before clamping
 * @safe_output: LUT index of the minimum safe voltage (OPEN_LOOP or DISABLED)
 * @tune_high_out_start: LUT index to start with when tuning to high range
 * @tune_high_out_min: LUT index of the minimum high tuning range voltage
 * @minimax_output: LUT index of the minimum level of the maximum CL voltage
 * @dvco_rate_min: @out_rate_min, rounded to (@ref_rate / 2)
 * @lut_min: LUT index of the minimum voltage to send to the PMIC
 * @lut_max: LUT index of the maximum voltage to send to the PMIC
 * @therm_caps_idx: LUT index of the maximum volatage
 * @therm_floors_idx: LUT index of the minimum voltage (due to cold die effects)
 * @last_req: most recent closed-loop output frequency request
 * @tune_state: in closed-loop mode: is the DFLL in low- or high-voltage regime?
 * @mode: DFLL hardware operating mode: disabled, open-loop, or closed-loop
 * @resume_mode: DFLL mode to be set to when resuming
 * @tune_timer: timer used to wait during TUNE_HIGH_REQUEST for high Vdd
 * @tune_delay: interval between tune_timer_cb calls during TUNE_HIGH_REQUEST
 * @flags: (see "Possible values for struct tegra_dfll.flags" above)
 * @tune0_low_voltage_range: DFLL_TUNE0 reg value in the low-voltage regime
 * @tune0_high_voltage_range: DFLL_TUNE0 reg value in the high-voltage regime
 * @tune1: DFLL_TUNE1 reg value (valid for both voltage regimes)
 * @droop_rate_min: min ring osc freq before voltage droop control is enabled
 * @tune_high_min_mv: starting voltage (in mV) of the high-voltage tuning range
 * @dent: dentry for DFLL debugfs (at /DRIVER_NAME)
 * @sample_rate: control loop sample rate
 * @force_mode: I2C: force PMIC voltage during a freq change?  if so, how?
 * @cf: I2C: duration to force the PMIC voltage after frequency change
 * @cg: loop gain (signed)
 * @cg_scale: set to 1 to divide loop gain by 8
 * @ci: loop integral gain selector
 * @droop_cut_value: control output clock scaler at minimum ring osc freq
 * @droop_restore_ramp: clock recovery rate after a voltage droop event
 * @out_rate_min: "FmaxAtVmin": DFLL max operating freq @ minimum voltage
 * @scale_out_ramp: voltage output ramp rate
 * @min_millivolts: minimum voltage (in millivolts)
 * @cvb_max_millivolts: CVB maximum voltage limit (in millivolts)
 * @cvb_speedo_scale: silicon characterization constant - from fuses
 * @cvb_voltage_scale: CVB voltage scale factor (to microvolts)
 * @speedo_id: CPU Speedo ID - from process characterization
 * @process_id: CPU process ID - from process characterization
 * @speedo_value: CPU Speedo value - from process characterization
 * @pmic_i2c_addr: PMIC I2C address (when I2C interface is active)
 * @pmic_i2c_voltage_reg: PMIC I2C voltage control register
 * @pmic_i2c_fs_rate: I2C FS bus rate
 * @pmic_i2c_hs_rate: I2C HS bus rate - set to talk to PMIC in HS mode
 * @pmic_i2c_hs_master_code: PMIC I2C HS master code (only needed for HS mode)
 * @pmic_i2c_ten_bit_addrs: use 10-bit I2C address to address PMIC
 * @vdd: regulator controlling the DFLL's voltage rail
 * @vdd_step: linear step size between VSEL values (from regulator framework)
 * @dfll_min_microvolt: lowest voltage the DFLL should try to program (from DT)
 * @dfll_max_microvolt: highest voltage the DFLL should try to program (from DT)
 * @therm_caps_num: number of entries in @therm_caps
 * @therm_caps: maximum voltage caps at various temperatures
 * @therm_floors_num: number of entries in @therm_floors
 * @therm_floors: minimum voltage floors at various cold die temperatures
 * @calibration_timer: timer to periodically recalibrate dvco_min_rate
 * @calibration_delay: time interval between calibrations - see DFLL_CALIBR_TIME
 * @last_calibration: ktime_t that the last calibration started
 * @calibration_range_min: absolute minimum rate of the DFLL calibration range
 * @calibration_range_max: absolute maximum rate of the DFLL calibration range
 * @cdev: pointer to registered DFLL thermal device (if loaded)
 * @lock: spinlock to protect accesses to DFLL registers and state
 *
 * @dfll_min_microvolt must be greater than or equal to the PMIC
 * regulator's low voltage limit.  @dfll_max_microvolt must be lesser
 * than or equal to the PMIC's high voltage limit.
 */
struct tegra_dfll {
	void __iomem			*base;

	struct clk			*soc_clk;
	struct clk			*ref_clk;
	struct clk			*i2c_clk;
	struct clk			*dfll_clk;
	unsigned long			ref_rate;

	u8				safe_output;
	u8				tune_high_out_start;
	u8				tune_high_out_min;
	u8				minimax_output;
	u8				lut_min;
	u8				lut_max;
	u8				therm_caps_idx;
	u8				therm_caps_num;
	u8				therm_floors_idx;
	u8				therm_floors_num;
	u8				flags;
	u8				vdd_map_size;
	u8				cg_scale;
	struct tegra_dfll_voltage_reg_map	*vdd_map;
	struct tegra_dfll_voltage_reg_map	*out_map[MAX_DFLL_VOLTAGES];
	struct tegra_dfll_dvfs_info *dvfs_info;
	unsigned long			dvco_rate_min;

	struct tegra_dfll_therm		therm_caps[MAX_THERMAL_CAPS];
	struct tegra_dfll_therm		therm_floors[MAX_THERMAL_FLOORS];
	struct dfll_rate_req		last_req;
	enum tegra_dfll_tune_state	tune_state;
	enum tegra_dfll_ctrl_mode	mode;
	enum tegra_dfll_ctrl_mode	resume_mode;

	struct timer_list		tune_timer;
	unsigned long			tune_delay;

	u32				tune0_low_voltage_range;
	u32				tune0_high_voltage_range;
	u32				tune1;
	u32				droop_rate_min;
	u32				tune_high_min_mv;

	struct dentry			*dent;

	u32				sample_rate;
	u32				force_mode;
	u32				cf;
	u32				ci;
	u32				cg;
	u32				droop_cut_value;
	u32				droop_restore_ramp;
	unsigned long			out_rate_min;
	u32				scale_out_ramp;

	u32				min_millivolts;
	u32				cvb_max_millivolts;

	int				cvb_speedo_scale; /* must be signed */
	int				cvb_voltage_scale; /* must be signed */

	int				speedo_id;
	int				process_id;

	int				speedo_value;

	u32				pmic_i2c_addr;
	u32				pmic_i2c_voltage_reg;
	u32				pmic_i2c_fs_rate;
	u32				pmic_i2c_hs_rate;
	u32				pmic_i2c_hs_master_code;
	u32				pmic_i2c_ten_bit_addrs;

	struct regulator		*vdd;
	int				vdd_step; /* must be signed */

	u32				dfll_min_microvolt;
	u32				dfll_max_microvolt;

	struct timer_list		calibration_timer;
	unsigned long			calibration_delay;
	ktime_t				last_calibration;
	unsigned long			calibration_range_min;
	unsigned long			calibration_range_max;

	struct tegra_dfll_clk_hw	dfll_clk_hw;
	struct thermal_cooling_device	*cdev_floor;
	struct thermal_cooling_device	*cdev_cap;
	spinlock_t			lock;	/* see kerneldoc above */
};

/*
 * Conversion macros (different scales for frequency request, and
 * monitored rate is not a typo)
 */
#define RATE_STEP(td)				((td)->ref_rate / 2)
#define GET_REQUEST_FREQ(rate, ref_rate)	((rate) / ((ref_rate) / 2))
#define GET_REQUEST_RATE(freq, ref_rate)	((freq) * ((ref_rate) / 2))
#define GET_MONITORED_RATE(freq, ref_rate)	((freq) * ((ref_rate) / 4))
#define GET_DROOP_FREQ(rate, ref_rate)		((rate) / ((ref_rate) / 4))
#define ROUND_MIN_RATE(rate, ref_rate)		\
	(DIV_ROUND_UP(rate, (ref_rate) / 2) * ((ref_rate) / 2))
#define GET_DIV(ref_rate, out_rate, scale)	\
	DIV_ROUND_UP((ref_rate), (out_rate) * (scale))

/* fcpu_dfll_pdev: the DFLL platform device instance */
static struct platform_device *fcpu_dfll_pdev;

/* mode_name: map numeric DFLL modes to names for friendly console messages */
static const char * const mode_name[] = {
	[TEGRA_DFLL_UNINITIALIZED] = "uninitialized",
	[TEGRA_DFLL_DISABLED] = "disabled",
	[TEGRA_DFLL_OPEN_LOOP] = "open_loop",
	[TEGRA_DFLL_CLOSED_LOOP] = "closed_loop",
};

/* Static functions */

static inline u32 dfll_readl(struct tegra_dfll *td, u32 offs)
{
	return __raw_readl(td->base + offs);
}

static inline void dfll_writel(struct tegra_dfll *td, u32 val, u32 offs)
{
	__raw_writel(val, td->base + offs);
}

/**
 * dfll_wmb - ensure all MMIO writes from the CPU to the DFLL have completed
 * @td: struct tegra_dfll * device context
 *
 * Ensure that all writes from the CPU to the memory-mapped I/O space
 * of the DFLL IP block have completed.  Assumes that the CPU that
 * this code is currently running on has excluded other CPUs on the
 * system from accessing the DFLL IP block MMIO space.
 */
static inline void dfll_wmb(struct tegra_dfll *td)
{
	dfll_readl(td, DFLL_CTRL);
}

/**
 * i2c_output_enable - enable generation of voltage adjustment I2C commands
 * @pdev: DFLL instance
 * @val: ptr to the value to write to DFLL_OUTPUT_CFG register
 *
 * Tell the DFLL-I2C interface to start transmitting voltage-set
 * commands to the PMIC.  The variable pointed to by @v must be loaded
 * from DFLL_OUTPUT_CFG, or manually set, before calling this code.
 * No return value.
 */
static void i2c_output_enable(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;

	val = dfll_readl(td, DFLL_OUTPUT_CFG);
	val |= DFLL_OUTPUT_CFG_I2C_ENABLE;
	dfll_writel(td, val, DFLL_OUTPUT_CFG);
	dfll_wmb(td);
}

/**
 * i2c_output_disable - stop generating voltage adjustment I2C commands
 * @pdev: DFLL instance
 * @val: ptr to the value of the DFLL_OUTPUT_CFG register
 *
 * Tell the DFLL-I2C interface to stop transmitting voltage-set
 * commands to the PMIC.  The variable pointed to by @v must be loaded
 * from DFLL_OUTPUT_CFG, or manually set, before calling this code.
 * Most code should not call this directly - instead call
 * i2c_output_disable_flush(). No return value.
 */
static void i2c_output_disable(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;

	val = dfll_readl(td, DFLL_OUTPUT_CFG);
	val &= ~DFLL_OUTPUT_CFG_I2C_ENABLE;
	dfll_writel(td, val, DFLL_OUTPUT_CFG);
	dfll_wmb(td);
}

/**
 * is_output_i2c_req_pending - is an I2C voltage-set command in progress?
 * @pdev: DFLL instance
 *
 * Returns 1 if an I2C request is in progress, or 0 if not.  The DFLL
 * IP block requires two back-to-back reads of the I2C_REQ_PENDING
 * field to return 0 before the software can be sure that no I2C
 * request is currently pending.  Also, a minimum time interval
 * between DFLL_I2C_STS reads is required by the IP block.
 */
static int is_output_i2c_req_pending(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 sts;

	sts = dfll_readl(td, DFLL_I2C_STS);
	if (sts & DFLL_I2C_STS_I2C_REQ_PENDING)
		return 1;

	udelay(I2C_OUTPUT_ACTIVE_TEST_US);

	sts = dfll_readl(td, DFLL_I2C_STS);
	if (sts & DFLL_I2C_STS_I2C_REQ_PENDING)
		return 1;

	return 0;
}

/**
 * i2c_output_disable_flush - disable I2C output and wait for I2C command to finish
 * @pdev: DFLL instance
 *
 * Prevent the DFLL I2C controller from sending any further
 * voltage-set commands, then wait for any in-progress commands to
 * complete.  This is the normal (non-workaround) path.  Returns 0 if
 * the flush completed within the timeout interval, or -ETIMEDOUT
 * otherwise.
 */
static int i2c_output_disable_flush(struct platform_device *pdev)
{
	int i, t;

	t = DFLL_OUTPUT_PENDING_TIMEOUT / I2C_OUTPUT_ACTIVE_TEST_US;

	i2c_output_disable(pdev);

	for (i = 0; i < t; i++) {
		if (!is_output_i2c_req_pending(pdev))
			return 0;

		udelay(I2C_OUTPUT_ACTIVE_TEST_US);
	}

	/* I2C request is still pending - report error */
	return -ETIMEDOUT;
}

/**
 * set_mode - change the DFLL control mode
 * @pdev: DFLL instance
 * @mode: DFLL control mode (see enum tegra_dfll_ctrl_mode)
 *
 * Change the DFLL's operating mode from open-loop mode to closed-loop
 * mode, or vice versa.  No return value.
 */
static void set_mode(struct platform_device *pdev,
		     enum tegra_dfll_ctrl_mode mode)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	td->mode = mode;
	dfll_writel(td, mode - 1, DFLL_CTRL);
	dfll_wmb(td);
}

/**
 * get_output_cap - return LUT index of the maximum non-DFLL safe voltage
 * @pdev: DFLL instance
 *
 * Return the LUT index of the maximum non-DFLL safe voltage, given
 * the current temperature of the SoC.
 */
static u8 get_output_cap(struct platform_device *pdev,
			 struct dfll_rate_req *req)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 thermal_cap = td->dvfs_info->num_voltages - 1;

	if (td->therm_caps_idx && (td->therm_caps_idx <= td->therm_caps_num))
		thermal_cap = td->therm_caps[td->therm_caps_idx - 1].output;
	if (req && (req->cap < thermal_cap))
		return req->cap;
	return thermal_cap;
}

/**
 * get_output_min - return LUT index of the minimum non-DFLL safe voltage
 * @pdev: DFLL instance
 *
 * Return the LUT index of the minimum non-DFLL safe voltage, given
 * the DFLL's current tuning state and the current temperature of the
 * SoC.
 */
static u8 get_output_min(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 tune_min, thermal_min;

	tune_min = (td->tune_state == TEGRA_DFLL_TUNE_LOW) ?
		0 : td->tune_high_out_min;

	thermal_min = 0;
	if (td->therm_floors_idx < td->therm_floors_num)
		thermal_min = td->therm_floors[td->therm_floors_idx].output;

	return max(tune_min, thermal_min);
}

/*
 * Voltage lookup table operations
 */

/**
 * _load_lut - load voltage lookup table into DFLL RAM
 * @pdev: DFLL instance
 *
 * Load the voltage-to-PMIC register value lookup table into the DFLL
 * IP block LUT memory.  td->lut_min and td->lut_max are used to cap
 * the minimum and maximum voltage requested.  This function shouldn't
 * be called directly by code other than dfll_load_lut(), since this
 * function doesn't handle the necessary pre- and post-requisites.  No
 * return value.
 */
static void _load_lut(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;
	u32 val;

	val = td->out_map[td->lut_min]->reg_value;
	for (i = 0; i <= td->lut_min; i++)
		dfll_writel(td, val, DFLL_OUTPUT_LUT + i * 4);

	for (; i < td->lut_max; i++) {
		val = td->out_map[i]->reg_value;
		dfll_writel(td, val, DFLL_OUTPUT_LUT + i * 4);
	}

	val = td->out_map[td->lut_max]->reg_value;
	for (; i < td->dvfs_info->num_voltages; i++)
		dfll_writel(td, val, DFLL_OUTPUT_LUT + i * 4);

	dfll_wmb(td);
}

/**
 * dfll_load_lut - load the voltage lookup table, disabling output if needed
 * @td: struct tegra_dfll *
 *
 * Load the voltage-to-PMIC register value lookup table into the DFLL
 * IP block memory, disabling and re-enabling the I2C output if
 * needed.  Look-up tables can be loaded at any time.  No return
 * value.
 */
static void dfll_load_lut(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val = dfll_readl(td, DFLL_OUTPUT_CFG);
	int output_quiet_before_disable;
	bool disable_out_for_load;

	output_quiet_before_disable = (td->flags &
				       TEGRA_DFLL_FLAGS_I2C_FORCE_QUIET);

	/*
	 * If the I2C output can be disabled at any time, and it's currently
	 * enabled, then disable it.
	 */
	disable_out_for_load = !output_quiet_before_disable &&
		(val & DFLL_OUTPUT_CFG_I2C_ENABLE);
	if (disable_out_for_load) {
		i2c_output_disable(pdev);
		udelay(I2C_OUTPUT_ACTIVE_TEST_US);
	}

	_load_lut(pdev);

	/* Re-enable the I2C output if we disabled it previously. */
	if (disable_out_for_load)
		i2c_output_enable(pdev);
}

/**
 * set_tune_state - set the internal tune state variable and log some debug
 * @pdev: DFLL instance
 * @state: tune_state to change the internal state variable to
 *
 * Set the internal tune_state variable to @state, and optionally output some
 * debug.  Does not affect the hardware.  No return value.
 */
static void set_tune_state(struct platform_device *pdev,
			   enum tegra_dfll_tune_state state)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	td->tune_state = state;
	pr_debug("%s: set tune state %d\n", __func__, state);
}

/**
 * tune_low - fine-tune DFLL and CPU clock shaper for low voltages
 * @pdev: DFLL instance
 * @state: tune_state to change the internal state variable to
 *
 * Fine-tune the DFLL oscillator parameters and the CPU clock shaper for
 * the low-voltage range.  The top end of the low-voltage range is
 * represented by the index (td->tune_high_out_start - 1).  No return value.
 */
static void tune_low(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	dfll_writel(td, td->tune0_low_voltage_range, DFLL_TUNE0);
	dfll_wmb(td);
}

/**
 * tune_high - fine-tune DFLL and CPU clock shaper for high voltages
 * @pdev: DFLL instance
 * @state: tune_state to change the internal state variable to
 *
 * Fine-tune the DFLL oscillator parameters and the CPU clock shaper
 * for the high-voltage range.  The bottom end of the high-voltage
 * range is represented by the index td->tune_high_out_start.
 * Used in closed-loop mode.  No return value.
 */
static void tune_high(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	dfll_writel(td, td->tune0_high_voltage_range, DFLL_TUNE0);
	dfll_wmb(td);
}

/**
 * tune_to_low_state - switch to the minimum safe non-DFLL voltage
 * @pdev: DFLL instance
 *
 * Set the DFLL's voltage output floor to the minimum safe non-DFLL
 * voltage, and tune the DFLL for the low-voltage range.  Used before
 * switching to open-loop mode.  This prevents the DFLL from entering
 * open-loop mode with an output voltage below the minimum safe
 * non-DFLL voltage.
 */
static void tune_to_low_state(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 out_min;

	set_tune_state(pdev, TEGRA_DFLL_TUNE_LOW);
	tune_low(pdev);

	out_min = get_output_min(pdev);
	if (td->lut_min != out_min) {
		td->lut_min = out_min;
		dfll_load_lut(pdev);
	}
}

/**
 * set_ol_config - prepare to switch to open-loop mode
 * @pdev: DFLL instance
 *
 * Prepare to switch the DFLL to open-loop mode.  This involves ensuring
 * that the DFLL is in the low-voltage tuning regime, and disabling
 * output frequency scaling.  No return value.
 */
static void set_ol_config(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;

	/* always tune low (safe) in open loop */
	if (td->tune_state != TEGRA_DFLL_TUNE_LOW)
		tune_to_low_state(pdev);

	/* 1:1 scaling in open loop */
	val = dfll_readl(td, DFLL_FREQ_REQ);
	val |= (SCALE_MAX - 1) << DFLL_FREQ_REQ_SCALE_SHIFT;
	val &= ~DFLL_FREQ_REQ_FORCE_ENABLE;
	dfll_writel(td, val, DFLL_FREQ_REQ);
}

/**
 * set_cl_config - prepare to switch to closed-loop mode
 * @pdev: DFLL instance
 * @req: requested output rate
 *
 * Prepare to switch the DFLL to closed-loop mode.  This involves
 * switching the DFLL's tuning voltage regime (if necessary), and
 * rewriting the LUT to restrict the minimum and maximum voltages.  No
 * return value.
 */
static void set_cl_config(struct platform_device *pdev,
			  struct dfll_rate_req *req)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 out_max, out_min;
	u8 out_cap = get_output_cap(pdev, req);

	switch (td->tune_state) {
	case TEGRA_DFLL_TUNE_LOW:
		if (out_cap > td->tune_high_out_start) {
			set_tune_state(pdev, TEGRA_DFLL_TUNE_WAIT_DFLL);
			mod_timer(&td->tune_timer, jiffies + td->tune_delay);
		}
		break;

	case TEGRA_DFLL_TUNE_HIGH:
	case TEGRA_DFLL_TUNE_WAIT_DFLL:
	case TEGRA_DFLL_TUNE_WAIT_PMIC:
		if (out_cap <= td->tune_high_out_start) {
			set_tune_state(pdev, TEGRA_DFLL_TUNE_LOW);
			tune_low(pdev);
		}
		break;
	default:
		BUG();
	}

	out_min = get_output_min(pdev);
	if (out_cap > out_min + 1)
		req->output = out_cap - 1;
	else
		req->output = out_min + 1;
	if (req->output == td->safe_output)
		req->output++;
	out_max = max((u8)(req->output + 1), td->minimax_output);

	if ((td->lut_min != out_min) || (td->lut_max != out_max)) {
		td->lut_min = out_min;
		td->lut_max = out_max;
		dfll_load_lut(pdev);
	}
}

/**
 * tune_timer_cb - timer callback during TUNE_HIGH_REQUEST
 * @data: struct platform_device * of the DFLL instance
 *
 * Timer callback, used when switching from TUNE_LOW to TUNE_HIGH in
 * closed-loop mode.  Waits for DFLL I2C voltage command output to
 * reach tune_high_out_min, then waits for the PMIC to react to the
 * command.  No return value.
 */
static void tune_timer_cb(unsigned long data)
{
	struct platform_device *pdev = (struct platform_device *)data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val, out_min, out_last;
	unsigned long flags;

	spin_lock_irqsave(&td->lock, flags);

	if (td->tune_state == TEGRA_DFLL_TUNE_WAIT_DFLL) {
		out_min = td->lut_min;

		val = dfll_readl(td, DFLL_I2C_STS);
		out_last = (val >> DFLL_I2C_STS_I2C_LAST_SHIFT) & OUT_MASK;

		if (!is_output_i2c_req_pending(pdev) &&
		    (out_last >= td->tune_high_out_min) &&
		    (out_min >= td->tune_high_out_min)) {
			set_tune_state(pdev, TEGRA_DFLL_TUNE_WAIT_PMIC);
			mod_timer(&td->tune_timer, jiffies +
				  usecs_to_jiffies(DFLL_OUTPUT_RAMP_DELAY));
		} else {
			mod_timer(&td->tune_timer, jiffies + td->tune_delay);
		}
	} else if (td->tune_state == TEGRA_DFLL_TUNE_WAIT_PMIC) {
		set_tune_state(pdev, TEGRA_DFLL_TUNE_HIGH);
		tune_high(pdev);
	}

	spin_unlock_irqrestore(&td->lock, flags);
}

/**
 * tegra_dfll_calibrate - recalibrate dvco_rate_min to match reality
 * @pdev: DFLL instance
 *
 * Adjust our estimate of the DVCO minimum rate based on the measured
 * rate. This is only done if the DFLL is in closed loop mode, the
 * last request engaged the clock skipper, a minimum amount of time
 * has passed since the last calibration attempt, and there is no I2C
 * transaction pending.  No return value.
 */
static void tegra_dfll_calibrate(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;
	ktime_t now;
	unsigned long data;
	u8 out_min;

	if ((td->mode != TEGRA_DFLL_CLOSED_LOOP) ||
	    (td->last_req.rate > td->dvco_rate_min))
		return;

	now = ktime_get();
	if (ktime_us_delta(now, td->last_calibration) < DFLL_CALIBR_TIME)
		return;

	out_min = get_output_min(pdev);

	/* Skip calibration this time if I2C transaction is pending */
	val = dfll_readl(td, DFLL_I2C_STS);
	if (is_output_i2c_req_pending(pdev)) {
		mod_timer(&td->calibration_timer,
			  jiffies + td->calibration_delay);
		return;
	}

	/* XXX Cache DFLL_MONITOR_CTRL to avoid an unnecessary DFLL read */
	if (dfll_readl(td, DFLL_MONITOR_CTRL) != DFLL_MONITOR_CTRL_FREQ)
		dfll_writel(td, DFLL_MONITOR_CTRL_FREQ, DFLL_MONITOR_CTRL);

	/* Synchronize with sample period, and get rate measurements */
	data = dfll_readl(td, DFLL_MONITOR_DATA);
	do {
		data = dfll_readl(td, DFLL_MONITOR_DATA);
	} while (!(data & DFLL_MONITOR_DATA_NEW_MASK));
	do {
		data = dfll_readl(td, DFLL_MONITOR_DATA);
	} while (!(data & DFLL_MONITOR_DATA_NEW_MASK));

	td->last_calibration = now;

	/* Adjust minimum rate */
	data &= DFLL_MONITOR_DATA_VAL_MASK;
	data = GET_MONITORED_RATE(data, td->ref_rate);
	if ((val > out_min) || (data < (td->dvco_rate_min - RATE_STEP(td))))
		td->dvco_rate_min -= RATE_STEP(td);
	else if (data > (td->dvco_rate_min + RATE_STEP(td)))
		td->dvco_rate_min += RATE_STEP(td);
	else
		return;

	td->dvco_rate_min = clamp(td->dvco_rate_min, td->calibration_range_min,
				  td->calibration_range_max);
	mod_timer(&td->calibration_timer, jiffies + td->calibration_delay);
	dev_dbg(&pdev->dev, "calibrated dvco_rate_min %lu\n",
		td->dvco_rate_min);
}

/**
 * calibration_timer_cb - calibrate the DFLL; called from a timer
 * @data: struct platform_device * of the DFLL instance
 *
 * Take the DFLL lock and run the calibration process.  Called as a
 * timer callback.  No return value.
 */
static void calibration_timer_cb(unsigned long data)
{
	struct platform_device *pdev = (struct platform_device *)data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long flags;

	spin_lock_irqsave(&td->lock, flags);
	tegra_dfll_calibrate(pdev);
	spin_unlock_irqrestore(&td->lock, flags);
}

/**
 * set_request - ask the DFLL to tune to a different frequency
 * @pdev: DFLL instance
 * @req: target frequency request
 *
 * Tell the DFLL to try to change its output frequency to the
 * frequency represented by @req.  DFLL must be in closed-loop mode.
 * No return value.
 */
static void set_request(struct platform_device *pdev, struct dfll_rate_req *req)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;
	int force_val = req->output - td->safe_output;
	int coef = 128; /* FIXME: td->cg_scale? */;

	force_val = force_val * coef / td->cg;
	force_val = clamp(force_val, FORCE_MIN, FORCE_MAX);

	val = req->freq << DFLL_FREQ_REQ_FREQ_SHIFT;
	val |= req->scale << DFLL_FREQ_REQ_SCALE_SHIFT;
	val |= ((u32)force_val << DFLL_FREQ_REQ_FORCE_SHIFT) &
		DFLL_FREQ_REQ_FORCE_MASK;
	val |= DFLL_FREQ_REQ_FREQ_VALID | DFLL_FREQ_REQ_FORCE_ENABLE;

	dfll_writel(td, val, DFLL_FREQ_REQ);
	dfll_wmb(td);
}

/**
 * find_mv_out_cap - find the out_map index with voltage >= @mv
 * @pdev: DFLL instance
 * @mv: millivolts
 *
 * Find the out_map index with voltage greater than or equal to @mv,
 * and return it.  If all of the voltages in out_map are less than
 * @mv, then return the out_map index * corresponding to the highest
 * possible voltage, even though it's less than @mv.
 */
static u8 find_mv_out_cap(struct platform_device *pdev, int mv)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u8 cap;

	for (cap = 0; cap < td->dvfs_info->num_voltages; cap++)
		if (td->out_map[cap]->reg_uv >= mv * 1000)
			return cap;

	return cap - 1;	/* maximum possible output */
}

/**
 * find_mv_out_floor - find the largest out_map index with voltage < @mv
 * @pdev: DFLL instance
 * @mv: millivolts
 *
 * Find the largest out_map index with voltage lesser to @mv,
 * and return it.  If all of the voltages in out_map are greater than
 * @mv, then return the out_map index * corresponding to the minimum
 * possible voltage, even though it's greater than @mv.
 */
static u8 find_mv_out_floor(struct platform_device *pdev, int mv)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u8 floor;

	for (floor = 0; floor < td->dvfs_info->num_voltages; floor++) {
		if (td->out_map[floor]->reg_uv > mv * 1000) {
			if (!floor)
				/* minimum possible output */
				return 0;
			break;
		}
	}
	return floor - 1;
}

/**
 * find_safe_output - find minimum safe voltage for a desired clock frequency
 * @pdev: DFLL instance
 * @rate: desired clock frequency
 * @safe_output: ptr to a variable to return the minimum safe voltage index
 *
 * For a desired clock frequency @rate, finds the voltage @safe_output
 * that is unconditionally safe to run at for any PVT point.  There is
 * guaranteed to be some voltage below @safe_output that the DFLL can
 * adjust down to.  Returns 0 if found, or -ENOENT if no safe voltage
 * could be found for @rate.
 */
static int find_safe_output(struct platform_device *pdev, unsigned long rate,
			    u8 *safe_output)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < td->dvfs_info->num_freqs; i++) {
		if (td->dvfs_info->freqs[i] >= rate) {
			*safe_output = td->dvfs_info->clk_dvfs_map[i];
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * find_dvco_rate_min - find min output freq for an output voltage LUT index
 * @pdev: DFLL instance
 * @out_min: an output voltage LUT index to find the minimum frequency for
 *
 * Given an output voltage LUT index @out_min, return the "minimum" clock
 * frequency that the DVCO will generate (based on the CVB table data).
 * Note that this frequency may not really be a strict minimum, due to
 * calibration/characterization errors.
 */
static unsigned long find_dvco_rate_min(struct platform_device *pdev,
					u8 out_min)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < td->dvfs_info->num_freqs; i++)
		if (td->dvfs_info->clk_dvfs_map[i] > out_min)
			break;

	i = i ? i-1 : 0;
	return td->dvfs_info->freqs[i];
}

/**
 * set_dvco_rate_min - set the minimum DVCO output rate & interval
 * @pdev: DFLL instance
 *
 * Find and cache the "minimum" DVCO output frequency, as well as the
 * lower and upper calibration boundaries around this frequency.  No
 * return value.
 */
static void set_dvco_rate_min(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long rate = td->out_rate_min;

	if (td->therm_floors_idx < td->therm_floors_num)
		rate = find_dvco_rate_min(
			pdev, td->therm_floors[td->therm_floors_idx].output);

	/* round minimum rate to request unit (ref_rate/2) boundary */
	td->dvco_rate_min = ROUND_MIN_RATE(rate, td->ref_rate);

	/* dvco min rate is under-estimated - skewed range up */
	td->calibration_range_min = td->dvco_rate_min - 2 * RATE_STEP(td);
	td->calibration_range_max = td->dvco_rate_min + 8 * RATE_STEP(td);
}

/**
 * get_cvb_voltage - returns the CVB voltage based on frequency & silicon
 * @pdev: DFLL instance
 * @c0: calibration data 0 (see below)
 * @c1: calibration data 1 (see below)
 * @c2: calibration data 2 (see below)
 *
 * Return the CVB voltage (in millivolts) for a particular set of
 * frequency and silicon parameters.  The formula used to compute the
 * voltage is:
 *
 * cvb_mv = ((c2 * speedo / s_scale + c1) * speedo / s_scale + c0) / v_scale
 */
static int get_cvb_voltage(struct platform_device *pdev, int c0, int c1,
			   int c2)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	/* apply only speedo scale: output mv = cvb_mv * v_scale */
	int mv;

	/* combined: apply voltage scale and round to cvb alignment step */
	mv = DIV_ROUND_CLOSEST(c2 * td->speedo_value, td->cvb_speedo_scale);
	mv = DIV_ROUND_CLOSEST((mv + c1) * td->speedo_value,
			       td->cvb_speedo_scale) + c0;

	return DIV_ROUND_UP(mv * 1000,
			    td->cvb_voltage_scale * td->vdd_step) *
		td->vdd_step / 1000;
}

/**
 * find_vdd_map_entry_exact - find vdd_map entry with voltage equal to @mv
 * @pdev: DFLL instance
 * @mv: millivolts to look up in vdd_map[]
 *
 * Attempt to find the entry in the vdd_map with the voltage is equal
 * to @mv.  Returns a pointer to the matching vdd_map entry if found;
 * otherwise, returns NULL.
 */
static struct tegra_dfll_voltage_reg_map *find_vdd_map_entry_exact(
	struct platform_device *pdev, int mv)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < td->vdd_map_size; i++)
		if (mv == td->vdd_map[i].reg_uv / 1000)
			return &td->vdd_map[i];

	return NULL;
}

/**
 * find_vdd_map_entry_min - find first vdd_map entry with voltage >= @mv
 * @pdev: DFLL instance
 * @mv: millivolts to look up in vdd_map[]
 *
 * Attempt to find the lowest-voltage entry in the vdd_map with a
 * voltage equal to or greater than @mv.  Returns a pointer to the
 * matching vdd_map entry if found; otherwise, returns NULL.
 */
static struct tegra_dfll_voltage_reg_map *find_vdd_map_entry_min(
	struct platform_device *pdev, int mv)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < td->vdd_map_size; i++)
		if (mv <= td->vdd_map[i].reg_uv / 1000)
			return &td->vdd_map[i];

	return NULL;
}

/**
 * prepare_volt_freq_table - analyze the CVB table and build indexes
 * @pdev: DFLL instance
 *
 * Loop through the CVB table data, generating the starting output
 * voltage for each frequency point.  (Each CVB entry specifies CPU
 * frequency and CVB coefficients to calculate the respective voltage
 * when DFLL is used as the CPU clock source.)
 *
 * Minimum voltage limit is applied only to DFLL source.  Maximum
 * voltage limit is applied by directly clipping voltage for DFLL.
 *
 * Returns 0 upon success, or -EINVAL if the CVB table or DFLL configuration
 * data is invalid.
 */
static int prepare_volt_freq_table(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct tegra_dfll_cvb *tdc;
	u32 dfll_mv;
	int i;
	unsigned long fmax_at_vmin = 0;

	for (i = 0; i < td->dvfs_info->cvb_table_len; i++) {
		tdc = &td->dvfs_info->cvb_table[i];

		dfll_mv = get_cvb_voltage(pdev, tdc->c0, tdc->c1, tdc->c2);

		/* Check maximum frequency at minimum voltage */
		if (dfll_mv > td->min_millivolts) {
			if (!i)
				break;	/* 1st entry already above Vmin */
			if (!fmax_at_vmin)
				fmax_at_vmin = td->dvfs_info->freqs[i - 1];
		}

		dfll_mv = max(dfll_mv, td->min_millivolts);

		td->dvfs_info->freqs[i] = tdc->freq;
		td->dvfs_info->cpu_dfll_millivolts[i] = min(dfll_mv,
						 td->cvb_max_millivolts);
		td->dvfs_info->num_freqs = i + 1;

		/*
		 * "Round-up" frequency list cut-off (keep first entry that
		 *  exceeds max voltage - the voltage limit will be enforced
		 *  anyway, so when requested this frequency dfll will settle
		 *  at whatever high frequency it can on the particular chip)
		 */
		if (dfll_mv > td->cvb_max_millivolts)
			break;
	}

	/*
	 * Table must not be empty and must have at least
	 * 1. one entry <= Vmin and
	 * 2. one entry >= Vmin
	 */
	if (!i || !fmax_at_vmin) {
		dev_err(&pdev->dev, "invalid cpu dvfs table\n");
		return -EINVAL;
	}

	td->out_rate_min = fmax_at_vmin;

	return 0;
}

/**
 * dfll_init_maps - build output voltage-related maps
 * @pdev: DFLL instance
 *
 * Build the LUT index-to-output PMIC voltage value map (out_map), and
 * the CVB table index-to-LUT index map (clk_dvfs_map).  Returns 0
 * upon success or -EINVAL, -ERANGE, or -ENOENT upon error.
 */
static int __must_check dfll_init_maps(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i, j, v, v_max, n, r;
	struct tegra_dfll_voltage_reg_map *m;

	BUILD_BUG_ON(MAX_DFLL_VOLTAGES > OUT_MASK + 1);

	r = prepare_volt_freq_table(pdev);
	if (r) {
		dev_err(&pdev->dev, "unable to prepare voltage table\n");
		return -EINVAL;
	}

	n = td->dvfs_info->num_freqs;
	if (n >= MAX_DFLL_VOLTAGES) {
		dev_err(&pdev->dev, "too many frequencies (%d)\n", n);
		return -EINVAL;
	}

	v_max = td->dvfs_info->cpu_dfll_millivolts[n - 1];

	v = td->min_millivolts;
	if (v > td->dvfs_info->cpu_dfll_millivolts[0]) {
		dev_err(&pdev->dev,
			"min voltage %d > lowest voltage in map %d\n",
			td->min_millivolts,
			td->dvfs_info->cpu_dfll_millivolts[0]);
		return -ERANGE;
	}

	td->out_map[0] = find_vdd_map_entry_exact(pdev, v);
	if (!td->out_map[0]) {
		dev_err(&pdev->dev, "no vdd_map entry for min voltage %d\n", v);
		return -ENOENT;
	}

	for (i = 0, j = 1; i < n; i++) {
		for (;;) {
			v += max(1, (v_max - v) / (MAX_DFLL_VOLTAGES - j));
			if (v >= td->dvfs_info->cpu_dfll_millivolts[i])
				break;

			m = find_vdd_map_entry_min(pdev, v);
			if (!m) {
				dev_err(&pdev->dev,
					"no vdd_map entry for map voltage %d\n",
					v);
				return -ENOENT;
			}
			if (m != td->out_map[j - 1])
				td->out_map[j++] = m;
		}

		v = td->dvfs_info->cpu_dfll_millivolts[i];
		m = find_vdd_map_entry_exact(pdev, v);
		if (!m) {
			dev_err(&pdev->dev,
				"no exact vdd_map entry for voltage %d\n", v);
			return -ENOENT;
		}
		if (m != td->out_map[j - 1])
			td->out_map[j++] = m;
		td->dvfs_info->clk_dvfs_map[i] = j - 1;
		if (j > MAX_DFLL_VOLTAGES) {
			dev_err(&pdev->dev, "too many out_map entries\n");
			return -ENOMEM;
		}
	}
	td->dvfs_info->num_voltages = j;

	return 0;
}

/**
 * dfll_init_tuning_thresholds - set up the high voltage range, if possible
 * @pdev: DFLL instance
 *
 * Determine whether the DFLL tuning parameters need to be
 * reprogrammed when the DFLL voltage reaches a certain minimum
 * threshold.  No return value.
 */
static void dfll_init_tuning_thresholds(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u8 out_min, out_start, max_voltage_index;

	max_voltage_index = td->dvfs_info->num_voltages - 1;

	/*
	 * Convert high tuning voltage threshold into output LUT
	 * index, and add necessary margin.  If voltage threshold is
	 * outside operating range set it at maximum output level to
	 * effectively disable tuning parameters adjustment.
	 */
	td->tune_high_out_min = max_voltage_index;
	td->tune_high_out_start = max_voltage_index;
	if (td->tune_high_min_mv < td->min_millivolts)
		return;	/* no difference between low & high voltage range */

	out_min = find_mv_out_cap(pdev, td->tune_high_min_mv);
	if ((out_min + 2) > max_voltage_index)
		return;

	out_start = out_min + DFLL_TUNE_HIGH_MARGIN_STEPS;
	if (out_start > max_voltage_index)
		return;

	td->tune_high_out_min = out_min;
	td->tune_high_out_start = out_start;
	if (td->minimax_output <= out_min)
		td->minimax_output = out_min + 1;
}

/**
 * dfll_init_hot_output_cap - set a thermally-sensitive maximum voltage cap
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization, map the temperature-dependent
 * voltage caps from millivolts to output LUT indexes, and make sure
 * there is room for regulation below the minimum thermal cap.  Must
 * be called after the therm-caps property has been parsed from DT.
 * Must be called after td->safe_output is first set by taking
 * td->therm_caps[0] into consideration.  The voltage cap must
 * monotonically decrease as temperatures monotonically increase.
 * Returns 0 upon success, -EINVAL if the voltage cap data is in an
 * invalid format, or -ENOSPC if there's no space to adjust the
 * voltage above the minimum.
 */
static int __must_check dfll_init_hot_output_cap(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;
	u8 output;

	if (td->therm_caps_num == 0)
		return 0;

	for (i = 0; i < td->therm_caps_num; i++) {
		if (i > 0 &&
		    (td->therm_caps[i].mv >= td->therm_caps[i-1].mv ||
		     td->therm_caps[i].temp <= td->therm_caps[i-1].temp)) {
			dev_err(&pdev->dev,
				"format error in thermal cap data\n");
			return -EINVAL;
		}

		td->therm_caps[i].output =
			find_mv_out_floor(pdev, td->therm_caps[i].mv);
	}

	output = td->therm_caps[td->therm_caps_num - 1].output;
	if (output < td->minimax_output) {
		dev_err(&pdev->dev,
			"no space available for regulation below max\n");
		return -ENOSPC;
	}

	return 0;
}

/**
 * dfll_init_cold_output_floor - set a thermally-sensitive minimum voltage floor
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization, map the temperature-dependent
 * voltage floors from millivolts to output LUT indexes, and make sure
 * there is room for regulation above the maximum thermal floor.  Must
 * be called after the therm-floors property has been parsed from DT.
 * Must be called before td->safe_output is first set by taking
 * td->therm_floors[0] into consideration.  The voltage floor must
 * monotonically decrease as temperatures monotonically increase.
 * Returns 0 upon success, -EINVAL if the voltage floor data is in an
 * invalid format, or -ENOSPC if there's no space to adjust the
 * voltage above the minimum.
 */
static int __must_check dfll_init_cold_output_floor(
	struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int i;

	if (td->therm_floors_num == 0)
		return 0;

	for (i = 0; i < td->therm_floors_num; i++) {
		if (i > 0 &&
		    (td->therm_floors[i].mv >= td->therm_floors[i-1].mv ||
		     td->therm_floors[i].temp <= td->therm_floors[i-1].temp)) {
			dev_err(&pdev->dev,
				"format error in thermal floor data\n");
			return -EINVAL;
		}

		td->therm_floors[i].output =
			find_mv_out_cap(pdev, td->therm_floors[i].mv);
	}

	if (td->therm_floors[0].output + 2 >= td->dvfs_info->num_voltages) {
		dev_err(&pdev->dev,
			"no space available for regulation above min\n");
		return -ENOSPC;
	}

	return 0;
}

/**
 * dfll_init_output_thresholds - set various DFLL min/max voltage thresholds
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization, set the minimum safe voltage
 * output LUT index (safe_output), along with the maximum voltage that
 * the DFLL can request in closed-loop mode (minimax_output).  Returns
 * 0 upon success, or passes along the return value from
 * dfll_init_cold_output_floor() upon error.
 */
static int __must_check dfll_init_output_thresholds(
	struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int r;

	td->minimax_output = 0;
	td->tune_state = TEGRA_DFLL_TUNE_LOW;

	dfll_init_tuning_thresholds(pdev);

	r = dfll_init_cold_output_floor(pdev);
	if (r) {
		dev_err(&pdev->dev, "could not set cold output floor: %d\n", r);
		return r;
	}

	/* make sure safe output is safe at any temperature */
	td->safe_output = td->therm_floors[0].output ? : 1;
	if (td->minimax_output <= td->safe_output)
		td->minimax_output = td->safe_output + 1;

	/* init caps after minimax output is determined */
	r = dfll_init_hot_output_cap(pdev);
	if (r) {
		dev_err(&pdev->dev, "could not set hot output cap: %d\n", r);
		return r;
	}

	return 0;
}

/*
 * DFLL PMIC I2C controller
 */

/**
 * dfll_init_i2c_clk - initialize clock for the DFLL's internal I2C controller
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization, set up the DFLL's internal I2C
 * controller.  Only used if the DFLL is using I2C to communicate with
 * the PMIC (which is the only method currently supported in this
 * driver).  td->i2c_clk must be enabled before calling.  Returns 0
 * upon success or -ERANGE if the computed I2C divisor is out of
 * range.
 */
static int dfll_init_i2c_clk(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;
	u32 div = 2; /* default hs divisor just in case */
	unsigned long i2c_clk_rate;

	i2c_clk_rate = clk_get_rate(td->i2c_clk);

	val = GET_DIV(i2c_clk_rate, td->pmic_i2c_fs_rate,
		      DFLL_I2C_CLK_DIVISOR_PREDIV);
	if (val == 0 || val > DFLL_I2C_CLK_DIVISOR_MASK) {
		dev_err(&pdev->dev, "I2C FS clock divisor out of range\n");
		return -ERANGE;
	}
	val = (val - 1) << DFLL_I2C_CLK_DIVISOR_FS_SHIFT;
	if (td->pmic_i2c_hs_rate) {
		div = GET_DIV(i2c_clk_rate, td->pmic_i2c_hs_rate,
			      DFLL_I2C_CLK_DIVISOR_HSMODE_PREDIV);
		if (div == 0 || div > DFLL_I2C_CLK_DIVISOR_MASK) {
			dev_err(&pdev->dev,
				"I2C HS clock divisor out of range\n");
			return -ERANGE;
		}
	}
	val |= (div - 1) << DFLL_I2C_CLK_DIVISOR_HS_SHIFT;
	dfll_writel(td, val, DFLL_I2C_CLK_DIVISOR);
	dfll_wmb(td);

	return 0;
}

/*
 * DFLL-to-I2C controller interface
 */

/**
 * dfll_init_i2c_if - set up the DFLL's DFLL-I2C interface
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization, program the DFLL-I2C interfae
 * with the PMU slave address, vdd register offset, and transfer mode.
 * This data is used by the DFLL to automatically construct I2C
 * voltage-set commands, which are then passed to the DFLL's internal
 * I2C controller.  No return value.
 */
static void dfll_init_i2c_if(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;

	val = td->pmic_i2c_addr << DFLL_I2C_CFG_SLAVE_ADDR_SHIFT;

	if (td->pmic_i2c_ten_bit_addrs)
		val |= DFLL_I2C_CFG_SLAVE_ADDR_10;
	if (td->pmic_i2c_hs_rate) {
		val |= (td->pmic_i2c_hs_master_code <<
			DFLL_I2C_CFG_HS_CODE_SHIFT);
		val |= DFLL_I2C_CFG_PACKET_ENABLE;
	}
	val |= DFLL_I2C_CFG_SIZE_MASK;
	val |= DFLL_I2C_CFG_ARB_ENABLE;
	dfll_writel(td, val, DFLL_I2C_CFG);
	dfll_writel(td, td->pmic_i2c_voltage_reg, DFLL_I2C_VDD_REG_ADDR);
}

/**
 * dfll_init_out_if - prepare DFLL-to-PMIC interface
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization or resume from context loss,
 * disable the I2C command output to the PMIC, set safe voltage and
 * output limits, and disable and clear limit interrupts.  No return
 * value.
 */
static void dfll_init_out_if(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val, out_min, out_max;

	/* Necessary for the call to get_output_min() */
	td->tune_state = TEGRA_DFLL_TUNE_LOW;
	td->therm_caps_idx = td->therm_caps_num;
	td->therm_floors_idx = 0;
	set_dvco_rate_min(pdev);

	/*
	 * Allow the entire range of LUT indexes, but limit output
	 * voltage in LUT mapping (this "indirect" application of
	 * limits is used, because h/w does not support dynamic change
	 * of index limits, but dynamic reload of LUT is fine).
	 */
	out_min = 0;
	out_max = td->dvfs_info->num_voltages - 1;
	td->lut_min = get_output_min(pdev);
	td->lut_max = get_output_cap(pdev, NULL);

	val = (td->safe_output << DFLL_OUTPUT_CFG_SAFE_SHIFT) |
		(out_max << DFLL_OUTPUT_CFG_MAX_SHIFT) |
		(out_min << DFLL_OUTPUT_CFG_MIN_SHIFT);
	dfll_writel(td, val, DFLL_OUTPUT_CFG);
	dfll_wmb(td);

	dfll_writel(td, 0, DFLL_OUTPUT_FORCE);
	dfll_writel(td, 0, DFLL_INTR_EN);
	dfll_writel(td, DFLL_INTR_MAX_MASK | DFLL_INTR_MIN_MASK, DFLL_INTR_STS);

	dfll_load_lut(pdev);

	dfll_init_i2c_if(pdev);
	dfll_init_i2c_clk(pdev);
}

/**
 * dfll_init_cntrl_logic - set up some of the DFLL registers
 * @pdev: DFLL instance
 *
 * During DFLL driver initialization or resume from context loss, set
 * up many of the DFLL IP block registers, including the DFLL mode,
 * control loop parameters, and tuning.  Returns 0 upon success or
 * -ERANGE if a value destined for a register bitfield will exceed
 * that bitfield (which indicates a problem with the DT data).
 */
static int __must_check dfll_init_cntrl_logic(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;

	set_mode(pdev, TEGRA_DFLL_DISABLED);

	val = GET_DIV(td->ref_rate, td->sample_rate, DFLL_CONFIG_DIV_PRESCALE);
	if (val > DFLL_CONFIG_DIV_MASK) {
		dev_err(&pdev->dev, "sample-rate exceeds bitfield width\n");
		return -ERANGE;
	}
	dfll_writel(td, val, DFLL_CONFIG);

	val = (td->force_mode << DFLL_PARAMS_FORCE_MODE_SHIFT) |
		(td->cf << DFLL_PARAMS_CF_PARAM_SHIFT) |
		(td->ci << DFLL_PARAMS_CI_PARAM_SHIFT) |
		((u8)td->cg << DFLL_PARAMS_CG_PARAM_SHIFT) |
		(td->cg_scale << DFLL_PARAMS_CG_SCALE_SHIFT);
	dfll_writel(td, val, DFLL_PARAMS);

	tune_low(pdev);
	dfll_writel(td, td->tune1, DFLL_TUNE1);

	/* configure droop (skipper 1) and scale (skipper 2) */
	val = GET_DROOP_FREQ(td->droop_rate_min, td->ref_rate);
	val <<= DFLL_DROOP_CTRL_MIN_FREQ_SHIFT;
	if (val > DFLL_DROOP_CTRL_MIN_FREQ_MASK) {
		dev_err(&pdev->dev, "droop-rate-min exceeds bitfield width\n");
		return -ERANGE;
	}

	val |= (td->droop_cut_value << DFLL_DROOP_CTRL_CUT_SHIFT);
	val |= (td->droop_restore_ramp << DFLL_DROOP_CTRL_RAMP_SHIFT);
	dfll_writel(td, val, DFLL_DROOP_CTRL);

	td->last_req.cap = 0;
	td->last_req.freq = 0;
	td->last_req.output = 0;
	td->last_req.scale = SCALE_MAX - 1;
	dfll_writel(td, DFLL_FREQ_REQ_SCALE_MASK, DFLL_FREQ_REQ);
	dfll_writel(td, td->scale_out_ramp, DFLL_SCALE_RAMP);

	/* select frequency for monitoring */
	dfll_writel(td, DFLL_MONITOR_CTRL_FREQ, DFLL_MONITOR_CTRL);
	dfll_wmb(td);

	return 0;
}

/**
 * dfll_enable_clocks - enable all clocks needed by the DFLL
 * @pdev: DFLL instance
 *
 * Enable all clocks needed by the DFLL.  Assumes that clk_prepare()
 * has already been called on all the clocks.  Assumes that the DFLL
 * communicates with the PMIC via the DFLL I2C interface, which is
 * currently the only option supported by this driver.  No return
 * value.
 */
static void dfll_enable_clocks(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	clk_enable(td->i2c_clk);
	clk_enable(td->ref_clk);
	clk_enable(td->soc_clk);
}

/**
 * dfll_disable_clocks - disable all clocks needed by the DFLL
 * @pdev: DFLL instance
 *
 * Disable all clocks needed by the DFLL.  Assumes that other code
 * will later call clk_unprepare().  Assumes that the DFLL
 * communicates with the PMIC via the DFLL I2C interface, which is
 * currently the only option supported by this driver.  No return
 * value.
 */
/* XXX: Shouldn't the clock ordering be reversed from the enable? */
static void dfll_disable_clocks(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	clk_disable(td->i2c_clk);
	clk_disable(td->ref_clk);
	clk_disable(td->soc_clk);
}

/**
 * dfll_init_timers - prepare the TUNE_HIGH and calibration timers
 * @pdev: DFLL instance
 *
 * Fill in the required fields for the TUNE_HIGH and calibration
 * timers.  The TUNE_HIGH timer is used in DFLL configurations with
 * both low and high voltage tuning ranges.  The timer callback is
 * activated upon a request to switch to the high voltage tuning range
 * - see set_cl_config().  The calibration timer is used to adjust
 * the DVCO minimum rate.  No return value.
 */
static void dfll_init_timers(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	init_timer(&td->tune_timer);
	td->tune_timer.function = tune_timer_cb;
	td->tune_timer.data = (unsigned long)pdev;
	td->tune_delay = usecs_to_jiffies(DFLL_TUNE_HIGH_DELAY);

	init_timer_deferrable(&td->calibration_timer);
	td->calibration_timer.function = calibration_timer_cb;
	td->calibration_timer.data = (unsigned long)pdev;
	td->calibration_delay = usecs_to_jiffies(DFLL_CALIBR_TIME);
}

/**
 * dfll_init - Prepare the DFLL IP block for use
 * @pdev: DFLL instance
 *
 * Do everything necessary to prepare the DFLL IP block for use.
 * The DFLL will be left in DISABLED state.  Called by
 * tegra_dfll_probe().  Returns 0 upon success, or passes along the
 * error from whatever function returned it.
 */
static int dfll_init(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int ret;

	tegra124_clock_deassert_dfll_dvco_reset();

	ret = clk_prepare_enable(td->i2c_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable i2c_clk\n");
		return ret;
	}
	/* Enable module clocks, release control logic reset */
	ret = clk_prepare_enable(td->ref_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable ref_clk\n");
		goto di_err1;
	}
	ret = clk_prepare_enable(td->soc_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable soc_clk\n");
		goto di_err2;
	}

	td->ref_rate = clk_get_rate(td->ref_clk);
	if (td->ref_rate == 0) {
		dev_err(&pdev->dev, "ref_clk rate cannot be 0\n");
		goto di_err3;
	}

	dfll_init_timers(pdev);

	ret = dfll_init_maps(pdev);
	if (ret) {
		dev_err(&pdev->dev, "could not init the voltage map\n");
		goto di_err3;
	}

	ret = dfll_init_output_thresholds(pdev);
	if (ret) {
		dev_err(&pdev->dev, "could not init the voltage thresholds\n");
		goto di_err3;
	}

	dfll_init_out_if(pdev);

	/* Configure control registers in disabled mode */
	ret = dfll_init_cntrl_logic(pdev);
	if (ret) {
		dev_err(&pdev->dev, "could not init the control logic\n");
		goto di_err3;
	}

	spin_lock_init(&td->lock);

	dfll_disable_clocks(pdev);

	return 0;
di_err3:
	clk_disable_unprepare(td->soc_clk);
di_err2:
	clk_disable_unprepare(td->ref_clk);
di_err1:
	clk_disable_unprepare(td->i2c_clk);

	tegra124_clock_assert_dfll_dvco_reset();

	return ret;
}

/*
 * DFLL enable/disable & open-loop <-> closed-loop transitions
 */

/**
 * tegra_dfll_disable - switch from open-loop mode to disabled mode
 * @pdev: DFLL instance
 *
 * Switch from OPEN_LOOP state to DISABLED state.  No return value.
 */
static void tegra_dfll_disable(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long flags;

	spin_lock_irqsave(&td->lock, flags);

	if (td->mode != TEGRA_DFLL_OPEN_LOOP) {
		dev_err(&pdev->dev, "cannot disable DFLL in %s mode\n",
			mode_name[td->mode]);
		goto tdd_exit;
	}

	set_mode(pdev, TEGRA_DFLL_DISABLED);

tdd_exit:
	spin_unlock_irqrestore(&td->lock, flags);
}

/**
 * tegra_dfll_enable - switch a disabled DFLL to open-loop mode
 * @pdev: DFLL instance
 *
 * Switch from DISABLED state to OPEN_LOOP state.  Returns 0 upon success
 * or -EPERM if the DFLL is not currently in open-loop mode.
 */
static int tegra_dfll_enable(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&td->lock, flags);

	if (td->mode != TEGRA_DFLL_DISABLED) {
		dev_err(&pdev->dev, "cannot enable DFLL in %s mode\n",
			mode_name[td->mode]);
		ret = -EPERM;
		goto tde_exit;
	}

	set_mode(pdev, TEGRA_DFLL_OPEN_LOOP);

tde_exit:
	spin_unlock_irqrestore(&td->lock, flags);
	return ret;
}

/**
 * tegra_dfll_lock - switch from open-loop to closed-loop mode
 * @pdev: DFLL instance
 *
 * Switch from OPEN_LOOP state to CLOSED_LOOP state.  Returns 0 upon success,
 * -EINVAL if the DFLL's target rate hasn't been set yet, or -EPERM if the
 * DFLL is not currently in open-loop mode.
 */
static int tegra_dfll_lock(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct dfll_rate_req *req = &td->last_req;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&td->lock, flags);

	if (td->mode != TEGRA_DFLL_OPEN_LOOP) {
		WARN(1, "DFLL: cannot lock in %s mode\n", mode_name[td->mode]);
		ret = -EPERM;
		goto tdl_exit;
	}

	if (req->freq == 0) {
		dev_err(&pdev->dev, "cannot lock DFLL at rate 0\n");
		ret = -EINVAL;
		goto tdl_exit;
	}

	/*
	 * Update control logic setting with last rate request; sync
	 * output limits with current tuning and thermal state, enable
	 * output and switch to closed loop mode.
	 */
	set_cl_config(pdev, req);
	i2c_output_enable(pdev);
	set_mode(pdev, TEGRA_DFLL_CLOSED_LOOP);
	set_request(pdev, req);
	mod_timer(&td->calibration_timer, jiffies + td->calibration_delay);

tdl_exit:
	spin_unlock_irqrestore(&td->lock, flags);

	return ret;
}

/**
 * tegra_dfll_unlock - switch from closed-loop to open-loop mode
 * @pdev: DFLL instance
 *
 * Switch from CLOSED_LOOP state to OPEN_LOOP state.  Passes along the
 * return value from i2c_output_disable_flush(), or -EPERM if the
 * DFLL is not currently in closed-loop mode.
 */
static int tegra_dfll_unlock(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&td->lock, flags);

	if (td->mode != TEGRA_DFLL_CLOSED_LOOP) {
		WARN(1, "DFLL: cannot unlock in %s mode\n",
		     mode_name[td->mode]);
		ret = -EPERM;
		goto tdu_exit;
	}

	set_ol_config(pdev);
	ret = i2c_output_disable_flush(pdev);
	if (ret)
		dev_warn(&pdev->dev, "I2C pending timeout\n");
	set_mode(pdev, TEGRA_DFLL_OPEN_LOOP);

tdu_exit:
	spin_unlock_irqrestore(&td->lock, flags);

	return ret;
}

/*
 * Set/get the DFLL's targeted output clock rate
 */

/**
 * reprogram_last_request - reprogram the last frequency request in CL mode
 * @pdev: DFLL instance
 *
 * Reprogram the last frequency request when the DFLL is in
 * closed-loop mode, changing voltage regimes if necessary.  No return
 * value.
 */
static void reprogram_last_request(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	if (td->mode != TEGRA_DFLL_CLOSED_LOOP)
		return;

	set_cl_config(pdev, &td->last_req);
	set_request(pdev, &td->last_req);
}

/**
 * calc_req_scale_mult - calculate DFLL parameters for a given rate
 * @pdev: DFLL instance
 * @req: DFLL-rate-request structure record pointer
 * @dvco_rate: ptr to the unsigned long containing the DFLL DVCO rate
 *
 * Populate the DFLL-rate-request record @req fields with the scale
 * and freq parameters, based on the target input rate, based in by
 * the caller in req->rate.  Pass back the approximate rate that the
 * DVCO will generate in @dvco_rate.  Returns 0 upon success, or
 * -EINVAL if the requested rate in req->rate is too high or low for
 * the DFLL to generate.
 */
static int calc_req_scale_mult(struct platform_device *pdev,
			       struct dfll_rate_req *req,
			       unsigned long *dvco_rate)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	u32 val;
	int scale;
	unsigned long rate;

	rate = req->rate;

	/* Determine DFLL output scale */
	req->scale = SCALE_MAX - 1;
	if (rate < td->dvco_rate_min) {
		scale = DIV_ROUND_CLOSEST((rate / 1000 * SCALE_MAX),
					  (td->dvco_rate_min / 1000));
		if (!scale) {
			dev_err(&pdev->dev,
				"rate %lu is below scalable range\n",
				rate);
			return -EINVAL;
		}
		req->scale = scale - 1;
		rate = td->dvco_rate_min;
	}

	/* Convert requested rate into frequency request and scale settings */
	val = GET_REQUEST_FREQ(rate, td->ref_rate);
	if (val > FREQ_MAX) {
		dev_err(&pdev->dev, "rate %lu is above dfll range\n", rate);
		return -EINVAL;
	}
	req->freq = val;

	*dvco_rate = GET_REQUEST_RATE(val, td->ref_rate);

	return 0;
}

/**
 * tegra_dfll_request_rate - set the next rate for the DFLL to tune to
 * @pdev: DFLL instance
 * @rate: clock rate to target
 *
 * Convert the requested clock rate @rate into the DFLL control logic
 * settings. In closed-loop mode, update new settings immediately to
 * adjust DFLL output rate accordingly.  Otherwise, just save them
 * until the next switch to closed loop.  Returns 0 upon success,
 * -EPERM if the DFLL driver has not yet been initialized, or -EINVAL
 * if @rate is outside the DFLL's tunable range or if there is no safe
 * output voltage for @rate.
 */
static int tegra_dfll_request_rate(struct platform_device *pdev,
				   unsigned long rate)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct dfll_rate_req req;
	unsigned long flags, dvco_rate;
	int r;
	int ret = 0;

	spin_lock_irqsave(&td->lock, flags);

	req.rate = rate;

	if (td->mode == TEGRA_DFLL_UNINITIALIZED) {
		dev_err(&pdev->dev, "cannot set DFLL rate in %s mode\n",
			mode_name[td->mode]);
		ret = -EPERM;
		goto tdrr_exit;
	}

	/* Calibrate dfll minimum rate */
	tegra_dfll_calibrate(pdev);

	r = calc_req_scale_mult(pdev, &req, &dvco_rate);
	if (r)
		goto tdrr_exit;

	/* Find safe voltage for requested rate */
	if (find_safe_output(pdev, dvco_rate, &req.output)) {
		dev_err(&pdev->dev, "failed to find safe output for rate %lu\n",
			dvco_rate);
		ret = -EINVAL;
		goto tdrr_exit;
	}
	req.cap = req.output;

	/*
	 * Save validated request, and in CLOSED_LOOP mode actually update
	 * control logic settings; use request output to set maximum voltage
	 * limit, but keep one LUT step room above safe voltage
	 */
	td->last_req = req;

	if (td->mode == TEGRA_DFLL_CLOSED_LOOP)
		reprogram_last_request(pdev);

tdrr_exit:
	spin_unlock_irqrestore(&td->lock, flags);

	return ret;
}

/**
 * calc_dfll_request_rate - return the approximate DFLL output clock rate
 * @pdev: DFLL instance
 * @req: ptr to a DFLL-rate-request structure record
 *
 * Return the approximate DFLL output clock rate that @req would
 * generate, after the scaling skipper.
 */
static unsigned long calc_dfll_request_rate(struct platform_device *pdev,
					    struct dfll_rate_req *req)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long rate;

	if ((req->scale + 1) < SCALE_MAX)
		rate = (req->rate / 1000) * 1000;
	else
		rate = GET_REQUEST_RATE(req->freq, td->ref_rate);

	return rate;
}

/**
 * tegra_dfll_request_get - return the DFLL's target rate from the last request
 * @pdev: DFLL instance
 *
 * If the DFLL is currently in closed-loop mode, return the clock rate
 * that the DFLL is targeting.  If the DFLL is currently in open-loop
 * mode or is disabled, return the clock rate that the DFLL will
 * target when it next enters closed-loop mode.  The rates come from
 * the driver's cache of the last request, not the hardware.  Note
 * that the actual DFLL output rate may not exactly match the targeted rate,
 * and may vary.  Returns the rate in Hz.
 */
static unsigned long tegra_dfll_request_get(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct dfll_rate_req *req = &td->last_req;
	unsigned long flags;
	u32 rate;

	spin_lock_irqsave(&td->lock, flags);
	rate = calc_dfll_request_rate(pdev, req);
	spin_unlock_irqrestore(&td->lock, flags);

	return rate;
}

/*
 * Clock framework integration
 */

static int tegra_dfll_clk_is_enabled(struct clk_hw *hw)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct tegra_dfll *td = dev_get_drvdata(&tdc->pdev->dev);

	return (td->mode == TEGRA_DFLL_OPEN_LOOP ||
		td->mode == TEGRA_DFLL_CLOSED_LOOP);
}

/**
 * tegra_dfll_clk_prepare - enable the DFLL clocks
 * @hw: DFLL instance struct clk_hw *
 *
 * Enable the DFLL clocks.  XXX This is a temporary workaround until
 * the clock framework gets multiple-parent or re-entrancy support.
 */
static int tegra_dfll_clk_prepare(struct clk_hw *hw)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;

	dfll_enable_clocks(pdev);
	return 0;
}

/**
 * tegra_dfll_clk_unprepare - disable the DFLL clocks
 * @hw: DFLL instance struct clk_hw *
 *
 * Disable the DFLL clocks.  XXX This is a temporary workaround until
 * the clock framework gets multiple-parent or re-entrancy support.
 */
static void tegra_dfll_clk_unprepare(struct clk_hw *hw)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;

	/* XXX Validate the current state of the DFLL first */

	dfll_disable_clocks(pdev);
}

static int tegra_dfll_clk_enable(struct clk_hw *hw)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;

	/* XXX Validate the current state of the DFLL first */

	return tegra_dfll_enable(pdev);
}

static void tegra_dfll_clk_disable(struct clk_hw *hw)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;

	/* XXX Validate the current state of the DFLL first */

	tegra_dfll_disable(pdev);
}

static unsigned long tegra_dfll_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;

	/* XXX Validate the current state of the DFLL first */

	return tegra_dfll_request_get(pdev);
}

static long tegra_dfll_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;
	struct dfll_rate_req req;
	int r;
	unsigned long dvco_rate;

	req.rate = rate;
	r = calc_req_scale_mult(pdev, &req, &dvco_rate);
	if (r)
		return r;

	return calc_dfll_request_rate(pdev, &req);
}

static int tegra_dfll_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct tegra_dfll_clk_hw *tdc = to_tegra_dfll_clk_hw(hw);
	struct platform_device *pdev = tdc->pdev;

	return tegra_dfll_request_rate(pdev, rate);
}

static const struct clk_ops tegra_dfll_clk_ops = {
	.is_enabled = tegra_dfll_clk_is_enabled,
	.prepare = tegra_dfll_clk_prepare,
	.unprepare = tegra_dfll_clk_unprepare,
	.enable = tegra_dfll_clk_enable,
	.disable = tegra_dfll_clk_disable,
	.recalc_rate = tegra_dfll_clk_recalc_rate,
	.round_rate = tegra_dfll_clk_round_rate,
	.set_rate = tegra_dfll_clk_set_rate,
};

static const char *const dfll_clk_parents[] = { "dfll_soc" };

static const struct clk_init_data dfll_clk_init_data = {
	.name		= DFLL_OUTPUT_CLOCK_NAME,
	.ops		= &tegra_dfll_clk_ops,
	.parent_names	= (const char **)dfll_clk_parents,
	.num_parents	= ARRAY_SIZE(dfll_clk_parents),
};

/**
 * register_dfll_clk - register the DFLL output clock with the clock framework
 * @pdev: DFLL instance
 *
 * Register the DFLL's output clock (i.e., the clock source that the
 * DVCO generates) with the Linux clock framework.  Returns 0
 * upon success or -EINVAL upon failure.
 */
static int register_dfll_clk(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int ret;

	td->dfll_clk_hw.pdev = pdev;
	td->dfll_clk_hw.hw.init = &dfll_clk_init_data;

	td->dfll_clk = clk_register(&pdev->dev, &td->dfll_clk_hw.hw);
	if (IS_ERR(td->dfll_clk)) {
		dev_err(&pdev->dev, "DFLL clock registration error\n");
		return -EINVAL;
	}

	ret = clk_register_clkdev(td->dfll_clk, DFLL_OUTPUT_CLOCK_NAME, NULL);
	if (ret) {
		dev_err(&pdev->dev, "DFLL clkdev registration error\n");
		goto rdc_err;
	}

	return 0;
rdc_err:
	clk_unregister(td->dfll_clk);
	return ret;
}

/*
 * Debugfs interface
 */

#ifdef CONFIG_DEBUG_FS

static int enable_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	*val = (td->mode == TEGRA_DFLL_CLOSED_LOOP ||
	      td->mode == TEGRA_DFLL_OPEN_LOOP);
	return 0;
}
static int enable_set(void *data, u64 val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	if (val)
		clk_prepare_enable(td->dfll_clk);
	else
		clk_disable_unprepare(td->dfll_clk);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, enable_get, enable_set, "%llu\n");

static int lock_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	*val = td->mode == TEGRA_DFLL_CLOSED_LOOP;
	return 0;
}
static int lock_set(void *data, u64 val)
{
	struct platform_device *pdev = data;
	int r;

	if (val)
		r = tegra_dfll_lock(pdev);
	else
		r = tegra_dfll_unlock(pdev);

	if (r)
		dev_err(&pdev->dev, "lock_set %llu returned %d\n", val, r);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lock_fops, lock_get, lock_set, "%llu\n");

static int monitor_get(void *data, u64 *val)
{
	u32 v, s;
	unsigned long flags;
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	clk_enable(td->soc_clk);

	spin_lock_irqsave(&td->lock, flags);

	v = dfll_readl(td, DFLL_MONITOR_DATA) & DFLL_MONITOR_DATA_VAL_MASK;

	if (dfll_readl(td, DFLL_MONITOR_CTRL) == DFLL_MONITOR_CTRL_FREQ) {
		v = GET_MONITORED_RATE(v, td->ref_rate);
		s = dfll_readl(td, DFLL_FREQ_REQ);
		s = (s & DFLL_FREQ_REQ_SCALE_MASK) >> DFLL_FREQ_REQ_SCALE_SHIFT;
		*val = (u64)v * (s + 1) / 256;
	} else {
		*val = v;
	}

	spin_unlock_irqrestore(&td->lock, flags);

	clk_disable(td->soc_clk);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(monitor_fops, monitor_get, NULL, "%llu\n");

static int target_rate_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;

	*val = tegra_dfll_request_get(pdev);
	return 0;
}
static int target_rate_set(void *data, u64 val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	return clk_set_rate(td->dfll_clk, val);
}
DEFINE_SIMPLE_ATTRIBUTE(target_rate_fops, target_rate_get, target_rate_set,
			"%llu\n");

static int vmax_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	*val = td->out_map[td->lut_max]->reg_uv / 1000;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vmax_fops, vmax_get, NULL, "%llu\n");

static int vmin_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	*val = td->out_map[td->lut_min]->reg_uv / 1000;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vmin_fops, vmin_get, NULL, "%llu\n");

static int tune_high_mv_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	*val = td->tune_high_min_mv;
	return 0;
}
static int tune_high_mv_set(void *data, u64 val)
{
	unsigned long flags;
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int r, ret;

	spin_lock_irqsave(&td->lock, flags);

	td->tune_high_min_mv = val;
	r = dfll_init_output_thresholds(pdev);
	if (r) {
		dev_err(&pdev->dev, "could not set output thresholds\n");
		ret = -EINVAL;
		goto thms_err;
	}

	if (td->mode == TEGRA_DFLL_CLOSED_LOOP)
		reprogram_last_request(pdev);

	ret = 0;
thms_err:
	spin_unlock_irqrestore(&td->lock, flags);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(tune_high_mv_fops, tune_high_mv_get, tune_high_mv_set,
			"%llu\n");

static int fmin_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	*val = td->dvco_rate_min;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dvco_rate_min_fops, fmin_get, NULL, "%llu\n");

static int calibr_delay_get(void *data, u64 *val)
{
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	*val = jiffies_to_msecs(td->calibration_delay);
	return 0;
}
static int calibr_delay_set(void *data, u64 val)
{
	unsigned long flags;
	struct platform_device *pdev = data;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	spin_lock_irqsave(&td->lock, flags);
	td->calibration_delay = msecs_to_jiffies(val);
	spin_unlock_irqrestore(&td->lock, flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(calibr_delay_fops, calibr_delay_get, calibr_delay_set,
			"%llu\n");

static int cl_register_show(struct seq_file *s, void *data)
{
	unsigned long flags;
	u32 offs;
	struct platform_device *pdev = s->private;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	clk_enable(td->soc_clk);

	spin_lock_irqsave(&td->lock, flags);

	seq_puts(s, "CONTROL REGISTERS:\n");
	for (offs = 0; offs <= DFLL_MONITOR_DATA; offs += 4)
		seq_printf(s, "[0x%02x] = 0x%08x\n",
			   offs, dfll_readl(td, offs));

	seq_puts(s, "\nI2C and INTR REGISTERS:\n");
	for (offs = DFLL_I2C_CFG; offs <= DFLL_I2C_STS; offs += 4)
		seq_printf(s, "[0x%02x] = 0x%08x\n",
			   offs, dfll_readl(td, offs));

	offs = DFLL_INTR_STS;
	seq_printf(s, "[0x%02x] = 0x%08x\n", offs, dfll_readl(td, offs));
	offs = DFLL_INTR_EN;
	seq_printf(s, "[0x%02x] = 0x%08x\n", offs, dfll_readl(td, offs));
	offs = DFLL_I2C_CLK_DIVISOR;
	seq_printf(s, "[0x%02x] = 0x%08x\n", offs, dfll_readl(td, offs));

	seq_puts(s, "\nLUT:\n");
	for (offs = DFLL_OUTPUT_LUT;
	     offs < DFLL_OUTPUT_LUT + 4 * MAX_DFLL_VOLTAGES;
	     offs += 4)
		seq_printf(s, "[0x%02x] = 0x%08x\n",
			   offs, dfll_readl(td, offs));

	spin_unlock_irqrestore(&td->lock, flags);

	clk_disable(td->soc_clk);
	return 0;
}

static int cl_register_open(struct inode *inode, struct file *file)
{
	return single_open(file, cl_register_show, inode->i_private);
}

static ssize_t cl_register_write(struct file *file,
				 const char __user *userbuf, size_t count,
				 loff_t *ppos)
{
	struct platform_device *pdev = file->f_path.dentry->d_inode->i_private;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	char buf[80];
	u32 offs, val;

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	/*
	 * terminate buffer and trim - white spaces may be appended at
	 * the end when invoked from shell command line
	 */
	buf[count] = '\0';
	strim(buf);

	if (sscanf(buf, "[0x%x] = 0x%x", &offs, &val) != 2)
		return -1;

	clk_enable(td->soc_clk);
	dfll_writel(td, val, offs & (~0x3));
	clk_disable(td->soc_clk);
	return count;
}

static const struct file_operations cl_register_fops = {
	.open		= cl_register_open,
	.read		= seq_read,
	.write		= cl_register_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tegra_dfll_debug_init(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int ret;

	if (!td || (td->mode == TEGRA_DFLL_UNINITIALIZED))
		return 0;

	td->dent = debugfs_create_dir(DRIVER_NAME, NULL);
	if (!td->dent)
		return -ENOMEM;

	ret = -ENOMEM;

	if (!debugfs_create_file("enable", S_IRUGO | S_IWUSR,
				 td->dent, pdev, &enable_fops))
		goto err_out;

	if (!debugfs_create_file("lock", S_IRUGO | S_IWUSR,
				 td->dent, pdev, &lock_fops))
		goto err_out;

	if (!debugfs_create_file("monitor", S_IRUGO,
				 td->dent, pdev, &monitor_fops))
		goto err_out;

	if (!debugfs_create_file("target_rate", S_IRUGO,
				 td->dent, pdev, &target_rate_fops))
		goto err_out;

	if (!debugfs_create_file("vmax_mv", S_IRUGO,
				 td->dent, pdev, &vmax_fops))
		goto err_out;

	if (!debugfs_create_file("vmin_mv", S_IRUGO,
				 td->dent, pdev, &vmin_fops))
		goto err_out;

	if (!debugfs_create_file("tune_high_mv", S_IRUGO, td->dent, pdev,
				 &tune_high_mv_fops))
		goto err_out;

	if (!debugfs_create_file("dvco_min", S_IRUGO, td->dent, pdev,
				 &dvco_rate_min_fops))
		goto err_out;

	if (!debugfs_create_file("calibr_delay", S_IRUGO, td->dent, pdev,
				 &calibr_delay_fops))
		goto err_out;

	if (!debugfs_create_file("registers", S_IRUGO | S_IWUSR,
				 td->dent, pdev, &cl_register_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(td->dent);
	return ret;
}

#endif		/* CONFIG_DEBUG_FS */

/*
 * Interface for the thermal reaction driver and thermal zone
 */

/**
 * tegra124_dfll_update_thermal_index - tell the DFLL how hot it is
 * @pdev: DFLL instance
 * @type: type of thermal floor or cap
 * @new_idx: current DFLL temperature index (into therm_floors)
 *
 * Update the DFLL driver's sense of what temperature the DFLL is
 * running at.  @new_idx is an index into td->therm_floors - provided
 * earlier to the thermal cooling driver.  Intended to be called by
 * the function supplied to the struct
 * thermal_cooling_device_ops.set_cur_state function pointer.  Returns
 * 0 upon success or -ERANGE if @new_idx is out of range.
 */
int tegra124_dfll_update_thermal_index(struct platform_device *pdev,
				       enum tegra_dfll_therm_type type,
				       unsigned long new_idx)
{
	unsigned long flags;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	if (type == TEGRA_DFLL_THERM_FLOOR) {
		if (new_idx > td->therm_floors_num)
			return -ERANGE;

		spin_lock_irqsave(&td->lock, flags);

		td->therm_floors_idx = new_idx;
		set_dvco_rate_min(pdev);
		if (td->mode == TEGRA_DFLL_CLOSED_LOOP)
			reprogram_last_request(pdev);

		spin_unlock_irqrestore(&td->lock, flags);
	} else if (type == TEGRA_DFLL_THERM_CAP) {
		if (new_idx > td->therm_caps_num)
			return -ERANGE;

		spin_lock_irqsave(&td->lock, flags);

		td->therm_caps_idx = new_idx;
		if (td->mode == TEGRA_DFLL_CLOSED_LOOP)
			reprogram_last_request(pdev);

		spin_unlock_irqrestore(&td->lock, flags);
	}

	return 0;
}
EXPORT_SYMBOL(tegra124_dfll_update_thermal_index);

/**
 * tegra124_dfll_get_thermal_index - return the DFLL's current thermal state
 * @pdev: DFLL instance
 * @type: type of thermal floor or cap
 *
 * Return the DFLL driver's copy of the DFLL's current temperature
 * index, set by tegra124_dfll_update_thermal_index().  Intended to be
 * called by the function supplied to the struct
 * thermal_cooling_device_ops.get_cur_state function pointer.
 */
int tegra124_dfll_get_thermal_index(struct platform_device *pdev,
				    enum tegra_dfll_therm_type type)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	if (type == TEGRA_DFLL_THERM_FLOOR)
		return td->therm_floors_idx;
	else if (type == TEGRA_DFLL_THERM_CAP)
		return td->therm_caps_idx;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(tegra124_dfll_get_thermal_index);

/**
 * tegra124_dfll_count_therm_states - return the number of thermal states
 * @pdev: DFLL instance
 * @type: type of thermal floor or cap
 *
 * Return the number of thermal states passed into the DFLL driver
 * from the DT data.  Intended to be called by the function supplied
 * to the struct thermal_cooling_device_ops.get_max_state function
 * pointer, and by the integration code that binds a thermal zone to
 * the DFLL thermal reaction driver.
 */
int tegra124_dfll_count_therm_states(struct platform_device *pdev,
				     enum tegra_dfll_therm_type type)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	if (type == TEGRA_DFLL_THERM_FLOOR)
		return td->therm_floors_num;
	else if (type == TEGRA_DFLL_THERM_CAP)
		return td->therm_caps_num;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(tegra124_dfll_count_therm_states);

/**
 * tegra124_dfll_get_therm_state_temp - return the state temp at @index
 * @pdev: DFLL instance
 * @type: type of thermal floor or cap
 * @index: index into the therm_floors or therm_caps array (zero-based)
 *
 * Return the temperature corresponding to @index from the therm_floors or
 * therm_caps array.  Intended to be called by integration code
 * binding a thermal zone to the DFLL thermal reaction driver.
 * Returns -ERANGE if @index would result in an access off the end of
 * the array, or returns the temperature corresponding to @index in
 * degrees Celsius.
 */
int tegra124_dfll_get_therm_state_temp(struct platform_device *pdev,
				       enum tegra_dfll_therm_type type,
				       unsigned long index)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	if (type == TEGRA_DFLL_THERM_FLOOR) {
		if (index >= td->therm_floors_num)
			return -ERANGE;
		return td->therm_floors[index].temp;
	} else if (type == TEGRA_DFLL_THERM_CAP) {
		if (index >= td->therm_caps_num)
			return -ERANGE;
		return td->therm_caps[index].temp;
	} else {
		return -EINVAL;
	}
}
EXPORT_SYMBOL(tegra124_dfll_get_therm_state_temp);

/**
 * tegra124_dfll_attach_thermal - attach the "cooling device" @cdev to @pdev
 * @pdev: DFLL instance
 * @type: type of thermal floor or cap
 * @cdev: DFLL "cooling device" instance
 *
 * Attach a thermal reaction "cooling device" to the DFLL instance
 * @pdev.  As long as a thermal reaction driver is attached, the DFLL
 * driver can't be unbound from the DFLL device.  Returns -EINVAL if
 * the DFLL instance hasn't been initialized yet, -EBUSY if a thermal
 * driver is already associated with @pdev, or 0 upon success.
 */
int tegra124_dfll_attach_thermal(struct platform_device *pdev,
				 enum tegra_dfll_therm_type type,
				 struct thermal_cooling_device *cdev)
{
	unsigned long flags;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int ret = -EINVAL;

	if (!td)
		return ret;

	spin_lock_irqsave(&td->lock, flags);

	if (td->mode == TEGRA_DFLL_UNINITIALIZED) {
		dev_err(&pdev->dev, "DFLL not yet initialized\n");
		ret = -EINVAL;
		goto tdat_out;
	}

	if (type == TEGRA_DFLL_THERM_FLOOR) {
		if (td->cdev_floor) {
			dev_err(&pdev->dev,
				"DFLL floor already bound to thermal driver\n");
			ret = -EBUSY;
			goto tdat_out;
		}

		td->cdev_floor = cdev;
		ret = 0;
	} else if (type == TEGRA_DFLL_THERM_CAP) {
		if (td->cdev_cap) {
			dev_err(&pdev->dev,
				"DFLL cap already bound to thermal driver\n");
			ret = -EBUSY;
			goto tdat_out;
		}

		td->cdev_cap = cdev;
		ret = 0;
	} else {
		ret = -EINVAL;
	}

tdat_out:
	spin_unlock_irqrestore(&td->lock, flags);

	return ret;
}
EXPORT_SYMBOL(tegra124_dfll_attach_thermal);

/**
 * tegra124_dfll_detach_thermal - detach the "cooling device" @cdev from @pdev
 * @pdev: DFLL instance
 * @type: type of thermal floor or cap
 * @cdev: DFLL "cooling device" instance
 *
 * Detach a thermal reaction "cooling device" from the DFLL instance
 * @pdev.  Returns -EINVAL if @cdev isn't currently associated with
 * the DFLL instance, or 0 upon success.
 */
int tegra124_dfll_detach_thermal(struct platform_device *pdev,
				 enum tegra_dfll_therm_type type,
				 struct thermal_cooling_device *cdev)
{
	unsigned long flags;
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int ret = -EINVAL;

	spin_lock_irqsave(&td->lock, flags);

	if (type == TEGRA_DFLL_THERM_FLOOR) {
		if (td->cdev_floor != cdev) {
			dev_err(&pdev->dev, "floor cooling device mismatch\n");
			ret = -EINVAL;
			goto tddt_out;
		}

		td->cdev_floor = NULL;
		td->therm_floors_idx = 0;
		ret = 0;
	} else if (type == TEGRA_DFLL_THERM_CAP) {
		if (td->cdev_cap != cdev) {
			dev_err(&pdev->dev, "cap cooling device mismatch\n");
			ret = -EINVAL;
			goto tddt_out;
		}

		td->cdev_cap = NULL;
		td->therm_caps_idx = 0;
		ret = 0;
	} else {
		ret = -EINVAL;
	}

tddt_out:
	spin_unlock_irqrestore(&td->lock, flags);

	return ret;
}
EXPORT_SYMBOL(tegra124_dfll_detach_thermal);

/*
 * Interface to lock and unlock DFLL loop
 */

/**
 * tegra124_dfll_lock_loop - enable closed loop
 *
 * The wrapper of tegra_dfll_lock(). Return -EPERM if the static DFLL
 * platform device instance does not get initialized.
 */
int tegra124_dfll_lock_loop(void)
{
	if (!fcpu_dfll_pdev) {
		WARN(1, "Tegra124 DFLL is not initialized\n");
		return -EPERM;
	}

	return tegra_dfll_lock(fcpu_dfll_pdev);
}
EXPORT_SYMBOL(tegra124_dfll_lock_loop);

/**
 * tegra124_dfll_unlock_loop - disable closed loop
 *
 * The wrapper of tegra_dfll_lock(). Return -EPERM if the static DFLL
 * platform device instance does not get initialized.
 */
int tegra124_dfll_unlock_loop(void)
{
	if (!fcpu_dfll_pdev) {
		WARN(1, "Tegra124 DFLL is not initialized\n");
		return -EPERM;
	}

	return tegra_dfll_unlock(fcpu_dfll_pdev);
}
EXPORT_SYMBOL(tegra124_dfll_unlock_loop);

/**
 * tegra124_dfll_get_fv_table - get freq/volt table for cpug
 *
 * @num_freqs: number of frequencies
 * @freqs: the array of frequencies
 * @millivolts: the array of voltages
 */
int tegra124_dfll_get_fv_table(int *num_freqs, unsigned long **freqs,
		int **millivolts)
{
	struct tegra_dfll *td;

	if (!fcpu_dfll_pdev)
		return -EPERM;

	if (!num_freqs || !freqs || !millivolts)
		return -EINVAL;

	td = dev_get_drvdata(&fcpu_dfll_pdev->dev);
	*num_freqs = td->dvfs_info->num_freqs;
	*freqs = td->dvfs_info->freqs;
	*millivolts = td->dvfs_info->cpu_dfll_millivolts;

	return 0;
}
EXPORT_SYMBOL(tegra124_dfll_get_fv_table);

/*
 * DFLL initialization
 */

/**
 * tegra_dfll_init_clks - clk_get() the DFLL source clocks
 * @pdev: DFLL instance
 *
 * Call clk_get() on the DFLL source clocks and save the pointers for later
 * use.  Returns 0 upon success or -ENODEV if one or more of the clocks
 * couldn't be looked up.
 */
static int tegra_dfll_init_clks(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int ret;

	td->ref_clk = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(td->ref_clk)) {
		dev_err(&pdev->dev, "missing ref clock\n");
		ret = -ENODEV;
		goto tdic_err;
	}

	td->soc_clk = devm_clk_get(&pdev->dev, "soc");
	if (IS_ERR(td->soc_clk)) {
		dev_err(&pdev->dev, "missing soc clock\n");
		ret = -ENODEV;
		goto tdic_err;
	}

	td->i2c_clk = devm_clk_get(&pdev->dev, "i2c");
	if (IS_ERR(td->i2c_clk)) {
		dev_err(&pdev->dev, "missing i2c clock\n");
		ret = -ENODEV;
		goto tdic_err;
	}

	return 0;

tdic_err:
	return ret;
}

/**
 * tegra_dfll_regulator_probe_voltages - build vdd_map[] from the regulator
 * @pdev: DFLL instance
 *
 * Build the vdd_map from regulator framework and DFLL DT data.
 * Returns 0 upon success, or -ENOSPC on a memory allocation failure.
 */
static int tegra_dfll_regulator_probe_voltages(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int c, i, j, vdd_uv;
	struct tegra_dfll_voltage_reg_map *vdd_map;

	c = regulator_count_voltages(td->vdd);
	if (c < 0)
		return c;

	vdd_map = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_dfll_voltage_reg_map) * c,
			GFP_KERNEL);
	if (!vdd_map)
		return -ENOMEM;

	j = 0;
	for (i = 0; i < c; i++) {
		vdd_uv = regulator_list_voltage(td->vdd, i);
		if (vdd_uv <= 0)
			continue;

		if (vdd_uv < td->dfll_min_microvolt)
			continue;

		if (vdd_uv > td->dfll_max_microvolt)
			break;

		vdd_map[j].reg_value = i;
		vdd_map[j].reg_uv = vdd_uv;
		j++;
	}

	td->vdd_map_size = j;
	td->vdd_map = vdd_map;

	return 0;
}

/*
 * DT data fetch
 */

/**
 * parse_of_dfll_pmic_integration - find and extract the DT DFLL-PMIC data
 * @pdev: DFLL instance
 * @dn: DT node of the DFLL instance data
 *
 * Read the DFLL PMIC integration data from the DT node with the
 * "nvidia,tegra124-dfll,pmic-integration" property.  This data
 * includes the PMIC I2C address, the I2C bus speed, the I2C voltage
 * register address.  Returns 0 upon success or -EINVAL upon error.
 */
static int parse_of_dfll_pmic_integration(struct platform_device *pdev,
					  struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	const __be32 *prop;
	struct device_node *p_dn;

	prop = of_get_property(dn, "pmic-integration", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"pmic-integration");
		return -EINVAL;
	}

	p_dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!p_dn) {
		dev_err(&pdev->dev, "missing DFLL PMIC integration DT data\n");
		return -EINVAL;
	}

	if (of_property_read_u32(p_dn, "pmic-i2c-address",
				 &td->pmic_i2c_addr)) {
		dev_err(&pdev->dev, "missing PMIC I2C address\n");
		return -EINVAL;
	}

	if (of_property_read_u32(p_dn, "pmic-i2c-voltage-register",
				 &td->pmic_i2c_voltage_reg)) {
		dev_err(&pdev->dev,
			"missing PMIC I2C voltage register address\n");
		return -EINVAL;
	}

	if (of_property_read_u32(p_dn, "dfll-min-microvolt",
				 &td->dfll_min_microvolt)) {
		dev_err(&pdev->dev,
			"missing DFLL regulator minimum voltage\n");
		return -EINVAL;
	}

	if (of_property_read_u32(p_dn, "dfll-max-microvolt",
				 &td->dfll_max_microvolt)) {
		dev_err(&pdev->dev,
			"missing DFLL regulator maximum voltage\n");
		return -EINVAL;
	}

	of_property_read_u32(p_dn, "i2c-hs-rate", &td->pmic_i2c_hs_rate);

	if ((td->pmic_i2c_hs_rate > 0) &&
	    of_property_read_u32(p_dn, "i2c-hs-master-code",
				 &td->pmic_i2c_hs_master_code)) {
		dev_err(&pdev->dev, "missing I2C HS master code\n");
		return -EINVAL;
	}

	td->pmic_i2c_ten_bit_addrs = !!of_get_property(p_dn,
						       "i2c-10-bit-addresses",
						       NULL);

	if (of_property_read_u32(p_dn, "i2c-fs-rate", &td->pmic_i2c_fs_rate)) {
		dev_err(&pdev->dev, "missing I2C FS bus rate\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * parse_of_dfll_board_data - find and extract the DFLL per-board parameters
 * @pdev: DFLL instance
 * @dn: DT node of the DFLL instance data
 *
 * Read the DFLL per-board data from the DT node with the
 * "nvidia,tegra124-dfll,board-data" compatible property.  This data
 * is the result of the DFLL characterization process for a particular
 * board design.  It includes loop and voltage droop avoidance
 * parameters.  It does not include PMIC integration; see
 * parse_of_dfll_pmic_integration() for that.  Returns 0 upon success
 * or -EINVAL upon error.
 */
static int parse_of_dfll_board_data(struct platform_device *pdev,
				    struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct device_node *b_dn;
	int fixed_forcing, auto_forcing, no_forcing;
	int i = 0, ret = 0;
	const __be32 *prop;
	u32 dtp;

	prop = of_get_property(dn, "board-params", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "board-params");
		return -EINVAL;
	}

	b_dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!b_dn) {
		dev_err(&pdev->dev, "missing DFLL board DT data\n");
		return -EINVAL;
	}

	if (of_property_read_u32(b_dn, "sample-rate", &td->sample_rate)) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "sample-rate");
		return -EINVAL;
	}

	if (of_property_read_u32(b_dn, "cf", &td->cf)) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "cf");
		return -EINVAL;
	}

	if (of_property_read_u32(b_dn, "ci", &td->ci)) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "ci");
		return -EINVAL;
	}

	if (of_property_read_u32(b_dn, "cg", &td->cg)) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "cg");
		return -EINVAL;
	}

	dtp = !!of_get_property(b_dn, "cg-scale", NULL);
	if (dtp)
		td->cg_scale = 1;

	if (of_property_read_u32(b_dn, "droop-cut-value",
				 &td->droop_cut_value)) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"droop-cut-value");
		return -EINVAL;
	}

	if (of_property_read_u32(b_dn, "droop-restore-ramp",
				 &td->droop_restore_ramp)) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"droop-restore-ramp");
		return -EINVAL;
	}

	if (of_property_read_u32(b_dn, "scale-out-ramp", &td->scale_out_ramp)) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"scale-out-ramp");
		return -EINVAL;
	}

	fixed_forcing = !!of_get_property(b_dn, "fixed-output-forcing", NULL);
	auto_forcing = !!of_get_property(b_dn, "auto-output-forcing", NULL);
	no_forcing = !!of_get_property(b_dn, "no-output-forcing", NULL);
	if (fixed_forcing) {
		td->force_mode = TEGRA_DFLL_FORCE_FIXED;
		i++;
	}
	if (auto_forcing) {
		td->force_mode = TEGRA_DFLL_FORCE_AUTO;
		i++;
	}
	if (no_forcing) {
		td->force_mode = TEGRA_DFLL_FORCE_NONE;
		i++;
	}
	if (i != 1) {
		dev_err(&pdev->dev,
			"must specify one and only one forcing param in DT\n");
		return -EINVAL;
	}

	return ret;
}

/**
 * parse_of_dfll_tuning - read the DFLL tuning data from DT
 * @pdev: DFLL instance
 * @dn: DT node of the DFLL instance data
 *
 * Read the DFLL tuning data from DT.  Returns 0 upon success, -EINVAL
 * if required data fields are missing, or -ENOENT if no DFLL tuning
 * node could be found that matches the current chip's Speedo ID and
 * process id.
 */
static int parse_of_dfll_tuning(struct platform_device *pdev,
				struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct device_node *t_dn, *tc_dn;
	const __be32 *prop;
	u32 speedo_id, process_id;

	prop = of_get_property(dn, "tuning", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "tuning");
		return -EINVAL;
	}

	t_dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!t_dn) {
		dev_err(&pdev->dev, "missing DFLL tuning DT data\n");
		return -EINVAL;
	}

	/* Read the loop params */
	for_each_child_of_node(t_dn, tc_dn) {
		if (of_property_read_u32(tc_dn, "speedo-id", &speedo_id)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"speedo-id");
			return -EINVAL;
		}

		if (speedo_id != td->speedo_id)
			continue;

		if (!of_property_read_u32(tc_dn, "process-id", &process_id))
			if (process_id != td->process_id)
				continue;

		if (of_property_read_u32(tc_dn, "tune0-low-voltage-range",
					 &td->tune0_low_voltage_range)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"tune0-low-voltage-range");
			return -EINVAL;
		}

		if (of_property_read_u32(tc_dn, "tune0-high-voltage-range",
					 &td->tune0_high_voltage_range)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"tune0-high-voltage-range");
			return -EINVAL;
		}

		if (of_property_read_u32(tc_dn, "tune1", &td->tune1)) {
			dev_err(&pdev->dev, "missing %s in DT data\n", "tune1");
			return -EINVAL;
		}

		if (of_property_read_u32(tc_dn, "droop-rate-min",
					 &td->droop_rate_min)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"droop-rate-min");
			return -EINVAL;
		}

		if (of_property_read_u32(tc_dn, "min-millivolts",
					 &td->min_millivolts)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"min-millivolts");
			return -EINVAL;
		}

		of_property_read_u32(tc_dn, "tune-high-min-millivolts",
				     &td->tune_high_min_mv);

		return 0;
	}

	return -ENOENT;
}

/**
 * parse_of_cvbs - parse the per-frequency voltage curve data
 * @pdev: DFLL instance
 * @dn: struct device_node * to the "cvb" DT node
 *
 * Read the CVB per-frequency data from DT.  Other code walks this
 * table and builds a "cooked" table, used during runtime.  Returns 0
 * upon success or -EINVAL upon error.
 */
static int parse_of_cvbs(struct platform_device *pdev, struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct tegra_dfll_cvb *tdc;
	struct property *prop;
	const __be32 *p;
	u32 u;
	int i, j, l, rem;

	prop = of_find_property(dn, "cvb-voltage-curves", &l);
	if (!prop || (l == 0)) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "cvb");
		return -EINVAL;
	}
	l /= sizeof(__be32);

	rem = l % CVB_ROW_WIDTH;
	if (rem > 0) {
		dev_err(&pdev->dev, "CVB data format error\n");
		return -EINVAL;
	}

	l /= CVB_ROW_WIDTH;
	tdc = devm_kzalloc(&pdev->dev, l * sizeof(struct tegra_dfll_cvb),
			GFP_KERNEL);
	if (!tdc)
		goto poct_err;

	i = 0;
	of_property_for_each_u32(dn, "cvb-voltage-curves", prop, p, u) {
		j = i / CVB_ROW_WIDTH;

		switch (i % CVB_ROW_WIDTH) {
		case 0:
			tdc[j].freq = u;
			break;
		case 1:
			tdc[j].c0 = u;
			break;
		case 2:
			tdc[j].c1 = u;
			break;
		case 3:
			tdc[j].c2 = u;
			break;
		}
		i++;
	}

	td->dvfs_info->cvb_table = tdc;
	td->dvfs_info->cvb_table_len = i / CVB_ROW_WIDTH;

	return 0;

poct_err:
	return -EINVAL;
}

/**
 * parse_of_cvb_table_meta - read the CVB table metadata from DT
 * @pdev: DFLL instance
 * @lpc_dn: 'characterization' DT node
 *
 * Read the CVB table metadata from the DeviceTree data.  Returns 0
 * upon success, or -EINVAL upon failure.
 */
static int parse_of_cvb_table_meta(struct platform_device *pdev,
				   struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	if (of_property_read_u32(dn, "cvb-max-millivolts",
				 &td->cvb_max_millivolts)) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"cvb-max-millivolts");
		return -EINVAL;
	}

	if (of_property_read_u32(dn, "cvb-speedo-scale",
				 &td->cvb_speedo_scale)) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"cvb-speedo-scale");
		return -EINVAL;
	}

	if (of_property_read_u32(dn, "cvb-voltage-scale",
				 &td->cvb_voltage_scale)) {
		dev_err(&pdev->dev, "missing %s in DT data\n",
			"cvb-voltage-scale");
		return -EINVAL;
	}

	return 0;
}

/**
 * parse_of_therm_caps - parse the thermal voltage caps
 * @pdev: DFLL instance
 * @dn: struct device_node * of the "characterization" DT node
 *
 * Read the array of u32s from the 'therm-caps' property, under the
 * DT device node @dn.  The first of the two values represents the
 * trip point temperature in degrees Celsius.  The second value
 * represents the maximum voltage cap for that temperature, and, if
 * there are no lower temperatures specified, any temperatures below
 * it.  Returns 0 if the 'therm-caps' property doesn't exist (it's
 * optional) or if the data was read successfully, or -EINVAL if
 * something is wrong with the DT data.
 */
static int parse_of_therm_caps(struct platform_device *pdev,
				 struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct property *prop;
	const __be32 *p;
	u32 u;
	int i, j;

	i = 0;
	of_property_for_each_u32(dn, "therm-caps", prop, p, u) {
		j = i / THERM_CAPS_ROW_WIDTH;
		if (j >= MAX_THERMAL_CAPS)
			break;

		if (!(i % THERM_CAPS_ROW_WIDTH)) {
			/* XXX Test u32 -> u8 overflow */
			td->therm_caps[j].temp = u;
		} else {
			td->therm_caps[j].mv = u;
		}
		i++;
	}

	if (i % THERM_CAPS_ROW_WIDTH) {
		dev_err(&pdev->dev, "therm-caps data format error\n");
		return -EINVAL;
	}

	td->therm_caps_num = i / THERM_CAPS_ROW_WIDTH;

	return 0;
}

/**
 * parse_of_therm_floors - parse the thermal voltage floors
 * @pdev: DFLL instance
 * @dn: struct device_node * of the "characterization" DT node
 *
 * Read the array of u32s from the 'therm-floors' property, under the
 * DT device node @dn.  The first of the two values represents the
 * trip point temperature in degrees Celsius.  The second value
 * represents the minimum voltage floor for that temperature, and, if
 * there are no lower temperatures specified, any temperatures below
 * it.  Returns 0 if the 'therm-floors' property doesn't exist (it's
 * optional) or if the data was read successfully, or -EINVAL if
 * something is wrong with the DT data.
 */
static int parse_of_therm_floors(struct platform_device *pdev,
				 struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct property *prop;
	const __be32 *p;
	u32 u;
	int i, j;

	i = 0;
	of_property_for_each_u32(dn, "therm-floors", prop, p, u) {
		j = i / THERM_FLOORS_ROW_WIDTH;
		if (j >= MAX_THERMAL_FLOORS)
			break;

		if (!(i % THERM_FLOORS_ROW_WIDTH)) {
			/* XXX Test u32 -> u8 overflow */
			td->therm_floors[j].temp = u;
		} else {
			td->therm_floors[j].mv = u;
		}
		i++;
	}

	if (i % THERM_FLOORS_ROW_WIDTH) {
		dev_err(&pdev->dev, "therm-floors data format error\n");
		return -EINVAL;
	}

	td->therm_floors_num = i / THERM_FLOORS_ROW_WIDTH;

	return 0;
}

/**
 * parse_of_cvb_table - read CVB table data from DeviceTree
 * @pdev: DFLL instance
 * @dn: DT node of the DFLL instance data
 *
 * Reads the CVB table data and CVB metadata that corresponds to the
 * current chip's Speedo and process ID.  Returns 0 upon success,
 * -EINVAL upon error, or -ENOENT if no CVB table node could be found
 * that matches the current chip's Speedo ID and process id.
 */
static int parse_of_cvb_table(struct platform_device *pdev,
			      struct device_node *dn)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	struct device_node *d_dn, *dc_dn;
	const __be32 *prop;
	u32 speedo_id, process_id;
	int r;

	prop = of_get_property(dn, "cvb-table", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "missing %s in DT data\n", "cvb-table");
		return -EINVAL;
	}

	d_dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!d_dn) {
		dev_err(&pdev->dev, "missing DFLL cvb table DT data\n");
		return -EINVAL;
	}

	for_each_child_of_node(d_dn, dc_dn) {
		if (of_property_read_u32(dc_dn, "speedo-id", &speedo_id)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"speedo-id");
			return -EINVAL;
		}

		if (speedo_id != td->speedo_id)
			continue;

		if (!of_property_read_u32(dc_dn, "process-id", &process_id))
			if (process_id != td->process_id)
				continue;

		if (parse_of_therm_caps(pdev, dc_dn)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"therm-caps");
			return -EINVAL;
		}

		if (parse_of_therm_floors(pdev, dc_dn)) {
			dev_err(&pdev->dev, "missing %s in DT data\n",
				"therm-floors");
			return -EINVAL;
		}

		r = parse_of_cvb_table_meta(pdev, dc_dn);
		if (r) {
			dev_err(&pdev->dev, "couldn't parse CVB table meta\n");
			return r;
		}

		r = parse_of_cvbs(pdev, dc_dn);
		if (r) {
			dev_err(&pdev->dev, "couldn't parse CVB table\n");
			return r;
		}

		return 0;
	}

	return -ENOENT;
}

/**
 * tegra_dfll_init_pmic_data - initialize PMIC regulator data
 * @pdev: DFLL instance
 *
 * Read the PMIC integration data, including regulator data, from DT
 * and the the regulator framework.  Build the voltage map from
 * regulator data.  Returns 0 upon success or -EINVAL upon error.
 */
static int __must_check tegra_dfll_init_pmic_data(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	int r;

	td->vdd = devm_regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(td->vdd)) {
		dev_err(&pdev->dev, "couldn't locate regulator\n");
		return -EINVAL;
	}

	td->vdd_step = regulator_get_linear_step(td->vdd);
	if (!td->vdd_step) {
		dev_err(&pdev->dev, "only linear map regulators supported\n");
		goto tdipd_err;
	}

	r = parse_of_dfll_pmic_integration(pdev, pdev->dev.of_node);
	if (r) {
		dev_err(&pdev->dev, "DFLL PMIC integration parse error\n");
		goto tdipd_err;
	}

	r = tegra_dfll_regulator_probe_voltages(pdev);
	if (r) {
		dev_err(&pdev->dev, "couldn't probe regulator voltages\n");
		goto tdipd_err;
	}

	return 0;

tdipd_err:
	return -EINVAL;
}

/*
 * platform_device integration
 */

/**
 * tegra_dfll_probe - probe the instance of the DFLL IP block
 * @pdev: DFLL instance
 *
 * Called when a DFLL device is bound to this driver by the driver
 * core.  Registers the DFLL output clock with the clock framework.
 * Returns 0 upon success, or -ENOMEM if memory couldn't be allocated,
 * -EINVAL if various calls fail, or can pass along the error returned
 * by several other functions.
 */
static int tegra_dfll_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct resource *mem;
	struct tegra_dfll *td;
	struct tegra_dfll_dvfs_info *dvfs_info;
	int ret = -EINVAL;
	int r;

	td = devm_kzalloc(&pdev->dev, sizeof(*td), GFP_KERNEL);
	if (!td)
		return -ENOMEM;

	dvfs_info = devm_kzalloc(&pdev->dev, sizeof(*dvfs_info), GFP_KERNEL);
	if (!dvfs_info)
		return -ENOMEM;

	td->dvfs_info = dvfs_info;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "missing register MMIO resource\n");
		ret = -EINVAL;
		goto tdp_err;
	}

	td->base = ioremap(mem->start, resource_size(mem));
	if (!td->base) {
		dev_err(&pdev->dev, "couldn't ioremap() register MMIO area\n");
		ret = -EINVAL;
		goto tdp_err;
	}

	td->speedo_id = tegra_get_cpu_speedo_id();
	td->process_id = tegra_get_cpu_process_id();
	td->speedo_value = tegra_get_cpu_speedo_value();

	platform_set_drvdata(pdev, td);

	r = tegra_dfll_init_clks(pdev);
	if (r) {
		dev_err(&pdev->dev, "DFLL clock init error\n");
		ret = r;
		goto tdp_err;
	}

	/* Set some parameters from the IP block's DT data */
	r = parse_of_cvb_table(pdev, dn);
	if (r) {
		dev_err(&pdev->dev, "DFLL CVB data parse error\n");
		ret = r;
		goto tdp_err;
	}

	r = parse_of_dfll_tuning(pdev, dn);
	if (r) {
		dev_err(&pdev->dev, "DFLL tuning data parse error\n");
		ret = r;
		goto tdp_err;
	}

	r = parse_of_dfll_board_data(pdev, dn);
	if (r) {
		dev_err(&pdev->dev, "DFLL board data parse error\n");
		ret = -EINVAL;
		goto tdp_err;
	}

	r = tegra_dfll_init_pmic_data(pdev);
	if (r) {
		dev_err(&pdev->dev, "DFLL PMIC data parse error\n");
		ret = r;
		goto tdp_err;
	}

	/* Enable the clocks and set the device up */
	ret = dfll_init(pdev);
	if (ret)
		goto tdp_err;

	fcpu_dfll_pdev = pdev;

	ret = register_dfll_clk(pdev);
	if (ret) {
		dev_err(&pdev->dev, "DFLL clk registration failed\n");
		goto tdp_err;
	}

#ifdef CONFIG_DEBUG_FS
	tegra_dfll_debug_init(pdev);
#endif

	dev_info(&pdev->dev, "Tegra T124 DFLL clock driver initialized\n");

	return 0;

tdp_err:
	fcpu_dfll_pdev = NULL;
	iounmap(td->base);
	return ret;
}

/**
 * tegra_dfll_remove - unbind the DFLL driver from the DFLL device
 * @pdev: DFLL instance
 *
 * Releases the DFLL platform_driver from the DFLL platform_device
 * @pdev.  The DFLL must be in the DISABLED state before this can be
 * successful.  Returns 0 upon success, or -EBUSY if the DFLL is still
 * generating a clock.
 */
static int tegra_dfll_remove(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);

	/* Try to prevent removal while the DFLL is active */
	if (td->mode != TEGRA_DFLL_DISABLED) {
		dev_err(&pdev->dev,
			"must disable DFLL before removing driver\n");
		return -EBUSY;
	}

	if (td->cdev_floor || td->cdev_cap) {
		dev_err(&pdev->dev,
			"must unload thermal driver before removing DFLL\n");
		return -EBUSY;
	}

	clk_unregister(td->dfll_clk);
	tegra124_clock_assert_dfll_dvco_reset();
	regulator_put(td->vdd);
	iounmap(td->base);
	kfree(td);
	fcpu_dfll_pdev = NULL;

	return 0;
}

/*
 * dfll controls clock/voltage to other devices, including CPU. Therefore,
 * dfll driver pm suspend callback does not stop cl-dvfs operations. It is
 * only used to enforce cold voltage limit, since SoC may cool down during
 * suspend without waking up. The correct temperature zone after supend will
 * be updated via dfll cooling device interface during resume of temperature
 * sensor.
 */
int tegra124_dfll_suspend(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	unsigned long flags;

	spin_lock_irqsave(&td->lock, flags);

	td->therm_caps_idx = td->therm_caps_num;
	td->therm_floors_idx = 0;
	set_dvco_rate_min(pdev);
	if (td->mode == TEGRA_DFLL_CLOSED_LOOP) {
		set_cl_config(pdev, &td->last_req);
		set_request(pdev, &td->last_req);
	}

	spin_unlock_irqrestore(&td->lock, flags);

	if (td->mode == TEGRA_DFLL_CLOSED_LOOP) {
		tegra_dfll_unlock(pdev);
		td->resume_mode = TEGRA_DFLL_CLOSED_LOOP;
	}

	return 0;
}

/**
 * tegra_dfll_resume - reprogram the DFLL after context-loss
 * @pdev: DFLL instance
 *
 * Re-initialize and enable target device clock in open loop mode. Called
 * directly from SoC clock resume syscore operation. Closed loop will be
 * re-entered in platform syscore ops as well.
 */
void tegra124_dfll_resume(struct platform_device *pdev)
{
	struct tegra_dfll *td = dev_get_drvdata(&pdev->dev);
	enum tegra_dfll_ctrl_mode mode = td->mode;
	struct dfll_rate_req req = td->last_req;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&td->lock, flags);

	tegra124_clock_deassert_dfll_dvco_reset();

	dfll_enable_clocks(pdev);

	/* Setup PMU interface, and configure controls in disabled mode */
	dfll_init_out_if(pdev);
	ret = dfll_init_cntrl_logic(pdev);
	if (ret)
		dev_err(&pdev->dev, "could not init the control logic\n");

	dfll_disable_clocks(pdev);

	/* Restore last request and mode */
	td->last_req = req;
	if (mode != TEGRA_DFLL_DISABLED) {
		set_mode(pdev, TEGRA_DFLL_OPEN_LOOP);
		WARN(mode > TEGRA_DFLL_OPEN_LOOP,
		     "DFLL was left locked in suspend\n");
	}

	spin_unlock_irqrestore(&td->lock, flags);

	if (td->resume_mode == TEGRA_DFLL_CLOSED_LOOP) {
		tegra_dfll_lock(pdev);
		td->resume_mode = TEGRA_DFLL_UNINITIALIZED;
	}
}

/* Match table for OF platform binding */
static struct of_device_id tegra_dfll_of_match[] = {
	{ .compatible = "nvidia,tegra124-dfll", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_dfll_of_match);

static struct platform_driver tegra_dfll_driver = {
	.probe		= tegra_dfll_probe,
	.remove		= tegra_dfll_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_dfll_of_match),
	},
};

static int __init tegra_dfll_driver_init(void)
{
	return platform_driver_register(&tegra_dfll_driver);
}

static void __exit tegra_dfll_driver_exit(void)
{
	platform_driver_unregister(&tegra_dfll_driver);
}

fs_initcall(tegra_dfll_driver_init);
module_exit(tegra_dfll_driver_exit);

MODULE_DESCRIPTION("Tegra DFLL clock source driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Aleksandr Frid <afrid@nvidia.com>");
MODULE_AUTHOR("Paul Walmsley <pwalmsley@nvidia.com>");
