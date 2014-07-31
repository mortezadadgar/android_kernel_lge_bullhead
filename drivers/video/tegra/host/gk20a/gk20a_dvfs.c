/*
 * gk20a DVFS driver
 *
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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/tegra-soc.h>

#include <linux/platform_data/tegra_throttle.h>

#include "clk_gk20a.h"
#include "dev.h"
#include "gk20a.h"
#include "gk20a_dvfs.h"
#include "nvhost_scale.h"

#define KHZ 1000
#define GPU_SUPPLY_NAME "vddgpu"
#define DEFAULT_STEP_UV 10000

/*
 * TODO: no thermal support yet, so just use the highest voltage in all
 * thermal ranges.
 */
#define SAFE_THERMAL_INDEX 0

static struct gk20a_dvfs gpu_dvfs;

static struct gk20a_cvb_info gpu_cvb_info_table[] = {
	{
		.speedo_id =  0,
		.process_id = -1,
		.max_mv = 1200,
		.min_mv = 800,
		.speedo_scale = 100,
		.thermal_scale = 10,
		.voltage_scale = 1000,
		.cvb_table = {
			{   72, { 1209886, -36468,  515,   417, -13123,  203}, },
			{  108, { 1130804, -27659,  296,   298, -10834,  221}, },
			{  180, { 1162871, -27110,  247,   238, -10681,  268}, },
			{  252, { 1220458, -28654,  247,   179, -10376,  298}, },
			{  324, { 1280953, -30204,  247,   119,  -9766,  304}, },
			{  396, { 1344547, -31777,  247,   119,  -8545,  292}, },
			{  468, { 1420168, -34227,  269,    60,  -7172,  256}, },
			{  540, { 1490757, -35955,  274,    60,  -5188,  197}, },
			{  612, { 1599112, -42583,  398,     0,  -1831,  119}, },
			{  648, { 1366986, -16459, -274,     0,  -3204,   72}, },
			{    0, { }, },
		},
		.vmin_trips_table = { 20, },
		.therm_floors_table = { 900, },
		.vts_trips_table = { -10, 10, 30, 50, 70, },
	},
	{
		.speedo_id =  1,
		.process_id = -1,
		.max_mv = 1200,
		.min_mv = 800,
		.speedo_scale = 100,
		.thermal_scale = 10,
		.voltage_scale = 1000,
		.cvb_table = {
			{   72, { 1209886, -36468,  515,   417, -13123,  203}, },
			{  108, { 1130804, -27659,  296,   298, -10834,  221}, },
			{  180, { 1162871, -27110,  247,   238, -10681,  268}, },
			{  252, { 1220458, -28654,  247,   179, -10376,  298}, },
			{  324, { 1280953, -30204,  247,   119,  -9766,  304}, },
			{  396, { 1344547, -31777,  247,   119,  -8545,  292}, },
			{  468, { 1420168, -34227,  269,    60,  -7172,  256}, },
			{  540, { 1490757, -35955,  274,    60,  -5188,  197}, },
			{  612, { 1599112, -42583,  398,     0,  -1831,  119}, },
			{  648, { 1366986, -16459, -274,     0,  -3204,   72}, },
			{  684, { 1391884, -17078, -274,   -60,  -1526,   30}, },
			{  708, { 1415522, -17497, -274,   -60,   -458,    0}, },
			{  756, { 1464061, -18331, -274,  -119,   1831,  -72}, },
			{  804, { 1524225, -20064, -254,  -119,   4272, -155}, },
			{  852, { 1608418, -21643, -269,     0,    763,  -48}, },
			{    0, { }, },
		},
		.vmin_trips_table = { 20, },
		.therm_floors_table = { 900, },
		.vts_trips_table = { -10, 10, 30, 50, 70, },
	}
};

static int gpu_millivolts[MAX_THERMAL_RANGES][MAX_DVFS_FREQS];

static struct tegra_cooling_device gpu_vmin_cdev = {
	.cdev_type = "gpu_cold",
};

static struct tegra_cooling_device gpu_vts_cdev = {
	.cdev_type = "gpu_scaling",
};

static struct gk20a_dvfs_rail gpu_rail = {
	.supply_name = GPU_SUPPLY_NAME,
	.max_uv = 1350000,
	.min_uv = 700000,
	.vts_cdev = &gpu_vts_cdev,
	.vmin_cdev = &gpu_vmin_cdev,
};

static DEFINE_MUTEX(gk20a_dvfs_lock);


static int gk20a_dvfs_init_pmic(struct platform_device *pdev, struct gk20a *g);

/**
 * Validate thermal dvfs settings:
 * - trip-points are montonically increasing
 * - voltages in any temperature range are montonically increasing with
 *   frequency (can go up/down across ranges at iso frequency)
 * - voltage for any frequency/thermal range combination must be within
 *   rail minimum/maximum limits
 */
static int init_thermal_dvfs_trips(struct platform_device *pdev,
				   int *therm_trips_table,
				   struct gk20a_dvfs_rail *rail)
{
	int i;

	if (!rail->vts_cdev) {
		WARN(1, "%s: missing thermal dvfs cooling device\n",
		     rail->supply_name);
		return -ENOENT;
	}

	if (!rail->vts_cdev->trip_temperatures) {
		int *trips_t;

		trips_t = devm_kzalloc(&pdev->dev,
				       sizeof(int) * MAX_THERMAL_LIMITS,
				       GFP_KERNEL);
		if (!trips_t) {
			dev_err(&pdev->dev, "Failed to kzalloc trips table\n");
			return -ENOMEM;
		}
		rail->vts_cdev->trip_temperatures = trips_t;
	}

	for (i = 0; i < MAX_THERMAL_LIMITS - 1; i++) {
		if (therm_trips_table[i] >= therm_trips_table[i+1])
			break;
	}

	rail->vts_cdev->trip_temperatures_num = i + 1;
	memcpy(rail->vts_cdev->trip_temperatures, therm_trips_table,
	       sizeof(int) * MAX_THERMAL_LIMITS);

	return 0;
}

/**
 * cvb_mv = ((c2 * speedo / s_scale + c1) * speedo / s_scale + c0) / v_scale
 */
static inline int get_cvb_voltage(int speedo, int s_scale,
	struct gk20a_cvb_parameters *cvb)
{
	/* apply only speedo scale: output mv = cvb_mv * v_scale */
	int mv;
	mv = DIV_ROUND_CLOSEST(cvb->c2 * speedo, s_scale);
	mv = DIV_ROUND_CLOSEST((mv + cvb->c1) * speedo, s_scale) + cvb->c0;
	return mv;
}

/**
 * cvb_t_mv =
 * ((c3 * speedo / s_scale + c4 + c5 * T / t_scale) * T / t_scale) / v_scale
 */
static inline int get_cvb_t_voltage(int speedo, int s_scale,
	int t, int t_scale, struct gk20a_cvb_parameters *cvb)
{
	/* apply speedo & temperature scales: output mv = cvb_t_mv * v_scale */
	int mv;
	mv = DIV_ROUND_CLOSEST(cvb->c3 * speedo, s_scale) + cvb->c4 +
		DIV_ROUND_CLOSEST(cvb->c5 * t, t_scale);
	mv = DIV_ROUND_CLOSEST(mv * t, t_scale);
	return mv;
}

static int round_cvb_voltage(int mv, int v_scale, int step_uv)
{
	/* combined: apply voltage scale and round to cvb alignment step */
	int uv = mv * 1000;
	int step = (step_uv ? : 1000) * v_scale;

	uv = DIV_ROUND_UP(uv, step) * step_uv;
	return uv / 1000;
}

/**
 * round_voltage - round the max and min voltage by rail step uv
 *
 * @uv: microvolts to round
 * @step_uv: step uv for the rail
 */
static int round_voltage(int mv, int step_uv, bool up)
{
	int uv = mv * 1000;

	BUG_ON(!step_uv);
	uv = (uv + (up ? step_uv - 1 : 0)) / step_uv;
	return uv * step_uv / 1000;
}

static int validate_thermal_dvfs_voltages(
	int *therm_voltages, int freqs_num, int ranges_num,
	struct gk20a_dvfs *dvfs, struct gk20a_dvfs_rail *rail)
{
	int *millivolts;
	int freq_idx, therm_idx;

	for (therm_idx = 0; therm_idx < ranges_num; therm_idx++) {
		millivolts = therm_voltages + therm_idx * MAX_DVFS_FREQS;

		for (freq_idx = 0; freq_idx < freqs_num; freq_idx++) {
			int mv = millivolts[freq_idx];

			if ((mv > rail->max_uv / 1000) ||
			    (mv < rail->min_uv / 1000) ||
			    (freq_idx && (mv < millivolts[freq_idx - 1]))) {
				WARN(1, "gk20a_dvfs: invalid thermal dvfs entry %d(%d, %d)\n",
					mv, freq_idx, therm_idx);
				return -EINVAL;
			}
		}
	}

	dvfs->therm_dvfs = true;
	return 0;
}

static bool match_gpu_cvb_one(struct gk20a_cvb_info *info,
	int speedo_id, int process_id)
{
	if ((info->process_id != -1 && info->process_id != process_id) ||
		(info->speedo_id != -1 && info->speedo_id != speedo_id)) {
		pr_info("tegra124_dvfs: rejected gpu cvb speedo %d, process %d",
			info->speedo_id, info->process_id);
		return false;
	}
	return true;
}

static int gk20a_dvfs_rail_apply_limits(struct gk20a_dvfs_rail *rail,
					unsigned long rate,
					int millivolts)
{
	int target_mv, min_mv, max_mv;
	int idx;
	int *dvfs_mv;
	unsigned long target_rate;

	min_mv = rail->min_uv / 1000;
	max_mv = rail->max_uv / 1000;
	target_mv = millivolts;
	if (rate)
		target_rate = rate;
	else
		target_rate = gpu_dvfs.cur_rate;

	if (rail->therm_mv_floors) {
		idx = rail->therm_floor_idx;
		if (idx < rail->therm_mv_floors_num)
			min_mv = rail->therm_mv_floors[idx];
	}

	if (gpu_dvfs.therm_dvfs) {
		int i;

		idx = rail->therm_scale_idx;
		dvfs_mv = gpu_dvfs.millivolts + idx * MAX_DVFS_FREQS;

		for (i = 0; i < gpu_dvfs.num_freqs; i++) {
			if (!target_rate)
				goto no_rate;

			if (target_rate <= gpu_dvfs.freqs[i])
				break;
	       }

		if (i == gpu_dvfs.num_freqs)
			i--;

		max_mv = dvfs_mv[i];
	}

no_rate:

	if (target_mv < min_mv)
		target_mv = min_mv;
	else if (target_mv > max_mv)
		target_mv = max_mv;

	return target_mv;
}

static void gk20a_dvfs_rail_update_voltage(struct gk20a_dvfs_rail *rail)
{
	int mv;

	mv = gk20a_dvfs_rail_apply_limits(gpu_dvfs.rail, 0,
					  gpu_dvfs.cur_millivolts);

	if (mv != gpu_dvfs.cur_millivolts) {
		int max_mv, ret;

		max_mv = gpu_dvfs.max_millivolts;

		ret = regulator_set_voltage(gpu_dvfs.rail->reg,
					    mv * 1000,
					    max_mv * 1000);
		if (ret)
			pr_warn("Failed to set voltage to %duV\n",
				mv * 1000);
		else
			gpu_dvfs.cur_millivolts = mv;
	}
}

#ifdef CONFIG_THERMAL
static int gk20a_dvfs_rail_get_vmin_cdev_max_state(
	struct thermal_cooling_device *cdev, unsigned long *max_state)
{
	struct gk20a_dvfs_rail *rail = (struct gk20a_dvfs_rail *)cdev->devdata;
	*max_state = rail->vmin_cdev->trip_temperatures_num;

	return 0;
}

static int gk20a_dvfs_rail_get_vmin_cdev_cur_state(
	struct thermal_cooling_device *cdev, unsigned long *cur_state)
{
	struct gk20a_dvfs_rail *rail = (struct gk20a_dvfs_rail *)cdev->devdata;
	*cur_state = rail->therm_floor_idx;

	return 0;
}

static int gk20a_dvfs_rail_set_vmin_cdev_state(
	struct thermal_cooling_device *cdev, unsigned long cur_state)
{
	struct gk20a_dvfs_rail *rail = (struct gk20a_dvfs_rail *)cdev->devdata;

	if (IS_ERR(gpu_dvfs.rail->reg))
		return -EINVAL;

	if (rail->therm_floor_idx == cur_state)
		return 0;

	rail->therm_floor_idx = cur_state;
	gk20a_dvfs_rail_update_voltage(rail);

	return 0;
}

static struct thermal_cooling_device_ops gk20a_dvfs_vmin_cooling_ops = {
	.get_max_state = gk20a_dvfs_rail_get_vmin_cdev_max_state,
	.get_cur_state = gk20a_dvfs_rail_get_vmin_cdev_cur_state,
	.set_cur_state = gk20a_dvfs_rail_set_vmin_cdev_state,
};

static void gk20a_dvfs_rail_register_vmin_cdev(struct gk20a_dvfs_rail *rail)
{
	if (!rail->vmin_cdev)
		return;

	rail->therm_floor_idx = 0;
	/* just report error - initialized for cold temperature, anyway */
	if (IS_ERR(thermal_cooling_device_register(
		rail->vmin_cdev->cdev_type, (void *)rail,
		&gk20a_dvfs_vmin_cooling_ops)))
		pr_err("gk20a cooling device %s failed to register\n",
		       rail->vmin_cdev->cdev_type);
}

static int gk20a_dvfs_rail_get_vts_cdev_max_state(
	struct thermal_cooling_device *cdev, unsigned long *max_state)
{
	struct gk20a_dvfs_rail *rail = (struct gk20a_dvfs_rail *)cdev->devdata;
	*max_state = rail->vts_cdev->trip_temperatures_num;

	return 0;
}

static int gk20a_dvfs_rail_get_vts_cdev_cur_state(
	struct thermal_cooling_device *cdev, unsigned long *cur_state)
{
	struct gk20a_dvfs_rail *rail = (struct gk20a_dvfs_rail *)cdev->devdata;
	*cur_state = rail->therm_scale_idx;

	return 0;
}

static int gk20a_dvfs_rail_set_vts_cdev_state(
	struct thermal_cooling_device *cdev, unsigned long cur_state)
{
	struct gk20a_dvfs_rail *rail = (struct gk20a_dvfs_rail *)cdev->devdata;

	if (IS_ERR(gpu_dvfs.rail->reg))
		return -EINVAL;

	if (rail->therm_scale_idx == cur_state)
		return 0;

	rail->therm_scale_idx = cur_state;
	gk20a_dvfs_rail_update_voltage(rail);

	return 0;
}

static struct thermal_cooling_device_ops gk20a_dvfs_vts_cooling_ops = {
	.get_max_state = gk20a_dvfs_rail_get_vts_cdev_max_state,
	.get_cur_state = gk20a_dvfs_rail_get_vts_cdev_cur_state,
	.set_cur_state = gk20a_dvfs_rail_set_vts_cdev_state,
};

static void gk20a_dvfs_rail_register_vts_cdev(struct gk20a_dvfs_rail *rail)
{
	if (!rail->vts_cdev)
		return;

	rail->therm_scale_idx = 0;
	/* just report error - initialized for cold temperature, anyway */
	if (IS_ERR(thermal_cooling_device_register(
		rail->vts_cdev->cdev_type, (void *)rail,
		&gk20a_dvfs_vts_cooling_ops)))
		pr_err("gk20a cooling device %s failed to register\n",
		       rail->vts_cdev->cdev_type);
}
#else
static inline void gk20a_dvfs_rail_register_vmin_cdev(struct dvfs_rail *rail)
{ return; }
static inline void gk20a_dvfs_rail_register_vts_cdev(struct dvfs_rail *rail)
{ return; }
#endif

struct tegra_cooling_device *tegra_get_gpu_vmin_cdev(void)
{
	if (!gpu_rail.vmin_cdev)
		return NULL;
	if ((!gpu_rail.vmin_cdev->trip_temperatures) ||
	    (!gpu_rail.reg))
		return ERR_PTR(-EPROBE_DEFER);

	return gpu_rail.vmin_cdev;
}

struct tegra_cooling_device *tegra_get_gpu_vts_cdev(void)
{
	if (!gpu_rail.vts_cdev)
		return NULL;
	if ((!gpu_rail.vts_cdev->trip_temperatures) ||
	    (!gpu_rail.reg))
		return ERR_PTR(-EPROBE_DEFER);

	return gpu_rail.vts_cdev;
}

static int get_gk20a_thermal_profile_size(int *trips_table,
					  int *limits_table,
					  struct gk20a_dvfs_rail *rail)
{
	int i;

	for (i = 0; i < MAX_THERMAL_LIMITS - 1; i++) {
		if (!limits_table[i + 1])
			break;

		if ((trips_table[i] >= trips_table[i + 1]) ||
		    (limits_table[i] < limits_table[i + 1])) {
			pr_warn("%s: not ordered profile\n", rail->supply_name);
			return -EINVAL;
		}
	}

	if ((limits_table[i] * 1000) < rail->min_uv) {
		pr_warn("%s: thermal profile below Vmin\n", rail->supply_name);
		return -EINVAL;
	}

	if ((limits_table[0] * 100) > rail->max_uv) {
		pr_warn("%s: thermal profile above Vmax\n", rail->supply_name);
		return -EINVAL;
	}

	return i + 1;
}

static void gk20a_dvfs_rail_init_vmin_thermal_profile(
		int *therm_trips_table, int *therm_floors_table,
		struct gk20a_dvfs_rail *rail)
{
	int i = get_gk20a_thermal_profile_size(therm_trips_table,
					 therm_floors_table, rail);
	if (i <= 0) {
		rail->vmin_cdev = NULL;
		pr_warn("%s: invalid Vmin thermal profile\n",
			rail->supply_name);
		return;
	}

	/* Install validated thermal floors */
	rail->therm_mv_floors = therm_floors_table;
	rail->therm_mv_floors_num = i;

	/* Setup trip-points if applicable */
	if (rail->vmin_cdev) {
		rail->vmin_cdev->trip_temperatures_num = i;
		rail->vmin_cdev->trip_temperatures = therm_trips_table;
	}
}

/**
 * set_gpu_dvfs_data - setup the global dvfs instance by cvb and rail info
 *
 * @pdev: the gk20a platform device instance
 * @info: cvb information with the dedicated speedo/process id
 * @dvfs: the global dvfs instance
 *
 * The caller must hold @gk20a_dvfs_lock.
 */
static int set_gpu_dvfs_data(struct platform_device *pdev,
			     struct gk20a_cvb_info *info,
			     struct gk20a_dvfs *dvfs)
{
	int i, j, thermal_ranges, mv;
	struct gk20a_cvb_table *table = NULL;
	int speedo = tegra_get_gpu_speedo_value();
	struct gk20a_dvfs_rail *rail = dvfs->rail;

	info->max_mv = round_voltage(info->max_mv, rail->step_uv, false);
	info->min_mv = round_voltage(info->min_mv, rail->step_uv, true);
	if (info->min_mv < rail->min_uv / 1000) {
		WARN(1, "the rounded voltage is below the minimum\n");
		return -ENOENT;
	}

	/*
	 * Init thermal trips, find number of thermal ranges; note that the
	 * first trip-point is used for voltage calculations within the lowest
	 * range, but should not be actually set. Hence, at least 2 trip-points
	 * must be specified.
	 */
	if (init_thermal_dvfs_trips(pdev, info->vts_trips_table, rail))
		return -ENOENT;
	thermal_ranges = rail->vts_cdev->trip_temperatures_num;
	rail->vts_cdev->trip_temperatures_num--;

	if (thermal_ranges < 2)
		WARN(1, "gk20a_dvfs: %d gpu trip: thermal dvfs is broken\n",
		     thermal_ranges);

	/*
	 * Use CVB table to fill in gpu dvfs frequencies and voltages. Each
	 * CVB entry specifies gpu frequency and CVB coefficients to calculate
	 * the respective voltage.
	 */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		table = &info->cvb_table[i];
		if (!table->freq)
			break;

		mv = get_cvb_voltage(speedo, info->speedo_scale,
				     &table->cvb_param);

		for (j = 0; j < thermal_ranges; j++) {
			int mvj = mv;
			int t = rail->vts_cdev->trip_temperatures[j];

			/* get thermal offset for this trip-point */
			mvj += get_cvb_t_voltage(speedo, info->speedo_scale,
				t, info->thermal_scale, &table->cvb_param);
			mvj = round_cvb_voltage(mvj, info->voltage_scale,
						rail->step_uv);

			/* clip to minimum, abort if above maximum */
			mvj = max(mvj, info->min_mv);
			if (mvj > info->max_mv)
				break;

			/* update voltage for adjacent ranges bounded by this
			   trip-point (cvb & dvfs are transpose matrices) */
			gpu_millivolts[j][i] = mvj;
			if (j && (gpu_millivolts[j-1][i] < mvj))
				gpu_millivolts[j-1][i] = mvj;
		}
		/* Make sure all voltages for this frequency are below max */
		if (j < thermal_ranges)
			break;

		/* fill in gpu dvfs tables */
		dvfs->freqs[i] = table->freq;
	}

	/*
	 * Table must not be empty, must have at least one entry in range, and
	 * must specify monotonically increasing voltage on frequency dependency
	 * in each temperature range.
	 */
	if (!i || validate_thermal_dvfs_voltages(&gpu_millivolts[0][0],
					i, thermal_ranges, dvfs, rail)) {
		WARN(1, "gk20a_dvfs: invalid gpu dvfs table\n");
		return -ENOENT;
	}

	dvfs->millivolts = &gpu_millivolts[0][0];

	/* Shift out the 1st trip-point */
	for (j = 1; j < thermal_ranges; j++)
		rail->vts_cdev->trip_temperatures[j - 1] =
		rail->vts_cdev->trip_temperatures[j];

	/* dvfs tables are successfully populated - fill in the gpu dvfs */
	dvfs->num_freqs = i;
	dvfs->speedo_id = info->speedo_id;
	dvfs->process_id = info->process_id;
	dvfs->freqs_mult = KHZ;
	dvfs->rail->nominal_uv = info->max_mv;
	dvfs->max_millivolts = info->max_mv;

	gk20a_dvfs_rail_init_vmin_thermal_profile(info->vmin_trips_table,
						  info->therm_floors_table,
						  &gpu_rail);

	return 0;
}

/*
 * Public interfaces
 */

/**
 * gk20a_dvfs_adjust_voltage - adjust voltage according to the dvfs table
 *
 * @g: the gk20a instance
 * @rate: the target rate in MHz
 */
int gk20a_dvfs_adjust_voltage(struct gk20a *g, unsigned long rate)
{
	int index;
	int uv, mv;
	int ret;

	if (!g || !gpu_dvfs.rail->reg)
		gk20a_dvfs_init_pmic(g->dev, g);

	for (index = 0; index < gpu_dvfs.num_freqs; index++)
		if (rate <= gpu_dvfs.freqs[index])
			break;

	if (index == gpu_dvfs.num_freqs)
		uv = gpu_dvfs.max_millivolts * 1000;
	else
		uv = gpu_dvfs.millivolts[index] * 1000;

	mv  = gk20a_dvfs_rail_apply_limits(gpu_dvfs.rail, rate, (uv / 1000));
	uv = mv * 1000;

	dev_dbg(dev_from_gk20a(g),
		"%s - target rate=%luMHz, volt=%duv\n", __func__, rate, uv);
	ret = regulator_set_voltage(gpu_dvfs.rail->reg, uv,
			gpu_dvfs.max_millivolts * 1000);

	if (!ret) {
		gpu_dvfs.cur_millivolts = uv / 1000;
		gpu_dvfs.cur_rate = rate;
	}

	WARN(ret, "failed to set voltage to %duV\n", uv);

	return ret;
}

int gk20a_dvfs_get_freqs(struct gk20a *g, unsigned long **freqs,
	int *num_freqs)
{
	if (!g || !freqs || !num_freqs)
		return -EINVAL;

	*freqs = gpu_dvfs.freqs;
	*num_freqs = gpu_dvfs.num_freqs;

	return 0;
}

/**
 * gk20a_dvfs_get_min_freq - get the minimal frequency in the dvfs table
 *
 * @g: the gk20a instance
 */
unsigned long gk20a_dvfs_get_min_freq(struct gk20a *g)
{
	if (!g) {
		WARN_ON(1);
		return 0;
	}

	return gpu_dvfs.freqs[0];
}

/**
 * gk20a_dvfs_get_max_freq - get the maximal frequency in the dvfs table
 *
 * @g: the gk20a instance
 */
unsigned long gk20a_dvfs_get_max_freq(struct gk20a *g)
{
	if (!g || !gpu_dvfs.num_freqs) {
		WARN_ON(1);
		return 0;
	}

	return gpu_dvfs.freqs[gpu_dvfs.num_freqs - 1];
}

/**
 * gk20a_dvfs_init_pmic - connect to regulator and setup the global @gpu_rail
 *
 * @pdev: the gk20a platform device instance
 * @g: the gk20a instance
 *
 * When this function gets called, it can't be guaranteed that the regulator
 * has finished its initialization. The devm_regulator_get() may not return
 * failure becasue we have dummy regulator support. So in order to not break
 * the pmic initialization here, we have to give a default step microvolt.
 *
 * This function should also function even if the gk20a dvfs is disabled.
 */
static int gk20a_dvfs_init_pmic(struct platform_device *pdev, struct gk20a *g)
{
	struct regulator *reg;
	struct device *dev;
	unsigned int step;
	struct gk20a_dvfs_rail *rail;

	dev = &pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	rail = gpu_dvfs.rail;
	if (!rail || !rail->reg) {
		reg = devm_regulator_get_optional(dev, gpu_rail.supply_name);
		if (IS_ERR(reg))
			dev_warn(dev, "failed to get supply %s\n",
				 gpu_rail.supply_name);
	}

	if (!IS_ERR(reg)) {
		step = regulator_get_linear_step(reg);
	} else {
		dev_warn(dev,
			 "step uv is not supported, assign a default %dmV\n",
			 DEFAULT_STEP_UV / 1000);
		step = DEFAULT_STEP_UV;

		/* Keep reg as NULL and may initialize it again later */
		reg = NULL;
	}

	mutex_lock(&gk20a_dvfs_lock);
	gpu_rail.dev = dev;
	gpu_rail.reg = reg;
	gpu_rail.step_uv = step;
	gpu_dvfs.rail = &gpu_rail;
	mutex_unlock(&gk20a_dvfs_lock);

	gk20a_dvfs_init_dvfs_table(pdev, g);

	return 0;
}

/**
 * gk20a_dvfs_enable_rail - power on the rail of gk20a
 *
 * @pdev: the platform device instance of gk20a
 *
 * This function should also function even if the gk20a dvfs is disabled.
 */
int gk20a_dvfs_enable_rail(struct platform_device *pdev, struct gk20a *g)
{
	int ret = 0;
	struct gk20a_dvfs_rail *rail = gpu_dvfs.rail;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!rail || !rail->reg) {
		ret = gk20a_dvfs_init_pmic(pdev, g);
		if (ret) {
			dev_warn(&pdev->dev,
					"failed to initialize pmic when enabling gpu rail\n");
			return ret;
		}

		rail = gpu_dvfs.rail;
	}

	if (!regulator_is_enabled(rail->reg))
		return regulator_enable(rail->reg);

	return 0;
}

/**
 * gk20a_dvfs_disable_rail - power off the rail of gk20a
 *
 * @pdev: the platform device instance of gk20a
 *
 * This function should also function even if the gk20a dvfs is disabled.
 */
int gk20a_dvfs_disable_rail(struct platform_device *pdev, struct gk20a *g)
{
	int ret = 0;
	struct gk20a_dvfs_rail *rail = gpu_dvfs.rail;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!rail || !rail->reg) {
		ret = gk20a_dvfs_init_pmic(pdev, g);
		if (ret) {
			dev_warn(&pdev->dev,
					"failed to initialize pmic when disabling gpu rail\n");
			return ret;
		}

		rail = gpu_dvfs.rail;
	}

	if (regulator_is_enabled(rail->reg))
		return regulator_disable(rail->reg);

	return 0;
}

/**
 * gk20a_dvfs_init_dvfs_table - initialize dvfs table
 *
 * @pdev: the gk20a platform device instance
 * @g: the gk20a instance
 */
int gk20a_dvfs_init_dvfs_table(struct platform_device *pdev, struct gk20a *g)
{
	int gpu_speedo_id, gpu_process_id;
	int i, ret;

	gpu_speedo_id = tegra_get_gpu_speedo_id();
	gpu_process_id = tegra_get_gpu_process_id();

	for (ret = 0, i = 0; i < ARRAY_SIZE(gpu_cvb_info_table); i++) {
		struct gk20a_cvb_info *info = &gpu_cvb_info_table[i];
		if (match_gpu_cvb_one(info, gpu_speedo_id, gpu_process_id)) {
			mutex_lock(&gk20a_dvfs_lock);
			ret = set_gpu_dvfs_data(pdev, info, &gpu_dvfs);
			mutex_unlock(&gk20a_dvfs_lock);
			break;
		}
	}

	if (i == ARRAY_SIZE(gpu_cvb_info_table) || ret)
		return -ENODEV;

	return 0;
}

/**
 * gk20a_dvfs_init - connect to the regulator and initialize the dvfs table
 *
 * @pdev: platform device to the gk20a
 * @g: the gk20a instance
 */
int gk20a_dvfs_init(struct platform_device *pdev, struct gk20a *g)
{
	struct device *dev;
	int ret;

	if (!g)
		return -EINVAL;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	dev = &pdev->dev;

	mutex_lock(&gk20a_dvfs_lock);
	gpu_dvfs.g = g;
	mutex_unlock(&gk20a_dvfs_lock);

	ret = gk20a_dvfs_init_pmic(pdev, g);
	if (ret) {
		dev_warn(&pdev->dev, "failed to initialize pmic\n");
		return ret;
	}

	ret = gk20a_dvfs_init_dvfs_table(pdev, g);
	if (ret) {
		dev_warn(&pdev->dev, "failed to initialize dvfs table\n");
		return ret;
	}

	gk20a_dvfs_rail_register_vmin_cdev(&gpu_rail);
	gk20a_dvfs_rail_register_vts_cdev(&gpu_rail);

	return 0;
}
