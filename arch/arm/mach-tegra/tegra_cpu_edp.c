/*
 * Copyright (C) 2011-2013, NVIDIA CORPORATION. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/suspend.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-dvfs.h>
#include <linux/platform_data/tegra_edp.h>

#include "tegra_edp_private.h"

#define KHZ	1000
#define FREQ_STEP_KHZ		12750
#define OVERRIDE_DEFAULT	6000

/*
 * "Safe entry" to be used when no match for speedo_id /
 * regulator_cur is found; must be the last one
 */
static struct tegra_edp_limits edp_default_limits[] = {
	{85, {1000000, 1000000, 1000000, 1000000} },
};

/* Constants for EDP calculations */
static const int temperatures[] = { /* degree celcius (C) */
	23, 40, 50, 60, 70, 74, 78, 82, 86, 90, 94, 98, 102,
};

static DEFINE_MUTEX(tegra_cpu_lock);
static struct tegra_edp cpu_edp;

static inline s64 edp_pow(s64 val, int pwr)
{
	s64 retval = 1;

	while (pwr) {
		if (pwr & 1)
			retval *= val;
		pwr >>= 1;
		if (pwr)
			val *= val;
	}

	return retval;
}

static int cal_leakage_ma(const struct tegra_edp_cpu_leakage_params *params,
			  int iddq_ma, unsigned int voltage_mv, int temp_c)
{
	int i, j, k;
	s64 leakage_ma, leakage_calc_step;

	/* Calculate leakage current */
	leakage_ma = 0;
	for (i = 0; i <= 3; i++) {
		for (j = 0; j <= 3; j++) {
			for (k = 0; k <= 3; k++) {
				unsigned int scaled;
				int ijk;

				ijk = params->leakage_consts_ijk[i][j][k];
				leakage_calc_step = ijk * edp_pow(iddq_ma, i);
				/* Convert (mA)^i to (A)^i */
				leakage_calc_step = div64_s64(leakage_calc_step,
							      edp_pow(1000, i));
				leakage_calc_step *= edp_pow(voltage_mv, j);
				/* Convert (mV)^j to (V)^j */
				leakage_calc_step = div64_s64(leakage_calc_step,
							      edp_pow(1000, j));
				leakage_calc_step *= edp_pow(temp_c, k);
				/* Convert (C)^k to (scaled_C)^k */
				scaled = params->temp_scaled;
				leakage_calc_step = div64_s64(leakage_calc_step,
							edp_pow(scaled, k));
				/* leakage_consts_ijk was scaled */
				leakage_calc_step = div64_s64(leakage_calc_step,
							params->ijk_scaled);
				leakage_ma += leakage_calc_step;
			}
		}
	}

	return leakage_ma;
}

/*
 * Find the maximum frequency that results in dynamic and leakage current that
 * is less than the regulator current limit.
 * temp_c - valid or -EINVAL
 * power_mw - valid or -1 (infinite) or -EINVAL
 */
static int edp_calculate_maxf(struct device *dev, int temp_idx, int power_mw,
			      int iddq_ma, int n_cores_idx,
			      struct tegra_edp_freq_vol_table *freq_voltage_lut,
			      unsigned int freq_voltage_lut_size)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	const struct tegra_edp_cpu_leakage_params *params;
	int temp_c = edp->temps[temp_idx];
	unsigned int voltage_mv, freq_khz;
	unsigned int cur_effective;
	int f;
	s64 leakage_ma, dyn_ma;
	s64 leakage_mw, dyn_mw;

	params = &edp->params[edp->cpu_speedo_idx];
	cur_effective = edp->regulator_cur - edp->edp_reg_override_ma;

	for (f = freq_voltage_lut_size - 1; f >= 0; f--) {
		freq_khz = freq_voltage_lut[f].freq / KHZ;
		voltage_mv = freq_voltage_lut[f].voltage_mv;

		/* Constrain Volt-Temp */
		if (params->volt_temp_cap.temperature &&
		    temp_c > params->volt_temp_cap.temperature &&
		    params->volt_temp_cap.voltage_limit_mv &&
		    voltage_mv > params->volt_temp_cap.voltage_limit_mv)
			continue;

		/* Calculate leakage current */
		leakage_ma = cal_leakage_ma(params,
					    iddq_ma, voltage_mv, temp_c);

		/* if specified, set floor for leakage current */
		if (params->leakage_min && leakage_ma <= params->leakage_min)
			leakage_ma = params->leakage_min;

		/* leakage cannot be negative => leakage model has error */
		if (leakage_ma <= 0) {
			dev_err(dev,
				"VDD_CPU EDP failed: IDDQ too high (%d mA)\n",
				iddq_ma);
			return -EINVAL;
		}

		leakage_ma *= params->leakage_consts_n[n_cores_idx];

		/* leakage_const_n was scaled */
		leakage_ma = div64_s64(leakage_ma, params->consts_scaled);

		/* Calculate dynamic current */
		dyn_ma = voltage_mv * freq_khz / 1000;
		/* Convert mV to V */
		dyn_ma = div64_s64(dyn_ma, 1000);
		dyn_ma *= params->dyn_consts_n[n_cores_idx];
		/* dyn_const_n was scaled */
		dyn_ma = div64_s64(dyn_ma, params->dyn_scaled);

		if (power_mw != -1) {
			leakage_mw = leakage_ma * voltage_mv;
			dyn_mw = dyn_ma * voltage_mv;
			if (div64_s64(leakage_mw + dyn_mw, 1000) <= power_mw)
				return freq_khz;
		} else if ((leakage_ma + dyn_ma) <= cur_effective) {
			return freq_khz;
		}
	}

	return -EINVAL;
}

static int edp_relate_freq_voltage(struct device *dev, struct clk *clk_cpu_g,
				   unsigned int cpu_speedo_idx,
				   struct tegra_edp_freq_vol_table *lut,
				   unsigned int lut_size)
{
	unsigned int i, freq;
	int voltage_mv;

	for (i = 0, freq = 0; i < lut_size; i++, freq += FREQ_STEP_KHZ) {
		/* Predict voltages */
		voltage_mv = tegra_dvfs_predict_millivolts(clk_cpu_g,
							   freq * KHZ);
		if (voltage_mv < 0) {
			dev_err(dev,
				"Couldn't predict voltage: freq %u; err %d\n",
				freq, voltage_mv);
			return -EINVAL;
		}

		/* Cache frequency / voltage / voltage constant relationship */
		lut[i].freq = freq * KHZ;
		lut[i].voltage_mv = voltage_mv;
	}

	return 0;
}

static int edp_find_speedo_idx(struct device *dev, int cpu_speedo_id)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < edp->params_size; i++)
		if (cpu_speedo_id == edp->params[i].cpu_speedo_id)
			return i;

	dev_err(dev, "Couldn't find cpu speedo id %d in freq/voltage LUT\n",
		cpu_speedo_id);

	return -EINVAL;
}

static int tegra_cpu_edp_cal_limits(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	unsigned int temp_idx, n_cores_idx;
	unsigned int cpu_g_minf, cpu_g_maxf;
	unsigned int cpu_speedo_idx;
	unsigned int cap, limit;
	struct tegra_edp_limits *edp_calculated_limits;
	const struct tegra_edp_cpu_leakage_params *params;
	int cpu_speedo_id, iddq_ma;
	size_t size;
	int ret;
	struct clk *clk_cpu_g = clk_get_sys(NULL, "cclk_g");
	struct tegra_edp_freq_vol_table *freq_voltage_lut;
	unsigned int freq_voltage_lut_size;

	/* Determine all inputs to EDP formula */
	cpu_speedo_id = tegra_get_cpu_speedo_id();
	iddq_ma = tegra_get_cpu_iddq_value();
	cpu_speedo_idx = edp_find_speedo_idx(dev, cpu_speedo_id);
	if (cpu_speedo_idx < 0)
		return -EINVAL;

	edp->cpu_speedo_idx = cpu_speedo_idx;
	params = &edp->params[cpu_speedo_idx];

	cpu_g_minf = 0;
	cpu_g_maxf = edp->max_cpu_freq;
	freq_voltage_lut_size = (cpu_g_maxf - cpu_g_minf) / FREQ_STEP_KHZ + 1;
	size = sizeof(struct tegra_edp_freq_vol_table) * freq_voltage_lut_size;
	freq_voltage_lut = kzalloc(size, GFP_KERNEL);
	if (!freq_voltage_lut) {
		dev_err(dev, "Failed to alloc mem for freq/voltage LUT\n");
		return -ENOMEM;
	}

	ret = edp_relate_freq_voltage(dev, clk_cpu_g, cpu_speedo_idx,
				      freq_voltage_lut,
				      freq_voltage_lut_size);
	if (ret)
		goto err_free_lut;

	size = sizeof(struct tegra_edp_limits) * edp->temps_size;
	edp_calculated_limits = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!edp_calculated_limits) {
		dev_err(dev, "Failed to alloc mem for edp calculatd limits\n");
		ret = -ENOMEM;
		goto err_free_lut;
	}

	/* Calculate EDP table */
	for (n_cores_idx = 0;
	     n_cores_idx < num_possible_cpus(); n_cores_idx++) {
		for (temp_idx = 0;
		     temp_idx < edp->temps_size; temp_idx++) {
			edp_calculated_limits[temp_idx].temperature =
						edp->temps[temp_idx];
			limit = edp_calculate_maxf(dev,
						   temp_idx,
						   -1,
						   iddq_ma,
						   n_cores_idx,
						   freq_voltage_lut,
						   freq_voltage_lut_size);
			if (limit == -EINVAL) {
				ret = -EINVAL;
				goto err_free_limits;
			}
			/* apply safety cap if it is specified */
			if (n_cores_idx < 4) {
				cap = params->safety_cap[n_cores_idx];
				if (cap && cap < limit)
					limit = cap;
			}
			edp_calculated_limits[temp_idx].
				freq_limits[n_cores_idx] = limit;
		}
	}

	/*
	 * If this is an EDP table update, need to overwrite old table.
	 * The old table's address must remain valid.
	 */
	if (edp->edp_limits != edp->def_edp_limits) {
		memcpy(edp->edp_limits, edp_calculated_limits, size);
		devm_kfree(dev, edp_calculated_limits);
	} else {
		edp->edp_limits = edp_calculated_limits;
		edp->edp_limits_size = edp->temps_size;
	}

	kfree(freq_voltage_lut);

	return 0;

err_free_limits:
	devm_kfree(dev, edp_calculated_limits);
err_free_lut:
	kfree(freq_voltage_lut);
	return ret;
}

int tegra_cpu_edp_init_trips(struct platform_device *pdev,
			     struct thermal_trip_info *trips,
			     int *num_trips, char *cdev_type,
			     int margin)
{
	struct device *dev;
	struct tegra_edp *cpu_edp;
	struct thermal_trip_info *trip_state;
	const struct tegra_edp_limits *cpu_edp_limits;
	int i, cpu_edp_limits_size;

	if (!pdev)
		return -EINVAL;

	dev = &pdev->dev;
	cpu_edp = dev_get_drvdata(dev);

	if (!cpu_edp || !cpu_edp->edp_init_done)
		return -EPROBE_DEFER;

	if (!trips || !num_trips)
		return -EINVAL;

	/* edp capping */
	cpu_edp_limits = cpu_edp->edp_limits;
	cpu_edp_limits_size = cpu_edp->edp_limits_size;

	for (i = 0; i < cpu_edp_limits_size-1; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = cdev_type;
		trip_state->trip_temp =
			(cpu_edp_limits[i].temperature * 1000) - margin;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = i + 1;
		trip_state->lower = trip_state->upper;
		trip_state->hysteresis = 1000;

		(*num_trips)++;

		if (*num_trips >= THERMAL_MAX_TRIPS)
			BUG();
	}

	return 0;
}
EXPORT_SYMBOL(tegra_cpu_edp_init_trips);

void tegra_cpu_edp_recal_limits(void)
{
	if (!cpu_edp.edp_limits)
		return;

	if (tegra_cpu_edp_cal_limits(&cpu_edp.pdev->dev) == 0)
		return;

	/* Revert to default EDP table on error */
	cpu_edp.edp_limits = cpu_edp.def_edp_limits;
	cpu_edp.edp_limits_size = cpu_edp.def_edp_limits_size;
}
EXPORT_SYMBOL(tegra_cpu_edp_recal_limits);

static unsigned int edp_predict_limit(struct tegra_edp *edp, unsigned int cpus)
{
	unsigned int limit = 0;

	BUG_ON(cpus == 0);
	if (edp->edp_limits) {
		int i = edp->edp_thermal_index;
		int size = edp->edp_limits_size;

		BUG_ON(i >= size);
		limit = edp->edp_limits[i].freq_limits[cpus - 1];
	}

	return limit;
}

/* Must be called while holding tegra_cpu_lock */
static void tegra_edp_update_limit(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	unsigned int cpus, limit;

	if (!edp->edp_limits)
		return;

	BUG_ON(!mutex_is_locked(&tegra_cpu_lock));

	cpus = cpumask_weight(&edp->edp_cpumask);
	limit = edp_predict_limit(edp, cpus);
	edp->edp_freq_limit = limit;
}

static unsigned int tegra_edp_governor_speed(struct device *dev,
				      unsigned int requested_speed)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);

	if ((!edp->edp_freq_limit) ||
	    (requested_speed <= edp->edp_freq_limit))
		return requested_speed;
	else
		return edp->edp_freq_limit;
}

static int tegra_cpu_edp_init_data(struct device *dev,
				   unsigned int regulator_ma)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);

	edp->edp_reg_override_ma = OVERRIDE_DEFAULT;
	edp->regulator_cur = regulator_ma + OVERRIDE_DEFAULT;
	edp->def_edp_limits = edp_default_limits;
	edp->def_edp_limits_size = ARRAY_SIZE(edp_default_limits);
	edp->temps = temperatures;
	edp->temps_size = ARRAY_SIZE(temperatures);
	edp->edp_limits = edp_default_limits;
	edp->edp_limits_size = ARRAY_SIZE(edp_default_limits);

	tegra_cpu_edp_cal_limits(dev);

	mutex_lock(&tegra_cpu_lock);
	edp->edp_cpumask = *cpu_online_mask;
	tegra_edp_update_limit(dev);
	mutex_unlock(&tegra_cpu_lock);

	dev_info(dev, "Init EDP limit: %u MHz\n", edp->edp_freq_limit/1000);

	return 0;
}

int tegra_cpu_edp_get_thermal_index(struct platform_device *pdev)
{
	struct tegra_edp *edp = dev_get_drvdata(&pdev->dev);
	return edp->edp_thermal_index;
}
EXPORT_SYMBOL(tegra_cpu_edp_get_thermal_index);

int tegra_cpu_edp_count_therm_floors(struct platform_device *pdev)
{
	struct tegra_edp *edp = dev_get_drvdata(&pdev->dev);
	return edp->edp_limits_size - 1;
}
EXPORT_SYMBOL(tegra_cpu_edp_count_therm_floors);

int tegra_cpu_edp_update_thermal_index(struct platform_device *pdev,
				       unsigned long new_idx)
{
	struct tegra_edp *edp = dev_get_drvdata(&pdev->dev);

	if (new_idx > (edp->edp_limits_size - 1))
		return -ERANGE;

	if (!cpufreq_get(0))
		return 0;

	mutex_lock(&tegra_cpu_lock);

	edp->edp_thermal_index = new_idx;
	edp->edp_cpumask = *cpu_online_mask;
	tegra_edp_update_limit(&pdev->dev);

	mutex_unlock(&tegra_cpu_lock);

	cpufreq_update_policy(0);

	return 0;
}
EXPORT_SYMBOL(tegra_cpu_edp_update_thermal_index);

bool tegra_cpu_edp_ready(void)
{
	return cpu_edp.edp_init_done;
}
EXPORT_SYMBOL(tegra_cpu_edp_ready);

#ifdef CONFIG_DEBUG_FS

static struct dentry *edp_debugfs_dir;

static int edp_limit_debugfs_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct tegra_edp *edp = dev_get_drvdata(dev);

	seq_printf(s, "%u\n", edp->edp_freq_limit);
	return 0;
}

static int edp_debugfs_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct tegra_edp *edp = dev_get_drvdata(dev);
	int i, th_idx;

	th_idx = edp->edp_thermal_index;
	seq_printf(s, "-- VDD_CPU %sEDP table (%umA = %umA - %umA) --\n",
		   edp->edp_limits == edp->def_edp_limits ? "**default** " : "",
		   edp->regulator_cur - edp->edp_reg_override_ma,
		   edp->regulator_cur, edp->edp_reg_override_ma);
	seq_printf(s, "%6s %10s %10s %10s %10s\n",
		   " Temp.", "1-core", "2-cores", "3-cores", "4-cores");
	for (i = 0; i < edp->edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   edp->edp_limits[i].temperature,
			   edp->edp_limits[i].freq_limits[0],
			   edp->edp_limits[i].freq_limits[1],
			   edp->edp_limits[i].freq_limits[2],
			   edp->edp_limits[i].freq_limits[3]);
	}

	return 0;
}

static int edp_reg_override_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct tegra_edp *edp = dev_get_drvdata(dev);
	seq_printf(s, "Limit override: %u mA. Effective limit: %u mA\n",
		   edp->edp_reg_override_ma,
		   edp->regulator_cur - edp->edp_reg_override_ma);
	return 0;
}

static int edp_reg_override_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct device *dev = file->f_path.dentry->d_inode->i_private;
	struct tegra_edp *edp = dev_get_drvdata(dev);
	char buf[32];
	unsigned long res;
	unsigned int edp_reg_override_ma_temp;
	unsigned int edp_reg_override_ma_prev = edp->edp_reg_override_ma;

	if (sizeof(buf) <= count)
		goto override_err;

	if (copy_from_user(buf, userbuf, count))
		goto override_err;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (!kstrtoul(buf, 10, &res))
		edp_reg_override_ma_temp = (unsigned int)res;
	else
		goto override_err;

	if (edp_reg_override_ma_temp >= edp->regulator_cur)
		goto override_err;

	if (edp->edp_reg_override_ma == edp_reg_override_ma_temp)
		return count;

	edp->edp_reg_override_ma = edp_reg_override_ma_temp;
	if (tegra_cpu_edp_cal_limits(dev)) {
		/* Revert to previous override value if new value fails */
		edp->edp_reg_override_ma = edp_reg_override_ma_prev;
		goto override_err;
	}

	if (cpufreq_update_policy(0)) {
		dev_err(dev,
			"FAILED: Set CPU freq cap with new VDD_CPU EDP table\n");
		goto override_out;
	}

	dev_info(dev,
		 "Reinitialized VDD_CPU EDP table with regulator current limit %u mA\n",
		 edp->regulator_cur - edp->edp_reg_override_ma);

	return count;

override_err:
	dev_err(dev,
		"FAILED: Reinitialize VDD_CPU EDP table with override \"%s\"",
		buf);
override_out:
	return -EINVAL;
}

static int edp_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_debugfs_show, inode->i_private);
}

static int edp_limit_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_limit_debugfs_show, inode->i_private);
}

static int edp_reg_override_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_reg_override_show, inode->i_private);
}

static const struct file_operations edp_debugfs_fops = {
	.open		= edp_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations edp_limit_debugfs_fops = {
	.open		= edp_limit_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations edp_reg_override_debugfs_fops = {
	.open		= edp_reg_override_open,
	.read		= seq_read,
	.write		= edp_reg_override_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tegra_cpu_edp_debugfs_init(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	struct dentry *d_edp;
	struct dentry *d_edp_limit;
	struct dentry *d_edp_reg_override;
	struct dentry *edp_dir;
	struct dentry *vdd_cpu_dir;

	if (!edp_debugfs_dir)
		edp_debugfs_dir =  debugfs_create_dir("edp", NULL);

	if (!edp_debugfs_dir)
		goto edp_dir_err;

	edp_dir = edp_debugfs_dir;

	vdd_cpu_dir = debugfs_create_dir("edp_vdd_cpu", edp_dir);

	if (!vdd_cpu_dir)
		goto vdd_cpu_dir_err;

	d_edp = debugfs_create_file("edp", S_IRUGO, vdd_cpu_dir, dev,
				&edp_debugfs_fops);

	if (!d_edp)
		goto edp_err;

	d_edp_limit = debugfs_create_file("edp_limit", S_IRUGO, vdd_cpu_dir,
				dev, &edp_limit_debugfs_fops);

	if (!d_edp_limit)
		goto edp_limit_err;

	d_edp_reg_override = debugfs_create_file("edp_reg_override",
				S_IRUGO | S_IWUSR, vdd_cpu_dir, dev,
				&edp_reg_override_debugfs_fops);

	if (!d_edp_reg_override)
		goto edp_reg_override_err;

	edp->dir = vdd_cpu_dir;

	return 0;

edp_reg_override_err:
	debugfs_remove(d_edp_limit);
edp_limit_err:
	debugfs_remove(d_edp);
edp_err:
	debugfs_remove(vdd_cpu_dir);
vdd_cpu_dir_err:
	debugfs_remove(edp_dir);
edp_dir_err:
	return -ENOMEM;
}

#endif /* CONFIG_DEBUG_FS */

static int tegra_cpu_edp_notify(
	struct notifier_block *nb, unsigned long event, void *hcpu)
{
	struct platform_device *pdev = cpu_edp.pdev;
	struct device *dev = &pdev->dev;
	int ret = 0;
	unsigned int cpu_speed, new_speed;
	int cpu = (long)hcpu;

	switch (event) {
	case CPU_UP_PREPARE:
		mutex_lock(&tegra_cpu_lock);
		cpu_set(cpu, cpu_edp.edp_cpumask);
		tegra_edp_update_limit(dev);

		cpu_speed = cpufreq_get(0);
		new_speed = tegra_edp_governor_speed(dev, cpu_speed);
		if (new_speed < cpu_speed) {
			ret = cpufreq_update_policy(0);
			dev_dbg(dev, "cpu-tegra:%sforce EDP limit %u kHz\n",
				ret ? " failed to " : " ", new_speed);
		}
		if (ret) {
			cpu_clear(cpu, cpu_edp.edp_cpumask);
			tegra_edp_update_limit(dev);
		}
		mutex_unlock(&tegra_cpu_lock);
		break;
	case CPU_DEAD:
		mutex_lock(&tegra_cpu_lock);
		cpu_clear(cpu, cpu_edp.edp_cpumask);
		tegra_edp_update_limit(dev);
		mutex_unlock(&tegra_cpu_lock);
		cpufreq_update_policy(0);
		break;
	}

	return notifier_from_errno(ret);
}

static struct notifier_block tegra_cpu_edp_notifier = {
	.notifier_call = tegra_cpu_edp_notify,
};

/**
 * edp_cpufreq_policy_notifier - Notifier callback for cpufreq policy change.
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to highjack the notification on cpufreq policy transition.
 * Every time there is a change in policy, we will intercept and
 * update the cpufreq policy with edp constraints.
 *
 * Return: 0 (success)
 */
static int edp_cpufreq_policy_notifier(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	/* Limit max freq to be within edp limit. */

	if (policy->max != cpu_edp.edp_freq_limit)
		cpufreq_verify_within_limits(policy, 0, cpu_edp.edp_freq_limit);

	return 0;
}

/* Notifier for cpufreq policy change */
static struct notifier_block edp_cpufreq_notifier_block = {
	.notifier_call = edp_cpufreq_policy_notifier,
};

typedef int (*of_cpu_edp_init_t)(struct device_node *);

static int of_cpu_edp_init(struct device_node *np,
			     const struct of_device_id *matches)
{
	int ret;

	for_each_matching_node(np, matches) {
		const struct of_device_id *match = of_match_node(matches, np);
		of_cpu_edp_init_t cpu_edp_init = (of_cpu_edp_init_t)match->data;

		if (!cpu_edp_init)
			return -EINVAL;
		ret = cpu_edp_init(np);
		if (ret)
			return ret;
	}

	cpufreq_register_notifier(&edp_cpufreq_notifier_block,
				  CPUFREQ_POLICY_NOTIFIER);

	return 0;
}

static const struct of_device_id tegra_cpu_edp_match[] = {
	{ .compatible = "nvidia,tegra124-cpu-edp",
	  .data = tegra124_cpu_edp_init },
	{},
};

static int tegra_cpu_edp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned int regulator_ma;
	u32 val;
	int err;
	struct device_node *np = pdev->dev.of_node;

	if (!cpufreq_get(0)) {
		dev_warn(dev, "CPU clocks are not ready.\n");
		return -EPROBE_DEFER;
	}

	err = of_property_read_u32(np, "regulator-ma", &val);
	if (!err)
		regulator_ma = val;
	else
		regulator_ma = 15000;

	dev_info(dev, "CPU regulator %d mA\n", regulator_ma);

	platform_set_drvdata(pdev, &cpu_edp);

	of_cpu_edp_init(np, tegra_cpu_edp_match);

	cpu_edp.pdev = pdev;

	err = tegra_cpu_edp_init_data(dev, regulator_ma);
	if (err) {
		dev_err(dev, "Failed to calculate cpu edp limits\n");
		return 0;
	}

	register_hotcpu_notifier(&tegra_cpu_edp_notifier);

#ifdef CONFIG_DEBUG_FS
	tegra_cpu_edp_debugfs_init(dev);
#endif

	cpu_edp.edp_init_done = true;

	return 0;
}

static int tegra_cpu_edp_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(cpu_edp.dir);
#endif
	unregister_hotcpu_notifier(&tegra_cpu_edp_notifier);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_cpu_edp_suspend(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	int size;

	mutex_lock(&tegra_cpu_lock);

	if (edp->edp_limits) {
		size = edp->edp_limits_size - 1;
		edp->edp_freq_limit = edp->edp_limits[size].freq_limits[3];
	} else {
		size = edp->def_edp_limits_size - 1;
		edp->edp_freq_limit = edp->def_edp_limits[size].freq_limits[3];
	}

	mutex_unlock(&tegra_cpu_lock);

	cpufreq_update_policy(0);

	return 0;
}

static int tegra_cpu_edp_resume(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);

	mutex_lock(&tegra_cpu_lock);
	edp->edp_cpumask = *cpu_online_mask;
	tegra_edp_update_limit(dev);
	mutex_unlock(&tegra_cpu_lock);

	cpufreq_update_policy(0);

	return 0;
}

static const struct dev_pm_ops tegra_cpu_edp_pm_ops = {
	.suspend = tegra_cpu_edp_suspend,
	.resume = tegra_cpu_edp_resume,
};
#endif

static struct platform_driver tegra_cpu_edp_driver = {
	.probe  = tegra_cpu_edp_probe,
	.remove = tegra_cpu_edp_remove,
	.driver = {
		.name   = "tegra-cpu-edp",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_cpu_edp_match,
#ifdef CONFIG_PM_SLEEP
		.pm	= &tegra_cpu_edp_pm_ops,
#endif
	},
};

static int __init tegra_cpu_edp_init(void)
{
	return platform_driver_register(&tegra_cpu_edp_driver);
}

static void __exit tegra_cpu_edp_exit(void)
{
	platform_driver_unregister(&tegra_cpu_edp_driver);
}

module_init(tegra_cpu_edp_init);
module_exit(tegra_cpu_edp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tegra VDD_CPU EDP management");
