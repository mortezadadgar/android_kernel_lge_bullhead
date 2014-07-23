/*
 * Copyright (c) 2010-2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/hwmon-sysfs.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_thermal.h>

#include "thermal_core.h"

#define HIST_LEN (20)

struct therm_est_subdevice {
	struct thermal_zone_device *th;
	long coeffs[HIST_LEN];
	long hist[HIST_LEN];
};

struct therm_estimator {
	struct thermal_zone_device *thz;
	int num_trips;
	struct thermal_trip_info *trips;
	struct thermal_zone_params *tzp;

	struct workqueue_struct *workqueue;
	struct delayed_work therm_est_work;
	long cur_temp;
	long low_limit;
	long high_limit;
	int ntemp;
	long toffset;
	long polling_period;
	int tc1;
	int tc2;
	int ndevs;
	struct therm_est_subdevice *devs;

#ifdef CONFIG_PM
	struct notifier_block pm_nb;
#endif
};

static int therm_est_update_tripped_state(struct therm_estimator *est,
					  int trip, unsigned long *temp)
{
	struct thermal_trip_info *trip_state = &est->trips[trip];
	unsigned long trip_temp, zone_temp, hyst;
	struct thermal_instance *instance;

	est->thz->ops->get_trip_temp(est->thz, trip, &trip_temp);
	zone_temp = est->thz->temperature;
	est->thz->ops->get_trip_hyst(est->thz, trip, &hyst);

	/*
	 * Check the instance has been created, if so update the
	 * trip_temp and trip_state, and break to avoid going through
	 * the rest of the list.
	 */
	list_for_each_entry(instance, &est->thz->thermal_instances, tz_node) {
		if (instance->trip != trip)
			continue;
		if (zone_temp >= trip_temp) {
			trip_temp -= hyst;
			trip_state->tripped = true;
		} else if (trip_state->tripped) {
			trip_temp -= hyst;
			if (zone_temp < trip_temp)
				trip_state->tripped = false;
		}

		break;
	}

	*temp = trip_temp;

	return 0;
}

static void therm_est_update_limits(struct therm_estimator *est)
{
	const int MAX_HIGH_TEMP = 128000;
	long low_temp = 0, high_temp = MAX_HIGH_TEMP;
	long trip_temp, passive_low_temp = MAX_HIGH_TEMP;
	enum thermal_trip_type trip_type;
	struct thermal_trip_info *trip_state;
	int i;

	for (i = 0; i < est->thz->trips; i++) {
		trip_state = &est->trips[i];
		therm_est_update_tripped_state(est, i, &trip_temp);
		est->thz->ops->get_trip_type(est->thz, i, &trip_type);

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

	est->low_limit = low_temp;
	est->high_limit = high_temp;
}

static void therm_est_work_func(struct work_struct *work)
{
	int i, j, sum = 0;
	struct delayed_work *dwork = container_of(work,
					struct delayed_work, work);
	struct therm_estimator *est = container_of(dwork,
					struct therm_estimator,
					therm_est_work);

	for (i = 0; i < est->ndevs; i++) {
		unsigned long temp;
		struct thermal_zone_device *thz = est->devs[i].th;

		if (!thz || thermal_zone_get_temp(thz, &temp))
			temp = 25000;

		est->devs[i].hist[est->ntemp] = temp;
	}

	for (i = 0; i < est->ndevs; i++)
		for (j = 0; j < HIST_LEN; j++) {
			int index;
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			sum += est->devs[i].hist[index] *
				est->devs[i].coeffs[j];
		}

	est->cur_temp = sum / 100 + est->toffset;

	est->ntemp++;
	est->ntemp = est->ntemp % HIST_LEN;

	if (est->thz &&
	    ((est->cur_temp < est->low_limit) ||
	    (est->cur_temp >= est->high_limit))) {
		thermal_zone_device_update(est->thz);
		therm_est_update_limits(est);
	}

	queue_delayed_work(est->workqueue, &est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
}

static int therm_est_get_temp(void *dev, long *temp)
{
	struct therm_estimator *est = dev;

	*temp = est->cur_temp;
	return 0;
}

static int therm_est_get_trend(void *dev, int trip, long *trend)
{
	struct therm_estimator *est = dev;
	struct thermal_zone_device *thz = est->thz;
	long trip_temp;
	enum thermal_trip_type trip_type;
	int new_trend;
	int cur_temp;

	thz->ops->get_trip_temp(thz, trip, &trip_temp);
	thz->ops->get_trip_type(thz, trip, &trip_type);

	cur_temp = thz->temperature;
	new_trend = (est->tc1 * (cur_temp - thz->last_temperature)) +
		    (est->tc2 * (cur_temp - trip_temp));

	switch (trip_type) {
	case THERMAL_TRIP_ACTIVE:
		/* aggressive active cooling */
		*trend = THERMAL_TREND_RAISING;
		break;
	case THERMAL_TRIP_PASSIVE:
		if (new_trend > 0)
			*trend = THERMAL_TREND_RAISING;
		else if (new_trend < 0)
			*trend = THERMAL_TREND_DROPPING;
		else
			*trend = THERMAL_TREND_STABLE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t show_coeff(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	ssize_t len, total_len = 0;
	int i, j;
	for (i = 0; i < est->ndevs; i++) {
		len = snprintf(buf + total_len,
				PAGE_SIZE - total_len, "[%d]", i);
		total_len += len;
		for (j = 0; j < HIST_LEN; j++) {
			len = snprintf(buf + total_len,
					PAGE_SIZE - total_len, " %ld",
					est->devs[i].coeffs[j]);
			total_len += len;
		}
		len = snprintf(buf + total_len, PAGE_SIZE - total_len, "\n");
		total_len += len;
	}
	return strlen(buf);
}

static ssize_t set_coeff(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int devid, scount;
	long coeff[20];

	if (HIST_LEN > 20)
		return -EINVAL;

	scount = sscanf(buf,
			"[%d] %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld "
			"%ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
			&devid,
			&coeff[0],
			&coeff[1],
			&coeff[2],
			&coeff[3],
			&coeff[4],
			&coeff[5],
			&coeff[6],
			&coeff[7],
			&coeff[8],
			&coeff[9],
			&coeff[10],
			&coeff[11],
			&coeff[12],
			&coeff[13],
			&coeff[14],
			&coeff[15],
			&coeff[16],
			&coeff[17],
			&coeff[18],
			&coeff[19]);

	if (scount != HIST_LEN + 1)
		return -EINVAL;

	if (devid < 0 || devid >= est->ndevs)
		return -EINVAL;

	/* This has obvious locking issues but don't worry about it */
	memcpy(est->devs[devid].coeffs, coeff, sizeof(coeff[0]) * HIST_LEN);

	return count;
}

static ssize_t show_offset(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%ld\n", est->toffset);
	return strlen(buf);
}

static ssize_t set_offset(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int offset;

	if (kstrtoint(buf, 0, &offset))
		return -EINVAL;

	est->toffset = offset;

	return count;
}

static ssize_t show_temps(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	ssize_t total_len = 0;
	int i, j;

	/* This has obvious locking issues but don't worry about it */
	for (i = 0; i < est->ndevs; i++) {
		total_len += snprintf(buf + total_len,
					PAGE_SIZE - total_len, "[%d]", i);
		for (j = 0; j < HIST_LEN; j++) {
			int index;
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			total_len += snprintf(buf + total_len,
						PAGE_SIZE - total_len, " %ld",
						est->devs[i].hist[index]);
		}
		total_len += snprintf(buf + total_len,
					PAGE_SIZE - total_len, "\n");
	}
	return strlen(buf);
}

static ssize_t show_tc1(struct device *dev,
			struct device_attribute *da,
			char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%d\n", est->tc1);
	return strlen(buf);
}

static ssize_t set_tc1(struct device *dev,
			struct device_attribute *da,
			const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int tc1;

	if (kstrtoint(buf, 0, &tc1))
		return -EINVAL;

	est->tc1 = tc1;

	return count;
}

static ssize_t show_tc2(struct device *dev,
			struct device_attribute *da,
			char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%d\n", est->tc2);
	return strlen(buf);
}

static ssize_t set_tc2(struct device *dev,
			struct device_attribute *da,
			const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int tc2;

	if (kstrtoint(buf, 0, &tc2))
		return -EINVAL;

	est->tc2 = tc2;

	return count;
}

static struct sensor_device_attribute therm_est_nodes[] = {
	SENSOR_ATTR(coeff, S_IRUGO | S_IWUSR, show_coeff, set_coeff, 0),
	SENSOR_ATTR(offset, S_IRUGO | S_IWUSR, show_offset, set_offset, 0),
	SENSOR_ATTR(tc1, S_IRUGO | S_IWUSR, show_tc1, set_tc1, 0),
	SENSOR_ATTR(tc2, S_IRUGO | S_IWUSR, show_tc2, set_tc2, 0),
	SENSOR_ATTR(temps, S_IRUGO, show_temps, 0, 0),
};

static int therm_est_init_history(struct therm_estimator *est)
{
	int i;

	for (i = 0; i < est->ndevs; i++) {
		struct thermal_zone_device *thz;
		unsigned long temp;
		int j;
		thz = est->devs[i].th;

		if (!thz || thermal_zone_get_temp(thz, &temp))
			temp = 25000;

		for (j = 0; j < HIST_LEN; j++)
			est->devs[i].hist[j] = temp;
	}

	return 0;
}

#ifdef CONFIG_PM
static int therm_est_pm_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct therm_estimator *est = container_of(
					nb,
					struct therm_estimator,
					pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&est->therm_est_work);
		break;
	case PM_POST_SUSPEND:
		est->low_limit = 0;
		est->high_limit = 0;
		therm_est_init_history(est);
		queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
		break;
	}

	return NOTIFY_OK;
}
#endif

static int __parse_dt_subdev(struct device *dev,
			     struct therm_estimator *est)
{
	struct device_node *np = dev->of_node;
	struct device_node *ch, *gch;
	int ndevs, i;

	ch = of_get_child_by_name(np, "sub-devs");
	if (!ch)
		return -ENODATA;

	ndevs = of_get_child_count(ch);
	if (ndevs == 0)
		return -ENODATA;

	est->ndevs = ndevs;

	if (!est->devs) {
		est->devs = devm_kzalloc(dev,
				sizeof(struct therm_est_subdevice) * ndevs,
				GFP_KERNEL);
		if (!est->devs)
			return -ENOMEM;
	}

	i = 0;
	for_each_child_of_node(ch, gch) {
		struct device_node *subdev_node;
		struct thermal_zone_device *th;
		const char *str;
		int j, ret;

		subdev_node = of_parse_phandle(gch, "dev", 0);
		if (IS_ERR(subdev_node))
			return -ENODATA;
		th = thermal_zone_get_zone_by_node(subdev_node);
		if (IS_ERR(th))
			return -EPROBE_DEFER;

		ret = of_property_read_string(gch, "coeffs", &str);
		if (ret < 0)
			return ret;
		j = 0;
		while (str && (j < HIST_LEN)) {
			char *sbegin;
			long *res;

			str = skip_spaces(str);
			sbegin = strsep((char **)&str, " ");
			res = &est->devs[i].coeffs[j++];
			if (!sbegin ||
			    (kstrtol((const char *)sbegin, 10, res) < 0))
				break;
		}

		if (j != HIST_LEN)
			return -EINVAL;

		est->devs[i].th = th;
		i++;
	}

	return 0;
}

static struct therm_estimator *therm_est_get_pdata(struct device *dev)
{
	struct therm_estimator *est;
	struct device_node *np = dev->of_node;
	u32 val;
	int ret;

	est = devm_kzalloc(dev,
			   sizeof(struct therm_estimator), GFP_KERNEL);
	if (IS_ERR_OR_NULL(est))
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "toffset", &val);
	if (ret < 0)
		goto error;
	est->toffset = val;

	ret = of_property_read_u32(np, "tc1", &val);
	if (ret < 0)
		goto error;
	est->tc1 = val;

	ret = of_property_read_u32(np, "tc2", &val);
	if (ret < 0)
		goto error;
	est->tc2 = val;

	ret = of_property_read_u32(np, "polling-period", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	est->polling_period = val;

	ret = __parse_dt_subdev(dev, est);
	if (ret < 0)
		goto error;

	return est;

error:
	devm_kfree(dev, est);
	return ERR_PTR(ret);
}

static int therm_est_probe(struct platform_device *pdev)
{
	struct therm_estimator *est;
	int i, ret;

	est = therm_est_get_pdata(&pdev->dev);
	if (IS_ERR(est))
		return PTR_ERR(est);

	platform_set_drvdata(pdev, est);

	/* initialize history */
	therm_est_init_history(est);

	est->thz = thermal_zone_of_sensor_register(&pdev->dev,
						   0,
						   est,
						   therm_est_get_temp,
						   therm_est_get_trend);
	if (IS_ERR(est->thz))
		return PTR_ERR(est->thz);

	thermal_update_governor(est->thz, "pid_thermal_gov");

	est->num_trips = est->thz->trips;

	est->trips = devm_kzalloc(&pdev->dev,
			sizeof(struct thermal_trip_info) * est->num_trips,
			GFP_KERNEL);
	if (IS_ERR_OR_NULL(est->trips)) {
		ret = -ENOMEM;
		goto err;
	}

	est->workqueue = alloc_workqueue(dev_name(&pdev->dev),
				    WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (IS_ERR_OR_NULL(est->workqueue)) {
		ret = -EINVAL;
		goto err;
	}

	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));

#ifdef CONFIG_PM
	est->pm_nb.notifier_call = therm_est_pm_notify,
	register_pm_notifier(&est->pm_nb);
#endif

	for (i = 0; i < ARRAY_SIZE(therm_est_nodes); i++)
		device_create_file(&pdev->dev, &therm_est_nodes[i].dev_attr);

	return 0;

err:
	thermal_zone_of_sensor_unregister(&pdev->dev, est->thz);
	return ret;
}

static int therm_est_remove(struct platform_device *pdev)
{
	struct therm_estimator *est = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&est->therm_est_work);
	destroy_workqueue(est->workqueue);

	thermal_zone_of_sensor_unregister(&pdev->dev, est->thz);

#ifdef CONFIG_PM
	unregister_pm_notifier(&est->pm_nb);
#endif

	return 0;
}

static const struct of_device_id therm_est_match[] = {
	{ .compatible = "nvidia,tegra124-therm-est", },
	{},
};

static struct platform_driver therm_est_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "therm_est",
		.of_match_table = therm_est_match,
	},
	.probe  = therm_est_probe,
	.remove = therm_est_remove,
};

static int __init therm_est_driver_init(void)
{
	return platform_driver_register(&therm_est_driver);
}
module_init(therm_est_driver_init);
