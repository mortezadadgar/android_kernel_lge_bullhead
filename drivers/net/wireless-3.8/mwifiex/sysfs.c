/* Marvell wireless LAN device driver: sysfs
 *
 * Copyright (C) 2015, Marvell International Ltd.
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available on the worldwide web at
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include "main.h"

static ssize_t
mwifiex_sysfs_get_rf_led(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mwifiex_private *priv = to_net_dev(dev)->ml_priv;

	return snprintf(buf, 10, "%d\n", priv->adapter->rf_led_enabled);
}

static ssize_t
mwifiex_sysfs_set_rf_led(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mwifiex_private *priv = to_net_dev(dev)->ml_priv;
	bool res;

	if (strtobool(buf, &res))
		return -EINVAL;

	priv->adapter->rf_led_enabled = !!res;

	return count;
}

static DEVICE_ATTR(rf_led, S_IRUGO | S_IWUSR,
		   mwifiex_sysfs_get_rf_led,
		   mwifiex_sysfs_set_rf_led);

int mwifiex_sysfs_register(struct mwifiex_private *priv)
{
	int ret;

	/* Create sysfs file to control RF LED feature */
	ret = device_create_file(&priv->netdev->dev, &dev_attr_rf_led);
	if (ret)
		dev_err(priv->adapter->dev,
			"failed to create sysfs file rf_led\n");

	return ret;
}

void mwifiex_sysfs_unregister(struct mwifiex_private *priv)
{
	device_remove_file(&priv->netdev->dev, &dev_attr_rf_led);
}

