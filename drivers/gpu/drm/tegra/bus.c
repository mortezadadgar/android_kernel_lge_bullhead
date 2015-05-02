/*
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "drm.h"

static DEFINE_MUTEX(clients_lock);
static LIST_HEAD(clients);

void drm_host1x_exit(struct drm_driver *driver, struct host1x_device *device)
{
	struct tegra_drm *tegra = dev_get_drvdata(&device->dev);

	drm_put_dev(tegra->drm);
}


int drm_host1x_init(struct drm_driver *driver, struct host1x_device *device)
{
	mutex_init(&clients_lock);
	INIT_LIST_HEAD(&device->clients);

	return 0;
}

int drm_host1x_register(struct host1x_client *client)
{
	mutex_lock(&clients_lock);
	list_add_tail(&client->list, &clients);
	mutex_unlock(&clients_lock);
	return 0;
}

int drm_host1x_unregister(struct host1x_client *client)
{
	struct host1x_client *c;

	list_for_each_entry(c, &clients, list) {
		if (c == client) {
			list_del(&c->list);
			return 0;
		}
	}

	return 1;
}

int drm_host1x_device_init(struct drm_device *drm, struct host1x_device *device)
{
	struct host1x_client *client;
	int err;

	mutex_lock(&clients_lock);
	list_for_each_entry(client, &clients, list) {
		if (client->ops && client->ops->init) {
			client->parent = drm->dev;
			err = client->ops->init(client);
			if (err < 0) {
				dev_err(&device->dev,
					"failed to initialize %s: %d\n",
					dev_name(client->dev), err);
				mutex_unlock(&clients_lock);
				return err;
			}
		}
	}

	mutex_unlock(&clients_lock);
	return 0;
}

int drm_host1x_device_exit(struct host1x_device *device)
{
	struct host1x_client *client;
	int err;

	mutex_lock(&clients_lock);
	list_for_each_entry_reverse(client, &clients, list) {
		if (client->ops && client->ops->exit) {
			err = client->ops->exit(client);
			if (err < 0) {
				dev_err(&device->dev,
					"failed to cleanup %s: %d\n",
					dev_name(client->dev), err);
				mutex_unlock(&clients_lock);
				return err;
			}
		}
	}

	mutex_unlock(&clients_lock);
	return 0;
}


