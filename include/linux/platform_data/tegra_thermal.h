/*
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
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

#ifndef _TEGRA_THERMAL__H
#define _TEGRA_THERMAL__H

/**
 * struct thermal_trip_info
 * @trip_temp: the temperature for this trip point.
 * @trip_type: the type of the trip point: active, passive, hot, critical.
 * @upper: the Maximum cooling state for this trip point.
 * @lower: the Minimum cooling state for this trip point.
 * hysteresis: hysteresis value for a trip point.
 * tripped: indicate if the trip point tripped.
 * bound: indicate if the trip point bind with cooling device.
 * cdev_type: the cooling device name which trip point bound with.
 */
struct thermal_trip_info {
	long trip_temp;
	enum thermal_trip_type trip_type;
	unsigned long upper;
	unsigned long lower;
	long hysteresis;
	bool tripped;
	bool bound;
	char *cdev_type;
};

struct tegra_cooling_device {
	char *cdev_type;
	int *trip_temperatures;
	int trip_temperatures_num;
};

#endif /* _TEGRA_THERMAL_H */
