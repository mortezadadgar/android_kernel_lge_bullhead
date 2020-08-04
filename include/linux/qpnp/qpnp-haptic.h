/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __QPNP_HAPTIC_H

/* interface for the other module to play different sequences */
#ifdef CONFIG_QPNP_HAPTIC
int qpnp_hap_play_byte(u8 data, bool on);
void qpnp_disable_haptics(void);
void qpnp_enable_haptics(void);

#else
int qpnp_hap_play_byte(u8 data, bool on);
{
	return 0;
}
void qpnp_disable_haptics(void) { }
void qpnp_disable_haptics(void) { }
#endif
#endif
