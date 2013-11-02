/*
 * intel-byt_sound.c: Intel platform setup code for Baytrail audio
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sfi.h>
#include <linux/module.h>
#include <asm/intel-mid.h>

extern void *byt_audio_platform_data(void *info);

const struct devs_id __initconst platform_device_ids[] = {
#ifdef CONFIG_SND_BYT_AK4614
		{"byt_ak4614",SFI_DEV_TYPE_IPC, 1, &byt_audio_platform_data, NULL },
#elif defined CONFIG_SND_BYT_RT5642
		{"byt_rt5642",SFI_DEV_TYPE_IPC, 1, &byt_audio_platform_data, NULL },
#elif defined CONFIG_SND_BYT_MAX98090
		{"byt_max98090",SFI_DEV_TYPE_IPC, 1, &byt_audio_platform_data, NULL },
#endif
		{},
};

static void intel_handle_platform_dev ( void )
{
	void *pdata = NULL;
	const struct devs_id *dev = &platform_device_ids[0];
	pr_debug("Platform : intel_handle_platform_dev entry\n");

	/*Callback function to register platform devices*/
	pdata = dev->get_platform_data(NULL);
	pr_debug("Platform : intel_handle_platform_dev exit\n");

}

static int __init intel_platform_init(void)
{
	intel_handle_platform_dev ();

	return 0;
}
device_initcall(intel_platform_init);
