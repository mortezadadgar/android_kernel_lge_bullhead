/*
 * platform_byt_audio.h: BYT audio platform data header file
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */ 

#ifndef _PLATFORM_BYT_AUDIO_H
#define _PLATFORM_BYT_AUDIO_H

#include <linux/intel_mid_i2s_common.h>
#include <../../../../sound/soc/mid-x86/mid_ssp.h>

enum {
  SSP_0 =0,
  SSP_1 =1,
  SSP_2 =2,
};

struct ssp_platform_config{
    struct intel_mid_i2s_settings i2s_settings;
    bool is_IA;
    int port_number;
    bool is_tdm;
    int is_master;
    int slot_width;
    int num_of_slot;
    int slot_mask;
    int bitclk;
    int framesync;
};

#endif
