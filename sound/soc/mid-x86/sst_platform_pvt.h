/*
 *  sst_platform_pvt.h - Intel MID Platform driver header file
 *
 *  Copyright (C) 2010 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 */

#ifndef __SST_PLATFORMDRVPVT_H__
#define __SST_PLATFORMDRVPVT_H__

#define SST_MONO		1
#define SST_STEREO		2
#define SST_TDM			8

#define SST_MIN_RATE		8000
#define SST_MAX_RATE		48000
#define SST_MIN_CHANNEL		1
#define SST_MAX_CHANNEL_STEREO	2
#define SST_MAX_CHANNEL_TDM	8

#define SST_MAX_BUFFER_STEREO		96000 /*500ms@48,16bit,2ch - BYT*/
#define SST_MIN_PERIOD_BYTES_STEREO	384  /*2ms@48,16bit,2ch - BYT*/
#define SST_MAX_PERIOD_BYTES_STEREO	48000 /*250ms@48,16bit,2ch - BYT*/
#define SST_MAX_BUFFER_TDM		384000 /*500ms@48,16bit,8ch - BYT*/
#define SST_MIN_PERIOD_BYTES_TDM	1536  /*2ms@48,16bit,8ch - BYT*/
#define SST_MAX_PERIOD_BYTES_TDM	192000 /*250ms@48,16bit,8ch - BYT*/

#define SST_MIN_PERIODS		2
#define SST_MAX_PERIODS		250
#define SST_FIFO_SIZE		0
#define SST_CLK_UNINIT		0x03
#define SST_CODEC_TYPE_PCM	1

#define SST_HEADSET_DAI "Headset-cpu-dai"
#define SST_SPEAKER_DAI "Speaker-cpu-dai"
#define SST_VIBRA1_DAI "Vibra1-cpu-dai"
#define SST_VIBRA2_DAI "Vibra2-cpu-dai"
#define SST_VOICE_DAI "Voice-cpu-dai"


/* DAI name for audio input/output DAI in AK4614 */
#define SST_BYT_FRONT_DAI	"Front-cpu-dai"
#define SST_BYT_REAR_DAI	"Rear-cpu-dai"
#define SST_BYT_SIDE_DAI	"Side-cpu-dai"
#define SST_BYT_CENTREWOOFER_DAI	"Centrewoofer-cpu-dai"
#define SST_BYT_HEADPHONE_DAI	"Headphone-cpu-dai"
#define SST_BYT_AUXOUT_DAI	"Auxout-cpu-dai"
#define SST_BYT_MICIN1_DAI	"Mic1-cpu-dai" 
#define SST_BYT_MICIN2_DAI	"Mic2-cpu-dai" 
#define SST_BYT_AUXIN_DAI	"Auxin-cpu-dai"


struct sst_device;

enum sst_drv_status {
	SST_PLATFORM_UNINIT,
	SST_PLATFORM_INIT,
	SST_PLATFORM_RUNNING,
	SST_PLATFORM_PAUSED,
	SST_PLATFORM_DROPPED,
};

ushort get_num_channels(void);

#endif
