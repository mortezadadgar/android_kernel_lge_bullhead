/*
 *	mfld_ssp.h - ASoC CPU DAI driver for
 *
 *  Copyright (C) 2012 Intel Corp
 *  Authors:	Selma Bensaid <selma.bensaid@intel.com>
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
 */
#ifndef MID_SSP_H_
#define MID_SSP_H_

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include <linux/intel_mid_i2s_if.h>

#define SSP_MODEM_DAI_NAME "ssp-modem-cpu-dai"
#define SSP_BT_DAI_NAME "ssp-bt-cpu-dai"
#define SSP0_CARD_AK4614_DAI_NAME "ssp0-ak4614-cpu-dai"
#define SSP1_CARD_AK4614_DAI_NAME "ssp1-ak4614-cpu-dai"
#define SSP2_CARD_AK4614_DAI_NAME "ssp2-ak4614-cpu-dai"

#define SSP_MIN_RATE 		8000
#define SSP_MAX_RATE 		48000
#define SSP_MIN_PERIODS		10
#define SSP_MAX_PERIODS		50
#define SSP_FIFO_SIZE 		0

/* I2S support */
#define SSP_MONO_CHANNEL 1
#define SSP_STEREO_CHANNEL 2

#define SSP_MAX_BUFFER 			96000
#define SSP_MIN_BUFFER 			96000
#define SSP_MIN_PERIOD_BYTES 	1764
#define SSP_MAX_PERIOD_BYTES 	48000

/* TDM 8 support */
#define SSP_TDM8_CHANNEL 8

#define SSP_MAX_BUFFER_TDM		768000 /*500ms@48,4bytes,8ch - BYT*/
#define SSP_MIN_PERIOD_BYTES_TDM	15360  /*10ms@48,4bytes,8ch - BYT*/
#define SSP_MAX_PERIOD_BYTES_TDM	76800/*Use DMA transfer limit or 50ms@48,4bytes,8ch - BYT*/

#define TRISTATE_BIT			0
#define FRAME_SYNC_RELATIVE_TIMING_BIT	1
#define DUMMY_START_ONE_PERIOD_OFFSET	2
#define DUMMY_START_ONE_PERIOD_MASK     0x3

#define IS_TRISTATE_ENABLED(x) (x & BIT(TRISTATE_BIT))
#define IS_NEXT_FRMS_ASSERTED_WITH_LSB_PREVIOUS_FRM(x) \
			((x & BIT(FRAME_SYNC_RELATIVE_TIMING_BIT)) \
					>> FRAME_SYNC_RELATIVE_TIMING_BIT)
#define IS_DUMMY_START_ONE_PERIOD_OFFSET(x) \
			((x >> DUMMY_START_ONE_PERIOD_OFFSET) \
					& DUMMY_START_ONE_PERIOD_MASK)

#define MID_SSP_RX_FIFO_THRESHOLD 8
#define MID_SSP_TX_FIFO_THRESHOLD 7

/*TODO: redefine bits later as clash with some  ASOC defines*/
/* data driven FALLING, data sampled RISING, idle LOW */
#define SSP_DAI_SCMODE_0        (1 << 4)
/* data driven RISING, data sampled FALLING, idle LOW */
#define SSP_DAI_SCMODE_1		(2 << 4)
/* data driven RISING, data sampled FALLING, idle HIGH */
#define SSP_DAI_SCMODE_2		(3 << 4)
/* data driven FALLING, data sampled RISING, idle HIGH */
#define SSP_DAI_SCMODE_3		(4 << 4)


/*
 * Structures Definition
 */

struct intel_ssp_config {
	struct intel_mid_i2s_hdl *i2s_handle;
	struct intel_mid_i2s_settings i2s_settings;
	struct ssp_platform_config *pdata;
	bool intel_mid_dma_alloc;
};

struct intel_ssp_info {
	struct workqueue_struct *ssp_dai_wq;
	struct ssp_platform_config *pdata;
	bool ssp_dai_tx_allocated;
	bool ssp_dai_rx_allocated;
};

struct intel_alsa_ssp_stream_info {
	struct snd_pcm_substream *substream;
	struct work_struct ssp_ws;
	struct intel_ssp_config *ssp_config;
	unsigned long stream_status;
	u32 period_req_index;
	s32 period_cb_index;
	u8 *addr;
	int length;
};


/*
 * Enum Definition
 */

enum intel_alsa_ssp_stream_status {
	INTEL_ALSA_SSP_STREAM_INIT = 0,
	INTEL_ALSA_SSP_STREAM_STARTED,
	INTEL_ALSA_SSP_STREAM_RUNNING,
	INTEL_ALSA_SSP_STREAM_PAUSED,
	INTEL_ALSA_SSP_STREAM_DROPPED,
};
enum ssp_clk_def {
	SSP_CLK_ONCHIP = 0x0,
	SSP_CLK_NET,
	SSP_CLK_EXT,
	SSP_CLK_AUDIO
};

#endif /* MID_SSP_H_ */
