/*
 * byt_machine.h - ASoc Machine driver for Intel Baytrail platform
 */
#ifndef __BYT_MACHINE_H__
#define __BYT_MACHINE_H__

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "mid_ssp.h"

/*
 * Bit depth of PCM samples
 */
#define SSP_16_SLOT_WIDTH		16
#define SSP_S24_SLOT_WIDTH		24
#define SSP_S32_SLOT_WIDTH		32

/*
 * Stereo, 2 slot are needed.
 * Each TX/RX bit in slot mask represents 1 channel.
 * LSB = ch 1, MSB = ch 8.
 */
#define SSP_STEREO_SLOT_NB_SLOT		2
#define SSP_STEREO_SLOT_RX_MASK		0x3
#define SSP_STEREO_SLOT_TX_MASK		0x3

/*
 * For TDM 8, 8 slots are needed.
 * Each TX/RX bit in slot mask represents 1 channel.
 * LSB = ch 1, MSB = ch 8.
 */
#define SSP_TDM8_SLOT_NB_SLOT		8
#define SSP_TDM8_SLOT_RX_MASK		0xFF
#define SSP_TDM8_SLOT_TX_MASK		0xFF

static struct snd_pcm_hardware ssp_pcm_hw_stereo = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_MMAP|
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S24_LE),
	.rates = (SNDRV_PCM_RATE_48000),
	.rate_min = SSP_MAX_RATE,      /* Force to support only 48kHz*/
	.rate_max = SSP_MAX_RATE,
	.channels_min =	SSP_MONO_CHANNEL,
	.channels_max =	SSP_STEREO_CHANNEL,
	.buffer_bytes_max = SSP_MAX_BUFFER,
	.period_bytes_min = SSP_MIN_PERIOD_BYTES,
	.period_bytes_max = SSP_MAX_PERIOD_BYTES,
	.periods_min = SSP_MIN_PERIODS,
	.periods_max = SSP_MAX_PERIODS,
	.fifo_size = SSP_FIFO_SIZE,
};

static struct snd_pcm_hardware ssp_pcm_hw_tdm8 = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_MMAP|
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S32_LE),
	.rates = (SNDRV_PCM_RATE_48000),
	.rate_min = SSP_MAX_RATE,      /* Force to support only 48kHz*/
	.rate_max = SSP_MAX_RATE,
	.channels_min =	SSP_TDM8_CHANNEL, /* Force to support only 8 channels */
	.channels_max =	SSP_TDM8_CHANNEL,
	.buffer_bytes_max = SSP_MAX_BUFFER_TDM,
	.period_bytes_min = SSP_MIN_PERIOD_BYTES_TDM,
	.period_bytes_max = SSP_MAX_PERIOD_BYTES_TDM,
	.periods_min = SSP_MIN_PERIODS,
	.periods_max = SSP_MAX_PERIODS,
	.fifo_size = SSP_FIFO_SIZE,
};

#endif
