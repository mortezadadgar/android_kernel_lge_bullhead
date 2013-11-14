/*
 *  sst_platform.c - Intel MID Platform driver
 *
 *  Copyright (C) 2010-2013 Intel Corp
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
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/intel_sst_ioctl.h>
#include <sound/compress_driver.h>
#include "sst_platform.h"
#include "sst_platform_pvt.h"
#include <linux/delay.h>
#include <linux/module.h>

/* Defaults for Module Parameter */
#define DEFAULT_CHANNEL_NUM 2

static ushort useMultiChannels = DEFAULT_CHANNEL_NUM;
module_param(useMultiChannels, ushort, 0);
MODULE_PARM_DESC(useMultiChannels, "To configure & select how many channels in use for SST driver");

struct sst_device *sst_dsp;
static DEFINE_MUTEX(sst_dsp_lock);

static struct sst_device *sst;

static struct snd_pcm_hardware sst_platform_pcm_hw_stereo = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_MMAP|
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S24 | SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_48000),
	.rate_min = SST_MAX_RATE,      /* Force to support only 48kHz*/
	.rate_max = SST_MAX_RATE,
	.channels_min =	SST_MAX_CHANNEL_STEREO,  /* Force to support only stereo */
	.channels_max =	SST_MAX_CHANNEL_STEREO,
	.buffer_bytes_max = SST_MAX_BUFFER_STEREO,
	.period_bytes_min = SST_MIN_PERIOD_BYTES_STEREO,
	.period_bytes_max = SST_MAX_PERIOD_BYTES_STEREO,
	.periods_min = SST_MIN_PERIODS,
	.periods_max = SST_MAX_PERIODS,
	.fifo_size = SST_FIFO_SIZE,
};

static struct snd_pcm_hardware sst_platform_pcm_hw_tdm8 = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_MMAP|
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S24 | SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_48000),
	.rate_min = SST_MAX_RATE,      /* Force to support only 48kHz*/
	.rate_max = SST_MAX_RATE,
	.channels_min =	SST_MAX_CHANNEL_TDM,  /* Force to support only 8 channels */
	.channels_max =	SST_MAX_CHANNEL_TDM,
	.buffer_bytes_max = SST_MAX_BUFFER_TDM,
	.period_bytes_min = SST_MIN_PERIOD_BYTES_TDM,
	.period_bytes_max = SST_MAX_PERIOD_BYTES_TDM,
	.periods_min = SST_MIN_PERIODS,
	.periods_max = SST_MAX_PERIODS,
	.fifo_size = SST_FIFO_SIZE,
};

static const struct snd_soc_component_driver sst_component = {
	.name		= "sst",
};

ushort get_num_channels(void)
{
	return useMultiChannels;
}

static int sst_platform_ihf_set_tdm_slot(struct snd_soc_dai *dai,
			unsigned int tx_mask, unsigned int rx_mask,
			int slots, int slot_width)
{
	struct snd_sst_runtime_params params_data;
	int channels = slots;

	/* registering with SST driver to get access to SST APIs to use */
	if (!sst_dsp) {
		pr_err("sst: DSP not registered\n");
		return -EIO;
	}
	params_data.type = SST_SET_CHANNEL_INFO;
	params_data.str_id = SND_SST_DEVICE_IHF;
	params_data.size = sizeof(channels);
#ifdef SST_DRV_BYT
	params_data.params[0] = channels;
#else
	params_data.addr = &channels;
#endif
	return sst_dsp->ops->set_generic_params(SST_SET_RUNTIME_PARAMS,
							(void *)&params_data);
}

/* helper functions */
static inline void sst_set_stream_status(struct sst_runtime_stream *stream,
					int state)
{
	unsigned long flags;
	spin_lock_irqsave(&stream->status_lock, flags);
	stream->stream_status = state;
	spin_unlock_irqrestore(&stream->status_lock, flags);
}

static inline int sst_get_stream_status(struct sst_runtime_stream *stream)
{
	int state;
	unsigned long flags;

	spin_lock_irqsave(&stream->status_lock, flags);
	state = stream->stream_status;
	spin_unlock_irqrestore(&stream->status_lock, flags);
	return state;
}

#ifdef SST_DRV_BYT
/* This function is specific to BYT because of <snd_sst_frames_info> is passed to LPE Core */
static void sst_fill_frame_info(struct snd_pcm_substream *substream,
                               struct snd_sst_frames_info *frame_info)
{
	unsigned int channels;
	snd_pcm_uframes_t period_size;
	ssize_t periodbytes;
	ssize_t buffer_bytes;
	u32 buffer_addr;

	buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	buffer_addr = substream->dma_buffer.addr;

	channels = substream->runtime->channels;
	period_size = substream->runtime->period_size;
	periodbytes = samples_to_bytes(substream->runtime, period_size);

	/* Fill in frame info */
	frame_info->num_entries = 1;  /* filling jut one DMA buffer */
	frame_info->rsrvd =0;
	frame_info->ring_buf_info[0].addr = buffer_addr;
	frame_info->ring_buf_info[0].size = buffer_bytes;

	/* For frame_info.frag_size is the period count = # of frame in 4ms period
	 * that is pulled by FW.
	 * (Previously calculated as "frame_info->frag_size = periodbytes * channels")
	 */
	frame_info->frag_size = (substream->runtime->rate) *
		(substream->runtime->channels)*(substream->runtime->sample_bits/8) ;
	frame_info->frag_size = (frame_info->frag_size/1000)*4 ;

	pr_debug("period_size = %d\n", frame_info->frag_size);
	pr_debug("ring_buf_addr = 0x%x\n", frame_info->ring_buf_info[0].addr);
}
#endif

static void sst_fill_pcm_params(struct snd_pcm_substream *substream,
				struct snd_sst_stream_params *param)
{
	param->uc.pcm_params.num_chan = (u8) substream->runtime->channels;
	param->uc.pcm_params.pcm_wd_sz = substream->runtime->sample_bits;
	param->uc.pcm_params.reserved = 0;
	param->uc.pcm_params.sfreq = substream->runtime->rate;

	/* PCM stream via ALSA interface */
	param->uc.pcm_params.use_offload_path = 0;

	memset(param->uc.pcm_params.channel_map, 0, sizeof(u8));

	if ( param->uc.pcm_params.num_chan == 1 )
	{
		param->uc.pcm_params.channel_map[0] = 0x00;
	}
	else
	{
		param->uc.pcm_params.channel_map[0] = 0x00;
		param->uc.pcm_params.channel_map[1] = 0x01;     
	}

	pr_debug("sfreq= %d, wd_sz = %d\n",
	        param->uc.pcm_params.sfreq, param->uc.pcm_params.pcm_wd_sz);
}

static int sst_platform_alloc_stream(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream =
			substream->runtime->private_data;
	struct snd_sst_stream_params param = {{{0,},},};
	struct snd_sst_params str_params = {0};
	struct snd_sst_frames_info frame_info = {0};
	int ret_val;

	/* set codec params and inform SST driver the same */
	sst_fill_pcm_params(substream, &param);
	sst_fill_frame_info(substream, &frame_info);
	substream->runtime->dma_area = substream->dma_buffer.area;
	str_params.sparams = param;

	str_params.frame_info = frame_info;

	/* Todo: codec type should be enhanced to handle all codec type
	 * Currently its hard coded to PCM only
	 */
	str_params.codec =  SST_CODEC_TYPE_PCM;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		str_params.ops = STREAM_OPS_PLAYBACK;
		str_params.device_type = substream->pcm->device + 1;
		pr_debug("Playback stream,Device %d\n",
					substream->pcm->device);
	} else {
		str_params.ops = STREAM_OPS_CAPTURE;
		str_params.device_type = SND_SST_DEVICE_CAPTURE;
		pr_debug("Capture stream,Device %d\n",
					substream->pcm->device);
	}

	ret_val = stream->ops->open(&str_params);
	pr_debug("platform prepare: stream open ret_val = 0x%x\n", ret_val);
	if (ret_val <= 0)
		return ret_val;

	stream->stream_info.str_id = ret_val;
	pr_debug("platform allocated strid:  %d\n", stream->stream_info.str_id);

	return ret_val;
}

static void sst_period_elapsed(void *mad_substream)
{
	struct snd_pcm_substream *substream = mad_substream;
	struct sst_runtime_stream *stream;
	int status;

	if (!substream || !substream->runtime) {
		pr_debug("In %s : Null Substream pointer\n", __func__);
		return;
	}

	stream = substream->runtime->private_data;
	if (!stream) {
		pr_debug("In %s : Null Stream pointer\n", __func__);
		return;
	}
	status = sst_get_stream_status(stream);
	if (status != SST_PLATFORM_RUNNING) {
		pr_debug("In %s : Stream Status=%d\n", __func__, status);
		return;
	}
	snd_pcm_period_elapsed(substream);
}

static int sst_platform_init_stream(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream =
			substream->runtime->private_data;
	int ret_val;

	pr_debug("setting buffer ptr param\n");
	sst_set_stream_status(stream, SST_PLATFORM_INIT);
	stream->stream_info.period_elapsed = sst_period_elapsed;
	stream->stream_info.mad_substream = substream;
	stream->stream_info.buffer_ptr = 0;
	stream->stream_info.sfreq = substream->runtime->rate;

	pr_debug("pcm_substream %p, period_elapsed %p\n",
			stream->stream_info.mad_substream, stream->stream_info.period_elapsed);

	ret_val = stream->ops->device_control(
					SST_SND_STREAM_INIT, &stream->stream_info);
	if (ret_val)
		pr_err("control_set ret error %d\n", ret_val);

	return ret_val;

}
/* end -- helper functions */

static int sst_media_open(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret_val = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sst_runtime_stream *stream;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;
	spin_lock_init(&stream->status_lock);

	/* get the sst ops */
	mutex_lock(&sst_dsp_lock);
	if (!sst_dsp ||
	    !try_module_get(sst_dsp->dev->driver->owner)) {
		pr_err("no device available to run\n");
		ret_val = -ENODEV;
		goto out_ops;
	}
	stream->ops = sst_dsp->ops;
	mutex_unlock(&sst_dsp_lock);

	stream->stream_info.str_id = 0;
	sst_set_stream_status(stream, SST_PLATFORM_UNINIT);
	stream->stream_info.mad_substream = substream;
	runtime->private_data = stream;

	/* Make sure, that the period size is always even */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
			   SNDRV_PCM_HW_PARAM_PERIODS, 2);

	pr_debug("buf_ptr %llx\n", stream->stream_info.buffer_ptr);

	return snd_pcm_hw_constraint_integer(runtime,
			 SNDRV_PCM_HW_PARAM_PERIODS);
out_ops:
	kfree(stream);
	mutex_unlock(&sst_dsp_lock);

	return ret_val;
}

static void sst_media_close(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;

	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	if (str_id)
		ret_val = stream->ops->close(str_id);
	module_put(sst_dsp->dev->driver->owner);
	kfree(stream);
	pr_debug("%s: %d\n", __func__, ret_val);
}

static int sst_media_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;

	pr_debug("%s\n", __func__);
	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	if (stream->stream_info.str_id)
		return ret_val;

	ret_val = sst_platform_alloc_stream(substream);
	if (ret_val <= 0)
		return ret_val;
	snprintf(substream->pcm->id, sizeof(substream->pcm->id),
			"%d", stream->stream_info.str_id);

	ret_val = sst_platform_init_stream(substream);
	if (ret_val)
		return ret_val;
	substream->runtime->hw.info = SNDRV_PCM_INFO_BLOCK_TRANSFER;

	return ret_val;
}

static int sst_media_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	pr_debug("%s\n", __func__);
	snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));

	return 0;
}

static int sst_media_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return snd_pcm_lib_free_pages(substream);
}

static struct snd_soc_dai_ops sst_media_dai_ops = {
	.startup = sst_media_open,
	.shutdown = sst_media_close,
	.prepare = sst_media_prepare,
	.hw_params = sst_media_hw_params,
	.hw_free = sst_media_hw_free,
	.set_tdm_slot = sst_platform_ihf_set_tdm_slot,
};

static struct snd_soc_dai_driver sst_platform_dai_stereo[] = {
#ifdef SST_DRV_BYT_FRONT_DAI
{
	.name = SST_BYT_FRONT_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,    
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_FRONT_DAI */
#ifdef SST_DRV_BYT_MIC1_DAI 
{
	.name = SST_BYT_MICIN1_DAI,
	.ops = &sst_media_dai_ops,
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_MIC1_DAI */

#ifdef SST_DRV_BYT_REAR_DAI
{
	.name = SST_BYT_REAR_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,    
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_REAR_DAI */
#ifdef SST_DRV_BYT_SIDE_DAI
{
	.name = SST_BYT_SIDE_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,    
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_SIDE_DAI */
#ifdef SST_DRV_BYT_CENTREWOOFER_DAI
{
	.name = SST_BYT_CENTREWOOFER_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,    
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_CENTREWOOFER_DAI */

#ifdef SST_DRV_BYT_HEADPHONE_DAI
{
	.name = SST_BYT_HEADPHONE_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
#if 1
		.rates = SNDRV_PCM_RATE_48000,    
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
#else
		/* ToDo: BYT to add more DAI sample rate later */
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
#endif
	},
},
#endif /* SST_DRV_BYT_HEADSET_DAI */
#ifdef SST_DRV_BYT_AUXOUT_DAI 
{
	.name = SST_BYT_AUXOUT_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_AUXOUT_DAI */

#ifdef SST_DRV_BYT_MIC2_DAI 
{
	.name = SST_BYT_MICIN2_DAI,
	.ops = &sst_media_dai_ops,
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_MIC2_DAI */
#ifdef SST_DRV_BYT_AUXIN_DAI 
{
	.name = SST_BYT_AUXIN_DAI,
	.ops = &sst_media_dai_ops,
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE |SNDRV_PCM_FMTBIT_S16_LE) ,
	},
},
#endif /* SST_DRV_BYT_AUXIN_DAI */
};

static struct snd_soc_dai_driver sst_platform_dai_tdm8[] = {
#ifdef SST_DRV_BYT_FRONT_DAI
{
	.name = SST_BYT_FRONT_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_TDM,
		.channels_max = SST_TDM,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S16_LE),
	},
},
#endif /* SST_DRV_BYT_FRONT_DAI */
#ifdef SST_DRV_BYT_MIC1_DAI
{
	.name = SST_BYT_MICIN1_DAI,
	.ops = &sst_media_dai_ops,
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_TDM,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	},
},
#endif /* SST_DRV_BYT_MIC1_DAI */
#ifdef SST_DRV_BYT_MIC2_DAI
{
	.name = SST_BYT_MICIN2_DAI,
	.ops = &sst_media_dai_ops,
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_TDM,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	},
},
#endif /* SST_DRV_BYT_MIC2_DAI */
};

static int sst_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("sst_platform_open called:%s\n", dai_link->cpu_dai_name);
	runtime = substream->runtime;
	switch (get_num_channels())
	{
		case 2:
			runtime->hw = sst_platform_pcm_hw_stereo;
			break;
		case 8:
			runtime->hw = sst_platform_pcm_hw_tdm8;
			break;
		default :
			pr_err("Invalid module param value, %d channels are not supported in driver\n", get_num_channels());
			return -EINVAL;
	}

	return 0;
}

static int sst_platform_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("sst_platform_close called:%s\n", dai_link->cpu_dai_name);

	return 0;
}

static int sst_platform_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	int ret_val = 0, str_id;
	struct sst_runtime_stream *stream;
	int str_cmd, status, alsa_state;

	pr_debug("sst_platform_pcm_trigger called\n");
	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	alsa_state = substream->runtime->status->state;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("Trigger Start\n");
		str_cmd = SST_SND_START;
		status = SST_PLATFORM_RUNNING;
		stream->stream_info.mad_substream = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("Trigger stop\n");
		str_cmd = SST_SND_DROP;
		status = SST_PLATFORM_DROPPED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*some mdelay in pause to avoid underrun happen*/
		mdelay(10);
		pr_debug("Trigger pause\n");
		str_cmd = SST_SND_PAUSE;
		status = SST_PLATFORM_PAUSED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("Trigger pause release\n");
		str_cmd = SST_SND_RESUME;
		status = SST_PLATFORM_RUNNING;
		break;
	default:
		return -EINVAL;
	}
	ret_val = stream->ops->device_control(str_cmd, &str_id);
	if (!ret_val)
		sst_set_stream_status(stream, status);

	return ret_val;
}


static snd_pcm_uframes_t sst_platform_pcm_pointer
			(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream;
	int ret_val, status;
	struct pcm_stream_info *str_info;

	stream = substream->runtime->private_data;
	status = sst_get_stream_status(stream);
	if (status == SST_PLATFORM_INIT)
		return 0;
	str_info = &stream->stream_info;
	ret_val = stream->ops->device_control(
				SST_SND_BUFFER_POINTER, str_info);
	if (ret_val) {
		pr_err("sst: error code = %d\n", ret_val);
		return ret_val;
	}
	substream->runtime->delay = str_info->pcm_delay;

	return str_info->buffer_ptr;
}

static struct snd_pcm_ops sst_platform_ops = {
	.open = sst_platform_open,
	.close = sst_platform_close,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = sst_platform_pcm_trigger,
	.pointer = sst_platform_pcm_pointer,
};

static void sst_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("sst_pcm_free called\n");
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static int sst_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	int retval = 0;

	pr_debug("sst_pcm_new called\n");
	if (dai->driver->playback.channels_min ||
			dai->driver->capture.channels_min) {
		switch (get_num_channels())
		{
			case 2:
				retval = snd_pcm_lib_preallocate_pages_for_all(pcm,
					SNDRV_DMA_TYPE_DEV,
					NULL,
					SST_MAX_BUFFER_STEREO, SST_MAX_BUFFER_STEREO);
				break;
			case 8:
				retval = snd_pcm_lib_preallocate_pages_for_all(pcm,
					SNDRV_DMA_TYPE_DEV,
					NULL,
					SST_MAX_BUFFER_TDM, SST_MAX_BUFFER_TDM);
				break;
			default :
				retval = -EINVAL;
				pr_err("Invalid module param value, %d channels are not supported in driver\n", get_num_channels());
		}
		if (retval) {
			pr_err("dma buffer allocationf fail\n");
			return retval;
		}
	}

	return retval;
}

/* compress stream operations */
#ifndef SST_DRV_BYT
static void sst_compr_fragment_elapsed(void *arg)
{
	struct snd_compr_stream *cstream = (struct snd_compr_stream *)arg;

	pr_debug("fragment elapsed by driver\n");
	if (cstream)
		snd_compr_fragment_elapsed(cstream);
}
#endif /* SST_DRV_BYT */

static int sst_platform_compr_open(struct snd_compr_stream *cstream)
{

	int ret_val = 0;
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct sst_runtime_stream *stream;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	spin_lock_init(&stream->status_lock);

	/* get the sst ops */
	if (!sst || !try_module_get(sst->dev->driver->owner)) {
		pr_err("no device available to run\n");
		ret_val = -ENODEV;
		goto out_ops;
	}
	stream->compr_ops = sst->compr_ops;

	stream->id = 0;
	sst_set_stream_status(stream, SST_PLATFORM_INIT);
	runtime->private_data = stream;
	return 0;
out_ops:
	kfree(stream);
	return ret_val;
}

static int sst_platform_compr_free(struct snd_compr_stream *cstream)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;

	stream = cstream->runtime->private_data;
	/*need to check*/
	str_id = stream->id;
	if (str_id)
		ret_val = stream->compr_ops->close(str_id);
	module_put(sst->dev->driver->owner);
	kfree(stream);
	pr_debug("%s: %d\n", __func__, ret_val);
	return 0;
}

static int sst_platform_compr_set_params(struct snd_compr_stream *cstream,
					struct snd_compr_params *params)
{
#ifndef SST_DRV_BYT
	struct sst_runtime_stream *stream;
	int retval;
	struct snd_sst_params str_params;
	struct sst_compress_cb cb;

	stream = cstream->runtime->private_data;
	/* construct fw structure for this*/
	memset(&str_params, 0, sizeof(str_params));

	str_params.ops = STREAM_OPS_PLAYBACK;
	str_params.stream_type = SST_STREAM_TYPE_MUSIC;
	str_params.device_type = SND_SST_DEVICE_COMPRESS;

	switch (params->codec.id) {
	case SND_AUDIOCODEC_MP3: {
		str_params.codec = SST_CODEC_TYPE_MP3;
		str_params.sparams.uc.mp3_params.codec = SST_CODEC_TYPE_MP3;
		str_params.sparams.uc.mp3_params.num_chan = params->codec.ch_in;
		str_params.sparams.uc.mp3_params.pcm_wd_sz = 16;
		break;
	}

	case SND_AUDIOCODEC_AAC: {
		str_params.codec = SST_CODEC_TYPE_AAC;
		str_params.sparams.uc.aac_params.codec = SST_CODEC_TYPE_AAC;
		str_params.sparams.uc.aac_params.num_chan = params->codec.ch_in;
		str_params.sparams.uc.aac_params.pcm_wd_sz = 16;
		if (params->codec.format == SND_AUDIOSTREAMFORMAT_MP4ADTS)
			str_params.sparams.uc.aac_params.bs_format =
							AAC_BIT_STREAM_ADTS;
		else if (params->codec.format == SND_AUDIOSTREAMFORMAT_RAW)
			str_params.sparams.uc.aac_params.bs_format =
							AAC_BIT_STREAM_RAW;
		else {
			pr_err("Undefined format%d\n", params->codec.format);
			return -EINVAL;
		}
		str_params.sparams.uc.aac_params.externalsr =
						params->codec.sample_rate;
		break;
	}

	default:
		pr_err("codec not supported, id =%d\n", params->codec.id);
		return -EINVAL;
	}

	str_params.aparams.ring_buf_info[0].addr  =
					virt_to_phys(cstream->runtime->buffer);
	str_params.aparams.ring_buf_info[0].size =
					cstream->runtime->buffer_size;
	str_params.aparams.sg_count = 1;
	str_params.aparams.frag_size = cstream->runtime->fragment_size;

	cb.param = cstream;
	cb.compr_cb = sst_compr_fragment_elapsed;

	retval = stream->compr_ops->open(&str_params, &cb);
	if (retval < 0) {
		pr_err("stream allocation failed %d\n", retval);
		return retval;
	}

	stream->id = retval;
#endif /* SST_DRV_BYT */
	return 0;
}

static int sst_platform_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	struct sst_runtime_stream *stream =
		cstream->runtime->private_data;

	return stream->compr_ops->control(cmd, stream->id);
}

static int sst_platform_compr_pointer(struct snd_compr_stream *cstream,
					struct snd_compr_tstamp *tstamp)
{
	struct sst_runtime_stream *stream;

	stream  = cstream->runtime->private_data;
	stream->compr_ops->tstamp(stream->id, tstamp);
	tstamp->byte_offset = tstamp->copied_total %
				 (u32)cstream->runtime->buffer_size;
	pr_debug("calc bytes offset/copied bytes as %d\n", tstamp->byte_offset);
	return 0;
}

static int sst_platform_compr_ack(struct snd_compr_stream *cstream,
					size_t bytes)
{
	struct sst_runtime_stream *stream;

	stream  = cstream->runtime->private_data;
	stream->compr_ops->ack(stream->id, (unsigned long)bytes);
	stream->bytes_written += bytes;

	return 0;
}

static int sst_platform_compr_get_caps(struct snd_compr_stream *cstream,
					struct snd_compr_caps *caps)
{
	struct sst_runtime_stream *stream =
		cstream->runtime->private_data;

	return stream->compr_ops->get_caps(caps);
}

static int sst_platform_compr_get_codec_caps(struct snd_compr_stream *cstream,
					struct snd_compr_codec_caps *codec)
{
	struct sst_runtime_stream *stream =
		cstream->runtime->private_data;

	return stream->compr_ops->get_codec_caps(codec);
}

static int sst_platform_compr_set_metadata(struct snd_compr_stream *cstream,
					struct snd_compr_metadata *metadata)
{
	struct sst_runtime_stream *stream  =
		 cstream->runtime->private_data;

	return stream->compr_ops->set_metadata(stream->id, metadata);
}

static struct snd_compr_ops sst_platform_compr_ops = {

	.open = sst_platform_compr_open,
	.free = sst_platform_compr_free,
	.set_params = sst_platform_compr_set_params,
	.set_metadata = sst_platform_compr_set_metadata,
	.trigger = sst_platform_compr_trigger,
	.pointer = sst_platform_compr_pointer,
	.ack = sst_platform_compr_ack,
	.get_caps = sst_platform_compr_get_caps,
	.get_codec_caps = sst_platform_compr_get_codec_caps,
};

static struct snd_soc_platform_driver sst_soc_platform_drv = {
	.ops		= &sst_platform_ops,
	.compr_ops	= &sst_platform_compr_ops,
	.pcm_new	= sst_pcm_new,
	.pcm_free	= sst_pcm_free,
};

int sst_register_dsp(struct sst_device *dev)
{
	if (!dev)
		return -ENODEV;
	mutex_lock(&sst_dsp_lock);
	if (sst_dsp) {
		pr_err("we already have a device %s\n", sst_dsp->name);
		mutex_unlock(&sst_dsp_lock);
		return -EEXIST;
	}
	pr_debug("registering device %s\n", dev->name);
	sst_dsp = dev;
	mutex_unlock(&sst_dsp_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(sst_register_dsp);

int sst_unregister_dsp(struct sst_device *dev)
{
	if (dev != sst_dsp)
		return -EINVAL;

	mutex_lock(&sst_dsp_lock);
	if (sst_dsp) {
		mutex_unlock(&sst_dsp_lock);
		return -EIO;
       }

	pr_debug("unregister %s\n", sst_dsp->name);
	sst_dsp = NULL;
	mutex_unlock(&sst_dsp_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(sst_unregister_dsp);

static int sst_platform_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug("sst_platform_probe called\n");
	ret = snd_soc_register_platform(&pdev->dev, &sst_soc_platform_drv);
	if (ret) {
		pr_err("registering soc platform failed\n");
		return ret;
	}

	pr_info("sst-platform module param useMultiChannels is set to %d \n", get_num_channels());
	switch (get_num_channels())
	{
		case 2:
			ret = snd_soc_register_component(&pdev->dev, &sst_component,
				sst_platform_dai_stereo, ARRAY_SIZE(sst_platform_dai_stereo));
			break;
		case 8:
			ret = snd_soc_register_component(&pdev->dev, &sst_component,
				sst_platform_dai_tdm8, ARRAY_SIZE(sst_platform_dai_tdm8));
			break;
		default :
			ret = -EINVAL;
			pr_err("Invalid module param value, %d channels are not supported in driver\n", get_num_channels());
	}
	if (ret) {
		pr_err("registering cpu dais failed\n");
		snd_soc_unregister_platform(&pdev->dev);
	}
	return ret;
}

static int sst_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);
	pr_debug("sst_platform_remove success\n");
	return 0;
}

static struct platform_driver sst_platform_driver = {
	.driver		= {
		.name		= "sst-platform",
		.owner		= THIS_MODULE,
	},
	.probe		= sst_platform_probe,
	.remove		= sst_platform_remove,
};

static int __init sst_soc_platform_init(void)
{
	pr_debug("sst_soc_platform_init called\n");
	return  platform_driver_register(&sst_platform_driver);
}
module_init(sst_soc_platform_init);

static void __exit sst_soc_platform_exit(void)
{
	platform_driver_unregister(&sst_platform_driver);
	pr_debug("sst_soc_platform_exit success\n");
}
module_exit(sst_soc_platform_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Platform driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sst-platform");
