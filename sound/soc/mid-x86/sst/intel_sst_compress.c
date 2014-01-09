/*
 *  intel_sst_compress.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-11	Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_dsp.h"
#include <sound/compress_driver.h>
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"
#include "../sst_platform.h"

#define NUM_CODEC 2
#define MIN_FRAGMENT 2
#define MAX_FRAGMENT 4
#define MIN_FRAGMENT_SIZE (50 * 1024)
#define MAX_FRAGMENT_SIZE (1024 * 1024)
#define SST_GET_BYTES_PER_SAMPLE(pcm_wd_sz)  (((pcm_wd_sz + 15) >> 4) << 1)


#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_open(struct snd_sst_params *str_params,
		struct sst_compress_cb *cb)
{
	int str_id, retval;
	struct stream_info *stream;

	retval = intel_sst_check_device();
	if (retval)
		return retval;

	str_id = sst_get_stream(str_params);
	if (str_id > 0) {
		pr_debug("stream allocated in sst_cdev_open %d\n", str_id);
		stream = &sst_drv_ctx->streams[str_id];
		stream->compr_cb = cb->compr_cb;
		stream->compr_cb_param = cb->param;
	} else {
		pr_err("stream encountered error during alloc %d\n", str_id);
		str_id = -EINVAL;
		pm_runtime_put(&sst_drv_ctx->pci->dev);
	}

	return str_id;
}
#endif /* CONFIG_SND_COMPRESS_SST */

#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_close(unsigned int str_id)
{
	int retval;
	struct stream_info *stream;

	stream = get_stream_info(str_id);
	if (!stream)
		return -EINVAL;
	retval = sst_free_stream(str_id);
	pm_runtime_put(&sst_drv_ctx->pci->dev);

	return retval;
}
#endif /* CONFIG_SND_COMPRESS_SST */

#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_ack(unsigned int str_id, unsigned long bytes)
{
	struct stream_info *stream;

	struct snd_sst_tstamp fw_tstamp = {0,};  //  For BYT, we can share the same name with other SoC

	int offset;
	void __iomem *addr;

	pr_debug("sst:  ackfor %d\n", str_id);
	stream = get_stream_info(str_id);
	if (!stream)
		return -EINVAL;

	/* update bytes sent */
	stream->cumm_bytes += bytes;
	pr_debug("bytes copied %d inc by %ld\n", stream->cumm_bytes, bytes);

	memcpy_fromio(&fw_tstamp,
		((void *)(sst_drv_ctx->mailbox + sst_drv_ctx->tstamp)
		+(str_id * sizeof(fw_tstamp))),
		sizeof(fw_tstamp));

	fw_tstamp.bytes_copied = stream->cumm_bytes;
	pr_debug("bytes sent to fw %llx inc by %ld\n", fw_tstamp.bytes_copied,
							 bytes);

	addr =  ((void *)(sst_drv_ctx->mailbox + sst_drv_ctx->tstamp)) +
			(str_id * sizeof(fw_tstamp));

	offset =  offsetof(struct snd_sst_tstamp, bytes_copied);

	if ( (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID) || 
	     (sst_drv_ctx->pci_id == SST_BYT_PCI_ID) )   // ------------------ [OK] BYT is 64-bit too
		sst_shim_write64(addr, offset, fw_tstamp.bytes_copied);
	else
		sst_shim_write(addr, offset, fw_tstamp.bytes_copied);

	return 0;

}
#endif /* CONFIG_SND_COMPRESS_SST */

#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_control(unsigned int cmd, unsigned int str_id)
{
	pr_debug("recieved cmd %d on stream %d\n", cmd, str_id);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return sst_pause_stream(str_id);
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		return sst_resume_stream(str_id);
	case SNDRV_PCM_TRIGGER_START: {
		struct stream_info *str_info;
		str_info = get_stream_info(str_id);
		if (!str_info)
			return -EINVAL;
		str_info->prev = str_info->status;
		str_info->status = STREAM_RUNNING;
		return sst_start_stream(str_id);
	}
	case SNDRV_PCM_TRIGGER_STOP:
		return sst_drop_stream(str_id);
	case SND_COMPR_TRIGGER_DRAIN:
		return sst_drain_stream(str_id);
	}

	return -EINVAL;
}
#endif /* CONFIG_SND_COMPRESS_SST */

#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_tstamp(unsigned int str_id, struct snd_compr_tstamp *tstamp)
{
	struct snd_sst_tstamp fw_tstamp = {0,};    // For BYT, it can share with other SoC
	struct stream_info *stream;

	memcpy_fromio(&fw_tstamp,
		((void *)(sst_drv_ctx->mailbox + SST_TIME_STAMP)
		+(str_id * sizeof(fw_tstamp))),
		sizeof(fw_tstamp));

	stream = get_stream_info(str_id);
	if (!stream)
		return -EINVAL;
	pr_debug("rb_counter %llx in bytes\n", fw_tstamp.ring_buffer_counter);

	tstamp->copied_total = fw_tstamp.ring_buffer_counter;
	tstamp->pcm_frames = fw_tstamp.frames_decoded;
	tstamp->pcm_io_frames = fw_tstamp.hardware_counter /
			((stream->num_ch) * SST_GET_BYTES_PER_SAMPLE(24));
	tstamp->sampling_rate = fw_tstamp.sampling_frequency;
	pr_debug("PCM  = %lu\n", tstamp->pcm_io_frames);
	pr_debug("Pointer Query on strid = %u  copied_total %u, decodec %lu\n",
		str_id, tstamp->copied_total, tstamp->pcm_frames);
	pr_debug("rendered %lu\n", tstamp->pcm_io_frames);

	return 0;
}
#endif /* CONFIG_SND_COMPRESS_SST */


#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_caps(struct snd_compr_caps *caps)
{
	caps->num_codecs = NUM_CODEC;
	caps->min_fragment_size = MIN_FRAGMENT_SIZE;  /* 50KB */
	caps->max_fragment_size = MAX_FRAGMENT_SIZE;  /* 1024KB */
	caps->min_fragments = MIN_FRAGMENT;
	caps->max_fragments = MAX_FRAGMENT;
	caps->codecs[0] = SND_AUDIOCODEC_MP3;
	caps->codecs[1] = SND_AUDIOCODEC_AAC;

	return 0;
}
#endif /* CONFIG_SND_COMPRESS_SST */

#ifdef CONFIG_SND_COMPRESS_SST
static int sst_cdev_codec_caps(struct snd_compr_codec_caps *codec)
{
	if (codec->codec == SND_AUDIOCODEC_MP3) {
		codec->num_descriptors = 2;
		codec->descriptor[0].max_ch = 2;
		codec->descriptor[0].sample_rates = SNDRV_PCM_RATE_8000_48000;
		codec->descriptor[0].bit_rate[0] = 320; /* 320kbps */
		codec->descriptor[0].bit_rate[1] = 192;
		codec->descriptor[0].num_bitrates = 2;
		codec->descriptor[0].profiles = 0;
		codec->descriptor[0].modes = SND_AUDIOCHANMODE_MP3_STEREO;
		codec->descriptor[0].formats = 0;
	} else if (codec->codec == SND_AUDIOCODEC_AAC) {
		codec->num_descriptors = 2;
		codec->descriptor[1].max_ch = 2;
		codec->descriptor[1].sample_rates = SNDRV_PCM_RATE_8000_48000;
		codec->descriptor[1].bit_rate[0] = 320; /* 320kbps */
		codec->descriptor[1].bit_rate[1] = 192;
		codec->descriptor[1].num_bitrates = 2;
		codec->descriptor[1].profiles = 0;
		codec->descriptor[1].modes = 0;
		codec->descriptor[1].formats =
			(SND_AUDIOSTREAMFORMAT_MP4ADTS |
				SND_AUDIOSTREAMFORMAT_RAW);
	} else {
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_SND_COMPRESS_SST */

#ifdef CONFIG_SND_COMPRESS_SST
static struct compress_sst_ops compress = {
	.name = "intel_sst",
	.open = sst_cdev_open,
	.close = sst_cdev_close,
	.control = sst_cdev_control,
	.tstamp = sst_cdev_tstamp,
	.ack = sst_cdev_ack,
	.get_caps = sst_cdev_caps,
	.get_codec_caps = sst_cdev_codec_caps,
};
#endif /* CONFIG_SND_COMPRESS_SST */

void sst_cdev_fragment_elapsed(int str_id)
{
	struct stream_info *stream;

	pr_debug("fragment elapsed from firmware for str_id %d\n", str_id);
	stream = &sst_drv_ctx->streams[str_id];
	if (stream->compr_cb)
		stream->compr_cb(stream->compr_cb_param);
}

#ifdef CONFIG_SND_COMPRESS_SST
int intel_sst_register_compress(struct intel_sst_drv *sst)
{
	int ret;

	pr_debug("register for compressed device\n");
	ret = compress_sst_register_dev(&sst->pci->dev, &compress);
	if (ret)
		pr_err("failed to register for compressed device %d\n", ret);

	return ret;
}

int intel_sst_remove_compress(struct intel_sst_drv *sst)
{
	compress_sst_deregister_dev(&sst->pci->dev);

	return 0;
}

#else
int intel_sst_register_compress(struct intel_sst_drv *sst)
{
	return 0;
}

int intel_sst_remove_compress(struct intel_sst_drv *sst)
{
	return 0;
}
#endif
