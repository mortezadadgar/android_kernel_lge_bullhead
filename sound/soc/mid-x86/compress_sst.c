/*
 *  sst_compress.c - SST Compress framework driver
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Vinod Koul <vinod.koul@linux.intel.com>
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
#define FORMAT(fmt) "%s %d: " fmt, __func__, __LINE__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#include <linux/file.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/uio.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>
#include <sound/intel_sst_ioctl.h>
#include "sst_platform.h"
#include <sound/compress_sst.h>

/* FIXME: add DAPM codec output
 * stream registeration mechnaism */
static struct compress_sst_ops *compress;
static DEFINE_MUTEX(lock);

struct sst_stream {
	struct sst_device *sst;
	struct compress_sst_ops *ops;
	unsigned int id;
	size_t bytes_written;
};

static void sst_compr_fragment_elapsed(void *arg)
{
	struct snd_compr_stream *cstream = (struct snd_compr_stream *)arg;

	pr_debug("fragment elapsed by driver\n");
	if (cstream)
		snd_compr_fragment_elapsed(cstream);
}

int compress_sst_register_dev(struct device *dev,
		struct compress_sst_ops *devops)
{
	BUG_ON(!devops);

	if (!try_module_get(dev->driver->owner))
		return -ENODEV;

	mutex_lock(&lock);
	if (compress) {
		pr_err("already registered %s, got new %s\n",
			compress->name, devops->name);
		module_put(dev->driver->owner);
		mutex_unlock(&lock);
		return -EAGAIN;
	}
	pr_debug("registering %s\n", devops->name);
	compress = devops;
	mutex_unlock(&lock);

	return 0;
}
EXPORT_SYMBOL_GPL(compress_sst_register_dev);

int compress_sst_deregister_dev(struct device *dev)
{
	BUG_ON(!dev);

	mutex_lock(&lock);
	if (!compress) {
		mutex_unlock(&lock);
		return -EIO;
	}
	pr_debug("dereg %s\n", compress->name);
	module_put(dev->driver->owner);
	compress = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(compress_sst_deregister_dev);

static int sst_compr_open(struct snd_compr_stream *cstream)
{
	struct sst_device *sst = (struct sst_device *)cstream->private_data;
	struct sst_stream *stream;

	pr_debug("entry\n");
	mutex_lock(&lock);
	if (!compress) {
		mutex_unlock(&lock);
		return -ENODEV;
	}
	mutex_unlock(&lock);

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;
	stream->sst = sst;
	stream->ops = compress;
	cstream->private_data = stream;
	sst->usage_count++;

	return 0;
}

static int sst_compr_free(struct snd_compr_stream *cstream)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;

	pr_debug("entry\n");
	stream->ops->close(stream->id);
	stream->sst->usage_count--;
	kfree(stream);

	return 0;
}

static int sst_compr_set_params(struct snd_compr_stream *cstream,
		struct snd_compr_params *params)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;
	int retval;
	struct snd_sst_params str_params;
	struct sst_compress_cb cb;

	pr_debug("entry\n");
	/* construct fw structure for this*/
	memset(&str_params, 0, sizeof(str_params));

	str_params.ops = STREAM_OPS_PLAYBACK;
	str_params.stream_type = SST_STREAM_TYPE_MUSIC;
	str_params.device_type = SND_SST_DEVICE_COMPRESS;
	pr_debug("Device Type %d\n", str_params.device_type);

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

	retval = stream->ops->open(&str_params, &cb);
	if (retval < 0) {
		pr_err("stream allocation failed %d\n", retval);
		return retval;
	}
	stream->id = retval;

	return 0;
}

static int sst_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;

	pr_debug("sending trigger cmd %d\n", cmd);

	return stream->ops->control(cmd, stream->id);
}

static int sst_compr_pointer(struct snd_compr_stream *cstream,
		struct snd_compr_tstamp *tstamp)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;
	stream->ops->tstamp(stream->id, tstamp);
	tstamp->byte_offset = tstamp->copied_total %
				 (u32)cstream->runtime->buffer_size;
	pr_debug("calc bytes offset/copied bytes as %d\n", tstamp->byte_offset);

	return 0;
}

static int sst_compr_ack(struct snd_compr_stream *cstream, size_t bytes)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;
	pr_debug("bytes written %d\n", bytes);
	stream->ops->ack(stream->id, (unsigned long)bytes);
	stream->bytes_written += bytes;

	return 0;
}

static int sst_compr_get_caps(struct snd_compr_stream *cstream,
		struct snd_compr_caps *caps)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;

	return stream->ops->get_caps(caps);
}

static int sst_compr_get_codec_caps(struct snd_compr_stream *cstream,
		struct snd_compr_codec_caps *codec)
{
	struct sst_stream *stream = (struct sst_stream *)cstream->private_data;

	return stream->ops->get_codec_caps(codec);
}

static struct snd_compr_ops sst_compr_ops = {
	.open = sst_compr_open,
	.free = sst_compr_free,
	.set_params = sst_compr_set_params,
	.trigger = sst_compr_trigger,
	.pointer = sst_compr_pointer,
	.ack = sst_compr_ack,
	.get_caps = sst_compr_get_caps,
	.get_codec_caps = sst_compr_get_codec_caps,
};

static struct snd_compr sst_compr = {
	.name = "intel_sst",
	.ops = &sst_compr_ops,
};

static int __devinit intel_sst_compr_probe(struct platform_device *pdev)
{
	int ret;
	struct sst_device *sst;
	struct snd_card *card;

	pr_debug("sst_compress_probe called\n");

	compress = NULL;

	sst = kzalloc(sizeof(*sst), GFP_KERNEL);
	if (!sst) {
		pr_err("malloc fail\n");
		return -ENOMEM;
	}

	/*Register compressed ops */
	sst_compr.dev = &pdev->dev;
	ret = snd_card_create(SNDRV_DEFAULT_IDX1, sst_compr.name,
					THIS_MODULE, 0, &card);
	if (ret) {
		pr_err("failed to create the sound card\n");
		goto unreg_sst;
	}
	sst_compr.card = card;

	ret = snd_compress_new(card, 0,
			SND_COMPRESS_PLAYBACK,
			&sst_compr);
	if (ret)
		goto out;
	ret = snd_compress_register(&sst_compr);
	if (ret) {
		pr_err("couldn't register compr device\n");
		goto unreg_sst;
	}
	sst_compr.private_data = (void *)sst;
	sst->pdev = pdev;
	platform_set_drvdata(pdev, sst);

	goto out;
unreg_sst:
	kfree(sst);
out:
	return ret;
}

static int __devexit intel_sst_compr_remove(struct platform_device *pdev)
{
	struct sst_device *sst = platform_get_drvdata(pdev);

	if (compress || sst->usage_count)
		return -EBUSY;
	snd_compress_deregister(&sst_compr);
	platform_set_drvdata(pdev, NULL);
	kfree(sst);

	return 0;
}

static struct platform_driver sst_compress_driver = {
	.driver		= {
		.name		= "compress-sst",
		.owner		= THIS_MODULE,
	},
	.probe		= intel_sst_compr_probe,
	.remove		= intel_sst_compr_remove,
};
static int __init intel_sst_init(void)
{
	pr_debug("compress-sst init\n");
	return  platform_driver_register(&sst_compress_driver);
}

static void __exit intel_sst_exit(void)
{
	pr_debug("compress-sst exit\n");
	platform_driver_unregister(&sst_compress_driver);
}
module_init(intel_sst_init);
module_exit(intel_sst_exit);

MODULE_DESCRIPTION("Intel(R) MID Smart Sound Technology driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sst-compress");
