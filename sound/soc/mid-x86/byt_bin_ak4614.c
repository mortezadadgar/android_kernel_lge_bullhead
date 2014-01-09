/*
 *  byt_bin_ak4614.c - ASoc Machine driver for Intel Baytrail platform (binary)
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author:
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/intel_sst_ioctl.h>

#include "sst_platform_pvt.h"
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
#include "../codecs/ak4614.h"

/* For MCLK it is driver by on-board X'tal at MCKI
 * This is the master clock used for sampling at ADC & DAC
 */
#define AK4614_DEFAULT_MCLK	48000
#endif /* CONFIG_SND_SOC_DUMMY_CODEC */

struct byt_mc_private {
	struct platform_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
};

static int byt_lpe_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	unsigned int fmt;
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;
#endif

	/* AK4614 Code DAI modes*/
	fmt =   SND_SOC_DAIFMT_NB_NF |	 /* Normal bit block & frame */
		SND_SOC_DAIFMT_CBM_CFM ; /* Codec clk & FRM master */ 

	switch (get_num_channels())
	{
		case 2:
			fmt |= SND_SOC_DAIFMT_I2S; /* AK4616 is set to I2S in AK4614_03_CONTROL1 */
			break;
		case 8:
			fmt |= SND_SOC_DAIFMT_TDM256_LEFT_J; /* AK4616 is set to mode 23 in AK4614_03_CONTROL1 */
			break;
		default :
			pr_err("Invalid module param value, %d channels are not supported in driver\n", get_num_channels());
			return -EINVAL;
	}

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
	/* Set format in codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("%s : Set codec FMT failed (%d)\n", __func__, ret);
		return ret;
	}
#endif

	/* BYT:  	Users to uncomment if codec code needs to set clk frequency
	 * 			Commented out for AK4614 AIC as the MCLK
	 * 			is driven by on-board X'tal at MCKI
	 */
	/* Set clock configuration in codec DAI configuration */
/*	ret = snd_soc_dai_set_sysclk(codec_dai, AK4614_CLKID_MCLK,
			AK4614_DEFAULT_MCLK, SND_SOC_CLOCK_IN);

	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
*/
	return 0;

}/*byt_lpe_hw_params*/

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret = 0;
	return ret;
}/*byt_init*/


static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count	= ARRAY_SIZE(rates_48000),
	.list	= rates_48000,
};

static int byt_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_48000);
} /*byt_startup*/

/*SOC ops for FW driven path*/
static struct snd_soc_ops byt_asp_ops = {
	.startup = byt_startup,
	.hw_params = byt_lpe_hw_params,
};

static struct snd_soc_dai_link byt_msic_dailink[] = {
	{
		.name = "Baytrail Front DAI",
		.stream_name = "Audio Front",
		.cpu_dai_name = SST_BYT_FRONT_DAI,
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		.codec_dai_name = SOC_DAI_DRIVER_AK4614,
		/* Codec_name is created when I2C client is registered through
		 * arch/x86/platform/intel-mid/device_libs/platform_byt_audio.c
		 * "1-0010" can be obtained from /sys/bus/i2c/devices node
		 */
		.codec_name = "ak4614.1-0010",
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
		.platform_name = "sst-platform",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_asp_ops,
	},
	{
		.name = "Baytrail Mic-1 DAI",
		.stream_name = "Voice Mic-1",
		.cpu_dai_name = SST_BYT_MICIN1_DAI,
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		.codec_dai_name = SOC_DAI_DRIVER_AK4614,
		.codec_name = "ak4614.1-0010", /*Board-info on i2c bus-device*/
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
		.platform_name = "sst-platform",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_asp_ops,
	},
};

static int snd_byt_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static int snd_byt_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_byt_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_byt = {
	.name = "Baytrail_audio",
	.dai_link = byt_msic_dailink,
	.num_links = ARRAY_SIZE(byt_msic_dailink),
};

//forward declaration
static void byt_async_register_card(void *data, async_cookie_t cookie);

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;

	pr_debug("In %s\n", __func__);
	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	drv->socdev = pdev;
	snd_soc_card_byt.dev = &pdev->dev;
	/* register the soc card */
	async_schedule(byt_async_register_card,&snd_soc_card_byt);
	platform_set_drvdata(pdev, &snd_soc_card_byt);
	snd_soc_card_set_drvdata(&snd_soc_card_byt, drv);

	pr_info("successfully exited probe\n");

	return ret_val;
}

static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	kfree(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void byt_async_register_card(void *data, async_cookie_t cookie)
{
	int ret_val = 0;
	struct snd_soc_card *soc_card = data;
	
	/* To preserve the sound card order */
	async_synchronize_cookie(cookie);
	
	ret_val = snd_soc_register_card(soc_card);
	if (ret_val)
	{
	  //sound card registration failed, perform necessary cleanup
	  struct byt_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	  snd_byt_mc_remove(ctx->socdev);
	}
  
}

const struct dev_pm_ops snd_byt_mc_pm_ops = {
	.suspend = snd_byt_suspend,
	.resume = snd_byt_resume,
	.poweroff = snd_byt_poweroff,
};

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_bin_ak4614",
		.pm   = &snd_byt_mc_pm_ops,
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
};

static int __init snd_byt_driver_init(void)
{
	pr_info("BYT: Baytrail Machine Driver byt_bin_ak4614 is registering.\n");
	return platform_driver_register(&snd_byt_mc_driver);
}
late_initcall(snd_byt_driver_init);

static void __exit snd_byt_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_byt_mc_driver);
}

module_exit(snd_byt_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver - FW");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:byt-bin-ak4614");
