/*
 *  byt_ia_ak4614.c - ASoc Machine driver for Intel Baytrail platform (IA)
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
#include <linux/intel_mid_dma.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/intel_sst_ioctl.h>
#include <asm/platform_byt_audio.h>

#include "sst_platform_pvt.h"
#include "mid_ssp.h"
#include "byt_machine.h"

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
#include "../codecs/ak4614.h"
/* For MCLK it is driver by on-board X'tal at MCKI
 * This is the master clock used for sampling at ADC & DAC
 */
#define AK4614_DEFAULT_MCLK	48000
#endif /* CONFIG_SND_SOC_DUMMY_CODEC */

struct byt_mc_private {
	struct ssp_platform_config *pdata;
	struct platform_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
};

/* Notes on hw_param callback:
 * Each sound card may have its own unique function if the settings are unique.
 * The following need to be determined according to:
 * 1) Formats:
 * 	Refer soc_dai.h for ASoC supported flags
 * 	Current BYT supported PCM format
 *  	-SND_SOC_DAIFMT_I2S
 *  	-SND_SOC_DAIFMT_TDM256_LEFT_J [new]
 *
 * 	BYT-AK4614 Clock schemes:
 *     -TDM8 uses Normal Bit Clk, Normal Frame Clk
 *             SND_SOC_DAIFMT_NB_NF
 *     -I2S uses inverted Frame Clk
 *             SND_SOC_DAIFMT_NB_IF
 *
 *     -Both Bit and Frame Clk shall be from same source.
 *      i.e. both from codec (CBM, CFM); or
 *               both from SSP (CBS, CFS)
 *
 * 2) TDM slot parameters:
 * 	 Refer byt_machine.h for defined macro
 * 		nb_slot = <desired slot size>; (2 vs 8)
 * 		slot_width =  <desired slot size>; (32bits)
 * 		tx_mask = <format slot mask>;
 * 		rx_mask = <format slot mask>;
 *
 * 3) Tristate settings:
 * 	-  Tristate offset should be careful set depending on desired behaviour
 * 		e.g. tristate = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
 */
static int byt_ssp0_ia_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
#endif

	int ret = 0;
	unsigned int fmt;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;

	fmt =  SND_SOC_DAIFMT_TDM256_LEFT_J |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM; //Codec Master, SSP Slave

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		/*Set format for codec DAI*/
		ret = snd_soc_dai_set_fmt(codec_dai, fmt);

		if (ret < 0) {
			pr_err("%s : Set codec FMT Failed (%d)\n",__func__, ret);
			return -EINVAL;
		}
#endif

		fmt |= SSP_DAI_SCMODE_0;

		/*Set format for CPU DAI*/
		ret = snd_soc_dai_set_fmt(cpu_dai, fmt);

		if (ret < 0) {
			pr_err("%s : Set cpu FMT Failed (%d)\n",__func__, ret);
			return -EINVAL;
		}

		/*PCM slot params*/
		nb_slot = SSP_TDM8_SLOT_NB_SLOT;
		slot_width =  SSP_S32_SLOT_WIDTH;
		tx_mask = SSP_TDM8_SLOT_TX_MASK;
		rx_mask = SSP_TDM8_SLOT_RX_MASK;

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Machine:  Set TDM Slot Fails %d\n",
				ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Machine: Set Tristate Fails %d\n",
				ret);
		return -EINVAL;
	}

	pr_debug("BYT Machine: slot_width = %d\n",
			slot_width);
	pr_debug("BYT Machine: tx_mask = 0X%08x\n",
			tx_mask);
	pr_debug("BYT Machine: rx_mask = 0X%08x\n",
			rx_mask);
	pr_debug("BYT Machine: tristate_offset = 0X%08x\n",
			tristate_offset);

	return 0;

} /* byt_ssp0_ia_dai_link_hw_params*/

static int byt_ssp1_ia_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
#endif

	int ret = 0;
	unsigned int fmt;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;

	fmt =   SND_SOC_DAIFMT_TDM256_LEFT_J |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS; //Codec Slave, SSP Master

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		/*Set format for codec DAI*/
		ret = snd_soc_dai_set_fmt(codec_dai, fmt);

		if (ret < 0) {
			pr_err("%s : Set codec FMT Failed (%d)\n",__func__, ret);
			return -EINVAL;
		}
#endif

		fmt |= SSP_DAI_SCMODE_0;

		/*Set format for CPU DAI*/
		ret = snd_soc_dai_set_fmt(cpu_dai, fmt);

		if (ret < 0) {
			pr_err("%s : Set cpu FMT Failed (%d)\n",__func__, ret);
			return -EINVAL;
		}

		/*PCM slot params*/
		nb_slot = SSP_TDM8_SLOT_NB_SLOT;
		slot_width =  SSP_S32_SLOT_WIDTH;
		tx_mask = SSP_TDM8_SLOT_TX_MASK;
		rx_mask = SSP_TDM8_SLOT_RX_MASK;


		ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Machine:  Set TDM Slot Fails %d\n",
				ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Machine: Set Tristate Fails %d\n",
				ret);
		return -EINVAL;
	}

	pr_debug("BYT Machine: slot_width = %d\n",
			slot_width);
	pr_debug("BYT Machine: tx_mask = 0X%08x\n",
			tx_mask);
	pr_debug("BYT Machine: rx_mask = 0X%08x\n",
			rx_mask);
	pr_debug("BYT Machine: tristate_offset = 0X%08x\n",
			tristate_offset);

	return 0;

} /* byt_ssp1_ia_dai_link_hw_params */

static int byt_ssp2_ia_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
#endif

	int ret = 0;
	unsigned int fmt;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;

	fmt =   SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS; //Codec Slave, SSP Master

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		/*Set format for codec DAI*/
		ret = snd_soc_dai_set_fmt(codec_dai, fmt);

		if (ret < 0) {
			pr_err("%s : Set codec FMT Failed (%d)\n",__func__, ret);
			return -EINVAL;
		}
#endif

		fmt |= SSP_DAI_SCMODE_0;

		/*Set format for CPU DAI*/
		ret = snd_soc_dai_set_fmt(cpu_dai, fmt);

		if (ret < 0) {
			pr_err("%s : Set cpu FMT Failed (%d)\n",__func__, ret);
			return -EINVAL;
		}

		/*PCM slot params*/
		nb_slot = SSP_STEREO_SLOT_NB_SLOT;
		slot_width =  SSP_S24_SLOT_WIDTH;
		tx_mask = SSP_STEREO_SLOT_TX_MASK;
		rx_mask = SSP_STEREO_SLOT_RX_MASK;


		ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Machine:  Set TDM Slot Fails %d\n",
				ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Machine: Set Tristate Fails %d\n",
				ret);
		return -EINVAL;
	}

	pr_debug("BYT Machine: slot_width = %d\n",
			slot_width);
	pr_debug("BYT Machine: tx_mask = 0X%08x\n",
			tx_mask);
	pr_debug("BYT Machine: rx_mask = 0X%08x\n",
			rx_mask);
	pr_debug("BYT Machine: tristate_offset = 0X%08x\n",
			tristate_offset);

	return 0;

} /* byt_ssp2_ia_dai_link_hw_params*/

static int byt_ssp0_ia_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	pr_debug("%s substream->runtime->rate %d\n",
			__func__,
			substream->runtime->rate);

	/* select clock source
			SSP_CLK_ONCHIP = 0x0,
			SSP_CLK_NET,
			SSP_CLK_EXT,
			SSP_CLK_AUDIO
	 */

	snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);

	return 0;
}/*byt_ssp0_ia_dai_link_prepare*/

static int byt_ssp1_ia_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	pr_debug("%s substream->runtime->rate %d\n",
			__func__,
			substream->runtime->rate);

	/* select clock source
			SSP_CLK_ONCHIP = 0x0,
			SSP_CLK_NET,
			SSP_CLK_EXT,
			SSP_CLK_AUDIO
	 */
	snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);

	return 0;
}/*byt_ssp1_ia_dai_link_prepare*/

static int byt_ssp2_ia_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	pr_debug("%s substream->runtime->rate %d\n",
			__func__,
			substream->runtime->rate);

	/* select clock source
			SSP_CLK_ONCHIP = 0x0,
			SSP_CLK_NET,
			SSP_CLK_EXT,
			SSP_CLK_AUDIO
	 */
	snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);

	return 0;
}/*byt_ssp1_ia_dai_link_prepare*/


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

static unsigned int channels_2[] = {
	2,
};

static unsigned int channels_8[] = {
	8,
};

static struct snd_pcm_hw_constraint_list constraints_2ch = {
	.count	= ARRAY_SIZE(channels_2),
	.list	= channels_2,
};

static struct snd_pcm_hw_constraint_list constraints_8ch = {
	.count	= ARRAY_SIZE(channels_8),
	.list	= channels_8,
};

/*User can consider splitting this to 3 machine separately or scalable decision making.
 * e.g. 8k, 16k and 48k for different SSPs
 */
static int byt_ssp_ia_dai_link_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *str_runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "BYT Machine (SSP startup): ERROR NULL substream->pcm\n");
	if (!substream->pcm)
		return -EINVAL;

	if(ctx->pdata->is_tdm) {
		pr_info("%s : Card %s setting hardware configs for TDM8\n",
				__func__, soc_card->name);
		str_runtime->hw = ssp_pcm_hw_tdm8;
		ret = snd_pcm_hw_constraint_list(str_runtime, 0,
								SNDRV_PCM_HW_PARAM_CHANNELS,
							&constraints_8ch);
	} else {
		pr_info("%s : Card %s setting hardware configs for I2S\n",
						__func__, soc_card->name);
		str_runtime->hw = ssp_pcm_hw_stereo;
		ret = snd_pcm_hw_constraint_list(str_runtime, 0,
						SNDRV_PCM_HW_PARAM_CHANNELS,
					&constraints_2ch);
	}

	if (ret) {
		pr_err("BYT : %s : Fail to set channel constraint for stereo\n", __func__);
		return ret;
	}

	return snd_pcm_hw_constraint_list(str_runtime, 0,
					   SNDRV_PCM_HW_PARAM_RATE,
					   &constraints_48000);
}/*byt_ssp_ia_dai_link_startup*/

/*SOC ops for IA path*/
static struct snd_soc_ops byt_ssp0_dai_link_ops = {
	.startup = byt_ssp_ia_dai_link_startup,
	.hw_params = byt_ssp0_ia_dai_link_hw_params,
	.prepare = byt_ssp0_ia_dai_link_prepare,
};

static struct snd_soc_ops byt_ssp1_dai_link_ops = {
	.startup = byt_ssp_ia_dai_link_startup,
	.hw_params = byt_ssp1_ia_dai_link_hw_params,
	.prepare = byt_ssp1_ia_dai_link_prepare,
};

static struct snd_soc_ops byt_ssp2_dai_link_ops = {
	.startup = byt_ssp_ia_dai_link_startup,
	.hw_params = byt_ssp2_ia_dai_link_hw_params,
	.prepare = byt_ssp2_ia_dai_link_prepare,
};

static struct snd_soc_dai_link byt_msic_ssp0_dailink[] = {
	{
		.name = "Baytrail SSP0 DAI",
		.stream_name = "SSP0 Stream",
		.cpu_dai_name = SSP0_CARD_AK4614_DAI_NAME,
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		.codec_dai_name = SOC_DAI_DRIVER_AK4614,
		/* Codec_name is created when I2C client is registered through
		 * arch/x86/platform/intel-mid/device_libs/platform_byt_audio.c
		 * User to check i2c bus-device registration when using different codec
		 */
		.codec_name = "ak4614.1-0011",
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
		.platform_name = "mid-ssp-dai.0",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_ssp0_dai_link_ops,
	},
};

static struct snd_soc_dai_link byt_msic_ssp1_dailink[] = {
	{
		.name = "Baytrail SSP1 DAI",
		.stream_name = "SSP1 Stream",
		.cpu_dai_name = SSP1_CARD_AK4614_DAI_NAME,
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		.codec_dai_name = SOC_DAI_DRIVER_AK4614,
		/* Codec_name is created when I2C client is registered through
		 * arch/x86/platform/intel-mid/device_libs/platform_byt_audio.c
		 * User to check i2c bus-device registration when using different codec
		 */
		.codec_name = "ak4614.0-0010",
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
		.platform_name = "mid-ssp-dai.1",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_ssp1_dai_link_ops,
	},
};

static struct snd_soc_dai_link byt_msic_ssp2_dailink[] = {
	{
		.name = "Baytrail SSP2 DAI",
		.stream_name = "SSP2 Stream",
		.cpu_dai_name = SSP2_CARD_AK4614_DAI_NAME,
#ifndef CONFIG_SND_SOC_DUMMY_CODEC
		.codec_dai_name = SOC_DAI_DRIVER_AK4614,
		/* Codec_name is created when I2C client is registered through
		 * arch/x86/platform/intel-mid/device_libs/platform_byt_audio.c
		 * User to check i2c bus-device registration when using different codec
		 */
		.codec_name = "ak4614.1-0010",
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
		.platform_name = "mid-ssp-dai.2",
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_ssp2_dai_link_ops,
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

/* SSP0 SoC card */
static struct snd_soc_card snd_soc_card_ssp0_byt = {
	.name = "BYT_SSP0",
	.dai_link = byt_msic_ssp0_dailink,
	.num_links = ARRAY_SIZE(byt_msic_ssp0_dailink),
};

/* SSP1 SoC card */
static struct snd_soc_card snd_soc_card_ssp1_byt = {
	.name = "BYT_SSP1",
	.dai_link = byt_msic_ssp1_dailink,
	.num_links = ARRAY_SIZE(byt_msic_ssp1_dailink),
};

/* SSP2 SoC card */
static struct snd_soc_card snd_soc_card_ssp2_byt = {
	.name = "BYT_SSP2",
	.dai_link = byt_msic_ssp2_dailink,
	.num_links = ARRAY_SIZE(byt_msic_ssp2_dailink),
};

/* forward declaration */
static void byt_async_register_card(void *data, async_cookie_t cookie);

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;
	struct ssp_platform_config *pdata = pdev->dev.platform_data;

	pr_debug("In %s\n", __func__);
	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	drv->socdev = pdev;
	drv->pdata = pdata;

	if (pdata->port_number == 0)
	{
		/* ssp0 sound card */
		snd_soc_card_ssp0_byt.dev = &pdev->dev;
		/* register the soc card */
		async_schedule(byt_async_register_card,&snd_soc_card_ssp0_byt);
		snd_soc_card_set_drvdata(&snd_soc_card_ssp0_byt, drv);

		platform_set_drvdata(pdev, &snd_soc_card_ssp0_byt);
	}
	else if (pdata->port_number == 1)
	{
		/* ssp1 sound card */
		snd_soc_card_ssp1_byt.dev = &pdev->dev;
		/* register the soc card */
		async_schedule(byt_async_register_card,&snd_soc_card_ssp1_byt);
		snd_soc_card_set_drvdata(&snd_soc_card_ssp1_byt, drv);
		platform_set_drvdata(pdev, &snd_soc_card_ssp1_byt);
	}
	else if (pdata->port_number == 2)
	{
		/* ssp2 sound card */
		snd_soc_card_ssp2_byt.dev = &pdev->dev;
		/* register the soc card */
		async_schedule(byt_async_register_card,&snd_soc_card_ssp2_byt);
		snd_soc_card_set_drvdata(&snd_soc_card_ssp2_byt, drv);
		platform_set_drvdata(pdev, &snd_soc_card_ssp2_byt);
	}

	pr_debug("successfully exited probe\n");
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
	  /*sound card registration failed, perform necessary cleanup*/
	  struct byt_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	  snd_byt_mc_remove(ctx->socdev);
	}
  
}
const struct dev_pm_ops snd_byt_mc_iassp_pm_ops = {
	.suspend = snd_byt_suspend,
	.resume = snd_byt_resume,
	.poweroff = snd_byt_poweroff,
};

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_ia_ak4614",
		.pm   = &snd_byt_mc_iassp_pm_ops,
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
};

static int __init snd_byt_driver_init(void)
{
	pr_info("BYT: Baytrail Machine Driver byt_ia_ak4614 is registering.\n");
	return platform_driver_register(&snd_byt_mc_driver);
}
late_initcall(snd_byt_driver_init);

static void __exit snd_byt_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_byt_mc_driver);
}
module_exit(snd_byt_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver - IA");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:byt-ia-ak4614");
