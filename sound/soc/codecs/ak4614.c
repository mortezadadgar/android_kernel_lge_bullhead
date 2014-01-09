/*
 * ak4614.c  --  audio driver for AK4614
 *
 * Copyright (C) 2012 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      12/11/06	    1.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#undef NEW_SOUND_FORMAT

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/core.h>

#include "ak4614.h"

#define AK4614_DEBUG

#ifdef AK4614_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif

/* AK4614 Codec Private Data */
struct ak4614_priv {
	u8 reg_cache[AK4614_MAX_REGISTERS];
	void *control_data;
};

/* ak4614 register cache & default register settings */
static const u8 ak4614_reg[AK4614_MAX_REGISTERS] = {
	0x0F,	/*	0x00	AK4614_00_POWER_MANAGEMENT1	*/
	0x07,	/*	0x01	AK4614_01_POWER_MANAGEMENT2	*/
	0x3F,	/*	0x02	AK4614_02_POWER_MANAGEMENT3	*/
	0x20,	/*	0x03	AK4614_03_CONTROL1	*/
	0x20,	/*	0x04	AK4614_04_CONTROL2	*/
	0x55,	/*	0x05	AK4614_05_DE_EMPHASIS1	*/
	0x05,	/*	0x06	AK4614_06_DE_EMPHASIS2	*/
	0x07,	/*	0x07	AK4614_07_OVERFLOW_DETECT	*/
	0x0F,	/*	0x08	AK4614_08_ZERO_DETECT	*/
	0x07,	/*	0x09	AK4614_09_INPUT_CONTROL	*/
	0x3F,	/*	0x0A	AK4614_0A_OUTPUT_CONTROL	*/
	0x00,	/*	0x0B	AK4614_0B_LOUT1_VOLUME_CONTROL	*/
	0x00,	/*	0x0C	AK4614_0C_ROUT1_VOLUME_CONTROL	*/
	0x00,	/*	0x0D	AK4614_0D_LOUT2_VOLUME_CONTROL	*/
	0x00,	/*	0x0E	AK4614_0E_ROUT2_VOLUME_CONTROL	*/
	0x00,	/*	0x0F	AK4614_0F_LOUT3_VOLUME_CONTROL	*/
	0x00,	/*	0x10	AK4614_10_ROUT3_VOLUME_CONTROL	*/
	0x00,	/*	0x11	AK4614_11_LOUT4_VOLUME_CONTROL	*/
	0x00,	/*	0x12	AK4614_12_ROUT4_VOLUME_CONTROL	*/
	0x00,	/*	0x13	AK4614_13_LOUT5_VOLUME_CONTROL	*/
	0x00,	/*	0x14	AK4614_14_ROUT5_VOLUME_CONTROL	*/
	0x00,	/*	0x15	AK4614_15_LOUT6_VOLUME_CONTROL	*/
	0x00,	/*	0x16	AK4614_16_ROUT6_VOLUME_CONTROL	*/

};

static const struct {
	int readable;   /* Mask of readable bits */
	int writable;   /* Mask of writable bits */
} ak4614_access_masks[] = {
    { 0xFF, 0x0F },
    { 0xFF, 0x07 },
    { 0xFF, 0x3F },
    { 0xFF, 0xFF },
    { 0xFF, 0x7F },
    { 0xFF, 0xFF },
    { 0xFF, 0x0F },
    { 0xFF, 0x0F },
    { 0xFF, 0xCF },
    { 0xFF, 0x07 },
    { 0xFF, 0x3F },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF },
    { 0xFF, 0xFF }
};

/*
 * Output Digital volume control: (DATT)
 * from -127.0 to 0 dB in 0.5 dB steps (mute instead of -127.5 dB)
 */
static DECLARE_TLV_DB_SCALE(datt_tlv, -12750, 50, 1);

static const char *ats_select[]  = 
{
	"4096/fs", "2048/fs", "512/fs", "256/fs",
};

static const char *dem_select[]  = 
{
	"44.1kHz", "off", "48kHz", "32kHz",
};

static const struct soc_enum ak4614_enum[] = 
{
    /* Note: Dont change the order, then you need to change
     * in mixer control also */
    /* arg1: number of elements arg2: elements name array*/
    SOC_ENUM_SINGLE(AK4614_03_CONTROL1, 1, 4, ats_select),
    SOC_ENUM_SINGLE(AK4614_05_DE_EMPHASIS1, 0, 4, dem_select),
    SOC_ENUM_SINGLE(AK4614_05_DE_EMPHASIS1, 2, 4, dem_select),
    SOC_ENUM_SINGLE(AK4614_05_DE_EMPHASIS1, 4, 4, dem_select),
    SOC_ENUM_SINGLE(AK4614_05_DE_EMPHASIS1, 6, 4, dem_select),
    SOC_ENUM_SINGLE(AK4614_06_DE_EMPHASIS2, 0, 4, dem_select),
    SOC_ENUM_SINGLE(AK4614_06_DE_EMPHASIS2, 2, 4, dem_select),
};


#if 0 /* unused code */
static int ak4614_writeMask(struct snd_soc_codec *, u16, u16, u16);
#endif

static const struct snd_kcontrol_new ak4614_snd_controls[] = {
	SOC_SINGLE_TLV("LOUT1 Volume",
			AK4614_0B_LOUT1_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("ROUT1 Volume",
			AK4614_0C_ROUT1_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("LOUT2 Volume",
			AK4614_0D_LOUT2_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("ROUT2 Volume",
			AK4614_0E_ROUT2_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("LOUT3 Volume",
			AK4614_0F_LOUT3_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("ROUT3 Volume",
			AK4614_10_ROUT3_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("LOUT4 Volume",
			AK4614_11_LOUT4_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("ROUT4 Volume",
			AK4614_12_ROUT4_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("LOUT5 Volume",
			AK4614_13_LOUT5_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("ROUT5 Volume",
			AK4614_14_ROUT5_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("LOUT6 Volume",
			AK4614_15_LOUT6_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),
	SOC_SINGLE_TLV("ROUT6 Volume",
			AK4614_16_ROUT6_VOLUME_CONTROL, 0, 0xFF, 1, datt_tlv),

    SOC_SINGLE("PMAD1", AK4614_01_POWER_MANAGEMENT2, 0, 1, 0),
    SOC_SINGLE("PMAD2", AK4614_01_POWER_MANAGEMENT2, 1, 1, 0),
    SOC_SINGLE("PMAD3", AK4614_01_POWER_MANAGEMENT2, 2, 1, 0),
    SOC_SINGLE("PMDA1", AK4614_02_POWER_MANAGEMENT3, 0, 1, 0),
    SOC_SINGLE("PMDA2", AK4614_02_POWER_MANAGEMENT3, 1, 1, 0),
    SOC_SINGLE("PMDA3", AK4614_02_POWER_MANAGEMENT3, 2, 1, 0),
    SOC_SINGLE("PMDA4", AK4614_02_POWER_MANAGEMENT3, 3, 1, 0),
    SOC_SINGLE("PMDA5", AK4614_02_POWER_MANAGEMENT3, 4, 1, 0),
    SOC_SINGLE("PMDA6", AK4614_02_POWER_MANAGEMENT3, 5, 1, 0),
    SOC_SINGLE("ADC1_DIFF_INPUT", AK4614_09_INPUT_CONTROL, 0, 1, 0),
    SOC_SINGLE("ADC2_DIFF_INPUT", AK4614_09_INPUT_CONTROL, 1, 1, 0),
    SOC_SINGLE("ADC3_DIFF_INPUT", AK4614_09_INPUT_CONTROL, 2, 1, 0),
    SOC_SINGLE("DAC1_DIFF_OUTPUT", AK4614_0A_OUTPUT_CONTROL, 0, 1, 0),
    SOC_SINGLE("DAC2_DIFF_OUTPUT", AK4614_0A_OUTPUT_CONTROL, 1, 1, 0),
    SOC_SINGLE("DAC3_DIFF_OUTPUT", AK4614_0A_OUTPUT_CONTROL, 2, 1, 0),
    SOC_SINGLE("DAC4_DIFF_OUTPUT", AK4614_0A_OUTPUT_CONTROL, 3, 1, 0),
    SOC_SINGLE("DAC5_DIFF_OUTPUT", AK4614_0A_OUTPUT_CONTROL, 4, 1, 0),
    SOC_SINGLE("DAC6_DIFF_OUTPUT", AK4614_0A_OUTPUT_CONTROL, 5, 1, 0),

    SOC_SINGLE("SMUTE", AK4614_03_CONTROL1, 0, 1, 0),
	SOC_ENUM("ATS", ak4614_enum[0]),
	SOC_ENUM("DEM1", ak4614_enum[1]),
	SOC_ENUM("DEM2", ak4614_enum[2]),
	SOC_ENUM("DEM3", ak4614_enum[3]),
	SOC_ENUM("DEM4", ak4614_enum[4]),
	SOC_ENUM("DEM5", ak4614_enum[5]),
	SOC_ENUM("DEM6", ak4614_enum[6]),
};


static const struct snd_soc_dapm_widget ak4614_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADC", "Capture", AK4614_00_POWER_MANAGEMENT1, 2, 0),
	/* FIXME: [1/3] Workaround to permanently turn on DAC to solve 100% gain issue.
	 * The line below is removed due to PMDAC register bit toggling will cause playback volume reset.
	 * Related to workaround in "ak4614_set_bias_level".
	 */
	//SND_SOC_DAPM_DAC("DAC", "Playback", AK4614_00_POWER_MANAGEMENT1, 1, 0),

	SND_SOC_DAPM_INPUT("LINEIN1"),
	SND_SOC_DAPM_INPUT("LINEIN2"),
	SND_SOC_DAPM_INPUT("LINEIN3"),

	SND_SOC_DAPM_OUTPUT("LINEOUT1"),
	SND_SOC_DAPM_OUTPUT("LINEOUT2"),
	SND_SOC_DAPM_OUTPUT("LINEOUT3"),
	SND_SOC_DAPM_OUTPUT("LINEOUT4"),
	SND_SOC_DAPM_OUTPUT("LINEOUT5"),
	SND_SOC_DAPM_OUTPUT("LINEOUT6"),
};

static const struct snd_soc_dapm_route ak4614_intercon[] = {
	/* Sink, Control, Source */
	{ "ADC", NULL, "LINEIN1" },
	{ "ADC", NULL, "LINEIN2" },
	{ "ADC", NULL, "LINEIN3" },

	/*FIXME: [2/3] Workaround to permanently turn on DAC to solve 100% gain issue.
	 * Removed the routing below due to "ak4614_dapm_widgets[] DAC widget" is taken out.
	 */
	/*{ "LINEOUT1", NULL, "DAC" },
	{ "LINEOUT2", NULL, "DAC" },
	{ "LINEOUT3", NULL, "DAC" },
	{ "LINEOUT4", NULL, "DAC" },
	{ "LINEOUT5", NULL, "DAC" },
	{ "LINEOUT6", NULL, "DAC" },*/
};

static int ak4614_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 	fs;

	akdbgprt("\t[AK4614] %s(%d)\n",__FUNCTION__,__LINE__);

	fs = snd_soc_read(codec, AK4614_04_CONTROL2);
	fs &= ~(AK4614_CKS_DFS);   /* Clear out CKS & DFS bits */

	switch (params_rate(params)) {
		case 32000:
		case 44100:
		case 48000:
			fs |= AK4614_CKS_DFS_256FS_48KHZ;   /* The default codec crystal freq = 12.288MHz*/
			break;
		case 88200:
		case 96000:
			fs |= AK4614_CKS_DFS_256FS_96KHZ;
			break;
		case 176400:
		case 192000:
			fs |= AK4614_CKS_DFS_128FS_192KHZ;
			break;

		default:
			return -EINVAL;
	}
	snd_soc_write(codec, AK4614_04_CONTROL2, fs);

	return 0;
}

static int ak4614_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 	fs;

	switch (clk_id) {
	case AK4614_CLKID_MCLK:
		break;
	default:
		return -EINVAL;
	}

	fs = snd_soc_read(codec, AK4614_04_CONTROL2);
	fs &= ~(AK4614_CKS_DFS);   /* Clear out CKS & DFS bits */

	switch ( freq ) {
		case 32000:
		case 44100:
		case 48000:
			fs |= AK4614_CKS_DFS_256FS_48KHZ;   /* The default codec crystal freq = 12.288Mhz*/
			break;
		case 88200:
		case 96000:
			fs |= AK4614_CKS_DFS_256FS_96KHZ;
			break;
		case 176400:
		case 192000:
			fs |= AK4614_CKS_DFS_128FS_192KHZ;
			break;

		default:
			return -EINVAL;
	}

	akdbgprt("\t[AK4614] %s Write AK4614_04_CONTROL2 =0x%x \n" , __func__, fs);
	snd_soc_write(codec, AK4614_04_CONTROL2, fs);

	return 0;
}

static int ak4614_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	struct snd_soc_codec *codec = dai->codec;
	u8 format;

	/* set master/slave audio interface */

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			/* Codec clock & framing clock is set by M/S-pin, no effect here */
			break;
		case SND_SOC_DAIFMT_CBM_CFM: 
			/* Codec clock & framing clock is set by M/S-pin, no effect here */
			break;
		case SND_SOC_DAIFMT_CBS_CFM:
		case SND_SOC_DAIFMT_CBM_CFS:
		default:
			dev_err(codec->dev, "Clock mode unsupported");
			return -EINVAL;
    	}

	format = snd_soc_read(codec, AK4614_03_CONTROL1);
	format &= ~AK4614_TDM_DIF;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			format |= AK4614_DIF_I2S_MODE;
			break;

		case SND_SOC_DAIFMT_LEFT_J:
			format |= AK4614_DIF_24LEFT_MODE;
			break;

		case SND_SOC_DAIFMT_RIGHT_J:
			format |= AK4614_DIF_16RIGHT_MODE;
			break;
	/* Currently, in Linux Kernel 3.8, the following DAI formats
	 * are not originally supported and have been newly added.
	 * To add new formats, please add to soc-dai.h
	 */
		case SND_SOC_DAIFMT_TDM256_RIGHT_J:
			format |= AK4614_DIF_TDM256_24RIGHT_MODE;
			break;
		case SND_SOC_DAIFMT_TDM256_LEFT_J:
			format |= AK4614_DIF_TDM256_24LEFT_MODE;
			break;

		default:
			return -EINVAL;
	}

	/* set mode and format */
	akdbgprt("\t[AK4614] %s Write AK4614_04_CONTROL1 =0x%x \n" , __func__, format);
	snd_soc_write(codec, AK4614_03_CONTROL1, format);

	mdelay(2);

	return 0;
}

static int ak4614_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	int	ret;

	switch (reg) {
//		case :
//			ret = 1;
		default:
			ret = 0;
			break;
	}
	return(ret);
}

static int ak4614_readable(struct snd_soc_codec *codec, unsigned int reg)
{

	if (reg >= ARRAY_SIZE(ak4614_access_masks))
		return 0;
	return ak4614_access_masks[reg].readable != 0;
}

/*
 * Read ak4614 register cache
 */
static inline u32 ak4614_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4614_reg));
    return (u32)cache[reg];
}

/*
 * Write ak4614 register cache
 */
static inline void ak4614_write_reg_cache(
struct snd_soc_codec *codec, 
u16 reg,
u16 value)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4614_reg));
    cache[reg] = (u8)value;
}

unsigned int ak4614_i2c_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
	if (ret < 0)
		pr_err("\t[AK4614] %s(%d), i2c_smbus_read_byte_data returns %d\n",
				__FUNCTION__, __LINE__, ret);
	return ret;
}

static int ak4614_i2c_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int ret;

	ak4614_write_reg_cache(codec, reg, value);
	ret = i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF),
			(u8)(value & 0xFF));
	if (ret < 0) {
		pr_err("\t[AK4614] %s(%d), i2c_smbus_write_byte_data returns %d\n",
				__FUNCTION__, __LINE__, ret);
		return -EIO;
	}
	return 0;
}

#if 0 /* unused code */
/*
 * Write with Mask to  AK4614 register space
 */
static int ak4614_writeMask(
struct snd_soc_codec *codec,
u16 reg,
u16 mask,
u16 value)
{
    u16 olddata;
    u16 newdata;

	if ( (mask == 0) || (mask == 0xFF) ) {
		newdata = value;
	}
	else {
		olddata = ak4614_read_reg_cache(codec, reg);
	    newdata = (olddata & ~(mask)) | value;
	}

	ak4614_write_reg_cache(codec, reg, newdata);

	snd_soc_write(codec, (unsigned int)reg, (unsigned int)newdata);

	akdbgprt("\t[ak4614_writeMask] %s(%d): (addr,data)=(%x, %x)\n",__FUNCTION__,__LINE__, reg, newdata);

    return(0);
}
#endif

static int ak4614_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	u8 reg;

	akdbgprt("\t[AK4614] %s(%d)\n",__FUNCTION__,__LINE__);
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		reg = snd_soc_read(codec, AK4614_00_POWER_MANAGEMENT1);

		/*
		 * FIXME: [3/3] Workaround to permanently turn on DAC to solve 100% gain issue.
		 */
		snd_soc_write(codec, AK4614_00_POWER_MANAGEMENT1, reg | AK4614_PMVCM_RSTN | AK4614_PMDAC);
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, AK4614_00_POWER_MANAGEMENT1, 0x00);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define AK4614_RATES		(SNDRV_PCM_RATE_48000)
#define AK4614_FORMATS_PLAYBACK		(SNDRV_PCM_FMTBIT_S16_LE\
										| SNDRV_PCM_FMTBIT_S24_3LE\
										| SNDRV_PCM_FMTBIT_S24_LE\
										| SNDRV_PCM_FMTBIT_S32_LE)
#define AK4614_FORMATS_CAPTURE		(SNDRV_PCM_FMTBIT_S16_LE\
										| SNDRV_PCM_FMTBIT_S24_3LE\
										| SNDRV_PCM_FMTBIT_S24_LE\
										| SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops ak4614_dai_ops = {
	.hw_params	= ak4614_hw_params,
	.set_sysclk	= ak4614_set_dai_sysclk,
	.set_fmt	= ak4614_set_dai_fmt,
};

struct snd_soc_dai_driver ak4614_dai[] = {
	{
		.name = SOC_DAI_DRIVER_AK4614,
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 12,
		       .rates = AK4614_RATES,
		       .formats = AK4614_FORMATS_PLAYBACK,
		},
		.capture = {
		       .stream_name = "Capture",
		       .channels_min = 1,
		       .channels_max = 8, //Actual is 6 channels only.
		       .rates = AK4614_RATES,
		       .formats = AK4614_FORMATS_CAPTURE,
		},
		.ops = &ak4614_dai_ops,
	},
};

static int ak4614_write_cache_reg(
struct snd_soc_codec *codec,
u16  regs,
u16  rege)
{
	u32	reg, cache_data;

	reg = regs;
	do {
		cache_data = ak4614_read_reg_cache(codec, reg);
		snd_soc_write(codec, (unsigned int)reg, (unsigned int)cache_data);
		reg ++;
	} while (reg <= rege);

	return(0);
}

static int ak4614_suspend(struct snd_soc_codec *codec)
{
	return ak4614_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int ak4614_resume(struct snd_soc_codec *codec)
{
	ak4614_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	ak4614_write_cache_reg(codec, 0, AK4614_16_ROUT6_VOLUME_CONTROL);

	return 0;
}


static void ak4614_reset(
struct snd_soc_codec *codec)
{
//  Set PDN pin to "L".
	mdelay(1);
//  Set PDN pin to "H".
	snd_soc_write(codec, AK4614_00_POWER_MANAGEMENT1, AK4614_PMVCM_RSTN);
}

static int ak4614_probe(struct snd_soc_codec *codec)
{
	struct ak4614_priv *ak4614 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	akdbgprt("\t[AK4614] %s(%d)\n", __FUNCTION__, __LINE__);

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	codec->control_data = ak4614->control_data;
	codec->write = ak4614_i2c_write;
	codec->read = ak4614_i2c_read;

	ak4614_reset(codec);
	ak4614_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	snd_soc_add_codec_controls(codec, ak4614_snd_controls,
	                   ARRAY_SIZE(ak4614_snd_controls));
    return ret;
}

static int ak4614_remove(struct snd_soc_codec *codec)
{
	akdbgprt("\t[AK4614] %s(%d)\n", __FUNCTION__, __LINE__);
	return ak4614_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

struct snd_soc_codec_driver soc_codec_dev_ak4614 = {
	.probe = ak4614_probe,
	.remove = ak4614_remove,
	.suspend =	ak4614_suspend,
	.resume =	ak4614_resume,
	.set_bias_level = ak4614_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(ak4614_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = ak4614_reg,
	.readable_register = ak4614_readable,
	.volatile_register = ak4614_volatile,	
	.dapm_widgets = ak4614_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4614_dapm_widgets),
	.dapm_routes = ak4614_intercon,
	.num_dapm_routes = ARRAY_SIZE(ak4614_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4614);

static int ak4614_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ak4614_priv *ak4614;
	int ret;

	akdbgprt("\t[AK4614] %s(%d)\n",__FUNCTION__, __LINE__);
	ak4614 = devm_kzalloc(&i2c->dev, sizeof(struct ak4614_priv), GFP_KERNEL);
	if (ak4614 == NULL)
		return -ENOMEM;
	ak4614->control_data = i2c;
	i2c_set_clientdata(i2c, ak4614);

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4614, ak4614_dai, ARRAY_SIZE(ak4614_dai));
	if (ret < 0)
		pr_err("\t[AK4614 Error!] %s(%d)\n", __FUNCTION__, __LINE__);
	return ret;
}

static int __exit ak4614_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id ak4614_i2c_id[] = {
	{ "ak4614", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4614_i2c_id);

static struct i2c_driver ak4614_i2c_driver = {
	.driver = {
		.name = "ak4614",
		.owner = THIS_MODULE,
	},
	.probe = ak4614_i2c_probe,
	.remove = ak4614_i2c_remove,
	.id_table = ak4614_i2c_id,
};

module_i2c_driver(ak4614_i2c_driver);

MODULE_DESCRIPTION("ASoC ak4614 codec driver");
MODULE_LICENSE("GPL");
