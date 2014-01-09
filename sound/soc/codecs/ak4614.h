/*
 * ak4614.h  --  audio driver for ak4614
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


#ifndef _AK4614_H
#define _AK4614_H


#define SOC_DAI_DRIVER_AK4614 "ak4614-AIF1"
#define AK4614_CLKID_MCLK 0

#define AK4614_00_POWER_MANAGEMENT1		0x00
#define AK4614_01_POWER_MANAGEMENT2		0x01
#define AK4614_02_POWER_MANAGEMENT3		0x02
#define AK4614_03_CONTROL1				0x03
#define AK4614_04_CONTROL2				0x04
#define AK4614_05_DE_EMPHASIS1			0x05
#define AK4614_06_DE_EMPHASIS2			0x06
#define AK4614_07_OVERFLOW_DETECT		0x07
#define AK4614_08_ZERO_DETECT			0x08
#define AK4614_09_INPUT_CONTROL			0x09
#define AK4614_0A_OUTPUT_CONTROL		0x0A
#define AK4614_0B_LOUT1_VOLUME_CONTROL	0x0B
#define AK4614_0C_ROUT1_VOLUME_CONTROL	0x0C
#define AK4614_0D_LOUT2_VOLUME_CONTROL	0x0D
#define AK4614_0E_ROUT2_VOLUME_CONTROL	0x0E
#define AK4614_0F_LOUT3_VOLUME_CONTROL	0x0F
#define AK4614_10_ROUT3_VOLUME_CONTROL	0x10
#define AK4614_11_LOUT4_VOLUME_CONTROL	0x11
#define AK4614_12_ROUT4_VOLUME_CONTROL	0x12
#define AK4614_13_LOUT5_VOLUME_CONTROL	0x13
#define AK4614_14_ROUT5_VOLUME_CONTROL	0x14
#define AK4614_15_LOUT6_VOLUME_CONTROL	0x15
#define AK4614_16_ROUT6_VOLUME_CONTROL	0x16

#define AK4614_MAX_REGISTERS	(AK4614_16_ROUT6_VOLUME_CONTROL + 1)

/* Bitfield Definitions */

/* AK4614_00_POWER_MANAGEMENT1 (0x00) Fields */
#define AK4614_PMVCM_RSTN				0x09

/* Make AK4614 DAC ON*/
#define AK4614_PMDAC				 0x02

/* AK4614_03_CONTROL1 (0x03) Fields */
#define AK4614_TDM_DIF			         0xF8
#define AK4614_DIF_16RIGHT_MODE			(0 << 3)
#define AK4614_DIF_24LEFT_MODE			(3 << 3)
#define AK4614_DIF_I2S_MODE				(4 << 3)
#define AK4614_DIF_TDM256_24RIGHT_MODE	(0x12 << 3)
#define AK4614_DIF_TDM256_24LEFT_MODE	(0x13 << 3)
#define AK4614_DIF_TDM256_I2S_MODE		(0x14 << 3)

/* AK4614_04_CONTROL2 (0x04) Fields */
#define AK4614_CKS_DFS					0x3C
#define AK4614_CKS_DFS_256FS_48KHZ     (0 << 2)
#define AK4614_CKS_DFS_256FS_96KHZ     (1 << 2)
#define AK4614_CKS_DFS_128FS_192KHZ    (2 << 2)
#define AK4614_CKS_DFS_512FS_48KHZ     (8 << 2)

#endif
