/*
 * platform_byt_audio_rt.c:Baytrail audio platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/intel-mid.h>
#include "platform_byt_audio.h"
#include <asm/platform_byt_audio.h>

#define RT5642_BASE_ADDRESS 0x1c
#define ASOC_BIN_PLATFORM_DRIVER "sst-platform"
#define ASOC_MACHINE_DRIVER "byt_rt5642"

#define NUMBER_OF_I2C_BUS 1

int port_number = 2;
	
enum byt_i2c_bus {
	BYT_I2C_BUS0 = 0x0,
};

static const struct i2c_board_info i2c_bus_info[] = {

	{ I2C_BOARD_INFO("rt5640", RT5642_BASE_ADDRESS) },

};

static struct ssp_platform_config byt_config[]={
		[SSP_2] ={
				.i2s_settings={ //I2S settings
						.master_mode_clk_selection = SSP_MASTER_CLOCK_UNDEFINED,
						.master_mode_standard_freq = 0xFFFF,
						.tx_tristate_phase = TXD_TRISTATE_LAST_PHASE_OFF,  //OFF
						.slave_clk_free_running_status = SLAVE_SSPCLK_ON_ALWAYS, //SCFR Always On
						.ssp_duplex_mode = RX_AND_TX_MODE,
						.ssp_trailing_byte_mode = SSP_TRAILING_BYTE_HDL_BY_IA, //Handled by processor
						.ssp_tx_dma = SSP_TX_DMA_ENABLE, //DMA on: SSP_TX_DMA_ENABLE, off: SSP_TX_DMA_MASK
						.ssp_rx_dma = SSP_RX_DMA_ENABLE, //DMA on: SSP_RX_DMA_ENABLE, off: SSP_RX_DMA_MASK
						.rx_fifo_interrupt = SSP_RX_FIFO_OVER_INT_DISABLE, //RIM - run time
						.tx_fifo_interrupt = SSP_TX_FIFO_UNDER_INT_DISABLE, //TIM - run time
						.ssp_rx_timeout_interrupt_status = SSP_RX_TIMEOUT_INT_DISABLE, //Receiver Time-out interrupts are disabled
						.ssp_trailing_byte_interrupt_status = SSP_TRAILING_BYTE_INT_DISABLE, //Peripheral Trailing Byte Interrupts are disabled
						.ssp_loopback_mode_status = SSP_LOOPBACK_OFF,
						.ssp_rx_fifo_threshold = MID_SSP_RX_FIFO_THRESHOLD, //8 (X-1 will be programmed)
						.ssp_tx_fifo_threshold = MID_SSP_TX_FIFO_THRESHOLD, //7
						.ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_LOW, //i2s starts low
						.ssp_end_transfer_state = SSP_END_DATA_TRANSFER_STATE_LOW, //i2s ok
						.ssp_psp_T1 = 0,/*Opt for Slave mode*/
						.ssp_psp_T2 = 1,/*Opt for Slave mode*/
						.ssp_psp_T4 = 0,
						.ssp_psp_T5 = 0,
						.ssp_psp_T6 = 0x1F, /*Frm width is required for Master mode, but ignored in slave mode(user can set to 0)*/
						.ssp_divider_bypass = BYPASS,
						.ssp_divider_enable = DIV_ENABLE,
						.ssp_divider_update = MN_UPDATE,
						.m_value = 0x60000180,
						.n_value = 0x00000c35,

			},
			.is_IA = false,
			.port_number = 2,
			.is_tdm = false,
		},

};

int create_platform_device(const char *name, int count, bool is_IA)
{
	int ret = 0;
	struct platform_device *pdev;
	
	pr_debug("create_platform_device %s count=%d is_IA=%d",name,count, is_IA);

	pdev = platform_device_alloc(name, -1);
		
	if (pdev == NULL)
	{
		pr_err("out of memory for platform device %s\n",name);
		return ret;
	}
		
	ret = platform_device_add_data(pdev,&byt_config[count],sizeof(byt_config[count]));
	if (ret)
	{
		 pr_err("failed to add machine audio data\n");

	}
		
	ret = platform_device_add(pdev);
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
	}

	return 0;
}

void *byt_audio_platform_data(void *info)
{
	int ret;
	pr_debug("BYT : in byt_audio_platform_data\n");
	
	//Create BIN SSP platform driver
	ret = create_platform_device(ASOC_BIN_PLATFORM_DRIVER, port_number, false);
	
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
		return NULL;
	}
	
	//Create BIN SSP Machine driver
	ret = create_platform_device(ASOC_MACHINE_DRIVER, port_number, false);
	
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
		return NULL;
	}
		
	/* bus number - the i2c port which is connected with the codec */
	i2c_register_board_info(0, &i2c_bus_info[0], 1);

	return NULL;
}
