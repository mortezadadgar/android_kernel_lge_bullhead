/*
 * platform_byt_audio.c:Baytrail audio platform data initilization file
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

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
/* AK4614 base slave address is 0x10. The slave address can be configured by
 * the CAD0 and CAD1 switches on the board */
#define AK4614_BASE_ADDRESS 0x10
#define CAD0 	1
#define CAD1 	1 << 1
#define AK4614_SLAVE_ADDRESS_CAD_OFF (AK4614_BASE_ADDRESS)
#define AK4614_SLAVE_ADDRESS_CAD0 (AK4614_BASE_ADDRESS | CAD0)
#define NUMBER_OF_I2C_BUS 2
#endif /* CONFIG_SND_SOC_DUMMY_CODEC */

#define ASOC_BIN_MACHINE_DRIVER "byt_bin_ak4614"
#define ASOC_IA_MACHINE_DRIVER "byt_ia_ak4614"
#define ASOC_BIN_PLATFORM_DRIVER "sst-platform"
#define ASOC_IA_PLATFORM_DRIVER "mid-ssp-dai"

int num_of_ia_ssp = 0;
int num_of_bin_ssp = 0;

#ifndef CONFIG_SND_SOC_DUMMY_CODEC
enum byt_i2c_bus {
	BYT_I2C_BUS0 = 0x0,
	BYT_I2C_BUS1
};

static const struct i2c_board_info i2c_bus_info[] = {

	{ I2C_BOARD_INFO("ak4614", AK4614_SLAVE_ADDRESS_CAD_OFF) },
	{ I2C_BOARD_INFO("ak4614", AK4614_SLAVE_ADDRESS_CAD0) },

};
#endif

static struct ssp_platform_config byt_config[]={
		[SSP_0] ={
				/*
				 * TO BE DONE: use mixer to make it more flexible
				 * Please refer to the Baytrail EDS for programming the SSP register
				 * in the i2s_settings below.
				 */
				.i2s_settings={ //TDM settings
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
						.ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH, //i2s starts low
						.ssp_psp_T1 = 0,/*Opt for Slave mode*/
						.ssp_psp_T2 = 0,/*Opt for Slave mode*/
						.ssp_psp_T4 = 0,
						.ssp_psp_T5 = 0,
						.ssp_psp_T6 = 0x1F, /*Frm width is required for Master mode, but ignored in slave mode(user can set to 0)*/
						.ssp_divider_bypass = BYPASS,
						.ssp_divider_enable = DIV_ENABLE,
						.ssp_divider_update = MN_UPDATE,
						.m_value = 0x60000600,
						.n_value = 0x00000c35,
				},
				.is_IA = true,
				.port_number = 0,
				.is_tdm = true,
		},
		[SSP_1] ={
				.i2s_settings={ //TDM settings
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
						.ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH, //i2s starts low
						.ssp_psp_T1 = 0,/*Opt for Slave mode*/
						.ssp_psp_T2 = 0,/*Opt for Slave mode*/
						.ssp_psp_T4 = 0,
						.ssp_psp_T5 = 0,
						.ssp_psp_T6 = 0x1F, /*Frm width is required for Master mode, but ignored in slave mode(user can set to 0)*/
						.ssp_divider_bypass = BYPASS,
						.ssp_divider_enable = DIV_ENABLE,
						.ssp_divider_update = MN_UPDATE,
						.m_value = 0x60000600,
						.n_value = 0x00000c35,
				},
				.is_IA = true,
				.port_number = 1,
				.is_tdm = true,
		},
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
						.ssp_end_transfer_state = SSP_END_DATA_TRANSFER_STATE_LOW,
						.ssp_psp_T1 = 0,
						.ssp_psp_T2 = 0x1,
						.ssp_psp_T4 = 0,
						.ssp_psp_T5 = 0,
						.ssp_psp_T6 = 0x1F, /*Frm width is required for Master mode, but ignored in slave mode(user can set to 0)*/
						.ssp_divider_bypass = BYPASS,
						.ssp_divider_enable = DIV_ENABLE,
						.ssp_divider_update = MN_UPDATE,
						.m_value = 0x60000180,
						.n_value = 0x00000C35,

				},
				.is_IA = false,
				.port_number = 2,
				.is_tdm = false,
		},

};

int to_load_bin_ssp(void)
{
  return num_of_bin_ssp;
}


int get_number_of_ia_ssp(void)
{
  return num_of_ia_ssp;
}

//FIXME: to make it generic call for the user
int get_ia_port_number(void)
{
    int i;
    static int is_used =0; //enumerate from first data_structure, and then keep track of the ssp in used
    
    for(i= is_used; i<NUMBER_OF_SSP_PORT; i++)
    {
        if(byt_config[i].is_IA)
           break;
    }
    
    is_used = i; //keep track of the ssp_port
    is_used++;
    
    return byt_config[i].port_number;
}

int create_platform_device(const char *name, int count, bool is_IA)
{
	int ret = 0;
	int i,k;
	int dev_count = 0;
	struct platform_device *pdev;
	
	pr_debug("create_platform_device %s count=%d is_IA=%d",name,count, is_IA);
  	for (i = 0; i < count; i++)
	{
		for(k=dev_count; k<NUMBER_OF_SSP_PORT; k++)
		{
		  if(is_IA)
		  {
		    if(is_IA== byt_config[k].is_IA)
		    {
		      dev_count = k;
		      break;
		    }
		  }
		  else
		  {
		    if(is_IA== byt_config[k].is_IA)
		    {
		      dev_count = k;
		      break;
		    }
		  }
		}
		
		if(1 == count)
		  pdev = platform_device_alloc(name, -1);
		else
		  pdev = platform_device_alloc(name, dev_count);
		
		if (pdev == NULL) 
		{
			pr_err("out of memory for platform device %s\n",name);
			return ret;
		}
		
		ret = platform_device_add_data(pdev,&byt_config[dev_count],sizeof(byt_config[dev_count]));
		if (ret) 
		{
		    pr_err("failed to add machine audio data\n");

		}
		
		ret = platform_device_add(pdev);
		if (ret)
		{
			pr_err("BYT : Platform : platform_device_add failed \n");
		}
		dev_count++;
	}
	return 0;
}

void *byt_audio_platform_data(void *info)
{
	int ret, i;
	pr_debug("BYT : in byt_audio_platform_data\n");
	
	//Check the availability ssp port
	for(i=0; i < NUMBER_OF_SSP_PORT; i++)
	{
	  if(byt_config[i].is_IA)
	    num_of_ia_ssp++;
	  else
	    num_of_bin_ssp++;
	}

	//Create IA SSP platform driver
	ret = create_platform_device(ASOC_IA_PLATFORM_DRIVER, num_of_ia_ssp, true);
	
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
		return NULL;
	}
	
	//Create BIN SSP platform driver
	ret = create_platform_device(ASOC_BIN_PLATFORM_DRIVER, num_of_bin_ssp, false);
	
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
		return NULL;
	}
	
	//Create BIN SSP Machine driver
	ret = create_platform_device(ASOC_BIN_MACHINE_DRIVER, num_of_bin_ssp, false);
	
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
		return NULL;
	}
	
	//Create IA SSP Machine driver
	ret = create_platform_device(ASOC_IA_MACHINE_DRIVER, num_of_ia_ssp, true);
	
	if (ret)
	{
		pr_err("BYT : Platform : platform_device_add failed \n");
		return NULL;
	}

#ifndef CONFIG_SND_SOC_DUMMY_CODEC		
	/* bus number - the i2c port which is connected with the codec */
	for(i=0; i < NUMBER_OF_I2C_BUS; i++)
	{
	    switch(i)
	    {
	      case BYT_I2C_BUS0:
		i2c_register_board_info(i, &i2c_bus_info[0], 1);
		break;
	      case BYT_I2C_BUS1:
		i2c_register_board_info(i, i2c_bus_info, ARRAY_SIZE(i2c_bus_info));
		break;
	      default:
		break;
	    }
	}
#endif

	return NULL;
}
