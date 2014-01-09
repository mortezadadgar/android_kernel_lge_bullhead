/*
 * <Driver for I2S protocol on SSP (Moorestown and Medfield hardware)>
 * Copyright (c) 2010, Intel Corporation.
 * Louis LE GALL <louis.le.gall intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef MID_I2S_IF_H_
#define MID_I2S_IF_H_

#include <linux/types.h>

#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_dma.h>

#define USE_DMA_LLI /*May turn this off is only using I2S*/


#define SSP0_INSTANCE 0
#define SSP1_INSTANCE 1
#define SSP2_INSTANCE 2

enum intel_mid_i2s_ssp_usage {
	SSP_USAGE_UNASSIGNED = 0x00,
	SSP_USAGE_BLUETOOTH_FM = 0x01,
	SSP_USAGE_MODEM = 0x02,
	SSP_USAGE_CARD_AK4614_0 = 0x03,
	SSP_USAGE_CARD_AK4614_1 = 0x04,
	SSP_USAGE_CARD_AK4614_2 = 0x05,
	SSP_USAGE_COUNT = 0x06
};

enum intel_mid_i2s_ssp_cmd {
	SSP_CMD_UNASSIGNED = 0x00,
	SSP_CMD_SET_HW_CONFIG = 0x01,
	SSP_CMD_ENABLE_SSP = 0x02,
	SSP_CMD_DISABLE_SSP = 0x03,
	SSP_CMD_ALLOC_TX = 0x04,
	SSP_CMD_FREE_TX = 0x05,
	SSP_CMD_ALLOC_RX = 0x06,
	SSP_CMD_FREE_RX = 0x07,
	SSP_CMD_ABORT = 0x08,
	SSP_CMD_PAUSE_TX = 0x09,
	SSP_CMD_PAUSE_RX = 0x0A,
	SSP_CMD_RESUME_TX = 0x0B,
	SSP_CMD_RESUME_RX = 0x0C,
	SSP_CMD_COUNT = 0x0D,
};

struct intel_mid_i2s_lli {
	u32 *addr;
	u32 leng;
};
enum i2s_lli_mode {
	I2S_NON_CIRCULAR_MODE,
	I2S_CIRCULAR_MODE
};

enum i2s_stream_direction {
	I2S_STREAM_PLAYBACK,
	I2S_STREAM_CAPTURE
};

struct intel_mid_i2s_hdl *intel_mid_i2s_open(
			enum intel_mid_i2s_ssp_usage usage);

void  intel_mid_i2s_close(struct intel_mid_i2s_hdl *handle);

int intel_mid_i2s_rd_req(struct intel_mid_i2s_hdl *handle, u32 *dst,
			size_t len, void *param);
int intel_mid_i2s_wr_req(struct intel_mid_i2s_hdl *handle, u32 *src,
			size_t len, void *param);

int intel_mid_i2s_lli_rd_req(struct intel_mid_i2s_hdl *drv_data,
		struct intel_mid_i2s_lli *lli_array, int lli_length,
		enum i2s_lli_mode lli_mode, void *param);

int intel_mid_i2s_lli_wr_req(struct intel_mid_i2s_hdl *drv_data,
		struct intel_mid_i2s_lli *lli_array, int lli_length,
		enum i2s_lli_mode lli_mode, void *param);

int intel_mid_i2s_set_busy(struct intel_mid_i2s_hdl	*drv_data, enum i2s_stream_direction stream_direction);

int intel_mid_i2s_set_wr_cb(struct intel_mid_i2s_hdl *handle,
					int (*write_callback)(void *param));
int intel_mid_i2s_set_rd_cb(struct intel_mid_i2s_hdl *handle,
					int (*read_callback)(void *param));

int intel_mid_i2s_command(struct intel_mid_i2s_hdl *drv_data,
			enum intel_mid_i2s_ssp_cmd cmd,
			const struct intel_mid_i2s_settings *hw_ssp_settings,
			int streamdir);

int intel_mid_i2s_get_tx_fifo_level(struct intel_mid_i2s_hdl *handle);
int intel_mid_i2s_get_rx_fifo_level(struct intel_mid_i2s_hdl *handle);
int intel_mid_i2s_flush(struct intel_mid_i2s_hdl *handle);
void intel_mid_i2s_set_modem_probe_cb(void(*probe_cb)(void));
void intel_mid_i2s_set_modem_remove_cb(void(*remove_cb)(void));

#endif /* MID_I2S_IF_H_ */
