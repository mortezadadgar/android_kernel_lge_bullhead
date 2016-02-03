/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2015 - 2016 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2015 - 2016 Intel Deutschland GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#include "iwl-trans.h"
#include "iwl-op-mode.h"
#include "iwl-fw.h"
#include "iwl-csr.h"

#include "xvt.h"
#include "iwl-dnt-cfg.h"

#define XVT_UCODE_ALIVE_TIMEOUT	HZ

struct iwl_xvt_alive_data {
	bool valid;
	u32 scd_base_addr;
};

static inline const struct fw_img *
iwl_get_ucode_image(struct iwl_xvt *xvt, enum iwl_ucode_type ucode_type)
{
	if (ucode_type >= IWL_UCODE_TYPE_MAX)
		return NULL;

	return &xvt->fw->img[ucode_type];
}

void iwl_xvt_free_fw_paging(struct iwl_xvt *xvt)
{
	int i;

	if (!xvt->fw_paging_db[0].fw_paging_block)
		return;

	for (i = 0; i < NUM_OF_FW_PAGING_BLOCKS; i++) {
		if (!xvt->fw_paging_db[i].fw_paging_block) {
			IWL_DEBUG_FW(xvt,
				     "Paging: block %d already freed, continue to next page\n",
				     i);

			continue;
		}

		__free_pages(xvt->fw_paging_db[i].fw_paging_block,
			     get_order(xvt->fw_paging_db[i].fw_paging_size));
	}

	memset(xvt->fw_paging_db, 0, sizeof(xvt->fw_paging_db));
}

static int iwl_xvt_fill_paging_mem(struct iwl_xvt *xvt,
				   const struct fw_img *image)
{
	int sec_idx, idx;
	u32 offset = 0;

	/*
	 * find where is the paging image start point:
	 * if CPU2 exist and it's in paging format, then the image looks like:
	 * CPU1 sections (2 or more)
	 * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between CPU1 to CPU2
	 * CPU2 sections (not paged)
	 * PAGING_SEPARATOR_SECTION delimiter - separate between CPU2
	 * non paged to CPU2 paging sec
	 * CPU2 paging CSS
	 * CPU2 paging image (including instruction and data)
	 */
	for (sec_idx = 0; sec_idx < IWL_UCODE_SECTION_MAX; sec_idx++) {
		if (image->sec[sec_idx].offset == PAGING_SEPARATOR_SECTION) {
			sec_idx++;
			break;
		}
	}

	if (sec_idx >= IWL_UCODE_SECTION_MAX) {
		IWL_ERR(xvt, "driver didn't find paging image\n");
		iwl_xvt_free_fw_paging(xvt);
		return -EINVAL;
	}

	/* copy the CSS block to the dram */
	IWL_DEBUG_FW(xvt, "Paging: load paging CSS to FW, sec = %d\n",
		     sec_idx);

	memcpy(page_address(xvt->fw_paging_db[0].fw_paging_block),
	       image->sec[sec_idx].data,
	       xvt->fw_paging_db[0].fw_paging_size);

	IWL_DEBUG_FW(xvt,
		     "Paging: copied %d CSS bytes to first block\n",
		     xvt->fw_paging_db[0].fw_paging_size);

	sec_idx++;

	/*
	 * copy the paging blocks to the dram
	 * loop index start from 1 since that CSS block already copied to dram
	 * and CSS index is 0.
	 * loop stop at num_of_paging_blk since that last block is not full.
	 */
	for (idx = 1; idx < xvt->num_of_paging_blk; idx++) {
		memcpy(page_address(xvt->fw_paging_db[idx].fw_paging_block),
		       image->sec[sec_idx].data + offset,
		       xvt->fw_paging_db[idx].fw_paging_size);

		IWL_DEBUG_FW(xvt,
			     "Paging: copied %d paging bytes to block %d\n",
			     xvt->fw_paging_db[idx].fw_paging_size,
			     idx);

		offset += xvt->fw_paging_db[idx].fw_paging_size;
	}

	/* copy the last paging block */
	if (xvt->num_of_pages_in_last_blk > 0) {
		memcpy(page_address(xvt->fw_paging_db[idx].fw_paging_block),
		       image->sec[sec_idx].data + offset,
		       FW_PAGING_SIZE * xvt->num_of_pages_in_last_blk);

		IWL_DEBUG_FW(xvt,
			     "Paging: copied %d pages in the last block %d\n",
			     xvt->num_of_pages_in_last_blk, idx);
	}

	return 0;
}

static int iwl_xvt_alloc_fw_paging_mem(struct iwl_xvt *xvt,
				       const struct fw_img *image)
{
	struct page *block;
	dma_addr_t phys = 0;
	int blk_idx = 0;
	int order, num_of_pages;
	int dma_enabled;

	if (xvt->fw_paging_db[0].fw_paging_block)
		return 0;

	dma_enabled = is_device_dma_capable(xvt->trans->dev);

	/* ensure BLOCK_2_EXP_SIZE is power of 2 of PAGING_BLOCK_SIZE */
	BUILD_BUG_ON(BIT(BLOCK_2_EXP_SIZE) != PAGING_BLOCK_SIZE);

	num_of_pages = image->paging_mem_size / FW_PAGING_SIZE;
	xvt->num_of_paging_blk = ((num_of_pages - 1) /
				    NUM_OF_PAGE_PER_GROUP) + 1;

	xvt->num_of_pages_in_last_blk =
		num_of_pages -
		NUM_OF_PAGE_PER_GROUP * (xvt->num_of_paging_blk - 1);

	IWL_DEBUG_FW(xvt,
		     "Paging: allocating mem for %d paging blocks, each block holds 8 pages, last block holds %d pages\n",
		     xvt->num_of_paging_blk,
		     xvt->num_of_pages_in_last_blk);

	/* allocate block of 4Kbytes for paging CSS */
	order = get_order(FW_PAGING_SIZE);
	block = alloc_pages(GFP_KERNEL, order);
	if (!block) {
		/* free all the previous pages since we failed */
		iwl_xvt_free_fw_paging(xvt);
		return -ENOMEM;
	}

	xvt->fw_paging_db[blk_idx].fw_paging_block = block;
	xvt->fw_paging_db[blk_idx].fw_paging_size = FW_PAGING_SIZE;

	if (dma_enabled) {
		phys = dma_map_page(xvt->trans->dev, block, 0,
				    PAGE_SIZE << order, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(xvt->trans->dev, phys)) {
			/*
			 * free the previous pages and the current one since
			 * we failed to map_page.
			 */
			iwl_xvt_free_fw_paging(xvt);
			return -ENOMEM;
		}
		xvt->fw_paging_db[blk_idx].fw_paging_phys = phys;
	} else {
		/* Currently not Supported */
	}

	IWL_DEBUG_FW(xvt,
		     "Paging: allocated 4K(CSS) bytes (order %d) for firmware paging.\n",
		     order);

	/*
	 * allocate blocks in dram.
	 * since that CSS allocated in fw_paging_db[0] loop start from index 1
	 */
	for (blk_idx = 1; blk_idx < xvt->num_of_paging_blk + 1; blk_idx++) {
		/* allocate block of PAGING_BLOCK_SIZE (32K) */
		order = get_order(PAGING_BLOCK_SIZE);
		block = alloc_pages(GFP_KERNEL, order);
		if (!block) {
			/* free all the previous pages since we failed */
			iwl_xvt_free_fw_paging(xvt);
			return -ENOMEM;
		}

		xvt->fw_paging_db[blk_idx].fw_paging_block = block;
		xvt->fw_paging_db[blk_idx].fw_paging_size = PAGING_BLOCK_SIZE;

		if (dma_enabled) {
			phys = dma_map_page(xvt->trans->dev, block, 0,
					    PAGE_SIZE << order,
					    DMA_BIDIRECTIONAL);
			if (dma_mapping_error(xvt->trans->dev, phys)) {
				/*
				 * free the previous pages and the current one
				 * since we failed to map_page.
				 */
				iwl_xvt_free_fw_paging(xvt);
				return -ENOMEM;
			}
			xvt->fw_paging_db[blk_idx].fw_paging_phys = phys;
		} else {
			/* Currently not Supported */
		}

		IWL_DEBUG_FW(xvt,
			     "Paging: allocated 32K bytes (order %d) for firmware paging.\n",
			     order);
	}

	return 0;
}

static int iwl_xvt_save_fw_paging(struct iwl_xvt *xvt,
				  const struct fw_img *fw)
{
	int ret;

	ret = iwl_xvt_alloc_fw_paging_mem(xvt, fw);
	if (ret)
		return ret;

	return iwl_xvt_fill_paging_mem(xvt, fw);
}

/* send paging cmd to FW in case CPU2 has paging image */
static int iwl_xvt_send_paging_cmd(struct iwl_xvt *xvt, const struct fw_img *fw)
{
	int blk_idx;
	__le32 dev_phy_addr;
	struct iwl_fw_paging_cmd fw_paging_cmd = {
		.flags =
			cpu_to_le32(PAGING_CMD_IS_SECURED |
				    PAGING_CMD_IS_ENABLED |
				    (xvt->num_of_pages_in_last_blk <<
				    PAGING_CMD_NUM_OF_PAGES_IN_LAST_GRP_POS)),
		.block_size = cpu_to_le32(BLOCK_2_EXP_SIZE),
		.block_num = cpu_to_le32(xvt->num_of_paging_blk),
	};

	/* loop for for all paging blocks + CSS block */
	for (blk_idx = 0; blk_idx < xvt->num_of_paging_blk + 1; blk_idx++) {
		dev_phy_addr =
			cpu_to_le32(xvt->fw_paging_db[blk_idx].fw_paging_phys >>
				    PAGE_2_EXP_SIZE);
		fw_paging_cmd.device_phy_addr[blk_idx] = dev_phy_addr;
	}

	return iwl_xvt_send_cmd_pdu(xvt, iwl_cmd_id(FW_PAGING_BLOCK_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(fw_paging_cmd), &fw_paging_cmd);
}

static bool iwl_alive_fn(struct iwl_notif_wait_data *notif_wait,
			 struct iwl_rx_packet *pkt,
			 void *data)
{
	struct iwl_xvt *xvt =
		container_of(notif_wait, struct iwl_xvt, notif_wait);
	struct iwl_xvt_alive_data *alive_data = data;
	struct xvt_alive_resp *palive;
	struct xvt_alive_resp_ver2 *palive2;
	struct xvt_alive_resp_ver3 *palive3;

	xvt->support_umac_log = false;

	if (iwl_rx_packet_payload_len(pkt) == sizeof(*palive)) {
		palive = (void *)pkt->data;
		xvt->error_event_table = le32_to_cpu(
						palive->error_event_table_ptr);
		alive_data->scd_base_addr = le32_to_cpu(
						palive->scd_base_ptr);
		alive_data->valid = le16_to_cpu(palive->status) ==
							IWL_ALIVE_STATUS_OK;
		xvt->fw_major_ver = palive->ucode_major;
		xvt->fw_minor_ver = palive->ucode_minor;

		IWL_DEBUG_FW(xvt, "Alive ucode status 0x%04x revision 0x%01X "
			     "0x%01X\n", le16_to_cpu(palive->status),
			     palive->ver_type, palive->ver_subtype);
	} else if (iwl_rx_packet_payload_len(pkt) == sizeof(*palive2)) {

		palive2 = (void *)pkt->data;

		xvt->error_event_table =
			le32_to_cpu(palive2->error_event_table_ptr);
		alive_data->scd_base_addr = le32_to_cpu(palive2->scd_base_ptr);
		xvt->sf_space.addr = le32_to_cpu(palive2->st_fwrd_addr);
		xvt->sf_space.size = le32_to_cpu(palive2->st_fwrd_size);

		alive_data->valid = le16_to_cpu(palive2->status) ==
				    IWL_ALIVE_STATUS_OK;
		xvt->fw_major_ver = palive2->ucode_major;
		xvt->fw_minor_ver = palive2->ucode_minor;
		xvt->umac_error_event_table =
			le32_to_cpu(palive2->error_info_addr);
		if (xvt->umac_error_event_table)
			xvt->support_umac_log = true;

		IWL_DEBUG_FW(xvt,
			     "Alive VER2 ucode status 0x%04x revision 0x%01X "
			     "0x%01X flags 0x%01X\n",
			     le16_to_cpu(palive2->status), palive2->ver_type,
			     palive2->ver_subtype, palive2->flags);

		IWL_DEBUG_FW(xvt,
			     "UMAC version: Major - 0x%x, Minor - 0x%x\n",
			     palive2->umac_major, palive2->umac_minor);
	} else {
		palive3 = (void *)pkt->data;

		xvt->error_event_table =
			le32_to_cpu(palive3->error_event_table_ptr);
		alive_data->scd_base_addr = le32_to_cpu(palive3->scd_base_ptr);
		xvt->sf_space.addr = le32_to_cpu(palive3->st_fwrd_addr);
		xvt->sf_space.size = le32_to_cpu(palive3->st_fwrd_size);

		alive_data->valid = le16_to_cpu(palive3->status) ==
				    IWL_ALIVE_STATUS_OK;
		xvt->fw_major_ver = le32_to_cpu(palive3->ucode_major);
		xvt->fw_minor_ver = le32_to_cpu(palive3->ucode_minor);
		xvt->umac_error_event_table =
			le32_to_cpu(palive3->error_info_addr);
		if (xvt->umac_error_event_table)
			xvt->support_umac_log = true;

		IWL_DEBUG_FW(xvt,
			     "Alive VER3 ucode status 0x%04x revision 0x%01X 0x%01X flags 0x%01X\n",
			     le16_to_cpu(palive3->status), palive3->ver_type,
			     palive3->ver_subtype, palive3->flags);

		IWL_DEBUG_FW(xvt,
			     "UMAC version: Major - 0x%x, Minor - 0x%x\n",
			     palive3->umac_major, palive3->umac_minor);
	}

	return true;
}

static int iwl_xvt_load_ucode_wait_alive(struct iwl_xvt *xvt,
					 enum iwl_ucode_type ucode_type)
{
	struct iwl_notification_wait alive_wait;
	struct iwl_xvt_alive_data alive_data;
	const struct fw_img *fw;
	int ret;
	enum iwl_ucode_type old_type = xvt->cur_ucode;
	static const u16 alive_cmd[] = { XVT_ALIVE };

	xvt->cur_ucode = ucode_type;
	fw = iwl_get_ucode_image(xvt, ucode_type);

	if (!fw)
		return -EINVAL;

	iwl_init_notification_wait(&xvt->notif_wait, &alive_wait,
				   alive_cmd, ARRAY_SIZE(alive_cmd),
				   iwl_alive_fn, &alive_data);

	ret = iwl_trans_start_fw_dbg(xvt->trans, fw,
				     ucode_type == IWL_UCODE_INIT,
				     xvt->sw_stack_cfg.fw_dbg_flags);
	if (ret) {
		xvt->cur_ucode = old_type;
		iwl_remove_notification(&xvt->notif_wait, &alive_wait);
		return ret;
	}

	/*
	 * Some things may run in the background now, but we
	 * just wait for the ALIVE notification here.
	 */
	ret = iwl_wait_notification(&xvt->notif_wait, &alive_wait,
				    XVT_UCODE_ALIVE_TIMEOUT);
	if (ret) {
		xvt->cur_ucode = old_type;
		return ret;
	}

	if (!alive_data.valid) {
		IWL_ERR(xvt, "Loaded ucode is not valid!\n");
		xvt->cur_ucode = old_type;
		return -EIO;
	}

	/* fresh firmware was loaded */
	xvt->fw_error = false;

	/*
	 * update the sdio allocation according to the pointer we get in the
	 * alive notification.
	 */
	ret = iwl_trans_update_sf(xvt->trans, &xvt->sf_space);

	iwl_trans_fw_alive(xvt->trans, alive_data.scd_base_addr);

	/*
	 * configure and operate fw paging mechanism.
	 * driver configures the paging flow only once, CPU2 paging image
	 * included in the IWL_UCODE_INIT image.
	 */
	if (fw->paging_mem_size) {
		/*
		 * When dma is not enabled, the driver needs to copy / write
		 * the downloaded / uploaded page to / from the smem.
		 * This gets the location of the place were the pages are
		 * stored.
		 */
		if (!is_device_dma_capable(xvt->trans->dev))
			return -1; /* not supported yet */

		ret = iwl_xvt_save_fw_paging(xvt, fw);
		if (ret) {
			IWL_ERR(xvt, "failed to save the FW paging image\n");
			return ret;
		}

		ret = iwl_xvt_send_paging_cmd(xvt, fw);
		if (ret) {
			IWL_ERR(xvt, "failed to send the paging cmd\n");
			iwl_xvt_free_fw_paging(xvt);
			return ret;
		}
	}

	if (ucode_type == IWL_UCODE_REGULAR)
		iwl_trans_ac_txq_enable(xvt->trans,
					IWL_XVT_DEFAULT_TX_QUEUE,
					IWL_XVT_DEFAULT_TX_FIFO, 0);

	xvt->fw_running = true;

	return 0;
}

int iwl_xvt_run_fw(struct iwl_xvt *xvt, u32 ucode_type, bool cont_run)
{
	int ret;

	if (ucode_type >= IWL_UCODE_TYPE_MAX)
		return -EINVAL;

	lockdep_assert_held(&xvt->mutex);

	if (xvt->state != IWL_XVT_STATE_UNINITIALIZED) {
		if (xvt->fw_running) {
			xvt->fw_running = false;
			if (xvt->cur_ucode == IWL_UCODE_REGULAR)
				iwl_trans_txq_disable(xvt->trans,
						      IWL_XVT_DEFAULT_TX_QUEUE,
						      true);
		}
		_iwl_trans_stop_device(xvt->trans, !cont_run);
	}

	if (cont_run)
		ret = _iwl_trans_start_hw(xvt->trans, false);
	else
		ret = iwl_trans_start_hw(xvt->trans);
	if (ret) {
		IWL_ERR(xvt, "Failed to start HW\n");
		return ret;
	}

	iwl_trans_set_bits_mask(xvt->trans,
				CSR_HW_IF_CONFIG_REG,
				CSR_HW_IF_CONFIG_REG_BIT_MAC_SI,
				CSR_HW_IF_CONFIG_REG_BIT_MAC_SI);

	/* Will also start the device */
	ret = iwl_xvt_load_ucode_wait_alive(xvt, ucode_type);
	if (ret) {
		IWL_ERR(xvt, "Failed to start ucode: %d\n", ret);
		iwl_trans_stop_device(xvt->trans);
	}
	iwl_dnt_start(xvt->trans);

	return ret;
}
