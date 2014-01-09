/*
 *  intel_sst_stream.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com>
 *		KP Jeeja <jeeja.kp@intel.com>
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
 *  This file contains the stream operations of SST driver
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

#if 0 /* unused code */
/*
 * sst_check_device_type - Check the medfield device type
 *
 * @device: Device to be checked
 * @num_ch: Number of channels queried
 * @pcm_slot: slot to be enabled for this device
 *
 * This checks the deivce against the map and calculates pcm_slot value
 */
/*BYT TODO: Refer to mfld code under sst_alloc_stream_mfld */
static int sst_check_device_type(u32 device, u32 num_chan, u32 *pcm_slot)
{
	/* device id range starts from 1 */
	if (device > MAX_NUM_STREAMS) {
		pr_debug("sst: device type invalid %d\n", device);
		return -EINVAL;
	}
	if (sst_drv_ctx->streams[device].status == STREAM_UN_INIT) {
		if (device == SND_SST_DEVICE_VIBRA && num_chan == 1)
			*pcm_slot = 0x10;
		else if (device == SND_SST_DEVICE_HAPTIC && num_chan == 1)
			*pcm_slot = 0x20;
		else if (device == SND_SST_DEVICE_IHF && num_chan == 1)
			*pcm_slot = 0x04;
		else if (device == SND_SST_DEVICE_IHF && num_chan == 2)
			*pcm_slot = 0x0C;
		else if (device == SND_SST_DEVICE_HEADSET && num_chan == 1)
			*pcm_slot = 0x01;
		else if (device == SND_SST_DEVICE_HEADSET && num_chan == 2)
			*pcm_slot = 0x03;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 1)
			*pcm_slot = 0x01;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 2)
			*pcm_slot = 0x03;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 3)
			*pcm_slot = 0x07;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 4)
			*pcm_slot = 0x0F;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan > 4)
			*pcm_slot = 0x1F;
		else if (device == SND_SST_DEVICE_COMPRESS &&
				(num_chan == 2 || num_chan == 1))
			*pcm_slot = sst_drv_ctx->compressed_slot;
		else {
			pr_debug("No condition satisfied.. ret err\n");
			return -EINVAL;
		}
	} else {
		pr_debug("this stream state is not uni-init, is %d\n",
					sst_drv_ctx->streams[device].status);
		return -EBADRQC;
	}
	pr_debug("returning slot %x\n", *pcm_slot);

	return 0;
}
#endif

static unsigned int get_byt_stream_id(u32 device)
{
	int str_id;
	/*device id range starts from 1 */
	pr_debug("device_id %d\n", device);
	if (device == SND_SST_DEVICE_HEADSET)
		str_id = 1;
	else if (device == SND_SST_DEVICE_CAPTURE)
		str_id = 2;
	else if (device == SND_SST_DEVICE_COMPRESS)
		str_id = 3;
	else
		return -EINVAL;

	if (sst_drv_ctx->streams[str_id].status != STREAM_UN_INIT) {
		pr_debug("this stream state is not uni-init, is %d\n",
					sst_drv_ctx->streams[str_id].status);
		return -EBADRQC;
	}
	pr_debug("str_id %d\n", str_id);

	return str_id;
}


static void sst_stream_tstamp_byte_copied_update( int str_id, u64 byte_copied )
{
	struct snd_sst_tstamp fw_tstamp;
	int offset;
	void __iomem *addr;

	/* The time-stamp structure is per stream, so remember 
     * to offer to the correct stream-id
     */	
	memcpy_fromio(&fw_tstamp,
		((void *)(sst_drv_ctx->mailbox + sst_drv_ctx->tstamp)
		+(str_id * sizeof(fw_tstamp))),
		sizeof(fw_tstamp));

	fw_tstamp.bytes_copied = byte_copied/2;

	addr =  ((void *)(sst_drv_ctx->mailbox + sst_drv_ctx->tstamp)) +
			(str_id * sizeof(fw_tstamp));

	offset =  offsetof(struct snd_sst_tstamp, bytes_copied);
	sst_shim_write64(addr, offset, fw_tstamp.bytes_copied);
}


int sst_alloc_stream_byt(char *params)
{
	struct ipc_post *msg = NULL;
	struct snd_sst_alloc_params_byt alloc_param;  /* alloc_param as defined in ipc_fw.h */
	struct snd_sst_params *str_params;           /* intel_sst_ioctl.h */
	struct snd_sst_stream_params_byt *sparams;   /* intel_sst_fw_ipc.h */
	struct snd_sst_frames_info *frame_info;      /* intel_sst_ioctl.h */
	struct stream_info *str_info;
	unsigned int pcm_slot = 0x0C, num_ch;
	int str_id, i;
	unsigned int stream_ops, device;
	unsigned long irq_flags;
	u8 codec;
	u64 byte_copied = 0;

	pr_debug("In %s\n", __func__);

	if (sst_drv_ctx->pci_id != SST_BYT_PCI_ID)
	{
		pr_debug("SST: alloc_stream_byt is registered for non BYT!!! \n");		
		BUG_ON(1);
	}

	BUG_ON(!params);

	/*  Parse the sound param input from user via "snd_sst_params" */
	str_params = (struct snd_sst_params *)params;
	stream_ops = str_params->ops;
	codec = str_params->codec;
	device = str_params->device_type;
	sparams = (struct snd_sst_stream_params_byt *) &str_params->sparams;
	frame_info = &str_params->frame_info;
	num_ch = sst_get_num_channel(str_params);

	/* Mapping the sound device to audio stream by refering to get_byt_stream_id() */
	mutex_lock(&sst_drv_ctx->stream_lock);
	str_id = get_byt_stream_id(device);   
	mutex_unlock(&sst_drv_ctx->stream_lock);
	if (str_id <= 0)
		return -EBUSY;

	/* allocate device type context*/ 
	/* init the stream's <struct stream_info> i.e sst_drv_ctx->streams[str_id] */
	sst_init_stream(&sst_drv_ctx->streams[str_id], codec, str_id, stream_ops, pcm_slot);

	/* send msg to FW to allocate a stream */
	/* Create <ipc_post> message space together with its mailbox */
	if (sst_create_large_msg(&msg))
		return -ENOMEM;

	/* Filling in the Stream Type (snd_sst_str_type) */
	alloc_param.str_type.codec_type = codec;   
	alloc_param.str_type.str_type = SST_STREAM_TYPE_MUSIC;
	alloc_param.str_type.operation = stream_ops;
	alloc_param.str_type.protected_str = 0; /* non drm */
	alloc_param.str_type.time_slots = pcm_slot;
	alloc_param.str_type.reserved = 0;
	alloc_param.str_type.result = 0;

	/* Copy in <snd_pcm_params_byt> passed from sst_platform.c ---> filled in sst_fill_pcm_params */
	memcpy(&alloc_param.stream_params, sparams, sizeof(*sparams));
	/* Copy in <snd_sst_frames_info> passed from sst_platform.c ---> filled in sst_fill_frames_info */
	memcpy(&alloc_param.frame_info, frame_info, sizeof(*frame_info));
	/* Prepare the IPC message (ipc_post->ipc_header_byt) */
	sst_fill_header_byt(&msg->byt_header, IPC_IA_ALLOC_STREAM, 1, str_id);
	/* Set the size of mailbox in IPC header "data" field (ipc_post->ipc_header_byt)*/
	msg->byt_header.p.header_low_payload.part.data = sizeof(alloc_param) + sizeof(u32);
	/* Prepare mailbox content (driver copy) - IPC low header, write to mailbox will happen in sst_post_message_byt() */
	memcpy( (u8 *) msg->mailbox_data, (u8 *) &(msg->byt_header.p.header_low_payload.full), sizeof(u32));
	/* Prepare mailbox content (driver copy) - <struct snd_sst_alloc_params_byt>, write to mailbox will happen in sst_post_message_byt() */
	memcpy( (u8 *) msg->mailbox_data + sizeof(u32), (u8 *) &alloc_param, sizeof(alloc_param));

	/* Update the time-stamp bytes_copied byte to FW LPE */
	for ( i=0 ; (i < alloc_param.frame_info.num_entries) ; i++ ) {
		byte_copied += alloc_param.frame_info.ring_buf_info[i].size;
	}
	sst_stream_tstamp_byte_copied_update( str_id, byte_copied );

	/* Updating the stream info into SST Driver Context */
	str_info = &sst_drv_ctx->streams[str_id];
	str_info->ctrl_blk.condition = false;
	str_info->ctrl_blk.ret_code = 0;
	str_info->ctrl_blk.on = true;
	str_info->num_ch = num_ch;

	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);   /* Send IPC message */

	return str_id;
}

int sst_alloc_stream(char *params)
{
	if (sst_drv_ctx->pci_id == SST_BYT_PCI_ID)
		return sst_alloc_stream_byt(params);

	return 0;
}

/*
 * sst_alloc_stream_response - process alloc reply
 *
 * @str_id:	stream id for which the stream has been allocated
 * @resp		the stream response from firware
 *
 * This function is called by firmware as a response to stream allcoation
 * request
 */
int sst_alloc_stream_response(unsigned int str_id,
				struct snd_sst_alloc_response *resp)
{
	int retval = 0;
	struct stream_info *str_info;
	struct snd_sst_lib_download *lib_dnld;

	pr_debug("SST DEBUG: stream number given = %d\n", str_id);
	str_info = &sst_drv_ctx->streams[str_id];

	if (resp->str_type.result == SST_LIB_ERR_LIB_DNLD_REQUIRED) {
		lib_dnld = kzalloc(sizeof(*lib_dnld), GFP_KERNEL);
			if (!lib_dnld) {
				pr_debug("SST DBG: mem alloc failed\n");
				retval = -ENOMEM;
			} else {
				memcpy(lib_dnld, &resp->lib_dnld,
						sizeof(*lib_dnld));
			}
	} else {
		lib_dnld = NULL;
	}
	if (str_info->ctrl_blk.on == true) {
		str_info->ctrl_blk.on = false;
		str_info->ctrl_blk.data = lib_dnld;
		str_info->ctrl_blk.condition = true;
		str_info->ctrl_blk.ret_code = resp->str_type.result;
		pr_debug("SST DEBUG: sst_alloc_stream_response: waking up.\n");
		wake_up(&sst_drv_ctx->wait_queue);
	} else {
		kfree(lib_dnld);
		pr_debug("SST DEBUG: sst_alloc_stream_response: ctrl block not on\n");
		retval = -EIO;
	}

	return retval;
}


/**
* sst_get_fw_info - Send msg to query for firmware configurations
* @info: out param that holds the firmare configurations
*
* This function is called when the firmware configurations are queiried for
*/
int sst_get_fw_info(struct snd_sst_fw_info *info)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	unsigned long irq_flags;

	pr_debug("sst_get_fw_info called\n");

	if (sst_create_short_msg(&msg)) {
		pr_err("message creation failed\n");
		return -ENOMEM;
	}

#ifdef SST_DRV_BYT
	/* Fill the ipc_header for BYT */
	sst_fill_header_byt(&msg->byt_header, IPC_IA_GET_FW_INFO, 0, 0);
#else
	sst_fill_header(&msg->header, IPC_IA_GET_FW_INFO, 0, 0);
#endif /* SST_DRV_BYT */

	sst_drv_ctx->fw_info_blk.condition = false;
	sst_drv_ctx->fw_info_blk.ret_code = 0;
	sst_drv_ctx->fw_info_blk.on = true;
	sst_drv_ctx->fw_info_blk.data = info;

	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_timeout(sst_drv_ctx, &sst_drv_ctx->fw_info_blk);

	if (retval) {
		pr_err("error in fw_info = %d\n", retval);
		retval = -EIO;
	}

	return retval;
}


/**
* sst_stream_stream - Send msg for a pausing stream
* @str_id:	 stream ID
*
* This function is called by any function which wants to start
* a stream.
*/

/* FIXME: For BYT, currently sst_start_stream() always does "start_stream_params->byte_offset=0"
 * A new function maybe needed to support non-zero
 * snd_sst_start_stream_params-->byte_offset 
 */

int sst_start_stream(int str_id)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;

	pr_debug("sst_start_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	if (str_info->status != STREAM_RUNNING)
		return -EBADRQC;

	if (sst_create_large_msg(&msg))
		return -ENOMEM;

#ifdef SST_DRV_BYT
	if (sst_drv_ctx->pci_id == SST_BYT_PCI_ID) {
		struct snd_sst_start_stream_params start_stream_params;	

		/* Prepare ipc_header_byt header */
		sst_fill_header_byt(&msg->byt_header, IPC_IA_START_STREAM, 1, str_id);
		/* Set the size of mailbox in IPC header "data" field (ipc_post->ipc_header_byt) */
		msg->byt_header.p.header_low_payload.part.data = sizeof(start_stream_params) + sizeof(u32);

		/* For BYT case, we need to pass in <struct snd_sst_start_stream_params> intel_sst_fw_ipc.h */
		start_stream_params.byte_offset = 0;

		/* Prepare mailbox content (driver copy) - IPC low header, write to mailbox will happen in sst_post_message_byt() */
		memcpy( (u8 *) msg->mailbox_data, (u8 *) &(msg->byt_header.p.header_low_payload.full), sizeof(u32));

		/* Prepare mailbox content (driver copy) - <snd_sst_start_stream_params>, write to mailbox will happen in sst_post_message_byt() */
		memcpy( (u8 *) msg->mailbox_data + sizeof(u32), (u8 *) &start_stream_params, sizeof(start_stream_params));
	}
#else
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID) {
		sst_fill_header_mrfld(&msg->mrfld_header,
				IPC_IA_START_STREAM, 1, str_id);
		msg->mrfld_header.p.header_low_payload =
				sizeof(u64) + sizeof(u64);
		memcpy(msg->mailbox_data,
			&msg->mrfld_header.p.header_high.full, sizeof(u32));
		memcpy(msg->mailbox_data + sizeof(u32),
			&msg->mrfld_header.p.header_low_payload, sizeof(u32));
		memset(msg->mailbox_data + sizeof(u64), 0, sizeof(u32));
	} else {
		sst_fill_header(&msg->header, IPC_IA_START_STREAM, 1, str_id);
		msg->header.part.data =  sizeof(u32) + sizeof(u32);
		memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
		memset(msg->mailbox_data + sizeof(u32), 0, sizeof(u32));
	}
#endif

	sst_drv_ctx->ops->sync_post_message(msg);

	return retval;
}

/*
 * sst_pause_stream - Send msg for a pausing stream
 * @str_id:	 stream ID
 *
 * This function is called by any function which wants to pause
 * an already running stream.
 */
int sst_pause_stream(int str_id)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;

	pr_debug("SST DBG:sst_pause_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;
	if (str_info->status == STREAM_PAUSED)
		return 0;
	if (str_info->status == STREAM_RUNNING ||
		str_info->status == STREAM_INIT) {
		if (str_info->prev == STREAM_UN_INIT)
			return -EBADRQC;
		if (str_info->ctrl_blk.on == true) {
			pr_err("control path is in use\n");
			return -EINVAL;
		}
		if (sst_create_short_msg(&msg))
			return -ENOMEM;

#ifdef SST_DRV_BYT /* use BYT specific header */
		sst_fill_header_byt(&msg->byt_header, IPC_IA_PAUSE_STREAM, 0, str_id);
#else
		sst_fill_header(&msg->header, IPC_IA_PAUSE_STREAM, 0, str_id);
#endif
		str_info->ctrl_blk.condition = false;
		str_info->ctrl_blk.ret_code = 0;
		str_info->ctrl_blk.on = true;
		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node,
				&sst_drv_ctx->ipc_dispatch_list);

		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		retval = sst_wait_timeout(sst_drv_ctx, &str_info->ctrl_blk);

		if (retval == 0) {
			str_info->prev = str_info->status;
			str_info->status = STREAM_PAUSED;
		} else if (retval == SST_ERR_INVALID_STREAM_ID) {
			retval = -EINVAL;
			mutex_lock(&sst_drv_ctx->stream_lock);
			sst_clean_stream(str_info);
			mutex_unlock(&sst_drv_ctx->stream_lock);
		}
	} else {
		retval = -EBADRQC;
		pr_debug("SST DBG:BADRQC for stream\n ");
	}

	return retval;
}

/**
 * sst_resume_stream - Send msg for resuming stream
 * @str_id:		stream ID
 *
 * This function is called by any function which wants to resume
 * an already paused stream.
 */
int sst_resume_stream(int str_id)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;

	pr_debug("SST DBG:sst_resume_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;
	if (str_info->status == STREAM_RUNNING)
			return 0;
	if (str_info->status == STREAM_PAUSED) {
		if (str_info->ctrl_blk.on == true) {
			pr_err("SST ERR: control path in use\n");
			return -EINVAL;
		}
		if (sst_create_short_msg(&msg)) {
			pr_err("SST ERR: mem allocation failed\n");
			return -ENOMEM;
		}

#ifdef SST_DRV_BYT /* use BYT specific header */
			sst_fill_header_byt(&msg->byt_header, IPC_IA_RESUME_STREAM, 0, str_id);
#else
		sst_fill_header(&msg->header, IPC_IA_RESUME_STREAM, 0, str_id);
#endif

		str_info->ctrl_blk.condition = false;
		str_info->ctrl_blk.ret_code = 0;
		str_info->ctrl_blk.on = true;
		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node,
				&sst_drv_ctx->ipc_dispatch_list);
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		retval = sst_wait_timeout(sst_drv_ctx, &str_info->ctrl_blk);

		if (!retval) {
			if (str_info->prev == STREAM_RUNNING)
				str_info->status = STREAM_RUNNING;
			else
				str_info->status = STREAM_INIT;
			str_info->prev = STREAM_PAUSED;
		} else if (retval == -SST_ERR_INVALID_STREAM_ID) {
			retval = -EINVAL;
			mutex_lock(&sst_drv_ctx->stream_lock);
			sst_clean_stream(str_info);
			mutex_unlock(&sst_drv_ctx->stream_lock);
		}
	} else {
		retval = -EBADRQC;
		pr_err("SST ERR: BADQRC for stream\n");
	}

	return retval;
}


/**
 * sst_drop_stream - Send msg for stopping stream
 * @str_id:		stream ID
 *
 * This function is called by any function which wants to stop
 * a stream.
 */
int sst_drop_stream(int str_id)
{
	int retval = 0;
	struct stream_info *str_info;

	pr_debug("SST DBG:sst_drop_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;

	if (str_info->status != STREAM_UN_INIT) {

		str_info->prev = STREAM_UN_INIT;
		str_info->status = STREAM_INIT;
		str_info->cumm_bytes = 0;
		sst_send_sync_msg(IPC_IA_DROP_STREAM, str_id);
	} else {
		retval = -EBADRQC;
		pr_debug("BADQRC for stream, state %x\n", str_info->status);
	}

	return retval;
}

/**
* sst_drain_stream - Send msg for draining stream
* @str_id:		stream ID
*
* This function is called by any function which wants to drain
* a stream.
*/
int sst_drain_stream(int str_id)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;

	pr_debug("SST DBG:sst_drain_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;
	if (str_info->status != STREAM_RUNNING &&
		str_info->status != STREAM_INIT &&
		str_info->status != STREAM_PAUSED) {
			pr_err("SST ERR: BADQRC for stream = %d\n",
				       str_info->status);
			return -EBADRQC;
	}

	if (str_info->status == STREAM_INIT) {
		if (sst_create_short_msg(&msg)) {
			pr_err("SST ERR: mem allocation failed\n");
			return -ENOMEM;
		}

#ifdef SST_DRV_BYT
			sst_fill_header_byt(&msg->byt_header, IPC_IA_DRAIN_STREAM, 0, str_id);
#else
		sst_fill_header(&msg->header, IPC_IA_DRAIN_STREAM, 0, str_id);
#endif

		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		str_info->data_blk.condition = false;
		str_info->data_blk.ret_code = 0;
		str_info->data_blk.on = true;
		retval = sst_wait_interruptible(sst_drv_ctx,
						&str_info->data_blk);
	}

	return retval;
}

/**
 * sst_free_stream - Frees a stream
 * @str_id:		stream ID
 *
 * This function is called by any function which wants to free
 * a stream.
 */
int sst_free_stream(int str_id)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;

	pr_debug("SST DBG:sst_free_stream for %d\n", str_id);

	mutex_lock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->sst_state == SST_UN_INIT) {
		mutex_unlock(&sst_drv_ctx->sst_lock);
		return -ENODEV;
	}
	mutex_unlock(&sst_drv_ctx->sst_lock);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;

	mutex_lock(&str_info->lock);
	if (str_info->status != STREAM_UN_INIT) {
		str_info->prev =  str_info->status;
		str_info->status = STREAM_UN_INIT;
		mutex_unlock(&str_info->lock);
		if (str_info->ctrl_blk.on == true) {
			pr_err("SST ERR: control path in use\n");
			return -EINVAL;
		}
		if (sst_create_short_msg(&msg)) {
			pr_err("SST ERR: mem allocation failed\n");
			return -ENOMEM;
		}
#ifdef SST_DRV_BYT

		sst_fill_header_byt(&msg->byt_header, IPC_IA_FREE_STREAM, 0, str_id);
#else
		if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
			sst_fill_header_mrfld(&msg->mrfld_header, 
						IPC_IA_FREE_STREAM, 0, str_id);
		else
			sst_fill_header(&msg->header, IPC_IA_FREE_STREAM,
								 0, str_id);
#endif /* SST_DRV_BYT */

		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		if (str_info->data_blk.on == true) {
			str_info->data_blk.condition = true;
			str_info->data_blk.ret_code = 0;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		str_info->ctrl_blk.on = true;
		str_info->ctrl_blk.condition = false;
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		retval = sst_wait_timeout(sst_drv_ctx, &str_info->ctrl_blk);

		pr_debug("sst: wait for free returned %d\n", retval);
		mutex_lock(&sst_drv_ctx->stream_lock);
		sst_clean_stream(str_info);
		mutex_unlock(&sst_drv_ctx->stream_lock);
		pr_debug("SST DBG:Stream freed\n");
	} else {
		mutex_unlock(&str_info->lock);
		retval = -EBADRQC;
		pr_debug("SST DBG:BADQRC for stream\n");
	}

	return retval;
}


