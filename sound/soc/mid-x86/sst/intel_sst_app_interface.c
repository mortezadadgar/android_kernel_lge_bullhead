/*
 *  intel_sst_interface.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *  Harsha Priya <priya.harsha@intel.com>
 *  Dharageswari R <dharageswari.r@intel.com>
 *  Jeeja KP <jeeja.kp@intel.com>
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 *  This driver exposes the audio engine functionalities to the ALSA
 *	and middleware.
 *  Upper layer interfaces (MAD driver, MMF) to SST driver
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/uio.h>
#include <linux/aio.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include <linux/ioctl.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

#define AM_MODULE 1
#define STREAM_MODULE 0

/**
 * intel_sst_open_cntrl - opens a handle to driver
 *
 * @i_node:	inode structure
 * @file_ptr:pointer to file
 *
 * This function is called by OS when a user space component
 * tries to get a driver handle to /dev/intel_sst_control.
 * Only one handle at a time will be allowed
 * This is for control operations only
 */
int intel_sst_open_cntrl(struct inode *i_node, struct file *file_ptr)
{
	unsigned int retval;

	/* audio manager open */
	mutex_lock(&sst_drv_ctx->stream_lock);
	retval = intel_sst_check_device();
	if (retval) {
		mutex_unlock(&sst_drv_ctx->stream_lock);
		return retval;
	}

	if (sst_drv_ctx->am_cnt < MAX_AM_HANDLES) {
		sst_drv_ctx->am_cnt++;
		pr_debug("AM handle opened...\n");
		
		if(file_ptr)
			file_ptr->private_data = NULL;

	} else {
		retval = -EACCES;
	}

	mutex_unlock(&sst_drv_ctx->stream_lock);

	return retval;
}


int intel_sst_release_cntrl(struct inode *i_node, struct file *file_ptr)
{
	/* audio manager close */
	mutex_lock(&sst_drv_ctx->stream_lock);
	sst_drv_ctx->am_cnt--;
	pm_runtime_put(&sst_drv_ctx->pci->dev);
	mutex_unlock(&sst_drv_ctx->stream_lock);
	pr_debug("AM handle closed\n");

	return 0;
}

/**
 * sst_create_algo_ipc - create ipc msg for algorithm parameters
 *
 * @algo_params: Algorithm parameters
 * @msg: post msg pointer
 *
 * This function is called to create ipc msg
 * For copying the mailbox data the function returns offset in bytes to mailbox
 * memory where the mailbox data should be copied after msg header
 */
static int sst_create_algo_ipc_byt(struct snd_ppp_params *algo_params,
					struct ipc_post **msg)
{
	u32 ipc_msg_size = sizeof(u32) + sizeof(*algo_params);

	if (ipc_msg_size > SST_MAILBOX_SIZE)
		return -ENOMEM;
	if (sst_create_large_msg(msg))
		return -ENOMEM;

	/* Prepare the IPC message (ipc_post->ipc_header_byt) */
	sst_fill_header_byt(&(*msg)->byt_header, IPC_IA_ALG_PARAMS, 1, algo_params->str_id);

	/* Set the size of mailbox in IPC header "data" field (ipc_post->ipc_header_byt)*/
	(*msg)->byt_header.p.header_low_payload.part.data = sizeof(*algo_params) + sizeof(u32);

	/* Prepare mailbox content (driver copy) - IPC low header, write to mailbox will happen in sst_post_message_byt()*/
	memcpy( (u8 *) (*msg)->mailbox_data, (u8 *) &((*msg)->byt_header.p.header_low_payload.full), sizeof(u32));

	/* Prepare mailbox content (driver copy) - <struct snd_ppp_params>, write to mailbox will happen in sst_post_message_byt()*/
	memcpy( (u8 *) (*msg)->mailbox_data + sizeof(u32), (u8 *) algo_params, sizeof(*algo_params));

	return 0;
}


/**
 * sst_send_algo_ipc - send ipc msg for algorithm parameters
 *
 * @msg: post msg pointer
 *
 * This function is called to send ipc msg
 */
static int sst_send_algo_ipc(struct ipc_post **msg)
{
	unsigned long irq_flags;

	sst_drv_ctx->ppp_params_blk.condition = false;
	sst_drv_ctx->ppp_params_blk.ret_code = 0;
	sst_drv_ctx->ppp_params_blk.on = true;
	sst_drv_ctx->ppp_params_blk.data = NULL;
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&(*msg)->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);

	return sst_wait_timeout(sst_drv_ctx, &sst_drv_ctx->ppp_params_blk);
}

/**
 * intel_sst_ioctl_dsp - receives the device ioctl's
 *
 * @cmd:Ioctl cmd
 * @arg:data
 *
 * This function is called when a user space component
 * sends a DSP Ioctl to SST driver
 */
static long intel_sst_ioctl_dsp(unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct snd_ppp_params algo_params;
	struct snd_ppp_params *algo_params_copied;
	struct ipc_post *msg;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNDRV_SST_SET_ALGO):
		if (copy_from_user(&algo_params, (void __user *)arg,
							sizeof(algo_params)))
			return -EFAULT;

		pr_debug("Algo ID %d Str id %d Enable %d Size %d\n",
			algo_params.algo_id, algo_params.str_id,
			algo_params.enable, algo_params.size);


#ifdef SST_DRV_BYT
		algo_params.operation = 0;   /* Set Algorithm */
		retval = sst_create_algo_ipc_byt(&algo_params, &msg);
		if (retval)
			break;
#else
		algo_params.reserved = 0;
		retval = sst_create_algo_ipc(&algo_params, &msg);
		if (retval < 0)
			break;
		if (copy_from_user(msg->mailbox_data + retval,
				algo_params.params, algo_params.size)) {
			kfree(msg);
			return -EFAULT;
		}
#endif /* SST_DRV_BYT */

		retval = sst_send_algo_ipc(&msg);
		if (retval) {
			pr_err("Error in sst_set_algo = %d\n", retval);
			retval = -EIO;
		}
		break;

	case _IOC_NR(SNDRV_SST_GET_ALGO):
		if (copy_from_user(&algo_params, (void __user *)arg,
							sizeof(algo_params)))
			return -EFAULT;
		pr_debug("Algo ID %d Str id %d Enable %d Size %d\n",
			algo_params.algo_id, algo_params.str_id,
			algo_params.enable, algo_params.size);

		algo_params.operation = 1;   /* Get Algorithm */
		retval = sst_create_algo_ipc_byt(&algo_params, &msg);
		if (retval)
			break;

		retval = sst_send_algo_ipc(&msg);
		if (retval) {
			pr_err("Error in sst_get_algo = %d\n", retval);
			retval = -EIO;
			break;
		}


		/* For BYT, "ppp_params_blk.data" = pointer to <struct snd_ppp_params>
		 * Please refer to sst_process_reply_byt() in intel_sst_ipc.c
		 */
		algo_params_copied = (struct snd_ppp_params *)
					sst_drv_ctx->ppp_params_blk.data;

		if (algo_params_copied->size > algo_params.size) {
			pr_err("mem insufficient to copy\n");
			retval = -EMSGSIZE;
			goto free_mem;
		} else {
			char __user *tmp;

			if (copy_to_user( &algo_params.param_block,
					  &algo_params_copied->param_block,
					  algo_params_copied->size )) 
			{
				retval = -EFAULT;
				goto free_mem;
			}

			/*Update <struct snd_ppp_params>->size to user copy*/
			tmp = (char __user *)arg + offsetof(
					struct snd_ppp_params, size);
			if (copy_to_user(tmp, &algo_params_copied->size,
						 sizeof(u32))) {
				retval = -EFAULT;
				goto free_mem;
			}
			/*Update <struct snd_ppp_params>->enable to user copy*/
			tmp = (char __user *)arg + offsetof(
					struct snd_ppp_params, enable);
			if (copy_to_user(tmp, &algo_params_copied->enable,
						 sizeof(u8))) {
				retval = -EFAULT;
				goto free_mem;
			}

		}
free_mem:
#ifdef SST_DRV_BYT
		/* For BYT, there is no <struct snd_ppp_params>->params and kzalloc(GFP_KERNEL)
		 * is not done in sst_process_reply_byt() in intel_sst_ipc.c
		 */
#else
		kfree(algo_params_copied->params);
#endif
		kfree(algo_params_copied);
		break;
	}

	return retval;
}


static int sst_ioctl_tuning_params(unsigned int cmd, unsigned long arg)
{
	struct snd_sst_tuning_params tuning_params;
	struct ipc_post *msg;
#ifndef SST_DRV_BYT
	unsigned long address;
#endif
	if (copy_from_user(&tuning_params, (void __user *)arg, sizeof(tuning_params)))
		return -EFAULT;
	pr_debug("sst: Parameter %d, Stream %d, Size %d\n", tuning_params.type,
			tuning_params.str_id, tuning_params.size);
	if (sst_create_large_msg(&msg))
		return -ENOMEM;

#ifndef SST_DRV_BYT
	address = (unsigned long)tuning_params.addr;
#endif

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNDRV_SST_TUNING_PARAMS):
		sst_fill_header_byt(&msg->byt_header, IPC_IA_TUNING_PARAMS, 1, 
			tuning_params.str_id);

		break;
	case _IOC_NR(SNDRV_SST_SET_RUNTIME_PARAMS):
#ifdef SST_DRV_BYT
		sst_fill_header_byt(&msg->byt_header, IPC_IA_SET_RUNTIME_PARAMS, 1, 
			tuning_params.str_id);
#else
		sst_fill_header(&msg->header, IPC_IA_SET_RUNTIME_PARAMS, 1,
				tuning_params.str_id);
#endif /* SST_DRV_BYT */
		break;
	}

#ifdef SST_DRV_BYT
	msg->byt_header.p.header_low_payload.part.data = sizeof(tuning_params) + sizeof(u32);
	memcpy( (u8 *) msg->mailbox_data, (u8 *) &(msg->byt_header.p.header_low_payload.full), sizeof(u32));
	memcpy( (u8 *) msg->mailbox_data + sizeof(u32), (u8 *) &tuning_params, sizeof(tuning_params));
#else
	msg->header.part.data = sizeof(u32) + sizeof(tuning_params) + tuning_params.size;
	memcpy(msg->mailbox_data, &msg->header.full, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), &tuning_params, sizeof(tuning_params));

	/* driver doesn't need to send address, so overwrite addr with data */
	if (copy_from_user(msg->mailbox_data + sizeof(u32)
				+ sizeof(tuning_params) - sizeof(tuning_params.addr),
			(void __user *)address, tuning_params.size)) {
		kfree(msg->mailbox_data);
		kfree(msg);
		return -EFAULT;
	}
#endif /* SST_DRV_BYT */

	return sst_send_algo_ipc(&msg);
}


/**
 * intel_sst_ioctl - receives the device ioctl's
 * @file_ptr:pointer to file
 * @cmd:Ioctl cmd
 * @arg:data
 *
 * This function is called by OS when a user space component
 * sends an Ioctl to SST driver
 */
long intel_sst_ioctl(struct file *file_ptr, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct ioctl_pvt_data *data = NULL;
	int str_id = 0, minor = 0;

	data = file_ptr->private_data;
	if (data) {
		minor = 0;
		str_id = data->str_id;
	} else
		minor = 1;

	if (sst_drv_ctx->sst_state != SST_FW_RUNNING)
		return -EBUSY;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNDRV_SST_DRIVER_INFO): {
		struct snd_sst_driver_info info;

		pr_debug("SNDRV_SST_DRIVER_INFO received\n");
		info.version = SST_VERSION_NUM;
		/* hard coding, shud get sumhow later */
		info.active_pcm_streams = sst_drv_ctx->stream_cnt;
		info.max_pcm_streams = MAX_ACTIVE_STREAM;
		info.max_enc_streams = MAX_ENC_STREAM;
		if (copy_to_user((void __user *)arg, &info,
				sizeof(info)))
			retval = -EFAULT;
		break;
	}
	case _IOC_NR(SNDRV_SST_FW_INFO): {
		/*<struct snd_sst_fw_info> in intel_sst_ioctl.h is BYT specific*/
		struct snd_sst_fw_info *fw_info;

		pr_debug("SNDRV_SST_FW_INFO received\n");

		fw_info = kzalloc(sizeof(*fw_info), GFP_ATOMIC);
		if (!fw_info) {
			retval = -ENOMEM;
			break;
		}

		retval = sst_get_fw_info(fw_info);

		if (retval) {
			retval = -EIO;
			kfree(fw_info);
			break;
		}
		if (copy_to_user((struct snd_sst_fw_info __user *)arg,
				fw_info, sizeof(*fw_info))) {
			kfree(fw_info);
			retval = -EFAULT;
			break;
		}
		kfree(fw_info);
		break;
	}
	case _IOC_NR(SNDRV_SST_GET_ALGO):
	case _IOC_NR(SNDRV_SST_SET_ALGO):
		if (minor != AM_MODULE) {
			retval = -EBADRQC;
			break;
		}
		retval = intel_sst_ioctl_dsp(cmd, arg);
		break;

#ifdef SST_DRV_BYT
	/* BYT LPE FW yet to support SET_RUNTIME_PARAMS, so we just return -E_INVAL for now.*/
	case _IOC_NR(SNDRV_SST_SET_RUNTIME_PARAMS):
		pr_err("intel_sst_ioctl: BYT LPE FW does not support SET_RUNTIME_PARAMS yet !!! Error=%d\n", retval);
		retval = -EINVAL;
		break;

	case _IOC_NR(SNDRV_SST_TUNING_PARAMS):
#else
	case _IOC_NR(SNDRV_SST_TUNING_PARAMS):
	case _IOC_NR(SNDRV_SST_SET_RUNTIME_PARAMS):
#endif
		if (minor != AM_MODULE) {
			retval = -EBADRQC;
			break;
		}
		retval = sst_ioctl_tuning_params(cmd, arg);
		break;

	default:
		retval = -EINVAL;
	}
	pr_debug("intel_sst_ioctl:complete ret code = %d\n", retval);

	return retval;
}

