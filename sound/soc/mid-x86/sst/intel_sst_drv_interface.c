/*
 *  intel_sst_interface.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com)
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

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <sound/intel_sst_ioctl.h>
#include <sound/pcm.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

#ifdef SST_DRV_BYT_FW_CONTEXT_RESTORATION
static void sst_restore_fw_context(void)
{
	struct snd_sst_ctxt_params fw_context;
	struct ipc_post *msg = NULL;
	int retval = 0;
	unsigned long irq_flags;

	pr_debug("restore_fw_context\n");
	/*nothing to restore*/
	if (!sst_drv_ctx->fw_cntx_size)
		return;
	pr_debug("restoring context......\n");
	/*send msg to fw*/
	if (sst_create_large_msg(&msg))
		return;

	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_CTXT_RESTORE);
	
	sst_fill_header_byt(&msg->byt_header, IPC_IA_SET_FW_CTXT, 1, 0);

	sst_drv_ctx->alloc_block[0].sst_id = FW_DWNL_ID;
	sst_drv_ctx->alloc_block[0].ops_block.condition = false;
	sst_drv_ctx->alloc_block[0].ops_block.ret_code = 0;
	sst_drv_ctx->alloc_block[0].ops_block.on = true;

#ifdef SST_DRV_BYT
	/* Set the size of mailbox in IPC header "data" field (ipc_post->ipc_header_byt) */
	msg->byt_header.p.header_low_payload.part.data = sizeof(fw_context) + sizeof(u32);

	/* ToDo: Currently LPE FW built with _MY_WIN_FW_ flag which
	 * ToDo: IPC_IA_GET_FW_CTXT & IPC_IA_SET_FW_CTXT are not functional yet....
	 * ToDO: So, we just set all zero in its structure fills.
	 */
	fw_context.num_entries = 0;
	fw_context.rsrvd = 0;
	fw_context.ring_buf_info[0].addr = 0;
	fw_context.ring_buf_info[0].size = 0;

	/* Prepare mailbox content (driver copy) - IPC low header,
     * write to mailbox will happen in sst_post_message_byt() 
     *  -------------------- [OK] */
	memcpy( (u8 *) msg->mailbox_data, (u8 *) &(msg->byt_header.p.header_low_payload.full), sizeof(u32));

	/* Prepare mailbox content (driver copy) - <struct snd_sst_ctxt_params>,
     * write to mailbox will happen in sst_post_message_byt() 
     */ 
	memcpy( (u8 *) msg->mailbox_data + sizeof(u32), (u8 *) &fw_context, sizeof(fw_context));
#else
	msg->header.part.data = sizeof(fw_context) + sizeof(u32);
	fw_context.address = (u32)sst_drv_ctx->fw_cntx_handle;
	fw_context.size = sst_drv_ctx->fw_cntx_size;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32),
				&fw_context, sizeof(fw_context));
#endif /* SST_DRV_BYT */


	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[0].ops_block);
	sst_drv_ctx->alloc_block[0].sst_id = BLOCK_UNINIT;

	if (retval)
		pr_err("sst_restore_fw_context..timeout!\n");

	return;
}
#endif /* SST_DRV_BYT_FW_CONTEXT_RESTORATION */

/*
 * sst_download_fw - download the audio firmware to DSP
 *
 * This function is called when the FW needs to be downloaded to SST DSP engine
 */
int sst_download_fw(void)
{
	int retval;

	if (sst_drv_ctx->sst_state != SST_START_INIT)
		return -EAGAIN;
	if (!sst_drv_ctx->fw_in_mem) {
		retval = sst_request_fw();
		if (retval)
			return retval;
		pr_debug("sst_request_fw success\n");
	}
	sst_drv_ctx->alloc_block[0].sst_id = FW_DWNL_ID;
	sst_drv_ctx->alloc_block[0].ops_block.condition = false;
	/* Prevent C-states beyond C6 */
	pm_qos_update_request(sst_drv_ctx->qos, CSTATE_EXIT_LATENCY_S0i1 - 1);
	retval = sst_load_fw(sst_drv_ctx->fw_in_mem, NULL);
	/* Re-enable Deeper C-states beyond C6 */
	pm_qos_update_request(sst_drv_ctx->qos, PM_QOS_DEFAULT_VALUE);
	if (retval)
		goto end_restore;

		retval = sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[0].ops_block);
		if (retval) {
			pr_err("fw download failed %d\n" , retval);
			/* assume FW d/l failed due to timeout*/
			retval = -EBUSY;
		}


end_restore:
	sst_drv_ctx->alloc_block[0].sst_id = BLOCK_UNINIT;
	if (retval)
		return retval;

#ifdef SST_DRV_BYT_FW_CONTEXT_RESTORATION
	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID)
		sst_restore_fw_context();
#endif

	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_RUNNING);

	return retval;
}

void free_stream_context(unsigned int str_id)
{
	struct stream_info *stream;
	stream = get_stream_info(str_id);

	if (stream) {
		/* str_id is valid, so stream is alloacted */
		if (sst_free_stream(str_id))
			sst_clean_stream(&sst_drv_ctx->streams[str_id]);
		if (stream->ops == STREAM_OPS_PLAYBACK)
			sst_drv_ctx->pb_streams--;
		else if (stream->ops == STREAM_OPS_CAPTURE)
			sst_drv_ctx->cp_streams--;
	}
}

void sst_send_lpe_mixer_algo_params(void)  
{
	struct snd_ppp_params algo_param;      
	unsigned int input_mixer, stream_device_id;
	int retval;
#ifndef SST_DRV_BYT
	struct snd_ppp_mixer_params mixer_param; 
#endif

	retval = intel_sst_check_device();
	if (retval)
		return;

	mutex_lock(&sst_drv_ctx->mixer_ctrl_lock);
	input_mixer = (sst_drv_ctx->device_input_mixer)
				& SST_INPUT_STREAM_MIXED;
	pr_debug("Input Mixer settings %d", input_mixer);
	stream_device_id = sst_drv_ctx->device_input_mixer - input_mixer;

	algo_param.algo_id = SST_CODEC_MIXER;
	algo_param.str_id = stream_device_id;
	algo_param.enable = 1;
	algo_param.size = sizeof(algo_param);

#ifdef SST_DRV_BYT
/* Set param operation */
/* BYT LPE FW has "operation" field - ref FW ipc_IA_ppp_param
 * in ipc_media.h
 */
	algo_param.operation = 0;  

	/* Refer to intel_sst_ioctl.h, the <struct snd_ppp_params> carries param_block */
	algo_param.param_block.type = SST_ALGO_PARAM_MIXER_STREAM_CFG;
	algo_param.param_block.size = sizeof(input_mixer);
	algo_param.param_block.params[0] = input_mixer;	
#else	
	algo_param.reserved = 0;

	mixer_param.type = SST_ALGO_PARAM_MIXER_STREAM_CFG;
	mixer_param.input_stream_bitmap = input_mixer;
	mixer_param.size = sizeof(input_mixer);
	algo_param.params = &mixer_param;
#endif /* SST_DRV_BYT */

	mutex_unlock(&sst_drv_ctx->mixer_ctrl_lock);
	pr_err("setting pp param\n");
	pr_debug("Algo ID %d Str id %d Enable %d Size %d\n",
			algo_param.algo_id, algo_param.str_id,
			algo_param.enable, algo_param.size);
	sst_send_algo_param(&algo_param);
	pm_runtime_put(&sst_drv_ctx->pci->dev);
}


/*
 * sst_get_stream_allocated - this function gets a stream allocated with
 * the given params
 *
 * @str_param : stream params
 * @lib_dnld : pointer to pointer of lib downlaod struct
 *
 * This creates new stream id for a stream, in case lib is to be downloaded to
 * DSP, it downloads that
 */
int sst_get_stream_allocated(struct snd_sst_params *str_param,
		struct snd_sst_lib_download **lib_dnld)
{
	int retval, str_id;
	struct stream_info *str_info;

	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		pr_debug("Sending LPE mixer algo Params\n");
		sst_send_lpe_mixer_algo_params();
	}
	retval = sst_alloc_stream((char *) str_param);

	if (retval < 0) {
		pr_err("sst_alloc_stream failed %d\n", retval);
		return retval;
	}
	pr_debug("Stream allocated %d\n", retval);
	str_id = retval;
	str_info = &sst_drv_ctx->streams[str_id];
	/* Block the call for reply */
	retval = sst_wait_timeout(sst_drv_ctx, &str_info->ctrl_blk);
	if ((retval != 0) || (str_info->ctrl_blk.ret_code != 0)) {
		pr_err("sst: FW alloc failed retval %d, ret_code %d\n",\
				retval, str_info->ctrl_blk.ret_code);
		if (retval == SST_ERR_STREAM_IN_USE) {
			pr_err("sst:FW not in clean state, send free for:%d\n",
					str_id);
			sst_free_stream(str_id);
		}
		str_id = -str_info->ctrl_blk.ret_code; /*return error*/
		if (str_id == 0)
			str_id = retval; /*FW timed out*/
		*lib_dnld = str_info->ctrl_blk.data;
		sst_clean_stream(str_info);
	} else
		pr_debug("FW Stream allocated success\n");

	return str_id; /*will ret either error (in above if) or correct str id*/
}

/*
 * sst_get_sfreq - this function returns the frequency of the stream
 *
 * @str_param : stream params
 */
int sst_get_sfreq(struct snd_sst_params *str_param)
{
	switch (str_param->codec) {
	case SST_CODEC_TYPE_PCM:
		return str_param->sparams.uc.pcm_params.sfreq;
	case SST_CODEC_TYPE_AAC:
		return str_param->sparams.uc.aac_params.externalsr;
	case SST_CODEC_TYPE_MP3:
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * sst_get_sfreq - this function returns the frequency of the stream
 *
 * @str_param : stream params
 */
int sst_get_num_channel(struct snd_sst_params *str_param)
{
	switch (str_param->codec) {
	case SST_CODEC_TYPE_PCM:
		return str_param->sparams.uc.pcm_params.num_chan;
	case SST_CODEC_TYPE_MP3:
		return str_param->sparams.uc.mp3_params.num_chan;
	case SST_CODEC_TYPE_AAC:
		return str_param->sparams.uc.aac_params.num_chan;
	default:
		return -EINVAL;
	}
}

int sst_get_wdsize(struct snd_sst_params *str_param)
{
	switch (str_param->codec) {
	case SST_CODEC_TYPE_PCM:
		return str_param->sparams.uc.pcm_params.pcm_wd_sz;
	case SST_CODEC_TYPE_MP3:
		return str_param->sparams.uc.mp3_params.pcm_wd_sz;
	case SST_CODEC_TYPE_AAC:
		return str_param->sparams.uc.aac_params.pcm_wd_sz;

/* Currently unsupported by BYT */
#ifndef SST_DRV_BYT
	case SST_CODEC_TYPE_WMA9:
		return str_param->sparams.uc.wma_params.pcm_wd_sz;
#endif

	default:
		return -EINVAL;
	}
}

/*
 * sst_get_stream - this function prepares for stream allocation
 *
 * @str_param : stream param
 */
int sst_get_stream(struct snd_sst_params *str_param)
{
	int i, retval;
	struct stream_info *str_info;
	struct snd_sst_lib_download *lib_dnld;

	/* stream is not allocated, we are allocating */
	retval = sst_get_stream_allocated(str_param, &lib_dnld);
	if (retval == -(SST_LIB_ERR_LIB_DNLD_REQUIRED)) {
		/* codec download is required */
		struct snd_sst_alloc_response *response;

		pr_debug("Codec is required.... trying that\n");
		if (lib_dnld == NULL) {
			pr_err("lib download null!!! abort\n");
			return -EIO;
		}
		i = sst_get_block_stream(sst_drv_ctx);
		if (i < 0) {
			pr_err("invalid value for number of stream\n ");
			kfree(lib_dnld);
			return i;
		}
		response = sst_drv_ctx->alloc_block[i].ops_block.data;
		pr_debug("alloc block allocated = %d\n", i);

		retval = sst_load_library(lib_dnld, str_param->ops);
		kfree(lib_dnld);

		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		if (!retval) {
			pr_debug("codec was downloaded successfully\n");

			retval = sst_get_stream_allocated(str_param, &lib_dnld);
			if (retval <= 0) {
				retval = -EIO;
				goto err;
			}

			pr_debug("Alloc done stream id %d\n", retval);
		} else {
			pr_debug("codec download failed\n");
			retval = -EIO;
			goto err;
		}
	} else if  (retval <= 0) {
		retval = -EIO;
		goto err;
	}
	/*else
		set_port_params(str_param, str_param->ops);*/

	/* store sampling freq */
	str_info = &sst_drv_ctx->streams[retval];
	str_info->sfreq = sst_get_sfreq(str_param);

	/* power on the analog, if reqd */
	if (str_param->ops == STREAM_OPS_PLAYBACK) {
		/*Only if the playback is MP3 - Send a message*/
		sst_drv_ctx->pb_streams++;
	} else if (str_param->ops == STREAM_OPS_CAPTURE) {

		/*Send a messageif not sent already*/
		sst_drv_ctx->cp_streams++;
	}
err:
	return retval;
}

/**
* intel_sst_check_device - checks SST device
*
* This utility function checks the state of SST device and downlaods FW if
* not done, or resumes the device if suspended
*/
int intel_sst_check_device(void)
{
	int retval = 0;

	pm_runtime_get_sync(&sst_drv_ctx->pci->dev);
	mutex_lock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->sst_state == SST_UN_INIT) {
		sst_drv_ctx->sst_state = SST_START_INIT;
		mutex_unlock(&sst_drv_ctx->sst_lock);
		/* FW is not downloaded */
		pr_debug("DSP Downloading FW now...\n");
		retval = sst_download_fw();
		if (retval) {
			pr_err("FW download fail %x\n", retval);
			sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
			pm_runtime_put(&sst_drv_ctx->pci->dev);
			return retval;
		}
	} else
		mutex_unlock(&sst_drv_ctx->sst_lock);

	mutex_lock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->sst_state != SST_FW_RUNNING) {
		mutex_unlock(&sst_drv_ctx->sst_lock);
		pm_runtime_put(&sst_drv_ctx->pci->dev);
		return -EAGAIN;
	}
	mutex_unlock(&sst_drv_ctx->sst_lock);

	return retval;
}

void sst_process_mad_ops(struct work_struct *work)
{
	struct mad_ops_wq *mad_ops =
			container_of(work, struct mad_ops_wq, wq);
	int retval = 0;

	switch (mad_ops->control_op) {
	case SST_SND_PAUSE:
		retval = sst_pause_stream(mad_ops->stream_id);
		break;
	case SST_SND_RESUME:
		retval = sst_resume_stream(mad_ops->stream_id);
		break;
	default:
		pr_err(" wrong control_ops reported\n");
	}
	if (retval)
		pr_err("%s(): op: %d, retval: %d\n",
				__func__, mad_ops->control_op, retval);
	kfree(mad_ops);

	return;
}

void sst_fill_compressed_slot(unsigned int device_type)
{
	if (device_type == SND_SST_DEVICE_HEADSET)
		sst_drv_ctx->compressed_slot = 0x03;
	else if (device_type == SND_SST_DEVICE_IHF)
		sst_drv_ctx->compressed_slot = 0x0C;
}

/*
 * sst_open_pcm_stream - Open PCM interface
 *
 * @str_param: parameters of pcm stream
 *
 * This function is called by MID sound card driver to open
 * a new pcm interface
 */
static int sst_open_pcm_stream(struct snd_sst_params *str_param)
{
	struct stream_info *str_info;
	int retval;

	if (!str_param)
		return -EINVAL;

	pr_debug("open_pcm, doing rtpm_get\n");

	retval = intel_sst_check_device();

	if (retval)
		return retval;

	retval = sst_get_stream(str_param);
	if (retval > 0) {
		sst_drv_ctx->stream_cnt++;
		str_info = &sst_drv_ctx->streams[retval];
	} else
		pm_runtime_put(&sst_drv_ctx->pci->dev);

	return retval;
}

/*
 * sst_close_pcm_stream - Close PCM interface
 *
 * @str_id: stream id to be closed
 *
 * This function is called by MID sound card driver to close
 * an existing pcm interface
 */
static int sst_close_pcm_stream(unsigned int str_id)
{
	struct stream_info *stream;

	pr_debug("stream free called\n");
	stream = get_stream_info(str_id);
	if (!stream)
		return -EINVAL;
	free_stream_context(str_id);
	stream->pcm_substream = NULL;
	stream->status = STREAM_UN_INIT;
	stream->period_elapsed = NULL;
	sst_drv_ctx->stream_cnt--;
	pr_debug("will call runtime put now\n");
	pm_runtime_put(&sst_drv_ctx->pci->dev);

	return 0;
}

int sst_send_sync_msg(int ipc, int str_id)
{
	struct ipc_post *msg = NULL;

	if (sst_create_short_msg(&msg))
		return -ENOMEM;

	sst_fill_header_byt(&msg->byt_header, ipc, 0, str_id);

	return sst_drv_ctx->ops->sync_post_message(msg);
}

static inline int sst_calc_tstamp(struct pcm_stream_info *info,
		struct snd_pcm_substream *substream,
		struct snd_sst_tstamp *fw_tstamp)
{
	size_t delay_bytes, delay_frames;
	size_t buffer_sz;
	size_t pointer_bytes, pointer_samples;

	pr_debug("mrfld ring_buffer_counter %llx in bytes\n",
			fw_tstamp->ring_buffer_counter);
	pr_debug("mrfld hardware_counter %llx in bytes\n",
			 fw_tstamp->hardware_counter);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		delay_bytes = fw_tstamp->ring_buffer_counter -
					fw_tstamp->hardware_counter;
	else
		delay_bytes = fw_tstamp->hardware_counter -
					fw_tstamp->ring_buffer_counter;
	delay_frames = bytes_to_frames(substream->runtime, delay_bytes);
	buffer_sz = snd_pcm_lib_buffer_bytes(substream);
	pointer_bytes = ((size_t)fw_tstamp->ring_buffer_counter) % buffer_sz;
	pointer_samples = bytes_to_samples(substream->runtime, pointer_bytes);

	pr_debug("pcm delay %zd in bytes\n", delay_bytes);

	info->buffer_ptr = pointer_samples / substream->runtime->channels;

	info->pcm_delay = delay_frames / substream->runtime->channels;
	pr_debug("buffer ptr %llx pcm_delay rep: %llx\n",
			info->buffer_ptr, info->pcm_delay);

	return 0;
}

static int sst_read_timestamp(struct pcm_stream_info *info)
{
	struct snd_sst_tstamp fw_tstamp;

	struct stream_info *stream;
	struct snd_pcm_substream *substream;
	unsigned int str_id;

	str_id = info->str_id;
	stream = get_stream_info(str_id);
	if (!stream)
		return -EINVAL;

	if (!stream->pcm_substream)
		return -EINVAL;
	substream = stream->pcm_substream;

	memcpy_fromio(&fw_tstamp,
			((void *)(sst_drv_ctx->mailbox + sst_drv_ctx->tstamp)
				+ (str_id * sizeof(fw_tstamp))),
			sizeof(fw_tstamp));
		return sst_calc_tstamp(info, substream, &fw_tstamp);

}

/*
 * sst_device_control - Set Control params
 *
 * @cmd: control cmd to be set
 * @arg: command argument
 *
 * This function is called by MID sound card driver to set
 * SST/Sound card controls for an opened stream.
 * This is registered with MID driver
 */
static int sst_device_control(int cmd, void *arg)
{
	int retval = 0, str_id = 0;

	if (sst_drv_ctx->sst_state == SST_UN_INIT) {
		return 0;
	}
	switch (cmd) {
	case SST_SND_PAUSE:
	case SST_SND_RESUME: { 
		struct mad_ops_wq *work = kzalloc(sizeof(*work), GFP_ATOMIC);
		if (!work)
			return -ENOMEM;
		INIT_WORK(&work->wq, sst_process_mad_ops);
		work->control_op = cmd;
		work->stream_id = *(int *)arg;
		queue_work(sst_drv_ctx->mad_wq, &work->wq);
		break;
	}
	case SST_SND_START: {
		struct stream_info *str_info;
		int ipc;
		str_id = *(int *)arg;
		str_info = get_stream_info(str_id);
		if (!str_info)
			return -EINVAL;
		ipc = IPC_IA_START_STREAM;
		str_info->prev = str_info->status;
		str_info->status = STREAM_RUNNING;

		if (sst_drv_ctx->pci_id != SST_MFLD_PCI_ID)
			sst_start_stream(str_id);
		else
			retval = sst_send_sync_msg(ipc, str_id);

		break;
	}
	case SST_SND_DROP: {
		struct stream_info *str_info;
		int ipc;
		str_id = *(int *)arg;
		str_info = get_stream_info(str_id);
		if (!str_info)
			return -EINVAL;
		ipc = IPC_IA_DROP_STREAM;
		str_info->prev = STREAM_UN_INIT;
		str_info->status = STREAM_INIT;
		retval = sst_send_sync_msg(ipc, str_id);
		break;
	}
	case SST_SND_STREAM_INIT: {
		struct pcm_stream_info *str_info;
		struct stream_info *stream;

		pr_debug("stream init called\n");
		str_info = (struct pcm_stream_info *)arg;
		str_id = str_info->str_id;
		stream = get_stream_info(str_id);
		if (!stream) {
			retval = -EINVAL;
			break;
		}
		pr_debug("setting the period ptrs\n");
		stream->pcm_substream = str_info->mad_substream;
		stream->period_elapsed = str_info->period_elapsed;
		stream->sfreq = str_info->sfreq;
		stream->prev = stream->status;
		stream->status = STREAM_INIT;
		pr_debug("pcm_substream %p, period_elapsed %p, sfreq %d, status %d\n",
				stream->pcm_substream, stream->period_elapsed, stream->sfreq, stream->status);
		break;
	}

	case SST_SND_BUFFER_POINTER: {
		struct pcm_stream_info *stream_info;

		stream_info = (struct pcm_stream_info *)arg;
		retval = sst_read_timestamp(stream_info);
		pr_debug("pointer %llx, delay %llx\n",
			stream_info->buffer_ptr, stream_info->pcm_delay);
		break;
	}
	default:
		/* Illegal case */
		pr_warn("illegal req\n");
		return -EINVAL;
	}

	return retval;
}

/*
 * sst_copy_runtime_param - copy runtime params from src to dst
 *				 structure.
 *
 *@dst: destination runtime structure
 *@src: source runtime structure
 *
 * This helper function is called to copy the runtime parameter
 * structure.
*/
static int sst_copy_runtime_param(struct snd_sst_runtime_params *dst,
			struct snd_sst_runtime_params *src)
{
	dst->type = src->type;
	dst->str_id = src->str_id;
	dst->size = src->size;
#ifdef SST_DRV_BYT
/* BYT does not use the dynamic memory in this param*/
	dst->params[0] = src->params[0];
#else
	if (dst->addr) {
		pr_err("mem allocated in prev setting, use the same memory\n");
		return -EINVAL;
	}
	dst->addr = kzalloc(dst->size, GFP_KERNEL);
	if (!dst->addr)
		return -ENOMEM;
	memcpy(dst->addr, src->addr, dst->size);
#endif /* SST_DRV_BYT */

	return 0;
}
/*
 * sst_set_generic_params - Set generic params
 *
 * @cmd: control cmd to be set
 * @arg: command argument
 *
 * This function is called by MID sound card driver to configure
 * SST runtime params.
 */
static int sst_set_generic_params(enum sst_controls cmd, void *arg)
{
	int ret_val = 0;
	pr_debug("Enter:%s, cmd:%d\n", __func__, cmd);

	if (NULL == arg)
		return -EINVAL;

	switch (cmd) {
	case SST_SET_RUNTIME_PARAMS: {
		struct snd_sst_runtime_params *src;
		struct snd_sst_runtime_params *dst;

		src = (struct snd_sst_runtime_params *)arg;
		dst = &(sst_drv_ctx->runtime_param.param);
		ret_val = sst_copy_runtime_param(dst, src);
		break;
		}
	case SST_SET_ALGO_PARAMS: {
		unsigned int device_input_mixer = *((unsigned int *)arg);
		pr_debug("LPE mixer algo param set %x\n", device_input_mixer);
		mutex_lock(&sst_drv_ctx->mixer_ctrl_lock);
		sst_drv_ctx->device_input_mixer = device_input_mixer;
		mutex_unlock(&sst_drv_ctx->mixer_ctrl_lock);
		sst_send_lpe_mixer_algo_params();
		break;
	}
	default:
		pr_err("Invalid cmd request:%d\n", cmd);
		ret_val = -EINVAL;
	}

	return ret_val;
}

static struct sst_ops pcm_ops = {
	.open = sst_open_pcm_stream,
	.device_control = sst_device_control,
	.set_generic_params = sst_set_generic_params,
	.close = sst_close_pcm_stream,
};

static struct sst_device sst_dsp_device = {
	.name = "Intel(R) SST LPE",
	.dev = NULL,
	.ops = &pcm_ops,
};

/*
 * register_sst - function to register DSP
 *
 * This functions registers DSP with the platform driver
 */
int register_sst(struct device *dev)
{
	int ret_val;

	sst_dsp_device.dev = dev;
	ret_val = sst_register_dsp(&sst_dsp_device);
	if (ret_val)
		pr_err("Unable to register DSP with platform driver\n");

	return ret_val;
}

int unregister_sst(struct device *dev)
{
	return sst_unregister_dsp(&sst_dsp_device);
}
