/*
 *  intel_sst_pvt.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10	Intel Corp
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
 *  This driver exposes the audio engine functionalities to the ALSA
 *	and middleware.
 *
 *  This file contains all private functions
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <sound/asound.h>
#include <sound/pcm.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

#define SST_EXCE_DUMP_BASE	0xFFFF2c00
#define SST_EXCE_DUMP_WORD 4
#define SST_EXCE_DUMP_LEN 32
#define SST_EXCE_DUMP_SIZE ((SST_EXCE_DUMP_LEN)*(SST_EXCE_DUMP_WORD))

/*
 * sst_get_block_stream - get a new block stream
 *
 * @sst_drv_ctx: Driver context structure
 *
 * This function assigns a block for the calls that dont have stream context yet
 * the blocks are used for waiting on Firmware's response for any operation
 * Should be called with stream lock held
 */
int sst_get_block_stream(struct intel_sst_drv *sst_drv_ctx)
{
	int i;

	for (i = 0; i < MAX_ACTIVE_STREAM; i++) {
		if (sst_drv_ctx->alloc_block[i].sst_id == BLOCK_UNINIT) {
			sst_drv_ctx->alloc_block[i].ops_block.condition = false;
			sst_drv_ctx->alloc_block[i].ops_block.ret_code = 0;
			sst_drv_ctx->alloc_block[i].sst_id = 0;
			break;
		}
	}
	if (i == MAX_ACTIVE_STREAM) {
		pr_err("max alloc_stream reached\n");
		i = -EBUSY; /* active stream limit reached */
	}

	return i;
}

/*
 * sst_wait_interruptible - wait on event
 *
 * @sst_drv_ctx: Driver context
 * @block: Driver block to wait on
 *
 * This function waits without a timeout (and is interruptable) for a
 * given block event
 */
int sst_wait_interruptible(struct intel_sst_drv *sst_drv_ctx,
				struct sst_block *block)
{
	int retval = 0;

	if (!wait_event_interruptible(sst_drv_ctx->wait_queue,
				block->condition)) {
		/* event wake */
		if (block->ret_code < 0) {
			pr_err("stream failed %d\n", block->ret_code);
			retval = -EBUSY;
		} else {
			pr_debug("event up\n");
			retval = 0;
		}
	} else {
		pr_err("signal interrupted\n");
		retval = -EINTR;
	}

	return retval;

}

void dump_sst_shim(struct intel_sst_drv *sst)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&sst->ipc_spin_lock, irq_flags);
	pr_err("audio shim registers:\n"
		"CSR: %llx\n"
		"PISR: %llx\n"
		"PIMR: %llx\n"
		"ISRX: %llx\n"
		"ISRD: %llx\n"
		"IMRX: %llx\n"
		"IMRD: %llx\n"
		"IPCX: %llx\n"
		"IPCD: %llx\n"
		"ISRSC: %llx\n"
		"ISRLPESC: %llx\n"
		"IMRSC: %llx\n"
		"IMRLPESC: %llx\n"
		"IPCSC: %llx\n"
		"IPCLPESC: %llx\n"
		"CLKCTL: %llx\n"
		"CSR2: %llx\n",
		sst_shim_read64(sst->shim, SST_CSR),
		sst_shim_read64(sst->shim, SST_PISR),
		sst_shim_read64(sst->shim, SST_PIMR),
		sst_shim_read64(sst->shim, SST_ISRX),
		sst_shim_read64(sst->shim, SST_ISRD),
		sst_shim_read64(sst->shim, SST_IMRX),
		sst_shim_read64(sst->shim, SST_IMRD),
		sst_shim_read64(sst->shim, SST_IPCX),
		sst_shim_read64(sst->shim, SST_IPCD),
		sst_shim_read64(sst->shim, SST_ISRSC),
		sst_shim_read64(sst->shim, SST_ISRLPESC),
		sst_shim_read64(sst->shim, SST_IMRSC),
		sst_shim_read64(sst->shim, SST_IMRLPESC),
		sst_shim_read64(sst->shim, SST_IPCSC),
		sst_shim_read64(sst->shim, SST_IPCLPESC),
		sst_shim_read64(sst->shim, SST_CLKCTL),
		sst_shim_read64(sst->shim, SST_CSR2));
	spin_unlock_irqrestore(&sst->ipc_spin_lock, irq_flags);
}

void reset_sst_shim(struct intel_sst_drv *sst)
{
	pr_err("Resetting few Shim registers\n");
	sst_shim_write64(sst->shim, SST_IPCX, 0x0);
	sst_shim_write64(sst->shim, SST_IPCD, 0x0);
	sst_shim_write64(sst->shim, SST_ISRX, 0x0);
	sst_shim_write64(sst->shim, SST_ISRD, 0x0);
	sst_shim_write64(sst->shim, SST_IPCSC, 0x0);
	sst_shim_write64(sst->shim, SST_IPCLPESC, 0x0);
	sst_shim_write64(sst->shim, SST_ISRSC, 0x0);
	sst_shim_write64(sst->shim, SST_ISRLPESC, 0x0);
	sst_shim_write64(sst->shim, SST_PISR, 0x0);
}

static void dump_sst_crash_area(void)
{
	void __iomem *fw_dump_area;
	u32 dump_word;
	u16 i;

	/* dump the firmware SRAM where the exception details are stored */
	fw_dump_area = ioremap_nocache(SST_EXCE_DUMP_BASE, SST_EXCE_DUMP_SIZE);
	pr_err("Firmware exception dump begins:\n");
	pr_err("Exception start signature:%#x\n", readl(fw_dump_area + SST_EXCE_DUMP_WORD));
	pr_err("EXCCAUSE:\t\t\t%#x\n", readl(fw_dump_area + SST_EXCE_DUMP_WORD*2));
	pr_err("EXCVADDR:\t\t\t%#x\n", readl(fw_dump_area + (SST_EXCE_DUMP_WORD*3)));
	pr_err("Firmware additional data:\n");

	/* dump remaining FW debug data */
	for (i = 1; i < (SST_EXCE_DUMP_LEN-4+1); i++) {
		dump_word = readl(fw_dump_area + (SST_EXCE_DUMP_WORD*3)
						+ (i*SST_EXCE_DUMP_WORD));
		pr_err("Data[%d]=%#x\n", i, dump_word);
	}
	iounmap(fw_dump_area);
	pr_err("Firmware exception dump ends\n");
}

#if 0 /* unused codes */
static void sst_stream_recovery(struct intel_sst_drv *sst)
{
	struct stream_info *str_info;
	u8 i;

	for (i = 1; i <= sst->max_streams; i++) {
		pr_err("Audio: Stream %d, state %d\n", i, sst->streams[i].status);
		if (sst->streams[i].status != STREAM_UN_INIT) {
			str_info = &sst_drv_ctx->streams[i];
			if (str_info->pcm_substream)
				snd_pcm_stop(str_info->pcm_substream, SNDRV_PCM_STATE_SETUP);
		}
	}
}
#endif

static void sst_do_recovery(struct intel_sst_drv *sst)  
{
	struct ipc_post *m, *_m;
	unsigned long irq_flags;
	/*
	 * setting firmware state as uninit so that the firmware will get
	 * redownloaded on next request.This is because firmare not responding
	 * for 1 sec is equalant to some unrecoverable error of FW.
	 */
	pr_err("Audio: Intel SST engine encountered an unrecoverable error\n");
	pr_err("Audio: trying to reset the dsp now\n");
	mutex_lock(&sst->sst_lock);
	sst->sst_state = SST_UN_INIT;
#if 0   /* Not supported in BYT */
	sst_stream_recovery(sst);
#endif
	mutex_unlock(&sst->sst_lock);
	dump_stack();
	dump_sst_shim(sst);
#if 0   /* Not supported in BYT */
	reset_sst_shim(sst);
#endif
	dump_sst_crash_area();

	spin_lock_irqsave(&sst->ipc_spin_lock, irq_flags);
	if (list_empty(&sst->ipc_dispatch_list))
		pr_err("List is Empty\n");
	spin_unlock_irqrestore(&sst->ipc_spin_lock, irq_flags);
	list_for_each_entry_safe(m, _m, &sst->ipc_dispatch_list, node) {
		pr_err("pending msg header %#x\n", m->header.full);
#if 0   /* Not supported in BYT */
		list_del(&m->node);
		kfree(m->mailbox_data);
		kfree(m);
#endif
	}
}

/*
 * sst_wait_timeout - wait on event for timeout
 *
 * @sst_drv_ctx: Driver context
 * @block: Driver block to wait on
 *
 * This function waits with a timeout value (and is not interruptible) on a
 * given block event
 */
int sst_wait_timeout(struct intel_sst_drv *sst_drv_ctx, struct sst_block *block)
{
	int retval = 0;

	/* NOTE:
	Observed that FW processes the alloc msg and replies even
	before the alloc thread has finished execution */
	pr_debug("sst: waiting for condition %x\n",
		       block->condition);
	if (wait_event_timeout(sst_drv_ctx->wait_queue,
				block->condition,
				msecs_to_jiffies(SST_BLOCK_TIMEOUT))) {
		/* event wake */
		pr_debug("sst: Event wake %x\n", block->condition);
		pr_debug("sst: message ret: %d\n", block->ret_code);
		retval = block->ret_code;
	} else {
		block->on = false;

		pr_err("sst: Wait timed-out %x\n", block->condition);
		sst_do_recovery(sst_drv_ctx);
		/* settign firmware state as uninit so that the
		firmware will get redownloaded on next request
		this is because firmare not responding for 5 sec
		is equalant to some unrecoverable error of FW */
		retval = -EBUSY;
	}

	return retval;
}

/*
 * sst_create_large_msg - create a large IPC message
 *
 * @arg: ipc message
 *
 * this function allocates structures to send a large message to the firmware
 */
int sst_create_large_msg(struct ipc_post **arg)
{
	struct ipc_post *msg;

	msg = kzalloc(sizeof(struct ipc_post), GFP_ATOMIC);
	if (!msg) {
		pr_err("kzalloc msg failed\n");
		return -ENOMEM;
	}

	msg->mailbox_data = kzalloc(SST_MAILBOX_SIZE, GFP_ATOMIC);
	if (!msg->mailbox_data) {
		kfree(msg);
		pr_err("kzalloc mailbox_data failed");
		return -ENOMEM;
	}
	*arg = msg;

	return 0;
}

/*
 * sst_create_short_msg - create a short IPC message
 *
 * @arg: ipc message
 *
 * this function allocates structures to send a short message to the firmware
 */
int sst_create_short_msg(struct ipc_post **arg)
{
	struct ipc_post *msg;

	msg = kzalloc(sizeof(*msg), GFP_ATOMIC);
	if (!msg) {
		pr_err("kzalloc msg failed\n");
		return -ENOMEM;
	}
	msg->mailbox_data = NULL;
	*arg = msg;

	return 0;
}

/*
 * sst_clean_stream - clean the stream context
 *
 * @stream: stream structure
 *
 * this function resets the stream contexts
 * should be called in free
 */
void sst_clean_stream(struct stream_info *stream)
{
	stream->status = STREAM_UN_INIT;
	stream->prev = STREAM_UN_INIT;
	mutex_lock(&stream->lock);
	stream->cumm_bytes = 0;
	mutex_unlock(&stream->lock);
}

/*
 * sst_wake_up_alloc_block - wake up waiting block
 *
 * @sst_drv_ctx: Driver context
 * @sst_id: stream id
 * @status: status of wakeup
 * @data: data pointer of wakeup
 *
 * This function wakes up a sleeping block event based on the response
 */
void sst_wake_up_alloc_block(struct intel_sst_drv *sst_drv_ctx,
		u8 sst_id, int status, void *data)
{
	int i;

	/* Unblock with retval code */
	for (i = 0; i < MAX_ACTIVE_STREAM; i++) {
		if (sst_id == sst_drv_ctx->alloc_block[i].sst_id) {
			sst_drv_ctx->alloc_block[i].ops_block.condition = true;
			sst_drv_ctx->alloc_block[i].ops_block.ret_code = status;
			sst_drv_ctx->alloc_block[i].ops_block.data = data;
			wake_up(&sst_drv_ctx->wait_queue);
			break;
		}
	}
}

