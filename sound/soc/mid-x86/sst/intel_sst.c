/*
 *  intel_sst.c - Intel SST Driver for audio engine
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
 *	 and middleware.
 *
 *  This file contains all init functions
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <linux/delay.h>
#include <asm/intel-mid.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"
#include <linux/intel_mid_dma.h> /*PCI container*/

#include <linux/module.h>

MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_AUTHOR("Dharageswari R <dharageswari.r@intel.com>");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
MODULE_DESCRIPTION("Intel (R) SST(R) Audio Engine Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SST_DRIVER_VERSION);


extern int intel_mid_i2s_probe(struct pci_dev *, const struct pci_device_id *);
extern void intel_mid_i2s_remove(struct pci_dev *);

extern spinlock_t imrx_spin_lock;
extern unsigned long imrx_irq_flags;

struct intel_sst_drv *sst_drv_ctx;
static struct mutex drv_ctx_lock;
struct class *sst_class;
static const struct file_operations intel_sst_fops_cntrl = {
	.owner = THIS_MODULE,
	.open = intel_sst_open_cntrl,
	.release = intel_sst_release_cntrl,
	.unlocked_ioctl = intel_sst_ioctl,
};

static struct miscdevice lpe_ctrl = {
	.minor = MISC_DYNAMIC_MINOR,/* dynamic allocation */
	.name = "intel_sst_ctrl",/* /dev/intel_sst_ctrl */
	.fops = &intel_sst_fops_cntrl
};

#ifdef SST_DRV_BYT
static irqreturn_t intel_sst_irq_thread_byt(int irq, void *context)
{
	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;
	union ipc_header_byt header;
	struct stream_info *stream;
	unsigned int size = 0, str_id;

	/* Read LPE->IA message request from IPCD register*/
	header.f = sst_shim_read64(drv->shim, SST_IPCD);
	
	if (header.p.header_low_payload.part.msg_id == IPC_SST_PERIOD_ELAPSED) 
	{
		sst_drv_ctx->ops->clear_interrupt();
		str_id = header.p.header_low_payload.part.str_id;
		stream = &sst_drv_ctx->streams[str_id];
		if (stream->period_elapsed)
			stream->period_elapsed(stream->pcm_substream);
		return IRQ_HANDLED;
	}

	if (header.p.header_low_payload.part.large)
	{
		size = header.p.header_low_payload.part.data;
 	}

	if (header.p.header_low_payload.part.msg_id & REPLY_MSG) 
	{
		sst_drv_ctx->ipc_process_msg.byt_header = header;
		memcpy_fromio(sst_drv_ctx->ipc_process_msg.mailbox,
			drv->mailbox + SST_MAILBOX_RCV, size);
		queue_work(sst_drv_ctx->process_msg_wq,
				&sst_drv_ctx->ipc_process_msg.wq);
	} else {
		sst_drv_ctx->ipc_process_reply.byt_header = header;
		memcpy_fromio(sst_drv_ctx->ipc_process_reply.mailbox,
			drv->mailbox + SST_MAILBOX_RCV, size);
		queue_work(sst_drv_ctx->process_reply_wq,
				&sst_drv_ctx->ipc_process_reply.wq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t intel_sst_interrupt_byt(int irq, void *context)
{
	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;

	union isrx_reg_byt isrx;
	union imrx_reg_byt imrx;
	union ipcx_reg_byt ipcx;

	irqreturn_t retval = IRQ_NONE;

	/* Do not handle interrupt in suspended state */
	if (drv->sst_state == SST_SUSPENDED)
		return IRQ_NONE;

	/* Interrupt arrived, check src */
	isrx.full = sst_shim_read64(drv->shim, SST_ISRX);

	
	if ( isrx.part.lpe_ia_ipc_done_status ) {
		/* LPE has responded to IA's message-request */
		/* IA->LPE request done */
		/* IA to clear IPCX's done bit  */
		spin_lock(&sst_drv_ctx->ipc_spin_lock);

		ipcx.full = sst_shim_read64(drv->shim, SST_IPCX);
		ipcx.part.lpe_ia_done = 0;
		sst_shim_write64(drv->shim, SST_IPCX, ipcx.full);

		/* write 1 to clear ISRX's status register */;
		isrx.part.lpe_ia_ipc_done_status = 1;
		sst_shim_write64(drv->shim, SST_ISRX, isrx.full);

		spin_unlock(&sst_drv_ctx->ipc_spin_lock);

		queue_work(sst_drv_ctx->post_msg_wq,
			&sst_drv_ctx->ipc_post_msg.wq);
		retval = IRQ_HANDLED;
	}
	if ( isrx.part.lpe_ia_ipc_request_status ) {
		/* LPE has sent message-request to IA */
		/* IA to mask out ISRX's lpe_ia_ipc_request_status  */
		/* by using IMRX's lpe_ia_ipc_request_mask */

		spin_lock(&sst_drv_ctx->ipc_spin_lock);
		spin_lock_irqsave(&imrx_spin_lock, imrx_irq_flags);

		imrx.full = sst_shim_read64(drv->shim, SST_IMRX);
		imrx.part.lpe_ia_ipc_request_mask = 1;
		sst_shim_write64(drv->shim, SST_IMRX, imrx.full);

		spin_unlock_irqrestore(&imrx_spin_lock, imrx_irq_flags);
		spin_unlock(&sst_drv_ctx->ipc_spin_lock);
		retval = IRQ_WAKE_THREAD;
	}
	return retval;
}
#endif /*SST_DRV_BYT*/

struct intel_sst_ops byt_ops = {
	.interrupt = intel_sst_interrupt_byt,
	.irq_thread = intel_sst_irq_thread_byt,
	.clear_interrupt = intel_sst_clear_intr_byt,
	.start = sst_start_byt,
	.reset = intel_sst_reset_dsp_byt,
	.post_message = sst_post_message_byt,
	.sync_post_message = sst_sync_post_message_byt,
	.process_message = sst_process_message_byt,
	.process_reply = sst_process_reply_byt,
};

static int sst_driver_ops(unsigned int pci_id)
{
	if(pci_id == SST_BYT_PCI_ID){
		sst_drv_ctx->tstamp =  SST_TIME_STAMP_BYT;
		sst_drv_ctx->ops = &byt_ops;

		return 0;
	}
	else{	
		pr_err("SST Driver capabilities missing for pci_id: %x", pci_id);
	}	
	return -EINVAL;
}

static int intel_sst_byt_irq_enable(struct pci_dev *dev)
{
	struct io_apic_irq_attr irq_attr;
    /*
	 *  Host IPC	IRQ29 (used for LPE_to_IA IPC interrupt)
	 */
	dev->irq=BYT_IPC_IRQ;
	irq_attr.ioapic = mp_find_ioapic(BYT_IPC_IRQ);
	if(irq_attr.ioapic<0){
 			printk(KERN_ERR "ERROR: No IOAPIC for IRQ=%d DID=0x%x \n",dev->irq, dev->device);
 			return irq_attr.ioapic;
 	}
	irq_attr.ioapic_pin = BYT_IPC_IRQ;
	irq_attr.trigger = 1; /* level */
	irq_attr.polarity = 1; /* active low */
	io_apic_set_pci_routing(&dev->dev, BYT_IPC_IRQ, &irq_attr);
	return 0;
}

/*
* intel_sst_probe - PCI probe function
*
* @pci:	PCI device structure
* @pci_id: PCI device ID structure
*
* This function is called by OS when a device is found
* This enables the device, interrupt etc
*/
int intel_sst_probe(struct pci_dev *pdev,
			const struct pci_device_id *pci_id)
{
	int i, ret = 0;
	struct intel_sst_ops *ops;
	u32 base,alloc_len;

	pr_debug("Probe for DID %x\n", pdev->device);
        mutex_init(&drv_ctx_lock);
        mutex_lock(&drv_ctx_lock);
	if (sst_drv_ctx) {
		pr_err("Only one sst handle is supported\n");
		mutex_unlock(&drv_ctx_lock);
		return -EBUSY;
	}

	sst_drv_ctx = kzalloc(sizeof(*sst_drv_ctx), GFP_KERNEL);
	if (!sst_drv_ctx) {
		pr_err("malloc fail\n");
		mutex_unlock(&drv_ctx_lock);
		return -ENOMEM;
	}
	mutex_unlock(&drv_ctx_lock);

	sst_drv_ctx->pci_id = pdev->device;

	if (0 != sst_driver_ops(sst_drv_ctx->pci_id)) {
		kfree(sst_drv_ctx);
		kfree(pdev);
		return -EINVAL;
	}
	mutex_init(&sst_drv_ctx->stream_lock);
	mutex_init(&sst_drv_ctx->sst_lock);
	mutex_init(&sst_drv_ctx->mixer_ctrl_lock);

	sst_drv_ctx->stream_cnt = 0;
	sst_drv_ctx->am_cnt = 0;
	sst_drv_ctx->pb_streams = 0;
	sst_drv_ctx->cp_streams = 0;
	sst_drv_ctx->fw = NULL;
	sst_drv_ctx->fw_in_mem = NULL;
	ops = sst_drv_ctx->ops;

	INIT_LIST_HEAD(&sst_drv_ctx->ipc_dispatch_list);
	INIT_WORK(&sst_drv_ctx->ipc_post_msg.wq, ops->post_message);
	INIT_WORK(&sst_drv_ctx->ipc_process_msg.wq, ops->process_message);
	INIT_WORK(&sst_drv_ctx->ipc_process_reply.wq, ops->process_reply);
	init_waitqueue_head(&sst_drv_ctx->wait_queue);
	sst_drv_ctx->mad_wq = create_singlethread_workqueue("sst_mad_wq");
	if (!sst_drv_ctx->mad_wq)
		goto do_free_drv_ctx;
	sst_drv_ctx->post_msg_wq =
		create_singlethread_workqueue("sst_post_msg_wq");
	if (!sst_drv_ctx->post_msg_wq)
		goto free_mad_wq;
	sst_drv_ctx->process_msg_wq =
		create_singlethread_workqueue("sst_process_msg_wq");
	if (!sst_drv_ctx->process_msg_wq)
		goto free_post_msg_wq;
	sst_drv_ctx->process_reply_wq =
		create_singlethread_workqueue("sst_proces_reply_wq");
	if (!sst_drv_ctx->process_reply_wq)
		goto free_process_msg_wq;

	for (i = 0; i < MAX_ACTIVE_STREAM; i++) {
		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		sst_drv_ctx->alloc_block[i].ops_block.condition = false;
	}
	spin_lock_init(&sst_drv_ctx->ipc_spin_lock);
	sst_drv_ctx->max_streams = MAX_ACTIVE_STREAM;
	pr_debug("Got drv data max stream %u\n", sst_drv_ctx->max_streams);
	for (i = 1; i <= sst_drv_ctx->max_streams; i++) {
		struct stream_info *stream = &sst_drv_ctx->streams[i];
		mutex_init(&stream->lock);
	}

	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		sst_drv_ctx->device_input_mixer = SST_STREAM_DEVICE_IHF
							| SST_INPUT_STREAM_PCM;
	}

	/* Init the device */
   	if(sst_drv_ctx->pci_id == SST_BYT_PCI_ID)
    		ret = intel_sst_byt_irq_enable(pdev);
	else
		ret = pci_enable_device(pdev);
	if (ret) {
		pr_err("device can't be enabled\n");
		goto do_free_mem;
	}

	sst_drv_ctx->pci = pci_dev_get(pdev);
#ifndef SSP_DRIVER_ON
	ret = pci_request_regions(pdev, SST_DRV_NAME);
	if (ret)
		goto do_disable_device;
#endif
	/* map registers */
	if(sst_drv_ctx->pci_id == SST_BYT_PCI_ID) {

		sst_drv_ctx->ddr = NULL;
		pr_debug("BYT sst: DDR Ptr %p\n", sst_drv_ctx->ddr);
	}

	/* reading from PCI CONFIG for 1MB DDR cache allocation */
	pci_bus_read_config_dword(pdev->bus, pdev->devfn,
		SST_BYT_DDR_CACHE_PHY_ADDR_OFFSET, &base);
	pci_bus_read_config_dword(pdev->bus, pdev->devfn,
		SST_BYT_DDR_CACHE_ALLOC_SIZE_OFFSET, &alloc_len);
	if( base==0 || base % 0x20000000 ) {
		pr_err("addr %x is not 512M alligned\n",base);
		goto do_release_regions;
	}
	if( 0x00100000 != alloc_len) {
		pr_err("length %x is not 1MB\n",alloc_len);
		goto do_release_regions;
	}
	/* SST Extended FW 1MB allocation */
	sst_drv_ctx->fw_ext_phy_add = base;
	sst_drv_ctx->fw_ext_base = ioremap(sst_drv_ctx->fw_ext_phy_add,
		0x100000);
	if(!sst_drv_ctx->fw_ext_base)
		goto do_release_regions;

	/* SST Shim */
	sst_drv_ctx->shim_phy_add = pci_resource_start(pdev, 0);
	sst_drv_ctx->lpe_base        =  sst_drv_ctx->shim_phy_add;
              
	sst_drv_ctx->shim = ioremap(sst_drv_ctx->lpe_base + SST_SHIM_START_ADDRESS,
		(SST_SHIM_END_ADDRESS-SST_SHIM_START_ADDRESS)+1);
	
	if (!sst_drv_ctx->shim)
		goto do_unmap_fw_ext;
	pr_debug("SST Shim Ptr %p\n", sst_drv_ctx->shim);

	/* Shared SRAM */
	sst_drv_ctx->mailbox = ioremap(sst_drv_ctx->lpe_base+ SST_MAILBOX_START_ADDRESS,
		(SST_MAILBOX_END_ADDRESS- SST_MAILBOX_START_ADDRESS)+1);
	
	if (!sst_drv_ctx->mailbox)
		goto do_unmap_shim;
	pr_debug("SRAM Ptr %p\n", sst_drv_ctx->mailbox);

	/* IRAM */
	sst_drv_ctx->iram_base = sst_drv_ctx->lpe_base +SST_ICCM_START_ADDRESS;
	sst_drv_ctx->iram_end = sst_drv_ctx->lpe_base +SST_ICCM_START_ADDRESS+ 
		(SST_ICCM_END_ADDRESS-SST_ICCM_START_ADDRESS)+1;
	sst_drv_ctx->iram = ioremap(sst_drv_ctx->lpe_base+ SST_ICCM_START_ADDRESS,
		(SST_ICCM_END_ADDRESS-SST_ICCM_START_ADDRESS)+1);
	if (!sst_drv_ctx->iram)
		goto do_unmap_sram;
	pr_debug("IRAM Ptr %p\n", sst_drv_ctx->iram);

	/* DRAM */
	sst_drv_ctx->dram_base = sst_drv_ctx->lpe_base+ SST_DCCM_START_ADDRESS;
	sst_drv_ctx->dram_end = sst_drv_ctx->lpe_base+ SST_DCCM_START_ADDRESS+
		(SST_DCCM_END_ADDRESS-SST_DCCM_START_ADDRESS)+1;
	sst_drv_ctx->dram =ioremap(sst_drv_ctx->lpe_base+ SST_DCCM_START_ADDRESS,
		(SST_DCCM_END_ADDRESS-SST_DCCM_START_ADDRESS)+1);
	
	if (!sst_drv_ctx->dram)
		goto do_unmap_iram;
	pr_debug("DRAM Ptr %p\n", sst_drv_ctx->dram);
	/* Save the physical address in the first 4 bytes of the mailbox */
	memcpy(sst_drv_ctx->mailbox, &(sst_drv_ctx->fw_ext_phy_add), sizeof(u32));

	sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);

	/* Register the ISR */
	ret = request_threaded_irq(pdev->irq, sst_drv_ctx->ops->interrupt,
		sst_drv_ctx->ops->irq_thread, IRQF_SHARED, SST_DRV_NAME,
		sst_drv_ctx);
	if (ret)
		goto do_unmap_dram;

	pr_debug("Registered IRQ 0x%x\n", pdev->irq);

	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		ret = intel_sst_register_compress(sst_drv_ctx);
		if (ret) {
			pr_err("couldn't register compress device\n");
			goto do_free_irq;
		}
	}
	/*Register LPE Control as misc driver*/
	ret = misc_register(&lpe_ctrl);
	if (ret) {
		pr_err("couldn't register control device\n");
		goto do_free_irq;
	}

	if ((sst_drv_ctx->pci_id == SST_MFLD_PCI_ID) ||
			(sst_drv_ctx->pci_id == SST_CLV_PCI_ID)) {
		u32 csr;
		u32 csr2;
		u32 clkctl;

		/*allocate mem for fw context save during suspend*/
		sst_drv_ctx->fw_cntx = kzalloc(FW_CONTEXT_MEM, GFP_KERNEL);
		if (!sst_drv_ctx->fw_cntx) {
			ret = -ENOMEM;
			goto do_free_misc;
		}
		/*setting zero as that is valid mem to restore*/
		sst_drv_ctx->fw_cntx_size = 0;

		/*set lpe start clock and ram size*/
		csr = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
		csr |= 0x30000;
		/*make sure clksel set to OSC for SSP0,1 (default)*/
		csr &= 0xFFFFFFF3;
		sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr);

		/*set clock output enable for SSP0,1,3*/
		clkctl = sst_shim_read(sst_drv_ctx->shim, SST_CLKCTL);
		if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
			clkctl |= (0x7 << 16);
		else
			clkctl |= ((1<<16)|(1<<17));
		sst_shim_write(sst_drv_ctx->shim, SST_CLKCTL, clkctl);

		/* set SSP0 & SSP1 disable DMA Finish*/
		csr2 = sst_shim_read(sst_drv_ctx->shim, SST_CSR2);
		/*set SSP3 disable DMA finsh for SSSP3 */
		csr2 |= BIT(1)|BIT(2);
		sst_shim_write(sst_drv_ctx->shim, SST_CSR2, csr2);
	}

	/* GPIO_PIN 12,13,74,75 needs to be configured in
	 * ALT_FUNC_2 mode for SSP3 IOs
	 */

	pci_set_drvdata(pdev, sst_drv_ctx);
	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID) {
		pm_runtime_allow(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
	}
	register_sst(&pdev->dev);
	/*sst_debugfs_init(sst_drv_ctx);*/
	sst_drv_ctx->qos = kzalloc(sizeof(struct pm_qos_request),
				GFP_KERNEL);
	if (!sst_drv_ctx->qos)
		goto do_free_misc;
	pm_qos_add_request(sst_drv_ctx->qos, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
	pr_info("%s successfully done!\n", __func__);

	return ret;

do_free_misc:
	misc_deregister(&lpe_ctrl);
do_free_irq:
	free_irq(pdev->irq, sst_drv_ctx);
do_unmap_dram:
	iounmap(sst_drv_ctx->dram);
do_unmap_iram:
	iounmap(sst_drv_ctx->iram);
do_unmap_sram:
	iounmap(sst_drv_ctx->mailbox);
do_unmap_shim:
	iounmap(sst_drv_ctx->shim);
do_unmap_fw_ext:
	iounmap(sst_drv_ctx->fw_ext_base);
do_release_regions:
	pci_release_regions(pdev);
#ifndef SSP_DRIVER_ON
do_disable_device:
#endif
	pci_disable_device(pdev);
do_free_mem:
	destroy_workqueue(sst_drv_ctx->process_reply_wq);
free_process_msg_wq:
	destroy_workqueue(sst_drv_ctx->process_msg_wq);
free_post_msg_wq:
	destroy_workqueue(sst_drv_ctx->post_msg_wq);
free_mad_wq:
	destroy_workqueue(sst_drv_ctx->mad_wq);
do_free_drv_ctx:
	kfree(sst_drv_ctx);
	sst_drv_ctx = NULL;
	kfree(pdev);

	pr_err("Probe failed with %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(intel_sst_probe);

/**
* intel_sst_remove - PCI remove function
*
* @pci:	PCI device structure
*
* This function is called by OS when a device is unloaded
* This frees the interrupt etc
*/
static void __exit intel_sst_remove(struct pci_dev *pci)
{
	/*sst_debugfs_exit(sst_drv_ctx);*/
	pm_runtime_get_noresume(&pci->dev);
	pm_runtime_forbid(&pci->dev);
	unregister_sst(&pci->dev);
	pci_dev_put(sst_drv_ctx->pci);
	sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
	misc_deregister(&lpe_ctrl);
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		intel_sst_remove_compress(sst_drv_ctx);
	free_irq(pci->irq, sst_drv_ctx);
	iounmap(sst_drv_ctx->dram);
	iounmap(sst_drv_ctx->iram);
	iounmap(sst_drv_ctx->mailbox);
	iounmap(sst_drv_ctx->shim);
	kfree(sst_drv_ctx->fw_cntx);
#ifndef SST_DRV_BYT
	kfree(sst_drv_ctx->runtime_param.param.addr);
#endif /* SST_DRV_BYT */
	flush_scheduled_work();
	destroy_workqueue(sst_drv_ctx->process_reply_wq);
	destroy_workqueue(sst_drv_ctx->process_msg_wq);
	destroy_workqueue(sst_drv_ctx->post_msg_wq);
	destroy_workqueue(sst_drv_ctx->mad_wq);
	release_firmware(sst_drv_ctx->fw);
	pm_qos_remove_request(sst_drv_ctx->qos);
	kfree(sst_drv_ctx->qos);
	sst_drv_ctx->fw = NULL;
	kfree(sst_drv_ctx->fw_sg_list.src);
	kfree(sst_drv_ctx->fw_sg_list.dst);
	sst_drv_ctx->fw_sg_list.list_len = 0;
	kfree(sst_drv_ctx->fw_in_mem);
	sst_drv_ctx->fw_in_mem = NULL;
	kfree(sst_drv_ctx);
	sst_drv_ctx = NULL;
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);

/*	intel_mid_i2s_remove(pci); */

}

#ifdef SST_DRV_BYT_FW_CONTEXT_RESTORATION
static void sst_save_dsp_context(void)
{
	struct snd_sst_ctxt_params fw_context;
	unsigned int pvt_id;
	struct ipc_post *msg = NULL;
	unsigned long irq_flags;

	/*check cpu type*/
	if (sst_drv_ctx->pci_id == SST_MRST_PCI_ID)
		return;
	/*not supported for rest*/
	if (sst_drv_ctx->sst_state != SST_FW_RUNNING) {
		pr_debug("fw not running no context save ...\n");
		return;
	}

	/*send msg to fw*/
	if (sst_create_large_msg(&msg))
		return;
	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	sst_drv_ctx->alloc_block[0].sst_id = pvt_id;
	sst_drv_ctx->alloc_block[0].ops_block.condition = false;
	sst_drv_ctx->alloc_block[0].ops_block.on = true;

#ifdef SST_DRV_BYT
	/* Prepare the IPC message (ipc_post->ipc_header_byt) -------------------- [OK]*/
	sst_fill_header_byt(&msg->byt_header, IPC_IA_GET_FW_CTXT, 1, pvt_id);

	/* Set the size of mailbox in IPC header "data" field (ipc_post->ipc_header_byt)  -------------------- [OK] */
	msg->byt_header.p.header_low_payload.part.data = sizeof(fw_context) + sizeof(u32);

	/* ToDo: Currently LPE FW built with _MY_WIN_FW_ flag which
	 * ToDo: IPC_IA_GET_FW_CTXT & IPC_IA_SET_FW_CTXT are not functional yet....
	 * ToDO: So, we just set all zero in its structure fills.
	 */
	fw_context.num_entries = 0;
	fw_context.rsrvd = 0;
	fw_context.ring_buf_info[0].addr = 0;
	fw_context.ring_buf_info[0].size = 0;

	/* Prepare mailbox content (driver copy) - IPC low header, write to mailbox will happen in sst_post_message_byt() -------------------- [OK] */
	memcpy( (u8 *) msg->mailbox_data, (u8 *) &(msg->byt_header.p.header_low_payload.full), sizeof(u32));

	/* Prepare mailbox content (driver copy) - <struct snd_sst_ctxt_params>, write to mailbox will happen in sst_post_message_byt() -------------------- [OK]*/
	memcpy( (u8 *) msg->mailbox_data + sizeof(u32), (u8 *) &fw_context, sizeof(fw_context));
#else
	sst_fill_header(&msg->header, IPC_IA_GET_FW_CTXT, 1, pvt_id);
	msg->header.part.data = sizeof(fw_context) + sizeof(u32);
	fw_context.address = virt_to_phys((void *)sst_drv_ctx->fw_cntx);
	fw_context.size = FW_CONTEXT_MEM;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32),
				&fw_context, sizeof(fw_context));
#endif /*SST_DRV_BYT*/

	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	/*wait for reply*/
	if (sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[0].ops_block))
		pr_err("sst: err fw context save timeout  ...\n");
	sst_drv_ctx->alloc_block[0].sst_id = BLOCK_UNINIT;
	pr_debug("fw context saved  ...\n");

	return;
}
#endif /* SST_DRV_BYT_FW_CONTEXT_RESTORATION */

/*
 * The runtime_suspend/resume is pretty much similar to the legacy
 * suspend/resume with the noted exception below: The PCI core takes care of
 * taking the system through D3hot and restoring it back to D0 and so there is
 * no need to duplicate that here.
 */
static int intel_sst_runtime_suspend(struct device *dev)
{
#ifdef SST_DRV_BYT
	union csr_reg_byt csr; 
#else
	union config_status_reg csr;
#endif /* SST_DRV_BYT */

	pr_debug("runtime_suspend called\n");
	if (sst_drv_ctx->sst_state == SST_SUSPENDED) {
		pr_err("System already in Suspended state");
		return 0;
	}

#ifdef SST_DRV_BYT_FW_CONTEXT_RESTORATION
	/*save fw context*/
	sst_save_dsp_context();
#endif 

	/*Assert RESET on LPE Processor*/
#ifdef SST_DRV_BYT
	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	sst_drv_ctx->csr_value = csr.full;
	csr.full = csr.full | 0x4;       /* For BYT, we use stall-bit*/
#else
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	sst_drv_ctx->csr_value = csr.full;
	csr.full = csr.full | 0x2;
#endif /* SST_DRV_BYT */

	/* Move the SST state to Suspended */
	mutex_lock(&sst_drv_ctx->sst_lock);
	sst_drv_ctx->sst_state = SST_SUSPENDED;
#ifdef SST_DRV_BYT
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);
#else
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
#endif /* SST_DRV_BYT */
	mutex_unlock(&sst_drv_ctx->sst_lock);


	flush_workqueue(sst_drv_ctx->post_msg_wq);
	flush_workqueue(sst_drv_ctx->process_msg_wq);
	flush_workqueue(sst_drv_ctx->process_reply_wq);

	return 0;
}

static int intel_sst_runtime_resume(struct device *dev)
{
#ifdef SST_DRV_BYT
	u64 csr;
#else
	u32 csr;
#endif /* SST_DRV_BYT */

	pr_debug("runtime_resume called\n");
	if (sst_drv_ctx->sst_state != SST_SUSPENDED) {
		pr_err("SST is not in suspended state\n");
		return 0;
	}
#ifdef SST_DRV_BYT
	csr = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
#else
	csr = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
#endif /* SST_DRV_BYT */

#ifdef SST_DRV_BYT
	/*
	 * For BYT LPE FW, to restore the csr_value,
	 * we need to 
 	 */
	csr |= (sst_drv_ctx->csr_value & ~(0x4));
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr);
#else
	/*
	 * To restore the csr_value after S0ix and S3 states.
	 * The value 0x30000 is to enable LPE dram high and low addresses.
	 * Reference:
	 * Penwell Audio Voice Module HAS 1.61 Section - 13.12.1 -
	 * CSR - Configuration and Status Register.
	 */
	csr |= (sst_drv_ctx->csr_value | 0x30000);
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr);
#endif /* SST_DRV_BYT*/
	/* GPIO_PIN 12,13,74,75 needs to be configured in
	 * ALT_FUNC_2 mode for SSP3 IOs
	 */

	sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);

	return 0;
}

static int intel_sst_runtime_idle(struct device *dev)
{
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		return -EBUSY;
	pr_debug("runtime_idle called\n");
	if (!sst_drv_ctx->am_cnt && sst_drv_ctx->sst_state != SST_UN_INIT) {
		pm_schedule_suspend(dev, SST_SUSPEND_DELAY);
		return -EBUSY;
	} else {
		return 0;
	}

	return -EBUSY;

}

static const struct dev_pm_ops intel_sst_pm = {
	.suspend = intel_sst_runtime_suspend,
	.resume = intel_sst_runtime_resume,
	.runtime_suspend = intel_sst_runtime_suspend,
	.runtime_resume = intel_sst_runtime_resume,
	.runtime_idle = intel_sst_runtime_idle,
};

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_sst_ids) = {
	{ PCI_VDEVICE(INTEL, SST_MRST_PCI_ID), 3},
	{ PCI_VDEVICE(INTEL, SST_MFLD_PCI_ID), 5},		
	{ PCI_VDEVICE(INTEL, SST_BYT_PCI_ID), 3},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_sst_ids);

#ifndef SSP_DRIVER_ON
static struct pci_driver driver = {
	.name = SST_DRV_NAME,
	.id_table = intel_sst_ids,
	.probe = intel_sst_probe,
	.remove = intel_sst_remove,
	.driver = {
		.pm = &intel_sst_pm,
	},
};
#endif

