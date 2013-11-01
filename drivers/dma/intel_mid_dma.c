/*
 *  intel_mid_dma.c - Intel Langwell DMA Drivers
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  The driver design is based on dw_dmac driver
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
 *
 */
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/intel_mid_dma.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "dmaengine.h"

#define MAX_CHAN	8 /*max ch across controllers*/

#include "intel_mid_dma_regs.h"

#define INTEL_MID_DMAC1_ID		0x0814
#define INTEL_MID_DMAC2_ID		0x0813
#define INTEL_MID_GP_DMAC2_ID	0x0827
#define INTEL_MFLD_DMAC1_ID		0x0830
#define LNW_PERIPHRAL_MASK_BASE		0xFFAE8008
#define LNW_PERIPHRAL_MASK_SIZE		0x20
#define LNW_PERIPHRAL_STATUS		0x0
#define LNW_PERIPHRAL_MASK		0x8

#ifdef PCI_ENCASE
	static struct lpe_device *pci;
#else
#define pci->dma_ctx pdev
#endif

DEFINE_SPINLOCK(imrx_spin_lock);
unsigned long imrx_irq_flags;

struct intel_mid_dma_probe_info {
	u8 max_chan;
	u8 ch_base;
	u32 block_size;
	u32 pimr_mask;
	u32 pimr_base;
	u8 dword_trf;
	u32 pimr_offset;
	struct intel_mid_dma_ops *pdma_ops;
};

#define INFO(_max_chan, _ch_base, _block_size, _pimr_mask,	\
		_pimr_base, _dword_trf, _pimr_offset, _pdma_ops)				\
	((kernel_ulong_t)&(struct intel_mid_dma_probe_info) {	\
		.max_chan = (_max_chan),			\
		.ch_base = (_ch_base),				\
		.block_size = (_block_size),			\
		.pimr_mask = (_pimr_mask),			\
		.pimr_base = (_pimr_base),			\
		.dword_trf = (_dword_trf),			\
		.pimr_offset = (_pimr_offset),			\
		.pdma_ops = (_pdma_ops)				\
	})

extern int to_load_bin_ssp(void);
/*****************************************************************************
Utility Functions*/
/**
 * get_ch_index	-	convert status to channel
 * @status: status mask
 * @base: dma ch base value
 *
 * Returns the channel index by checking the status bits.
 * If none of the bits in status are set, then returns -1.
 */
static int get_ch_index(int status, unsigned int base)
{
	int i;

	for (i = 0; i < MAX_CHAN; i++) {
		if (status & (1 << (i + base)))
			return i;
	}

	return -1;
}

static void dump_dma_reg(struct dma_chan *chan)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	struct middma_device	*mid = to_middma_device(chan->device);

	if (!mid->pimr_base)
		return;

	pr_debug("<<<<<<<<<<<< DMA Dump Start >>>>>>>>>>>>");
	pr_debug("DMA Dump for Channel id:%d & Chnl Base:%#x",
					midc->ch_id, (unsigned int)midc->ch_regs);
	/* dump common DMA registers */
	pr_debug("PIMR:\t%#x", readl(mid->mask_reg) - 8);
	pr_debug("ISRX:\t%#x", readl(mid->mask_reg));
	pr_debug("ISRD:\t%#x", readl(mid->mask_reg + 0x8));
	pr_debug("IMRX:\t%#x", readl(mid->mask_reg + 0x10));
	pr_debug("IMRD:\t%#x", readl(mid->mask_reg + 0x18));
	pr_debug("DMA_CHAN_EN:\t%#x", readl(midc->dma_base + DMA_CHAN_EN));
	pr_debug("DMA_CFG:\t%#x", readl(midc->dma_base + DMA_CFG));
	pr_debug("INTR_STATUS:\t%#x", readl(midc->dma_base + INTR_STATUS));
	pr_debug("MASK_TFR:\t%#x", readl(midc->dma_base + MASK_TFR));
	pr_debug("MASK_BLOCK:\t%#x", readl(midc->dma_base + MASK_BLOCK));
	pr_debug("MASK_ERR:\t%#x", readl(midc->dma_base + MASK_ERR));
	pr_debug("RAW_TFR:\t%#x", readl(midc->dma_base + RAW_TFR));
	pr_debug("RAW_BLOCK:\t%#x", readl(midc->dma_base + RAW_BLOCK));
	pr_debug("RAW_ERR:\t%#x", readl(midc->dma_base + RAW_ERR));
	pr_debug("STATUS_TFR:\t%#x", readl(midc->dma_base + STATUS_TFR));
	pr_debug("STATUS_BLOCK:\t%#x", readl(midc->dma_base + STATUS_BLOCK));
	pr_debug("STATUS_ERR:\t%#x", readl(midc->dma_base + STATUS_ERR));
	if (!mid->dword_trf) {
		pr_debug("FIFO_PARTITION0_LO:\t%#x",
				readl(midc->dma_base + FIFO_PARTITION0_LO));
		pr_debug("FIFO_PARTITION0_HI:\t%#x",
				readl(midc->dma_base + FIFO_PARTITION0_HI));
		pr_debug("FIFO_PARTITION1_LO:\t%#x",
				readl(midc->dma_base + FIFO_PARTITION1_LO));
		pr_debug("FIFO_PARTITION1_HI:\t%#x",
				readl(midc->dma_base + FIFO_PARTITION1_HI));
		pr_debug("CH_SAI_ERR:\t%#x", readl(midc->dma_base + CH_SAI_ERR));
	}

	/* dump channel specific registers */
	pr_debug("SAR:\t%#x", readl(midc->ch_regs + SAR));
	pr_debug("DAR:\t%#x", readl(midc->ch_regs + DAR));
	pr_debug("LLP:\t%#x", readl(midc->ch_regs + LLP));
	pr_debug("CTL_LOW:\t%#x", readl(midc->ch_regs + CTL_LOW));
	pr_debug("CTL_HIGH:\t%#x", readl(midc->ch_regs + CTL_HIGH));
	pr_debug("CFG_LOW:\t%#x", readl(midc->ch_regs + CFG_LOW));
	pr_debug("CFG_HIGH:\t%#x", readl(midc->ch_regs + CFG_HIGH));
	pr_debug("<<<<<<<<<<<< DMA Dump ends >>>>>>>>>>>>");
}

/**
 * get_block_ts	-	calculates dma transaction length
 * @len: dma transfer length
 * @tx_width: dma transfer src width
 * @block_size: dma controller max block size
 * @dword_trf: is transfer dword size aligned and needs the data transfer to
 *   be in terms of data items and not bytes
 *
 * Based on src width calculate the DMA trsaction length in data items
 * return data items or FFFF if exceeds max length for block
 */
static unsigned int get_block_ts(int len, int tx_width,
				int block_size, int dword_trf)
{
	int byte_width = 0, block_ts = 0;

	switch (tx_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		byte_width = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		byte_width = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
	default:
		byte_width = 4;
		break;
	}
	if (dword_trf)
		block_ts = len/byte_width;
	else
		block_ts = len;
	if (block_ts > block_size)
		block_ts = 0xFFFF;

	return block_ts;
}

/**
 * get_reg_width	-	computes the DMA sample width
 * @kernel_width: Kernel DMA slave bus width
 *
 * converts the DMA kernel slave bus width in the Intel DMA
 * bus width
 */
static int get_reg_width(enum dma_slave_buswidth kernel_width)
{
	int reg_width = -1;

	switch (kernel_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		reg_width = 0;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		reg_width = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		reg_width = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_UNDEFINED:
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
	default:
		pr_err("ERR_MDMA: get_reg_width unsupported reg width\n");
		break;
	}
	return reg_width;
}


/*****************************************************************************
DMAC1 interrupt Functions*/

/**
 * dmac1_mask_periphral_intr -	mask the periphral interrupt
 * @mid: dma device for which masking is required
 *
 * Masks the DMA periphral interrupt
 * this is valid for DMAC1 family controllers only
 * This controller should have periphral mask registers already mapped
 */
static void dmac1_mask_periphral_intr(struct middma_device *mid)
{
	u32 pimr;

	spin_lock_irqsave(&imrx_spin_lock, imrx_irq_flags);
	if (mid->pimr_mask) {
		pimr = readl(mid->mask_reg + mid->pimr_offset);
		pimr |= mid->pimr_mask;
		writel(pimr, mid->mask_reg + mid->pimr_offset);
	}
	spin_unlock_irqrestore(&imrx_spin_lock, imrx_irq_flags);

	return;
}

/**
 * dmac1_unmask_periphral_intr -	unmask the periphral interrupt
 * @midc: dma channel for which masking is required
 *
 * UnMasks the DMA periphral interrupt,
 * this is valid for DMAC1 family controllers only
 * This controller should have periphral mask registers already mapped
 */
static void dmac1_unmask_periphral_intr(struct intel_mid_dma_chan *midc)
{
	u32 pimr;
	struct middma_device *mid = to_middma_device(midc->chan.device);

	spin_lock_irqsave(&imrx_spin_lock, imrx_irq_flags);
	if (mid->pimr_mask && mid->dword_trf) {
		pimr = readl(mid->mask_reg + mid->pimr_offset);
		pimr &= ~mid->pimr_mask;
		writel(pimr, mid->mask_reg + mid->pimr_offset);
	}
	if (mid->pimr_mask && !mid->dword_trf) {
		pimr = readl(mid->mask_reg + mid->pimr_offset);
		pimr &= ~(1 << (midc->ch_id + PIMR_DMA1_INT_MASK_BASE));
		writel(pimr, mid->mask_reg + mid->pimr_offset);
	}
	spin_unlock_irqrestore(&imrx_spin_lock, imrx_irq_flags);

	return;
}

/**
 * enable_dma_interrupt -	enable the periphral interrupt
 * @midc: dma channel for which enable interrupt is required
 *
 * Enable the DMA periphral interrupt,
 * this is valid for DMAC1 family controllers only
 * This controller should have periphral mask registers already mapped
 */
static void enable_dma_interrupt(struct intel_mid_dma_chan *midc)
{
	struct middma_device *mid = to_middma_device(midc->chan.device);

	dmac1_unmask_periphral_intr(midc);

	/*en ch interrupts*/
	iowrite32(UNMASK_INTR_REG(midc->ch_id), midc->dma_base + MASK_TFR);
	set_bit(midc->ch_id, &mid->tfr_intr_mask);
	iowrite32(UNMASK_INTR_REG(midc->ch_id), midc->dma_base + MASK_ERR);

	return;
}

/**
 * disable_dma_interrupt -	disable the periphral interrupt
 * @midc: dma channel for which disable interrupt is required
 *
 * Disable the DMA periphral interrupt,
 * this is valid for DMAC1 family controllers only
 * This controller should have periphral mask registers already mapped
 */
static void disable_dma_interrupt(struct intel_mid_dma_chan *midc)
{
	struct middma_device *mid = to_middma_device(midc->chan.device);
	u32 pimr;

	/*Check LPE PISR, make sure fwd is disabled*/
	iowrite32(MASK_INTR_REG(midc->ch_id), midc->dma_base + MASK_BLOCK);
	clear_bit(midc->ch_id, &mid->block_intr_mask);
	iowrite32(MASK_INTR_REG(midc->ch_id), midc->dma_base + MASK_TFR);
	clear_bit(midc->ch_id, &mid->tfr_intr_mask);
	iowrite32(MASK_INTR_REG(midc->ch_id), midc->dma_base + MASK_ERR);

	spin_lock_irqsave(&imrx_spin_lock, imrx_irq_flags);
	if (mid->pimr_mask && !mid->dword_trf) {
		pimr = readl(mid->mask_reg + mid->pimr_offset);
		pimr |= (1 << (midc->ch_id + PIMR_DMA1_INT_MASK_BASE));
		writel(pimr, mid->mask_reg + mid->pimr_offset);
	}
	spin_unlock_irqrestore(&imrx_spin_lock, imrx_irq_flags);

	return;
}

/**
 * clear_dma_channel_interrupt - clear channel interrupt
 * @midc: dma channel for which clear interrupt is required
 *
 */
static void clear_dma_channel_interrupt(struct intel_mid_dma_chan *midc)
{
	struct middma_device *mid = to_middma_device(midc->chan.device);
	u32 isr;

	/*clearing this interrupts first*/
	iowrite32((1 << midc->ch_id), mid->dma_base + CLEAR_TFR);
	iowrite32((1 << midc->ch_id), mid->dma_base + CLEAR_BLOCK);
	iowrite32((1 << midc->ch_id), mid->dma_base + CLEAR_ERR);

	if (mid->pimr_mask && !mid->dword_trf) {
		isr = readl(mid->mask_reg);

		isr &= mid->pimr_mask;

		writel(isr, mid->mask_reg);

	}

	return;
}

/*****************************************************************************
DMA channel helper Functions*/
/**
 * mid_desc_get		-	get a descriptor
 * @midc: dma channel for which descriptor is required
 *
 * Obtain a descriptor for the channel. Returns NULL if none are free.
 * Once the descriptor is returned it is private until put on another
 * list or freed
 */
static struct intel_mid_dma_desc *midc_desc_get(struct intel_mid_dma_chan *midc)
{
	struct intel_mid_dma_desc *desc, *_desc;
	struct intel_mid_dma_desc *ret = NULL;

	spin_lock_bh(&midc->lock);
	list_for_each_entry_safe(desc, _desc, &midc->free_list, desc_node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del(&desc->desc_node);
			ret = desc;
			break;
		}
	}
	spin_unlock_bh(&midc->lock);

	return ret;
}

/**
 * mid_desc_put		-	put a descriptor
 * @midc: dma channel for which descriptor is required
 * @desc: descriptor to put
 *
 * Return a descriptor from lwn_desc_get back to the free pool
 */
static void midc_desc_put(struct intel_mid_dma_chan *midc,
			struct intel_mid_dma_desc *desc)
{
	if (desc) {
		spin_lock_bh(&midc->lock);
		list_add_tail(&desc->desc_node, &midc->free_list);
		spin_unlock_bh(&midc->lock);
	}
}
/**
 * midc_dostart		-		begin a DMA transaction
 * @midc: channel for which txn is to be started
 * @first: first descriptor of series
 *
 * Load a transaction into the engine. This must be called with midc->lock
 * held and bh disabled.
 */
static int midc_dostart(struct intel_mid_dma_chan *midc,
			struct intel_mid_dma_desc *first)
{
	struct middma_device *mid = to_middma_device(midc->chan.device);

	/*  channel is idle */
	if (midc->busy && test_ch_en(midc->dma_base, midc->ch_id)) {
		/*error*/
		pr_err("ERR_MDMA: channel is busy in start\n");
		/* The tasklet will hopefully advance the queue... */
		return -EBUSY;
	}
	midc->busy = true;
	/*write registers and en*/
	iowrite32(first->sar, midc->ch_regs + SAR);
	iowrite32(first->dar, midc->ch_regs + DAR);
	iowrite32(first->lli_phys, midc->ch_regs + LLP);
	iowrite32(first->cfg_hi, midc->ch_regs + CFG_HIGH);
	iowrite32(first->cfg_lo, midc->ch_regs + CFG_LOW);
	iowrite32(first->ctl_lo, midc->ch_regs + CTL_LOW);
	iowrite32(first->ctl_hi, midc->ch_regs + CTL_HIGH);
	pr_debug("MDMA:TX SAR %x,DAR %x,CFGH %x,CFGL %x,CTLH %x, CTLL %x LLI %x",
		(int)first->sar, (int)first->dar, first->cfg_hi,
		first->cfg_lo, first->ctl_hi, first->ctl_lo, (int)first->lli_phys);
	first->status = DMA_IN_PROGRESS;

	iowrite32(ENABLE_CHANNEL(midc->ch_id), mid->dma_base + DMA_CHAN_EN);

	return 0;
}

/**
 * midc_descriptor_complete	-	process completed descriptor
 * @midc: channel owning the descriptor
 * @desc: the descriptor itself
 *
 * Process a completed descriptor and perform any callbacks upon
 * the completion. The completion handling drops the lock during the
 * callbacks but must be called with the lock held.
 */
static void midc_descriptor_complete(struct intel_mid_dma_chan *midc,
		struct intel_mid_dma_desc *desc)
		__releases(&midc->lock) __acquires(&midc->lock)
{
	struct dma_async_tx_descriptor	*txd = &desc->txd;
	dma_async_tx_callback callback_txd = NULL;
	struct intel_mid_dma_lli	*llitem;
	void *param_txd = NULL;
	struct dma_chan *chan;

	chan = &midc->chan;
	txd->cookie = chan->cookie;
	dma_cookie_complete(txd);
	callback_txd = txd->callback;
	param_txd = txd->callback_param;

	if (desc->lli != NULL) {
		/*clear the DONE bit of completed LLI in memory*/
		llitem = desc->lli + desc->current_lli;
		llitem->ctl_hi &= desc->clear_done;
		if (desc->current_lli < desc->lli_length-1)
			(desc->current_lli)++;
		else
			desc->current_lli = 0;
	}
	if (midc->raw_tfr) {
		list_del(&desc->desc_node);
		desc->status = DMA_SUCCESS;
		if (desc->lli != NULL && desc->lli->llp != 0)
			pci_pool_free(desc->lli_pool, desc->lli,
						desc->lli_phys);
		list_add(&desc->desc_node, &midc->free_list);
		midc->busy = false;
		midc->raw_tfr = 0;
		spin_unlock_bh(&midc->lock);
	} else {
		spin_unlock_bh(&midc->lock);
	}
	if (callback_txd) {
		pr_debug("MDMA: TXD callback set ... calling\n");
		callback_txd(param_txd);
	}

	spin_lock_bh(&midc->lock);
}

static struct intel_mid_dma_desc *midc_first_queued(struct intel_mid_dma_chan *midc)
{
	return list_entry(midc->queue.next, struct intel_mid_dma_desc, desc_node);
}

static void midc_collect_descriptors(struct middma_device *mid,
				struct intel_mid_dma_chan *midc)
{
	struct intel_mid_dma_desc *desc = NULL, *_desc = NULL;

	/*tx is complete*/
	list_for_each_entry_safe(desc, _desc, &midc->active_list, desc_node) {
		if (desc->status == DMA_IN_PROGRESS)
			midc_descriptor_complete(midc, desc);
	}
}

/**
 * midc_start_descriptors -		start the descriptors in queue
 *
 * @mid: device
 * @midc: channel to scan
 *
 */
static void midc_start_descriptors(struct middma_device *mid,
				struct intel_mid_dma_chan *midc)
{
	if (!list_empty(&midc->queue)) {
		pr_debug("MDMA: submitting txn in queue\n");
		if (0 == midc_dostart(midc, midc_first_queued(midc)))
			list_splice_init(&midc->queue, &midc->active_list);
		else
			pr_warn("Submit failed as ch is busy\n");
	}
	return;
}

/**
 * midc_lli_fill_sg -		Helper function to convert
 *				SG list to Linked List Items.
 *@midc: Channel
 *@desc: DMA descriptor
 *@sglist: Pointer to SG list
 *@sglen: SG list length
 *@flags: DMA transaction flags
 *
 * Walk through the SG list and convert the SG list into Linked
 * List Items (LLI).
 */
static int midc_lli_fill_sg(struct intel_mid_dma_chan *midc,
				struct intel_mid_dma_desc *desc,
				struct scatterlist *src_sglist,
				struct scatterlist *dst_sglist,
				unsigned int sglen,
				unsigned int flags)
{
	struct intel_mid_dma_slave *mids;
	struct scatterlist  *sg;
	dma_addr_t lli_next, sg_phy_addr;
	struct intel_mid_dma_lli *lli_bloc_desc;
	union intel_mid_dma_ctl_lo ctl_lo;
	u32 ctl_hi;
	int i;

	pr_debug("MDMA: Entered %s\n", __func__);
	mids = midc->mid_slave;

	lli_bloc_desc = desc->lli;
	lli_next = desc->lli_phys;

	ctl_lo.ctl_lo = desc->ctl_lo;
	ctl_hi = desc->ctl_hi;
	for_each_sg(src_sglist, sg, sglen, i) {
		/*Populate CTL_LOW and LLI values*/
		if (i != sglen - 1) {
			lli_next = lli_next +
				sizeof(struct intel_mid_dma_lli);
		} else {
		/*Check for circular list, otherwise terminate LLI to ZERO*/
			if (flags & DMA_PREP_CIRCULAR_LIST) {
				pr_debug("MDMA: LLI is configured in circular mode\n");
				lli_next = desc->lli_phys;
			} else {
				lli_next = 0;
				/* llp_dst_en = 0 llp_src_en = 0 */
				ctl_lo.ctl_lo &= ~(1 << CTL_LO_BIT_LLP_DST_EN);
				ctl_lo.ctl_lo &= ~(1 << CTL_LO_BIT_LLP_SRC_EN);
			}
		}
		/*Populate CTL_HI values*/
		ctl_hi = get_block_ts(sg->length, desc->width,
					midc->dma->block_size, midc->dma->dword_trf);
		/*Populate SAR and DAR values*/
		sg_phy_addr = sg_phys(sg);
		if (desc->dirn ==  DMA_MEM_TO_DEV) {
			lli_bloc_desc->sar  = sg_phy_addr;
			lli_bloc_desc->dar  = mids->dma_slave.dst_addr;
		} else if (desc->dirn ==  DMA_DEV_TO_MEM) {
			lli_bloc_desc->sar  = mids->dma_slave.src_addr;
			lli_bloc_desc->dar  = sg_phy_addr;
		} else if (desc->dirn == DMA_TRANS_NONE && dst_sglist) {
				lli_bloc_desc->sar = sg_phy_addr;
				lli_bloc_desc->dar = sg_phys(dst_sglist);
		}
		/*Copy values into block descriptor in system memory*/
		lli_bloc_desc->llp = lli_next;
		lli_bloc_desc->ctl_lo = ctl_lo.ctl_lo;
		lli_bloc_desc->ctl_hi = ctl_hi;

		pr_debug("MDMA:Calc CTL LO %x, CTL HI %x src: %x dest: %x sg->l:%x\n",
					ctl_lo.ctl_lo, lli_bloc_desc->ctl_hi,
					lli_bloc_desc->sar, lli_bloc_desc->dar, sg->length);
		lli_bloc_desc++;
		if (dst_sglist)
			dst_sglist = sg_next(dst_sglist);
	}
	/*Copy very first LLI values to descriptor*/
	desc->ctl_lo = desc->lli->ctl_lo;
	desc->ctl_hi = desc->lli->ctl_hi;
	desc->sar = desc->lli->sar;
	desc->dar = desc->lli->dar;

	return 0;
}

/*****************************************************************************
DMA engine callback Functions*/
/**
 * intel_mid_dma_tx_submit -	callback to submit DMA transaction
 * @tx: dma engine descriptor
 *
 * Submit the DMA transaction for this descriptor, start if ch idle
 */
static dma_cookie_t intel_mid_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct intel_mid_dma_desc	*desc = to_intel_mid_dma_desc(tx);
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(tx->chan);
	dma_cookie_t		cookie;

	spin_lock_bh(&midc->lock);

	if (unlikely(!midc->in_use)) {
		spin_unlock_bh(&midc->lock);
		WARN(1, "chan[%d] gets new request after close",
			tx->chan->chan_id);
		return -EIO;
	}

	cookie = dma_cookie_assign(tx);

	if (list_empty(&midc->active_list))
		list_add_tail(&desc->desc_node, &midc->active_list);
	else
		list_add_tail(&desc->desc_node, &midc->queue);

	midc_dostart(midc, desc);
	spin_unlock_bh(&midc->lock);

	return cookie;
}

/**
 * intel_mid_dma_issue_pending -	callback to issue pending txn
 * @chan: chan where pending trascation needs to be checked and submitted
 *
 * Call for scan to issue pending descriptors
 */
static void intel_mid_dma_issue_pending(struct dma_chan *chan)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);

	spin_lock_bh(&midc->lock);
	if (!list_empty(&midc->queue))
		midc_start_descriptors(to_middma_device(chan->device), midc);
	spin_unlock_bh(&midc->lock);
}

/**
 * intel_mid_dma_tx_status -	Return status of txn
 * @chan: chan for where status needs to be checked
 * @cookie: cookie for txn
 * @txstate: DMA txn state
 *
 * Return status of DMA txn
 */
static enum dma_status intel_mid_dma_tx_status(struct dma_chan *chan,
						dma_cookie_t cookie,
						struct dma_tx_state *txstate)
{
	struct intel_mid_dma_chan *midc = to_intel_mid_dma_chan(chan);
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret != DMA_SUCCESS) {
		spin_lock_bh(&midc->lock);
		midc_start_descriptors(to_middma_device(chan->device), midc);
		spin_unlock_bh(&midc->lock);

		ret = dma_cookie_status(chan, cookie, txstate);
	}
	return ret;
}

static int intel_mid_dma_pause(struct dma_chan *chan)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	union intel_mid_dma_cfg_lo cfg_lo;
	cfg_lo.cfg_lo = ioread32(midc->ch_regs + CFG_LOW);
	cfg_lo.cfg_lo |= (1 << CFG_LO_BIT_CH_SUSP);
	iowrite32(cfg_lo.cfg_lo, midc->ch_regs + CFG_LOW);
	return 0;
}

static int intel_mid_dma_resume(struct dma_chan *chan)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	union intel_mid_dma_cfg_lo cfg_lo;
	cfg_lo.cfg_lo = ioread32(midc->ch_regs + CFG_LOW);
	cfg_lo.cfg_lo &= ~(1 << CFG_LO_BIT_CH_SUSP);
	iowrite32(cfg_lo.cfg_lo, midc->ch_regs + CFG_LOW);
	return 0;
}

static int dma_slave_control(struct dma_chan *chan, unsigned long arg)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	struct dma_slave_config  *slave = (struct dma_slave_config *)arg;
	struct intel_mid_dma_slave *mid_slave;

	BUG_ON(!midc);
	BUG_ON(!slave);
	pr_debug("MDMA: slave control called\n");

	mid_slave = to_intel_mid_dma_slave(slave);

	BUG_ON(!mid_slave);

	midc->mid_slave = mid_slave;

	return 0;
}

/**
 * intel_mid_dma_device_control -	DMA device control
 * @chan: chan for DMA control
 * @cmd: control cmd
 * @arg: cmd arg value
 *
 * Perform DMA control command
 */
static int intel_mid_dma_device_control(struct dma_chan *chan,
			enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	struct middma_device	*mid = to_middma_device(chan->device);
	struct intel_mid_dma_desc	*desc, *_desc;
	union intel_mid_dma_cfg_lo cfg_lo;
	u32 status;

	if (cmd == DMA_SLAVE_CONFIG)
		return dma_slave_control(chan, arg);

	if (cmd == DMA_PAUSE)
			return intel_mid_dma_pause(chan);
	if (cmd == DMA_RESUME)
			return intel_mid_dma_resume(chan);

	if (cmd != DMA_TERMINATE_ALL)
		return -ENXIO;

	spin_lock_bh(&midc->lock);
	if (midc->busy == false) {
		spin_unlock_bh(&midc->lock);
		return 0;
	}

	/* from this point onward is for FREE operation (eg _FREE_RX, _FREE_TX) */
	/* Suspend and drain the channel */
	cfg_lo.cfg_lo = ioread32(midc->ch_regs + CFG_LOW);
	cfg_lo.cfg_lo |= (1 << CFG_LO_BIT_CH_SUSP);
	cfg_lo.cfg_lo |= (1 << CFG_LO_BIT_CH_DRAIN);
	iowrite32(cfg_lo.cfg_lo, midc->ch_regs + CFG_LOW);

	/* to check fifo is empty before proceed */
	do {
		mdelay(5);
		status = readl(midc->ch_regs + CFG_LOW) & CFG_LO_FIFO_EMPTY;
	} while (status == 0);

	iowrite32(DISABLE_CHANNEL(midc->ch_id), mid->dma_base + DMA_CHAN_EN);

	/* to check chenreg bit are clear be4 proceed */
	do {
		mdelay(5);
		status = readl(midc->dma_base + DMA_CHAN_EN) & (0x01 << midc->ch_id) ;
	} while (status);

	/* Disable CH interrupts */
	disable_dma_interrupt(midc);
	/* clear channel interrupts */
	clear_dma_channel_interrupt(midc);

	/*Remove suspend/drain the channel*/
	cfg_lo.cfg_lo = ioread32(midc->ch_regs + CFG_LOW);
	cfg_lo.cfg_lo &= ~(1 << CFG_LO_BIT_CH_SUSP);
	cfg_lo.cfg_lo &= ~(1 << CFG_LO_BIT_CH_DRAIN);
	iowrite32(cfg_lo.cfg_lo, midc->ch_regs + CFG_LOW);

	midc->busy = false;
	midc->descs_allocated = 0;

	list_for_each_entry_safe(desc, _desc, &midc->active_list, desc_node) {
		list_del(&desc->desc_node);
		if (desc->lli != NULL)
			pci_pool_free(desc->lli_pool, desc->lli,
						desc->lli_phys);
		list_add(&desc->desc_node, &midc->free_list);
	}
	spin_unlock_bh(&midc->lock);

	return 0;
}

/**
 * intel_mid_dma_prep_memcpy -	Prep memcpy txn
 * @chan: chan for DMA transfer
 * @dest: destn address
 * @src: src address
 * @len: DMA transfer len
 * @flags: DMA flags
 *
 * Perform a DMA memcpy. Note we support slave peripheral DMA transfers only
 * The peripheral txn details should be filled in slave structure properly
 * Returns the descriptor for this txn
 */
static struct dma_async_tx_descriptor *intel_mid_dma_prep_memcpy(
			struct dma_chan *chan, dma_addr_t dest,
			dma_addr_t src, size_t len, unsigned long flags)
{
	struct intel_mid_dma_chan *midc;
	struct intel_mid_dma_desc *desc = NULL;
	struct intel_mid_dma_slave *mids;
	union intel_mid_dma_ctl_lo ctl_lo;
	u32 ctl_hi;
	union intel_mid_dma_cfg_lo cfg_lo;
	union intel_mid_dma_cfg_hi cfg_hi;
	enum dma_slave_buswidth width;
	int dst_reg_width = 0;
	int src_reg_width = 0;

	pr_debug("MDMA: Prep for memcpy\n");
	BUG_ON(!chan);
	if (!len)
		return NULL;

	midc = to_intel_mid_dma_chan(chan);
	BUG_ON(!midc);

	mids = midc->mid_slave;
	BUG_ON(!mids);

	if (unlikely(!midc->in_use)) {
		pr_err("ERR_MDMA: %s: channel not in use", __func__);
		return NULL;
	}

	pr_debug("MDMA:called for DMA %x CH %d Length %zu\n",
				midc->dma->pci_id, midc->ch_id, len);
	pr_debug("MDMA:Cfg passed Mode %x, Dirn %x, HS %x, Width %x\n",
			mids->cfg_mode, mids->dma_slave.direction,
			mids->hs_mode, mids->dma_slave.src_addr_width);

	/*calculate CFG_LO*/
	if (mids->hs_mode == LNW_DMA_SW_HS) {
		cfg_lo.cfg_lo = 0;
		cfg_lo.cfgx.hs_sel_dst = 1;
		cfg_lo.cfgx.hs_sel_src = 1;
	} else if (mids->hs_mode == LNW_DMA_HW_HS)
		cfg_lo.cfg_lo = 0x00000;

	/*calculate CFG_HI*/
	if (mids->cfg_mode == LNW_DMA_MEM_TO_MEM) {
		/*SW HS only*/
		cfg_hi.cfg_hi = 0;
	} else {
		cfg_hi.cfg_hi = 0;
		if (midc->dma->pimr_mask) {
			cfg_hi.cfgx.protctl = 0x0; /*default value*/
			cfg_hi.cfgx.fifo_mode = 1;
			if (mids->dma_slave.direction == DMA_MEM_TO_DEV) {
				cfg_hi.cfgx.src_per = 0;
				if (mids->device_instance == 0)
					cfg_hi.cfgx.dst_per = 3;
				if (mids->device_instance == 1)
					cfg_hi.cfgx.dst_per = 1;
			} else if (mids->dma_slave.direction == DMA_DEV_TO_MEM) {
				if (mids->device_instance == 0)
					cfg_hi.cfgx.src_per = 2;
				if (mids->device_instance == 1)
					cfg_hi.cfgx.src_per = 0;
				cfg_hi.cfgx.dst_per = 0;
			}
		} else {
			cfg_hi.cfgx.protctl = 0x1; /*default value*/
			cfg_hi.cfgx.src_per = cfg_hi.cfgx.dst_per =
					midc->ch_id - midc->dma->chan_base;
		}
	}
	/*calculate CTL_HI*/
	width = mids->dma_slave.src_addr_width;
	ctl_hi = get_block_ts(len, width, midc->dma->block_size, midc->dma->dword_trf);
	pr_debug("MDMA:calc len %d for block size %d\n",
				ctl_hi, midc->dma->block_size);
	/*calculate CTL_LO*/
	ctl_lo.ctl_lo = 0;
	ctl_lo.ctlx.int_en = 1;

	dst_reg_width = get_reg_width(mids->dma_slave.dst_addr_width);
	if (dst_reg_width < 0) {
		pr_err("ERR_MDMA: Failed to get DST reg width\n");
		return NULL;

	}
	ctl_lo.ctlx.dst_tr_width = dst_reg_width;

	src_reg_width = get_reg_width(mids->dma_slave.src_addr_width);
	if (src_reg_width < 0) {
		pr_err("ERR_MDMA: Failed to get SRC reg width\n");
				return NULL;
	}
	ctl_lo.ctlx.src_tr_width = src_reg_width;
	ctl_lo.ctlx.dst_msize = mids->dma_slave.src_maxburst;
	ctl_lo.ctlx.src_msize = mids->dma_slave.dst_maxburst;

	if (mids->cfg_mode == LNW_DMA_MEM_TO_MEM) {
		ctl_lo.ctlx.tt_fc = 0;
		ctl_lo.ctlx.sinc = 0;
		ctl_lo.ctlx.dinc = 0;
	} else {
		if (mids->dma_slave.direction == DMA_MEM_TO_DEV) {
			ctl_lo.ctlx.sinc = 0;
			ctl_lo.ctlx.dinc = 2;
			ctl_lo.ctlx.tt_fc = 1;
		} else if (mids->dma_slave.direction == DMA_DEV_TO_MEM) {
			ctl_lo.ctlx.sinc = 2;
			ctl_lo.ctlx.dinc = 0;
			ctl_lo.ctlx.tt_fc = 2;
		}
	}

	pr_debug("MDMA:Calc CTL LO %x, CTL HI %x, CFG LO %x, CFG HI %x\n",
		ctl_lo.ctl_lo, ctl_hi, cfg_lo.cfg_lo, cfg_hi.cfg_hi);

	enable_dma_interrupt(midc);

	desc = midc_desc_get(midc);
	if (desc == NULL)
		goto err_desc_get;
	desc->sar = src;
	desc->dar = dest ;
	desc->len = len;
	desc->cfg_hi = cfg_hi.cfg_hi;
	desc->cfg_lo = cfg_lo.cfg_lo;
	desc->ctl_lo = ctl_lo.ctl_lo;
	desc->ctl_hi = ctl_hi;
	desc->width = width;
	desc->dirn = mids->dma_slave.direction;
	desc->lli_phys = 0;
	desc->lli = NULL;
	desc->lli_pool = NULL;
	desc->clear_done = CLEAR_DONE_v1;

	return &desc->txd;

err_desc_get:
	pr_err("ERR_MDMA: Failed to get desc\n");
	midc_desc_put(midc, desc);
	return NULL;
}

/**
 * intel_mid_dma_prep_memcpy_v2 - Prep memcpy txn
 * @chan: chan for DMA transfer
 * @dest: destn address
 * @src: src address
 * @len: DMA transfer len
 * @flags: DMA flags
 *
 * Perform a DMA memcpy. Note we support slave periphral DMA transfers only
 * The periphral txn details should be filled in slave structure properly
 * Returns the descriptor for this txn
 */
static struct dma_async_tx_descriptor *intel_mid_dma_prep_memcpy_v2(
			struct dma_chan *chan, dma_addr_t dest,
			dma_addr_t src, size_t len, unsigned long flags)
{
	struct intel_mid_dma_chan *midc;
	struct intel_mid_dma_desc *desc = NULL;
	struct intel_mid_dma_slave *mids;
	union intel_mid_dma_ctl_lo ctl_lo;
	u32 ctl_hi;
	union intel_mid_dma_cfg_lo cfg_lo;
	union intel_mid_dma_cfg_hi cfg_hi;
	enum dma_slave_buswidth width;
	int dst_reg_width = 0;
	int src_reg_width = 0;

	pr_debug("MDMA:%s\n", __func__);
	BUG_ON(!chan);
	if (!len)
		return NULL;

	midc = to_intel_mid_dma_chan(chan);
	BUG_ON(!midc);

	mids = midc->mid_slave;
	BUG_ON(!mids);

	if (unlikely(!midc->in_use)) {
		pr_err("ERR_MDMA: %s: channel not in use", __func__);
		return NULL;
	}

	pr_debug("MDMA:called for DMA %x CH %d Length %zu\n",
				midc->dma->pci_id, midc->ch_id, len);
	pr_debug("MDMA:Cfg passed Mode %x, Dirn %x, HS %x, Width %x\n",
			mids->cfg_mode, mids->dma_slave.direction,
			mids->hs_mode, mids->dma_slave.src_addr_width);

	/*calculate CFG_LO*/
	cfg_lo.cfgx_v2.dst_burst_align = 1;
	cfg_lo.cfgx_v2.src_burst_align = 1;

	/*calculate CFG_HI*/
	if (mids->cfg_mode == LNW_DMA_MEM_TO_MEM) {
		/*SW HS only*/
		cfg_hi.cfg_hi = 0;
	} else {
		cfg_hi.cfg_hi = 0;
		if (midc->dma->pimr_mask) {
			if (mids->dma_slave.direction == DMA_MEM_TO_DEV) {
				cfg_hi.cfgx_v2.src_per = 0;
				if (mids->device_instance == 0)
					cfg_hi.cfgx_v2.dst_per = 1;
				if (mids->device_instance == 1)
					cfg_hi.cfgx_v2.dst_per = 3;
				if (mids->device_instance == 2)
					cfg_hi.cfgx_v2.dst_per = 5;
			} else if (mids->dma_slave.direction == DMA_DEV_TO_MEM) {
				if (mids->device_instance == 0)
					cfg_hi.cfgx_v2.src_per = 0;
				if (mids->device_instance == 1)
					cfg_hi.cfgx_v2.src_per = 2;
				if (mids->device_instance == 2)
					cfg_hi.cfgx_v2.src_per = 4;
				cfg_hi.cfgx_v2.dst_per = 0;
			}
		} else {
			cfg_hi.cfgx_v2.src_per = cfg_hi.cfgx_v2.dst_per =
					midc->ch_id - midc->dma->chan_base;
		}
	}
	/*calculate CTL_HI*/
	width = mids->dma_slave.src_addr_width;

	ctl_hi = get_block_ts(len, width, midc->dma->block_size, midc->dma->dword_trf);
	pr_debug("MDMA:calc len %d for block size %d\n",
				ctl_hi, midc->dma->block_size);
	/*calculate CTL_LO*/
	ctl_lo.ctl_lo = 0;
	ctl_lo.ctlx_v2.int_en = 1;

	dst_reg_width = get_reg_width(mids->dma_slave.dst_addr_width);
	if (dst_reg_width < 0) {
		pr_err("ERR_MDMA: Failed to get DST reg width\n");
		return NULL;

	}
	ctl_lo.ctlx_v2.dst_tr_width = dst_reg_width;

	src_reg_width = get_reg_width(mids->dma_slave.src_addr_width);
	if (src_reg_width < 0) {
		pr_err("ERR_MDMA: Failed to get SRC reg width\n");
				return NULL;
	}

	ctl_lo.ctlx_v2.src_tr_width = src_reg_width;
	ctl_lo.ctlx_v2.dst_msize = mids->dma_slave.src_maxburst;
	ctl_lo.ctlx_v2.src_msize = mids->dma_slave.dst_maxburst;

	if (mids->cfg_mode == LNW_DMA_MEM_TO_MEM) {
		ctl_lo.ctlx_v2.tt_fc = 0;
		ctl_lo.ctlx_v2.sinc = 0;
		ctl_lo.ctlx_v2.dinc = 0;
	} else {
		if (mids->dma_slave.direction == DMA_MEM_TO_DEV) {
			ctl_lo.ctlx_v2.sinc = 0;
			ctl_lo.ctlx_v2.dinc = 1;
			ctl_lo.ctlx_v2.tt_fc = 1;
		} else if (mids->dma_slave.direction == DMA_DEV_TO_MEM) {
			ctl_lo.ctlx_v2.sinc = 1;
			ctl_lo.ctlx_v2.dinc = 0;
			ctl_lo.ctlx_v2.tt_fc = 2;
		}
	}

	pr_debug("MDMA:Calc CTL LO %x, CTL HI %x, CFG LO %x, CFG HI %x\n",
		ctl_lo.ctl_lo, ctl_hi, cfg_lo.cfg_lo, cfg_hi.cfg_hi);

	enable_dma_interrupt(midc);

	desc = midc_desc_get(midc);
	if (desc == NULL)
		goto err_desc_get;
	desc->sar = src;
	desc->dar = dest ;
	desc->len = len;
	desc->cfg_hi = cfg_hi.cfg_hi;
	desc->cfg_lo = cfg_lo.cfg_lo;
	desc->ctl_lo = ctl_lo.ctl_lo;
	desc->ctl_hi = ctl_hi;
	desc->width = width;
	desc->dirn = mids->dma_slave.direction;
	desc->lli_phys = 0;
	desc->lli = NULL;
	desc->lli_pool = NULL;
	desc->clear_done = CLEAR_DONE_v2;

	return &desc->txd;

err_desc_get:
	pr_err("ERR_MDMA: Failed to get desc\n");
	midc_desc_put(midc, desc);
	return NULL;
}

/**
 * intel_mid_dma_chan_prep_desc
 * @chan: chan for DMA transfer
 * @src_sg: destination scatter gather list
 * @dst_sg: source scatter gather list
 * @flags: DMA flags
 * @src_sg_len: length of src sg list
 * @direction DMA transfer dirtn
 *
 * Prepares LLI based periphral transfer
 */
static struct dma_async_tx_descriptor *intel_mid_dma_chan_prep_desc(
			struct dma_chan *chan, struct scatterlist *src_sg,
			struct scatterlist *dst_sg, unsigned long flags,
			unsigned long src_sg_len,
			enum dma_transfer_direction direction)
{
	struct middma_device *mid = NULL;
	struct intel_mid_dma_chan *midc = NULL;
	struct intel_mid_dma_slave *mids = NULL;
	struct intel_mid_dma_desc *desc = NULL;
	struct dma_async_tx_descriptor *txd = NULL;
	union intel_mid_dma_ctl_lo ctl_lo;

	pr_debug("MDMA:intel_mid_dma_chan_prep_desc\n");

	midc = to_intel_mid_dma_chan(chan);
	BUG_ON(!midc);

	mid = to_middma_device(midc->chan.device);
	mids = midc->mid_slave;
	BUG_ON(!mids);

	if (!midc->dma->pimr_mask) {
		pr_err("MDMA: SG list is not supported by this controller\n");
		return  NULL;
	}

	txd = midc->dma->dma_ops.device_prep_dma_memcpy(chan, 0, 0, src_sg->length, flags);
	if (NULL == txd) {
		pr_err("MDMA: Prep memcpy failed\n");
		return NULL;
	}

	desc = to_intel_mid_dma_desc(txd);
	desc->dirn = direction;
	ctl_lo.ctl_lo = desc->ctl_lo;
	ctl_lo.ctl_lo |= (1 << CTL_LO_BIT_LLP_DST_EN);
	ctl_lo.ctl_lo |= (1 << CTL_LO_BIT_LLP_SRC_EN);
	desc->ctl_lo = ctl_lo.ctl_lo;
	desc->lli_length = src_sg_len;
	desc->current_lli = 0;
	/* DMA coherent memory pool for LLI descriptors*/
	desc->lli_pool = pci_pool_create("intel_mid_dma_lli_pool",
				midc->dma->pdev,
				(sizeof(struct intel_mid_dma_lli)*src_sg_len),
				32, 0);
	if (NULL == desc->lli_pool) {
		pr_err("MID_DMA:LLI pool create failed\n");
		return NULL;
	}
	midc->lli_pool = desc->lli_pool;

	desc->lli = pci_pool_alloc(desc->lli_pool, GFP_KERNEL, &desc->lli_phys);
	if (!desc->lli) {
		pr_err("MID_DMA: LLI alloc failed\n");
		pci_pool_destroy(desc->lli_pool);
		return NULL;
	}
	midc_lli_fill_sg(midc, desc, src_sg, dst_sg, src_sg_len, flags);
	if (flags & DMA_PREP_INTERRUPT) {
		/* Enable Block intr, disable TFR intr.
		* It's not required to enable TFR, when Block intr is enabled
		* Otherwise, for last block we will end up in invoking calltxd
		* two times */

		iowrite32(MASK_INTR_REG(midc->ch_id),
					midc->dma_base + MASK_TFR);
		clear_bit(midc->ch_id, &mid->tfr_intr_mask);
		iowrite32(UNMASK_INTR_REG(midc->ch_id),
					midc->dma_base + MASK_BLOCK);
		set_bit(midc->ch_id, &mid->block_intr_mask);
		midc->block_intr_status = true;
		pr_debug("MDMA: Enabled Block Interrupt\n");
	}

	return &desc->txd;

}

/**
 * intel_mid_dma_prep_sg -        Prep sg txn
 * @chan: chan for DMA transfer
 * @dst_sg: destination scatter gather list
 * @dst_sg_len: length of dest sg list
 * @src_sg: source scatter gather list
 * @src_sg_len: length of src sg list
 * @flags: DMA flags
 *
 * Prepares LLI based periphral transfer
 */
static struct dma_async_tx_descriptor *intel_mid_dma_prep_sg(
			struct dma_chan *chan, struct scatterlist *dst_sg,
			unsigned int dst_sg_len, struct scatterlist *src_sg,
			unsigned int src_sg_len, unsigned long flags)
{
	pr_debug("MDMA: Prep for memcpy SG\n");

	if ((dst_sg_len != src_sg_len) || (dst_sg == NULL) ||
							(src_sg == NULL)) {
		pr_err("MDMA: Invalid SG length\n");
		return NULL;
	}

	pr_debug("MDMA: SG Length = %d, Flags = %#lx, src_sg->length = %d\n",
				src_sg_len, flags, src_sg->length);

	return intel_mid_dma_chan_prep_desc(chan, src_sg, dst_sg, flags,
						src_sg_len, DMA_TRANS_NONE);

}

/**
 * intel_mid_dma_prep_slave_sg -	Prep slave sg txn
 * @chan: chan for DMA transfer
 * @sgl: scatter gather list
 * @sg_len: length of sg txn
 * @direction: DMA transfer dirtn
 * @flags: DMA flags
 * @context: transfer context (ignored)
 *
 * Prepares LLI based periphral transfer
 */
static struct dma_async_tx_descriptor *intel_mid_dma_prep_slave_sg(
			struct dma_chan *chan, struct scatterlist *sg,
			unsigned int sg_len, enum dma_transfer_direction direction,
			unsigned long flags, void *context)
{
	pr_debug("MDMA: Prep for slave SG\n");

	if (!sg_len || sg == NULL) {
		pr_err("MDMA: Invalid SG length\n");
		return NULL;
	}
	pr_debug("MDMA: SG Length = %d, direction = %d, Flags = %#lx\n",
				sg_len, direction, flags);
	if (direction != DMA_TRANS_NONE) {
		return intel_mid_dma_chan_prep_desc(chan, sg, NULL, flags,
							sg_len, direction);
	} else {
		pr_err("MDMA: Invalid Direction\n");
		return NULL;
	}
}

/**
 * intel_mid_dma_free_chan_resources -	Frees dma resources
 * @chan: chan requiring attention
 *
 * Frees the allocated resources on this DMA chan
 */
static void intel_mid_dma_free_chan_resources(struct dma_chan *chan)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	struct middma_device	*mid = to_middma_device(chan->device);
	struct intel_mid_dma_desc	*desc, *_desc;

	if (false == midc->in_use) {
		pr_err("ERR_MDMA: try to free chnl already freed\n");
		return;
	}
	if (true == midc->busy) {
		/*trying to free ch in use!!!!!*/
		pr_err("ERR_MDMA: trying to free ch in use\n");
		dump_dma_reg(chan);
	}

	/* Disable CH interrupts */
	disable_dma_interrupt(midc);
	clear_dma_channel_interrupt(midc);

	midc->block_intr_status = false;
	midc->in_use = false;
	midc->busy = false;

	tasklet_unlock_wait(&mid->tasklet);

	spin_lock_bh(&midc->lock);
	midc->descs_allocated = 0;
	list_for_each_entry_safe(desc, _desc, &midc->active_list, desc_node) {
		list_del(&desc->desc_node);
		pci_pool_free(mid->dma_pool, desc, desc->txd.phys);
	}
	list_for_each_entry_safe(desc, _desc, &midc->free_list, desc_node) {
		list_del(&desc->desc_node);
		pci_pool_free(mid->dma_pool, desc, desc->txd.phys);
	}
	list_for_each_entry_safe(desc, _desc, &midc->queue, desc_node) {
		list_del(&desc->desc_node);
		pci_pool_free(mid->dma_pool, desc, desc->txd.phys);
	}
	midc->raw_tfr = 0;
	spin_unlock_bh(&midc->lock);

	if (midc->lli_pool) {
		pci_pool_destroy(midc->lli_pool);
		midc->lli_pool = NULL;
	}

	/* Disable the channel */
	iowrite32(DISABLE_CHANNEL(midc->ch_id), mid->dma_base + DMA_CHAN_EN);
}

/**
 * intel_mid_dma_alloc_chan_resources -	Allocate dma resources
 * @chan: chan requiring attention
 *
 * Allocates DMA resources on this chan
 * Return the descriptors allocated
 */
static int intel_mid_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct intel_mid_dma_chan	*midc = to_intel_mid_dma_chan(chan);
	struct middma_device	*mid = to_middma_device(chan->device);
	struct intel_mid_dma_desc	*desc;
	dma_addr_t		phys;
	int	i = 0;

	if (mid->state == SUSPENDED) {
		if (dma_resume(&mid->pdev->dev)) {
			pr_err("ERR_MDMA: resume failed");
			return -EFAULT;
		}
	}

	/* ASSERT:  channel is idle */
	if (midc->in_use == true) {
		pr_err("ERR_MDMA: ch not idle\n");
		return -EIO;
	}
	dma_cookie_init(chan);

	spin_lock_bh(&midc->lock);
	while (midc->descs_allocated < DESCS_PER_CHANNEL) {
		spin_unlock_bh(&midc->lock);
		desc = pci_pool_alloc(mid->dma_pool, GFP_KERNEL, &phys);
		if (!desc) {
			pr_err("ERR_MDMA: desc failed\n");		
			return -ENOMEM;
			/*check*/
		}
		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.tx_submit = intel_mid_dma_tx_submit;
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.phys = phys;
		spin_lock_bh(&midc->lock);
		i = ++midc->descs_allocated;
		list_add_tail(&desc->desc_node, &midc->free_list);
	}
	midc->busy = false;
	spin_unlock_bh(&midc->lock);
	midc->in_use = true;
	midc->block_intr_status = false;

	return i;
}

/**
 * midc_handle_error -	Handle DMA txn error
 * @mid: controller where error occurred
 * @midc: chan where error occurred
 *
 * Scan the descriptor for error
 */
static void midc_handle_error(struct middma_device *mid,
		struct intel_mid_dma_chan *midc)
{
	midc_collect_descriptors(mid, midc);
	midc_start_descriptors(mid, midc);
}

/**
 * dma_tasklet -	DMA interrupt tasklet
 * @data: tasklet arg (the controller structure)
 *
 * Scan the controller for interrupts for completion/error
 * Clear the interrupt and call for handling completion/error
 */
static void dma_tasklet(unsigned long data)
{
	struct middma_device *mid = NULL;
	struct intel_mid_dma_chan *midc = NULL;
	u32 status, raw_tfr, raw_block;
	int i;

	mid = (struct middma_device *)data;
	if (mid == NULL) {
		pr_err("ERR_MDMA: tasklet Null param\n");
		return;
	}
	raw_tfr = ioread32(mid->dma_base + RAW_TFR);
	status = raw_tfr & mid->tfr_intr_mask;
	pr_debug("MDMA: in tasklet for device %x\n", mid->pci_id);

	while (status) {
		/*txn interrupt*/
		i = get_ch_index(status, mid->chan_base);
		if (i < 0) {
			pr_err("ERR_MDMA:Invalid ch index %x\n", i);
			return;
		}
		/* clear the status bit */
		status = status & ~(1 << (i + mid->chan_base));
		midc = &mid->ch[i];
		if (midc == NULL) {
			pr_err("ERR_MDMA:Null param midc\n");
			return;
		}
		pr_debug("MDMA:Tx complete interrupt %x, Ch No %d Index %d\n",
				status, midc->ch_id, i);
		spin_lock_bh(&midc->lock);
		midc->raw_tfr = raw_tfr;
		/*clearing this interrupts first*/
		iowrite32((1 << midc->ch_id), mid->dma_base + CLEAR_TFR);
		if (likely(midc->in_use)) {
			midc_collect_descriptors(mid, midc);
			midc_start_descriptors(mid, midc);
		}
		pr_debug("MDMA:Scan of desc... complete, unmasking\n");
		iowrite32(UNMASK_INTR_REG(midc->ch_id),
					mid->dma_base + MASK_TFR);
		spin_unlock_bh(&midc->lock);
	}

	raw_block = ioread32(mid->dma_base + RAW_BLOCK);
	status = raw_block & mid->block_intr_mask;
	pr_debug("MDMA: in tasklet for device %x\n", mid->pci_id);

	while (status) {
		/*txn interrupt*/
		i = get_ch_index(status, mid->chan_base);
		if (i < 0) {
			pr_err("ERR_MDMA:Invalid ch index %x\n", i);
			return;
		}
		/* clear the status bit */
		status = status & ~(1 << (i + mid->chan_base));
		midc = &mid->ch[i];
		if (midc == NULL) {
			pr_err("ERR_MDMA:Null param midc\n");
			return;
		}
		pr_debug("MDMA:Tx complete interrupt raw block  %x, Ch No %d Index %d\n",
				status, midc->ch_id, i);
		spin_lock_bh(&midc->lock);
		/*clearing this interrupts first*/

		midc->raw_block = raw_block;
		iowrite32((1 << midc->ch_id), mid->dma_base + CLEAR_BLOCK);
		if (midc->block_intr_status) {
			midc_collect_descriptors(mid, midc);
			midc_start_descriptors(mid, midc);
		}

		iowrite32(UNMASK_INTR_REG(midc->ch_id),
					mid->dma_base + MASK_BLOCK);
		spin_unlock_bh(&midc->lock);
	}

	status = ioread32(mid->dma_base + RAW_ERR);
	pr_debug("MDMA:raw error status:%#x\n", status);
	while (status) {
		/*err interrupt*/
		i = get_ch_index(status, mid->chan_base);
		if (i < 0) {
			pr_err("ERR_MDMA:Invalid ch index %x (raw err)\n", i);
			return;
		}
		status = status & ~(1 << (i + mid->chan_base));
		midc = &mid->ch[i];
		if (midc == NULL) {
			pr_err("ERR_MDMA:Null param midc (raw err)\n");
			return;
		}
		pr_debug("MDMA:Tx complete interrupt %x, Ch No %d Index %d\n",
				status, midc->ch_id, i);

		iowrite32((1 << midc->ch_id), mid->dma_base + CLEAR_ERR);
		spin_lock_bh(&midc->lock);
		midc_handle_error(mid, midc);
		iowrite32(UNMASK_INTR_REG(midc->ch_id),
				mid->dma_base + MASK_ERR);
		spin_unlock_bh(&midc->lock);
	}
	pr_debug("MDMA:Exiting takslet...\n");

	return;
}

static void dma_tasklet1(unsigned long data)
{
	pr_debug("MDMA:in takslet1...\n");
	dma_tasklet(data);
}

static void dma_tasklet2(unsigned long data)
{
	pr_debug("MDMA:in takslet2...\n");
	dma_tasklet(data);
}

/**
 * intel_mid_dma_interrupt -	DMA ISR
 * @irq: IRQ where interrupt occurred
 * @data: ISR cllback data (the controller structure)
 *
 * See if this is our interrupt if so then schedule the tasklet
 * otherwise ignore
 */
static irqreturn_t intel_mid_dma_interrupt(int irq, void *data)
{
	struct middma_device *mid = data;
	u32 tfr_status, err_status, block_status;
	int call_tasklet = 0;
	u32 isr;

	/*DMA Interrupt*/
	pr_debug("MDMA:Got an interrupt on irq %d\n", irq);
	if (!mid) {
		pr_err("ERR_MDMA:null pointer mid\n");
		return -EINVAL;
	}

	/* Read the interrupt status registers */
	tfr_status = ioread32(mid->dma_base + STATUS_TFR);
	err_status = ioread32(mid->dma_base + STATUS_ERR);
	block_status = ioread32(mid->dma_base + STATUS_BLOCK);

	if (!tfr_status && !err_status && !block_status)
		return IRQ_HANDLED;

	pr_debug("MDMA: trf_Status %x, Mask %x\n", tfr_status, mid->intr_mask);
	if (tfr_status) {
		/*need to disable intr*/
		iowrite32((tfr_status << INT_MASK_WE),
						mid->dma_base + MASK_TFR);
		call_tasklet = 1;
	}
	if (block_status) {
		/*need to disable intr*/
		iowrite32((block_status << INT_MASK_WE),
						mid->dma_base + MASK_BLOCK);
		call_tasklet = 1;
	}
	if (err_status) {
		iowrite32((err_status << INT_MASK_WE),
			  mid->dma_base + MASK_ERR);
		call_tasklet = 1;
	}
	/* in mrlfd we need to clear the pisr bits to stop intr as well
	 * so read the PISR register, see if we have pisr bits status and clear
	 * them
	 */
	if (mid->pimr_mask && !mid->dword_trf) {
		isr = readl(mid->mask_reg);
		pr_debug("isr says: %x", isr);
		if (isr) {
			isr &= mid->pimr_mask;
			pr_debug("writing isr: %x", isr);
			writel(isr, mid->mask_reg);
		}
	}

	if (call_tasklet)
		tasklet_schedule(&mid->tasklet);

	return IRQ_HANDLED;
}

static irqreturn_t intel_mid_dma_interrupt1(int irq, void *data)
{
	return intel_mid_dma_interrupt(irq, data);
}

static irqreturn_t intel_mid_dma_interrupt2(int irq, void *data)
{
	return intel_mid_dma_interrupt(irq, data);
}

static void config_dma_fifo_partition(struct middma_device *dma)
{
	/* program FIFO Partition registers - 128 bytes for each ch */
	iowrite32(DMA_FIFO_SIZE, dma->dma_base + FIFO_PARTITION0_HI);
	iowrite32(DMA_FIFO_SIZE, dma->dma_base + FIFO_PARTITION1_LO);
	iowrite32(DMA_FIFO_SIZE, dma->dma_base + FIFO_PARTITION1_HI);
	iowrite32(DMA_FIFO_SIZE | BIT(26), dma->dma_base + FIFO_PARTITION0_LO);
}

static struct intel_mid_dma_ops v1_dma_ops = {
	.device_alloc_chan_resources	= intel_mid_dma_alloc_chan_resources,
	.device_free_chan_resources		= intel_mid_dma_free_chan_resources,
	.device_prep_dma_memcpy			= intel_mid_dma_prep_memcpy,
	.device_prep_dma_sg				= intel_mid_dma_prep_sg,
	.device_prep_slave_sg			= intel_mid_dma_prep_slave_sg,
	.device_control					= intel_mid_dma_device_control,
	.device_tx_status				= intel_mid_dma_tx_status,
	.device_issue_pending			= intel_mid_dma_issue_pending,
};

static struct intel_mid_dma_ops v2_dma_ops = {
	.device_alloc_chan_resources    = intel_mid_dma_alloc_chan_resources,
	.device_free_chan_resources     = intel_mid_dma_free_chan_resources,
	.device_prep_dma_memcpy         = intel_mid_dma_prep_memcpy_v2,
	.device_prep_dma_sg             = intel_mid_dma_prep_sg,
	.device_prep_slave_sg           = intel_mid_dma_prep_slave_sg,
	.device_control                 = intel_mid_dma_device_control,
	.device_tx_status               = intel_mid_dma_tx_status,
	.device_issue_pending           = intel_mid_dma_issue_pending,
};

/**
 * mid_setup_dma -	Setup the DMA controller
 * @pdev: Controller PCI device structure
 *
 * Initialize the DMA controller, channels, registers with DMA engine,
 * ISR. Initialize DMA controller channels.
 */
static int mid_setup_dma(struct pci_dev *pdev)
{
	struct middma_device *dma = pci_get_drvdata(pdev);
	int err, i;

	/* DMA coherent memory pool for DMA descriptor allocations */
	dma->dma_pool = pci_pool_create("intel_mid_dma_desc_pool", pdev,
					sizeof(struct intel_mid_dma_desc),
					32, 0);
	if (NULL == dma->dma_pool) {
		pr_err("ERR_MDMA:pci_pool_create failed\n");
		err = -ENOMEM;
		kfree(dma);
		goto err_dma_pool;
	}

	INIT_LIST_HEAD(&dma->common.channels);
	dma->pci_id = pdev->device;

	if (dma->pimr_mask) {
		dma->mask_reg = ioremap(dma->pimr_base, LNW_PERIPHRAL_MASK_SIZE);
		if (dma->mask_reg == NULL) {
			pr_err("ERR_MDMA:Can't map periphral intr space !!\n");
			err = -ENOMEM;
			goto err_ioremap;
		}
	} else
		dma->mask_reg = NULL;

	pr_debug("MDMA:Adding %d channel for this controller\n", dma->max_chan);
	/*init CH structures*/
	dma->intr_mask = 0;
	dma->state = RUNNING;
	for (i = 0; i < dma->max_chan; i++) {
		struct intel_mid_dma_chan *midch = &dma->ch[i];

		midch->chan.device = &dma->common;
		dma_cookie_init(&midch->chan);
		midch->ch_id = dma->chan_base + i;
		pr_debug("MDMA:Init CH %d, ID %d\n", i, midch->ch_id);

		midch->dma_base = dma->dma_base;
		midch->ch_regs = dma->dma_base + DMA_CH_SIZE * midch->ch_id;
		midch->dma = dma;
		dma->intr_mask |= 1 << (dma->chan_base + i);
		spin_lock_init(&midch->lock);

		INIT_LIST_HEAD(&midch->active_list);
		INIT_LIST_HEAD(&midch->queue);
		INIT_LIST_HEAD(&midch->free_list);
		/*mask interrupts*/
		iowrite32(MASK_INTR_REG(midch->ch_id),
			dma->dma_base + MASK_BLOCK);
		iowrite32(MASK_INTR_REG(midch->ch_id),
			dma->dma_base + MASK_SRC_TRAN);
		iowrite32(MASK_INTR_REG(midch->ch_id),
			dma->dma_base + MASK_DST_TRAN);
		iowrite32(MASK_INTR_REG(midch->ch_id),
			dma->dma_base + MASK_ERR);
		iowrite32(MASK_INTR_REG(midch->ch_id),
			dma->dma_base + MASK_TFR);

		disable_dma_interrupt(midch);
		list_add_tail(&midch->chan.device_node, &dma->common.channels);
	}
	pr_debug("MDMA: Calc Mask as %x for this controller\n", dma->intr_mask);

	/*init dma structure*/
	dma_cap_zero(dma->common.cap_mask);
	dma_cap_set(DMA_MEMCPY, dma->common.cap_mask);
	dma_cap_set(DMA_SLAVE, dma->common.cap_mask);
	dma_cap_set(DMA_PRIVATE, dma->common.cap_mask);
	dma->common.dev = &pdev->dev;

	dma->common.device_alloc_chan_resources = dma->dma_ops.device_alloc_chan_resources;
	dma->common.device_free_chan_resources = dma->dma_ops.device_free_chan_resources;

	dma->common.device_tx_status = dma->dma_ops.device_tx_status;
	dma->common.device_prep_dma_memcpy = dma->dma_ops.device_prep_dma_memcpy;
	dma->common.device_prep_dma_sg = dma->dma_ops.device_prep_dma_sg;
	dma->common.device_issue_pending = dma->dma_ops.device_issue_pending;
	dma->common.device_prep_slave_sg = dma->dma_ops.device_prep_slave_sg;
	dma->common.device_control = dma->dma_ops.device_control;

	/*enable dma cntrl*/
	iowrite32(REG_BIT0, dma->dma_base + DMA_CFG);

	/*register irq */
	if (dma->pimr_mask) {
		pr_debug("MDMA:Requesting irq shared for DMAC1\n");
		err = request_irq(pdev->irq, intel_mid_dma_interrupt1,
			IRQF_SHARED, "INTEL_MID_DMAC1", dma);
		if (0 != err)
			goto err_irq;
	} else {
		pr_debug("MDMA:Requesting irq for DMAC2\n");
		err = request_irq(pdev->irq, intel_mid_dma_interrupt2,
			IRQF_SHARED, "INTEL_MID_DMAC2", dma);
		if (0 != err)
			goto err_irq;
	}
	/*register device w/ engine*/
	err = dma_async_device_register(&dma->common);
	if (0 != err) {
		pr_err("ERR_MDMA:device_register failed: %d\n", err);
		goto err_engine;
	}
	if (dma->pimr_mask) {
		pr_debug("setting up tasklet1 for DMAC1\n");
		tasklet_init(&dma->tasklet, dma_tasklet1, (unsigned long)dma);
	} else {
		pr_debug("setting up tasklet2 for DMAC2\n");
		tasklet_init(&dma->tasklet, dma_tasklet2, (unsigned long)dma);
	}
	if (!dma->dword_trf) {
		config_dma_fifo_partition(dma);
		/* Mask all interrupts from DMA controller to IA by default */
		dmac1_mask_periphral_intr(dma);
	}
	return 0;

err_engine:
	free_irq(pdev->irq, dma);
err_irq:
	if (dma->mask_reg)
		iounmap(dma->mask_reg);
err_ioremap:
	pci_pool_destroy(dma->dma_pool);
err_dma_pool:
	pr_err("ERR_MDMA:setup_dma failed: %d\n", err);
	return err;

}

/**
 * middma_shutdown -	Shutdown the DMA controller
 * @pdev: Controller PCI device structure
 *
 * Called by remove
 * Unregister DMa controller, clear all structures and free interrupt
 */
static void middma_shutdown(struct pci_dev *pdev)
{
	struct middma_device *device = pci_get_drvdata(pdev);

	dma_async_device_unregister(&device->common);
	pci_pool_destroy(device->dma_pool);
	if (device->mask_reg)
		iounmap(device->mask_reg);
	if (device->dma_base)
		iounmap(device->dma_base);
	free_irq(pdev->irq, device);

	return;
}

static int dma_byt_irq_enable(struct pci_dev *dev) {
	
	struct io_apic_irq_attr irq_attr;

	dev->irq=25;
	irq_attr.ioapic = mp_find_ioapic(25);
		if(irq_attr.ioapic<0){
			pr_err(KERN_ERR "ERROR: No IOAPIC for IRQ=%d DID=0x%x \n",dev->irq, dev->device);
			return irq_attr.ioapic;
		}
	irq_attr.ioapic_pin = 25;
	irq_attr.trigger = 1; /* level */
	irq_attr.polarity = 1; /* active low */
	io_apic_set_pci_routing(&dev->dev, 25, &irq_attr);

	return 0;

}

/**
 * intel_mid_dma_probe -	PCI Probe
 * @pdev: Controller PCI device structure
 * @id: pci device id structure
 *
 * Initialize the PCI device, map BARs, query driver data.
 * Call setup_dma to complete contoller and chan initilzation
 */
static int intel_mid_dma_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct middma_device *device;
	u32 base_addr, bar_size;
	struct intel_mid_dma_probe_info *info;
	int err;
#ifdef CONFIG_SND_BYT_AK4614
	int number_bin_ssp;
#endif

	pr_debug("MDMA: probe for %x\n", pdev->device);
	info = (void *)id->driver_data;
	pr_debug("MDMA: CH %d, base %d, block len %d, Periphral mask %x\n",
				info->max_chan, info->ch_base,
				info->block_size, info->pimr_mask);
#ifdef PCI_ENCASE
	pci = kzalloc(sizeof(struct lpe_device), GFP_KERNEL);
	if (!pci) {
			pr_err("PCI container malloc fail\n");
			return -ENOMEM;
	}
	pci->dma_ctx = kzalloc(sizeof(*pdev), GFP_KERNEL);

	memcpy(pci->dma_ctx, pdev, sizeof(*pdev) );
	if (!pci->dma_ctx)
		return -ENODEV;

	INIT_LIST_HEAD((&(&pci->dma_ctx->dev)->dma_pools)); /*Init the dma_pools lli for pci container*/
	INIT_LIST_HEAD((&(&pci->dma_ctx->dev)->devres_head)); /*Init the devres_head for pci container*/

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		pr_err("ERR_MDMA:kzalloc failed probe\n");
		err = -ENOMEM;
		goto err_kzalloc;
	}

	device->pdev = pci_dev_get(pdev);
	device->pci_id = pdev->device;

	/* Init the device */
	if(device->pci_id == SST_BYT_PCI_ID)
		err = dma_byt_irq_enable(pdev);
	else
		err = pci_enable_device(pdev);
	if (err) {
		pr_err("device can't be enabled\n");
		goto err_enable_device;
	}

#else
	err = pci_enable_device(pdev);
	if (err)
		goto err_enable_device;
#endif
	err = pci_request_regions(pdev, "intel_lpe_audio");

	if (err)
		goto err_request_regions;

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));

	if (err)
		goto err_set_dma_mask;

	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));

	if (err)
		goto err_set_dma_mask;

#ifndef PCI_ENCASE
	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		pr_err("ERR_MDMA:kzalloc failed probe\n");
		err = -ENOMEM;
		goto err_kzalloc;
	}
	device->pdev = pci_dev_get(pdev);
#endif
	base_addr = pci_resource_start(pdev, 0) + DMAC2_START_ADDRESS ;
	pr_debug("LPE-DMA base address: 0x%x \n", base_addr);

	bar_size  = pci_resource_len(pdev, 0);

	device->dma_base = ioremap_nocache(base_addr, DMA_REG_SIZE);
	if (!device->dma_base) {
		pr_err("ERR_MDMA:ioremap failed\n");
		err = -ENOMEM;
		goto err_ioremap;
	}
	pci_set_drvdata(pdev, device);

	pci_set_master(pdev);

	device->max_chan = info->max_chan;
	device->chan_base = info->ch_base;
	device->block_size = info->block_size;
	device->pimr_mask = info->pimr_mask;
	device->pimr_base = pci_resource_start(pdev, 0) + info->pimr_base;
	device->dword_trf = info->dword_trf;
	device->pimr_offset = info->pimr_offset;
	memcpy(&device->dma_ops, info->pdma_ops, sizeof(struct intel_mid_dma_ops));

#ifdef CONFIG_PRH_TEMP_WA_FOR_SPID
	/* PRH uses, ch 4,5,6,7 override the info table data */
	pr_info("Device is Bodegabay\n");
	device->max_chan = 4;
	device->chan_base = 4;
#endif
	err = mid_setup_dma(pdev);

	if (err)
		goto err_dma;

#ifdef CONFIG_SND_BYT_AK4614
	//PCI container sharing.
	number_bin_ssp = to_load_bin_ssp();
	if(number_bin_ssp > 0)
		intel_sst_probe(pci->dma_ctx, id);
	intel_mid_i2s_probe(pdev, id);
#elif defined CONFIG_SND_BYT_RT5642
		intel_sst_probe(pci->dma_ctx, id);
#elif defined CONFIG_SND_BYT_MAX98090
	intel_sst_probe(pci->dma_ctx, id);
#endif

	return 0;

err_dma:
	iounmap(device->dma_base);
err_ioremap:
	pci_dev_put(pdev);
err_set_dma_mask:
	pci_release_regions(pdev);
	pci_disable_device(pdev);
err_request_regions:
err_enable_device:
err_kzalloc:
	kfree(pci->dma_ctx);
	kfree(pci);
	kfree(device);
	pr_err("ERR_MDMA:Probe failed %d\n", err);
	return err;
}


/**
 * intel_mid_dma_remove -	PCI remove
 * @pdev: Controller PCI device structure
 *
 * Free up all resources and data
 * Call shutdown_dma to complete contoller and chan cleanup
 */
static void intel_mid_dma_remove(struct pci_dev *pdev)
{
	struct middma_device *device = pci_get_drvdata(pdev);

	/*TODO:Add:
	 * 1. intel_sst_remove
	 * 2. intel_mid_i2s_remove
	 * free pci container
	 */

	middma_shutdown(pdev);
	pci_dev_put(pdev);
	kfree(device);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

/* Power Management */
/*
* dma_suspend - suspend function
*
* @dev: device structure
*
* This function is called by OS when a power event occurs
*/
int dma_suspend(struct device *dev)
{
	int i;
	struct middma_device *device = dev_get_drvdata(dev);

	pr_debug("MDMA: dma_suspend called\n");

	for (i = 0; i < device->max_chan; i++) {
		if (device->ch[i].in_use)
			return -EAGAIN;
	}
	dmac1_mask_periphral_intr(device);
	device->state = SUSPENDED;
	return 0;
}

/**
* dma_resume - PCI resume function
*
* @pci:	PCI device structure
*
* This function is called by OS when a power event occurs
*/
int dma_resume(struct device *dev)
{
	struct middma_device *device = dev_get_drvdata(dev);

	pr_debug("MDMA: dma_resume called\n");

#ifdef CONFIG_PROTECT_AUDIO_DMAIRQ
	/* Force the audio DMAC Interrupt to CPU1 to prevent the DMA
	 * interrupt delayed by other irq long processing
	 */
	irq_set_affinity(device->pdev->irq,
		cpumask_of(find_last_bit(cpumask_bits(cpu_online_mask),
			NR_CPUS)));
#endif
	device->state = RUNNING;
	iowrite32(REG_BIT0, device->dma_base + DMA_CFG);

	if (!device->dword_trf)
		config_dma_fifo_partition(device);

	return pm_schedule_suspend(dev, 0);
}

/******************************************************************************
* PCI stuff
*/
static struct pci_device_id intel_mid_dma_ids[] = {
	{ PCI_VDEVICE(INTEL, INTEL_MID_DMAC1_ID),
					INFO(2, 6, 4095, 0x200020, 0xFFAE8008, 1, 0x8, &v1_dma_ops)},
	{ PCI_VDEVICE(INTEL, INTEL_MID_DMAC2_ID),
					INFO(2, 0, 2047, 0, 0, 1, 0, &v1_dma_ops)},
	{ PCI_VDEVICE(INTEL, INTEL_MID_GP_DMAC2_ID),
					INFO(2, 0, 2047, 0, 0, 1, 0, &v1_dma_ops)},
	{ PCI_VDEVICE(INTEL, INTEL_MFLD_DMAC1_ID),
					INFO(4, 0, 4095, 0x400040, 0xFFAE8008, 1, 0x8, &v1_dma_ops)},
	/* Byt */
	{ PCI_VDEVICE(INTEL, SST_BYT_PCI_ID),
					INFO(8, 0, 131071, 0xFF000000, 0x00140018, 0, 0x10, &v2_dma_ops)},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_mid_dma_ids);

static struct pci_driver intel_mid_dma_pci_driver = {
	.name		=	"Intel LPE Audio",
	.id_table	=	intel_mid_dma_ids,
	.probe		=	intel_mid_dma_probe,
	.remove		=	intel_mid_dma_remove,
};

static int __init intel_mid_dma_init(void)
{
	pr_debug("INFO_MDMA: LNW DMA Driver Version %s\n",
			INTEL_MID_DMA_DRIVER_VERSION);
	return pci_register_driver(&intel_mid_dma_pci_driver);
}

static void __exit intel_mid_dma_exit(void)
{
	pci_unregister_driver(&intel_mid_dma_pci_driver);
}

module_init(intel_mid_dma_init);
module_exit(intel_mid_dma_exit);

MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_DESCRIPTION("Intel (R) MID DMAC Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(INTEL_MID_DMA_DRIVER_VERSION);
