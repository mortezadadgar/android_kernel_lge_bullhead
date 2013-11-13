#ifndef __INTEL_SST_COMMON_H__
#define __INTEL_SST_COMMON_H__
/*
 *  intel_sst_common.h - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corporation
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
 *  Common private declarations for SST
 */

#include <linux/dmaengine.h>
#include <linux/intel_mid_dma.h>

#define SST_DRIVER_VERSION "3.0.8"
#define SST_VERSION_NUM 0x2004

/* driver names */
#define SST_DRV_NAME "intel_sst_driver"
#define SST_MRST_PCI_ID 0x080A
#define SST_MFLD_PCI_ID 0x082F
#define SST_CLV_PCI_ID	0x08E7
#define SST_MRFLD_PCI_ID  0x119A
#ifdef SST_BYT_PCI_ID
#undef SST_BYT_PCI_ID
#endif
#define SST_BYT_PCI_ID 0x0F28
#define PCI_ID_LENGTH 4
#define SST_SUSPEND_DELAY 2000
#define FW_CONTEXT_MEM (64*1024)
#define MAX_TIMEDIVSOR  0xFF
#define MAX_BASEUNIT 0x80
#define SST_ICCM_BOUNDARY 4

#define SST_SHIM_START_ADDRESS 0x140000
#define SST_SHIM_END_ADDRESS 0x143FFF
#define SST_MAILBOX_START_ADDRESS 0x144000
#define SST_MAILBOX_END_ADDRESS 0x147FFF

#define SST_ICCM_START_ADDRESS 0x0C0000
#define SST_ICCM_END_ADDRESS 0x0FFFFF

#define SST_DCCM_START_ADDRESS 0x100000
#define SST_DCCM_END_ADDRESS 0x13FFFF

#define SST_BYT_DDR_CACHE_PHY_ADDR_OFFSET 0xA8
#define SST_BYT_DDR_CACHE_ALLOC_SIZE_OFFSET  0xAC

#define CSTATE_EXIT_LATENCY_S0i1 1040

struct intel_sst_ops {
	irqreturn_t (*interrupt) (int, void *);
	irqreturn_t (*irq_thread) (int, void *);
	void (*clear_interrupt) (void);
	int (*start) (void);
	int (*reset) (void);
	void (*process_reply) (struct work_struct *work);
	void (*post_message) (struct work_struct *work);
	int (*sync_post_message) (struct ipc_post *msg);
	void (*process_message) (struct work_struct *work);
};

enum sst_states {
	SST_FW_LOADED = 1,
	SST_FW_RUNNING,
	SST_START_INIT,
	SST_UN_INIT,
	SST_ERROR,
	SST_SUSPENDED,
	SST_FW_CTXT_RESTORE
};

#define MAX_ACTIVE_STREAM	3
#define MAX_ENC_STREAM		1
#define MAX_AM_HANDLES		1
#define SST_BLOCK_TIMEOUT       1000
#define BLOCK_UNINIT		-1
#define RX_TIMESLOT_UNINIT	-1

/* SST register map */
#define SST_CSR			0x00
#define SST_PISR		0x08
#define SST_PIMR		0x10
#define SST_ISRX		0x18
#define SST_ISRD		0x20
#define SST_IMRX		0x28
#define SST_IMRD		0x30
#define SST_IPCX		0x38 /* IPC IA-SST */
#define SST_IPCD		0x40 /* IPC SST-IA */
#define SST_ISRSC		0x48
#define SST_ISRLPESC		0x50
#define SST_IMRSC		0x58
#define SST_IMRLPESC		0x60
#define SST_IPCSC		0x68
#define SST_IPCLPESC		0x70
#define SST_CLKCTL		0x78
#define SST_CSR2		0x80

#define SST_ISRD		0x20 /* dummy register for shim workaround */
#define SST_SHIM_SIZE		0x88
#define SST_PWMCTRL             0x1000

#define SPI_MODE_ENABLE_BASE_ADDR 0xffae4000
#define FW_SIGNATURE_SIZE	4
#define BYT_W48_XYZ		1

/* stream states */
enum sst_stream_states {
	STREAM_UN_INIT	= 0,	/* Freed/Not used stream */
	STREAM_RUNNING	= 1,	/* Running */
	STREAM_PAUSED	= 2,	/* Paused stream */
	STREAM_DECODE	= 3,	/* stream is in decoding only state */
	STREAM_INIT	= 4,	/* stream init, waiting for data */
};

enum sst_ram_type {
	SST_IRAM	= 1,
	SST_DRAM	= 2,
	SST_CACHE	= 3,
};


/* SST shim registers to structure mapping  */
union config_status_reg {
	struct {
		u32 mfld_strb:1;
		u32 sst_reset:1;
		u32 clk_sel:3;
		u32 sst_clk:2;
		u32 bypass:3;
		u32 run_stall:1;
		u32 rsvd1:2;
		u32 strb_cntr_rst:1;
		u32 rsvd:18;
	} part;
	u32 full;
};

union interrupt_reg {
	struct {
		u64 done_interrupt:1;
		u64 busy_interrupt:1;
		u64 rsvd:62;
	} part;
	u64 full;
};

union sst_imr_reg {
	struct {
		u32 done_interrupt:1;
		u32 busy_interrupt:1;
		u32 rsvd:30;
	} part;
	u32 full;
};

union sst_pisr_reg {
	struct {
		u32 pssp0:1;
		u32 pssp1:1;
		u32 rsvd0:3;
		u32 dmac:1;
		u32 rsvd1:26;
	} part;
	u32 full;
};

union sst_pimr_reg {
	struct {
		u32 ssp0:1;
		u32 ssp1:1;
		u32 rsvd0:3;
		u32 dmac:1;
		u32 rsvd1:10;
		u32 ssp0_sc:1;
		u32 ssp1_sc:1;
		u32 rsvd2:3;
		u32 dmac_sc:1;
		u32 rsvd3:10;
	} part;
	u32 full;
};

#ifdef SST_DRV_BYT
/****************************************************************************/ 
/*			BYT LPE Shim Registers Definition                   */
/****************************************************************************/

union csr_reg_byt {
	struct {
		u64 lpe_reset:1;
		u64 lpe_reset_vector:1;
		u64 runstall:1;
		u64 pwaitmode:1;
		u64 clk_sel:3;
		u64 rsvd2:4;
		u64 xt_snoop:1;
		u64 rsvd3:4;
		u64 clk_sel1:6;
		u64 rsvd:42;
	} part;
	u64 full;
};

union imrx_reg_byt {
	struct {
		u64 lpe_ia_ipc_done_mask:1;
		u64 lpe_ia_ipc_request_mask:1;
		u64 rsvd:1;
		u64 iapis_ssp0:1;
		u64 iapis_ssp1:1;
		u64 iapis_ssp2:1;
		u64 rsvd1:10;
		u64 dma0_interrupt_ia_mask:8;
		u64 dma1_interrupt_ia_mask:8;
		u64 rsvd2:32;
	} part;
	u64 full;
};

union imrd_reg_byt {
	struct {
		u64 ia_lpe_ipc_done_mask:1;
		u64 ia_lpe_ipc_request_mask:1;
		u64 rsvd:62;
	} part;
	u64 full;
};

union isrx_reg_byt {
	struct {
		u64 lpe_ia_ipc_done_status:1;
		u64 lpe_ia_ipc_request_status:1;
		u64 rsvd:1;
		u64 iapis_ssp0:1;
		u64 iapis_ssp1:1;
		u64 iapis_ssp2:1;
		u64 rsvd2:10;
		u64 iapis_dma0:8;
		u64 iapis_dma1:8;
		u64 rsvd3:32;
		
	} part;
	u64 full;
} ;

union isrd_reg_byt {
	struct {
		u64 ia_lpe_ipc_done_status:1;
		u64 ia_lpe_ipc_request_status:1;
		u64 rsvd:62;
	} part;
	u64 full;
};


union ipcd_reg_byt {
	struct {
		u64 lpe_ia_msg:62;
		u64 ia_lpe_done:1;
		u64 lpe_ia_busy:1;
	} part;
	u64 full;
};

union ipcx_reg_byt {
	struct {
		u64 ia_lpe_msg:62;
		u64 lpe_ia_done:1;
		u64 ia_lpe_busy:1;
	} part;
	u64 full;
};

#endif /* SST_DRV_BYT */

/*This structure is used to block a user/fw data call to another
fw/user call
*/
struct sst_block {
	bool	condition; /* condition for blocking check */
	int	ret_code; /* ret code when block is released */
	void	*data; /* data to be appsed for block if any */
	bool	on;
};

/**
 * struct stream_info - structure that holds the stream information
 *
 * @status : stream current state
 * @prev : stream prev state
 * @codec : stream codec
 * @sst_id : stream id
 * @ops : stream operation pb/cp/drm...
 * @bufs: stream buffer list
 * @lock : stream mutex for protecting state
 * @data_blk : stream block for data operations
 * @ctrl_blk : stream block for ctrl operations
 * @pcm_substream : PCM substream
 * @period_elapsed : PCM period elapsed callback
 * @sfreq : stream sampling freq
 * @str_type : stream type
 * @cumm_bytes : cummulative bytes decoded
 * @str_type : stream type
 * @src : stream source
 * @device : output device type (medfield only)
 * @pcm_slot : pcm slot value
 */
struct stream_info {
	unsigned int		status;
	unsigned int		prev;
	unsigned int		ops;
	struct mutex		lock; /* mutex */
	struct sst_block	data_blk; /* stream ops block */
	struct sst_block	ctrl_blk; /* stream control cmd block */
	void			*pcm_substream;
	void (*period_elapsed) (void *pcm_substream);
	unsigned int		sfreq;
	u32			cumm_bytes;
	void			*compr_cb_param;
	void			(*compr_cb) (void *compr_cb_param);
	unsigned int		num_ch;
};

/*
 * struct stream_alloc_bloc - this structure is used for blocking the user's
 * alloc calls to fw's response to alloc calls
 *
 * @sst_id : session id of blocked stream
 * @ops_block : ops block struture
 */
struct stream_alloc_block {
	int			sst_id; /* session id of blocked stream */
	struct sst_block	ops_block; /* ops block struture */
};

#define SST_FW_SIGN "$SST"
#define SST_FW_LIB_SIGN "$LIB"

/*
 * struct fw_header - FW file headers
 *
 * @signature : FW signature
 * @modules : # of modules
 * @file_format : version of header format
 * @reserved : reserved fields
 */
struct fw_header {
	unsigned char signature[FW_SIGNATURE_SIZE]; /* FW signature */
	u32 file_size; /* size of fw minus this header */
	u32 modules; /*  # of modules */
	u32 file_format; /* version of header format */
	u32 reserved[4];
};

struct fw_module_header {
	unsigned char signature[FW_SIGNATURE_SIZE]; /* module signature */
	u32 mod_size; /* size of module */
	u32 blocks; /* # of blocks */
	u32 type; /* codec type, pp lib */
	u32 entry_point;
};

struct dma_block_info {
	enum sst_ram_type	type;	/* IRAM/DRAM */
	u32			size;	/* Bytes */
	u32			ram_offset; /* Offset in I/DRAM */
	u32			rsvd;	/* Reserved field */
};

struct ioctl_pvt_data {
	int			str_id;
	int			pvt_id;
};

struct sst_ipc_msg_wq {
#ifdef SST_DRV_BYT
	union ipc_header_byt byt_header;
#endif
	union ipc_header_mrfld mrfld_header;
	union ipc_header	header;
	char mailbox[SST_MAILBOX_SIZE];
	struct work_struct	wq;
};


struct sst_dma {
	struct dma_chan *ch;
	struct intel_mid_dma_slave slave;
	struct pci_dev *dmac;
};

struct sst_runtime_param {
	struct snd_sst_runtime_params param;
};

struct sst_sg_list {
	struct scatterlist *src;
	struct scatterlist *dst;
	int list_len;
};


#ifdef CONFIG_DEBUG_FS
struct sst_debugfs {
	struct dentry *root;
	int runtime_pm_status;
};
#endif

#define PCI_DMAC_MFLD_ID 0x0830
#define PCI_DMAC_CLV_ID 0x08F0
#define SST_MAX_DMA_LEN (4095*4)

/***
 * struct intel_sst_drv - driver ops
 *
 * @sst_state : current sst device state
 * @pci_id : PCI device id loaded
 * @shim : SST shim pointer
 * @mailbox : SST mailbox pointer
 * @iram : SST IRAM pointer
 * @dram : SST DRAM pointer
 * @shim_phy_add : SST shim phy addr
 * @ipc_dispatch_list : ipc messages dispatched
 * @ipc_post_msg_wq : wq to post IPC messages context
 * @ipc_process_msg : wq to process msgs from FW context
 * @ipc_process_reply : wq to process reply from FW context
 * @ipc_post_msg : wq to post reply from FW context
 * @mad_ops : MAD driver operations registered
 * @mad_wq : MAD driver wq
 * @post_msg_wq : wq to post IPC messages
 * @process_msg_wq : wq to process msgs from FW
 * @process_reply_wq : wq to process reply from FW
 * @streams : sst stream contexts
 * @alloc_block : block structure for alloc
 * @tgt_dev_blk : block structure for target device
 * @fw_info_blk : block structure for fw info block
 * @vol_info_blk : block structure for vol info block
 * @mute_info_blk : block structure for mute info block
 * @hs_info_blk : block structure for hs info block
 * @list_lock : sst driver list lock (deprecated)
 * @ipc_spin_lock : spin lock to handle audio shim access and ipc queue
 * @scard_ops : sst card ops
 * @pci : sst pci device struture
 * @sst_lock : sst device lock
 * @stream_lock : sst stream lock
 * @unique_id : sst unique id
 * @stream_cnt : total sst active stream count
 * @pb_streams : total active pb streams
 * @cp_streams : total active cp streams
 * @lpe_stalled : lpe stall status
 * @pmic_port_instance : active pmic port instance
 * @lpaudio_start : lpaudio status
 * @audio_start : audio status
 * @max_streams : max streams allowed
 * @qos		: PM Qos struct
 * @fw_cntx	: CPU pointer to firmware-context save-resume block
 * @fw_cntx_size : size of save-resume block in bytes
 * @fw_cntx_handle : firmware 32-bit pointer to save-resume block
 */
struct intel_sst_drv {
	int			sst_state;
	unsigned int		pci_id;
	void __iomem		*ddr;
	void __iomem		*shim;
	void __iomem		*mailbox;
	void __iomem		*iram;
	void __iomem		*dram;
	void __iomem		*fw_ext_base;
	unsigned int		fw_ext_phy_add;
	unsigned int		iram_base;
	unsigned int		dram_base;
	unsigned int		shim_phy_add;
	unsigned int		iram_end;
	unsigned int		dram_end;
	unsigned int		ddr_end;
	unsigned int		ddr_base;
	unsigned int		lpe_base;
	struct list_head	ipc_dispatch_list;
	struct work_struct	ipc_post_msg_wq;
	struct sst_ipc_msg_wq	ipc_process_msg;
	struct sst_ipc_msg_wq	ipc_process_reply;
	struct sst_ipc_msg_wq	ipc_post_msg;
	wait_queue_head_t	wait_queue;
	struct workqueue_struct *mad_wq;
	struct workqueue_struct *post_msg_wq;
	struct workqueue_struct *process_msg_wq;
	struct workqueue_struct *process_reply_wq;
	unsigned int		tstamp;
	struct stream_info streams[MAX_NUM_STREAMS+1]; /*str_id 0 is not used*/
	struct stream_alloc_block alloc_block[MAX_ACTIVE_STREAM];
	struct sst_block	fw_info_blk, ppp_params_blk, dma_info_blk;
	struct mutex		list_lock;/* mutex for IPC list locking */
	spinlock_t		ipc_spin_lock; /* lock for Shim reg access and ipc queue */
	struct snd_pmic_ops	*scard_ops;
	struct pci_dev		*pci;
	struct mutex            sst_lock;
	struct mutex		stream_lock;
	unsigned int            unique_id;
	unsigned int		stream_cnt;	/* total streams */
	unsigned int		am_cnt;
	unsigned int		pb_streams;	/* pb streams active */
	unsigned int		cp_streams;	/* cp streams active */
	/* 1 - LPA stream(MP3 pb) in progress*/
	unsigned int		audio_start;
	unsigned int		max_streams;
	void			*fw_cntx;
	unsigned int		fw_cntx_size;
	dma_addr_t		fw_cntx_handle;
	unsigned int		compressed_slot;
#ifdef SST_DRV_BYT
	u64                     csr_value;
#else
	unsigned int		csr_value;
#endif /* SST_DRV_BYT */
	const struct firmware	*fw;

	struct sst_dma		dma;
	void			*fw_in_mem;
	struct sst_runtime_param runtime_param;
	unsigned int		device_input_mixer;
	struct mutex		mixer_ctrl_lock;
	struct dma_async_tx_descriptor *desc;
	struct sst_sg_list fw_sg_list, library_list;
	struct intel_sst_ops	*ops;
#ifdef CONFIG_DEBUG_FS
	struct sst_debugfs debugfs;
#endif
	struct pm_qos_request *qos;
};

extern struct intel_sst_drv *sst_drv_ctx;

/* misc definitions */
#define FW_DWNL_ID 0xFF

int sst_alloc_stream(char *params);
int sst_alloc_stream_response(unsigned int str_id,
				struct snd_sst_alloc_response *response);
int sst_alloc_stream_response_mrfld(unsigned int str_id);
int sst_stalled(void);
int sst_pause_stream(int id);
int sst_resume_stream(int id);
int sst_drop_stream(int id);
int sst_free_stream(int id);
int sst_start_stream(int str_id);

int sst_set_stream_param(int str_id, struct snd_sst_params *str_param);
int sst_get_fw_info(struct snd_sst_fw_info *info);
int sst_get_stream_params(int str_id,
		struct snd_sst_get_stream_params *get_params);
int sst_get_stream(struct snd_sst_params *str_param);
int sst_get_stream_allocated(struct snd_sst_params *str_param,
				struct snd_sst_lib_download **lib_dnld);
int sst_drain_stream(int str_id);


#ifdef SST_DRV_BYT
int sst_sync_post_message_byt(struct ipc_post *msg);
void sst_post_message_byt(struct work_struct *work);
void sst_process_message_byt(struct work_struct *work);
void sst_process_reply_byt(struct work_struct *work);
int sst_start_byt(void);
int intel_sst_reset_dsp_byt(void);
void intel_sst_clear_intr_byt(void);
#endif /* SST_DRV_BYT */

void sst_process_mad_ops(struct work_struct *work);
void sst_process_mad_jack_detection(struct work_struct *work);

long intel_sst_ioctl(struct file *file_ptr, unsigned int cmd,
			unsigned long arg);
int intel_sst_open_cntrl(struct inode *i_node, struct file *file_ptr);
int intel_sst_release_cntrl(struct inode *i_node, struct file *file_ptr);

int sst_request_fw(void);
int sst_load_fw(const void *fw_in_mem, void *context);
int sst_load_library(struct snd_sst_lib_download *lib, u8 ops);
int sst_get_block_stream(struct intel_sst_drv *sst_drv_ctx);

int sst_wait_interruptible(struct intel_sst_drv *sst_drv_ctx,
				struct sst_block *block);
int sst_wait_timeout(struct intel_sst_drv *sst_drv_ctx,
			struct sst_block *block);
int sst_create_large_msg(struct ipc_post **arg);
int sst_create_short_msg(struct ipc_post **arg);
void sst_wake_up_alloc_block(struct intel_sst_drv *sst_drv_ctx,
		u8 sst_id, int status, void *data);
int sst_download_fw(void);
void free_stream_context(unsigned int str_id);
void sst_clean_stream(struct stream_info *stream);
int intel_sst_register_compress(struct intel_sst_drv *sst);
int intel_sst_remove_compress(struct intel_sst_drv *sst);
void sst_cdev_fragment_elapsed(int str_id);
int vibra_pwm_configure(unsigned int enable);
int sst_send_algo_param(struct snd_ppp_params *algo_params);
int sst_send_sync_msg(int ipc, int str_id);
int sst_get_num_channel(struct snd_sst_params *str_param);
int sst_get_wdsize(struct snd_sst_params *str_param);
int sst_get_sfreq(struct snd_sst_params *str_param);
int intel_sst_check_device(void);
int parse_module_byt(struct fw_module_header *pmodule);

/*
 * sst_fill_header - inline to fill sst header
 *
 * @header : ipc header
 * @msg : IPC message to be sent
 * @large : is ipc large msg
 * @str_id : stream id
 *
 * this function is an inline function that sets the headers before
 * sending a message
 */
static inline void sst_fill_header(union ipc_header *header,
				int msg, int large, int str_id)
{
	header->part.msg_id = msg;
	header->part.str_id = str_id;
	header->part.large = large;
	header->part.done = 0;
	header->part.busy = 1;
	header->part.data = 0;
}

#ifdef SST_DRV_BYT
static inline void sst_fill_header_byt(union ipc_header_byt *header, int msg, int large, int str_id)
{
	header->f = 0;
	/* Refer to <struct ipc_header_low_byt> in intel_sst_fw_ipc.h */
	header->p.header_low_payload.part.msg_id = msg;
	header->p.header_low_payload.part.str_id = str_id;
	header->p.header_low_payload.part.large = large;
	header->p.header_low_payload.part.data = 0;  /* caller to fill the data after this function*/

	/* Refer to <struct ipc_header_high_byt> in intel_sst_fw_ipc.h */
	header->p.header_high.part.done = 0;
	header->p.header_high.part.busy = 1;
}
#endif /* SST_DRV_BYT */

/* sst_assign_pvt_id - assign a pvt id for stream
 *
 * @sst_drv_ctx : driver context
 *
 * this inline function assigns a private id for calls that dont have stream
 * context yet, should be called with lock held
 */
static inline unsigned int sst_assign_pvt_id(struct intel_sst_drv *sst_drv_ctx)
{
	sst_drv_ctx->unique_id++;
	if (sst_drv_ctx->unique_id > MAX_NUM_STREAMS)
		sst_drv_ctx->unique_id = 1;
	return sst_drv_ctx->unique_id;
}


/*
 * sst_init_stream - this function initialzes stream context
 *
 * @stream : stream struture
 * @codec : codec for stream
 * @sst_id : stream id
 * @ops : stream operation
 * @slot : stream pcm slot
 * @device : device type
 *
 * this inline function initialzes stream context for allocated stream
 */
static inline void sst_init_stream(struct stream_info *stream,
		int codec, int sst_id, int ops, u8 slot)
{
	stream->status = STREAM_INIT;
	stream->prev = STREAM_UN_INIT;
	stream->ops = ops;
	stream->data_blk.on = false;
	stream->data_blk.condition = false;
	stream->data_blk.ret_code = 0;
	stream->data_blk.data = NULL;
	stream->ctrl_blk.on = false;
	stream->ctrl_blk.condition = false;
	stream->ctrl_blk.ret_code = 0;
	stream->ctrl_blk.data = NULL;
}


/*
 * sst_validate_strid - this function validates the stream id
 *
 * @str_id : stream id to be validated
 *
 * returns 0 if valid stream
 */
static inline int sst_validate_strid(int str_id)
{
	if (str_id <= 0 || str_id > sst_drv_ctx->max_streams) {
		pr_err("SST ERR: invalid stream id : %d\n",
					str_id);
		return -EINVAL;
	} else
		return 0;
}

static inline int sst_shim_write(void __iomem *addr, int offset, int value)
{

	if (sst_drv_ctx->pci_id == SST_MRST_PCI_ID)
		writel(value, addr + SST_ISRD);	/*dummy*/
	writel(value, addr + offset);
	return 0;
}

static inline u32 sst_shim_read(void __iomem *addr, int offset)
{

	return readl(addr + offset);
}


static inline int sst_shim_write64(void __iomem *addr, int offset, u64 value)
{
	memcpy_toio(addr + offset, &value, sizeof(value));
	return 0;
}

static inline u64 sst_shim_read64(void __iomem *addr, int offset)
{
	u64 val;

	memcpy_fromio(&val, addr + offset, sizeof(val));
	return val;
}


static inline void
sst_set_fw_state_locked(struct intel_sst_drv *sst_drv_ctx, int sst_state)
{
	mutex_lock(&sst_drv_ctx->sst_lock);
	sst_drv_ctx->sst_state = sst_state;
	mutex_unlock(&sst_drv_ctx->sst_lock);
}
static inline struct stream_info *get_stream_info(int str_id)
{
	if (sst_validate_strid(str_id))
		return NULL;
	return &sst_drv_ctx->streams[str_id];
}
int register_sst(struct device *);
int unregister_sst(struct device *);

#ifdef CONFIG_DEBUG_FS
void sst_debugfs_init(struct intel_sst_drv *sst);
void sst_debugfs_exit(struct intel_sst_drv *sst);
#else
static inline void sst_debugfs_init(struct intel_sst_drv *sst)
{
}

static inline void sst_debugfs_exit(struct intel_sst_drv *sst)
{
}
#endif /* CONFIG_DEBUG_FS */
#endif /* __INTEL_SST_COMMON_H__ */
