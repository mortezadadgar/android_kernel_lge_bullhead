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
  * ANY WARRANTY; without evenp the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  * more details.
  *
  * You should have received a copy of the GNU General Public License along with
  * this program; if not, write to the Free Software Foundation, Inc.,
  * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  */
#ifndef MID_I2S_H_
#define MID_I2S_H_

#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_i2s_if.h>

#ifdef CONFIG_X86_MRFLD
//#define __MRFL_SPECIFIC__
#undef __MRFL_SPECIFIC__

/* For temporary MRFL work-around */
/* To Be Removed when fixed */
#define __MRFL_SPECIFIC_TMP__
#endif /* CONFIG_X86_MRFLD */

/*
 * Main Defines
 */
#define DRIVER_NAME "I2S SSP Driver"

/* Moorestone */
#define MRST_SSP0_DEVICE_ID     0x0815
#define MRST_LPE_DMA_DEVICE_ID  0x0814

/* Medfield */
#define MFLD_SSP0_DEVICE_ID     0x0832
#define MFLD_SSP1_DEVICE_ID     0x0825
#define MFLD_LPE_DMA_DEVICE_ID  0x0830

/* Cloverview */
#define CLV_SSP0_DEVICE_ID	0x08F1
#define CLV_SSP1_DEVICE_ID	0x08E8
#define CLV_LPE_DMA_DEVICE_ID	0x08F0

/* Merrifield */
#define MRFL_SSP_DEVICE_ID      0x1193
#define MRFL_LPE_DMA_DEVICE_ID  0x119B

#ifdef __MRFL_SPECIFIC_TMP__
/* FIXME: use of fixed addresses should be
 * replaced by a call to SST driver that will
 * take care to access LPE Shim registers */
#define MRFL_LPE_SHIM_REG_BASE_ADDRESS	(0xff340000)
#define MRFL_LPE_SHIM_REG_SIZE		(0xE8)
#endif /* __MRFL_SPECIFIC_TMP__ */

#define BYT_LPE_SHIM_BASE_ADDRESS 0x140000
#define BYT_LPE_SHIM_SIZE 		0x100
#define BYT_SSP_SIZE 			0x80
#define BYT_SSP0_START_ADDRESS 	0X0A0000
#define BYT_SSP1_START_ADDRESS 	0X0A1000
#define BYT_SSP2_START_ADDRESS 	0X0A2000

#define BYT_LPE_SHIM_IAPIS_SSP0_BIT 0x8
#define BYT_LPE_SHIM_IAPIS_SSP1_BIT 0x10
#define BYT_LPE_SHIM_IAPIS_SSP2_BIT 0x20

#define BYT_SSP0_INTERNAL_ADDRESS	(0xff2a0000)
#define BYT_SSP1_INTERNAL_ADDRESS	(0xff2a1000)
#define BYT_SSP2_INTERNAL_ADDRESS	(0xff2a2000)

/* SSP PCI device definitions */
#define MRST_SSP_BAR	0
#define MRST_LPE_BAR	1

#define SSP0_INSTANCE	0
#define SSP1_INSTANCE	1
#define SSP2_INSTANCE	2

/* SSP mode */
#define SSP_IN_MASTER_MODE		0x0
#define SSP_IN_SLAVE_MODE		0x1

/* SSP state */
#define SSP_OFF 0
#define SSP_ON  1


/*
 *	Macros: Register definitions
 */
#define DEFINE_REG(reg, offset) \
	const u8 OFFSET_ ## reg = offset; \
	static inline u32 read_ ## reg(void *p) \
		{ return __raw_readl(p + (OFFSET_ ## reg)); } \
	static inline void write_ ## reg(u32 v, void *p) \
		{ __raw_writel(v, p + (OFFSET_ ## reg)); }

/*
 *	Macros: Registers field definitions
 */
#define DEFINE_FIELD(reg, field, mask, shift) \
	const u32 reg ## _ ## field ## _MASK = (u32)(mask);  \
	const u8  reg ## _ ## field ## _SHIFT = (u8)(shift);  \
							\
	static inline u32 extract_ ## reg ## _ ## field \
				(u32 reg_value) { \
		return  ((reg_value) >> reg ## _ ## field ## _SHIFT) \
				& reg ## _ ## field ## _MASK; \
	} \
	  \
	static inline u32 replace_ ## reg ## _ ## field \
				(u32 reg_value, u32 field_value) { \
		return  (((field_value) & reg ## _ ## field ## _MASK) \
			<< reg ## _ ## field ## _SHIFT) \
			| ((reg_value) & ~(reg ## _ ## field ## _MASK \
			<< reg ## _ ## field ## _SHIFT)); \
	} \
	  \
	static inline u32 set_ ## reg ## _ ## field \
				(u32 reg_value) { \
		return  ((reg ## _ ## field ## _MASK) \
			<< reg ## _ ## field ## _SHIFT) \
			| ((reg_value) & ~(reg ## _ ## field ## _MASK \
			<< reg ## _ ## field ## _SHIFT)); \
	}


/*
 * LPE registers definitions
 * -------------------------
 */
DEFINE_REG(LPE_CSR, 0x00)		//BYT: ok
DEFINE_REG(LPE_PISR, 0x08)		//BYT: ok
DEFINE_REG(LPE_PIMR, 0x10)		//BYT: ok
DEFINE_REG(LPE_ISRX, 0x18)		//BYT: ok
DEFINE_REG(LPE_IMRX, 0x28)		//BYT: ok
DEFINE_REG(LPE_IPCX, 0x38)	/* IPC IA-SST */		//BYT: ok
DEFINE_REG(LPE_IPCD, 0x40)	/* IPC SST-IA */		//BYT: ok
DEFINE_REG(LPE_ISRD, 0x20)	/* dummy register for*/		//BYT: ok
				/* shim workaround   */
DEFINE_REG(LPE_CLKCTL, 0x78)

/* BYT LPE Shim for M/N Clock Divider control
 * These are 64 bit regs.
 * Write lower nibble then write upper nibble
 */
DEFINE_REG(LPE_SSP0_DIVC_L, 0xE8)
DEFINE_REG(LPE_SSP0_DIVC_H, 0xEC)

DEFINE_REG(LPE_SSP1_DIVC_L, 0xF0)
DEFINE_REG(LPE_SSP1_DIVC_H, 0xF4)

DEFINE_REG(LPE_SSP2_DIVC_L, 0xF8)
DEFINE_REG(LPE_SSP2_DIVC_H, 0xFC)

/* LPE_ISRX fields definitions */
DEFINE_FIELD(LPE_ISRX, IAPIS_SSP0, 0x01, 3);
DEFINE_FIELD(LPE_ISRX, IAPIS_SSP1, 0x01, 4);
DEFINE_FIELD(LPE_ISRX, IAPIS_SSP2, 0x01, 5);


/*
 * SSP registers definitions
 * -------------------------
 */
DEFINE_REG(SSCR0, 0x00)
DEFINE_REG(SSCR1, 0x04)
DEFINE_REG(SSSR, 0x08)
DEFINE_REG(SSITR, 0x0C)
DEFINE_REG(SSDR, 0x10)
DEFINE_REG(SSTO, 0x28)
DEFINE_REG(SSPSP, 0x2C)
DEFINE_REG(SSTSA, 0x30)	/* SSP Tx Timeslot Active */
DEFINE_REG(SSRSA, 0x34)	/* SSP Rx Timeslot Active */
DEFINE_REG(SSTSS, 0x38)
DEFINE_REG(SSACD, 0x3C)

/*BYT*/
DEFINE_REG(SSCR2, 0x40)
DEFINE_REG(SSCR3, 0x70)
DEFINE_REG(SSCR4, 0x74)
DEFINE_REG(SSCR5, 0x78)
DEFINE_REG(SSFS, 0x44)
DEFINE_REG(SFIFOL, 0x68)
DEFINE_REG(SFIFOTT, 0x6C)

#ifdef __MRFL_SPECIFIC__
DEFINE_REG(SSCR2, 0x40)
DEFINE_REG(SSFS, 0x44)
DEFINE_REG(FRAME_CNT0, 0x48)
DEFINE_REG(FRAME_CNT1, 0x4C)
DEFINE_REG(FRAME_CNT2, 0x50)
DEFINE_REG(FRAME_CNT3, 0x54)
DEFINE_REG(FRAME_CNT4, 0x58)
DEFINE_REG(FRAME_CNT5, 0x5C)
DEFINE_REG(FRAME_CNT6, 0x60)
DEFINE_REG(FRAME_CNT7, 0x64)
DEFINE_REG(SFIFOL, 0x68)
DEFINE_REG(SFIFOTT, 0x6C)
DEFINE_REG(SSCR3, 0x70)
DEFINE_REG(SSCR4, 0x74)
DEFINE_REG(SSCR5, 0x78)
#endif /* __MRFL_SPECIFIC__ */

/* SSP SSCR0 fields definitions */
DEFINE_FIELD(SSCR0, DSS, 0x0F, 0);	/* Data Size Select [4..16] */
DEFINE_FIELD(SSCR0, FRF, 0x03, 4);	/* Frame Format */
DEFINE_FIELD(SSCR0, ECS, 0x01, 6);	/* External clock select */
DEFINE_FIELD(SSCR0, SSE, 0x01, 7);	/* Synchronous Serial Port Enable */
DEFINE_FIELD(SSCR0, SCR, 0xFFF, 8);	/* Not implemented */
DEFINE_FIELD(SSCR0, EDSS, 0x1, 20);	/* Extended data size select */
DEFINE_FIELD(SSCR0, NCS, 0x1, 21);	/* Network clock select */
DEFINE_FIELD(SSCR0, RIM, 0x1, 22);	/* Receive FIFO overrrun int mask */
DEFINE_FIELD(SSCR0, TIM, 0x1, 23);	/* Transmit FIFO underrun int mask */
DEFINE_FIELD(SSCR0, FRDC, 0x7, 24);	/* Frame Rate Divider Control */
DEFINE_FIELD(SSCR0, ACS, 0x1, 30);	/* Audio clock select */
DEFINE_FIELD(SSCR0, MOD, 0x1, 31);	/* Mode (normal or network) */

#define SSCR0_DataSize(x)     ((x) - 1)	/* Data Size Select [4..16] */
#define SSCR0_SlotsPerFrm(x)  ((x) - 1)	/* Time slots per frame */
#define SSCR0_SerClkDiv(x)    ((x) - 1)	/* Divisor [1..4096],... */
					/*...not implemented on Langwell */

/* SSP SSCR1 fields definitions */
DEFINE_FIELD(SSCR1, TTELP, 0x1, 31);	/* TXD Tristate Enable on Last Phase */
DEFINE_FIELD(SSCR1, TTE, 0x1, 30);	/* TXD Tristate Enable */
DEFINE_FIELD(SSCR1, EBCEI, 0x1, 29);	/* Enable Bit Count Error Interrupt */
DEFINE_FIELD(SSCR1, SCFR, 0x1, 28);	/* Slave Clock Running */
DEFINE_FIELD(SSCR1, ECRA, 0x1, 27);	/* Enable Clock Request A */
DEFINE_FIELD(SSCR1, ECRB, 0x1, 26);	/* Enable Clock Request B */
DEFINE_FIELD(SSCR1, SCLKDIR, 0x1, 25);	/* SSPCLK Direction */
DEFINE_FIELD(SSCR1, SFRMDIR, 0x1, 24);	/* SSPFRM Direction */
DEFINE_FIELD(SSCR1, RWOT, 0x1, 23);	/* Receive without Transmit */
DEFINE_FIELD(SSCR1, TRAIL, 0x1, 22);	/* Trailing Byte */
DEFINE_FIELD(SSCR1, TSRE, 0x1, 21);	/* DMA Tx Service Request Enable */
DEFINE_FIELD(SSCR1, RSRE, 0x1, 20);	/* DMA Rx Service Request Enable */
DEFINE_FIELD(SSCR1, TINTE, 0x1, 19);	/* Receiver Timeout Interrupt Enable */
DEFINE_FIELD(SSCR1, PINTE, 0x1, 18);	/* Periph. Trailing Byte Int. Enable */
DEFINE_FIELD(SSCR1, IFS, 0x1, 16);	/* Invert Frame Signal */
DEFINE_FIELD(SSCR1, STRF, 0x1, 15);	/* Select FIFO for EFWR: test mode */
DEFINE_FIELD(SSCR1, EFWR, 0x1, 14);	/* Enable FIFO Write/Read: test mode */
DEFINE_FIELD(SSCR1, RFT, 0xF, 10);	/* Receive FIFO Trigger Threshold */
DEFINE_FIELD(SSCR1, TFT, 0xF, 6);	/* Transmit FIFO Trigger Threshold */
DEFINE_FIELD(SSCR1, MWDS, 0x1, 5);	/* Microwire Transmit Data Size */
DEFINE_FIELD(SSCR1, SPH, 0x1, 4);	/* Motorola SPI SSPSCLK phase setting*/
DEFINE_FIELD(SSCR1, SPO, 0x1, 3);	/* Motorola SPI SSPSCLK polarity */
DEFINE_FIELD(SSCR1, LBM, 0x1, 2);	/* Loopback mode: test mode */
DEFINE_FIELD(SSCR1, TIE, 0x1, 1);	/* Transmit FIFO Interrupt Enable */
DEFINE_FIELD(SSCR1, RIE, 0x1, 0);	/* Receive FIFO Interrupt Enable */

#define SSCR1_RxTresh(x) ((x) - 1)	/* level [1..16] */
#define SSCR1_TxTresh(x) (x)//((x) - 1)	/* level [1..16] */

/* SSP SSCR2 fields definition */
DEFINE_FIELD(SSCR2, ACG_EN, 0x1, 4);	/* ACG En/Dis base on fabric, FIFO and ctrl sig activity */
DEFINE_FIELD(SSCR2, CLK_DEL_EN, 0x1, 3);/* En/Dis 1/2 clock delay logic for capturing data*/
DEFINE_FIELD(SSCR2, SLV_EXT_CLK_RUN_EN, 0x1, 2); /*Enable free running clock to get state machine*/
DEFINE_FIELD(SSCR2, UNDERRUN_FIX_1, 0x1, 1); /*New data start on underrun slot*/
DEFINE_FIELD(SSCR2, UNDERRUN_FIX_0, 0x1, 1); /*New data start on slot 0*/

/* SSP SSCR3 fields definition */
DEFINE_FIELD(SSCR3, MST_CLK_EN, 0x1, 16);		/*Emulate I2S or LJ for single slot master mode*/
DEFINE_FIELD(SSCR3, STRETCH_RX, 0x1, 15);		/*Write 1 to preserve default value */
DEFINE_FIELD(SSCR3, STRETCH_TX, 0x1, 14);		/*Write 1 to preserve default value */
DEFINE_FIELD(SSCR3, I2S_RX_EN, 0x1, 10);		/*Enable data RX on both I2S slots*/
DEFINE_FIELD(SSCR3, I2S_TX_EN, 0x1, 9);			/*Enable data TX on both I2S slots*/
DEFINE_FIELD(SSCR3, I2S_RX_SS_FIX_EN, 0x1, 4);	/*Enable RX Slow Swap fix*/
DEFINE_FIELD(SSCR3, I2S_TX_SS_FIX_EN, 0x1, 3);	/*Enable TX Slow Swap fix*/
DEFINE_FIELD(SSCR3, I2S_MODE_EN, 0x1, 1); 		/*Enables I2S mode*/
DEFINE_FIELD(SSCR3, FRM_MST_EN, 0x1, 0); 		/*Set for I2S and PCM Master mode ONLY*/

/* SSP SSPSP fields definitions */	//------------------------------ BYT: ok
DEFINE_FIELD(SSPSP, FSRT, 0x1, 25);		/* Frame Sync Relative Timing Bit */
DEFINE_FIELD(SSPSP, DMYSTOP, 0x3, 23);	/* Dummy Stop in Num of SSPSCLKs:T4 */
DEFINE_FIELD(SSPSP, SFRMWDTH, 0x3F, 16);/* Serial Frame width : T6 */
DEFINE_FIELD(SSPSP, SFRMDLY, 0x7F, 9);	/* Serial Fr Delay in 1/2SSPSCLKs:T5 */
DEFINE_FIELD(SSPSP, DMYSTRT, 0x3, 7);	/* Dummy Start in Number of SSPSCLK */
					/* after STRTDLY, T2 */
					/*(master mode only) */
DEFINE_FIELD(SSPSP, STRTDLY, 0x7, 4);	/* Start Delay, T1 (master mode only)*/
DEFINE_FIELD(SSPSP, ETDS, 0x1, 3);		/* End of Transfer Data State */
DEFINE_FIELD(SSPSP, SFRMP, 0x1, 2);		/* Serial Frame Polarity */
DEFINE_FIELD(SSPSP, SCMODE, 0x3, 0);	/* Serial bit-rate Clock Mode */

/* SSP SSTSA fields definitions */
DEFINE_FIELD(SSTSA, TTSA, 0xFF, 0);	/*  */
/* SSP SSRSA fields definitions */
DEFINE_FIELD(SSRSA, RTSA, 0xFF, 0);	/*  */

/* SSP SSSR fields definitions */
DEFINE_FIELD(SSSR, BCE, 0x1, 23);	/* Bit Count Error: RW 1 to Clear */
DEFINE_FIELD(SSSR, CSS, 0x1, 22);	/* Clock Synchronization Status */
DEFINE_FIELD(SSSR, TUR, 0x1, 21);	/* Tx FIFO UnderRun: RW 1 to Clear */
DEFINE_FIELD(SSSR, EOC, 0x1, 20);	/* End Of Chain: RW 1 to Clear */
DEFINE_FIELD(SSSR, TINT, 0x1, 19);	/* Receiver Time-out Interrupt: */
					/* Read/Write 1 to Clear */
DEFINE_FIELD(SSSR, PINT, 0x1, 18);	/* Periph. Trailing Byte Interrupt: */
					/* Read/Write 1 to Clear */
DEFINE_FIELD(SSSR, RFL, 0xF, 12);	/* Receive FIFO Level */
DEFINE_FIELD(SSSR, TFL, 0xF, 8);	/* Transmit FIFO Level */
DEFINE_FIELD(SSSR, ROR, 0x1, 7);	/* Rx FIFO Overrun: RW 1 to Clear */
DEFINE_FIELD(SSSR, RFS, 0x1, 6);	/* Receive FIFO Service Request */
DEFINE_FIELD(SSSR, TFS, 0x1, 5);	/* Transmit FIFO Service Request */
DEFINE_FIELD(SSSR, BSY, 0x1, 4);	/* SSP Busy */
DEFINE_FIELD(SSSR, RNE, 0x1, 3);	/* Receive FIFO not empty */
DEFINE_FIELD(SSSR, TNF, 0x1, 2);	/* Transmit FIFO not Full */

#ifdef __MRFL_SPECIFIC__
/* SSP SFIFOL fields definitions */
DEFINE_FIELD(SFIFOL, RFL, 0xFFFF, 16)	/* Receive FIFO Level */
DEFINE_FIELD(SFIFOL, TFL, 0xFFFF, 0)	/* Transmit FIFO Level */

/* SSP SFIFOL fields definitions */
DEFINE_FIELD(SFIFOTT, RFT, 0xFFFF, 16)	/* Receive FIFO Trigger Threshold */
DEFINE_FIELD(SFIFOTT, TFT, 0xFFFF, 0)	/* Transmit FIFO Trigger Threshold */
#endif /* __MRFL_SPECIFIC__ */

/* SSP SSCR4 fields definitions */
DEFINE_FIELD(SSCR4, TOT_FRM_PRD, 0x1FF, 7); /* The total frame period (asserted and de-asserted) */
/* SSP SSCR5 fields definitions */
DEFINE_FIELD(SSCR5, FRM_ASRT_WIDTH, 0x1FFFFFF, 1); /* width of the asserted period of frame (actual=value+1)*/


/* SSP MN DIV Controller field definitions */
//DEFINE_REG(LPE_SSP0_DIVC_L, 0xE8)
//DEFINE_REG(LPE_SSP0_DIVC_H, 0xEC)
DEFINE_FIELD (LPE_SSP0_DIVC_H, DIV_BYPASS, 0x1, 31)
DEFINE_FIELD (LPE_SSP0_DIVC_H, DIV_EN, 0x1, 30)
DEFINE_FIELD (LPE_SSP0_DIVC_H, DIV_UPDATE, 0x1, 29)
DEFINE_FIELD (LPE_SSP0_DIVC_H, DIV_M, 0xFFFFF, 0)
DEFINE_FIELD (LPE_SSP0_DIVC_L, DIV_N, 0xFFFFF, 0)


/*
 *	list of differents types of SSP, value depends of adid entry of
 *	capability ID of the PCI
 */

/*
 *
 * The PCI header associated to SSP devices now includes a configuration
 * register. It provides information to a driver which is probed for the
 * SSP, specifying in which way the SSP is supposed to be used. Here is
 * the format of this byte register:
 *
 *	bits 1..0: Mode
 *		00=0x0 : Invalid, the register should be ignored
 *		01=0x1 : SSP to be used as SPI controller
 *		10=0x2: SSP to be used in I2S/ISS mode
 *		other: Reserved
 *
 *	bits 4..2: Configuration
 *	In I2S/ISS mode:
 *		000=0x0: Invalid
 *		001=0x1: Bluetooth
 *		010=0x2: Modem
 *		other: Reserved
 *	In SPI mode:
 *		Value is the SPI bus number connected to the SSP.
 *		To be used for registration to the Linux SPI
 *		framework.
 *	bit 5: SPI slave
 *	Relevant in SPI mode only. If set, indicates the SPI clock
 *	is not provided by the SSP: SPI slave mode.
 *
 *	bit 6..13: SSP_FS pin GPIO Mapping
 *	bit 14..15: SSP_FS pin Mode
 *
 * This configuration register is implemented in the adid field of the
 * Vendor Specific PCI capability associated to the SSP.
 *
 */

#define PCI_ADID_SSP_MODE_SPI  (1)
#define PCI_ADID_SSP_MODE_I2S  (2)

#define PCI_ADID_SSP_CONF_BT_FM  (1<<2)
#define PCI_ADID_SSP_CONF_MODEM  (2<<2)

#define PCI_ADID_SSP_FS_GPIO_MAPPING_SHIFT 6
#define PCI_ADID_SSP_FS_GPIO_MAPPING_MASK 0xFF

#define PCI_ADID_SSP_FS_GPIO_MODE_SHIFT 14
#define PCI_ADID_SSP_FS_GPIO_MODE_MASK 0x3



#define PCI_CAP_ADID_I2S_BT_FM  ((PCI_ADID_SSP_CONF_BT_FM) | \
					 (PCI_ADID_SSP_MODE_I2S))
#define PCI_CAP_ADID_I2S_MODEM  ((PCI_ADID_SSP_CONF_MODEM) | \
					 (PCI_ADID_SSP_MODE_I2S))

/* bit I2S_PORT_OPENED lock for open/close
 * bit I2S_PORT_READ_BUSY lock for read requests (serialized)
 * bit I2S_PORT_WRITE_BUSY lock for write requests (serialized)
 * bit I2S_PORT_CLOSING means close on going, waiting for pending callbacks.
 * bit I2S_PORT_COMPLETE_WRITE means deferred irq process is required
 *			to complete the Tx request when not using DMA
 * bit I2S_PORT_COMPLETE_READ means deferred irq process is required
 *			to complete the Rx request when not using DMA
 */
enum i2s_flags {
	I2S_PORT_OPENED,
	I2S_PORT_WRITE_BUSY,
	I2S_PORT_READ_BUSY,
	I2S_PORT_CLOSING,
	I2S_PORT_COMPLETE_WRITE,
	I2S_PORT_COMPLETE_READ
};

/*
 * variable "modem_found_and_i2s_setup_ok"
 * bit to set if modem is found on platform
 */
#define MODEM_FND 0

/*
 * variable "clv_ssps_found"
 * bits to set if CLV SSP0/1 found on platform
 */
#define CLV_SSP0_FND 0
#define CLV_SSP1_FND 1

#define FIFO_SIZE 16
/*
 *	Structures Definition
 */

/**
 * struct intel_mid_i2s_data - context struct to keep SSP I2S data
 * @pdev: pci dev pointer corresponding to context
 * @paddr:
 * @ioaddr:
 * @iolen:
 * @irq:
 * @clear_sr:
 * @mask_sr:
 * @dmac1:
 * @dmas_tx: dma slave structure for transmit
 * @dmas_rx: dma slave structure for receive
 * @txchan: Dma channel for transmit
 * @rxchan: Dma channel for receive
 *
 * @read_done:
 * @read_dst:
 * @read_len:
 *
 * @write_done:
 * @write_src:
 * @write_len:
 *
 * @mutex:  a mutex to make sure we have once-at-time critical functions.
 *
 * Longer description
 */

/* Locking rules:
 *
 * All the fields, not listed below, are set during probe, and then read only
 * So they do not require locking
 *
 * The fields that require locking are related to the I2S read and write
 * requests.
 *
 * We allow only 1 read at a time, and 1 write at a time.
 * We allow read in parallel of write but use separate variables.
 * We allow only 1 user per SSP/I2S port.
 * Typically this user will be a dedicated PulseAudio RT thread communicating
 * with cmt-speech driver which in turns communicates with intel_mid_ssp
 * driver.
 * PCM mixing is done before access to kernel drivers;typically within
 * PulseAudio or after; typically within the modem.
 * So no concurrent users, per I2S channel, to this driver are allowed
 * The read & write are triggered from a USER context
 * The read & write callbacks are called from a BH context
 * You should have not callback pending before calling close, close will wait
 * for remaining callback calls.
 * It is not allowed to call close function from read/write callback threads.
 *
 * Locking is handled via drv_data->flags & atomic bitwise operations
 *
 * I2S0 is dedicated for PCM transfer to/from the modem module
 * I2S1 is dedicated for PCM transfer to/from the Bluetooth or FM module
 *
 * read_done:
 * read_len:
 * read_dst:
 *
 * write_done:
 * write_src:
 * write_len:
 *
 * mutex:  a mutex to make sure we have once-at-time critical functions.
 *		once-at-a-time actions functions are:
 *			-intel_mid_i2s_open
 *			-intel_mid_i2s_close
 *			-intel_mid_i2s_rd_req
 *			-intel_mid_i2s_wr_req
 *			-intel_mid_i2s_set_rd_cb
 *			-intel_mid_i2s_set_wr_cb
 * These functions should not be called during a lock() neither in interrupt.
 */

struct intel_mid_i2s_hdl {
	/* Driver model hookup */
	struct pci_dev *pdev;
	/* register addresses */
	dma_addr_t paddr;
	/* SSP internal address */
	dma_addr_t pinternal_addr;
	void __iomem *ioaddr;
	/* lpe shim */
	void __iomem *shim;

	u32 iolen;
	int irq;

	/* SSP masks */
	u32 clear_sr;
	u32 mask_sr;

	/* SSP Configuration */
	/* DMA info */
	struct pci_dev *dmac1;

	struct intel_mid_dma_slave dmas_tx;
	struct intel_mid_dma_slave dmas_rx;
	struct dma_chan *txchan;
	struct dma_chan *rxchan;

	struct scatterlist *rxsgl;
	struct scatterlist *txsgl;

	unsigned int device_instance;

	/* Call back functions */
	int	(*read_callback)(void *param);
	int	(*write_callback)(void *param);
	size_t	read_len;	/* read_len > 0 <=> read_dma_running */
	size_t	write_len;	/* write_len > 0 <=> read_dma_running */
	void	*read_param;	/* context param for callback */
	void	*write_param;	/* context param for callback */
	union {
		dma_addr_t	dma;
		//BYT ORI: u16		*cpu;	/* DO_NOT_USE_DMA mode */
		u32 *cpu;
	} read_ptr;
	union {
		dma_addr_t	dma;
		//BYT ORI: u16		*cpu;	/* DO_NOT_USE_DMA mode */
		u32 *cpu;
	} write_ptr;

	unsigned long flags;
	struct mutex mutex;
	enum intel_mid_i2s_ssp_usage usage;

	struct intel_mid_i2s_settings current_settings;

};

struct intel_mid_ssp_gpio {
	u16 ssp_fs_gpio_mapping;
	u8 ssp_fs_mode;
};

static int wr_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param);
static int wr_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param);
static int rd_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param);
static int rd_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param);

static void i2s_read_done(void *arg);
static void i2s_write_done(void *arg);
static void i2s_lli_read_done(void *arg);
static void i2s_lli_write_done(void *arg);

static bool chan_filter(struct dma_chan *chan, void *param);
static void ssp1_dump_registers(struct intel_mid_i2s_hdl *);

static
irqreturn_t i2s_irq(int irq, void *dev_id);
static inline
irqreturn_t i2s_irq_handle_RFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr);
static inline
irqreturn_t i2s_irq_handle_TFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr);
static
irqreturn_t i2s_irq_deferred(int irq, void *dev_id);

//static int check_device(struct device *device_ptr, void *data);
static void set_ssp_i2s_hw(struct intel_mid_i2s_hdl *drv_data,
			const struct intel_mid_i2s_settings *ps_settings);


#ifdef CONFIG_PM
static int intel_mid_i2s_runtime_resume(struct device *device_ptr);
static int intel_mid_i2s_runtime_suspend(struct device *device_ptr);
#endif
int intel_mid_i2s_probe(struct pci_dev *pdev,
		const struct pci_device_id *ent);
/*static void intel_mid_i2s_remove(struct pci_dev *pdev);*/

/*static int bt_pcm_dma_init(struct intel_mid_i2s_hdl *drv_data);*/

void intel_mid_i2s_set_modem_probe_cb(void(*probe_cb)(void));
void intel_mid_i2s_set_modem_remove_cb(void(*remove_cb)(void));

#ifdef CONFIG_PM
static int  intel_mid_i2s_suspend(struct device *dev);
static int intel_mid_i2s_resume(struct device *dev);
#endif

/*
 * These define will clarify source code when accessing SSCRx registers
 */
#define SSCR0_reg(regbit, value)					\
	(((value) & SSCR0_##regbit##_MASK) << SSCR0_##regbit##_SHIFT)

#define SSCR1_reg(regbit, value)					\
	(((value) & SSCR1_##regbit##_MASK) << SSCR1_##regbit##_SHIFT)

#define SSPSP_reg(regbit, value)					\
	(((value) & SSPSP_##regbit##_MASK) << SSPSP_##regbit##_SHIFT)

#define SSRSA_reg(regbit, value)					\
	(((value) & SSRSA_##regbit##_MASK) << SSRSA_##regbit##_SHIFT)
#define SSTSA_reg(regbit, value)					\
	(((value) & SSTSA_##regbit##_MASK) << SSTSA_##regbit##_SHIFT)

#define SSCR4_reg(regbit, value)					\
	(((value) & SSCR4_##regbit##_MASK) << SSCR4_##regbit##_SHIFT)

#define SSCR5_reg(regbit, value)					\
	(((value) & SSCR5_##regbit##_MASK) << SSCR5_##regbit##_SHIFT)

#define LPE_SSP0_DIVC_H_reg(regbit, value)					\
	(((value) & LPE_SSP0_DIVC_H_##regbit##_MASK) << LPE_SSP0_DIVC_H_##regbit##_SHIFT)

#define LPE_SSP0_DIVC_L_reg(regbit, value)					\
	(((value) & LPE_SSP0_DIVC_L_##regbit##_MASK) << LPE_SSP0_DIVC_L_##regbit##_SHIFT)


#define change_SSCR0_reg(reg_pointer, regbit, value)			  \
	write_SSCR0((read_SSCR0(reg_pointer)				  \
	& (~((SSCR0_##regbit##_MASK << SSCR0_##regbit##_SHIFT))))	  \
	| (((value) & SSCR0_##regbit##_MASK) << SSCR0_##regbit##_SHIFT),  \
	reg_pointer);

#define set_SSCR0_reg(reg_pointer, regbit)				  \
	write_SSCR0(read_SSCR0(reg_pointer)				  \
	| (SSCR0_##regbit##_MASK << SSCR0_##regbit##_SHIFT),		\
	reg_pointer);

#define clear_SSCR0_reg(reg_pointer, regbit)				  \
	write_SSCR0((read_SSCR0(reg_pointer)				  \
	& (~((SSCR0_##regbit##_MASK << SSCR0_##regbit##_SHIFT)))),	  \
	reg_pointer);

#define change_SSCR1_reg(reg_pointer, regbit, value)			  \
	write_SSCR1((read_SSCR1(reg_pointer)				  \
	& (~((SSCR1_##regbit##_MASK << SSCR1_##regbit##_SHIFT))))	  \
	| (((value) & SSCR1_##regbit##_MASK) << SSCR1_##regbit##_SHIFT),  \
	reg_pointer);

#define set_SSCR1_reg(reg_pointer, regbit)				  \
	write_SSCR1(read_SSCR1(reg_pointer)				  \
	| (SSCR1_##regbit##_MASK << SSCR1_##regbit##_SHIFT),		  \
	reg_pointer);

#define clear_SSCR1_reg(reg_pointer, regbit)				  \
	write_SSCR1((read_SSCR1(reg_pointer)				  \
	& (~((SSCR1_##regbit##_MASK << SSCR1_##regbit##_SHIFT)))),	  \
	reg_pointer);

#define change_SSCR3_reg(reg_pointer, regbit, value)			  \
	write_SSCR3((read_SSCR3(reg_pointer)				  \
	& (~((SSCR3_##regbit##_MASK << SSCR3_##regbit##_SHIFT))))	  \
	| (((value) & SSCR3_##regbit##_MASK) << SSCR3_##regbit##_SHIFT),  \
	reg_pointer);

#define set_SSCR3_reg(reg_pointer, regbit)				  \
	write_SSCR3(read_SSCR3(reg_pointer)				  \
	| (SSCR3_##regbit##_MASK << SSCR3_##regbit##_SHIFT),		\
	reg_pointer);

#define clear_SSCR3_reg(reg_pointer, regbit)				  \
	write_SSCR3((read_SSCR3(reg_pointer)				  \
	& (~((SSCR3_##regbit##_MASK << SSCR3_##regbit##_SHIFT)))),	  \
	reg_pointer);

/* RX FIFO level */
#define GET_SSSR_val(x, regb)						  \
	((x & (SSSR_##regb##_MASK<<SSSR_##regb##_SHIFT))>>SSSR_##regb##_SHIFT)


/*
 * SSP hardware can be configured as I2S, PCM, SPI...
 * In order to allow flexibility without modifying the software driver, the
 * PCI header uses the configuration register 'adid':
 *
 * The PCI header associated to SSP devices includes a configuration register.
 * It provides information to a driver which is probed for the SSP, specifying
 * in which way the SSP is supposed to be used.
 * Here is the format of this configuration register (8 bits):
 *
 *   bits 2..0: Mode
 *       000: Invalid, the register should be ignored
 *       001: SSP to be used as SPI controller
 *       010: SSP to be used in I2S/ISS mode
 *       other: Reserved
 *
 *   bits 5..3: Configuration
 *       In I2S/ISS mode:
 *               000: Invalid
 *               001: Bluetooth
 *               010: Modem
 *               other: Reserved
 *       In SPI mode:
 *               Value is the SPI bus number connected to the SSP.
 *               To be used for registration to the Linux SPI
 *               framework.
 *
 *   bit 6: SPI slave
 *       Relevant in SPI mode only. If set, indicates the SPI clock
 *       is not provided by the SSP: SPI slave mode.
 *
 *   bit 7: Reserved (0)
 *
 *   This configuration register is implemented in the adid field of the
 *   Vendor Specific PCI capability associated to the SSP. The format of
 *   this capability is:
 *
 *   uint8_t     capId;              < Capability ID (vendor-specific)
 *   uint8_t     nextCap;            < Next Item Ptr
 *   uint8_t     length;             < Size of this capability (7)
 *   uint8_t     version;            < Version of this capability (1)
 *   uint8_t     lss;                < Logical subsystem info
 *                                         Bit 7 = PMU (0 = NC, 1 = SC)
 *                                         Bits 6:0 = LSS ID
 *   uint8_t     apmc;               < Additional PM capabilities
 *                                         Bit 7 = Rsvd
 *                                         Bit 6 = Wake capable
 *                                         Bit 5 = D3 support
 *                                         Bit 4 = D2 support
 *                                         Bit 3 = D1 support
 *                                         Bit 2 = D0i3 support
 *                                         Bit 1 = D0i2 support
 *                                         Bit 0 = D0i1 support
 *   uint16_t     adid;              < Additional device ID (dev-specific)
 *
 *   The capability data are in the PCI configuration space and the
 *   adid field can be modified using BMP tool.
 */
/* ADDID = Additional Device ID */
#define PCI_CAP_OFFSET_ADID 6



#define SSP_PLL_FREQ_05_622 (0<<4)
#define SSP_PLL_FREQ_11_345 (1<<4)
#define SSP_PLL_FREQ_12_235 (2<<4)
#define SSP_PLL_FREQ_14_847 (3<<4)
#define SSP_PLL_FREQ_32_842 (4<<4)
#define SSP_PLL_FREQ_48_000 (5<<4)

#define SSP_SYSCLK_DIV4_BYPASS (1<<3)

#define SSP_SYSCLK_DIV_1 (0<<0)
#define SSP_SYSCLK_DIV_2 (1<<0)
#define SSP_SYSCLK_DIV_4 (2<<0)
#define SSP_SYSCLK_DIV_8 (3<<0)
#define SSP_SYSCLK_DIV_16 (4<<0)
#define SSP_SYSCLK_DIV_32 (5<<0)

#define SSP_SSACD_NOT_AVAILABLE 0xff
#define SSP_CLK_SSCR0_SCR_NOT_AVAILABLE 0


/*
 * Following enums are for frequency calculation in master mode...
 */

enum mrst_ssp_bit_per_sample {
	SSP_BIT_PER_SAMPLE_8 = 0,
	SSP_BIT_PER_SAMPLE_16,
	SSP_BIT_PER_SAMPLE_32,
	SSP_BIT_PER_SAMPLE_SIZE
};

enum mrst_ssp_timeslot {
	SSP_TIMESLOT_1 = 0,
	SSP_TIMESLOT_2,
	SSP_TIMESLOT_4,
	SSP_TIMESLOT_8,
	SSP_TIMESLOT_SIZE
};

static u8
ssp_ssacd[SSP_FRM_FREQ_SIZE][SSP_BIT_PER_SAMPLE_SIZE][SSP_TIMESLOT_SIZE] = {

	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_SSACD_NOT_AVAILABLE,

	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_SSACD_NOT_AVAILABLE,

	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_SSACD_NOT_AVAILABLE,


	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,

	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1 | SSP_SYSCLK_DIV4_BYPASS,


	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,

	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1 | SSP_SYSCLK_DIV4_BYPASS,


	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,

	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1 | SSP_SYSCLK_DIV4_BYPASS,


	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,

	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_4,

	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_2,


	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,

	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,


	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,

	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,

	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_4,
};

#endif /* MID_I2S_H_*/
