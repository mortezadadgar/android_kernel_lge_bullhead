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
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/pci_regs.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/scatterlist.h>
#include <linux/list.h>
#include <linux/async.h>

#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_i2s_if.h>
#include "intel_mid_i2s.h"

#include <linux/intel_mid_dma.h>
#include <sound/intel_sst_ioctl.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#ifdef __MRFL_SPECIFIC__
#include <linux/io.h>
#endif
#include <linux/delay.h>

MODULE_AUTHOR("Louis LE GALL <louis.le.gall intel.com>");
MODULE_DESCRIPTION("Intel MID I2S/PCM SSP Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.4");

#define STEREO 2
#define TDM8 8
#define SSCR0_SSE 		0X00000080
#define CLRBIT_SFRMWDTH 0xFFC0FFFF /*clear SSPSP bit: SFRMWDTH*/
#define SSSR_CSS 		0x00400000
#define SSP_DATA_TX			0
#define SSP_DATA_RX			1

#ifdef CONFIG_X86_BYT
#define CLOCK_SOURCE_KHZ			25000000
#else
#define CLOCK_SOURCE_KHZ			19200000
#endif

#ifdef __MRFL_SPECIFIC_TMP__
/* FIXME: use of lpeshim_base_address should be
 * avoided and replaced by a call to SST driver that will
 * take care to access LPE Shim registers */

/* LPE Shim registers base address (needed for HW IRQ acknowledge) */
//static void __iomem *lpeshim_base_address;
#endif /* __MRFL_SPECIFIC_TMP__ */
#if 0
/*
 * Currently this limit to ONE modem on platform
 */
static unsigned long modem_found_and_i2s_setup_ok;
static unsigned long clv_ssps_found;

static void(*intel_mid_i2s_modem_probe_cb)(void);
static void(*intel_mid_i2s_modem_remove_cb)(void);
#endif
/*
 * structures for pci probing
 */
#ifdef CONFIG_PM
static const struct dev_pm_ops intel_mid_i2s_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_mid_i2s_suspend,
				intel_mid_i2s_resume)
	.runtime_suspend = intel_mid_i2s_runtime_suspend,
	.runtime_resume = intel_mid_i2s_runtime_resume,
};
#endif

static struct lpe_device *pci;
struct intel_mid_i2s_hdl *i2s_drv_data[NUMBER_OF_SSP_PORT];

/*
 * Local functions declaration
 */
static inline void i2s_enable(struct intel_mid_i2s_hdl *drv_data, const struct intel_mid_i2s_settings *ps_settings, int streamdir);
static inline void i2s_disable(struct intel_mid_i2s_hdl *drv_data);

static void i2s_finalize_read(struct intel_mid_i2s_hdl *drv_data);
static void i2s_finalize_write(struct intel_mid_i2s_hdl *drv_data);

extern int get_number_of_ia_ssp(void);
extern int get_ia_port_number(void);

/*
 * POWER MANAGEMENT FUNCTIONS
 */

#ifdef CONFIG_PM
/**
 * intel_mid_i2s_driver_suspend - driver power management suspend activity
 * @dev : pointer of the device to suspend
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_suspend(struct device *dev)
{
	struct intel_mid_i2s_hdl *drv_data = dev_get_drvdata(dev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return 0;
	dev_dbg(&drv_data->pdev->dev, "SUSPEND SSP ID %d\n",
					drv_data->pdev->device);

	return 0;
}

/**
 * intel_mid_i2s_driver_resume - driver power management suspend activity
 * @device_ptr : pointer of the device to resume
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_resume(struct device *dev)
{
	struct intel_mid_i2s_hdl *drv_data = dev_get_drvdata(dev);

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	dev_dbg(&drv_data->pdev->dev, "RESUME SSP ID %d\n",
					drv_data->pdev->device);

	dev_dbg(&drv_data->pdev->dev, "resumed\n");
	return 0;
}

/**
 * intel_mid_i2s_runtime_suspend - runtime power management suspend activity
 * @device_ptr : pointer of the device to resume
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_runtime_suspend(struct device *device_ptr)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data;
	void __iomem *reg;

	pdev = to_pci_dev(device_ptr);
	WARN(!pdev, "Pci dev=NULL\n");
	if (!pdev)
		return -EFAULT;
	drv_data = (struct intel_mid_i2s_hdl *) pci_get_drvdata(pdev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	if (test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_err(device_ptr,
			"Trying to suspend a device that is opened\n");
		return -ENODEV;
	}
	reg = drv_data->ioaddr;
	dev_dbg(&drv_data->pdev->dev, "Suspend of SSP requested !!\n");
	return intel_mid_i2s_suspend(device_ptr);
}

/**
 * intel_mid_i2s_runtime_resume - runtime power management resume activity
 * @device_ptr : pointer of the device to resume
 *
 * Output parameters
 *      error : 0 means no error
 */
static int intel_mid_i2s_runtime_resume(struct device *device_ptr)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data;
	pdev = to_pci_dev(device_ptr);
	WARN(!pdev, "Pci dev=NULL\n");
	if (!pdev)
		return -EFAULT;
	drv_data = (struct intel_mid_i2s_hdl *) pci_get_drvdata(pdev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	dev_dbg(&drv_data->pdev->dev, "RT RESUME SSP ID\n");
	return intel_mid_i2s_resume(device_ptr);
}

#endif
/*
 * INTERFACE FUNCTIONS
 */

/**
 * intel_mid_i2s_flush - This is the I2S flush request
 * @drv_data : pointer on private i2s driver data (by i2s_open function)
 *
 * It will flush the TX FIFO
 * WARNING: this function is used in a Burst Mode context where it is called
 * between Bursts i.e. when there is no FMSYNC, no transfer ongoing at
 * that time
 * If you need a flush while SSP configured in I2S is BUSY and FMSYNC are
 * generated, you have to write another function
 * (loop on BUSY bit and do not limit the flush to at most 16 samples)
 *
 * Output parameters
 *      int : number of samples flushed
 */
int intel_mid_i2s_flush(struct intel_mid_i2s_hdl *drv_data)
{
	u32 sssr, data;
	u32 num = 0;
	u8  rsre;
	void __iomem *reg;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return 0;
	reg = drv_data->ioaddr;
	sssr = read_SSSR(reg);
	dev_warn(&drv_data->pdev->dev, "in flush sssr=0x%08X\n", sssr);

	rsre = read_SSCR1(reg) & (SSCR1_RSRE_MASK << SSCR1_RSRE_SHIFT);
	if (rsre) {
		/*
		 * Flush "by hand" was generating spurious DMA SERV REQUEST
		 * from SSP to DMA => then buggy retrieval
		 * of data for next dma_req
		 * Disable: RX Service Request from RX fifo to DMA
		 * as we will flush by hand
		 */
		clear_SSCR1_reg(reg, RSRE);
	}

	/* i2s_flush is called in between 2 bursts
	 * => no FMSYNC at that time (i.e. SSP not busy)
	 * => at most 16 samples in the FIFO */
	while ((read_SSSR(reg) & (SSSR_RNE_MASK<<SSSR_RNE_SHIFT))
			&& (num < FIFO_SIZE)) {
		data = read_SSDR(reg);
		num++;
	}

	if (rsre) {
		/* Enable: RX Service Request from RX fifo to DMA
		 * as flush by hand is done
		 */
		set_SSCR1_reg(reg, RSRE);
	}

	sssr = read_SSSR(reg);
	dev_dbg(&drv_data->pdev->dev, "out flush sssr=0x%08X\n", sssr);
	return num;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_flush);

/**
 * intel_mid_i2s_get_rx_fifo_level - returns I2S rx fifo level
 * @drv_data : pointer on private i2s driver data (by i2s_open function)
 *
 * Output parameters
 *      int : Number of samples currently in the RX FIFO (negative = error)
 */
int intel_mid_i2s_get_rx_fifo_level(struct intel_mid_i2s_hdl *drv_data)
{
	u32 sssr;
	u32 rne, rfl;
	void __iomem *reg;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	reg = drv_data->ioaddr;
	sssr = read_SSSR(reg);
	rfl = GET_SSSR_val(sssr, RFL);
	rne = GET_SSSR_val(sssr, RNE);
	if (!rne)
		return 0;
	else
		return rfl+1;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_get_rx_fifo_level);

/**
 * intel_mid_i2s_get_tx_fifo_level - returns I2S tx fifo level
 * @drv_data : pointer on private i2s driver data (by i2s_open function)
 *
 * Output parameters
 *      int : number of samples currently in the TX FIFO (negative = error)
 */
int intel_mid_i2s_get_tx_fifo_level(struct intel_mid_i2s_hdl *drv_data)
{
	u32 sssr;
	u32 tnf, tfl;
	void __iomem *reg;
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	reg = drv_data->ioaddr;
	sssr = read_SSSR(reg);
	tfl = GET_SSSR_val(sssr, TFL);
	tnf = GET_SSSR_val(sssr, TNF);
	if (!tnf)
		return 16;
	else
		return tfl;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_get_tx_fifo_level);

/**
 * intel_mid_i2s_set_rd_cb - set the callback function after read is done
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @read_callback : pointer of callback function
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_set_rd_cb(struct intel_mid_i2s_hdl *drv_data,
				int (*read_callback)(void *param))
{
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	mutex_lock(&drv_data->mutex);
	if (!test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_WARN(&drv_data->pdev->dev, "set WR CB I2S_PORT NOT_OPENED");
		mutex_unlock(&drv_data->mutex);

		return -EPERM;
	}
	/* Do not change read parameters in the middle of a READ request */
	if (test_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_WARN(&drv_data->pdev->dev, "CB reject I2S_PORT_READ_BUSY");
		mutex_unlock(&drv_data->mutex);

		return -EBUSY;
	}
	drv_data->read_callback = read_callback;
	drv_data->read_len = 0;
	mutex_unlock(&drv_data->mutex);

	return 0;

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_rd_cb);

/**
 * intel_mid_i2s_set_wr_cb - set the callback function after write is done
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open  function)
 * @write_callback : pointer of callback function
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_set_wr_cb(struct intel_mid_i2s_hdl *drv_data,
				int (*write_callback)(void *param))
{
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	mutex_lock(&drv_data->mutex);
	if (!test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_warn(&drv_data->pdev->dev, "set WR CB I2S_PORT NOT_OPENED");
		mutex_unlock(&drv_data->mutex);

		return -EPERM;
	}
	/* Do not change write parameters in the middle of a WRITE request */
	if (test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
		dev_warn(&drv_data->pdev->dev, "CB reject I2S_PORT_WRITE_BUSY");
		mutex_unlock(&drv_data->mutex);

		return -EBUSY;
	}
	drv_data->write_callback = write_callback;
	drv_data->write_len = 0;
	mutex_unlock(&drv_data->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_wr_cb);

/**
 * intel_mid_i2s_lli_rd_req - request a LLI read from i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @lli_array:
 * @lli_length:
 * @lli_mode:
 * @param:
 */
int intel_mid_i2s_lli_rd_req(struct intel_mid_i2s_hdl *drv_data,
			struct intel_mid_i2s_lli *lli_array, int lli_length,
			enum i2s_lli_mode lli_mode, void *param)
{
	struct dma_async_tx_descriptor *rxdesc = NULL;
	struct scatterlist *temp_sg = NULL;
	struct dma_chan *rxchan = NULL;
	enum dma_ctrl_flags flag;
	int i;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;

	rxchan = drv_data->rxchan;

	if (!rxchan) {
		dev_WARN(&(drv_data->pdev->dev), "rd_req FAILED no rxchan\n");
		return -EINVAL;
	}
	if (lli_length <= 0) {
		dev_WARN(&(drv_data->pdev->dev), "wr_req length less than 1\n");
		return -EINVAL;
	}
	dev_dbg(&(drv_data->pdev->dev), "FCT Enter = %s\n", __func__);

	/*
	 * Prepare the scatterlist struct array
	 */
	dev_dbg(&(drv_data->pdev->dev), "kzalloc of scatterlist rd\n");
	temp_sg = kzalloc(sizeof(struct scatterlist)*lli_length, GFP_KERNEL);
	if (!temp_sg) {
		dev_WARN(&(drv_data->pdev->dev),
				 "temp_sg alloc of size %d bytes failed",
				 sizeof(struct scatterlist)*lli_length);
		return -ENOMEM;
	}
	dev_dbg(&(drv_data->pdev->dev), "kzalloc rd done\n");

	/*
	 * Fill the Link list with Destination addresses
	 * mutex_lock(&drv_data->mutex); between sg_set_buf not required ?
	 */
	drv_data->rxsgl = temp_sg;
	for (i = 0; i <= lli_length; i++)
		sg_set_buf(temp_sg++, lli_array[i].addr, lli_array[i].leng);


	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	if (lli_mode == I2S_CIRCULAR_MODE)
		flag |= DMA_PREP_CIRCULAR_LIST;

	dev_dbg(&(drv_data->pdev->dev), "Calling prep_slave_sg\n");

	/* Enable the SSP Read Request */
	set_SSCR1_reg((drv_data->ioaddr), RSRE);
	change_SSCR0_reg((drv_data->ioaddr), RIM,
			 ((drv_data->current_settings).rx_fifo_interrupt));

	rxdesc = rxchan->device->device_prep_slave_sg(rxchan, drv_data->rxsgl,
			lli_length, DMA_DEV_TO_MEM, flag, NULL);

	if (!rxdesc) {
		dev_WARN(&(drv_data->pdev->dev),
			"rd_req device_prep_slave_sg FAILED\n");
		return -EFAULT;
	}

	/*
	 * Only 1 LLI READ is allowed
	 */
	if (test_and_set_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_WARN(&drv_data->pdev->dev, "RD reject I2S_PORT READ_BUSY");
		return -EBUSY;
	}
	dev_dbg(&(drv_data->pdev->dev), "RD dma tx submit\n");
	rxdesc->callback = i2s_lli_read_done;
	drv_data->read_param = param;
	rxdesc->callback_param = drv_data;
	rxdesc->tx_submit(rxdesc);
	return 0;

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_lli_rd_req);

/**
 * intel_mid_i2s_lli_wr_req - request a LLI wrie from i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @lli_array:
 * @lli_length:
 * @lli_mode:
 * @param:
 */
int intel_mid_i2s_lli_wr_req(struct intel_mid_i2s_hdl *drv_data,
		struct intel_mid_i2s_lli *lli_array, int lli_length,
		enum i2s_lli_mode lli_mode, void *param)
{
	struct dma_async_tx_descriptor *txdesc = NULL;
	struct scatterlist *temp_sg = NULL;
	struct dma_chan *txchan = NULL;
	enum dma_ctrl_flags flag;
	int i;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;

	txchan = drv_data->txchan;

	if (!txchan) {
		dev_WARN(&(drv_data->pdev->dev), "wr_req but no txchan\n");
		return -EINVAL;
	}
	if (lli_length <= 0) {
		dev_WARN(&(drv_data->pdev->dev), "wr_req length less than 1\n");
		return -EINVAL;
	}

	/*
	 * Determine the list length
	 */
	dev_dbg(&(drv_data->pdev->dev), "kzalloc of scatterlist wr\n");
	temp_sg = kzalloc(sizeof(struct scatterlist)*lli_length, GFP_KERNEL);
	if (!temp_sg) {
		dev_WARN(&(drv_data->pdev->dev),
				 "temp_sg alloc of size %d bytes failed",
				 sizeof(struct scatterlist)*lli_length);
		return -ENOMEM;
	}
	dev_dbg(&(drv_data->pdev->dev), "kzalloc wr done\n");

	/*
	 * Fill the Link list with Source addresses
	 * mutex_lock(&drv_data->mutex); not required for sg_set_buf?
	 */
	drv_data->txsgl = temp_sg;
	for (i = 0; i < lli_length; i++)
		sg_set_buf(temp_sg++, lli_array[i].addr, lli_array[i].leng);

	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	if (lli_mode == I2S_CIRCULAR_MODE)
		flag |= DMA_PREP_CIRCULAR_LIST;
	/* Enable the SSP Write Request */
	set_SSCR1_reg((drv_data->ioaddr), TSRE);
	change_SSCR0_reg((drv_data->ioaddr), TIM,
			 ((drv_data->current_settings).tx_fifo_interrupt));

	dev_dbg(&(drv_data->pdev->dev), "prep slave sg wr\n");
	txdesc =  txchan->device->device_prep_slave_sg(txchan, drv_data->txsgl,
				lli_length, DMA_MEM_TO_DEV, flag, NULL);
	dev_dbg(&(drv_data->pdev->dev), "prep slave sg wr done\n");
	if (!txdesc) {
		dev_WARN(&(drv_data->pdev->dev),
			"wr_req device_prep_slave_sg FAILED\n");
		return -1;
	}

	/*
	 * Only 1 LLI WRITE is allowed
	 */
	if (test_and_set_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
		dev_WARN(&drv_data->pdev->dev, "WR reject I2S_PORT WRITE_BUSY");
		return -EBUSY;
	}
	dev_dbg(&(drv_data->pdev->dev), "WR dma tx summit\n");
	txdesc->callback = i2s_lli_write_done;
	drv_data->write_param = param;
	txdesc->callback_param = drv_data;
	txdesc->tx_submit(txdesc);
	dev_dbg(&(drv_data->pdev->dev), "wr dma req programmed\n");
	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_lli_wr_req);

/**
 * intel_mid_i2s_set_busy - request to set i2s port busy
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @stream_direction : direction of the stream
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_set_busy(struct intel_mid_i2s_hdl	*drv_data, enum i2s_stream_direction stream_direction)
{
	if (I2S_STREAM_PLAYBACK == stream_direction)
	{
		if (test_and_set_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags))
		{
			dev_WARN(&drv_data->pdev->dev, "WR reject I2S_PORT WRITE_BUSY");
			return -EBUSY;
		}
	}
	else
	{
		if (test_and_set_bit(I2S_PORT_READ_BUSY, &drv_data->flags))
		{
			dev_WARN(&drv_data->pdev->dev, "RD reject I2S_PORT READ_BUSY");
			return -EBUSY;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_set_busy);

/**
 * intel_mid_i2s_rd_req - request a read from i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @dst : destination buffer where the read sample should be put
 * @len : number of sample to be read (160 samples only right now)
 * @param : private context parameter to give back to read callback
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_rd_req(struct intel_mid_i2s_hdl	*drv_data,
			 u32				*destination,
			 size_t				len,
			 void				*param)
{
	int result;

	/* Checks */
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	if (!len) {
		dev_WARN(&drv_data->pdev->dev, "rd req invalid len=0");
		return -EINVAL;
	}

	/* Allow only 1 READ at a time. */
	if (test_and_set_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_WARN(&drv_data->pdev->dev, "RD reject I2S_PORT READ_BUSY");
		return -EBUSY;
	}

	dev_dbg(&drv_data->pdev->dev, "%s() dst=%p, len=%d, drv_data=%p",
		__func__, destination, len, drv_data);

	if (drv_data->current_settings.ssp_rx_dma == SSP_RX_DMA_ENABLE)
		result = rd_req_dma(drv_data, destination, len, param);
	else
		result = rd_req_cpu(drv_data, destination, len, param);

	/* Error management */
	if (result)
		goto return_clr_busy;

	return result;

	/* Error management: free resources */
return_clr_busy:
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);

	return result;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_rd_req);

static int rd_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param)
{
	dev_dbg(&drv_data->pdev->dev, "%s() - ENTER\n", __func__);

	/* Initiate read */
	drv_data->mask_sr |= ((SSSR_RFS_MASK << SSSR_RFS_SHIFT) |
			      (SSSR_ROR_MASK << SSSR_ROR_SHIFT));

	clear_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags);
	set_SSCR1_reg((drv_data->ioaddr), RIE);
	change_SSCR0_reg((drv_data->ioaddr), RIM,
			 ((drv_data->current_settings).rx_fifo_interrupt));

	/* Save param */
	drv_data->read_ptr.cpu	= destination;
	drv_data->read_len	= len;
	drv_data->read_param	= param;

	return 0;
}

static int rd_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param)
{
	/* Locals declaration */
	int				result = 0;
	struct dma_async_tx_descriptor	*rxdesc = NULL;
	struct dma_chan			*rxchan = drv_data->rxchan;
	enum dma_ctrl_flags		flag;
	dma_addr_t			ssdr_addr;
	dma_addr_t			dst;

	dev_dbg(&drv_data->pdev->dev, "%s() - ENTER", __func__);
	/* Checks */
	WARN(!drv_data->dmac1, "DMA device=NULL\n");
	if (!drv_data->dmac1)
		return -EFAULT;

	if (!rxchan) {
		dev_WARN(&(drv_data->pdev->dev), "rd_req FAILED no rxchan\n");
		return -EINVAL;
	}

	/* Map dma address */
	dst = dma_map_single(NULL, destination, len, DMA_DEV_TO_MEM);
	if (!dst) {
		dev_WARN(&drv_data->pdev->dev, "can't map DMA address %p",
			 destination);
		return -ENOMEM;
	}

	/* Prepare RX dma transfer */
	ssdr_addr = (dma_addr_t)(u32)(drv_data->pinternal_addr + OFFSET_SSDR);
	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

	rxdesc = rxchan->device->device_prep_dma_memcpy(
					rxchan,		/* DMA Channel */
					dst,		/* DAR */
					ssdr_addr,	/* SAR */
					len,		/* Data Length */
					flag);		/* Flag */
	if (!rxdesc) {
		dev_WARN(&drv_data->pdev->dev, "can not prep dma memcpy");
		result = -EFAULT;
		goto return_unmap;
	}

	set_SSCR1_reg((drv_data->ioaddr), RSRE);
	change_SSCR0_reg((drv_data->ioaddr), RIM,
			 ((drv_data->current_settings).rx_fifo_interrupt));

	/* Save param */
	drv_data->read_ptr.dma	= dst;
	drv_data->read_len	= len;
	drv_data->read_param	= param;

	rxdesc->callback	= i2s_read_done;
	rxdesc->callback_param	= drv_data;

	/* Submit DMA transfer */
	rxdesc->tx_submit(rxdesc);
	return result;

	/* Error management: free resources */
return_unmap:
	dma_unmap_single(NULL, dst, len, DMA_DEV_TO_MEM);
	return result;
}

/**
 * intel_mid_i2s_wr_req - request a write to i2s peripheral
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @src : source buffer where the samples to wrote should be get
 * @len : number of sample to be read (160 samples only right now)
 * @param : private context parameter to give back to write callback
 *
 * Output parameters
 *      error : 0 means no error
 */
int intel_mid_i2s_wr_req(struct intel_mid_i2s_hdl	*drv_data,
			 u32				*source,
			 size_t				len,
			 void				*param)
{
	int result;

	/* Checks */
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	if (!len) {
		dev_WARN(&drv_data->pdev->dev, "invalid wr len 0");
		return -EINVAL;
	}

	/* Allow only 1 WRITE at a time */
	if (test_and_set_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
		dev_WARN(&drv_data->pdev->dev, "WR reject I2S_PORT WRITE_BUSY");
		return -EBUSY;
	}

	dev_dbg(&drv_data->pdev->dev, "%s() src=%p, len=%d, drv_data=%p",
		__func__, source, len, drv_data);

	if (drv_data->current_settings.ssp_tx_dma == SSP_TX_DMA_ENABLE)
		result = wr_req_dma(drv_data, source, len, param);
	else
		result = wr_req_cpu(drv_data, source, len, param);

	/* Error management */
	if (result)
		goto return_clr_busy;

	return result;

	/* Error management: free resources */
return_clr_busy:
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);

	return result;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_wr_req);

static int wr_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param)
{
	dev_dbg(&drv_data->pdev->dev, "%s() - ENTER (src=0x%08x, len=%d"
		, __func__, (u32)source, len);
	/* Initiate write */
	drv_data->mask_sr |= ((SSSR_TFS_MASK << SSSR_TFS_SHIFT) |
			      (SSSR_TUR_MASK << SSSR_TUR_SHIFT));

	clear_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags);

	set_SSCR1_reg((drv_data->ioaddr), TIE);
	change_SSCR0_reg((drv_data->ioaddr), TIM,
			 ((drv_data->current_settings).tx_fifo_interrupt));

	/* Save param */
	drv_data->write_ptr.cpu	= source;
	drv_data->write_len	= len;
	drv_data->write_param	= param;

	return 0;
}

static int wr_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param)
{
	int				result = 0;
	struct dma_async_tx_descriptor	*txdesc = NULL;
	struct dma_chan			*txchan = drv_data->txchan;
	enum dma_ctrl_flags		flag;
	dma_addr_t			ssdr_addr;
	dma_addr_t			src;

	dev_dbg(&drv_data->pdev->dev, "%s() - ENTER", __func__);

	WARN(!drv_data->dmac1, "DMA device=NULL\n");
	if (!drv_data->dmac1)
		return -EFAULT;

	if (!txchan) {
		dev_WARN(&(drv_data->pdev->dev), "wr_req but no txchan\n");
		return -EINVAL;
	}

	/* Map DMA address */
	src = dma_map_single(NULL, source, len, DMA_MEM_TO_DEV);
	if (!src) {
		dev_WARN(&drv_data->pdev->dev, "can't map DMA address %p",
						source);
		return -ENOMEM;
	}

	/* Prepare TX dma transfer */
	ssdr_addr = (dma_addr_t)(u32)(drv_data->pinternal_addr + OFFSET_SSDR);
	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

	pr_debug("%s : txchan chan_id= %d \n"\
			"dest addr = 0x%016llx\n" \
			"src addr = 0x%016llx\n" \
			"len = %d\n" \
			"flag = %d\n" \
			, __func__, txchan->chan_id, ssdr_addr, src, len, flag);

	txdesc = txchan->device->device_prep_dma_memcpy(
					txchan,		/* DMA Channel */
					ssdr_addr,	/* DAR */
					src,		/* SAR */
					len,		/* Data Length */
					flag);		/* Flag */
	if (!txdesc) {
		dev_WARN(&(drv_data->pdev->dev),
			"wr_req dma memcpy FAILED(src=%08x,len=%d,txchan=%p)\n",
			(unsigned int)src, len, txchan);
		result = -EFAULT;
		goto return_unmap;
	}

	set_SSCR1_reg((drv_data->ioaddr), TSRE);
	change_SSCR0_reg((drv_data->ioaddr), TIM,
			 ((drv_data->current_settings).tx_fifo_interrupt));

	/* Save params */
	drv_data->write_ptr.dma	= src;
	drv_data->write_len	= len;
	drv_data->write_param	= param;

	txdesc->callback	= i2s_write_done;
	txdesc->callback_param	= drv_data;

	/* Submit DMA transfer */
	txdesc->tx_submit(txdesc);

	return result;

	/* Error management: free resources */
return_unmap:
	dma_unmap_single(NULL, src, len, DMA_MEM_TO_DEV);
	return result;
}

/**
 * intel_mid_i2s_open - reserve and start a SSP depending of it's usage
 * @usage : select which ssp i2s you need by giving usage (BT,MODEM...)
 * @ps_settings : hardware settings to configure the SSP module
 *
 * May sleep (driver_find_device) : no lock permitted when called.
 *
 * Output parameters
 *      handle : handle of the selected SSP, or NULL if not found
 */
struct intel_mid_i2s_hdl *intel_mid_i2s_open(enum intel_mid_i2s_ssp_usage usage)
{
	struct intel_mid_i2s_hdl *drv_data = NULL;
	struct device *found_device = NULL;

	pr_debug("%s : open called,searching for device with usage=%x !\n",
			DRIVER_NAME, usage);

	/* pci_get_device() will not work as PCI drv_data are in individual containers.
	 * Directly get drv data based on the usage.
	 */
	if(usage == SSP_USAGE_CARD_AK4614_0)
	{
	  pr_debug("usage == SSP0_USAGE\n");
	  drv_data = i2s_drv_data[SSP0_INSTANCE];
	}
	else if(usage == SSP_USAGE_CARD_AK4614_1)
	{
	  pr_debug("usage == SSP1_USAGE\n");
	  drv_data = i2s_drv_data[SSP1_INSTANCE];
	}
	else if(usage == SSP_USAGE_CARD_AK4614_2)
	{
	  pr_debug("usage == SSP2_USAGE\n");
	  drv_data = i2s_drv_data[SSP2_INSTANCE];
	}

	if (!drv_data) {
		dev_err(found_device, "no drv_data in open pdev=%p\n", drv_data);
		return NULL;
	}

	if (!test_and_set_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		/*If at first fail, try again*/
		if (!test_and_set_bit(I2S_PORT_OPENED, &drv_data->flags)) {
			pr_err(" %s : SSP port open fail (2nd attempt)\n", __func__);
			goto open_error;
		}

	}
	mutex_lock(&drv_data->mutex);
#ifdef CONFIG_PM
	/* pm_runtime */
	if (drv_data->pdev->device != SST_BYT_PCI_ID) /*Not supported in BYT*/
		pm_runtime_get_sync(&drv_data->pdev->dev);
#endif
	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags)) {
		dev_err(&drv_data->pdev->dev, "Opening a closing I2S!");
		goto open_error;
	}
	/* Indicate that the HW param config is not set yet */
	drv_data->current_settings.mode = SSP_INVALID_MODE;

	/* there is no need to "wake up" as we can not close an opening i2s */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	mutex_unlock(&drv_data->mutex);

	return drv_data;

open_error:
	if (drv_data->pdev->device != SST_BYT_PCI_ID) /*Not supported in BYT*/
		pm_runtime_put(&drv_data->pdev->dev);
	mutex_unlock(&drv_data->mutex);
	return NULL;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_open);

/**
 * intel_mid_i2s_close - release and stop the SSP
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 *
 * WARNING: This is not -yet- allowed to call close from a read/write callback !
 *
 * Output parameters
 *      none
 */
void intel_mid_i2s_close(struct intel_mid_i2s_hdl *drv_data)
{
	void __iomem *reg;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	mutex_lock(&drv_data->mutex);
	if (!test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_err(&drv_data->pdev->dev, "not opened but closing?");
		mutex_unlock(&drv_data->mutex);
		return;
	}

	set_bit(I2S_PORT_CLOSING, &drv_data->flags);
	dev_dbg(&drv_data->pdev->dev, "Status bit pending write=%d read=%d\n",
			test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags),
			test_bit(I2S_PORT_READ_BUSY, &drv_data->flags));
	if (test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags) ||
	     test_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
		dev_dbg(&drv_data->pdev->dev,
				"Pending callback in close...\n");
	}

	reg = drv_data->ioaddr;
	dev_dbg(&drv_data->pdev->dev, "Stopping the SSP\n");
	i2s_disable(drv_data);
	write_SSCR0(0, reg);
	/*
	 * Set the SSP in SLAVE Mode and Enable TX tristate
	 * to ensure that when leaving D0i3 the SSP does
	 * not drive the FS and TX pins.
	 *
	 */
	write_SSCR1((SSCR1_SFRMDIR_MASK << SSCR1_SFRMDIR_SHIFT)
			| (SSCR1_SCLKDIR_MASK << SSCR1_SCLKDIR_SHIFT)
			| (SSCR1_TTE_MASK << SSCR1_TTE_SHIFT),
			reg);

	dev_dbg(&(drv_data->pdev->dev), "SSP Stopped.\n");
	clear_bit(I2S_PORT_CLOSING, &drv_data->flags);
	clear_bit(I2S_PORT_OPENED, &drv_data->flags);

	/* pm runtime */
	if (drv_data->pdev->device != SST_BYT_PCI_ID) /*Not supported in BYT*/
		pm_runtime_put(&drv_data->pdev->dev);

	mutex_unlock(&drv_data->mutex);
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_close);

/*
 * INTERNAL FUNCTIONS
 */

/**
 * i2s_enable -  enable SSP device
 * @drv_data : pointer to driver data
 *
 * Writes SSP register to enable the device
 */
static inline void i2s_enable(struct intel_mid_i2s_hdl *drv_data, const struct intel_mid_i2s_settings *ps_settings, int streamdir)
{
	u32 status;
	u32 sssr;
	u32 saved_SSCR3 = 0;

	status = read_SSCR0(drv_data->ioaddr);
	if (status & SSCR0_SSE) { /*Refactor if-else logic*/
		sssr = read_SSSR(drv_data->ioaddr);

		if(GET_SSSR_val(sssr, ROR)) {
			if(STEREO == ps_settings->frame_rate_divider_control) { /*I2S fix*/
				/*Store current active TX/RX operation state*/
				saved_SSCR3 = read_SSCR3(drv_data->ioaddr);
				clear_SSCR3_reg(drv_data->ioaddr, I2S_TX_EN);
				clear_SSCR3_reg(drv_data->ioaddr, I2S_RX_EN);
			}/*I2S fix*/
			clear_SSCR0_reg(drv_data->ioaddr, SSE);
			while(read_SSCR0(drv_data->ioaddr) & SSCR0_SSE);
			set_SSCR0_reg(drv_data->ioaddr, SSE);

			if(STEREO == ps_settings->frame_rate_divider_control) { /*I2S fix*/
				if (SSPSCLK_SLAVE_MODE == ps_settings->sspslclk_direction)
					while((read_SSSR(drv_data->ioaddr) & SSSR_CSS));	/*sssr: CSS*/
				/*Restore previous active TX/RX operation state*/
				write_SSCR3(saved_SSCR3, drv_data->ioaddr);
				/*Enable TX/RX for new stream operation*/
				if(SSP_DATA_TX == streamdir) {
					set_SSCR3_reg(drv_data->ioaddr, I2S_TX_EN);

				} else {
					set_SSCR3_reg(drv_data->ioaddr, I2S_RX_EN);
				}
			}/*I2S fix*/
		}
	}
	else {
		set_SSCR0_reg(drv_data->ioaddr, SSE);

/*For CAPTURE usage in SSP MASTER mode
 * 	Dummy write to SSDR needed to initiate FS.
 *	N/A to Stereo mode
 */
		if((SSPSCLK_MASTER_MODE == ps_settings->sspslclk_direction)
				&& (SSP_DATA_RX == streamdir)
				&& (STEREO != ps_settings->frame_rate_divider_control )) {
		/*FORCE WRITE TO SSDR for clock for capture mode */
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
			write_SSDR(0x1fbeeff8, drv_data->ioaddr);
		} else if(SSPSCLK_SLAVE_MODE == ps_settings->sspslclk_direction
					&& (STEREO == ps_settings->frame_rate_divider_control )) { /*I2S fix*/
			while((read_SSSR(drv_data->ioaddr) & SSSR_CSS));
		}/*I2S fix*/

		if (STEREO == ps_settings->frame_rate_divider_control ) { /*I2S fix*/
			if(SSP_DATA_TX == streamdir) {
				set_SSCR3_reg(drv_data->ioaddr, I2S_TX_EN);

			} else {
				set_SSCR3_reg(drv_data->ioaddr, I2S_RX_EN);
			}
		}
	}

}/*i2s_enable*/

/**
 * i2s_disable -  disable SSP device
 * @drv_data : pointer to driver data
 *
 * Writes SSP register to disable the device
 */
static inline void i2s_disable(struct intel_mid_i2s_hdl *drv_data)
{
	clear_SSCR3_reg(drv_data->ioaddr, I2S_TX_EN);
	clear_SSCR3_reg(drv_data->ioaddr, I2S_RX_EN);
	clear_SSCR0_reg(drv_data->ioaddr, SSE);
}

#if 0
/**
 * check_device -  return if the device is the usage we want (usage =*data)
 * @device_ptr : pointer on device struct
 * @data : pointer pointer on usage we are looking for
 *
 * this is called for each device by find_device() from intel_mid_i2s_open()
 * Info : when found, the flag of driver is set to I2S_PORT_OPENED
 *
 * Output parameters
 *      integer : return 0 means not the device or already started. go next
 *		  return != 0 means stop the search and return this device
 */
static int
check_device(struct device *device_ptr, void *data)
{
	struct pci_dev *pdev;
	struct intel_mid_i2s_hdl *drv_data;
	enum intel_mid_i2s_ssp_usage usage;
	enum intel_mid_i2s_ssp_usage usage_to_find;

	pdev = to_pci_dev(device_ptr);
	WARN(!pdev, "Pci device=NULL\n");
	if (!pdev)
		return 0;
	drv_data = (struct intel_mid_i2s_hdl *) pci_get_drvdata(pdev);
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return 0;
	dev_dbg(&(pdev->dev), "Check device pci_dev ptr = 0X%p\n", pdev);
	usage_to_find = *((enum intel_mid_i2s_ssp_usage *) data);
	usage = drv_data->usage;

	/* Can be done in one "if" but separated in purpose : take care of
	 * test_and_set_bit that need to be done AFTER the check on usage.
	 */
	if (usage == usage_to_find) {
		if (!test_and_set_bit(I2S_PORT_OPENED, &drv_data->flags))
			return 1;  /* Already opened, do not use this result */
	};
	return 0; /* not usage we look for, or already opened */
}
#endif
/**
 * i2s_reset_command_done - reset driver state if dma callback is not excuted before stream stop
 * @cmd : command to be executed
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_reset_command_done(struct intel_mid_i2s_hdl *drv_data,
				enum intel_mid_i2s_ssp_cmd cmd)
{
	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;

	switch (cmd) {
	case SSP_CMD_FREE_TX:
		drv_data->write_len = 0;
		change_SSCR0_reg(drv_data->ioaddr, TIM,
				SSP_TX_FIFO_UNDER_INT_DISABLE);
		clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
		break;

	case SSP_CMD_FREE_RX:
		drv_data->read_len = 0;
		change_SSCR0_reg(drv_data->ioaddr, RIM,
				SSP_RX_FIFO_OVER_INT_DISABLE);
		clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
		break;

	default:
		dev_warn(&drv_data->pdev->dev, "Unknown reset cmd received!");
		break;
	}
	return;
}

/**
 * i2s_read_done - callback from the _dma tasklet_ after read
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_read_done(void *arg)
{
	int status = 0;

	struct intel_mid_i2s_hdl *drv_data = arg;
	void *param_complete;
	void __iomem *reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_READ_BUSY, &drv_data->flags))
		dev_WARN(&drv_data->pdev->dev, "spurious read dma complete");

	dma_unmap_single(NULL, drv_data->read_ptr.dma,
			 drv_data->read_len, DMA_DEV_TO_MEM);
	drv_data->read_len = 0;
	reg = drv_data->ioaddr;
	/* Rx fifo overrun Interrupt */
	change_SSCR0_reg(reg, RIM, SSP_RX_FIFO_OVER_INT_DISABLE);
	param_complete = drv_data->read_param;
	/* Do not change order sequence:
	 * READ_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->read_callback != NULL)
		status = drv_data->read_callback(param_complete);
	else
		dev_warn(&drv_data->pdev->dev, "RD done but not callback set");

}

/**
 * i2s_lli_read_done - callback from the _dma tasklet_ after read
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_lli_read_done(void *arg)
{
	int status = 0;

	struct intel_mid_i2s_hdl *drv_data = arg;
	void *param_complete;
	void __iomem *reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_READ_BUSY, &drv_data->flags))
		dev_WARN(&drv_data->pdev->dev, "spurious read dma complete");

	reg = drv_data->ioaddr;
	/* Rx fifo overrun Interrupt */
	change_SSCR0_reg(reg, RIM, SSP_RX_FIFO_OVER_INT_DISABLE);
	param_complete = drv_data->read_param;
	/* Do not change order sequence:
	 * READ_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);

	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->read_callback != NULL)
		status = drv_data->read_callback(param_complete);
	else
		dev_warn(&drv_data->pdev->dev, "RD done but not callback set");
}


/**
 * i2s_write_done() : callback from the _dma tasklet_ after write
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_write_done(void *arg)
{
	struct intel_mid_i2s_hdl	*drv_data	= arg;
	int				status		= 0;
	void				*param_complete;
	void __iomem			*reg ;

	WARN(!drv_data, "Driver data=NULL\n");

	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags))
		dev_warn(&drv_data->pdev->dev, "spurious write dma complete");

	dma_unmap_single(NULL, drv_data->write_ptr.dma,
			 drv_data->write_len, DMA_MEM_TO_DEV);
	drv_data->write_len	= 0;
	drv_data->write_ptr.cpu	= NULL;

	reg = drv_data->ioaddr;
	change_SSCR0_reg(reg, TIM, SSP_TX_FIFO_UNDER_INT_DISABLE);
	dev_dbg(&(drv_data->pdev->dev), "DMA channel disable..\n");
	param_complete = drv_data->write_param;

	/* Do not change order sequence:
	 * WRITE_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->write_callback != NULL)
		status = drv_data->write_callback(param_complete);
	else
		dev_warn(&drv_data->pdev->dev, "WR done but no callback set");
}

/**
 * i2s_lli_write_done - callback from the _dma tasklet_ after write
 * @arg : void pointer to that should be driver data (context)
 *
 * Output parameters
 *      none
 */
static void i2s_lli_write_done(void *arg)
{
	int status = 0;
	void *param_complete;
	struct intel_mid_i2s_hdl *drv_data = arg;
	void __iomem *reg ;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return;
	if (!test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags))
		dev_warn(&drv_data->pdev->dev, "spurious write dma complete");


	dev_dbg(&drv_data->pdev->dev, "lli wr Done!\n");


	reg = drv_data->ioaddr;
	change_SSCR0_reg(reg, TIM, SSP_TX_FIFO_UNDER_INT_DISABLE);
	dev_dbg(&(drv_data->pdev->dev), "DMA channel disable..\n");
	param_complete = drv_data->write_param;

	/* Do not change order sequence:
	 * WRITE_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);

	if (test_bit(I2S_PORT_CLOSING, &drv_data->flags))
		return;
	if (drv_data->write_callback != NULL)
		status = drv_data->write_callback(param_complete);
	else
		dev_warn(&drv_data->pdev->dev, "WR done but no callback set");
}

static bool chan_filter(struct dma_chan *chan, void *param)
{
	struct intel_mid_i2s_hdl *drv_data = (struct intel_mid_i2s_hdl *)param;
	bool ret = false;

	if (!drv_data->dmac1)
		goto out;
	if (chan->device->dev == &drv_data->dmac1->dev)
		ret = true;
out:
	return ret;
}

static int i2s_compute_dma_width(u16 ssp_data_size,
				 enum dma_slave_buswidth *dma_width)
{
	if (ssp_data_size <= 8)
		*dma_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (ssp_data_size <= 16)
		*dma_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else if (ssp_data_size <= 32)
		*dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	else
		return -EINVAL;


	return 0;
}
static int i2s_compute_dma_msize(u8 ssp_threshold,
				 enum intel_mid_dma_msize *dma_msize)
{

	switch (ssp_threshold) {
	case 1:
		*dma_msize = LNW_DMA_MSIZE_1;
		break;
	case 4:
		*dma_msize = LNW_DMA_MSIZE_4;
		break;
	case 8:
		*dma_msize = LNW_DMA_MSIZE_8;
		break;
	case 16:
		*dma_msize = LNW_DMA_MSIZE_16;
		break;
	case 32:
		*dma_msize = LNW_DMA_MSIZE_32;
		break;
	case 64:
		*dma_msize = LNW_DMA_MSIZE_64;
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

/**
 * intel_mid_i2s_command() : execute simple commands on ssp driver
 * @drv_data : handle of corresponding ssp i2s (given by i2s_open function)
 * @cmd : command to be executed
 *
 * Execute simple commands on ssp driver
 *
 * Output parameters
 *      error : 0 means no error
 */

int intel_mid_i2s_command(struct intel_mid_i2s_hdl *drv_data,
			enum intel_mid_i2s_ssp_cmd cmd,
			const struct intel_mid_i2s_settings *hw_ssp_settings,
			int streamdir)
{
	void __iomem *reg;

	struct intel_mid_dma_slave *rxs, *txs;
	struct intel_mid_i2s_settings *ssp_settings = NULL;
	dma_cap_mask_t mask;
	int retval = 0;
	int temp = 0;

	int s;
	struct dma_chan *channel;

	WARN(!drv_data, "Driver data=NULL\n");
	if (!drv_data)
		return -EFAULT;
	reg = drv_data->ioaddr;

	pr_debug("FCT %s CMD = %d\n", __func__, cmd);
	/* actions */
	switch (cmd) {

	case SSP_CMD_ABORT:
		/* Abort write */
		if (test_and_clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
			dev_dbg(&(drv_data->pdev->dev),
					"%s : abort write", __func__);

			clear_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags);
			i2s_finalize_write(drv_data);
		}

		/* Abort read */
		if (test_and_clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
			dev_dbg(&(drv_data->pdev->dev),
					"%s: abort read", __func__);

			clear_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags);
			i2s_finalize_read(drv_data);
		}

		i2s_disable(drv_data);
		break;

	case SSP_CMD_SET_HW_CONFIG:
		set_ssp_i2s_hw(drv_data, hw_ssp_settings);
		break;

	case SSP_CMD_ENABLE_SSP:
		i2s_enable(drv_data, hw_ssp_settings, streamdir);
		break;

	case SSP_CMD_DISABLE_SSP:
		i2s_disable(drv_data);
		break;

	case SSP_CMD_ALLOC_TX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_tx_dma != SSP_TX_DMA_ENABLE)
			break;

		WARN(!drv_data->dmac1, "DMA device=NULL\n");
		if (!drv_data->dmac1)
			return -EFAULT;

		if (ssp_settings->mode == SSP_INVALID_MODE) {
			WARN(1, "Trying to alloc TX channel however"
				"the HW Config is NOT set\n");
			retval = -1;
			goto err_exit;
		}

		WARN(drv_data->txchan, "Trying to realloc TX channel\n");
		if (drv_data->txchan != NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_tx_slots_map)),
				"Trying to alloc non active tx channel\n");


		/* 2. init tx channel */
		txs = &drv_data->dmas_tx;
		txs->dma_slave.direction = DMA_MEM_TO_DEV;
		txs->dma_slave.dst_addr = (dma_addr_t)(u32)(drv_data->pinternal_addr + OFFSET_SSDR);
		txs->hs_mode = LNW_DMA_HW_HS;
		txs->cfg_mode = LNW_DMA_MEM_TO_PER;
		txs->dma_slave.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; //bit depth : 8,16,32 only

		temp = i2s_compute_dma_width(ssp_settings->data_size,
						&txs->dma_slave.dst_addr_width);
		if (temp != 0) {
			dev_err(&(drv_data->pdev->dev),
				"TX DMA Channel Bad data_size = %d\n",
				ssp_settings->data_size);
			retval = -2;
			goto err_exit;

		}

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_tx_fifo_threshold+1,
			&txs->dma_slave.src_maxburst);

		if (temp != 0) {
			dev_err(&(drv_data->pdev->dev),
				"TX DMA Channel Bad TX FIFO Threshold = %d\n",
				ssp_settings->ssp_tx_fifo_threshold);
			retval = -3;
			goto err_exit;

		}

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_tx_fifo_threshold+1,
			&txs->dma_slave.dst_maxburst);

		if (temp != 0) {
			dev_err(&(drv_data->pdev->dev),
				"TX DMA Channel Bad TX FIFO Threshold = %d\n",
				ssp_settings->ssp_tx_fifo_threshold);
			retval = -4;
			goto err_exit;
		}

		txs->device_instance = drv_data->device_instance;
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		dma_cap_set(DMA_MEMCPY, mask);

		drv_data->txchan = dma_request_channel(mask, chan_filter,
							drv_data);
		if (!drv_data->txchan) {
			dev_err(&(drv_data->pdev->dev),
				"Could not get Tx channel\n");
			retval = -5;
			goto err_exit;
		}

		temp = drv_data->txchan->device->device_control(
				drv_data->txchan, DMA_SLAVE_CONFIG,
				(unsigned long) &txs->dma_slave);
		if (temp) {
			dev_err(&(drv_data->pdev->dev),
					"Tx slave control failed\n");
			retval = -6;
			goto err_exit;
		}
		break;

	case SSP_CMD_FREE_TX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_tx_dma != SSP_TX_DMA_ENABLE) {
			i2s_reset_command_done(drv_data, SSP_CMD_FREE_TX);
			break;
		}

		WARN(!drv_data->dmac1, "DMA device=NULL\n");
		if (!drv_data->dmac1)
			return -EFAULT;

		channel = drv_data->txchan;

		WARN(!channel, "Trying to free non existant TX channel\n");
		if (channel == NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_tx_slots_map)),
				"Trying to free non active tx channel\n");

		s = channel->device->device_control(channel,
							DMA_TERMINATE_ALL, 0);
		WARN(s, "DMA TERMINATE of TX returns error\n");

		dma_release_channel(channel);
		dma_unmap_single(NULL, drv_data->write_ptr.dma,
				 drv_data->write_len, DMA_MEM_TO_DEV);
		i2s_reset_command_done(drv_data, SSP_CMD_FREE_TX);
		drv_data->txchan = NULL;
		break;

	case SSP_CMD_ALLOC_RX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_rx_dma != SSP_RX_DMA_ENABLE)
			break;

		WARN(!drv_data->dmac1, "DMA device=NULL\n");
		if (!drv_data->dmac1)
			return -EFAULT;

		if (ssp_settings->mode == SSP_INVALID_MODE) {
			WARN(1, "Trying to alloc RX channel however"
				"the HW Config is NOT set\n");
			retval = -7;
			goto err_exit;
		}


		WARN(drv_data->rxchan, "Trying to realloc RX channel\n");
		if (drv_data->rxchan != NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_rx_slots_map)),
				"Trying to alloc non active rx channel\n");

		/* 1. init rx channel */
		rxs = &drv_data->dmas_rx;
		rxs->dma_slave.direction = DMA_DEV_TO_MEM;
		rxs->dma_slave.src_addr = (dma_addr_t)(u32)(drv_data->pinternal_addr + OFFSET_SSDR);
		rxs->hs_mode = LNW_DMA_HW_HS;
		rxs->cfg_mode = LNW_DMA_PER_TO_MEM;
		temp = i2s_compute_dma_width(ssp_settings->data_size,
						&rxs->dma_slave.src_addr_width);

		if (temp != 0) {
			dev_err(&(drv_data->pdev->dev),
				"RX DMA Channel Bad data_size = %d\n",
				ssp_settings->data_size);
			retval = -8;
			goto err_exit;

		}
		rxs->dma_slave.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; ////bit depth : 8,16,32 only

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_rx_fifo_threshold
			, &rxs->dma_slave.src_maxburst);

		if (temp != 0) {
			dev_err(&(drv_data->pdev->dev),
				"RX DMA Channel Bad RX FIFO Threshold\n");
			retval = -9;
			goto err_exit;

		}

		temp = i2s_compute_dma_msize(
			ssp_settings->ssp_rx_fifo_threshold
			, &rxs->dma_slave.dst_maxburst);

		if (temp != 0) {
			dev_err(&(drv_data->pdev->dev),
				"RX DMA Channel Bad RX FIFO Threshold\n");
			retval = -10;
			goto err_exit;

		}

		rxs->device_instance = drv_data->device_instance;
		dma_cap_zero(mask);
		dma_cap_set(DMA_MEMCPY, mask);
		dma_cap_set(DMA_SLAVE, mask);
		drv_data->rxchan = dma_request_channel(mask, chan_filter,
							drv_data);
		if (!drv_data->rxchan) {
			dev_err(&(drv_data->pdev->dev),
				"Could not get Rx channel\n");
			retval = -11;
			goto err_exit;
		}

		temp = drv_data->rxchan->device->device_control(
				drv_data->rxchan, DMA_SLAVE_CONFIG,
				(unsigned long) &rxs->dma_slave);

		if (temp) {
			dev_err(&(drv_data->pdev->dev),
				"Rx slave control failed\n");
			retval = -12;
			goto err_exit;
		}
		break;

	case SSP_CMD_FREE_RX:
		ssp_settings = &(drv_data->current_settings);

		/* Check if DMA channel allocation needed */
		if (ssp_settings->ssp_rx_dma != SSP_RX_DMA_ENABLE) {
			i2s_reset_command_done(drv_data, SSP_CMD_FREE_RX);
			break;
		}

		WARN(!drv_data->dmac1, "DMA device=NULL\n");
		if (!drv_data->dmac1)
			return -EFAULT;

		channel = drv_data->rxchan;

		WARN(!channel, "Trying to free non existant RX channel\n");
		if (channel == NULL)
			return -EFAULT;

		WARN((!(ssp_settings->ssp_active_rx_slots_map)),
				"Trying to free non active rx channel\n");

		s = channel->device->device_control(channel,
							DMA_TERMINATE_ALL, 0);

		WARN(s, "DMA TERMINATE of RX returns error\n");

		dma_release_channel(channel);
		dma_unmap_single(NULL, drv_data->read_ptr.dma,
				 drv_data->read_len, DMA_DEV_TO_MEM);
		i2s_reset_command_done(drv_data, SSP_CMD_FREE_RX);
		drv_data->rxchan = NULL;
		break;

	case SSP_CMD_PAUSE_TX:
		drv_data->txchan->device->device_control(drv_data->txchan,DMA_PAUSE,0);
		break;

	case SSP_CMD_PAUSE_RX:
		drv_data->rxchan->device->device_control(drv_data->rxchan,DMA_PAUSE,0);
		break;

	case SSP_CMD_RESUME_TX:
		drv_data->txchan->device->device_control(drv_data->txchan,DMA_RESUME,0);
		break;

	case SSP_CMD_RESUME_RX:
		drv_data->rxchan->device->device_control(drv_data->rxchan,DMA_RESUME,0);
		break;

	default:
		dev_warn(&drv_data->pdev->dev, "Incorrect command received !");
		return -EFAULT;
		break;
	}
	return 0;

err_exit:
	if (drv_data->txchan)
		dma_release_channel(drv_data->txchan);
	if (drv_data->rxchan)
		dma_release_channel(drv_data->rxchan);
	drv_data->rxchan = NULL;
	drv_data->txchan = NULL;
	return retval;

}
EXPORT_SYMBOL_GPL(intel_mid_i2s_command);


static void ssp1_dump_registers(struct intel_mid_i2s_hdl *drv_data)
{
	u32 irq_status;
	u32 status;

	void __iomem *reg = drv_data->ioaddr;
	struct device *ddbg = &(drv_data->pdev->dev);
	dev_dbg(ddbg, "Dump - Base Address = 0x%08X\n", (u32)reg);

	irq_status = read_SSSR(reg);
	dev_dbg(ddbg, "dump SSSR=0x%08X\n", irq_status);
	status = read_SSCR0(reg);
	dev_dbg(ddbg, "dump SSCR0=0x%08X\n", status);
	status = read_SSCR1(reg);
	dev_dbg(ddbg, "dump SSCR1=0x%08X\n", status);
	status = read_SSPSP(reg);
	dev_dbg(ddbg, "dump SSPSP=0x%08X\n", status);
	status = read_SSTSA(reg);
	dev_dbg(ddbg, "dump SSTSA=0x%08X\n", status);
	status = read_SSRSA(reg);
	dev_dbg(ddbg, "dump SSRSA=0x%08X\n", status);
	status = read_SSTO(reg);
	dev_dbg(ddbg, "dump SSTO=0x%08X\n", status);
	status = read_SSITR(reg);
	dev_dbg(ddbg, "dump SSITR=0x%08X\n", status);
	status = read_SSTSS(reg);
	dev_dbg(ddbg, "dump SSTSS=0x%08X\n", status);
	status = read_SSACD(reg);
	dev_dbg(ddbg, "dump SSACD=0x%08X\n", status);

	status = read_SSCR2(reg);
	dev_dbg(ddbg, "dump SSCR2=0x%08X\n", status);
	status = read_SSCR3(reg);
	dev_dbg(ddbg, "dump SSCR3=0x%08X\n", status);

}


/**
 * i2s_irq(): function that handles the SSP Interrupts (errors)
 * @irq : IRQ Number
 * @dev_id : structure that contains driver information
 *
 * This interrupts do nothing but warnings in case there is some problems
 * in I2S connection (underruns, overruns...). This may be reported by adding a
 * new interface to the driver, but not yet requested by "users" of this driver
 *
 * Output parameters
 *      NA
 */
static irqreturn_t i2s_irq(int irq, void *dev_id)
{
	irqreturn_t		 irq_status	= IRQ_NONE;
	struct intel_mid_i2s_hdl *drv_data	= dev_id;
	void __iomem		 *reg		= drv_data->ioaddr;
	struct device		 *ddbg		= &(drv_data->pdev->dev);
	/* SSSR register content */
	u32			 sssr		= 0;
	/* Monitored SSSR bit mask */
	u32			 sssr_masked	= 0;
	/* SSSR bits that need to be cleared after event handling */
	u32			 sssr_clr_mask	= 0;
	u32	isrx = 0;

#ifdef CONFIG_PM_SLEEP
	if (ddbg->power.is_prepared)
		goto i2s_irq_return;
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
	if (ddbg->power.runtime_status != RPM_ACTIVE)
		goto i2s_irq_return;
#endif /* CONFIG_PM_RUNTIME */

	sssr = read_SSSR(reg);
	sssr_masked = sssr & (drv_data->mask_sr);

	/* Any processing needed ? */
	if (!sssr_masked)
		goto i2s_irq_return;

	/*
	 * Some processing needs to be done
	 */
	irq_status = IRQ_HANDLED;

	/* may be improved by using a tasklet to send the error
	 * (underrun,...) to client by using callback
	 */
	/* Handle Receive Over Run event */
	if (sssr_masked & (SSSR_ROR_MASK << SSSR_ROR_SHIFT)) {
		dev_dbg(ddbg, "%s RX FIFO OVER RUN SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_ROR_MASK << SSSR_ROR_SHIFT);
	}
	/* Handle Transmit Under Run event */
	if (sssr_masked & (SSSR_TUR_MASK << SSSR_TUR_SHIFT)) {
		dev_dbg(ddbg, "%s TX FIFO UNDER RUN SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_TUR_MASK << SSSR_TUR_SHIFT);
	}
	/* Handle Receiver time-out INTerrupt event */
	if (sssr_masked & (SSSR_TINT_MASK << SSSR_TINT_SHIFT)) {
		dev_dbg(ddbg, "%s RX TIME OUT SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_TINT_MASK << SSSR_TINT_SHIFT);
	}
	/* Handle Peripheral trailing byte INTerrupt event */
	if (sssr_masked & (SSSR_PINT_MASK << SSSR_PINT_SHIFT)) {
		dev_dbg(ddbg, "%s TRAILING BYTE SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_PINT_MASK << SSSR_PINT_SHIFT);
	}
	/* Handle End Of Chain event */
	if (sssr_masked & (SSSR_EOC_MASK << SSSR_EOC_SHIFT)) {
		dev_dbg(ddbg, "%s END OF CHAIN SSSR=0x%08X\n",
			__func__, sssr);
		sssr_clr_mask |= (SSSR_EOC_MASK << SSSR_EOC_SHIFT);
	}

	/* Handle Receive Fifo Service request event */
	if (sssr_masked & (SSSR_RFS_MASK << SSSR_RFS_SHIFT))
		irq_status = i2s_irq_handle_RFS(drv_data, sssr);

	/* Handle Transmit Fifo Service request event */
	if (sssr_masked & (SSSR_TFS_MASK << SSSR_TFS_SHIFT))
		irq_status = i2s_irq_handle_TFS(drv_data, sssr);

	/* Handle BCE event */
	if (sssr_masked & (SSSR_BCE_MASK << SSSR_BCE_SHIFT)) {
		udelay(12);
		clear_SSCR0_reg(drv_data->ioaddr, SSE);
		while(read_SSCR0(drv_data->ioaddr) & 0x80);
	    	set_SSCR0_reg(drv_data->ioaddr, SSE);
	}

	/* Clear sticky bits */
	write_SSSR((sssr & sssr_clr_mask), reg);

#if 0
	/* FIXME: use of fixed addresses should be
	 * replaced by a call to SST driver that will
	 * take care to access LPE Shim registers */

	/* Clear LPE sticky bits depending on SSP instance */
	isrx = LPE_ISRX_IAPIS_SSP0_MASK << (LPE_ISRX_IAPIS_SSP0_SHIFT
					+ drv_data->device_instance);
	write_LPE_ISRX(isrx, lpeshim_base_address);
#endif /*  */

/*BYT : Clear the ISRX interrupt for SSP0
 *
 */
	if(SSP0_INSTANCE == drv_data->device_instance)
	  isrx = BYT_LPE_SHIM_IAPIS_SSP0_BIT; //Write bit 3 to clear SSP0 interrupt
	else if(SSP1_INSTANCE == drv_data->device_instance)
	  isrx = BYT_LPE_SHIM_IAPIS_SSP1_BIT; //Write bit 3 to clear SSP1 interrupt
	else if(SSP2_INSTANCE == drv_data->device_instance)
	  isrx = BYT_LPE_SHIM_IAPIS_SSP2_BIT; //Write bit 3 to clear SSP2 interrupt

	write_LPE_ISRX(isrx, drv_data->shim);

i2s_irq_return:
	return irq_status;
}

static inline
irqreturn_t i2s_irq_handle_RFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr)
{
	irqreturn_t	irq_status	= IRQ_HANDLED;
	u32		data_cnt	= 0;
	if (drv_data->current_settings.ssp_rx_dma == SSP_RX_DMA_ENABLE) {
		dev_WARN(&drv_data->pdev->dev,
			 "%s: DMA enabled, this function shouldn't be called",
			 __func__);
		goto i2s_irq_handle_RFS_return;
	}

	if ((drv_data->read_ptr.cpu == NULL) || (drv_data->read_len <= 0)) {
		dev_WARN(&drv_data->pdev->dev,
			 "%s: Invalid read param: addr=0x%08X, len=%i",
			 __func__, (int)drv_data->read_ptr.cpu,
			 drv_data->read_len);
		goto i2s_irq_handle_RFS_return;
	}

	/* Get available data count in receive FIFO */
	if (GET_SSSR_val(sssr, RNE))
		data_cnt = GET_SSSR_val(sssr, RFL) + 1;

	/* Read data from receive FIFO */
	while ((data_cnt > 0) && (drv_data->read_len > 0)) {
		*(drv_data->read_ptr.cpu) = (u32)read_SSDR(drv_data->ioaddr);
		drv_data->read_ptr.cpu++;
		drv_data->read_len--;
		data_cnt--;
	}

	/* Check for last data */
	if (drv_data->read_len <= 0) {
		/* Stop interruption */
		drv_data->mask_sr &= ~((SSSR_RFS_MASK << SSSR_RFS_SHIFT) |
				       (SSSR_ROR_MASK << SSSR_ROR_SHIFT));
		clear_SSCR1_reg((drv_data->ioaddr), RIE);
		change_SSCR0_reg(drv_data->ioaddr, RIM,
				 SSP_RX_FIFO_OVER_INT_DISABLE);

		/* Schedule irq thread for final treatment
		 * in i2s_irq_deferred */
		set_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags);
		irq_status = IRQ_WAKE_THREAD;
	}

i2s_irq_handle_RFS_return:
	return irq_status;
}

static inline
irqreturn_t i2s_irq_handle_TFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr)
{
	irqreturn_t	irq_status	= IRQ_HANDLED;
	u32		data_cnt	= FIFO_SIZE;
	if (drv_data->current_settings.ssp_tx_dma == SSP_TX_DMA_ENABLE) {
		dev_WARN(&drv_data->pdev->dev,
			 "%s: DMA enabled, this function shouldn't be called",
			 __func__);
		goto i2s_irq_handle_TFS_return;
	}

	if ((drv_data->write_ptr.cpu == NULL) || (drv_data->write_len <= 0)) {
		dev_WARN(&drv_data->pdev->dev,
			 "%s: Invalid write param: addr=0x%08X, len=%i",
			 __func__, (int)drv_data->write_ptr.cpu,
			 drv_data->write_len);
		goto i2s_irq_handle_TFS_return;
	}

	/* Get available space in transmit FIFO */
	if (GET_SSSR_val(sssr, TNF))
		data_cnt -= GET_SSSR_val(sssr, TFL);

	/* Write data to transmit FIFO */
	while ((data_cnt > 0) && (drv_data->write_len > 0)) {
		write_SSDR((u32)(*(drv_data->write_ptr.cpu)),
			   drv_data->ioaddr);
		drv_data->write_ptr.cpu++;
		drv_data->write_len--;
		data_cnt--;
	}

	/* Check for last data */
	if (drv_data->write_len <= 0) {
		/* Stop interruption */
		drv_data->mask_sr &= ~((SSSR_TFS_MASK << SSSR_TFS_SHIFT) |
				       (SSSR_TUR_MASK << SSSR_TUR_SHIFT));
		clear_SSCR1_reg((drv_data->ioaddr), TIE);
		change_SSCR0_reg(drv_data->ioaddr, TIM,
				 SSP_TX_FIFO_UNDER_INT_DISABLE);

		/* Schedule irq thread for final treatment
		 * in i2s_irq_deferred */
		set_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags);
		irq_status = IRQ_WAKE_THREAD;
	}

i2s_irq_handle_TFS_return:
	return irq_status;
}

static irqreturn_t i2s_irq_deferred(int irq, void *dev_id)
{
	/* Locals */
	struct intel_mid_i2s_hdl *drv_data = dev_id;

	dev_dbg(&(drv_data->pdev->dev), "%s", __func__);

	/* Finalize reading without DMA */
	if ((drv_data->current_settings.ssp_rx_dma != SSP_RX_DMA_ENABLE) &&
	    test_and_clear_bit(I2S_PORT_COMPLETE_READ, &drv_data->flags)) {

		if (!test_bit(I2S_PORT_READ_BUSY, &drv_data->flags)) {
			dev_warn(&drv_data->pdev->dev,
				 "%s: spurious read complete",
				 __func__);
		}

		dev_dbg(&(drv_data->pdev->dev),
			 "%s: read complete",
			 __func__);

		i2s_finalize_read(drv_data);

	/* Finalize writing without DMA */
	} else
	  if ((drv_data->current_settings.ssp_tx_dma != SSP_TX_DMA_ENABLE) &&
	      test_and_clear_bit(I2S_PORT_COMPLETE_WRITE, &drv_data->flags)) {

		if (!test_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags)) {
			dev_warn(&drv_data->pdev->dev,
				 "%s : spurious write complete",
				 __func__);
		}

		dev_dbg(&(drv_data->pdev->dev),
			"%s : write complete",
			__func__);

		i2s_finalize_write(drv_data);

	/* Error */
	} else {
		dev_warn(&drv_data->pdev->dev,
			 "%s: Unexpected function call"
			 "- flags=0x%08lx, rx_dma=%d, tx_dma=%d",
			 __func__,
			 drv_data->flags,
			 drv_data->current_settings.ssp_rx_dma,
			 drv_data->current_settings.ssp_tx_dma);
	}

	return IRQ_HANDLED;
}

static void i2s_finalize_read(struct intel_mid_i2s_hdl *drv_data)
{
	/* Mask Rx fifo overrun irq */
	change_SSCR0_reg(drv_data->ioaddr, RIM, SSP_RX_FIFO_OVER_INT_DISABLE);

	/* Reset read param:
	 *   must be done before call to the callback
	 *   where a read can be rearmed */
	drv_data->read_len	= 0;
	drv_data->read_ptr.cpu	= NULL;

	/* Do not change order sequence:
	 * READ_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_READ_BUSY, &drv_data->flags);
	if (!test_bit(I2S_PORT_CLOSING, &drv_data->flags)) {
		if (drv_data->read_callback != NULL)
			(void)drv_data->read_callback
					(drv_data->read_param);
		else
			dev_warn(&drv_data->pdev->dev,
				 "%s: no callback set",
				 __func__);
	}
}

static void i2s_finalize_write(struct intel_mid_i2s_hdl *drv_data)
{
	/* Mask Tx fifo underrun irq */
	change_SSCR0_reg(drv_data->ioaddr, TIM, SSP_TX_FIFO_UNDER_INT_DISABLE);

	/* Reset write param:
	 *   must be done before call to the callback
	 *   where a write can be rearmed */
	drv_data->write_len	= 0;
	drv_data->write_ptr.cpu	= NULL;

	/* Do not change order sequence:
	 * WRITE_BUSY clear, then test PORT_CLOSING
	 * wakeup for close() function
	 */
	clear_bit(I2S_PORT_WRITE_BUSY, &drv_data->flags);
	if (!test_bit(I2S_PORT_CLOSING, &drv_data->flags)) {
		if (drv_data->write_callback != NULL)
			(void)drv_data->write_callback
					(drv_data->write_param);
		else
			dev_warn(&drv_data->pdev->dev,
				 "%s: no callback set",
				 __func__);
	}
}

/**
 * calculate_sspsp_psp - separate function that calculate sspsp register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 *
 * Output parameters
 *      u32 : calculated SSPSP register
 */
static u32 calculate_sspsp_psp(const struct intel_mid_i2s_settings *ps_settings)
{
	u32 sspsp;
	sspsp = SSPSP_reg(FSRT,	ps_settings->ssp_frmsync_timing_bit)
		|SSPSP_reg(ETDS,	ps_settings->ssp_end_transfer_state)
		|SSPSP_reg(SCMODE,	ps_settings->ssp_serial_clk_mode)
		|SSPSP_reg(DMYSTOP,	ps_settings->ssp_psp_T4)
		|SSPSP_reg(SFRMDLY,	ps_settings->ssp_psp_T5)
		|SSPSP_reg(SFRMWDTH,ps_settings->ssp_psp_T6)
		|SSPSP_reg(SFRMP,	ps_settings->ssp_frmsync_pol_bit);
	return sspsp;
}

/**
 * calculate_sscr0_scr - separate function that calculate scr register (master)
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SCR register, or 0xFFFF if error
 */
static u32 calculate_sscr0_scr(struct intel_mid_i2s_hdl *drv_data,
		    const struct intel_mid_i2s_settings *ps_settings,
		    u8 *frame_rate_divider_new)
{

	long calculated_scr, remainder;
	u16 l_ssp_data_size = ps_settings->data_size;
	enum mrst_ssp_frm_freq freq = ps_settings->master_mode_standard_freq;
	struct device *ddbg = &(drv_data->pdev->dev);
	u16 delay = ps_settings->ssp_psp_T2 +
		    ps_settings->ssp_psp_T4 +
		    ps_settings->ssp_psp_T1;
	int frame_sync_length, hw_freq, requested_freq;

	dev_dbg(ddbg, "delay=%d\n", delay);

	/*
	 * A delay will be taken into account only if next frame sync is
	 * asserted at the end of T4 timing
	 */
	if ((ps_settings->ssp_frmsync_timing_bit !=
			NEXT_FRMS_ASS_AFTER_END_OF_T4) && (delay != 0)) {
		dev_warn(ddbg, "Review SSP config, clock not accurate\n");
	}

	dev_dbg(ddbg, "T1=%d, T2=%d, T4=%d, T5=%d, T6=%d\n",
		ps_settings->ssp_psp_T1, ps_settings->ssp_psp_T2,
		ps_settings->ssp_psp_T4, ps_settings->ssp_psp_T5,
		ps_settings->ssp_psp_T6);

	/* frequency */
	switch (freq) {
	case SSP_FRM_FREQ_8_000:
		requested_freq = 8000;
		break;
	case SSP_FRM_FREQ_11_025:
		requested_freq = 11025;
		break;
	case SSP_FRM_FREQ_16_000:
		requested_freq = 16000;
		break;
	case SSP_FRM_FREQ_22_050:
		requested_freq = 22050;
		break;
	case SSP_FRM_FREQ_44_100:
		requested_freq = 44100;
		break;
	case SSP_FRM_FREQ_48_000:
		requested_freq = 48000;
		break;
	default:
		dev_warn(ddbg, "frequency not unsupported\n");
		return 0xFFFF;
		break;
	};

	/* bits per sample */
	if ((l_ssp_data_size != 8) && (l_ssp_data_size != 16) &&
						(l_ssp_data_size != 32)) {
		dev_warn(ddbg, "Master mode bit per sample=%d unsupported\n",
							l_ssp_data_size);
		return 0xFFFF;
	}
	/*
	 * We consider that LPE selected 19.2Mhz CLK
	 */
	frame_sync_length = delay +
		ps_settings->frame_rate_divider_control * l_ssp_data_size;

	/*
	 * Calculate the divider, and the remainder of the division
	 */
	calculated_scr = CLOCK_SOURCE_KHZ / (requested_freq * frame_sync_length);
	remainder = CLOCK_SOURCE_KHZ % (requested_freq * frame_sync_length);

	dev_dbg(ddbg, "calculated_scr=%ld remainder=%ld\n",
						calculated_scr, remainder);

	/*
	 * If the rest is half the divider, increment the scr
	 */
	if (remainder > (frame_sync_length * requested_freq) >> 1) {
		calculated_scr += 1;
		dev_dbg(ddbg, "calculated_scr incremented=%ld\n",
						calculated_scr);
	}

	/*
	 * Calculate the real frequency that will be set up on HW
	 */
	hw_freq = CLOCK_SOURCE_KHZ / (calculated_scr * frame_sync_length);

	dev_dbg(ddbg, "hw_freq=%dHz High limit=%dHz Low limit=%dHz\n",
			hw_freq, (1004 * requested_freq / 1000),
			(996 * requested_freq / 1000));

	/*
	 * Allow the frequency to be requested frequency +/- 0,4%
	 */
	WARN(((hw_freq < (996 * requested_freq / 1000)) ||
				(hw_freq > (1004 * requested_freq / 1000))),
				"Master could not generate proper frequency");

	/*
	 * this do not change the frame_rate_divider, but may be optimized later
	 */
	*frame_rate_divider_new = ps_settings->frame_rate_divider_control;

	return (u32)((calculated_scr-1)<<SSCR0_SCR_SHIFT);
}


/**
 * calculate_ssacd - separate function that calculate ssacd register (master)
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSACD register, or 0xFFFF if error
 */
static u32 calculate_ssacd(struct intel_mid_i2s_hdl *drv_data,
		    const struct intel_mid_i2s_settings *ps_settings,
		    u8 *frame_rate_divider_new)
{
	u8 calculated_ssacd;
	u16 l_ssp_data_size = ps_settings->data_size;
	enum mrst_ssp_timeslot num_timeslot;
	enum mrst_ssp_bit_per_sample bit_per_sample;
	enum mrst_ssp_frm_freq freq = ps_settings->master_mode_standard_freq;
	struct device *ddbg = &(drv_data->pdev->dev);
	static u8 frame_rate_divider[SSP_TIMESLOT_SIZE] = {
		[SSP_TIMESLOT_1] = 1,
		[SSP_TIMESLOT_2] = 2,
		[SSP_TIMESLOT_4] = 4,
		[SSP_TIMESLOT_8] = 8 };



	/* timeslot */
	switch (ps_settings->frame_rate_divider_control) {
	case 1:
		num_timeslot = SSP_TIMESLOT_1;
		break;
	case 2:
		num_timeslot = SSP_TIMESLOT_2;
		break;
	case 4:
		num_timeslot = SSP_TIMESLOT_4;
		break;
	case 8:
		num_timeslot = SSP_TIMESLOT_8;
		break;
	default:
		dev_warn(ddbg, "Master mode timeslot=%d unsupported\n",
			ps_settings->frame_rate_divider_control);
		return 0xFFFF;
		break;
	};

	/* bits per sample */
	if (l_ssp_data_size == 8)
		bit_per_sample = SSP_BIT_PER_SAMPLE_8;
	else if (l_ssp_data_size == 16)
		bit_per_sample = SSP_BIT_PER_SAMPLE_16;
	else if (l_ssp_data_size == 32)
		bit_per_sample = SSP_BIT_PER_SAMPLE_32;
	else {
		dev_warn(ddbg, "Master mode bit per sample=%d unsupported\n",
				l_ssp_data_size);
		return 0xFFFF;
	}

/* 3 special cases when SSP clk force more than really used timeslots
 * to keep the wanted framesync freq. "Active" timeslots number do not change.
 * a) freq = 16Khz, active timeslot=1, bit per sample=8 => real timeslot=2
 * b) freq = 8Khz, active timeslot=1, bit per sample=8  => real timeslot=2
 * b) freq = 8Khz, active timeslot=1, bit per sample=16 => real timeslot=2
 * c) freq = 8Khz, active timeslot=2, bit per sample=8  => real timeslot=4
 * loop below will avoid these cases, and try to increment real timeslot to find
 * a working case.
 */

	while ((num_timeslot < SSP_TIMESLOT_8)
	 && (ssp_ssacd[freq][bit_per_sample][num_timeslot]
				== SSP_SSACD_NOT_AVAILABLE)) {
		num_timeslot++;
	}

/* Check of any unexpected error such as undefined freq with master mode */
	calculated_ssacd = ssp_ssacd[freq][bit_per_sample][num_timeslot];
	if (calculated_ssacd == SSP_SSACD_NOT_AVAILABLE) {
		dev_warn(ddbg, "Master mode unexpected error freq=%d,"
		"bitPerSamp=%d,calculated timeslot=%d, original timeslot=%d\n",
		freq, bit_per_sample, num_timeslot,
		ps_settings->frame_rate_divider_control);
		return 0xFFFF;
	}

	*frame_rate_divider_new = frame_rate_divider[num_timeslot];

	return (u32)(calculated_ssacd);
}


/**
 * calculate_sscr0_psp: separate function that calculate sscr0 register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSCR0 register
 */
u32 calculate_sscr0_psp(const struct intel_mid_i2s_settings *ps_settings)
{
	u16 l_ssp_data_size = ps_settings->data_size;
	u32 sscr0;

	if (l_ssp_data_size > 16) {
		sscr0 =   SSCR0_reg(DSS, SSCR0_DataSize(l_ssp_data_size - 16))
			| SSCR0_reg(EDSS, 1);
	} else {
		sscr0 =   SSCR0_reg(DSS, SSCR0_DataSize(l_ssp_data_size))
			| SSCR0_reg(EDSS, 0);
	}
/*
Can be replaced by code below :
sscr0 = SSCR0_reg(DSS, (l_ssp_data_size - 1) & 0x0F)
| SSCR0_reg(EDSS, ((l_ssp_data_size - 1) & 0x10) >> 8);
*/
	sscr0 |= SSCR0_reg(MOD,	ps_settings->mode)
		|SSCR0_reg(FRF,	ps_settings->frame_format)
		|SSCR0_reg(RIM,	SSP_RX_FIFO_OVER_INT_DISABLE)
		|SSCR0_reg(TIM,	SSP_TX_FIFO_UNDER_INT_DISABLE);
	return sscr0;
}

/**
 * calculate_sscr1_psp - separate function that calculate sscr1 register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSCR1 register
 */
u32 calculate_sscr1_psp(const struct intel_mid_i2s_settings *ps_settings)
{
	u32 sscr1;
	enum mrst_ssp_sspsfrm_direction i2s_frmdir_fix;
	i2s_frmdir_fix = ps_settings->sspsfrm_direction;

	if (ps_settings->frame_rate_divider_control == STEREO) /*I2S mode fix, Master & Slave*/
		i2s_frmdir_fix = SSPSFRM_SLAVE_MODE;

	sscr1 = SSCR1_reg(SFRMDIR, i2s_frmdir_fix)
	|SSCR1_reg(SCLKDIR, ps_settings->sspslclk_direction)
	|SSCR1_reg(EBCEI, 1)
	|SSCR1_reg(TTELP, ps_settings->tx_tristate_phase)
	|SSCR1_reg(TTE, ps_settings->tx_tristate_enable)
	|SSCR1_reg(TRAIL, ps_settings->ssp_trailing_byte_mode)
	|SSCR1_reg(TINTE, ps_settings->ssp_rx_timeout_interrupt_status)
	|SSCR1_reg(PINTE, ps_settings->ssp_trailing_byte_interrupt_status)
	|SSCR1_reg(LBM, ps_settings->ssp_loopback_mode_status)
	|SSCR1_reg(RWOT, ps_settings->ssp_duplex_mode)
	|SSCR1_reg(RFT, SSCR1_RxTresh(ps_settings->ssp_rx_fifo_threshold))
	|SSCR1_reg(TFT, SSCR1_TxTresh(ps_settings->ssp_tx_fifo_threshold));
	return sscr1;
}

/**
 * calculate_sscr2 - separate function that calculate sscr2 register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSCR2 register
 */
u32 calculate_sscr2(const struct intel_mid_i2s_settings *ps_settings,  u16 l_ssp_clk_frm_mode)
{
	u32 sscr2 = 0;
	if (ps_settings->frame_rate_divider_control == TDM8) {/*TDM8*/
		/*Fix for underrun conditions - New data will start on the underrun slot*/
		sscr2 |= (1 << SSCR2_UNDERRUN_FIX_1_SHIFT);
		if (l_ssp_clk_frm_mode == SSP_IN_SLAVE_MODE)
			sscr2 |= (1 << SSCR2_SLV_EXT_CLK_RUN_EN_SHIFT);
	}

	return sscr2;
}

/**
 * calculate_sscr3 - separate function that calculate sscr3 register
 * @ps_settings : pointer of the settings struct
 *
 * this function is to simplify/clarify set_ssp_i2s_hw function
 *
 * Output parameters
 *      u32 : calculated SSCR3 register
 */
u32 calculate_sscr3(const struct intel_mid_i2s_settings *ps_settings,  u16 l_ssp_clk_frm_mode)
{
	u32 sscr3 = 0;

	/*SW preserving default value*/
	sscr3 = ((1 << SSCR3_STRETCH_RX_SHIFT)
			| (1 << SSCR3_STRETCH_TX_SHIFT));

	if (ps_settings->frame_rate_divider_control == STEREO) { /*I2S fix*/
		sscr3 |= ((1 << SSCR3_I2S_RX_SS_FIX_EN_SHIFT)
				|(1 << SSCR3_I2S_TX_SS_FIX_EN_SHIFT)
				|(1 << SSCR3_I2S_MODE_EN_SHIFT)
				| 0x20004);
		if (l_ssp_clk_frm_mode == SSP_IN_MASTER_MODE) { /*Master mode*/
			sscr3 |= ((1 << SSCR3_MST_CLK_EN_SHIFT)
					|(1 << SSCR3_FRM_MST_EN_SHIFT));
		}/*I2S fix*/

	} else if ((ps_settings->frame_rate_divider_control >= STEREO)
			&& (l_ssp_clk_frm_mode == SSP_IN_MASTER_MODE)) { /*TDM8 Master mode*/

		sscr3 |= 0x20;
	}

	return sscr3;
}

/**
 * set_ssp_i2s_hw - configure the SSP driver according to the ps_settings
 * @drv_data : structure that contains all details about the SSP Driver
 * @ps_settings : structure that contains SSP Hardware settings
 *
 * it also store ps_settings the drv_data
 *
 * Output parameters
 *      NA
 */
static void set_ssp_i2s_hw(struct intel_mid_i2s_hdl *drv_data,
			const struct intel_mid_i2s_settings *ps_settings)
{
	u32 sscr0 = 0;
	u32 sscr1 = 0;
	u32 sscr2 = 0;
	u32 sscr3 = 0;
	u32 sscr4 = 0;
	u32 sscr5 = 0;
	u32 sstsa = 0;
	u32 ssrsa = 0;
	u32 sspsp = 0;
	u32 sssr = 0;
	u32 ssacd = 0;
	u32 sscr0_scr;

	u32 ssp_divc_l = 0;
	u32 ssp_divc_h = 0;

#ifdef __MRFL_SPECIFIC__
	u32 sfifott = 0;
#endif /* __MRFL_SPECIFIC__ */
	u8 frame_rate_divider;

	/* Get the SSP Settings */
	u16 l_ssp_clk_frm_mode = 0xFF;
	void __iomem *reg = drv_data->ioaddr;
	struct device *ddbg = &(drv_data->pdev->dev);
	dev_dbg(ddbg, "setup SSP I2S PCM1 configuration\n");

	/*
	 * Save the current SSP Configuration
	 */
	drv_data->current_settings = *ps_settings;

	if ((ps_settings->sspsfrm_direction == SSPSFRM_MASTER_MODE)
	   && (ps_settings->sspslclk_direction == SSPSCLK_MASTER_MODE)) {
		l_ssp_clk_frm_mode = SSP_IN_MASTER_MODE;
	} else if ((ps_settings->sspsfrm_direction == SSPSFRM_SLAVE_MODE)
		  && (ps_settings->sspslclk_direction == SSPSCLK_SLAVE_MODE)) {
		l_ssp_clk_frm_mode = SSP_IN_SLAVE_MODE;
	} else {
		dev_err(ddbg, "Unsupported I2S PCM1 configuration\n");
		goto leave;
	}
	frame_rate_divider = ps_settings->frame_rate_divider_control;

	dev_dbg(ddbg, "SSPSFRM_DIRECTION:%d:\n",
		ps_settings->sspsfrm_direction);
	dev_dbg(ddbg, "SSPSCLK_DIRECTION:%d:\n",
		ps_settings->sspslclk_direction);
	if (ps_settings->frame_format != PSP_FORMAT) {
		dev_err(ddbg, "UNSUPPORTED FRAME FORMAT:%d:\n",
			ps_settings->frame_format);
		goto leave;
	}

	/*********** DMA Transfer Mode ***********/
	dev_dbg(ddbg, "FORMAT :%d:\n", ps_settings->frame_format);
	sscr0 = calculate_sscr0_psp(ps_settings);
	dev_dbg(ddbg, " sscr0 :0x%08X\n", sscr0);
	sscr1 = calculate_sscr1_psp(ps_settings);
	dev_dbg(ddbg, " sscr1 :0x%08X\n", sscr1);

	if (ps_settings->mode == SSP_IN_NETWORK_MODE) {
		dev_dbg(ddbg, "MODE :%d:\n", ps_settings->mode);
		sscr0 |= SSCR0_reg(FRDC, SSCR0_SlotsPerFrm(frame_rate_divider));
		dev_dbg(ddbg, "sscr0 :0x%08X\n", sscr0);

	} else if(ps_settings->mode != SSP_IN_NORMAL_MODE){
		dev_err(ddbg, "UNSUPPORTED MODE");
		goto leave;
	}

	sspsp = calculate_sspsp_psp(ps_settings);
	dev_dbg(ddbg, "sspsp :0x%08X\n", sspsp);
	/* set the active TX time slot (bitmap) */
	sstsa = SSTSA_reg(TTSA, ps_settings->ssp_active_tx_slots_map);
	/* set the active RX time slot (bitmap) */
	ssrsa = SSRSA_reg(RTSA, ps_settings->ssp_active_rx_slots_map);

	/*
	 *  Master mode requires clk_output_enable in LPE_SHIM_CLKCTL
	 */
	if (l_ssp_clk_frm_mode == SSP_IN_MASTER_MODE) {
		switch (ps_settings->master_mode_clk_selection) {
		case SSP_ONCHIP_CLOCK:
			/* Baytrail to use M/N divider to generate clock.
			 * M/N divider directly supplies desired clock rate to SSP.
			 */
			if (drv_data->pdev->device == SST_BYT_PCI_ID) {
				sscr0_scr = 0<<SSCR0_SCR_SHIFT;
			} else {
				sscr0_scr = calculate_sscr0_scr(drv_data,
						ps_settings, &frame_rate_divider);
				if (ssacd == 0xFFFF) {
					dev_err(ddbg,
							"Error during SCR calculation");
					goto leave;
				}
			}
			sscr0 |= sscr0_scr;
			break;
		case SSP_NETWORK_CLOCK:
			sscr0 |= SSCR0_reg(NCS, 1);
			break;
		case SSP_EXTERNAL_CLOCK:
			sscr0 |= SSCR0_reg(ECS, 1);
			break;
		case SSP_ONCHIP_AUDIO_CLOCK:
			/*
			 * this mode may not be functionnal
			 * on all chipsets.
			 */
			ssacd = calculate_ssacd(drv_data,
					ps_settings, &frame_rate_divider);
			if (ssacd == 0xFFFF) {
				dev_err(ddbg,
						"Error during SSACD calculation");
				goto leave;
			}
			sscr0 |= SSCR0_reg(ACS, 1);
			break;
		default:
			dev_err(ddbg, "Master Mode clk select UNKNOWN");
			break;
		}

		if(frame_rate_divider == STEREO) {
			/*Frame control registers for I2S mode*/
			sscr5 = ps_settings->ssp_psp_T6;
			sscr4 = (sscr5 + 1) *  frame_rate_divider;
			sscr4 = SSCR4_reg(TOT_FRM_PRD, sscr4);
			sscr5 = SSCR5_reg(FRM_ASRT_WIDTH, sscr5);
		}

		sspsp |= SSPSP_reg(STRTDLY, ps_settings->ssp_psp_T1)
					|SSPSP_reg(DMYSTRT, ps_settings->ssp_psp_T2);

	} else {	/* Set the Slave Clock Free Running Status */
		sscr1 |= SSCR1_reg(SCFR,
				ps_settings->slave_clk_free_running_status);

	}

	if(frame_rate_divider == STEREO){

		sspsp = sspsp & CLRBIT_SFRMWDTH; /*clear SFRMWDTH*/
		sspsp |= SSPSP_reg(SFRMWDTH, 0x1) /*Overwrite T6: I2S fix for frmclk*/
				|SSPSP_reg(STRTDLY, ps_settings->ssp_psp_T1)
				|SSPSP_reg(DMYSTRT, ps_settings->ssp_psp_T2);
	}

	/*Set M/N Div Controller*/
	ssp_divc_h |= LPE_SSP0_DIVC_H_reg(DIV_BYPASS, ps_settings->ssp_divider_bypass)
			|LPE_SSP0_DIVC_H_reg(DIV_EN, ps_settings->ssp_divider_enable)
			|LPE_SSP0_DIVC_H_reg(DIV_UPDATE, ps_settings->ssp_divider_update)
			|LPE_SSP0_DIVC_H_reg(DIV_M, ps_settings->m_value);
	ssp_divc_l |= LPE_SSP0_DIVC_L_reg(DIV_N, ps_settings->n_value);

	sscr2 |= calculate_sscr2(ps_settings, l_ssp_clk_frm_mode);
	sscr3 |= calculate_sscr3(ps_settings, l_ssp_clk_frm_mode);

	/* Set SSP status mask */
	drv_data->mask_sr = ((SSSR_BCE_MASK << SSSR_BCE_SHIFT) |
			     (SSSR_EOC_MASK << SSSR_EOC_SHIFT) |
			     (SSSR_TINT_MASK << SSSR_TINT_SHIFT) |
			     (SSSR_PINT_MASK << SSSR_PINT_SHIFT));

	if (drv_data->current_settings.ssp_rx_dma == SSP_RX_DMA_ENABLE)
		drv_data->mask_sr |= (SSSR_ROR_MASK << SSSR_ROR_SHIFT);

	if (drv_data->current_settings.ssp_tx_dma == SSP_TX_DMA_ENABLE)
		drv_data->mask_sr |= (SSSR_TUR_MASK << SSSR_TUR_SHIFT);

	/* Clear status */
	sssr = (SSSR_BCE_MASK << SSSR_BCE_SHIFT)
	     | (SSSR_TUR_MASK << SSSR_TUR_SHIFT)
	     | (SSSR_TINT_MASK << SSSR_TINT_SHIFT)
	     | (SSSR_PINT_MASK << SSSR_PINT_SHIFT)
	     | (SSSR_ROR_MASK << SSSR_ROR_SHIFT);

#ifdef __MRFL_SPECIFIC__
	sfifott = replace_SFIFOTT_RFT(sfifott,
			SSCR1_RxTresh(ps_settings->ssp_rx_fifo_threshold));
	sfifott = replace_SFIFOTT_TFT(sfifott,
			SSCR1_TxTresh(ps_settings->ssp_tx_fifo_threshold));
	dev_dbg(ddbg, "WRITE SFIFOTT: 0x%08X\n", sfifott);
#endif /* __MRFL_SPECIFIC__ */

	/* disable SSP */
	i2s_disable(drv_data);
	dev_dbg(ddbg, "WRITE SSCR0 DISABLE\n");

	/* first set CR1 without interrupt and service enables */
	write_SSCR1(sscr1, reg);
	write_SSCR2(sscr2, reg);
	write_SSCR3(sscr3, reg);
	if (frame_rate_divider == STEREO && l_ssp_clk_frm_mode == SSP_IN_MASTER_MODE) {
		write_SSCR4(sscr4, reg);
		write_SSCR5(sscr5, reg);
	}

	write_SSPSP(sspsp, reg);
	write_SSTSA(sstsa, reg);
	write_SSRSA(ssrsa, reg);
//	write_SSACD(ssacd, reg);

	/* Clear status */
	write_SSSR(sssr, reg);
	dev_dbg(ddbg, "WRITE SSSR: 0x%08X\n", sssr);
	write_SSCR0(sscr0, reg);
	dev_dbg(ddbg, "WRITE SSCR0\n");

	/* set the time out for the reception */
	write_SSTO(0, reg);

	reg = drv_data->shim;
	if(SSP0_INSTANCE == drv_data->device_instance)
	{
	    write_LPE_SSP0_DIVC_L(ssp_divc_l, reg);
	    write_LPE_SSP0_DIVC_H(ssp_divc_h, reg);
	}
	else if(SSP1_INSTANCE == drv_data->device_instance)
	{
	    write_LPE_SSP1_DIVC_L(ssp_divc_l, reg);
	    write_LPE_SSP1_DIVC_H(ssp_divc_h, reg);
	}
	else if(SSP2_INSTANCE == drv_data->device_instance)
	{
	    write_LPE_SSP2_DIVC_L(ssp_divc_l, reg);
	    write_LPE_SSP2_DIVC_H(ssp_divc_h, reg);
	}

	ssp1_dump_registers(drv_data);

leave:
	return;
}

static int
intel_mid_i2s_find_usage(struct pci_dev *pdev,
			 struct intel_mid_i2s_hdl *drv_data,
			 enum intel_mid_i2s_ssp_usage *usage/*,
			 struct intel_mid_ssp_gpio *ssp_gpio*/)
{
	int status = 0;

	*usage = SSP_USAGE_UNASSIGNED;
	switch ( (drv_data->paddr & 0xFFFFF) ) {
		case BYT_SSP0_START_ADDRESS:
			*usage	= SSP_USAGE_CARD_AK4614_0;
			pr_debug("%s : Matched usage to AK4614 codec 1\n", __func__);
			break;
		case BYT_SSP1_START_ADDRESS:
		  	*usage	= SSP_USAGE_CARD_AK4614_1;
			pr_debug("%s : Matched usage to AK4614 codec 2\n", __func__);
			break;
		case BYT_SSP2_START_ADDRESS:
			*usage	= SSP_USAGE_CARD_AK4614_2;
			pr_debug("%s : Matched usage to AK4614 codec 3\n", __func__);
			break;
		default:
			pr_err("%s: No device 0x%08x found\n", __func__, (unsigned int)(drv_data->paddr & 0xFFFFF));
			status = -EINVAL;
	}
	return status;
}

static int ssp_byt_irq_enable(struct pci_dev *dev, unsigned int irq) {

	struct io_apic_irq_attr irq_attr;
	int ret = 0;
	dev->irq=irq;
	irq_attr.ioapic = mp_find_ioapic(irq);
		if(irq_attr.ioapic<0){
			printk(KERN_ERR "ERROR: No IOAPIC for IRQ=%d DID=0x%x \n",dev->irq, dev->device);
			return irq_attr.ioapic;
		}
	irq_attr.ioapic_pin = irq; 
	irq_attr.trigger = 1; /* level */
	irq_attr.polarity = 1; /* active low */
	ret = io_apic_set_pci_routing(&dev->dev, irq, &irq_attr);
	if(ret) {
		dev_err((&dev->dev), "Fail to route IO_APIC irq to PCI\n");
		return ret;
	}
	return 0;

}


/**
 * intel_mid_i2s_probe - probing function for the pci selected
 * @pdev : pci_dev pointer that is probed
 * @ent : pci_device_id
 *
 * Output parameters
 *      NA
 */
int intel_mid_i2s_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	enum intel_mid_i2s_ssp_usage usage;
	int i, num_of_ia_ssp, port_number = 0, status = 0;
#ifndef SSP_DRIVER_ON
        struct platform_device *asoc_pdev;
        int ret = 0;
#endif

	dev_info((&pdev->dev), "Detected PCI SSP (ID: %04x:%04x)\n",
				pdev->vendor, pdev->device);
	
	pci = kzalloc(sizeof(struct lpe_device), GFP_KERNEL);
	
	if (!pci) 
	{
	      dev_err((&pdev->dev), "PCI container malloc fail\n");
	      return -ENOMEM;
	}
	
	num_of_ia_ssp = get_number_of_ia_ssp();
	
	for(i=0; i<num_of_ia_ssp; i++)
	{
		port_number = get_ia_port_number();
		i2s_drv_data[port_number] = kzalloc(sizeof(struct intel_mid_i2s_hdl), GFP_KERNEL);
		dev_dbg(&(pdev->dev), "%s Probe,drv_data =%p\n", DRIVER_NAME, i2s_drv_data[port_number]);
		if (!i2s_drv_data[port_number]) 
	    	{
		   dev_err((&pdev->dev), "Can't alloc driver_data[%d] in probe\n",port_number);
		   status = -ENOMEM;
		   goto leave;
	    	}
	   	i2s_drv_data[port_number]->device_instance = port_number;
	}
	
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
	    pci->ssp_ctx[i] = kzalloc(sizeof(*pdev), GFP_KERNEL);

	    memcpy(pci->ssp_ctx[i], pdev, sizeof(*pdev));
	      
	    if (!pci->ssp_ctx[i]) 
	    {
		kfree(pci);
		for(i=0; i<NUMBER_OF_SSP_PORT; i++)
		{
			if (i2s_drv_data[i] != NULL)
		  		kfree(i2s_drv_data[i]);
			return -ENODEV;
	    	}
	    }
	}

#ifndef SSP_DRIVER_ON
	/* Enable device */
		status = pci_enable_device(pci->ssp_ctx);
	if (status) {
		dev_err((&pci->ssp_ctx->dev), "Can not enable device.Err=%d\n", status);
		goto err_i2s_probe0;
	}
#endif
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
	   		mutex_init(&i2s_drv_data[i]->mutex);
	}
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
		{
			if(SSP0_INSTANCE == i2s_drv_data[i]->device_instance)
			{
				ssp_byt_irq_enable(pci->ssp_ctx[i], BYT_SSP0_IRQ);
				i2s_drv_data[i]->pdev = pci->ssp_ctx[i];

				i2s_drv_data[i]->paddr  = pci_resource_start(pci->ssp_ctx[i], 0) + BYT_LPE_SHIM_BASE_ADDRESS;
				i2s_drv_data[i]->shim = ioremap(i2s_drv_data[i]->paddr, BYT_LPE_SHIM_SIZE);

				i2s_drv_data[i]->paddr = pci_resource_start(pci->ssp_ctx[i], 0) + BYT_SSP0_START_ADDRESS;
				i2s_drv_data[i]->iolen = BYT_SSP_SIZE;
				i2s_drv_data[i]->pinternal_addr = BYT_SSP0_INTERNAL_ADDRESS;

				i2s_drv_data[i]->ioaddr = ioremap(i2s_drv_data[i]->paddr, i2s_drv_data[i]->iolen);
				if (!i2s_drv_data[i]->ioaddr)
				{
					dev_err((&pci->ssp_ctx[i]->dev), "ioremap_nocache error\n");
					status = -ENOMEM;
					goto err_i2s_probe2;
				}
			} /* SSP0 instance */
			else if(SSP1_INSTANCE == i2s_drv_data[i]->device_instance)
			{
				ssp_byt_irq_enable(pci->ssp_ctx[i], BYT_SSP1_IRQ);
				i2s_drv_data[i]->pdev = pci->ssp_ctx[i];

				i2s_drv_data[i]->paddr  = pci_resource_start(pci->ssp_ctx[i], 0) + BYT_LPE_SHIM_BASE_ADDRESS;
				i2s_drv_data[i]->shim = ioremap(i2s_drv_data[i]->paddr, BYT_LPE_SHIM_SIZE);

				i2s_drv_data[i]->paddr = pci_resource_start(pci->ssp_ctx[i], 0) + BYT_SSP1_START_ADDRESS;
				i2s_drv_data[i]->iolen = BYT_SSP_SIZE;
				i2s_drv_data[i]->pinternal_addr = BYT_SSP1_INTERNAL_ADDRESS;

				i2s_drv_data[i]->ioaddr = ioremap(i2s_drv_data[i]->paddr, i2s_drv_data[i]->iolen);
				if (!i2s_drv_data[i]->ioaddr)
				{
					dev_err((&pci->ssp_ctx[i]->dev), "ioremap_nocache error\n");
					status = -ENOMEM;
					goto err_i2s_probe2;
				}
			} /* SSP1 instance */
			else if(SSP2_INSTANCE == i2s_drv_data[i]->device_instance)
			{
				ssp_byt_irq_enable(pci->ssp_ctx[i], BYT_SSP2_IRQ);
				i2s_drv_data[i]->pdev = pci->ssp_ctx[i];

				i2s_drv_data[i]->paddr  = pci_resource_start(pci->ssp_ctx[i], 0) + BYT_LPE_SHIM_BASE_ADDRESS;
				i2s_drv_data[i]->shim = ioremap(i2s_drv_data[i]->paddr, BYT_LPE_SHIM_SIZE);

				i2s_drv_data[i]->paddr = pci_resource_start(pci->ssp_ctx[i], 0) + BYT_SSP2_START_ADDRESS;
				i2s_drv_data[i]->iolen = BYT_SSP_SIZE;
				i2s_drv_data[i]->pinternal_addr = BYT_SSP2_INTERNAL_ADDRESS;

				i2s_drv_data[i]->ioaddr = ioremap(i2s_drv_data[i]->paddr, i2s_drv_data[i]->iolen);
				if (!i2s_drv_data[i]->ioaddr)
				{
					dev_err((&pci->ssp_ctx[i]->dev), "ioremap_nocache error\n");
					status = -ENOMEM;
					goto err_i2s_probe2;
				}
			} /* SSP2 instance */

		dev_dbg(&(pci->ssp_ctx[i]->dev), "ioaddr = : %p\n", i2s_drv_data[i]->ioaddr);
		} /* end of if condition */
	} /*end of for loop */

	/* Find SSP usage */
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
		{
	      		status = intel_mid_i2s_find_usage(pci->ssp_ctx[i], i2s_drv_data[i], &usage/*,*/ /*&ssp_fs_pin*/);	//---------------------Check this function again
	      		i2s_drv_data[i]->usage = usage;

	      		if (status)
	      			goto err_i2s_probe3;
		} /* end of if condition */
	} /*end of for loop */

	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
		{
	    		/* prepare for DMA channel allocation */
	    		/* get the pci_dev structure pointer */
	    		i2s_drv_data[i]->dmac1 = pdev;
	    		WARN(!i2s_drv_data[i]->dmac1, "%s : pdev yielded DMA device=NULL\n", __func__);
	    
	    		pci_set_drvdata(pci->ssp_ctx[i], i2s_drv_data[i]);
	    	} /* end of if condition */
	} /*end of for loop */

	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
		{
	      		/* set SSP FrameSync and CLK direction in INPUT mode in order
	       		* to avoid disturbing peripherals
	       		*/
	      		write_SSCR1((SSCR1_SFRMDIR_MASK << SSCR1_SFRMDIR_SHIFT)
		   	| (SSCR1_SCLKDIR_MASK << SSCR1_SCLKDIR_SHIFT)
		   	| (SSCR1_TTE_MASK << SSCR1_TTE_SHIFT),
		   	i2s_drv_data[i]->ioaddr);
		   
		  	/* Attach to IRQ */
		  	i2s_drv_data[i]->irq = pci->ssp_ctx[i]->irq;
		  	dev_dbg(&(pci->ssp_ctx[i]->dev), "attaching to IRQ: %04x\n", pci->ssp_ctx[i]->irq);
		  
		  	if(SSP0_INSTANCE == i2s_drv_data[i]->device_instance)
		  	{
				status = request_threaded_irq(i2s_drv_data[i]->irq,
					  i2s_irq,
					  i2s_irq_deferred,
					  IRQF_SHARED,
					  "i2s_ssp0",
					  i2s_drv_data[i]);
		  	}
		  	else if(SSP1_INSTANCE == i2s_drv_data[i]->device_instance)
		  	{
				status = request_threaded_irq(i2s_drv_data[i]->irq,
					  i2s_irq,
					  i2s_irq_deferred,
					  IRQF_SHARED,
					  "i2s_ssp1",
					  i2s_drv_data[i]);
		  	}
			else if(SSP2_INSTANCE == i2s_drv_data[i]->device_instance)
		  	{
				status = request_threaded_irq(i2s_drv_data[i]->irq,
					  i2s_irq,
					  i2s_irq_deferred,
					  IRQF_SHARED,
					  "i2s_ssp2",
					  i2s_drv_data[i]);
		  	}

		  	if (status < 0)	
		  	{
			 	dev_err(&pci->ssp_ctx[i]->dev, "can not get IRQ. status err=%d\n", status);
			 	goto err_i2s_probe3;
		  	}
		} /* end of if condition */
	} /*end of for loop */
	
	for (i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
		{
			writel(0x0E, i2s_drv_data[i]->shim + 0x88);
			break;
		} /* end of if condition */
	} /*end of for loop */
	
	goto leave;
err_i2s_probe3:
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
	  		iounmap(i2s_drv_data[i]->ioaddr);
	 }
err_i2s_probe2:
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
	  pci_release_region(pci->ssp_ctx[i], MRST_SSP_BAR);
	  pci_disable_device(pci->ssp_ctx[i]);
	}
#ifndef SSP_DRIVER_ON
err_i2s_probe0:
#endif
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
		if (i2s_drv_data[i] != NULL)
			kfree(i2s_drv_data[i]);
	  kfree(pci->ssp_ctx[i]);
	}
	kfree(pci);
leave:
	return status;
}
EXPORT_SYMBOL_GPL(intel_mid_i2s_probe);

#if 0
//static void __devexit intel_mid_i2s_remove(struct pci_dev *pdev)
static void intel_mid_i2s_remove(struct pci_dev *pdev)
{
	struct intel_mid_i2s_hdl *drv_data;
	enum intel_mid_i2s_ssp_usage usage;
#ifndef SSP_DRIVER_ON	
	u32 device = pdev->device;
#endif

	//drv_data = pci_get_drvdata(pdev);
	drv_data = pci_get_drvdata(pci->ssp_ctx);
	if (!drv_data) {
		dev_err(&pdev->dev, "no drv_data in pci device to remove!\n");
		goto leave;
	}
	/*FIX: Usage not in used without SFI */
	usage = drv_data->usage;
	if (test_bit(I2S_PORT_OPENED, &drv_data->flags)) {
		dev_warn(&pdev->dev, "Not closed before removing pci_dev!\n");
		intel_mid_i2s_close(drv_data);
	}
	//pci_set_drvdata(pdev, NULL);
	pci_set_drvdata(pci->ssp_ctx, NULL);
	/* Stop DMA is already done during close()  */
	pci_dev_put(drv_data->dmac1);
	/* Disable the SSP at the peripheral and SOC level */
	write_SSCR0(0, drv_data->ioaddr);
	free_irq(drv_data->irq, drv_data);
	iounmap(drv_data->ioaddr);
	pci_release_region(pci->ssp_ctx/*pdev*/, MRST_SSP_BAR);
	pci_release_region(pci->ssp_ctx/*pdev*/, MRST_LPE_BAR);
	pci_disable_device(pci->ssp_ctx/*pdev*/);
	kfree(drv_data);
#ifndef SSP_DRIVER_ON
	if (usage == SSP_USAGE_MODEM) {
		clear_bit(MODEM_FND, &modem_found_and_i2s_setup_ok);
		if (intel_mid_i2s_modem_remove_cb)
			(*intel_mid_i2s_modem_remove_cb)();
	}
	if (device == CLV_SSP0_DEVICE_ID)
		clear_bit(CLV_SSP0_FND, &clv_ssps_found);
	if (device == CLV_SSP1_DEVICE_ID)
		clear_bit(CLV_SSP1_FND, &clv_ssps_found);
#endif
leave:
	return;
}

EXPORT_SYMBOL_GPL(intel_mid_i2s_remove);
#endif
