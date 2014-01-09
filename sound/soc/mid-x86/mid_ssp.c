/*
 *	mid_ssp.c - ASoC CPU DAI driver for
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author:
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

#define FORMAT(fmt) "%s: " fmt, __func__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#include <linux/module.h>
#include "mid_ssp.h"
#include <asm/platform_byt_audio.h>

struct intel_ssp_info *byt_ssp_info[NUMBER_OF_SSP_PORT];

static const struct snd_soc_component_driver ssp_component = {
        .name           = "ssp",
};

/*
 * SSP DAI Internal functions
 */

/**
 * ssp_dma_req - This function programs a write or read request
 * to the Intel I2S driver
 *
 * @param substream Pointer to stream structure
 * return ret_val Status
 */
static int ssp_dma_req(struct snd_pcm_substream *substream)
{

	struct intel_alsa_ssp_stream_info *str_info;
	struct intel_ssp_config *ssp_config;
	struct snd_pcm_runtime *pl_runtime;
	u32 *dma_addr;
	int ret;
#ifdef USE_DMA_LLI
	int i;
	struct intel_mid_i2s_lli *dma_lli_list;
#endif

	WARN(!substream, "SSP DAI: "
			"ERROR NULL substream\n");
	if (!substream)
		return -EINVAL;


	pl_runtime = substream->runtime;

	str_info = pl_runtime->private_data;

	WARN(!str_info, "SSP DAI: "
			"ERROR NULL str_info\n");
	if (!str_info)
		return -EINVAL;

	ssp_config = str_info->ssp_config;

	WARN(!ssp_config, "SSP DAI: "
			"ERROR NULL ssp_config\n");
	if (!ssp_config)
		return -EINVAL;


	WARN(!ssp_config->i2s_handle, "SSP DAI: "
			"ERROR, trying to play a stream however "
			"ssp_config->i2s_handle is NULL\n");

	if (!ssp_config->i2s_handle)
		return -EINVAL;

	str_info->length = frames_to_bytes(pl_runtime, pl_runtime->period_size);

	str_info->addr = substream->runtime->dma_area;
	pr_debug("SSP DAI: FCT %s substream->runtime->dma_area = %p",
				__func__, substream->runtime->dma_area);

	dma_addr = (u32 *)(str_info->addr +
				str_info->length * str_info->period_req_index);

#ifdef USE_DMA_LLI
	dma_lli_list = kzalloc(sizeof(struct intel_mid_i2s_lli) * pl_runtime->periods, GFP_KERNEL);
	if(!dma_lli_list)
	{
		pr_err("ERROR: unable to create i2s lli array\n");
		return -ENOMEM;
	}

	for (i = 0; i < pl_runtime->periods; i++)
	{
		dma_lli_list[i].addr = (u32 *)(str_info->addr + (i * str_info->length));
		dma_lli_list[i].leng = str_info->length;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = intel_mid_i2s_lli_wr_req(ssp_config->i2s_handle, dma_lli_list,
			pl_runtime->periods, I2S_CIRCULAR_MODE, substream);
	else
		ret = intel_mid_i2s_lli_rd_req(ssp_config->i2s_handle, dma_lli_list,
			pl_runtime->periods, I2S_CIRCULAR_MODE, substream);
#else
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = intel_mid_i2s_wr_req(ssp_config->i2s_handle, dma_addr,
			str_info->length, substream);
	else
		ret = intel_mid_i2s_rd_req(ssp_config->i2s_handle, dma_addr,
			str_info->length, substream);
#endif

	if (ret == 0) {

		intel_mid_i2s_command(ssp_config->i2s_handle,
				SSP_CMD_ENABLE_SSP, &(ssp_config->i2s_settings),
				substream->stream);

		if (test_and_set_bit(INTEL_ALSA_SSP_STREAM_RUNNING,
				&str_info->stream_status)) {

			pr_err("SSP DAI: "
				"ERROR previous requested not handled\n");
			ret = -EBUSY;
			goto exit;
		}

		if (++(str_info->period_req_index) >= pl_runtime->periods)
			str_info->period_req_index = 0;
		ret = 0;
	} else {
		pr_err("SSP DAI: FCT %s read/write req ERROR\n",
				__func__);
		ret = -EINVAL;
	}
exit:
#ifdef USE_DMA_LLI
	kfree(dma_lli_list);
#endif

	return ret;
} /* ssp_dma_req */

/**
 * ssp_dma_complete - End of capture or playback callback
 * called in DMA Complete Tasklet context
 * This Callback has in charge of re-programming a new read or write
 * request to Intel MID I2S Driver if the stream has not been Closed.
 * It calls also the snd_pcm_period_elapsed if the stream is not
 * PAUSED or SUSPENDED to inform ALSA Kernel that the Ring Buffer
 * period has been sent or received properly
 *
 * @param param Pointer to a structure
 * return status
 */
static int ssp_dma_complete(void *param)
{

	struct snd_pcm_substream *substream;
	struct intel_alsa_ssp_stream_info *str_info;
	struct snd_pcm_runtime *pl_runtime;
	bool call_back = false;
	bool reset_index = false;

	substream = (struct snd_pcm_substream *)param;
	pl_runtime = substream->runtime;
	str_info = substream->runtime->private_data;

	WARN(!str_info, "SSP DAI: ERROR NULL str_info\n");
	if (!str_info)
		return -EINVAL;

	if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_RUNNING,
					&str_info->stream_status)) {
		bool dropped = test_and_clear_bit(INTEL_ALSA_SSP_STREAM_DROPPED,
					&str_info->stream_status);
		bool started = test_bit(INTEL_ALSA_SSP_STREAM_STARTED,
					&str_info->stream_status);

		if (started) {
			/*
			 * Whatever dropped or not,
			 * the stream is on going
			 */
			call_back = true;
		}
		if (started && dropped) {
			/*
			 * the stream has been dropped and restarted
			 * before the callback occurs
			 * in this case the we have to reprogram the
			 * requests to SSP driver
			 * and reset the stream's indexes
			 */
#ifndef USE_DMA_LLI
			reset_index = true;
#endif
		}

		if (!started && !dropped) {
			pr_err("SSP DAI: FCT %s neither started nor dropped",
						__func__);
			return -EBUSY;
		}

	} else {
		pr_err("SSP DAI: FCT %s called while not running ",
						__func__);
		return -EBUSY;
	}

	if (call_back == true) {
		pr_debug("SSP DAI: playback/capture (REQ=%d,CB=%d): "
				"DMA_REQ_COMPLETE\n",
				str_info->period_req_index,
				str_info->period_cb_index);
		if (reset_index) {
			str_info->period_cb_index = 0;
			str_info->period_req_index = 0;

		} else if (++(str_info->period_cb_index) >= pl_runtime->periods)
			str_info->period_cb_index = 0;

		/*
		 * Launch the next Capture/Playback request if
		 * no CLOSE has been requested
		 */
#ifndef USE_DMA_LLI
		ssp_dma_req(substream);
#else
		if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
		{
			intel_mid_i2s_set_busy(str_info->ssp_config->i2s_handle, I2S_STREAM_PLAYBACK);
		}
		else
		{
			intel_mid_i2s_set_busy(str_info->ssp_config->i2s_handle, I2S_STREAM_CAPTURE);
		}

		if (test_and_set_bit(INTEL_ALSA_SSP_STREAM_RUNNING,
				&str_info->stream_status))
		{
			pr_err("SSP DAI: "
				"ERROR previous requested not handled\n");
			return -EBUSY;
		}

		if (++(str_info->period_req_index) >= pl_runtime->periods)
			str_info->period_req_index = 0;
#endif

		/*
		* Call the snd_pcm_period_elapsed to inform ALSA kernel
		* that a ringbuffer period has been played
		*/
		snd_pcm_period_elapsed(substream);

	}
#ifdef USE_DMA_LLI
	else
	{
		if (!test_and_set_bit(INTEL_ALSA_SSP_STREAM_DROPPED, &str_info->stream_status))
		{
			if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
			{
				intel_mid_i2s_set_busy(str_info->ssp_config->i2s_handle, I2S_STREAM_PLAYBACK);
			}
			else
			{
				intel_mid_i2s_set_busy(str_info->ssp_config->i2s_handle, I2S_STREAM_CAPTURE);
			}

			if (test_and_set_bit(INTEL_ALSA_SSP_STREAM_RUNNING,
					&str_info->stream_status))
			{
				pr_err("SSP DAI: "
					"ERROR previous requested not handled\n");
				return -EBUSY;
			}

			if (++(str_info->period_cb_index) >= pl_runtime->periods)
				str_info->period_cb_index = 0;
			if (++(str_info->period_req_index) >= pl_runtime->periods)
				str_info->period_req_index = 0;
		}
	}
#endif
	return 0;
} /* ssp_dma_complete */

/**
 * intel_mid_ssp_transfer_data - send data buffers
 *
 * @param work Pointer to stream structure
 * return void
 */
void intel_mid_ssp_transfer_data(struct work_struct *work)
{
	struct intel_alsa_ssp_stream_info *str_info;
	struct snd_pcm_substream *substream;

	BUG_ON(!work);

	str_info = container_of(work, struct intel_alsa_ssp_stream_info,
			ssp_ws);

	BUG_ON(!str_info);

	substream = str_info->substream;

	BUG_ON(!substream);

	ssp_dma_req(substream);

} /* intel_mid_ssp_transfer_data */

/*
 * SSP PLATFORM
 */

/*
 * SSP Platform functions
 */
static int ssp_platform_pcm_new(struct snd_soc_pcm_runtime *soc_runtime)
{
	int retval = 0;
	struct snd_soc_dai *dai;
	struct snd_pcm *pcm;

	pr_debug("SSP DAI: FCT %s enters\n",
			__func__);
	/*
	 * Do pre-allocation to all substreams of the given pcm for the
	 * specified DMA type.
	 */
	dai = soc_runtime->cpu_dai;
	pcm = soc_runtime->pcm;

	if (dai->driver->playback.channels_min ||
			dai->driver->capture.channels_min) {

			retval =  snd_pcm_lib_preallocate_pages_for_all(pcm,
						SNDRV_DMA_TYPE_CONTINUOUS,
						snd_dma_continuous_data(GFP_KERNEL),
						SSP_MAX_BUFFER_TDM, SSP_MAX_BUFFER_TDM);

		if (retval) {
			pr_err("DMA buffer allocation fail\n");
			return retval;
		}
	}

	return retval;
} /* ssp_platform_pcm_new */

static void ssp_platform_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("SSP DAI: FCT %s enter\n",
			__func__);
	/*
	 * release all pre-allocated buffers on the pcm
	 *
	 */
	snd_pcm_lib_preallocate_free_for_all(pcm);
} /* ssp_platform_pcm_free */

/**
 * ssp_platform_hw_params - Allocate memory for Ring Buffer according
 * to hw_params.
 * It's called in a non-atomic context
 *
 * @param substream Substream for which the stream function is called
 * @param hw_params Stream command thats requested from upper layer
 * return status 0 ==> OK
 *
 */
static int ssp_platform_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{

	int ret_val;
	/*
	 * Allocates the DMA buffer for the substream
	 * This callback could be called several time
	 * snd_pcm_lib_malloc_pages allows to avoid memory leak
	 * as it release already allocated memory when already allocated
	 */
	ret_val = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));

	if (ret_val < 0)
		return ret_val;

	memset(substream->runtime->dma_area, 0, params_buffer_bytes(hw_params));

	return 0;
} /* ssp_platform_hw_params */

/*
 * ssp_platform_pointer- to send the current buffer pointer
 * processed by HW
 * This function is called by ALSA framework to get the current HW buffer ptr
 * to check the Ring Buffer Status
 *
 * @param substream Pointer to the substream for which the function
 *		is called
 *
 * return pcm_pointer Indicates the number of samples played
 *
 */
static
snd_pcm_uframes_t ssp_platform_pointer(struct snd_pcm_substream *substream)
{
	struct intel_alsa_ssp_stream_info *str_info;
	unsigned long pcm_pointer = 0;

	str_info = substream->runtime->private_data;

	WARN(!str_info, "SSP DAI: ERROR NULL str_info\n");
	if (!str_info)
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pcm_pointer = (unsigned long) (str_info->period_cb_index
				* substream->runtime->period_size);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pcm_pointer = (unsigned long) (str_info->period_cb_index
				* substream->runtime->period_size);

	pr_debug("SSP DAI: FCT %s Frame bits:: %d "
			"period_size :: %d periods :: %d\n",
			__func__,
			(int) substream->runtime->frame_bits,
			(int) substream->runtime->period_size,
			(int) substream->runtime->periods);


	pr_debug("SSP DAI: FCT %s returns %ld\n",
			__func__, pcm_pointer);

	return pcm_pointer;
} /* ssp_platform_pointer */

static struct snd_pcm_ops ssp_platform_ops = {
	.open = NULL,
	.close = NULL,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = ssp_platform_hw_params,
	.hw_free = NULL,
	.prepare = NULL,
	.trigger = NULL,
	.pointer = ssp_platform_pointer,
};

struct snd_soc_platform_driver soc_ssp_platform_drv = {
	.ops		= &ssp_platform_ops,
	.probe		= NULL,
	.pcm_new	= ssp_platform_pcm_new,
	.pcm_free	= ssp_platform_pcm_free,
};

/*
 * SND SOC DAI OPs
 */
static int ssp_probe(struct snd_soc_dai *cpu_dai)
{
	struct intel_ssp_config *ssp_config;
	struct intel_ssp_info *ssp_info;

	pr_info("SSP DAI: FCT %s enters for CPU_DAI %d\n",
			__func__, cpu_dai->id);

	ssp_config = kzalloc(sizeof(struct intel_ssp_config), GFP_KERNEL);

	if (ssp_config == NULL) {
		pr_err("Unable to allocate ssp_config\n");
		return -ENOMEM;
	}

	ssp_info = NULL; /* Initialize ssp_info */
	
	if(cpu_dai->id == SSP0_INSTANCE)
	  ssp_info = byt_ssp_info[SSP0_INSTANCE];
	else if(cpu_dai->id == SSP1_INSTANCE)
	  ssp_info = byt_ssp_info[SSP1_INSTANCE];
	else if(cpu_dai->id == SSP2_INSTANCE)
	  ssp_info = byt_ssp_info[SSP2_INSTANCE];

	if (!ssp_info)
	{
		kfree(ssp_config);
		return -EINVAL;
	}

	ssp_config->intel_mid_dma_alloc = false;
	ssp_config->i2s_settings = ssp_info->pdata->i2s_settings;

	pr_debug("SSP DAI: FCT %s ssp_config %p\n",
				__func__, ssp_config);

	cpu_dai->playback_dma_data = cpu_dai->capture_dma_data = ssp_config;

	return 0;
} /* ssp_probe */

static int ssp_remove(struct snd_soc_dai *cpu_dai)
{
	struct intel_ssp_config *ssp_config;

	WARN(!cpu_dai, "SSP DAI: "
			"ERROR NULL cpu_dai\n");
	if (!cpu_dai)
		return -EINVAL;

	ssp_config = cpu_dai->playback_dma_data;

	kfree(ssp_config);

	return 0;
} /* ssp_remove */

static int ssp_dai_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct intel_ssp_config *ssp_config;
	struct snd_pcm_runtime *pl_runtime;
	struct intel_alsa_ssp_stream_info *str_info;
	struct snd_soc_dai_driver *cpudai_drv = cpu_dai->driver;
	int ret = 0;

	WARN(!cpu_dai->driver, "SSP DAI: "
			"FCT %s ERROR NULL cpu_dai->driver\n",
			__func__);
	if (!cpu_dai->driver)
		return -EINVAL;

	pr_debug("SSP DAI: FCT %s enters for DAI Id = %d\n",
			__func__, cpu_dai->driver->id);

	pl_runtime = substream->runtime;

	ssp_config = snd_soc_dai_get_dma_data(cpu_dai, substream);

	WARN(!ssp_config, "SSP DAI: "
			"FCT %s ERROR NULL ssp_config\n",
			__func__);
	if (!ssp_config)
		return -EINVAL;

	/*
	 * setup the internal data structure stream pointers based on it being
	 * playback or capture stream
	 */
	str_info = kzalloc(sizeof(*str_info), GFP_KERNEL);

	if (!str_info) {
		pr_err("SSP DAI: str_info alloc failure\n");
		return -EINVAL;

	}
	str_info->substream = substream;
	str_info->ssp_config = ssp_config;
	str_info->stream_status = 0;

	INIT_WORK(&str_info->ssp_ws, intel_mid_ssp_transfer_data);

	/*
	 * Initialize SSPx [x=0,1] driver
	 * Store the Stream information
	 */
	pl_runtime->private_data = str_info;

	pr_debug("SSP DAI: FCT %s enters cpu_dai->card->name = %s\n",
			__func__, cpu_dai->card->name);

	if (!cpu_dai->active) {
		if (!strcmp(cpudai_drv->name, SSP0_CARD_AK4614_DAI_NAME)) {
				ssp_config->i2s_handle =
					intel_mid_i2s_open(SSP_USAGE_CARD_AK4614_0);
				pr_debug("opening the CPU_DAI for "\
						"SSP_USAGE_CARD_AK4614, i2s_handle = %p\n",
						ssp_config->i2s_handle);
		}
		else if (!strcmp(cpudai_drv->name, SSP1_CARD_AK4614_DAI_NAME)) {
				ssp_config->i2s_handle =
					intel_mid_i2s_open(SSP_USAGE_CARD_AK4614_1);
				pr_debug("opening the CPU_DAI for"\
						"SSP_USAGE_CARD_AK4614, i2s_handle = %p\n",
						ssp_config->i2s_handle);
		}
		else if (!strcmp(cpudai_drv->name, SSP2_CARD_AK4614_DAI_NAME)) {
				ssp_config->i2s_handle =
					intel_mid_i2s_open(SSP_USAGE_CARD_AK4614_2);
				pr_debug("opening the CPU_DAI for"\
						"SSP_USAGE_CARD_AK4614, i2s_handle = %p\n",
						ssp_config->i2s_handle);
		}
		else {
			pr_err("non Valid SOC CARD\n");
			return -EINVAL;
		}

		/* Set the Write Callback */
		ret = intel_mid_i2s_set_wr_cb(ssp_config->i2s_handle,
				ssp_dma_complete);
		if (ret) {
			pr_err("%s: unable to set write callback\n" , __func__);
			return ret;
		}
		/* Set the Default Read Callback */
		ret = intel_mid_i2s_set_rd_cb(ssp_config->i2s_handle,
				ssp_dma_complete);
		if (ret) {
			pr_err("%s: unable to set read callback\n" , __func__);
			return ret;
		}


	} else {
		/*
		 * do nothing because already Open by sibling substream
		 */
		pr_debug("SSP DAI: FCT %s Open DO NOTHING\n",
				__func__);
	}

	return 0;
} /* ssp_dai_startup */

static void ssp_dai_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	struct intel_ssp_config *ssp_config;
	struct intel_alsa_ssp_stream_info *str_info;
	struct snd_pcm_runtime *runtime;
	struct intel_ssp_info *ssp_info;

	ssp_config = snd_soc_dai_get_dma_data(cpu_dai, substream);

	BUG_ON(!ssp_config);

	runtime = substream->runtime;
	BUG_ON(!runtime->private_data);

	str_info = runtime->private_data;
	BUG_ON(!str_info);
	
	ssp_info = NULL; /* Initialize ssp_info */

	if(cpu_dai->id == SSP0_INSTANCE)
	  ssp_info = byt_ssp_info[SSP0_INSTANCE];
	else if(cpu_dai->id == SSP1_INSTANCE)
	  ssp_info = byt_ssp_info[SSP1_INSTANCE];
	else if(cpu_dai->id == SSP2_INSTANCE)
	  ssp_info = byt_ssp_info[SSP2_INSTANCE];
	
	BUG_ON(!ssp_info);

	/* Cancel pending work */
	cancel_work_sync(&str_info->ssp_ws);

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		/*
		 * Only Free Tx channel if no playback streams are active
		 * Shutdown can be called right after a startup if something
		 * failed (as a concurrency issue
		 * so this case can happen
		 */
		if ((!cpu_dai->playback_active) &&
			(ssp_config->i2s_settings.ssp_active_tx_slots_map)) {
			intel_mid_i2s_command(ssp_config->i2s_handle,
					SSP_CMD_FREE_TX, NULL, 0);
			pr_debug("SSP DAI: FCT %s TX DMA Channel released\n",
					__func__);
		}
		ssp_info->ssp_dai_tx_allocated = false;
		break;

	case SNDRV_PCM_STREAM_CAPTURE:
		/*
		 * Only Free Rx channel if no capture streams are active
		 */
		if ((!cpu_dai->capture_active) &&
			(ssp_config->i2s_settings.ssp_active_rx_slots_map)) {
			intel_mid_i2s_command(ssp_config->i2s_handle,
					SSP_CMD_FREE_RX, NULL, 0);
			pr_debug("SSP DAI: FCT %s RX DMA Channel released\n",
					__func__);
		}
		ssp_info->ssp_dai_rx_allocated = false;
		break;

	default:
		pr_err("SSP DAI: FCT %s Bad stream_dir: %d\n",
				__func__, substream->stream);
		break;
	}

	kfree(str_info);

	if (!cpu_dai->active) {
		pr_info("SSP DAI: FCT %s closing I2S\n",
				__func__);
		/*
		 * Close the Intel MID I2S connection
		 */
		intel_mid_i2s_close(ssp_config->i2s_handle);

		ssp_config->i2s_handle = NULL;
		ssp_config->intel_mid_dma_alloc = false;
	}

} /* ssp_dai_shutdown */

static int ssp_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct intel_ssp_config *ssp_config;
	struct intel_mid_i2s_settings *i2s_config;

	ssp_config = cpu_dai->playback_dma_data;

	WARN(!ssp_config, "SSP DAI: FCT %s ssp_config=NULL\n",
			__func__);
	if (!ssp_config)
		return -EINVAL;

	pr_debug("SSP DAI: FCT %s fmt = %d\n",
			__func__, fmt);

	i2s_config = &(ssp_config->i2s_settings);

	/*
	 * SSP CLK Direction
	 * SSP FRMSYNC Direction
	 * Note: SSP direction should be inverse of CBS or CFS
	 * 		e.g. CBS = codec bit CLK is slave, so SSP = Master
	 */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s_config->sspslclk_direction = SSPSCLK_MASTER_MODE;
		i2s_config->sspsfrm_direction = SSPSCLK_MASTER_MODE;
		/*
		 * Mandatory to be able to perform only RX without TX
		 * in SSP CLK Master Mode
		 *
		 */
		i2s_config->ssp_duplex_mode = RX_AND_TX_MODE;
		/* RX_AND_TX_MODE should not be set with SSP_IN_NETWORK_MODE enabled */
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		i2s_config->sspslclk_direction = SSPSCLK_MASTER_MODE;
		i2s_config->sspsfrm_direction = SSPSCLK_SLAVE_MODE;
		/*
		 * Mandatory to be able to perform only RX without TX
		 * in SSP CLK Master Mode
		 *
		 */
		i2s_config->ssp_duplex_mode = RX_AND_TX_MODE;
		/* RX_AND_TX_MODE should not be set with SSP_IN_NETWORK_MODE enabled */
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		i2s_config->sspslclk_direction = SSPSCLK_SLAVE_MODE;
		i2s_config->sspsfrm_direction = SSPSCLK_SLAVE_MODE;
		i2s_config->ssp_duplex_mode = RX_AND_TX_MODE;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		i2s_config->sspslclk_direction = SSPSCLK_SLAVE_MODE;
		i2s_config->sspsfrm_direction = SSPSCLK_MASTER_MODE;
		i2s_config->ssp_duplex_mode = RX_AND_TX_MODE;
		break;
	default:
		pr_err("SSP DAI: %s Bad DAI CLK/FS Mode=%d\n",
					__func__,
					(fmt & SND_SOC_DAIFMT_MASTER_MASK));
		return -EINVAL;
	}
	/*
	 * SSP Signal Inversion Mode
	 * Use clock gating bitfield for
	 * Serial bit-rate Clock Mode
	 */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SSP_DAI_SCMODE_0:
		i2s_config->ssp_serial_clk_mode = SSP_CLK_MODE_0;
		break;
	case SSP_DAI_SCMODE_1:
		i2s_config->ssp_serial_clk_mode = SSP_CLK_MODE_1;
		break;
	case SSP_DAI_SCMODE_2:
		i2s_config->ssp_serial_clk_mode = SSP_CLK_MODE_2;
		break;
	case SSP_DAI_SCMODE_3:
		i2s_config->ssp_serial_clk_mode = SSP_CLK_MODE_3;
		break;
	default:
		pr_err("SSP DAI: %s Bad DAI Signal Inversion Mode=%d\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_INV_MASK));
		return -EINVAL;
	}

	/*
	 * SSP FS Inversion Mode
	 */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
	case SND_SOC_DAIFMT_IB_NF:
		i2s_config->ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH;
		break;
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
		i2s_config->ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_LOW;
		break;
	default:
		pr_err("SSP DAI: %s Bad DAI FS Inversion Mode=%d\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_INV_MASK));
		return -EINVAL;
	}

	/*
	 * SSP Format Mode
	 */

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_TDM256_RIGHT_J:
	case SND_SOC_DAIFMT_TDM256_LEFT_J:
	case SND_SOC_DAIFMT_I2S:
		i2s_config->frame_format = PSP_FORMAT;
		break;

	default:
		pr_err("SSP DAI: %s Bad DAI format Mode=%d\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_FORMAT_MASK));
		return -EINVAL;
	}

	return 0;
} /* ssp_set_dai_fmt */

static int ssp_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct intel_ssp_config *ssp_config;
	struct intel_mid_i2s_settings *i2s_config;

	ssp_config = cpu_dai->playback_dma_data;

	WARN(!ssp_config, "SSP DAI: FCT %s ssp_config=NULL\n",
			__func__);
	if (!ssp_config)
		return -EINVAL;


	i2s_config = &(ssp_config->i2s_settings);

	i2s_config->frame_rate_divider_control = slots;
	i2s_config->data_size = slot_width;
	i2s_config->ssp_active_tx_slots_map = tx_mask;
	i2s_config->ssp_active_rx_slots_map = rx_mask;

	if (SSP_STEREO_CHANNEL == slots) /*Stereo*/
		i2s_config->mode = SSP_IN_NORMAL_MODE;
	else
		i2s_config->mode = SSP_IN_NETWORK_MODE;

	pr_debug("i2s_config->frame_rate_divider_control = %d\n",
			i2s_config->frame_rate_divider_control);
	pr_debug("i2s_config->data_size = %d\n",
			i2s_config->data_size);
	pr_debug("i2s_config->mode = %d\n",
			i2s_config->mode);
	pr_debug("i2s_config->ssp_active_tx_slots_map = %d\n",
			i2s_config->ssp_active_tx_slots_map);
	pr_debug("i2s_config->ssp_active_rx_slots_map = %d\n",
			i2s_config->ssp_active_rx_slots_map);

	return 0;
}

static int ssp_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct intel_ssp_config *ssp_config;
	struct intel_mid_i2s_settings *i2s_config;

	ssp_config = cpu_dai->playback_dma_data;

	BUG_ON(!ssp_config);

	i2s_config = &(ssp_config->i2s_settings);

	pr_debug("SSP DAI: FCT %s clk_id = %d\n",
			__func__, clk_id);

	switch (clk_id) {
	case SSP_CLK_ONCHIP:
		i2s_config->master_mode_clk_selection = SSP_ONCHIP_CLOCK;
		break;
	case SSP_CLK_NET:
		i2s_config->master_mode_clk_selection = SSP_NETWORK_CLOCK;
		break;
	case SSP_CLK_EXT:
		i2s_config->master_mode_clk_selection = SSP_EXTERNAL_CLOCK;
		break;
	case SSP_CLK_AUDIO:
		i2s_config->master_mode_clk_selection = SSP_ONCHIP_AUDIO_CLOCK;
		break;
	default:
		i2s_config->master_mode_standard_freq =
						SSP_MASTER_CLOCK_UNDEFINED;
		pr_err("SSP DAI: %s Bad clk_id=%d\n",
				__func__,
				clk_id);
		return -EINVAL;
	}

	pr_debug("SSP DAI:FCT %s freq = %d\n",
				__func__, freq);

	switch (freq) {
	case 48000:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_48_000;
		/* Users to determine serial protocol parameters */
		break;
	case 44100:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_44_100;
		break;
	case 22050:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_22_050;
		break;
	case 16000:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_16_000;
		break;
	case 11025:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_11_025;
		break;
	case 8000:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_8_000;
		break;
	default:
		i2s_config->master_mode_standard_freq = SSP_FRM_FREQ_UNDEFINED;
		pr_err("SSP DAI: %s Bad freq_out=%d\n",
				__func__,
				freq);
		return -EINVAL;
	}

	return 0;
}

static int ssp_set_dai_tristate(struct snd_soc_dai *cpu_dai,
	int tristate)
{
	struct intel_ssp_config *ssp_config;
	struct intel_mid_i2s_settings *i2s_config;

	ssp_config = cpu_dai->playback_dma_data;

	BUG_ON(!ssp_config);

	i2s_config = &(ssp_config->i2s_settings);

	if (IS_TRISTATE_ENABLED(tristate))
		i2s_config->tx_tristate_enable = TXD_TRISTATE_ON;
	else
		i2s_config->tx_tristate_enable = TXD_TRISTATE_OFF;

	if (IS_NEXT_FRMS_ASSERTED_WITH_LSB_PREVIOUS_FRM(tristate))
		i2s_config->ssp_frmsync_timing_bit =
				NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM;
	else
		i2s_config->ssp_frmsync_timing_bit =
				NEXT_FRMS_ASS_AFTER_END_OF_T4;

	pr_debug("FCT %s tristate %x\n", __func__, tristate);

	return 0;
}

/**
 * ssp_dai_trigger- stream activities are handled here
 * This function is called whenever a stream activity is invoked
 * The Trigger function is called in an atomic context
 *
 * @param substream Substream for which the stream function is called
 * @param cmd The stream command thats requested from upper layer
 * return status 0 ==> OK
 *
 */
static int ssp_dai_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *cpu_dai)
{
	struct intel_ssp_config *ssp_config;
	int ret_val = 0;
	struct intel_alsa_ssp_stream_info *str_info;
	struct snd_pcm_runtime *pl_runtime;
	struct intel_ssp_info *ssp_info;
	bool trigger_start = true;
	int stream = 0;

	pr_debug("SSP DAI: FCT %s enters\n",
			__func__);

	stream = substream->stream;


	pl_runtime = substream->runtime;
	ssp_config = snd_soc_dai_get_dma_data(cpu_dai, substream);
	WARN(!pl_runtime->private_data, "SSP DAI: ERROR "
			"NULL pl_runtime->private_data\n");
	if (!pl_runtime->private_data)
		return -EINVAL;

	WARN(!cpu_dai, "SSP DAI: ERROR NULL cpu_dai\n");
	if (!cpu_dai)
		return -EINVAL;

	WARN(!cpu_dai->dev, "SSP DAI: ERROR NULL cpu_dai->dev\n");
	if (!cpu_dai->dev)
		return -EINVAL;

	ssp_info = dev_get_drvdata(cpu_dai->dev);
	if(cpu_dai->id == SSP0_INSTANCE)
	  ssp_info = byt_ssp_info[SSP0_INSTANCE];
	else if(cpu_dai->id == SSP1_INSTANCE)
	  ssp_info = byt_ssp_info[SSP1_INSTANCE];
	else if(cpu_dai->id == SSP2_INSTANCE)
	  ssp_info = byt_ssp_info[SSP2_INSTANCE];

	WARN(!ssp_info->ssp_dai_wq, "SSP DAI: ERROR NULL ssp_dai_wq\n");
	if (!ssp_info->ssp_dai_wq)
		return -EINVAL;

	str_info = pl_runtime->private_data;

	pr_debug("SSP DAI: FCT %s CMD = 0x%04X\n",
			__func__, cmd);


	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			intel_mid_i2s_command(ssp_config->i2s_handle,
										SSP_CMD_RESUME_TX,
										&(ssp_config->i2s_settings),
										0);
		else if (stream == SNDRV_PCM_STREAM_CAPTURE)
			intel_mid_i2s_command(ssp_config->i2s_handle,
										SSP_CMD_RESUME_RX,
										&(ssp_config->i2s_settings),
										0);
		break;
	case SNDRV_PCM_TRIGGER_START:
		if (!test_and_set_bit(INTEL_ALSA_SSP_STREAM_STARTED,
				&str_info->stream_status)) {
			if (test_bit(INTEL_ALSA_SSP_STREAM_DROPPED,
					&str_info->stream_status)) {
				pr_debug("SSP DAI: FCT %s do not restart "
						"the trigger stream running "
						"already\n", __func__);
				trigger_start = false;
			} else
				trigger_start = true;
		} else {
			pr_err("SSP DAI: ERROR 2 consecutive TRIGGER_START\n");
			return -EBUSY;
		}

		/* Store the substream locally */
		if (trigger_start) {
			if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
				pr_debug("SSP DAI: queue Playback Work\n");
				queue_work(ssp_info->ssp_dai_wq,
						&str_info->ssp_ws);
			} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {

				pr_debug("SSP DAI: queue Capture Work\n");
				queue_work(ssp_info->ssp_dai_wq,
						&str_info->ssp_ws);
			} else {
				pr_err("SSP DAI: SNDRV_PCM_TRIGGER_START Bad Stream: %d\n",
						substream->stream);
				return -EINVAL;
			}
		}
		break;


	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
				intel_mid_i2s_command(ssp_config->i2s_handle,
										SSP_CMD_PAUSE_TX,
										&(ssp_config->i2s_settings),
										0);
		else if (stream == SNDRV_PCM_STREAM_CAPTURE)
				intel_mid_i2s_command(ssp_config->i2s_handle,
										SSP_CMD_PAUSE_RX,
										&(ssp_config->i2s_settings),
										0);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (test_and_clear_bit(INTEL_ALSA_SSP_STREAM_STARTED,
				&str_info->stream_status))
			set_bit(INTEL_ALSA_SSP_STREAM_DROPPED,
					&str_info->stream_status);
		else {
			pr_err("SSP DAI: trigger START/STOP mismatch\n");
			return -EBUSY;
		}
		break;

	default:
		pr_err("SSP DAI: snd_i2s_alsa_pcm_trigger Bad Command\n");
		return -EINVAL;
	}

	return ret_val;
} /* ssp_dai_trigger */

/**
 * ssp_dai_hw_params - Allocate memory for Ring Buffer according
 * to hw_params.
 * It's called in a non-atomic context
 *
 * @param substream Substream for which the stream function is called
 * @param hw_params Stream command thats requested from upper layer
 * @param cpu_dai Pointer to the CPU DAI that is used
 * return status 0 ==> OK
 *
 */
static int ssp_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params,
		struct snd_soc_dai *cpu_dai)
{

	return 0;
}

static int ssp_dai_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct intel_ssp_config *ssp_config;
	struct intel_ssp_info *ssp_info;

	pr_debug("SSP DAI: FCT %s enters\n",
			__func__);

	WARN(!cpu_dai, "SSP DAI: ERROR NULL cpu_dai\n");
	if (!cpu_dai)
		return -EINVAL;


	ssp_info = dev_get_drvdata(cpu_dai->dev);
	if(cpu_dai->id == SSP0_INSTANCE)
	  ssp_info = byt_ssp_info[SSP0_INSTANCE];
	else if(cpu_dai->id == SSP1_INSTANCE)
	  ssp_info = byt_ssp_info[SSP1_INSTANCE];
	else if(cpu_dai->id == SSP2_INSTANCE)
	  ssp_info = byt_ssp_info[SSP2_INSTANCE];

	WARN(!ssp_info, "SSP DAI: ERROR NULL ssp_info\n");
	if (!ssp_info)
		return -EINVAL;

	ssp_config = snd_soc_dai_get_dma_data(cpu_dai, substream);
	pr_debug("SSP DAI: FCT %s ssp_dai_tx_allocated %d"\
			"ssp_dai_rx_allocated %d\n",
			__func__,
			ssp_info->ssp_dai_tx_allocated,
			ssp_info->ssp_dai_rx_allocated);

	/*
	 * The set HW Config is only once for a CPU DAI
	 */

	if (!ssp_info->ssp_dai_tx_allocated && !ssp_info->ssp_dai_rx_allocated) {

		intel_mid_i2s_command(ssp_config->i2s_handle,
						SSP_CMD_SET_HW_CONFIG,
						&(ssp_config->i2s_settings), 0);
	}

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		if (!ssp_info->ssp_dai_tx_allocated) {
			if (intel_mid_i2s_command(ssp_config->i2s_handle,
					SSP_CMD_ALLOC_TX, NULL, 0)) {
				pr_err("can not alloc TX DMA Channel\n");
				return -EBUSY;
			}
			ssp_info->ssp_dai_tx_allocated = true;
		}
		break;

	case SNDRV_PCM_STREAM_CAPTURE:
		if (!ssp_info->ssp_dai_rx_allocated) {
			if (intel_mid_i2s_command(ssp_config->i2s_handle,
					SSP_CMD_ALLOC_RX, NULL, 0)) {
				pr_err("can not alloc RX DMA Channel\n");
				return -EBUSY;
			}
			ssp_info->ssp_dai_rx_allocated = true;
		}
		break;

	default:
		pr_err("SSP DAI: FCT %s Bad stream_dir: %d\n",
				__func__, substream->stream);
		return -EINVAL;
	}

	ssp_config->intel_mid_dma_alloc = true;

	pr_debug("SSP DAI: FCT %s leaves\n",
			__func__);

	return 0;
}

static struct snd_soc_dai_ops ssp_dai_ops = {
	.startup	= ssp_dai_startup,
	.shutdown	= ssp_dai_shutdown,
	.trigger	= ssp_dai_trigger,
	.hw_params	= ssp_dai_hw_params,
	.prepare    = ssp_dai_prepare,
	.set_sysclk	= ssp_set_dai_sysclk,
	.set_pll    = NULL,
	.set_fmt	= ssp_set_dai_fmt,
	.set_tdm_slot	= ssp_set_dai_tdm_slot,
	.set_tristate	= ssp_set_dai_tristate,
};

#define SSP_SUPPORTED_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define SSP_SUPPORTED_FORMATS ( SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8)

struct snd_soc_dai_driver intel_ssp_platform_dai[] = {
	{
		.name = SSP0_CARD_AK4614_DAI_NAME,
		.playback = {
			.channels_min = SSP_MONO_CHANNEL,
			.channels_max = SSP_STEREO_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},

		.capture = {
			.channels_min = SSP_MONO_CHANNEL,
			.channels_max = SSP_STEREO_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &ssp_dai_ops,
		.probe = ssp_probe,
		.remove = ssp_remove,
	},


	{
		.name = SSP1_CARD_AK4614_DAI_NAME,
		.playback = {
			.channels_min = SSP_MONO_CHANNEL,
			.channels_max = SSP_STEREO_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},
		.capture = {
			.channels_min = SSP_MONO_CHANNEL,
			.channels_max = SSP_STEREO_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &ssp_dai_ops,
		.probe = ssp_probe,
		.remove = ssp_remove,
	},
	{
		.name = SSP2_CARD_AK4614_DAI_NAME,
		.playback = {
			.channels_min = SSP_MONO_CHANNEL,
			.channels_max = SSP_STEREO_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},
		.capture = {
			.channels_min = SSP_MONO_CHANNEL,
			.channels_max = SSP_STEREO_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &ssp_dai_ops,
		.probe = ssp_probe,
		.remove = ssp_remove,
	},

};

static struct snd_soc_dai_driver intel_ssp_platform_dai_tdm8[] = {
	{
		.name = SSP0_CARD_AK4614_DAI_NAME,
		.playback = {
			.channels_min = SSP_STEREO_CHANNEL,
			.channels_max = SSP_TDM8_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE),
		},
		.capture = {
			.channels_min = SSP_STEREO_CHANNEL,
			.channels_max = SSP_TDM8_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE),
		},
		.ops = &ssp_dai_ops,
		.probe = ssp_probe,
		.remove = ssp_remove,
	},
	{
		.name = SSP1_CARD_AK4614_DAI_NAME,
		.playback = {
			.channels_min = SSP_STEREO_CHANNEL,
			.channels_max = SSP_TDM8_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE),
		},
		.capture = {
			.channels_min = SSP_STEREO_CHANNEL,
			.channels_max = SSP_TDM8_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE),
		},
		.ops = &ssp_dai_ops,
		.probe = ssp_probe,
		.remove = ssp_remove,
	},
	{
		.name = SSP2_CARD_AK4614_DAI_NAME,
		.playback = {
			.channels_min = SSP_STEREO_CHANNEL,
			.channels_max = SSP_TDM8_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE),
		},
		.capture = {
			.channels_min = SSP_STEREO_CHANNEL,
			.channels_max = SSP_TDM8_CHANNEL,
			.rates = SNDRV_PCM_RATE_48000,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE),
		},
		.ops = &ssp_dai_ops,
		.probe = ssp_probe,
		.remove = ssp_remove,
	},

};

static int ssp_dai_probe(struct platform_device *pdev)
{
	int ret;
	int index = pdev->id;
	struct ssp_platform_config *pdata = pdev->dev.platform_data;

	pr_info("SSP DAI: FCT %s enters\n",
			__func__);
	
	byt_ssp_info[index] = kzalloc(sizeof(struct intel_ssp_info), GFP_KERNEL);

	if (NULL == byt_ssp_info[index])
	{
		pr_err("Unable to allocate ssp_info\n");
		return -ENOMEM;
	}

	pr_debug("ssp_info address %p", byt_ssp_info[index]);
	
	ret = snd_soc_register_platform(&pdev->dev,
				&soc_ssp_platform_drv);
	if (ret) {
		pr_err("registering SSP PLATFORM failed\n");
		snd_soc_unregister_component(&pdev->dev);
		kfree(byt_ssp_info[index]);
		return -EBUSY;
	}
	byt_ssp_info[index]->pdata = pdata;
	
	ret = snd_soc_register_component(&pdev->dev, &ssp_component,
				intel_ssp_platform_dai_tdm8,
				ARRAY_SIZE(intel_ssp_platform_dai_tdm8));

	if (ret) {
		pr_err("registering cpu DAIs failed\n");
		snd_soc_unregister_component(&pdev->dev);
		kfree(byt_ssp_info[index]);
		return -EBUSY;
	}

	byt_ssp_info[index]->ssp_dai_wq = create_workqueue("ssp_transfer_data");
	byt_ssp_info[index]->ssp_dai_tx_allocated = false;
	byt_ssp_info[index]->ssp_dai_rx_allocated = false;
	
	if (!byt_ssp_info[index]->ssp_dai_wq)
	{
		pr_err("work queue failed\n");
		snd_soc_unregister_component(&pdev->dev);
		kfree(byt_ssp_info[index]);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, byt_ssp_info[index]);//workaround for set_driver data

	pr_debug("SSP DAI: FCT %s leaves %d\n",
			__func__, ret);

	return ret;
}

static int ssp_dai_remove(struct platform_device *pdev)
{
	int i;
	struct intel_ssp_info *ssp_info = platform_get_drvdata(pdev);

	pr_debug("SSP DAI: FCT %s enters\n",
			__func__);

	if (ssp_info == NULL) {
		pr_err("Unable to allocate ssp_info\n");
		return -ENOMEM;
	}
	pr_debug("ssp_info address %p", ssp_info);
	for(i=0; i<NUMBER_OF_SSP_PORT; i++)
	{
	    flush_workqueue(byt_ssp_info[i]->ssp_dai_wq);
	    destroy_workqueue(byt_ssp_info[i]->ssp_dai_wq);
	}
	platform_set_drvdata(pdev, NULL);

	snd_soc_unregister_component(&pdev->dev);

	snd_soc_unregister_platform(&pdev->dev);

	pr_debug("SSP DAI: FCT %s leaves\n",
			__func__);

	return 0;
}

static struct platform_driver intel_ssp_dai_driver = {
	.driver		= {
		.name		= "mid-ssp-dai",
		.owner		= THIS_MODULE,
	},
	.probe		= ssp_dai_probe,
	.remove		= ssp_dai_remove,
};


static int __init ssp_soc_dai_init(void)
{
	pr_info("SSP DAI: FCT %s called\n",
			__func__);

	return  platform_driver_register(&intel_ssp_dai_driver);
}
module_init(ssp_soc_dai_init);

static void __exit ssp_soc_dai_exit(void)
{
	pr_debug("SSP DAI: FCT %s called\n",
			__func__);

	platform_driver_unregister(&intel_ssp_dai_driver);

}
module_exit(ssp_soc_dai_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Platform driver");
MODULE_AUTHOR("Selma Bensaid");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mid-ssp-dai");
