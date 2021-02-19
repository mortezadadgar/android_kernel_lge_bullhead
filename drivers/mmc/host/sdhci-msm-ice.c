/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "sdhci-msm-ice.h"

static void sdhci_msm_ice_success_cb(void *host_ctrl,
				enum ice_event_completion evt)
{
	struct sdhci_msm_host *msm_host = (struct sdhci_msm_host *)host_ctrl;

	if ((msm_host->ice.state == SDHCI_MSM_ICE_STATE_DISABLED &&
	    evt == ICE_INIT_COMPLETION) || (msm_host->ice.state ==
	    SDHCI_MSM_ICE_STATE_SUSPENDED && evt == ICE_RESUME_COMPLETION))
		msm_host->ice.state = SDHCI_MSM_ICE_STATE_ACTIVE;

	complete(&msm_host->ice.async_done);
}

static void sdhci_msm_ice_error_cb(void *host_ctrl, enum ice_error_code evt)
{
	struct sdhci_msm_host *msm_host = (struct sdhci_msm_host *)host_ctrl;

	dev_err(&msm_host->pdev->dev, "%s: Error in ice operation %d",
		__func__, evt);

	if (msm_host->ice.state == SDHCI_MSM_ICE_STATE_ACTIVE)
		msm_host->ice.state = SDHCI_MSM_ICE_STATE_DISABLED;

	complete(&msm_host->ice.async_done);
}

static struct platform_device *sdhci_msm_ice_get_pdevice(struct device *dev)
{
	struct device_node *node;
	struct platform_device *ice_pdev = NULL;

	node = of_parse_phandle(dev->of_node, SDHC_MSM_CRYPTO_LABEL, 0);
	if (!node) {
		dev_dbg(dev, "%s: sdhc-msm-crypto property not specified\n",
			__func__);
		goto out;
	}
	ice_pdev = qcom_ice_get_pdevice(node);
out:
	return ice_pdev;
}

static
struct qcom_ice_variant_ops *sdhci_msm_ice_get_vops(struct device *dev)
{
	struct qcom_ice_variant_ops *ice_vops = NULL;
	struct device_node *node;

	node = of_parse_phandle(dev->of_node, SDHC_MSM_CRYPTO_LABEL, 0);
	if (!node) {
		dev_dbg(dev, "%s: sdhc-msm-crypto property not specified\n",
			__func__);
		goto out;
	}
	ice_vops = qcom_ice_get_variant_ops(node);
	of_node_put(node);
out:
	return ice_vops;
}

int sdhci_msm_ice_get_dev(struct sdhci_host *host)
{
	struct device *sdhc_dev;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	if (!msm_host || !msm_host->pdev) {
		pr_err("%s: invalid msm_host %p or msm_host->pdev\n",
			__func__, msm_host);
		return -EINVAL;
	}

	sdhc_dev = &msm_host->pdev->dev;
	msm_host->ice.vops  = sdhci_msm_ice_get_vops(sdhc_dev);
	msm_host->ice.pdev = sdhci_msm_ice_get_pdevice(sdhc_dev);

	if (msm_host->ice.pdev == ERR_PTR(-EPROBE_DEFER)) {
		dev_err(sdhc_dev, "%s: ICE device not probed yet\n",
			__func__);
		msm_host->ice.pdev = NULL;
		msm_host->ice.vops = NULL;
		return -EPROBE_DEFER;
	}

	if (!msm_host->ice.pdev) {
		dev_dbg(sdhc_dev, "%s: invalid platform device\n", __func__);
		msm_host->ice.vops = NULL;
		return -ENODEV;
	}
	if (!msm_host->ice.vops) {
		dev_dbg(sdhc_dev, "%s: invalid ice vops\n", __func__);
		msm_host->ice.pdev = NULL;
		return -ENODEV;
	}
	msm_host->ice.state = SDHCI_MSM_ICE_STATE_DISABLED;
	return 0;
}

int sdhci_msm_ice_init(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int err = 0;

	init_completion(&msm_host->ice.async_done);
	if (msm_host->ice.vops->config) {
		err = msm_host->ice.vops->init(msm_host->ice.pdev,
					msm_host,
					sdhci_msm_ice_success_cb,
					sdhci_msm_ice_error_cb);
		if (err) {
			pr_err("%s: ice init err %d\n",
				mmc_hostname(host->mmc), err);
			return err;
		}
	}

	if (!wait_for_completion_timeout(&msm_host->ice.async_done,
		msecs_to_jiffies(SDHCI_MSM_ICE_COMPLETION_TIMEOUT_MS))) {
		pr_err("%s: ice init timedout after %d ms\n",
				mmc_hostname(host->mmc),
				SDHCI_MSM_ICE_COMPLETION_TIMEOUT_MS);
		return -ETIMEDOUT;
	}

	if (msm_host->ice.state != SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}
	return 0;
}

int sdhci_msm_ice_cfg(struct sdhci_host *host, struct mmc_request *mrq,
			u32 slot)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int err = 0;
	struct ice_data_setting ice_set;
	sector_t lba = 0;
	unsigned int ctrl_info_val = 0;
	unsigned int bypass = SDHCI_MSM_ICE_ENABLE_BYPASS;
	struct request *req;

	if (msm_host->ice.state != SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}

	BUG_ON(!mrq);
	memset(&ice_set, 0, sizeof(struct ice_data_setting));
	req = mrq->req;
	if (req) {
		lba = req->__sector;
		if (msm_host->ice.vops->config) {
			err = msm_host->ice.vops->config(msm_host->ice.pdev,
							req, &ice_set);
			if (err) {
				pr_err("%s: ice config failed %d\n",
						mmc_hostname(host->mmc), err);
				return err;
			}
		}
		/* if writing data command */
		if (rq_data_dir(req) == WRITE)
			bypass = ice_set.encr_bypass ?
					SDHCI_MSM_ICE_ENABLE_BYPASS :
					SDHCI_MSM_ICE_DISABLE_BYPASS;
		/* if reading data command */
		else if (rq_data_dir(req) == READ)
			bypass = ice_set.decr_bypass ?
					SDHCI_MSM_ICE_ENABLE_BYPASS :
					SDHCI_MSM_ICE_DISABLE_BYPASS;
		pr_debug("%s: %s: slot %d encr_bypass %d bypass %d decr_bypass %d key_index %d\n",
				mmc_hostname(host->mmc),
				(rq_data_dir(req) == WRITE) ? "WRITE" : "READ",
				slot, ice_set.encr_bypass, bypass,
				ice_set.decr_bypass,
				ice_set.crypto_data.key_index);
	}

	/* Configure ICE index */
	ctrl_info_val =
		(ice_set.crypto_data.key_index &
		 MASK_SDHCI_MSM_ICE_CTRL_INFO_KEY_INDEX)
		 << OFFSET_SDHCI_MSM_ICE_CTRL_INFO_KEY_INDEX;

	/* Configure data unit size of transfer request */
	ctrl_info_val |=
		(SDHCI_MSM_ICE_TR_DATA_UNIT_512_B &
		 MASK_SDHCI_MSM_ICE_CTRL_INFO_CDU)
		 << OFFSET_SDHCI_MSM_ICE_CTRL_INFO_CDU;

	/* Configure ICE bypass mode */
	ctrl_info_val |=
		(bypass & MASK_SDHCI_MSM_ICE_CTRL_INFO_BYPASS)
		 << OFFSET_SDHCI_MSM_ICE_CTRL_INFO_BYPASS;

	writel_relaxed((lba & 0xFFFFFFFF),
		host->ioaddr + CORE_VENDOR_SPEC_ICE_CTRL_INFO_1_n + 16 * slot);
	writel_relaxed(((lba >> 32) & 0xFFFFFFFF),
		host->ioaddr + CORE_VENDOR_SPEC_ICE_CTRL_INFO_2_n + 16 * slot);
	writel_relaxed(ctrl_info_val,
		host->ioaddr + CORE_VENDOR_SPEC_ICE_CTRL_INFO_3_n + 16 * slot);

	/* Ensure ICE registers are configured before issuing SDHCI request */
	mb();
	return 0;
}

int sdhci_msm_ice_reset(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int err = 0;

	if (msm_host->ice.state != SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state before reset %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}

	init_completion(&msm_host->ice.async_done);

	if (msm_host->ice.vops->reset) {
		err = msm_host->ice.vops->reset(msm_host->ice.pdev);
		if (err) {
			pr_err("%s: ice reset failed %d\n",
					mmc_hostname(host->mmc), err);
			return err;
		}
	}

	if (!wait_for_completion_timeout(&msm_host->ice.async_done,
	     msecs_to_jiffies(SDHCI_MSM_ICE_COMPLETION_TIMEOUT_MS))) {
		pr_err("%s: ice reset timedout after %d ms\n",
			mmc_hostname(host->mmc),
			SDHCI_MSM_ICE_COMPLETION_TIMEOUT_MS);
		return -ETIMEDOUT;
	}

	if (msm_host->ice.state != SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state after reset %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}
	return 0;
}

int sdhci_msm_ice_resume(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int err = 0;

	if (msm_host->ice.state !=
			SDHCI_MSM_ICE_STATE_SUSPENDED) {
		pr_err("%s: ice is in invalid state before resume %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}

	init_completion(&msm_host->ice.async_done);

	if (msm_host->ice.vops->resume) {
		err = msm_host->ice.vops->resume(msm_host->ice.pdev);
		if (err) {
			pr_err("%s: ice resume failed %d\n",
					mmc_hostname(host->mmc), err);
			return err;
		}
	}

	if (!wait_for_completion_timeout(&msm_host->ice.async_done,
		msecs_to_jiffies(SDHCI_MSM_ICE_COMPLETION_TIMEOUT_MS))) {
		pr_err("%s: ice resume timedout after %d ms\n",
			mmc_hostname(host->mmc),
			SDHCI_MSM_ICE_COMPLETION_TIMEOUT_MS);
		return -ETIMEDOUT;
	}

	if (msm_host->ice.state != SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state after resume %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}
	return 0;
}

int sdhci_msm_ice_suspend(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int err = 0;

	if (msm_host->ice.state !=
			SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state before resume %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}

	if (msm_host->ice.vops->suspend) {
		err = msm_host->ice.vops->suspend(msm_host->ice.pdev);
		if (err) {
			pr_err("%s: ice suspend failed %d\n",
					mmc_hostname(host->mmc), err);
			return -EINVAL;
		}
	}
	msm_host->ice.state = SDHCI_MSM_ICE_STATE_SUSPENDED;
	return 0;
}

int sdhci_msm_ice_get_status(struct sdhci_host *host, int *ice_status)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int stat = -EINVAL;

	if (msm_host->ice.state != SDHCI_MSM_ICE_STATE_ACTIVE) {
		pr_err("%s: ice is in invalid state %d\n",
			mmc_hostname(host->mmc), msm_host->ice.state);
		return -EINVAL;
	}

	if (msm_host->ice.vops->status) {
		*ice_status = 0;
		stat = msm_host->ice.vops->status(msm_host->ice.pdev);
		if (stat < 0) {
			pr_err("%s: ice get sts failed %d\n",
					mmc_hostname(host->mmc), stat);
			return -EINVAL;
		}
		*ice_status = stat;
	}
	return 0;
}
