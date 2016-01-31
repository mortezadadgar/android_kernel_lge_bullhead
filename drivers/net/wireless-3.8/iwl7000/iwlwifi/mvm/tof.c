/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2015 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 * Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2015 Intel Deutschland GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#include <net/cfg80211.h>
#include <linux/etherdevice.h>
#include "mvm.h"
#include "iwl-io.h"
#include "iwl-prph.h"
#include "fw-api-tof.h"

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
static u32 tof_tsf_addr_hash(const void *key, u32 length, u32 seed)
{
	return jhash(key, ETH_ALEN, seed);
}

static const struct rhashtable_params tsf_rht_params = {
	.automatic_shrinking = true,
	.head_offset = offsetof(struct iwl_mvm_tof_tsf_entry, hash_node),
	.key_offset = offsetof(struct iwl_mvm_tof_tsf_entry, bssid),
	.key_len = ETH_ALEN,
	.hashfn = tof_tsf_addr_hash,
};
#endif

void iwl_mvm_tof_init(struct iwl_mvm *mvm)
{
	struct iwl_mvm_tof_data *tof_data = &mvm->tof_data;

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return;

	memset(tof_data, 0, sizeof(*tof_data));

	tof_data->tof_cfg.sub_grp_cmd_id = cpu_to_le32(TOF_CONFIG_CMD);

#ifdef CPTCFG_IWLWIFI_DEBUGFS
	if (IWL_MVM_TOF_IS_RESPONDER) {
		tof_data->responder_cfg.sub_grp_cmd_id =
			cpu_to_le32(TOF_RESPONDER_CONFIG_CMD);
		tof_data->responder_cfg.sta_id = IWL_MVM_STATION_COUNT;
		tof_data->responder_dyn_cfg.sub_grp_cmd_id =
			cpu_to_le32(TOF_RESPONDER_DYN_CONFIG_CMD);
	}
#endif

	tof_data->range_req.sub_grp_cmd_id = cpu_to_le32(TOF_RANGE_REQ_CMD);
	tof_data->range_req.req_timeout = 1;
	tof_data->range_req.initiator = 1;
	tof_data->range_req.report_policy = IWL_MVM_TOF_RESPONSE_COMPLETE;

	tof_data->range_req_ext.sub_grp_cmd_id =
		cpu_to_le32(TOF_RANGE_REQ_EXT_CMD);
	tof_data->range_req_ext.tsf_timer_offset_msec =
		cpu_to_le16(IWL_MVM_FTM_REQ_EXT_TSF_TIMER_OFFSET_MSEC_DFLT);
	tof_data->range_req_ext.min_delta_ftm =
		IWL_MVM_FTM_REQ_EXT_MIN_DELTA_FTM_DFLT;
	tof_data->range_req_ext.ftm_format_and_bw20M =
		IWL_MVM_FTM_REQ_EXT_FORMAT_AND_BW20M_DFLT;
	tof_data->range_req_ext.ftm_format_and_bw40M =
		IWL_MVM_FTM_REQ_EXT_FORMAT_AND_BW40M_DFLT;
	tof_data->range_req_ext.ftm_format_and_bw80M =
		IWL_MVM_FTM_REQ_EXT_FORMAT_AND_BW80M_DFLT;

	mvm->tof_data.active_request_id = IWL_MVM_TOF_RANGE_REQ_MAX_ID;
	mvm->tof_data.active_cookie = 0;

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
	{
		if (rhashtable_init(&tof_data->tsf_hash, &tsf_rht_params))
			IWL_ERR(mvm, "TSF hashtable init failed\n");
		else
			tof_data->tsf_hash_valid = true;
	}
#endif
}

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
void iwl_mvm_tof_update_tsf(struct iwl_mvm *mvm, struct iwl_rx_packet *pkt)
{
	u32 delta, ts;
	u8 delta_sign;
	struct iwl_mvm_tof_tsf_entry *tsf_entry;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)(pkt->data +
					sizeof(struct iwl_rx_mpdu_res_start));
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)hdr;

	if (!mvm->tof_data.tsf_hash_valid)
		return;

	ts = (u32)le64_to_cpu(mgmt->u.beacon.timestamp);
	if (ts > le32_to_cpu(mvm->last_phy_info.system_timestamp)) {
		delta = ts - le32_to_cpu(mvm->last_phy_info.system_timestamp);
		delta_sign = 0;
	} else {
		delta = le32_to_cpu(mvm->last_phy_info.system_timestamp) - ts;
		delta_sign = 1;
	}

	/* try to find this bss in the hash table */
	tsf_entry = rhashtable_lookup_fast(&mvm->tof_data.tsf_hash,
					   hdr->addr3, tsf_rht_params);
	if (tsf_entry) {
		tsf_entry->delta = delta;
		tsf_entry->delta_sign = delta_sign;
		return;
	}

	/* the bss is not found in the hash table */
	tsf_entry = kmalloc(sizeof(*tsf_entry), GFP_ATOMIC);
	if (!tsf_entry)
		return;

	tsf_entry->delta = delta;
	tsf_entry->delta_sign = delta_sign;
	ether_addr_copy(tsf_entry->bssid, hdr->addr3);

	rhashtable_insert_fast(&mvm->tof_data.tsf_hash, &tsf_entry->hash_node,
			       tsf_rht_params);
}

static void iwl_mvm_tof_range_req_fill_tsf(struct iwl_mvm *mvm)
{
	int i;
	struct iwl_tof_range_req_cmd *cmd = &mvm->tof_data.range_req;
	struct iwl_mvm_tof_tsf_entry *tsf_entry;

	if (!mvm->tof_data.tsf_hash_valid)
		return;

	for (i = 0; i < cmd->num_of_ap; i++) {
		tsf_entry = rhashtable_lookup_fast(&mvm->tof_data.tsf_hash,
						   cmd->ap[i].bssid,
						   tsf_rht_params);
		if (tsf_entry) {
			cmd->ap[i].tsf_delta = cpu_to_le32(tsf_entry->delta);
			cmd->ap[i].tsf_delta_direction = tsf_entry->delta_sign;
		} else {
			IWL_INFO(mvm, "Cannot find BSSID %pM\n",
				 cmd->ap[i].bssid);
			cmd->ap[i].tsf_delta = 0;
			cmd->ap[i].tsf_delta_direction = 0;
		}
	}
}

static void iwl_mvm_tsf_hash_free_elem(void *ptr, void *arg)
{
	kfree(ptr);
}
#endif

static void iwl_mvm_tof_reset_active(struct iwl_mvm *mvm)
{
	mvm->tof_data.active_request_id = IWL_MVM_TOF_RANGE_REQ_MAX_ID;
	mvm->tof_data.active_cookie = 0;
	kfree(mvm->tof_data.active_request.targets);
	mvm->tof_data.active_request.targets = NULL;
	memset(&mvm->tof_data.active_bssid_for_tsf, 0, ETH_ALEN);
}

void iwl_mvm_tof_clean(struct iwl_mvm *mvm)
{
	struct iwl_mvm_tof_data *tof_data = &mvm->tof_data;

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return;

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
	if (tof_data->tsf_hash_valid)
		rhashtable_free_and_destroy(&tof_data->tsf_hash,
					    iwl_mvm_tsf_hash_free_elem, NULL);
#endif

	kfree(tof_data->active_request.targets);
	memset(tof_data, 0, sizeof(*tof_data));
	mvm->tof_data.active_request_id = IWL_MVM_TOF_RANGE_REQ_MAX_ID;
}

static void iwl_tof_iterator(void *_data, u8 *mac,
			     struct ieee80211_vif *vif)
{
	bool *enabled = _data;

	/* non bss vif exists */
	if (ieee80211_vif_type_p2p(vif) !=  NL80211_IFTYPE_STATION)
		*enabled = false;
}

int iwl_mvm_tof_config_cmd(struct iwl_mvm *mvm)
{
	struct iwl_tof_config_cmd *cmd = &mvm->tof_data.tof_cfg;
	bool enabled;

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	ieee80211_iterate_active_interfaces_atomic(mvm->hw,
						   IEEE80211_IFACE_ITER_NORMAL,
						   iwl_tof_iterator, &enabled);
	if (!enabled) {
		IWL_DEBUG_INFO(mvm, "ToF is not supported (non bss vif)\n");
		return -EINVAL;
	}

	mvm->tof_data.active_request_id = IWL_MVM_TOF_RANGE_REQ_MAX_ID;
	mvm->tof_data.active_cookie = 0;
	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(*cmd), cmd);
}

int iwl_mvm_tof_range_abort_cmd(struct iwl_mvm *mvm, u8 id)
{
	struct iwl_tof_range_abort_cmd cmd = {
		.sub_grp_cmd_id = cpu_to_le32(TOF_RANGE_ABORT_CMD),
		.request_id = id,
	};

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	if (id != mvm->tof_data.active_request_id) {
		IWL_ERR(mvm, "Invalid range request id %d (active %d)\n",
			id, mvm->tof_data.active_request_id);
		return -EINVAL;
	}

	/* after abort is sent there's no active request anymore */
	iwl_mvm_tof_reset_active(mvm);

	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(cmd), &cmd);
}

static void
iwl_mvm_tof_set_responder(struct iwl_mvm *mvm,
			  struct ieee80211_vif *vif,
			  struct cfg80211_ftm_responder_params *params,
			  struct cfg80211_chan_def *def)
{
	struct iwl_tof_responder_config_cmd *cmd = &mvm->tof_data.responder_cfg;

	memset(cmd, 0, sizeof(*cmd));

	cmd->sub_grp_cmd_id = cpu_to_le32(TOF_RESPONDER_CONFIG_CMD);
	cmd->abort_responder = 0;

	cmd->channel_num = def->chan->hw_value;

	switch (def->width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
		cmd->bandwidth = IWL_TOF_BW_20_LEGACY;
		break;
	case NL80211_CHAN_WIDTH_20:
		cmd->bandwidth = IWL_TOF_BW_20_HT;
		break;
	case NL80211_CHAN_WIDTH_40:
		cmd->bandwidth = IWL_TOF_BW_40;
		if (def->center_freq1 > def->chan->center_freq)
			cmd->ctrl_ch_position = 1;
		break;
	case NL80211_CHAN_WIDTH_80:
		cmd->bandwidth = IWL_TOF_BW_80;
		if (def->center_freq1 > def->chan->center_freq)
			cmd->ctrl_ch_position = 2;
		if (abs((int)def->center_freq1 -
			 (int)def->chan->center_freq) > 20)
			cmd->ctrl_ch_position += 1;
		break;
	default:
		WARN_ON(1);
	}
}

int iwl_mvm_tof_responder_cmd(struct iwl_mvm *mvm,
			      struct ieee80211_vif *vif)
{
	struct iwl_tof_responder_config_cmd *cmd = &mvm->tof_data.responder_cfg;
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	if (vif->p2p || vif->type != NL80211_IFTYPE_AP ||
	    !mvmvif->ap_ibss_active) {
		IWL_ERR(mvm, "Cannot start responder, not in AP mode\n");
		return -EIO;
	}

	cmd->sta_id = mvmvif->bcast_sta.sta_id;
	memcpy(cmd->bssid, vif->addr, ETH_ALEN);
	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(*cmd), cmd);
}

static void
iwl_mvm_tof_set_responder_dyn(struct iwl_mvm *mvm,
			      struct ieee80211_vif *vif,
			      struct cfg80211_ftm_responder_params *params)
{
	struct iwl_tof_responder_dyn_config_cmd *cmd =
					&mvm->tof_data.responder_dyn_cfg;
	int aligned = ALIGN(params->lci_len + 2, 4);

	if (aligned + 2 + params->civic_len > IWL_TOF_LCI_CIVIC_BUF_SIZE)
		return;

	memset(cmd, 0, sizeof(*cmd));

	cmd->sub_grp_cmd_id = cpu_to_le32(TOF_RESPONDER_DYN_CONFIG_CMD);

	cmd->lci_len = cpu_to_le32(params->lci_len + 2);
	cmd->civic_len = cpu_to_le32(params->civic_len + 2);

	cmd->lci_civic[0] = WLAN_EID_MEASURE_REPORT;
	cmd->lci_civic[1] = params->lci_len;
	memcpy(cmd->lci_civic + 2, params->lci, params->lci_len);

	cmd->lci_civic[aligned] = WLAN_EID_MEASURE_REPORT;
	cmd->lci_civic[aligned + 1] = params->civic_len;
	memcpy(cmd->lci_civic + aligned + 2, params->civic, params->civic_len);
}

int iwl_mvm_tof_responder_dyn_cfg_cmd(struct iwl_mvm *mvm,
				      struct ieee80211_vif *vif)
{
	struct iwl_tof_responder_dyn_config_cmd *cmd =
		&mvm->tof_data.responder_dyn_cfg;
	u32 actual_lci_len =
		ALIGN(le32_to_cpu(cmd->lci_len), 4);
	u32 actual_civic_len =
		ALIGN(le32_to_cpu(cmd->civic_len), 4);

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	if (vif->p2p || vif->type != NL80211_IFTYPE_AP) {
		IWL_ERR(mvm, "Cannot start responder, not in AP mode\n");
		return -EIO;
	}

	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(*cmd) + actual_lci_len +
				    actual_civic_len, cmd);
}

int iwl_mvm_tof_start_responder(struct iwl_mvm *mvm,
				struct ieee80211_vif *vif,
				struct cfg80211_ftm_responder_params *params)
{
	struct ieee80211_chanctx_conf ctx, *pctx;
	u16 *phy_ctxt_id;
	struct iwl_mvm_phy_ctxt *phy_ctxt;
	int ret;

	lockdep_assert_held(&mvm->mutex);

	rcu_read_lock();
	pctx = rcu_dereference(vif->chanctx_conf);
	/* Copy the ctx to unlock the rcu and send the phy ctxt. We don't care
	 * about changes in the ctx after releasing the lock because the driver
	 * is still protected by the mutex. */
	ctx = *pctx;
	phy_ctxt_id  = (u16 *)pctx->drv_priv;
	rcu_read_unlock();

	phy_ctxt = &mvm->phy_ctxts[*phy_ctxt_id];
	ret = iwl_mvm_phy_ctxt_changed(mvm, phy_ctxt, &ctx.def,
				       ctx.rx_chains_static,
				       ctx.rx_chains_dynamic);
	if (ret)
		return ret;

	iwl_mvm_tof_set_responder(mvm, vif, params, &ctx.def);
	ret = iwl_mvm_tof_responder_cmd(mvm, vif);
	if (ret)
		return ret;

	if (params->lci_len || params->civic_len) {
		iwl_mvm_tof_set_responder_dyn(mvm, vif, params);
		ret = iwl_mvm_tof_responder_dyn_cfg_cmd(mvm, vif);
	}

	return ret;
}

void iwl_mvm_tof_restart_responder(struct iwl_mvm *mvm,
				   struct ieee80211_vif *vif)
{
	iwl_mvm_tof_responder_cmd(mvm, vif);
	if (mvm->tof_data.responder_dyn_cfg.lci_len ||
	    mvm->tof_data.responder_dyn_cfg.civic_len)
		iwl_mvm_tof_responder_dyn_cfg_cmd(mvm, vif);
}

int iwl_mvm_tof_perform_ftm(struct iwl_mvm *mvm, u64 cookie,
			    struct ieee80211_vif *vif,
			    struct cfg80211_ftm_request *req)
{
	struct iwl_tof_range_req_cmd *cmd = &mvm->tof_data.range_req;
	int i;
	int ret = 0;

	lockdep_assert_held(&mvm->mutex);

	/* nesting of range requests is not supported in FW */
	if (mvm->tof_data.active_request_id != IWL_MVM_TOF_RANGE_REQ_MAX_ID) {
		IWL_DEBUG_INFO(mvm,
			       "Cannot send range req, already active req %d\n",
			       mvm->tof_data.active_request_id);
		return -EBUSY;
	}

	/* FW requires sending the ext command prior to each range request */
	ret = iwl_mvm_tof_range_request_ext_cmd(mvm);
	if (ret)
		goto err;

	cmd->request_id++;
	if (cmd->request_id == 0)
		cmd->request_id++;
	cmd->one_sided_los_disable = 0;
	cmd->req_timeout = req->timeout;
	cmd->report_policy = IWL_MVM_TOF_RESPONSE_COMPLETE;
	cmd->num_of_ap = req->num_of_targets;
	cmd->macaddr_random = 1;
	memcpy(cmd->macaddr_template, req->macaddr_template, ETH_ALEN);
	for (i = 0; i < ETH_ALEN; i++)
		cmd->macaddr_mask[i] = ~req->macaddr_mask[i];

	for (i = 0; i < cmd->num_of_ap; i++) {
		struct cfg80211_ftm_target *req_target = &req->targets[i];
		struct iwl_tof_range_req_ap_entry *cmd_target = &cmd->ap[i];

		cmd_target->channel_num = ieee80211_frequency_to_channel(
				req_target->chan_def.chan->center_freq);
		switch (req_target->chan_def.width) {
		case NL80211_CHAN_WIDTH_20_NOHT:
			cmd_target->bandwidth = IWL_TOF_BW_20_LEGACY;
			break;
		case NL80211_CHAN_WIDTH_20:
			cmd_target->bandwidth = IWL_TOF_BW_20_HT;
			break;
		case NL80211_CHAN_WIDTH_40:
			cmd_target->bandwidth = IWL_TOF_BW_40;
			break;
		case NL80211_CHAN_WIDTH_80:
			cmd_target->bandwidth = IWL_TOF_BW_80;
			break;
		default:
			IWL_ERR(mvm, "Unsupported BW in FTM request (%d)\n",
				req_target->chan_def.width);
			ret = -EINVAL;
			goto err;
		}
		cmd_target->ctrl_ch_position =
			(req_target->chan_def.width > NL80211_CHAN_WIDTH_20) ?
			iwl_mvm_get_ctrl_pos(&req_target->chan_def) : 0;

		cmd_target->tsf_delta_direction = 0;
		cmd_target->tsf_delta = 0;

		memcpy(cmd_target->bssid, req_target->bssid, ETH_ALEN);
		cmd_target->measure_type = req_target->one_sided;
		cmd_target->num_of_bursts = req_target->num_of_bursts_exp;
		cmd_target->burst_period =
			cpu_to_le16(req_target->burst_period);
		cmd_target->samples_per_burst = req_target->samples_per_burst;
		cmd_target->retries_per_sample = req_target->retries;
		cmd_target->asap_mode = req_target->asap;
		cmd_target->enable_dyn_ack = 1;
		cmd_target->rssi = 0;

		if (req_target->lci)
			cmd_target->location_req |= IWL_TOF_LOC_LCI;
		if (req_target->civic)
			cmd_target->location_req |= IWL_TOF_LOC_CIVIC;
	}

	mvm->tof_data.active_cookie = cookie;
	memcpy(&mvm->tof_data.active_request, req,
	       sizeof(struct cfg80211_ftm_request));
	if (vif->bss_conf.assoc && req->report_tsf)
		memcpy(&mvm->tof_data.active_bssid_for_tsf,
		       &vif->bss_conf.bssid, ETH_ALEN);

	if (vif->bss_conf.assoc)
		memcpy(cmd->range_req_bssid, vif->bss_conf.bssid, ETH_ALEN);
	else
		eth_broadcast_addr(cmd->range_req_bssid);

	return iwl_mvm_tof_range_request_cmd(mvm);

err:
	iwl_mvm_tof_reset_active(mvm);
	return ret;
}

int iwl_mvm_tof_abort_ftm(struct iwl_mvm *mvm, u64 cookie)
{
	lockdep_assert_held(&mvm->mutex);

	if (cookie != mvm->tof_data.active_cookie)
		return -EINVAL;

	return iwl_mvm_tof_range_abort_cmd(mvm,
					   mvm->tof_data.active_request_id);
}

int iwl_mvm_tof_range_request_cmd(struct iwl_mvm *mvm)
{
	int err;
	struct iwl_host_cmd cmd = {
		.id = iwl_cmd_id(TOF_CMD, IWL_ALWAYS_LONG_GROUP, 0),
		.len = { sizeof(mvm->tof_data.range_req), },
		/* no copy because of the command size */
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
	};

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
	iwl_mvm_tof_range_req_fill_tsf(mvm);
#endif
	mvm->tof_data.active_request_id = mvm->tof_data.range_req.request_id;

	cmd.data[0] = &mvm->tof_data.range_req;
	err = iwl_mvm_send_cmd(mvm, &cmd);
	if (err)
		iwl_mvm_tof_reset_active(mvm);

	return err;
}

int iwl_mvm_tof_range_request_ext_cmd(struct iwl_mvm *mvm)
{
	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(mvm->tof_data.range_req_ext),
				    &mvm->tof_data.range_req_ext);
}

static struct cfg80211_ftm_target *
iwl_mvm_tof_find_target_in_request(struct iwl_mvm *mvm, const u8 *bssid)
{
	int i;

	for (i = 0; i < mvm->tof_data.active_request.num_of_targets; i++)
		if (ether_addr_equal_unaligned(
		    mvm->tof_data.active_request.targets[i].bssid, bssid))
			return &mvm->tof_data.active_request.targets[i];

	return NULL;
}

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
static u64 iwl_mvm_tof_get_tsf(struct iwl_mvm *mvm, u32 gp2_ts)
{
	struct iwl_mvm_tof_tsf_entry *tsf_entry;
	u8 *bssid = mvm->tof_data.active_bssid_for_tsf;

	tsf_entry = rhashtable_lookup_fast(&mvm->tof_data.tsf_hash, bssid,
					   tsf_rht_params);
	if (!tsf_entry)
		return 0;

	if (tsf_entry->delta_sign)
		return (u64)gp2_ts - tsf_entry->delta;
	else
		return (u64)gp2_ts + tsf_entry->delta;
}
#endif

static u64 iwl_mvm_tof_get_host_time(struct iwl_mvm *mvm, u32 msrment_gp2_ts)
{
	u32 curr_gp2, diff;
	u64 now_from_boot_ns;

	iwl_mvm_get_sync_time(mvm, &curr_gp2, &now_from_boot_ns);

	if (curr_gp2 >= msrment_gp2_ts)
		diff = curr_gp2 - msrment_gp2_ts;
	else
		diff = curr_gp2 + (U32_MAX - msrment_gp2_ts + 1);

	return now_from_boot_ns - (u64)diff * 1000;
}

static inline enum rate_info_bw iwl_mvm_tof_fw_bw_to_rate_info_bw(u8 fw_bw)
{
	switch (fw_bw) {
	case 0:
		return RATE_INFO_BW_20;
	case 1:
		return RATE_INFO_BW_40;
	case 2:
		return RATE_INFO_BW_80;
	default:
		break;
	}

	return -1;
}

static inline int iwl_mvm_tof_is_ht(struct iwl_mvm *mvm, u8 fw_bw)
{
	switch (iwl_mvm_tof_fw_bw_to_rate_info_bw(fw_bw)) {
	case RATE_INFO_BW_20:
		return mvm->tof_data.range_req_ext.ftm_format_and_bw20M ==
			IEEE80211_FTM_FORMAT_BW_HT_20;
	case RATE_INFO_BW_40:
		return mvm->tof_data.range_req_ext.ftm_format_and_bw40M ==
			IEEE80211_FTM_FORMAT_BW_HT_40;
	default:
		break;
	}

	return 0;
}

static inline int iwl_mvm_tof_is_vht(struct iwl_mvm *mvm, u8 fw_bw)
{
	switch (iwl_mvm_tof_fw_bw_to_rate_info_bw(fw_bw)) {
	case RATE_INFO_BW_20:
		return mvm->tof_data.range_req_ext.ftm_format_and_bw20M ==
			IEEE80211_FTM_FORMAT_BW_VHT_20;
	case RATE_INFO_BW_40:
		return mvm->tof_data.range_req_ext.ftm_format_and_bw40M ==
			IEEE80211_FTM_FORMAT_BW_VHT_40;
	case RATE_INFO_BW_80:
		return mvm->tof_data.range_req_ext.ftm_format_and_bw80M ==
			IEEE80211_FTM_FORMAT_BW_VHT_80;
	default:
		break;
	}

	return 0;
}

static int iwl_mvm_tof_range_resp(struct iwl_mvm *mvm, void *data)
{
	struct iwl_tof_range_rsp_ntfy *fw_resp = (void *)data;
	struct cfg80211_msrment_response user_resp = {0};
	int i;

	lockdep_assert_held(&mvm->mutex);

	if (fw_resp->request_id != mvm->tof_data.active_request_id) {
		IWL_ERR(mvm, "Request id mismatch, got %d, active %d\n",
			fw_resp->request_id,
			mvm->tof_data.active_request_id);
		return -EIO;
	}

	if (fw_resp->num_of_aps > mvm->tof_data.active_request.num_of_targets) {
		IWL_ERR(mvm, "FTM range response invalid\n");
		return -EINVAL;
	}

	user_resp.cookie = mvm->tof_data.active_cookie;
	user_resp.type = NL80211_MSRMENT_TYPE_FTM;
	user_resp.status = fw_resp->request_status ?
		NL80211_MSRMENT_STATUS_FAIL : NL80211_MSRMENT_STATUS_SUCCESS;
	user_resp.u.ftm.num_of_entries = fw_resp->num_of_aps;
	user_resp.u.ftm.entries = kzalloc(sizeof(*user_resp.u.ftm.entries) *
					  fw_resp->num_of_aps, GFP_KERNEL);
	if (!user_resp.u.ftm.entries) {
		iwl_mvm_tof_reset_active(mvm);
		return -ENOMEM;
	}

	for (i = 0; i < fw_resp->num_of_aps && i < IWL_MVM_TOF_MAX_APS; i++) {
		struct cfg80211_ftm_result *result =
			&user_resp.u.ftm.entries[i];
		struct iwl_tof_range_rsp_ap_entry_ntfy *fw_ap = &fw_resp->ap[i];
		struct cfg80211_ftm_target *target;
		u32 timestamp;

		target = iwl_mvm_tof_find_target_in_request(mvm, fw_ap->bssid);
		if (!target) {
			IWL_WARN(mvm,
				 "Unknown bssid (target #%d) in FTM response\n",
				 i);
			continue;
		}

		/* TODO: Once FW supports more meaningful status, use it. */
		result->status = fw_resp->ap[i].measure_status ?
			NL80211_FTM_RESP_FAIL : NL80211_FTM_RESP_SUCCESS;
		result->complete = fw_resp->last_in_batch;
		result->target = target;
		timestamp = le32_to_cpu(fw_ap->timestamp);
		result->host_time =
			iwl_mvm_tof_get_host_time(mvm, timestamp);
#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
		if (mvm->tof_data.active_request.report_tsf)
			result->tsf = iwl_mvm_tof_get_tsf(mvm, timestamp);
#endif
		/* TODO: FW to investigate */
		result->burst_index = 0;
		result->rssi = fw_ap->rssi;
		result->rssi_spread = fw_ap->rssi_spread;
		if (iwl_mvm_tof_is_ht(mvm, fw_ap->measure_bw))
			result->tx_rate_info.flags |= RATE_INFO_FLAGS_MCS;
		if (iwl_mvm_tof_is_vht(mvm, fw_ap->measure_bw))
			result->tx_rate_info.flags |= RATE_INFO_FLAGS_VHT_MCS;
		/* TODO: FW to investigate */
		result->tx_rate_info.mcs = 12;
		/* TODO: FW to investigate */
		result->tx_rate_info.legacy = 60;
		/* TODO: FW to investigate */
		result->tx_rate_info.nss = 1;
#if CFG80211_VERSION < KERNEL_VERSION(3,20,0)
		switch (iwl_mvm_tof_fw_bw_to_rate_info_bw(fw_ap->measure_bw)) {
		default:
		case RATE_INFO_BW_20:
			break;
		case RATE_INFO_BW_40:
			result->tx_rate_info.flags |= RATE_INFO_FLAGS_40_MHZ_WIDTH;
			break;
		case RATE_INFO_BW_80:
			result->tx_rate_info.flags |= RATE_INFO_FLAGS_80_MHZ_WIDTH;
			break;
		}
#else
		result->tx_rate_info.bw =
			iwl_mvm_tof_fw_bw_to_rate_info_bw(fw_ap->measure_bw);
#endif
		/* TODO: FW to investigate */
		result->rx_rate_info = result->tx_rate_info;
		result->rtt = le32_to_cpu(fw_ap->rtt);
		result->rtt_variance = le32_to_cpu(fw_ap->rtt_variance);
		result->rtt_spread = le32_to_cpu(fw_ap->rtt_spread);
	}

	cfg80211_measurement_response(mvm->hw->wiphy, &user_resp, GFP_KERNEL);
	kfree(user_resp.u.ftm.entries);

	/* for debugfs retrieving */
	memcpy(&mvm->tof_data.range_resp, fw_resp,
	       sizeof(struct iwl_tof_range_rsp_ntfy));

	if (fw_resp->last_in_batch)
		iwl_mvm_tof_reset_active(mvm);

	return 0;
}

static int iwl_mvm_tof_mcsi_notif(struct iwl_mvm *mvm, void *data)
{
	struct iwl_tof_mcsi_notif *resp = (struct iwl_tof_mcsi_notif *)data;

	IWL_DEBUG_INFO(mvm, "MCSI notification, token %d\n", resp->token);
	return 0;
}

static int iwl_mvm_tof_nb_report_notif(struct iwl_mvm *mvm, void *data)
{
	struct iwl_tof_neighbor_report *report =
		(struct iwl_tof_neighbor_report *)data;

	IWL_DEBUG_INFO(mvm, "NB report, bssid %pM, token %d, status 0x%x\n",
		       report->bssid, report->request_token, report->status);
	return 0;
}

void iwl_mvm_tof_resp_handler(struct iwl_mvm *mvm,
			      struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_tof_gen_resp_cmd *resp = (void *)pkt->data;

	lockdep_assert_held(&mvm->mutex);

	switch (le32_to_cpu(resp->sub_grp_cmd_id)) {
	case TOF_RANGE_RESPONSE_NOTIF:
		iwl_mvm_tof_range_resp(mvm, resp->data);
		break;
	case TOF_MCSI_DEBUG_NOTIF:
		iwl_mvm_tof_mcsi_notif(mvm, resp->data);
		break;
	case TOF_NEIGHBOR_REPORT_RSP_NOTIF:
		iwl_mvm_tof_nb_report_notif(mvm, resp->data);
		break;
	default:
	       IWL_ERR(mvm, "Unknown sub-group command 0x%x\n",
		       resp->sub_grp_cmd_id);
	       break;
	}
}
