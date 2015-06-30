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
 * Intel Linux Wireless <ilw@linux.intel.com>
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
#include "mvm.h"
#include "fw-api-tof.h"

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
#include <linux/etherdevice.h>

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

#define IWL_MVM_TOF_RANGE_REQ_MAX_ID 256

void iwl_mvm_tof_init(struct iwl_mvm *mvm)
{
	struct iwl_mvm_tof_data *tof_data = &mvm->tof_data;

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return;

	memset(tof_data, 0, sizeof(*tof_data));

	tof_data->tof_cfg.sub_grp_cmd_id = cpu_to_le32(TOF_CONFIG_CMD);

	if (IWL_MVM_TOF_IS_RESPONDER) {
		tof_data->responder_cfg.sub_grp_cmd_id =
			cpu_to_le32(TOF_RESPONDER_CONFIG_CMD);
		tof_data->responder_cfg.sta_id = IWL_MVM_STATION_COUNT;
	}

	tof_data->range_req.sub_grp_cmd_id = cpu_to_le32(TOF_RANGE_REQ_CMD);
	tof_data->range_req.req_timeout = 1;
	tof_data->range_req.initiator = 1;
	tof_data->range_req.report_policy = 3;

	tof_data->range_req_ext.sub_grp_cmd_id =
		cpu_to_le32(TOF_RANGE_REQ_EXT_CMD);

	mvm->tof_data.active_range_request = IWL_MVM_TOF_RANGE_REQ_MAX_ID;

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
		}
	}
}

static void iwl_mvm_tsf_hash_free_elem(void *ptr, void *arg)
{
	kfree(ptr);
}
#endif

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

	memset(tof_data, 0, sizeof(*tof_data));
	mvm->tof_data.active_range_request = IWL_MVM_TOF_RANGE_REQ_MAX_ID;
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

	mvm->tof_data.active_range_request = IWL_MVM_TOF_RANGE_REQ_MAX_ID;
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

	if (id != mvm->tof_data.active_range_request) {
		IWL_ERR(mvm, "Invalid range request id %d (active %d)\n",
			id, mvm->tof_data.active_range_request);
		return -EINVAL;
	}

	/* after abort is sent there's no active request anymore */
	mvm->tof_data.active_range_request = IWL_MVM_TOF_RANGE_REQ_MAX_ID;

	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(cmd), &cmd);
}

int iwl_mvm_tof_responder_cmd(struct iwl_mvm *mvm,
			      struct ieee80211_vif *vif)
{
	struct iwl_tof_responder_config_cmd *cmd = &mvm->tof_data.responder_cfg;
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	if (vif->p2p || vif->type != NL80211_IFTYPE_AP) {
		IWL_ERR(mvm, "Cannot start responder, not in AP mode\n");
		return -EIO;
	}

	cmd->sta_id = mvmvif->bcast_sta.sta_id;
	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(*cmd), cmd);
}

int iwl_mvm_tof_range_request_cmd(struct iwl_mvm *mvm,
				  struct ieee80211_vif *vif)
{
	struct iwl_host_cmd cmd = {
		.id = TOF_CMD,
		.len = { sizeof(mvm->tof_data.range_req), },
		/* no copy because of the command size */
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
	};

	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	if (ieee80211_vif_type_p2p(vif) !=  NL80211_IFTYPE_STATION) {
		IWL_ERR(mvm, "Cannot send range request, not STA mode\n");
		return -EIO;
	}

	/* nesting of range requests is not supported in FW */
	if (mvm->tof_data.active_range_request !=
		IWL_MVM_TOF_RANGE_REQ_MAX_ID) {
		IWL_ERR(mvm, "Cannot send range req, already active req %d\n",
			mvm->tof_data.active_range_request);
		return -EIO;
	}

#ifdef CPTCFG_IWLMVM_TOF_TSF_WA
	iwl_mvm_tof_range_req_fill_tsf(mvm);
#endif
	mvm->tof_data.active_range_request = mvm->tof_data.range_req.request_id;

	cmd.data[0] = &mvm->tof_data.range_req;
	return iwl_mvm_send_cmd(mvm, &cmd);
}

int iwl_mvm_tof_range_request_ext_cmd(struct iwl_mvm *mvm,
				      struct ieee80211_vif *vif)
{
	lockdep_assert_held(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa, IWL_UCODE_TLV_CAPA_TOF_SUPPORT))
		return -EINVAL;

	if (ieee80211_vif_type_p2p(vif) !=  NL80211_IFTYPE_STATION) {
		IWL_ERR(mvm, "Cannot send ext range req, not in STA mode\n");
		return -EIO;
	}

	return iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(TOF_CMD,
						    IWL_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(mvm->tof_data.range_req_ext),
				    &mvm->tof_data.range_req_ext);
}

static int iwl_mvm_tof_range_resp(struct iwl_mvm *mvm, void *data)
{
	struct iwl_tof_range_rsp_ntfy *resp = (void *)data;

	if (resp->request_id != mvm->tof_data.active_range_request) {
		IWL_ERR(mvm, "Request id mismatch, got %d, active %d\n",
			resp->request_id, mvm->tof_data.active_range_request);
		return -EIO;
	}

	memcpy(&mvm->tof_data.range_resp, resp,
	       sizeof(struct iwl_tof_range_rsp_ntfy));
	mvm->tof_data.active_range_request = IWL_MVM_TOF_RANGE_REQ_MAX_ID;

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
