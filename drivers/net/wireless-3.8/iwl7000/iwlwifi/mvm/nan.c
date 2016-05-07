/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2015-2016 Intel Deutschland GmbH
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
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 * Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2015-2016 Intel Deutschland GmbH
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
#include "fw-api-nan.h"

#define NAN_WARMUP_TIMEOUT_USEC  (120000000ULL)
#define NAN_CHANNEL_24           (6)
#define NAN_CHANNEL_52           (149)

enum srf_type {
	SRF_BF_TYPE             = BIT(0),
	SRF_INCLUDE             = BIT(1),
	SRF_BLOOM_FILTER_IDX    = BIT(2) | BIT(3),
};

static bool iwl_mvm_can_beacon(struct ieee80211_vif *vif,
			       enum ieee80211_band band, u8 channel)
{
	struct wiphy *wiphy = ieee80211_vif_to_wdev(vif)->wiphy;
	int freq = ieee80211_channel_to_frequency(channel, band);
	struct ieee80211_channel *chan = ieee80211_get_channel(wiphy,
							       freq);
	struct cfg80211_chan_def def;

	if (!chan)
		return false;

	cfg80211_chandef_create(&def, chan, NL80211_CHAN_NO_HT);
	return cfg80211_reg_can_beacon(wiphy, &def, vif->type);
}

int iwl_mvm_start_nan(struct ieee80211_hw *hw,
		      struct ieee80211_vif *vif,
		      struct cfg80211_nan_conf *conf)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	struct iwl_nan_cfg_cmd cmd = {};
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	int ret = 0;

	IWL_DEBUG_MAC80211(IWL_MAC80211_GET_MVM(hw), "Start NAN\n");

	if (!iwl_mvm_can_beacon(vif, NL80211_BAND_2GHZ, NAN_CHANNEL_24))
		return -EINVAL;

	mutex_lock(&mvm->mutex);

	cmd.action = cpu_to_le32(FW_CTXT_ACTION_ADD);
	cmd.tsf_id = cpu_to_le32(mvmvif->tsf_id);
	cmd.beacon_template_id = cpu_to_le32(mvmvif->id);

	ether_addr_copy(cmd.node_addr, vif->addr);
	cmd.sta_id = cpu_to_le32(mvm->aux_sta.sta_id);
	cmd.master_pref = conf->master_pref;
	if (conf->dual == NL80211_NAN_BAND_DUAL) {
		if (!iwl_mvm_can_beacon(vif, IEEE80211_BAND_5GHZ,
					NAN_CHANNEL_52)) {
			IWL_ERR(mvm, "Can't beacon on %d\n", NAN_CHANNEL_52);
			ret = -EINVAL;
			goto out;
		}

		cmd.dual_band = cpu_to_le32(1);
		cmd.chan52 = NAN_CHANNEL_52;
	}

	cmd.chan24 = NAN_CHANNEL_24;
	cmd.warmup_timer = cpu_to_le32(NAN_WARMUP_TIMEOUT_USEC);
	cmd.op_bands = 3;

	ret = iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(NAN_CONFIG_CMD,
						   NAN_GROUP, 0),
				   0, sizeof(cmd), &cmd);

	if (!ret)
		mvm->nan_vif = vif;

out:
	mutex_unlock(&mvm->mutex);

	return ret;
}

int iwl_mvm_stop_nan(struct ieee80211_hw *hw,
		     struct ieee80211_vif *vif)
{
	struct iwl_nan_cfg_cmd cmd = {};
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	int ret = 0;

	IWL_DEBUG_MAC80211(IWL_MAC80211_GET_MVM(hw), "Stop NAN\n");

	mutex_lock(&mvm->mutex);
	cmd.action = cpu_to_le32(FW_CTXT_ACTION_REMOVE);

	ret = iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(NAN_CONFIG_CMD,
						   NAN_GROUP, 0),
				   0, sizeof(cmd), &cmd);

	if (!ret)
		mvm->nan_vif = NULL;
	mutex_unlock(&mvm->mutex);

	return ret;
}

static enum iwl_fw_nan_func_type
iwl_fw_nan_func_type(enum nl80211_nan_function_type type)
{
	switch (type) {
	case NL80211_NAN_FUNC_PUBLISH:
		return NAN_DE_FUNC_PUBLISH;
	case NL80211_NAN_FUNC_SUBSCRIBE:
		return NAN_DE_FUNC_SUBSCRIBE;
	case NL80211_NAN_FUNC_FOLLOW_UP:
		return NAN_DE_FUNC_FOLLOW_UP;
	default:
		return NAN_DE_FUNC_NOT_VALID;
	}
}

static u8
iwl_mvm_get_match_filter_len(struct cfg80211_nan_func_filter *filters,
			     u8 num_filters)
{
	int i;
	unsigned int len = 0;

	len += num_filters;
	for (i = 0; i < num_filters; i++)
		len += filters[i].len;

	if (WARN_ON_ONCE(len > U8_MAX))
		return 0;

	return len;
}

static void iwl_mvm_copy_filters(struct cfg80211_nan_func_filter *filters,
				 u8 num_filters, u8 *cmd_data)
{
	int i;
	u8 offset = 0;

	for (i = 0; i < num_filters; i++) {
		memcpy(cmd_data + offset, &filters[i].len,
		       sizeof(u8));
		offset++;
		if (filters[i].len > 0)
			memcpy(cmd_data + offset, filters[i].filter,
			       filters[i].len);

		offset += filters[i].len;
	}
}

int iwl_mvm_add_nan_func(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif,
			 const struct cfg80211_nan_func *nan_func)
{
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct iwl_nan_add_func_cmd *cmd;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(NAN_DISCOVERY_FUNC_CMD, NAN_GROUP, 0),
		.flags = CMD_WANT_SKB,
	};
	struct iwl_nan_add_func_res *resp;
	struct iwl_rx_packet *pkt;
	u8 *cmd_data;
	u16 flags = 0;
	u8 tx_filt_len, rx_filt_len;
	size_t cmd_len;
	int ret = 0;

	IWL_DEBUG_MAC80211(IWL_MAC80211_GET_MVM(hw), "Add NAN func\n");

	mutex_lock(&mvm->mutex);

	/* We assume here that mac80211 properly validated the nan_func */
	cmd_len = sizeof(*cmd) + ALIGN(nan_func->serv_spec_info_len, 4);
	if (nan_func->srf_bf_len)
		cmd_len += ALIGN(nan_func->srf_bf_len + 1, 4);
	else if (nan_func->srf_num_macs)
		cmd_len += ALIGN(nan_func->srf_num_macs * ETH_ALEN + 1, 4);

	rx_filt_len = iwl_mvm_get_match_filter_len(nan_func->rx_filters,
						   nan_func->num_rx_filters);

	tx_filt_len = iwl_mvm_get_match_filter_len(nan_func->tx_filters,
						   nan_func->num_tx_filters);

	cmd_len += ALIGN(rx_filt_len, 4);
	cmd_len += ALIGN(tx_filt_len, 4);

	cmd = kzalloc(cmd_len, GFP_KERNEL);

	if (!cmd) {
		ret = -ENOBUFS;
		goto unlock;
	}

	hcmd.len[0] = cmd_len;
	hcmd.data[0] = cmd;

	cmd_data = cmd->data;
	cmd->action = cpu_to_le32(FW_CTXT_ACTION_ADD);
	cmd->type = iwl_fw_nan_func_type(nan_func->type);
	cmd->instance_id = nan_func->instance_id;

	memcpy(&cmd->service_id, nan_func->service_id, sizeof(cmd->service_id));

	/*
	 * TODO: Currently we want all the events, however we might need to be
	 * able to unset this flag for solicited publish to disable "Replied"
	 * events.
	 */
	flags |= NAN_DE_FUNC_FLAG_RAISE_EVENTS;
	if (nan_func->subscribe_active ||
	    nan_func->publish_type == NL80211_NAN_UNSOLICITED_PUBLISH)
		flags |= NAN_DE_FUNC_FLAG_UNSOLICITED_OR_ACTIVE;

	if (nan_func->close_range)
		flags |= NAN_DE_FUNC_FLAG_CLOSE_RANGE;

	if (nan_func->type == NL80211_NAN_FUNC_FOLLOW_UP ||
	    (nan_func->type == NL80211_NAN_FUNC_PUBLISH &&
	     !nan_func->publish_bcast))
		flags |= NAN_DE_FUNC_FLAG_UNICAST;

	if (nan_func->publish_type == NL80211_NAN_SOLICITED_PUBLISH)
		flags |= NAN_DE_FUNC_FLAG_SOLICITED;

	cmd->flags = cpu_to_le16(flags);
	cmd->ttl = cpu_to_le32(nan_func->ttl);
	cmd->serv_info_len = nan_func->serv_spec_info_len;
	if (nan_func->serv_spec_info_len)
		memcpy(cmd_data, nan_func->serv_spec_info,
		       nan_func->serv_spec_info_len);

	if (nan_func->type == NL80211_NAN_FUNC_FOLLOW_UP) {
		cmd->flw_up_id = nan_func->followup_id;
		cmd->flw_up_req_id = nan_func->followup_reqid;
		memcpy(cmd->flw_up_addr, nan_func->followup_dest.addr,
		       ETH_ALEN);
	}

	cmd_data += ALIGN(cmd->serv_info_len, 4);
	if (nan_func->srf_bf_len) {
		u8 srf_ctl = 0;

		srf_ctl |= SRF_BF_TYPE;
		srf_ctl |= (nan_func->srf_bf_idx << 2) & SRF_BLOOM_FILTER_IDX;
		if (nan_func->srf_include)
			srf_ctl |= SRF_INCLUDE;

		cmd->srf_len = nan_func->srf_bf_len + 1;
		memcpy(cmd_data, &srf_ctl, sizeof(srf_ctl));
		memcpy(cmd_data + 1, nan_func->srf_bf, nan_func->srf_bf_len);
	} else if (nan_func->srf_num_macs) {
		u8 srf_ctl = 0;
		int i;

		if (nan_func->srf_include)
			srf_ctl |= SRF_INCLUDE;

		cmd->srf_len = nan_func->srf_num_macs * ETH_ALEN + 1;
		memcpy(cmd_data, &srf_ctl, sizeof(srf_ctl));

		for (i = 0; i < nan_func->srf_num_macs; i++) {
			memcpy(cmd_data + 1 + i * ETH_ALEN,
			       nan_func->srf_macs[i].addr, ETH_ALEN);
		}
	}

	cmd_data += ALIGN(cmd->srf_len, 4);

	if (rx_filt_len > 0)
		iwl_mvm_copy_filters(nan_func->rx_filters,
				     nan_func->num_rx_filters, cmd_data);

	cmd->rx_filter_len = rx_filt_len;
	cmd_data += ALIGN(cmd->rx_filter_len, 4);

	if (tx_filt_len > 0)
		iwl_mvm_copy_filters(nan_func->tx_filters,
				     nan_func->num_tx_filters, cmd_data);

	cmd->tx_filter_len = tx_filt_len;

	ret = iwl_mvm_send_cmd(mvm, &hcmd);

	if (ret) {
		IWL_ERR(mvm, "Couldn't send NAN_DISCOVERY_FUNC_CMD: %d\n", ret);
		goto out_free;
	}

	pkt = hcmd.resp_pkt;
	if (WARN_ON(!pkt)) {
		ret = -EIO;
		goto out_free_resp;
	}

	if (WARN_ON(iwl_rx_packet_payload_len(pkt) != sizeof(*resp))) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;

	IWL_DEBUG_MAC80211(mvm,
			   "Add NAN func response status: %d, instance_id: %d\n",
			   resp->status, resp->instance_id);

	if (resp->status == NAN_DE_FUNC_STATUS_INSUFFICIENT_ENTRIES ||
	    resp->status == NAN_DE_FUNC_STATUS_INSUFFICIENT_MEMORY) {
		ret = -ENOBUFS;
		goto out_free_resp;
	}

	if (resp->status != NAN_DE_FUNC_STATUS_SUCCESSFUL) {
		ret = -EIO;
		goto out_free_resp;
	}

	if (cmd->instance_id &&
	    WARN_ON(resp->instance_id != cmd->instance_id)) {
		ret = -EIO;
		goto out_free_resp;
	}

	ret = 0;
out_free_resp:
	iwl_free_resp(&hcmd);
out_free:
	kfree(cmd);
unlock:
	mutex_unlock(&mvm->mutex);
	return ret;
}

void iwl_mvm_rm_nan_func(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif,
			 u8 instance_id)
{
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct iwl_nan_add_func_cmd cmd = {0};
	int ret;

	IWL_DEBUG_MAC80211(IWL_MAC80211_GET_MVM(hw), "Remove NAN func\n");
	mutex_lock(&mvm->mutex);
	cmd.action = cpu_to_le32(FW_CTXT_ACTION_REMOVE);
	cmd.instance_id = instance_id;

	ret = iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(NAN_DISCOVERY_FUNC_CMD,
						   NAN_GROUP, 0),
				   0, sizeof(cmd), &cmd);
	if (ret)
		IWL_ERR(mvm, "Failed to remove NAN func instance_id: %d\n",
			instance_id);

	mutex_unlock(&mvm->mutex);
}

static u8 iwl_cfg_nan_func_type(u8 fw_type)
{
	switch (fw_type) {
	case NAN_DE_FUNC_PUBLISH:
		return NL80211_NAN_FUNC_PUBLISH;
	case NAN_DE_FUNC_SUBSCRIBE:
		return NL80211_NAN_FUNC_SUBSCRIBE;
	case NAN_DE_FUNC_FOLLOW_UP:
		return NL80211_NAN_FUNC_FOLLOW_UP;
	default:
		return NL80211_NAN_FUNC_MAX_TYPE + 1;
	}
}

void iwl_mvm_nan_match(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_nan_disc_evt_notify *ev = (void *)pkt->data;
	struct cfg80211_nan_match_params match = {0};
	int len = iwl_rx_packet_payload_len(pkt);

	if (WARN_ON_ONCE(!mvm->nan_vif)) {
		IWL_ERR(mvm, "NAN vif is NULL\n");
		return;
	}

	if (WARN_ON_ONCE(len < sizeof(*ev))) {
		IWL_ERR(mvm, "Invalid NAN match event length: %d\n",
			len);
		return;
	}

	if (WARN_ON_ONCE(len < sizeof(*ev) + ev->service_info_len)) {
		IWL_ERR(mvm,
			"Invalid NAN match event length: %d, info_len: %d\n",
			len, ev->service_info_len);
		return;
	}

	match.type = iwl_cfg_nan_func_type(ev->type);

	if (WARN_ON_ONCE(match.type > NL80211_NAN_FUNC_MAX_TYPE)) {
		IWL_ERR(mvm, "Invalid func type\n");
		return;
	}

	match.inst_id = ev->instance_id;
	match.peer_inst_id = ev->peer_instance;
	match.addr = ev->peer_mac_addr;
	match.info = ev->service_info;
	match.info_len = ev->service_info_len;
	ieee80211_nan_func_match(mvm->nan_vif, &match,
				 GFP_ATOMIC);
}

void iwl_mvm_nan_de_term_notif(struct iwl_mvm *mvm,
			       struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_nan_de_term *ev = (void *)pkt->data;
	int len = iwl_rx_packet_payload_len(pkt);
	enum nl80211_nan_func_term_reason nl_reason;

	if (WARN_ON_ONCE(!mvm->nan_vif)) {
		IWL_ERR(mvm, "NAN vif is NULL\n");
		return;
	}

	if (WARN_ON_ONCE(len != sizeof(*ev))) {
		IWL_ERR(mvm, "NAN DE termination event bad length: %d\n",
			len);
		return;
	}

	switch (ev->reason) {
	case NAN_DE_TERM_TTL_REACHED:
		nl_reason = NL80211_NAN_FUNC_TERM_REASON_TTL_EXPIRED;
		break;
	case NAN_DE_TERM_USER_REQUEST:
		nl_reason = NL80211_NAN_FUNC_TERM_REASON_USER_REQUEST;
		break;
	case NAN_DE_TERM_FAILURE:
		nl_reason = NL80211_NAN_FUNC_TERM_REASON_ERROR;
		break;
	default:
		WARN_ON_ONCE(1);
		return;
	}

	ieee80211_nan_func_terminated(mvm->nan_vif, ev->instance_id, nl_reason,
				      GFP_ATOMIC);
}

int iwl_mvm_nan_config_nan_faw_cmd(struct iwl_mvm *mvm,
				   struct cfg80211_chan_def *chandef, u8 slots)
{
	struct iwl_nan_faw_config cmd = {};
	struct iwl_mvm_vif *mvmvif;
	int ret;

	if (WARN_ON(!mvm->nan_vif))
		return -EINVAL;

	mutex_lock(&mvm->mutex);

	mvmvif = iwl_mvm_vif_from_mac80211(mvm->nan_vif);

	/* Set the channel info data */
	cmd.faw_ci.band = (chandef->chan->band == IEEE80211_BAND_2GHZ ?
	      PHY_BAND_24 : PHY_BAND_5);

	cmd.faw_ci.channel = chandef->chan->hw_value;
	cmd.faw_ci.width = iwl_mvm_get_channel_width(chandef);
	cmd.faw_ci.ctrl_pos = iwl_mvm_get_ctrl_pos(chandef);
	ieee80211_chandef_to_operating_class(chandef, &cmd.op_class);
	cmd.slots = slots;
	cmd.type = NAN_POST_NAN_ATTR_FURTHER_NAN;
	cmd.id_n_color = cpu_to_le32(FW_CMD_ID_AND_COLOR(mvmvif->id,
							 mvmvif->color));

	ret = iwl_mvm_send_cmd_pdu(mvm, iwl_cmd_id(NAN_FAW_CONFIG_CMD,
						   NAN_GROUP, 0),
				   0, sizeof(cmd), &cmd);

	mutex_unlock(&mvm->mutex);

	return ret;
}
