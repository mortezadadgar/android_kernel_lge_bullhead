/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
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
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
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
#if CFG80211_VERSION > KERNEL_VERSION(3, 14, 0)

#include <linux/etherdevice.h>
#include <net/mac80211.h>
#include <net/netlink.h>
#include "mvm.h"
#include "vendor-cmd.h"
#include "fw-dbg.h"

#include "iwl-io.h"
#include "iwl-prph.h"

static const struct nla_policy
iwl_mvm_vendor_attr_policy[NUM_IWL_MVM_VENDOR_ATTR] = {
	[IWL_MVM_VENDOR_ATTR_LOW_LATENCY] = { .type = NLA_FLAG },
	[IWL_MVM_VENDOR_ATTR_COUNTRY] = { .type = NLA_STRING, .len = 2 },
	[IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA] = { .type = NLA_FLAG },
	[IWL_MVM_VENDOR_ATTR_FILTER_GTK] = { .type = NLA_FLAG },
	[IWL_MVM_VENDOR_ATTR_ADDR] = { .len = ETH_ALEN },
	[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_OPPPS_WA] = { .type = NLA_FLAG },
	[IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR] = { .len = ETH_ALEN },
	[IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK] = { .len = ETH_ALEN },
	[IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS] = { .type = NLA_NESTED },
	[IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE] = { .type = NLA_U8 },
	[IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST] = { .type = NLA_NESTED },
	[IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE] = { .type = NLA_U8 },
	[IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING] = { .type = NLA_U8 },
	[IWL_MVM_VENDOR_ATTR_RXFILTER] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_RXFILTER_OP] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER] = { .type = NLA_STRING },
	[IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ] = { .type = NLA_U32 },
	[IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS] = { .type = NLA_U8 },
};

static int iwl_mvm_parse_vendor_data(struct nlattr **tb,
				     const void *data, int data_len)
{
	if (!data)
		return -EINVAL;

	return nla_parse(tb, MAX_IWL_MVM_VENDOR_ATTR, data, data_len,
			 iwl_mvm_vendor_attr_policy);
}

static int iwl_mvm_set_low_latency(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int err = iwl_mvm_parse_vendor_data(tb, data, data_len);
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	bool prev;

	if (err)
		return err;

	if (!vif)
		return -ENODEV;

	mutex_lock(&mvm->mutex);
	prev = iwl_mvm_vif_low_latency(mvmvif);
	mvmvif->low_latency_vcmd = tb[IWL_MVM_VENDOR_ATTR_LOW_LATENCY];
	err = iwl_mvm_update_low_latency(mvm, vif, prev);
	mutex_unlock(&mvm->mutex);

	return err;
}

static int iwl_mvm_get_low_latency(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data, int data_len)
{
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);
	struct iwl_mvm_vif *mvmvif;
	struct sk_buff *skb;

	if (!vif)
		return -ENODEV;
	mvmvif = iwl_mvm_vif_from_mac80211(vif);

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, 100);
	if (!skb)
		return -ENOMEM;
	if (iwl_mvm_vif_low_latency(mvmvif) &&
	    nla_put_flag(skb, IWL_MVM_VENDOR_ATTR_LOW_LATENCY)) {
		kfree_skb(skb);
		return -ENOBUFS;
	}

	return cfg80211_vendor_cmd_reply(skb);
}

static int iwl_mvm_set_country(struct wiphy *wiphy,
			       struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct ieee80211_regdomain *regd;
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	int retval;

	if (!iwl_mvm_is_lar_supported(mvm))
		return -EOPNOTSUPP;

	retval = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (retval)
		return retval;

	if (!tb[IWL_MVM_VENDOR_ATTR_COUNTRY])
		return -EINVAL;

	mutex_lock(&mvm->mutex);

	/* set regdomain information to FW */
	regd = iwl_mvm_get_regdomain(wiphy,
				     nla_data(tb[IWL_MVM_VENDOR_ATTR_COUNTRY]),
				     iwl_mvm_is_wifi_mcc_supported(mvm) ?
				     MCC_SOURCE_3G_LTE_HOST :
				     MCC_SOURCE_OLD_FW, NULL);
	if (IS_ERR_OR_NULL(regd)) {
		retval = -EIO;
		goto unlock;
	}

	retval = regulatory_set_wiphy_regd(wiphy, regd);
	kfree(regd);
unlock:
	mutex_unlock(&mvm->mutex);
	return retval;
}

static int iwl_vendor_frame_filter_cmd(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int data_len)
{
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);
	int err = iwl_mvm_parse_vendor_data(tb, data, data_len);

	if (err)
		return err;
	if (!vif)
		return -EINVAL;
	vif->filter_grat_arp_unsol_na =
		tb[IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA];
	vif->filter_gtk = tb[IWL_MVM_VENDOR_ATTR_FILTER_GTK];

	return 0;
}

#ifdef CPTCFG_IWLMVM_TDLS_PEER_CACHE
static int iwl_vendor_tdls_peer_cache_add(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int data_len)
{
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct iwl_mvm_tdls_peer_counter *cnt;
	u8 *addr;
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);
	int err = iwl_mvm_parse_vendor_data(tb, data, data_len);

	if (err)
		return err;

	if (vif->type != NL80211_IFTYPE_STATION ||
	    !tb[IWL_MVM_VENDOR_ATTR_ADDR])
		return -EINVAL;

	mutex_lock(&mvm->mutex);
	if (mvm->tdls_peer_cache_cnt >= IWL_MVM_TDLS_CNT_MAX_PEERS) {
		err = -ENOSPC;
		goto out_unlock;
	}

	addr = nla_data(tb[IWL_MVM_VENDOR_ATTR_ADDR]);

	rcu_read_lock();
	cnt = iwl_mvm_tdls_peer_cache_find(mvm, addr);
	rcu_read_unlock();
	if (cnt) {
		err = -EEXIST;
		goto out_unlock;
	}

	cnt = kzalloc(sizeof(*cnt) +
		      sizeof(cnt->rx[0]) * mvm->trans->num_rx_queues,
		      GFP_KERNEL);
	if (!cnt) {
		err = -ENOMEM;
		goto out_unlock;
	}

	IWL_DEBUG_TDLS(mvm, "Adding %pM to TDLS peer cache\n", addr);
	ether_addr_copy(cnt->mac.addr, addr);
	cnt->vif = vif;
	list_add_tail_rcu(&cnt->list, &mvm->tdls_peer_cache_list);
	mvm->tdls_peer_cache_cnt++;

out_unlock:
	mutex_unlock(&mvm->mutex);
	return err;
}

static int iwl_vendor_tdls_peer_cache_del(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int data_len)
{
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct iwl_mvm_tdls_peer_counter *cnt;
	u8 *addr;
	int err = iwl_mvm_parse_vendor_data(tb, data, data_len);

	if (err)
		return err;

	if (!tb[IWL_MVM_VENDOR_ATTR_ADDR])
		return -EINVAL;

	addr = nla_data(tb[IWL_MVM_VENDOR_ATTR_ADDR]);

	mutex_lock(&mvm->mutex);
	rcu_read_lock();
	cnt = iwl_mvm_tdls_peer_cache_find(mvm, addr);
	if (!cnt) {
		IWL_DEBUG_TDLS(mvm, "%pM not found in TDLS peer cache\n", addr);
		err = -ENOENT;
		goto out_unlock;
	}

	IWL_DEBUG_TDLS(mvm, "Removing %pM from TDLS peer cache\n", addr);
	mvm->tdls_peer_cache_cnt--;
	list_del_rcu(&cnt->list);
	kfree_rcu(cnt, rcu_head);

out_unlock:
	rcu_read_unlock();
	mutex_unlock(&mvm->mutex);
	return err;
}

static int iwl_vendor_tdls_peer_cache_query(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    const void *data, int data_len)
{
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct iwl_mvm_tdls_peer_counter *cnt;
	struct sk_buff *skb;
	u32 rx_bytes, tx_bytes;
	u8 *addr;
	int err = iwl_mvm_parse_vendor_data(tb, data, data_len);

	if (err)
		return err;

	if (!tb[IWL_MVM_VENDOR_ATTR_ADDR])
		return -EINVAL;

	addr = nla_data(tb[IWL_MVM_VENDOR_ATTR_ADDR]);

	rcu_read_lock();
	cnt = iwl_mvm_tdls_peer_cache_find(mvm, addr);
	if (!cnt) {
		IWL_DEBUG_TDLS(mvm, "%pM not found in TDLS peer cache\n",
			       addr);
		err = -ENOENT;
	} else {
		int q;

		tx_bytes = cnt->tx_bytes;
		rx_bytes = 0;
		for (q = 0; q < mvm->trans->num_rx_queues; q++)
			rx_bytes += cnt->rx[q].bytes;
	}
	rcu_read_unlock();
	if (err)
		return err;

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, 100);
	if (!skb)
		return -ENOMEM;
	if (nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_TX_BYTES, tx_bytes) ||
	    nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_RX_BYTES, rx_bytes)) {
		kfree_skb(skb);
		return -ENOBUFS;
	}

	return cfg80211_vendor_cmd_reply(skb);
}
#endif /* CPTCFG_IWLMVM_TDLS_PEER_CACHE */

static int iwl_vendor_set_nic_txpower_limit(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct iwl_dev_tx_power_cmd cmd = {
		.v2.set_mode = cpu_to_le32(IWL_TX_POWER_MODE_SET_DEVICE),
		.v2.dev_24 = cpu_to_le16(IWL_DEV_MAX_TX_POWER),
		.v2.dev_52_low = cpu_to_le16(IWL_DEV_MAX_TX_POWER),
		.v2.dev_52_high = cpu_to_le16(IWL_DEV_MAX_TX_POWER),
	};
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int len = sizeof(cmd);
	int err;

	err = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (err)
		return err;

	if (tb[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24]) {
		s32 txp = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24]);

		if (txp < 0 || txp > IWL_DEV_MAX_TX_POWER)
			return -EINVAL;
		cmd.v2.dev_24 = cpu_to_le16(txp);
	}

	if (tb[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L]) {
		s32 txp = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L]);

		if (txp < 0 || txp > IWL_DEV_MAX_TX_POWER)
			return -EINVAL;
		cmd.v2.dev_52_low = cpu_to_le16(txp);
	}

	if (tb[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H]) {
		s32 txp = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H]);

		if (txp < 0 || txp > IWL_DEV_MAX_TX_POWER)
			return -EINVAL;
		cmd.v2.dev_52_high = cpu_to_le16(txp);
	}

	mvm->txp_cmd = cmd;

	if (!fw_has_api(&mvm->fw->ucode_capa, IWL_UCODE_TLV_API_TX_POWER_CHAIN))
		len = sizeof(cmd.v2);

	mutex_lock(&mvm->mutex);
	err = iwl_mvm_send_cmd_pdu(mvm, REDUCE_TX_POWER_CMD, 0, len, &cmd);
	mutex_unlock(&mvm->mutex);

	if (err)
		IWL_ERR(mvm, "failed to update device TX power: %d\n", err);
	return 0;
}

#ifdef CPTCFG_IWLMVM_P2P_OPPPS_TEST_WA
static int iwl_mvm_oppps_wa_update_quota(struct iwl_mvm *mvm,
					 struct ieee80211_vif *vif,
					 bool enable)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	struct ieee80211_p2p_noa_attr *noa = &vif->bss_conf.p2p_noa_attr;
	bool force_update = true;

	if (enable && noa->oppps_ctwindow & IEEE80211_P2P_OPPPS_ENABLE_BIT)
		mvm->p2p_opps_test_wa_vif = mvmvif;
	else
		mvm->p2p_opps_test_wa_vif = NULL;
	return iwl_mvm_update_quotas(mvm, force_update, NULL);
}

static int iwl_mvm_oppps_wa(struct wiphy *wiphy,
			    struct wireless_dev *wdev,
			    const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int err = iwl_mvm_parse_vendor_data(tb, data, data_len);
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);

	if (err)
		return err;

	if (!vif)
		return -ENODEV;

	mutex_lock(&mvm->mutex);
	if (vif->type == NL80211_IFTYPE_STATION && vif->p2p) {
		bool enable = !!tb[IWL_MVM_VENDOR_ATTR_OPPPS_WA];

		err = iwl_mvm_oppps_wa_update_quota(mvm, vif, enable);
	}
	mutex_unlock(&mvm->mutex);

	return err;
}
#endif

static int iwl_vendor_gscan_get_capabilities(struct wiphy *wiphy,
					     struct wireless_dev *wdev,
					     const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	const struct iwl_gscan_capabilities *gscan_capa =
		&mvm->fw->gscan_capa;
	struct sk_buff *skb;

	if (!fw_has_capa(&mvm->fw->ucode_capa,
			 IWL_UCODE_TLV_CAPA_GSCAN_SUPPORT))
		return -EOPNOTSUPP;

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, 100);
	if (!skb)
		return -ENOMEM;

	if (nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_CACHE_SIZE,
			gscan_capa->max_scan_cache_size) ||
	    nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_BUCKETS,
			gscan_capa->max_scan_buckets) ||
	    nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_CACHE_PER_SCAN,
			gscan_capa->max_ap_cache_per_scan) ||
	    nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_GSCAN_MAX_RSSI_SAMPLE_SIZE,
			gscan_capa->max_rssi_sample_size) ||
	    nla_put_u32(skb,
			IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_REPORTING_THRESHOLD,
			gscan_capa->max_scan_reporting_threshold) ||
	    nla_put_u32(skb, IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_APS,
			gscan_capa->max_hotlist_aps) ||
	    nla_put_u32(skb,
			IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SIGNIFICANT_CHANGE_APS,
			gscan_capa->max_significant_change_aps) ||
	    nla_put_u32(skb,
			IWL_MVM_VENDOR_ATTR_GSCAN_MAX_BSSID_HISTORY_ENTRIES,
			gscan_capa->max_bssid_history_entries)) {
		kfree_skb(skb);
		return -ENOBUFS;
	}

	return cfg80211_vendor_cmd_reply(skb);
}

static int iwl_vendor_gscan_parse_channels(struct nlattr *info,
					   struct iwl_gscan_bucket_spec *bucket)
{
	struct nlattr *nl_chan;
	struct nlattr *tb[MAX_IWL_MVM_VENDOR_CHANNEL_SPEC + 1];
	struct iwl_gscan_channel_spec *chans = bucket->channels;
	int rem_chan;
	u8 i = 0;
	static const struct nla_policy
	chan_policy[MAX_IWL_MVM_VENDOR_CHANNEL_SPEC + 1] = {
		[IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL] = { .type = NLA_U8 },
		[IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME] = { .type = NLA_U16 },
		[IWL_MVM_VENDOR_CHANNEL_SPEC_PASSIVE] = { .type = NLA_FLAG },
	};

	nla_for_each_nested(nl_chan, info, rem_chan) {
		if (nla_parse_nested(tb, MAX_IWL_MVM_VENDOR_CHANNEL_SPEC,
				     nl_chan, chan_policy) ||
		    !tb[IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL])
			return -EINVAL;

		chans[i].channel_number =
			nla_get_u8(tb[IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL]);

		if (tb[IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME]) {
			u16 dwell_time =
				nla_get_u16(tb[IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME]);

			chans[i].dwell_time = cpu_to_le16(dwell_time);
		}

		if (tb[IWL_MVM_VENDOR_CHANNEL_SPEC_PASSIVE])
			chans[i].channel_flags |= IWL_GSCAN_CHANNEL_PASSIVE;

		if (++i >= GSCAN_MAX_CHANNELS)
			break;
	}

	bucket->channel_count = i;
	return 0;
}

static int iwl_vendor_gscan_parse_buckets(struct nlattr *info, u32 max_buckets,
					  struct iwl_gscan_start_cmd *cmd)
{
	struct nlattr *nl_bucket;
	struct nlattr *tb[MAX_IWL_MVM_VENDOR_BUCKET_SPEC + 1];
	struct iwl_gscan_bucket_spec *buckets = cmd->buckets;
	int rem_bucket;
	u32 i = 0;
	static const struct nla_policy
	bucket_policy[MAX_IWL_MVM_VENDOR_BUCKET_SPEC + 1] = {
		[IWL_MVM_VENDOR_BUCKET_SPEC_INDEX] = { .type = NLA_U8 },
		[IWL_MVM_VENDOR_BUCKET_SPEC_BAND] = { .type = NLA_U32 },
		[IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD] = {.type = NLA_U32 },
		[IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE] = { .type = NLA_U32 },
		[IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS] = { .type = NLA_NESTED },
	};

	if (!max_buckets)
		return -EINVAL;

	nla_for_each_nested(nl_bucket, info, rem_bucket) {
		u32 tmp;

		if (nla_parse_nested(tb, MAX_IWL_MVM_VENDOR_BUCKET_SPEC,
				     nl_bucket, bucket_policy) ||
		    !tb[IWL_MVM_VENDOR_BUCKET_SPEC_INDEX] ||
		    !tb[IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD] ||
		    !tb[IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE])
			return -EINVAL;

		tmp = nla_get_u32(tb[IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD]);
		buckets[i].scan_interval = cpu_to_le32(tmp);
		tmp = nla_get_u32(tb[IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE]);
		if (tmp >= NUM_IWL_MVM_VENDOR_GSCAN_REPORT)
			return -EINVAL;

		buckets[i].report_policy = cpu_to_le32(tmp);
		buckets[i].index =
			nla_get_u8(tb[IWL_MVM_VENDOR_BUCKET_SPEC_INDEX]);

		/*
		 * If a band is specified, the channel list is not used and all
		 * channels in the specified band will be scanned. Parsing the
		 * channel list is needed only if no band is specified.
		 */
		if (tb[IWL_MVM_VENDOR_BUCKET_SPEC_BAND]) {
			tmp = nla_get_u32(tb[IWL_MVM_VENDOR_BUCKET_SPEC_BAND]);
			if (tmp >= NUM_IWL_GSCAN_BAND)
				return -EINVAL;

			buckets[i].band = cpu_to_le32(tmp);
		} else {
			if (!tb[IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS])
				return -EINVAL;

			buckets[i].band =
				cpu_to_le32(IWL_GSCAN_BAND_UNSPECIFIED);

			if (iwl_vendor_gscan_parse_channels(
					tb[IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS],
					&buckets[i]))
				return -EINVAL;
		}

		if (++i >= max_buckets)
			break;
	}

	cmd->bucket_count = cpu_to_le32(i);
	return 0;
}

static int iwl_vendor_start_gscan(struct wiphy *wiphy,
				  struct wireless_dev *wdev,
				  const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	const struct iwl_gscan_capabilities *capa =
		&mvm->fw->gscan_capa;
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_gscan_start_cmd *cmd = &gscan->scan_params;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(GSCAN_START_CMD, SCAN_GROUP, 0),
		.len = { sizeof(*cmd), },
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
		.data = { cmd, },
	};
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int err;
	u32 tmp;
	u64 boottime;

	if (!fw_has_capa(&mvm->fw->ucode_capa,
			 IWL_UCODE_TLV_CAPA_GSCAN_SUPPORT))
		return -EOPNOTSUPP;

	mutex_lock(&mvm->mutex);
	if (cmd->bucket_count) {
		err = -EBUSY;
		goto unlock;
	}

	err = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (err)
		goto unlock;

	if (!tb[IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN] ||
	    !tb[IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD] ||
	    !tb[IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS]) {
		err = -EINVAL;
		goto unlock;
	}

	if (tb[IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR] &&
	    tb[IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK]) {
		memcpy(cmd->mac_addr_template,
		       nla_data(tb[IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR]),
		       ETH_ALEN);
		memcpy(cmd->mac_addr_mask,
		       nla_data(tb[IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK]),
		       ETH_ALEN);
		cmd->flags = cpu_to_le32(IWL_GSCAN_START_FLAGS_MAC_RANDOMIZE);
	}

	tmp = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN]);
	if (tmp > capa->max_ap_cache_per_scan)
		tmp = capa->max_ap_cache_per_scan;

	cmd->max_scan_aps = cpu_to_le32(tmp);

	tmp = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD]);
	if (tmp > capa->max_scan_reporting_threshold)
		tmp = capa->max_scan_reporting_threshold;

	cmd->report_threshold = cpu_to_le32(tmp);

	err = iwl_vendor_gscan_parse_buckets(tb[IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS],
					     capa->max_scan_buckets, cmd);
	if (err)
		goto unlock;

	err = iwl_mvm_send_cmd(mvm, &hcmd);
	if (err) {
		IWL_ERR(mvm, "Failed to send gscan start command: %d\n", err);
		goto unlock;
	}

	gscan->wdev = wdev;

	iwl_mvm_get_sync_time(mvm, &gscan->gp2, &boottime);
	gscan->timestamp = DIV_ROUND_CLOSEST_ULL(boottime, NSEC_PER_USEC);

unlock:
	mutex_unlock(&mvm->mutex);
	return err;
}

static void iwl_mvm_gscan_clear(struct iwl_mvm *mvm)
{
	struct iwl_mvm_gscan_beacon *beacon, *tmp;

	spin_lock_bh(&mvm->gscan_beacons_lock);
	list_for_each_entry_safe(beacon, tmp, &mvm->gscan_beacons_list, list) {
		list_del(&beacon->list);
		kfree(beacon);
	}
	spin_unlock_bh(&mvm->gscan_beacons_lock);

	cancel_work_sync(&mvm->gscan_beacons_work);
}

int iwl_mvm_vendor_stop_gscan(struct wiphy *wiphy, struct wireless_dev *wdev,
			      const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_host_cmd cmd = {
		.id = iwl_cmd_id(GSCAN_STOP_CMD, SCAN_GROUP, 0),
	};
	int ret;

	if (!fw_has_capa(&mvm->fw->ucode_capa,
			 IWL_UCODE_TLV_CAPA_GSCAN_SUPPORT))
		return -EOPNOTSUPP;

	mutex_lock(&mvm->mutex);
	if (gscan->wdev != wdev) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = iwl_mvm_send_cmd(mvm, &cmd);
	if (ret) {
		IWL_ERR(mvm, "Failed to send gscan stop command: %d\n", ret);
		goto unlock;
	}

	memset(&gscan->scan_params, 0, sizeof(struct iwl_gscan_start_cmd));
	if (!gscan->hotlist_params.num_ap && !gscan->sc_params.num_ap)
		gscan->wdev = NULL;

	iwl_mvm_gscan_clear(mvm);

unlock:
	mutex_unlock(&mvm->mutex);
	return ret;
}

static u8
iwl_vendor_parse_ap_list(struct nlattr *info, u8 max,
			 struct iwl_gscan_ap_threshold_params *ap_list)
{
	struct nlattr *nl_ap;
	struct nlattr *tb[MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM + 1];
	int rem_ap;
	u8 i = 0;
	static const struct nla_policy
	ap_policy[MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM + 1] = {
		[IWL_MVM_VENDOR_AP_BSSID] = { .len = ETH_ALEN },
		[IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD] = { .type = NLA_S8 },
		[IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD] = { .type = NLA_S8 },
		[IWL_MVM_VENDOR_AP_CHANNEL_HINT] = { .type = NLA_U8 },
	};

	nla_for_each_nested(nl_ap, info, rem_ap) {
		if (i >= max)
			return 0;

		if (nla_parse_nested(tb,
				     MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM,
				     nl_ap, ap_policy) ||
		    !tb[IWL_MVM_VENDOR_AP_BSSID] ||
		    !tb[IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD] ||
		    !tb[IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD])
			return 0;

		memcpy(ap_list[i].bssid, nla_data(tb[IWL_MVM_VENDOR_AP_BSSID]),
		       ETH_ALEN);

		/* FW expects positive RSSI values, need to negate the real
		 * values */
		ap_list[i].low_threshold =
			-nla_get_s8(tb[IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD]);
		ap_list[i].high_threshold =
			-nla_get_s8(tb[IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD]);

		if (tb[IWL_MVM_VENDOR_AP_CHANNEL_HINT])
			ap_list[i].channel =
				nla_get_u8(tb[IWL_MVM_VENDOR_AP_CHANNEL_HINT]);

		i++;
	}
	return i;
}

static int iwl_mvm_vendor_send_set_hotlist_cmd(struct iwl_mvm *mvm,
					       struct wireless_dev *wdev,
					       struct nlattr *tb[])
{
	struct gscan_data *gscan = &mvm->gscan;
	const struct iwl_gscan_capabilities *capa = &mvm->fw->gscan_capa;
	struct iwl_gscan_bssid_hotlist_cmd *cmd = &gscan->hotlist_params;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(GSCAN_SET_HOTLIST_CMD, SCAN_GROUP, 0),
		.len = { sizeof(*cmd), },
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
		.data = { cmd, },
	};
	int ret;
	u32 max_aps;

	if (!tb[IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE] ||
	    !tb[IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST])
		return -EINVAL;

	mutex_lock(&mvm->mutex);
	if (gscan->wdev && gscan->wdev != wdev) {
		ret = -EINVAL;
		goto unlock;
	}

	cmd->lost_ap_sample_size =
		nla_get_u8(tb[IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE]);

	max_aps = min_t(u32, capa->max_hotlist_aps, ARRAY_SIZE(cmd->ap_list));
	cmd->num_ap =
		iwl_vendor_parse_ap_list(tb[IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST],
					 max_aps, cmd->ap_list);
	if (!cmd->num_ap) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = iwl_mvm_send_cmd(mvm, &hcmd);
	if (ret) {
		IWL_ERR(mvm,
			"Failed to send set bssid hotlist command: %d\n", ret);
		goto unlock;
	}

	gscan->wdev = wdev;

unlock:
	mutex_unlock(&mvm->mutex);
	return ret;
}

int iwl_mvm_vendor_send_reset_hotlist_cmd(struct iwl_mvm *mvm,
					  struct wireless_dev *wdev)
{
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(GSCAN_RESET_HOTLIST_CMD, SCAN_GROUP, 0),
	};
	int ret;

	mutex_lock(&mvm->mutex);
	if (gscan->wdev != wdev) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = iwl_mvm_send_cmd(mvm, &hcmd);
	if (ret) {
		IWL_ERR(mvm,
			"Failed to send reset bssid hotlist command: %d\n",
			ret);
		goto unlock;
	}

	memset(&gscan->hotlist_params, 0, sizeof(gscan->hotlist_params));
	if (!gscan->scan_params.bucket_count && !gscan->sc_params.num_ap)
		gscan->wdev = NULL;

unlock:
	mutex_unlock(&mvm->mutex);
	return ret;
}

static int iwl_vendor_gscan_set_bssid_hotlist(struct wiphy *wiphy,
					      struct wireless_dev *wdev,
					      const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int ret;

	if (!fw_has_capa(&mvm->fw->ucode_capa,
			 IWL_UCODE_TLV_CAPA_GSCAN_SUPPORT))
		return -EOPNOTSUPP;

	/* if no AP is configured, reset the bssid hotlist */
	if (!data)
		return iwl_mvm_vendor_send_reset_hotlist_cmd(mvm, wdev);

	ret = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (ret)
		return ret;

	return iwl_mvm_vendor_send_set_hotlist_cmd(mvm, wdev, tb);
}

static int
iwl_mvm_vendor_send_set_sig_change_cmd(struct iwl_mvm *mvm,
				       struct wireless_dev *wdev,
				       struct nlattr *tb[])
{
	const struct iwl_gscan_capabilities *capa = &mvm->fw->gscan_capa;
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_gscan_significant_change_cmd *cmd = &gscan->sc_params;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(GSCAN_SET_SIGNIFICANT_CHANGE_CMD,
				 SCAN_GROUP, 0),
		.len = { sizeof(*cmd), },
		.data = { cmd, },
	};
	int ret;
	u32 max_aps;

	if (!tb[IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE] ||
	    !tb[IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE] ||
	    !tb[IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING] ||
	    !tb[IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST])
		return -EINVAL;

	mutex_lock(&mvm->mutex);
	if (gscan->wdev && gscan->wdev != wdev) {
		ret = -EINVAL;
		goto unlock;
	}

	cmd->lost_ap_sample_size =
		nla_get_u8(tb[IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE]);
	cmd->rssi_sample_size =
		nla_get_u8(tb[IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE]);
	if (cmd->rssi_sample_size > capa->max_rssi_sample_size)
		cmd->rssi_sample_size = capa->max_rssi_sample_size;

	cmd->min_breaching =
		nla_get_u8(tb[IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING]);

	max_aps = min_t(u32, capa->max_significant_change_aps,
		        ARRAY_SIZE(cmd->ap_list));
	cmd->num_ap =
		iwl_vendor_parse_ap_list(tb[IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST],
					 max_aps, cmd->ap_list);
	if (!cmd->num_ap) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = iwl_mvm_send_cmd(mvm, &hcmd);

	if (ret) {
		IWL_ERR(mvm,
			"Failed to send gscan set significant change command: %d\n",
			ret);
		goto unlock;
	}

	gscan->wdev = wdev;

unlock:
	mutex_unlock(&mvm->mutex);
	return ret;
}

int iwl_mvm_vendor_send_reset_sig_change_cmd(struct iwl_mvm *mvm,
					     struct wireless_dev *wdev)
{
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(GSCAN_RESET_SIGNIFICANT_CHANGE_CMD,
				 SCAN_GROUP, 0),
	};
	int ret;

	mutex_lock(&mvm->mutex);
	if (gscan->wdev != wdev) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = iwl_mvm_send_cmd(mvm, &hcmd);
	if (ret) {
		IWL_ERR(mvm,
			"Failed to send reset significant change command: %d\n",
			ret);
		goto unlock;
	}

	memset(&gscan->sc_params, 0, sizeof(gscan->sc_params));
	if (!gscan->scan_params.bucket_count && !gscan->hotlist_params.num_ap)
		gscan->wdev = NULL;

unlock:
	mutex_unlock(&mvm->mutex);
	return ret;
}

static int
iwl_vendor_gscan_set_significant_change_list(struct wiphy *wiphy,
					     struct wireless_dev *wdev,
					     const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int ret;

	if (!fw_has_capa(&mvm->fw->ucode_capa,
			 IWL_UCODE_TLV_CAPA_GSCAN_SUPPORT))
		return -EOPNOTSUPP;

	/* If no AP is configured, this is a reset significant change command */
	if (!data)
		return iwl_mvm_vendor_send_reset_sig_change_cmd(mvm, wdev);

	ret = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (ret)
		return ret;

	return iwl_mvm_vendor_send_set_sig_change_cmd(mvm, wdev, tb);
}

void iwl_mvm_gscan_reconfig(struct iwl_mvm *mvm)
{
	struct gscan_data *gscan = &mvm->gscan;
	int err;
	struct iwl_host_cmd hcmd = {
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
	};

	/*
	 * Wait for pending gscan events before restarting gscan so
	 * the timestamp for these events is calculated correctly.
	 */
	iwl_mvm_wait_for_async_handlers(mvm);

	mutex_lock(&mvm->mutex);

	if (!fw_has_capa(&mvm->fw->ucode_capa,
			 IWL_UCODE_TLV_CAPA_GSCAN_SUPPORT))
		goto out;

	flush_work(&mvm->gscan_beacons_work);

	if (gscan->scan_params.bucket_count) {
		hcmd.id = iwl_cmd_id(GSCAN_START_CMD, SCAN_GROUP, 0);
		hcmd.len[0] = sizeof(gscan->scan_params);
		hcmd.data[0] = &gscan->scan_params;
		err = iwl_mvm_send_cmd(mvm, &hcmd);
		if (err)
			IWL_ERR(mvm,
				"Failed to send gscan start command: %d\n",
				err);
	}

	if (gscan->hotlist_params.num_ap) {
		hcmd.id = iwl_cmd_id(GSCAN_SET_HOTLIST_CMD, SCAN_GROUP, 0);
		hcmd.len[0] = sizeof(gscan->hotlist_params);
		hcmd.data[0] = &gscan->hotlist_params;
		err = iwl_mvm_send_cmd(mvm, &hcmd);
		if (err)
			IWL_ERR(mvm,
				"Failed to send bssid hotlist set command: %d\n",
				err);
	}

	if (gscan->sc_params.num_ap) {
		err = iwl_mvm_send_cmd_pdu(mvm,
					   iwl_cmd_id(GSCAN_SET_SIGNIFICANT_CHANGE_CMD,
						      SCAN_GROUP, 0), 0,
					   sizeof(gscan->sc_params),
					   &gscan->sc_params);
		if (err)
			IWL_ERR(mvm,
				"Failed to send bssid hotlist set command: %d\n",
				err);
	}
out:
	mutex_unlock(&mvm->mutex);
}

void iwl_mvm_active_rx_filters(struct iwl_mvm *mvm)
{
	int i, len, total = 0;
	struct iwl_mcast_filter_cmd *cmd;
	static const u8 ipv4mc[] = {0x01, 0x00, 0x5e};
	static const u8 ipv6mc[] = {0x33, 0x33};
	static const u8 ipv4_mdns[] = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb};
	static const u8 ipv6_mdns[] = {0x33, 0x33, 0x00, 0x00, 0x00, 0xfb};

	lockdep_assert_held(&mvm->mutex);

	if (mvm->rx_filters & IWL_MVM_VENDOR_RXFILTER_EINVAL)
		return;

	for (i = 0; i < mvm->mcast_filter_cmd->count; i++) {
		if (mvm->rx_filters & IWL_MVM_VENDOR_RXFILTER_MCAST4 &&
		    memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
			   ipv4mc, sizeof(ipv4mc)) == 0)
			total++;
		else if (memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
				ipv4_mdns, sizeof(ipv4_mdns)) == 0)
			total++;
		else if (mvm->rx_filters & IWL_MVM_VENDOR_RXFILTER_MCAST6 &&
			 memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
				ipv6mc, sizeof(ipv6mc)) == 0)
			total++;
		else if (memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
				ipv6_mdns, sizeof(ipv6_mdns)) == 0)
			total++;
	}

	/* FW expects full words */
	len = roundup(sizeof(*cmd) + total * ETH_ALEN, 4);
	cmd = kzalloc(len, GFP_KERNEL);
	if (!cmd)
		return;

	memcpy(cmd, mvm->mcast_filter_cmd, sizeof(*cmd));
	cmd->count = 0;

	for (i = 0; i < mvm->mcast_filter_cmd->count; i++) {
		bool copy_filter = false;

		if (mvm->rx_filters & IWL_MVM_VENDOR_RXFILTER_MCAST4 &&
		    memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
			   ipv4mc, sizeof(ipv4mc)) == 0)
			copy_filter = true;
		else if (memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
				ipv4_mdns, sizeof(ipv4_mdns)) == 0)
			copy_filter = true;
		else if (mvm->rx_filters & IWL_MVM_VENDOR_RXFILTER_MCAST6 &&
			 memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
				ipv6mc, sizeof(ipv6mc)) == 0)
			copy_filter = true;
		else if (memcmp(&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN],
				ipv6_mdns, sizeof(ipv6_mdns)) == 0)
			copy_filter = true;

		if (!copy_filter)
			continue;

		ether_addr_copy(&cmd->addr_list[cmd->count * ETH_ALEN],
				&mvm->mcast_filter_cmd->addr_list[i * ETH_ALEN]);
		cmd->count++;
	}

	kfree(mvm->mcast_active_filter_cmd);
	mvm->mcast_active_filter_cmd = cmd;
}

static int iwl_mvm_vendor_rxfilter(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data, int data_len)
{
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	enum iwl_mvm_vendor_rxfilter_flags filter, rx_filters, old_rx_filters;
	enum iwl_mvm_vendor_rxfilter_op op;
	bool first_set;
	u32 mask;
	int retval;

	retval = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (retval)
		return retval;

	if (!tb[IWL_MVM_VENDOR_ATTR_RXFILTER])
		return -EINVAL;

	if (!tb[IWL_MVM_VENDOR_ATTR_RXFILTER_OP])
		return -EINVAL;

	filter = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_RXFILTER]);
	op = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_RXFILTER_OP]);

	if (filter != IWL_MVM_VENDOR_RXFILTER_UNICAST &&
	    filter != IWL_MVM_VENDOR_RXFILTER_BCAST &&
	    filter != IWL_MVM_VENDOR_RXFILTER_MCAST4 &&
	    filter != IWL_MVM_VENDOR_RXFILTER_MCAST6)
		return -EINVAL;

	rx_filters = mvm->rx_filters & ~IWL_MVM_VENDOR_RXFILTER_EINVAL;
	switch (op) {
	case IWL_MVM_VENDOR_RXFILTER_OP_DROP:
		rx_filters &= ~filter;
		break;
	case IWL_MVM_VENDOR_RXFILTER_OP_PASS:
		rx_filters |= filter;
		break;
	default:
		return -EINVAL;
	}

	first_set = mvm->rx_filters & IWL_MVM_VENDOR_RXFILTER_EINVAL;

	/* If first time set - clear EINVAL value */
	mvm->rx_filters &= ~IWL_MVM_VENDOR_RXFILTER_EINVAL;

	if (rx_filters == mvm->rx_filters && !first_set)
		return 0;

	mutex_lock(&mvm->mutex);

	old_rx_filters = mvm->rx_filters;
	mvm->rx_filters = rx_filters;

	mask = IWL_MVM_VENDOR_RXFILTER_MCAST4 | IWL_MVM_VENDOR_RXFILTER_MCAST6;
	if ((old_rx_filters & mask) != (rx_filters & mask) || first_set) {
		iwl_mvm_active_rx_filters(mvm);
		iwl_mvm_recalc_multicast(mvm);
	}

	mask = IWL_MVM_VENDOR_RXFILTER_BCAST;
	if ((old_rx_filters & mask) != (rx_filters & mask) || first_set)
		iwl_mvm_configure_bcast_filter(mvm);

	mutex_unlock(&mvm->mutex);

	return 0;
}

static int iwl_mvm_vendor_dbg_collect(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	int err, len = 0;
	const char *trigger_desc;

	err = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (err)
		return err;

	if (!tb[IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER])
		return -EINVAL;

	trigger_desc = nla_data(tb[IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER]);
	len = nla_len(tb[IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER]);

	iwl_mvm_fw_dbg_collect(mvm, FW_DBG_TRIGGER_USER_EXTENDED, trigger_desc,
			       len, NULL);

	return 0;
}

static int iwl_mvm_vendor_nan_faw_conf(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int data_len)
{
	struct nlattr *tb[NUM_IWL_MVM_VENDOR_ATTR];
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct iwl_mvm *mvm = IWL_MAC80211_GET_MVM(hw);
	struct cfg80211_chan_def def = {};
	struct ieee80211_channel *chan;
	u32 freq;
	u8 slots;
	int retval;

	retval = iwl_mvm_parse_vendor_data(tb, data, data_len);
	if (retval)
		return retval;

	if (!tb[IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS])
		return -EINVAL;

	if (!tb[IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ])
		return -EINVAL;

	freq = nla_get_u32(tb[IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ]);
	slots = nla_get_u8(tb[IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS]);

	chan = ieee80211_get_channel(wiphy, freq);
	if (!chan)
		return -EINVAL;

	cfg80211_chandef_create(&def, chan, NL80211_CHAN_NO_HT);

	if (!cfg80211_chandef_usable(wiphy, &def, IEEE80211_CHAN_DISABLED))
		return -EINVAL;

	return iwl_mvm_nan_config_nan_faw_cmd(mvm, &def, slots);
}

static const struct wiphy_vendor_command iwl_mvm_vendor_commands[] = {
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_set_low_latency,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_get_low_latency,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_SET_COUNTRY,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_set_country,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_PROXY_FRAME_FILTERING,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_frame_filter_cmd,
	},
#ifdef CPTCFG_IWLMVM_TDLS_PEER_CACHE
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_ADD,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_tdls_peer_cache_add,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_DEL,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_tdls_peer_cache_del,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_QUERY,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_tdls_peer_cache_query,
	},
#endif /* CPTCFG_IWLMVM_TDLS_PEER_CACHE */
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_SET_NIC_TXPOWER_LIMIT,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_set_nic_txpower_limit,
	},
#ifdef CPTCFG_IWLMVM_P2P_OPPPS_TEST_WA
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_OPPPS_WA,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_oppps_wa,
	},
#endif
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_GET_CAPABILITIES,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_gscan_get_capabilities,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_START,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_start_gscan,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_STOP,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_vendor_stop_gscan,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_SET_BSSID_HOTLIST,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_gscan_set_bssid_hotlist,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd =
				IWL_MVM_VENDOR_CMD_GSCAN_SET_SIGNIFICANT_CHANGE,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_vendor_gscan_set_significant_change_list,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_RXFILTER,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_vendor_rxfilter,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,
			.subcmd = IWL_MVM_VENDOR_CMD_DBG_COLLECT,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_vendor_dbg_collect,
	},
	{
		.info = {
			.vendor_id = INTEL_OUI,

			.subcmd = IWL_MVM_VENDOR_CMD_NAN_FAW_CONF,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = iwl_mvm_vendor_nan_faw_conf,
	},
};

enum iwl_mvm_vendor_events_idx {
#ifdef CPTCFG_IWLMVM_TCM
	IWL_MVM_VENDOR_EVENT_IDX_TCM,
#endif
	IWL_MVM_VENDOR_EVENT_IDX_GSCAN_RESULTS,
	IWL_MVM_VENDOR_EVENT_IDX_HOTLIST_CHANGE,
	IWL_MVM_VENDOR_EVENT_IDX_SIGNIFICANT_CHANGE,
	IWL_MVM_VENDOR_EVENT_IDX_GSCAN_BEACON,
	NUM_IWL_MVM_VENDOR_EVENT_IDX
};

static const struct nl80211_vendor_cmd_info
iwl_mvm_vendor_events[NUM_IWL_MVM_VENDOR_EVENT_IDX] = {
#ifdef CPTCFG_IWLMVM_TCM
	[IWL_MVM_VENDOR_EVENT_IDX_TCM] = {
		.vendor_id = INTEL_OUI,
		.subcmd = IWL_MVM_VENDOR_CMD_TCM_EVENT,
	},
#endif
	[IWL_MVM_VENDOR_EVENT_IDX_GSCAN_RESULTS] = {
		.vendor_id = INTEL_OUI,
		.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_RESULTS_EVENT,
	},
	[IWL_MVM_VENDOR_EVENT_IDX_HOTLIST_CHANGE] = {
		.vendor_id = INTEL_OUI,
		.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_HOTLIST_CHANGE_EVENT,
	},
	[IWL_MVM_VENDOR_EVENT_IDX_SIGNIFICANT_CHANGE] = {
		.vendor_id = INTEL_OUI,
		.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_SIGNIFICANT_CHANGE_EVENT,
	},
	[IWL_MVM_VENDOR_EVENT_IDX_GSCAN_BEACON] = {
		.vendor_id = INTEL_OUI,
		.subcmd = IWL_MVM_VENDOR_CMD_GSCAN_BEACON_EVENT,
	},
};

void iwl_mvm_set_wiphy_vendor_commands(struct wiphy *wiphy)
{
	wiphy->vendor_commands = iwl_mvm_vendor_commands;
	wiphy->n_vendor_commands = ARRAY_SIZE(iwl_mvm_vendor_commands);
	wiphy->vendor_events = iwl_mvm_vendor_events;
	wiphy->n_vendor_events = ARRAY_SIZE(iwl_mvm_vendor_events);
}

#ifdef CPTCFG_IWLMVM_TCM
void iwl_mvm_send_tcm_event(struct iwl_mvm *mvm, struct ieee80211_vif *vif)
{
	struct sk_buff *msg =
		cfg80211_vendor_event_alloc(mvm->hw->wiphy,
					    ieee80211_vif_to_wdev(vif),
					    200, IWL_MVM_VENDOR_EVENT_IDX_TCM,
					    GFP_ATOMIC);

	if (!msg)
		return;

	if (vif) {
		struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);

		if (nla_put(msg, IWL_MVM_VENDOR_ATTR_VIF_ADDR,
			    ETH_ALEN, vif->addr) ||
		    nla_put_u8(msg, IWL_MVM_VENDOR_ATTR_VIF_LL,
			       iwl_mvm_vif_low_latency(mvmvif)) ||
		    nla_put_u8(msg, IWL_MVM_VENDOR_ATTR_VIF_LOAD,
			       mvm->tcm.result.load[mvmvif->id]))
			goto nla_put_failure;
	}

	if (nla_put_u8(msg, IWL_MVM_VENDOR_ATTR_LL, iwl_mvm_low_latency(mvm)) ||
	    nla_put_u8(msg, IWL_MVM_VENDOR_ATTR_LOAD,
		       mvm->tcm.result.global_load))
		goto nla_put_failure;

	cfg80211_vendor_event(msg, GFP_ATOMIC);
	return;

 nla_put_failure:
	kfree_skb(msg);
}
#endif

static int iwl_vendor_put_one_result(struct sk_buff *skb,
				     struct gscan_data *gscan,
				     struct iwl_gscan_scan_result *res)
{
	u32 gp2_ts, diff;
	u64 ts;

	/*
	 * Convert the timestamp of the result to boottime time,
	 * which is what gscan expects.
	 */
	gp2_ts = le32_to_cpu(res->timestamp);
	diff = gp2_ts - gscan->gp2;
	ts = gscan->timestamp + diff;

	/* FW sends RSSI as absolute values, so negate here to get dB values */
	if (nla_put_s8(skb, IWL_MVM_VENDOR_GSCAN_RESULT_RSSI, -res->rssi) ||
	    nla_put_u64(skb, IWL_MVM_VENDOR_GSCAN_RESULT_TIMESTAMP, ts) ||
	    nla_put_u8(skb, IWL_MVM_VENDOR_GSCAN_RESULT_CHANNEL,
		       res->channel) ||
	    nla_put(skb, IWL_MVM_VENDOR_GSCAN_RESULT_BSSID, ETH_ALEN,
		    res->bssid) ||
	    nla_put(skb, IWL_MVM_VENDOR_GSCAN_RESULT_SSID,
		    IEEE80211_MAX_SSID_LEN, res->ssid))
		return -ENOBUFS;

	return 0;
}

void iwl_mvm_rx_gscan_results_available(struct iwl_mvm *mvm,
					struct iwl_rx_cmd_buffer *rxb)
{
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_gscan_results_event *event = (void *)pkt->data;
	enum iwl_mvm_vendor_results_event_type event_type;
	struct sk_buff *msg;
	struct nlattr *results;
	u32 num_res = le32_to_cpu(event->num_res);
	int i, start_idx = 0;

	lockdep_assert_held(&mvm->mutex);

	if (WARN_ON(!gscan->wdev))
		return;

	while (start_idx < num_res) {
		msg = cfg80211_vendor_event_alloc(mvm->hw->wiphy, gscan->wdev,
						  3500,
						  IWL_MVM_VENDOR_EVENT_IDX_GSCAN_RESULTS,
						  GFP_KERNEL);
		if (!msg)
			return;

		event_type = le32_to_cpu(event->event_type);
		if (event_type >= NUM_IWL_VENDOR_RESULTS_NOTIF_EVENT_TYPE ||
		    nla_put_u32(msg,
				IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS_EVENT_TYPE,
				event_type)) {
			kfree_skb(msg);
			return;
		}

		results = nla_nest_start(msg,
					 IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS);
		if (!results) {
			kfree_skb(msg);
			return;
		}

		for (i = start_idx; i < num_res; i++) {
			struct nlattr *result = nla_nest_start(msg, i + 1);

			if (!result)
				break;

			if (iwl_vendor_put_one_result(msg, gscan,
						      &event->results[i])) {
				nla_nest_cancel(msg, result);
				break;
			}

			nla_nest_end(msg, result);
		}

		nla_nest_end(msg, results);

		start_idx = i;
		cfg80211_vendor_event(msg, GFP_KERNEL);
	}
}

static int iwl_vendor_put_hotlist_results(struct sk_buff *msg,
					  struct gscan_data *gscan, u32 num_res,
					  struct iwl_gscan_scan_result *results)
{
	struct nlattr *res;
	u32 i;

	res = nla_nest_start(msg, IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS);
	if (!res)
		return -ENOBUFS;

	for (i = 0; i < num_res; i++) {
		struct nlattr *result = nla_nest_start(msg, i + 1);

		if (!result ||
		    iwl_vendor_put_one_result(msg, gscan, &results[i]))
			return -ENOBUFS;

		nla_nest_end(msg, result);
	}

	nla_nest_end(msg, res);
	return 0;
}

void iwl_mvm_rx_gscan_hotlist_change_event(struct iwl_mvm *mvm,
					   struct iwl_rx_cmd_buffer *rxb)
{
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_gscan_hotlist_change_event *event = (void *)pkt->data;
	enum iwl_mvm_vendor_hotlist_ap_status ap_status;
	struct sk_buff *msg;
	u32 num_res = le32_to_cpu(event->num_res);

	lockdep_assert_held(&mvm->mutex);

	if (WARN_ON(!gscan->wdev))
		return;

	msg = cfg80211_vendor_event_alloc(mvm->hw->wiphy, gscan->wdev, 1024,
					  IWL_MVM_VENDOR_EVENT_IDX_HOTLIST_CHANGE,
					  GFP_KERNEL);
	if (!msg)
		return;

	ap_status = le32_to_cpu(event->status);
	if (ap_status >= NUM_IWL_MVM_VENDOR_HOTLIST_AP_STATUS ||
	    nla_put_u32(msg, IWL_MVM_VENDOR_ATTR_GSCAN_HOTLIST_AP_STATUS,
			ap_status) ||
	    iwl_vendor_put_hotlist_results(msg, gscan, num_res,
					   event->results)) {
		kfree_skb(msg);
		return;
	}

	cfg80211_vendor_event(msg, GFP_KERNEL);
}

static int iwl_mvm_put_ap(struct sk_buff *msg,
			  struct iwl_gscan_significant_change_result *res)
{
	struct nlattr *rssi;
	u32 i;

	if (res->num_rssi > MAX_RSSI_SAMPLE_SIZE)
		return -EINVAL;

	if (nla_put_u8(msg, IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_CHANNEL,
		       res->channel) ||
	    nla_put(msg, IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_BSSID, ETH_ALEN,
		    res->bssid))
		return -ENOBUFS;

	if (!res->num_rssi)
		return 0;

	rssi = nla_nest_start(msg,
			      IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RSSI_HISTORY);
	if (!rssi)
		return -ENOBUFS;

	/* FW uses positive values for RSSI, need to negate it to get real
	 * values */
	for (i = 0; i < res->num_rssi; i++) {
		if (nla_put_s8(msg, i + 1, -res->rssi_history[i]))
			return -ENOBUFS;
	}

	nla_nest_end(msg, rssi);

	return 0;
}

static int iwl_mvm_put_aps(struct sk_buff *msg, u32 num_aps,
			   struct iwl_gscan_significant_change_result *res)
{
	u32 i;
	struct nlattr *aps;

	aps = nla_nest_start(msg, IWL_MVM_VENDOR_ATTR_GSCAN_SIG_CHANGE_RESULTS);
	if (!aps)
		return -ENOBUFS;

	for (i = 0; i < num_aps; i++) {
		struct nlattr *result = nla_nest_start(msg, i + 1);

		if (!result || iwl_mvm_put_ap(msg, &res[i]))
			return -ENOBUFS;

		nla_nest_end(msg, result);
	}

	nla_nest_end(msg, aps);
	return 0;
}

void iwl_mvm_rx_gscan_significant_change_event(struct iwl_mvm *mvm,
					       struct iwl_rx_cmd_buffer *rxb)
{
	struct gscan_data *gscan = &mvm->gscan;
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_gscan_significant_change_event *event = (void *)pkt->data;
	struct sk_buff *msg;
	u32 num_aps = le32_to_cpu(event->num_aps);

	lockdep_assert_held(&mvm->mutex);

	if (WARN_ON(!gscan->wdev))
		return;

	msg = cfg80211_vendor_event_alloc(mvm->hw->wiphy, gscan->wdev, 1024,
					  IWL_MVM_VENDOR_EVENT_IDX_SIGNIFICANT_CHANGE,
					  GFP_KERNEL);
	if (!msg)
		return;

	if (iwl_mvm_put_aps(msg, num_aps, event->results)) {
		kfree_skb(msg);
		return;
	}

	cfg80211_vendor_event(msg, GFP_KERNEL);
}

static void iwl_mvm_send_gscan_beacon(struct iwl_mvm *mvm,
				      struct iwl_mvm_gscan_beacon *beacon)
{
	struct ieee80211_mgmt *mgmt = beacon->mgmt;
	struct gscan_data *gscan = &mvm->gscan;
	struct sk_buff *msg;
	struct nlattr *res;
	u32 diff;
	u64 timestamp;

	msg = cfg80211_vendor_event_alloc(mvm->hw->wiphy, mvm->gscan.wdev,
					  100 + beacon->len,
					  IWL_MVM_VENDOR_EVENT_IDX_GSCAN_BEACON,
					  GFP_ATOMIC);
	if (!msg)
		return;

	res = nla_nest_start(msg, IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS);
	if (!res) {
		kfree_skb(msg);
		return;
	}

	diff = beacon->gp2_ts - gscan->gp2;
	timestamp = gscan->timestamp + diff;

	if (nla_put_u64(msg, IWL_MVM_VENDOR_GSCAN_RESULT_TIMESTAMP,
			timestamp) ||
	    nla_put_s8(msg, IWL_MVM_VENDOR_GSCAN_RESULT_RSSI, beacon->signal) ||
	    nla_put_u8(msg, IWL_MVM_VENDOR_GSCAN_RESULT_CHANNEL,
		       beacon->channel) ||
	    nla_put(msg, IWL_MVM_VENDOR_GSCAN_RESULT_FRAME, beacon->len,
		    mgmt)) {
		kfree_skb(msg);
		return;
	}

	nla_nest_end(msg, res);
	cfg80211_vendor_event(msg, GFP_ATOMIC);
}

void iwl_mvm_gscan_beacons_work(struct work_struct *work)
{
	struct iwl_mvm_gscan_beacon *beacon, *tmp;
	struct iwl_mvm *mvm = container_of(work, struct iwl_mvm,
					   gscan_beacons_work);

	spin_lock_bh(&mvm->gscan_beacons_lock);
	list_for_each_entry_safe(beacon, tmp, &mvm->gscan_beacons_list, list) {
		iwl_mvm_send_gscan_beacon(mvm, beacon);
		list_del(&beacon->list);
		kfree(beacon);
	}
	spin_unlock_bh(&mvm->gscan_beacons_lock);
}
#endif
