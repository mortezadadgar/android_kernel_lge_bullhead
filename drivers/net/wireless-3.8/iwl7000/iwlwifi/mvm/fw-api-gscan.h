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
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
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

#ifndef __fw_gscan_api_h__
#define __fw_gscan_api_h__

/**
 * enum iwl_gscan_channel_flags - channel flags
 * @IWL_GSCAN_CHANNEL_PASSIVE: do only passive scan on this channel.
 */
enum iwl_gscan_channel_flags {
	IWL_GSCAN_CHANNEL_PASSIVE = BIT(0),
};

/**
 * struct iwl_gscan_channel_spec - gscan channel specification
 * @dwell_time: dwell time hint.
 * @channel_number: the channel number.
 * @channel_flags: bitmap - &enum iwl_gscan_channel_flags
 */
struct iwl_gscan_channel_spec {
	__le16 dwell_time;
	u8 channel_number;
	u8 channel_flags;
} __packed; /* GSCAN_CHANNEL_API_S_VER_1 */

/**
 * enum iwl_gscan_band - band specification
 * @IWL_GSCAN_BAND_UNSPECIFIED: unspecified
 * @IWL_GSCAN_BAND_BG: 2.4 GHz
 * @IWL_GSCAN_BAND_A: 5 GHz without DFS
 * @IWL_GSCAN_BAND_A_DFS: 5 GHz DFS only
 * @IWL_GSCAN_BAND_A_WITH_DFS: 5 GHz with DFS
 * @IWL_GSCAN_BAND_ABG: 2.4 GHz + 5 GHz; no DFS
 * @IWL_GSCAN_BAND_ABG_WITH_DFS: 2.4 GHz + 5 GHz with DFS
 * @NUM_IWL_GSCAN_BAND: number of defined band values
 */
enum iwl_gscan_band {
	IWL_GSCAN_BAND_UNSPECIFIED,
	IWL_GSCAN_BAND_BG,
	IWL_GSCAN_BAND_A,
	IWL_GSCAN_BAND_A_DFS,
	IWL_GSCAN_BAND_A_WITH_DFS,
	IWL_GSCAN_BAND_ABG,
	IWL_GSCAN_BAND_ABG_WITH_DFS,
	NUM_IWL_GSCAN_BAND,
};

#define GSCAN_MAX_CHANNELS 16

/**
 * struct iwl_gscan_bucket_spec - gscan bucket specification
 * @scan_interval: scan interval for this bucket. In milliseconds.
 * @band: the band to scan as specified in &enum iwl_gscan_band.
 *	If %IWL_GSCAN_BAND_UNSPECIFIED, use the channel list.
 * @report_policy: report policy for this bucket as specified in
 *	&enum iwl_mvm_vendor_gscan_report_mode.
 * @index: bucket index.
 * @channel_count: number of channels in channels array.
 * @reserved: reserved.
 * @channels: array of channels to scan.
 */
struct iwl_gscan_bucket_spec {
	__le32 scan_interval;
	__le32 band;
	__le32 report_policy;
	u8 index;
	u8 channel_count;
	__le16 reserved;
	struct iwl_gscan_channel_spec channels[GSCAN_MAX_CHANNELS];
} __packed; /* GSCAN_BUCKET_API_S_VER_1 */

/**
 * enum iwl_gscan_start_flags - start gscan command flags
 * @IWL_GSCAN_START_FLAGS_MAC_RANDOMIZE: should use mac randomization.
 */
enum iwl_gscan_start_flags {
	IWL_GSCAN_START_FLAGS_MAC_RANDOMIZE = BIT(0),
};

#define GSCAN_MAX_BUCKETS 16

/**
 * struct iwl_gscan_start_cmd - gscan start command
 * @max_scan_aps: number of AP's to store in each scan in the BSSID/RSSI history
 *	buffer (keep the highest RSSI AP's).
 * @flags: bitmap - enum iwl_gscan_start_flags.
 * @report_threshold: in percentage. When buffer is this much full, wake up AP.
 * @bucket_count: number of bucket in this gscan start command.
 * @mac_addr_template: sets the fixed part of a randomized MAC address: For any
 *	mask bit below, set to 0 to copy the value from the template.
 * @mac_addr_mask: bits set to 0 will be copied from the MAC address template.
 *	Bits set to 1 will be randomized by the UMAC.
 * @buckets: bucket specifications as described in struct
 *	%iwl_gscan_bucket_spec.
 */
struct iwl_gscan_start_cmd {
	__le32 max_scan_aps;
	__le32 flags;
	__le32 report_threshold;
	__le32 bucket_count;
	u8 mac_addr_template[ETH_ALEN];
	u8 mac_addr_mask[ETH_ALEN];
	struct iwl_gscan_bucket_spec buckets[GSCAN_MAX_BUCKETS];
} __packed; /* GSCAN_START_API_S_VER_1 */

/**
 * struct iwl_gscan_scan_result - gscan scan result
 * @bssid: BSSID.
 * @channel: channel number.
 * @rssi: RSSI. In dB.
 * @timestamp: time since boot when the result was recevied. in usecs.
 * @ssid: SSID.
 */
struct iwl_gscan_scan_result {
	u8 bssid[ETH_ALEN];
	u8 channel;
	u8 rssi;
	__le32 timestamp;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
} __packed; /* GSCAN_SCAN_RESULT_API_S_VER_1 */

/**
 * struct iwl_gscan_results_event - gscan results available event
 * @event_type: scan results available event type as specified in &enum
 *	iwl_mvm_vendor_results_event_type.
 * @num_res: number of available scan results.
 * @results: gscan results
 */
struct iwl_gscan_results_event {
	__le32 event_type;
	__le32 num_res;
	struct iwl_gscan_scan_result results[];
} __packed; /* GSCAN_SCAN_RESULTS_AVAILABLE_API_S_VER_1 */

#endif
