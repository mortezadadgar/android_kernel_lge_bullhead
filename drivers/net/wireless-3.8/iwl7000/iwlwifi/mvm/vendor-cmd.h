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
#ifndef __VENDOR_CMD_H__
#define __VENDOR_CMD_H__

#define INTEL_OUI	0x001735

/**
 * enum iwl_mvm_vendor_cmd - supported vendor commands
 * @IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY: set low-latency mode for the given
 *	virtual interface
 * @IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY: query low-latency mode
 * @IWL_MVM_VENDOR_CMD_TCM_EVENT: TCM event
 * @IWL_MVM_VENDOR_CMD_LTE_STATE: inform the LTE modem state
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_CONFIG_INFO: configure LTE-Coex static
 *	parameters
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_DYNAMIC_INFO: configure LTE dynamic parameters
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_SPS_INFO: configure semi oersistent info
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_WIFI_RPRTD_CHAN: Wifi reported channel as
 *	calculated by the coex-manager
 * @IWL_MVM_VENDOR_CMD_SET_COUNTRY: set a new mcc regulatory information
 * @IWL_MVM_VENDOR_CMD_PROXY_FRAME_FILTERING: filter GTK, gratuitous
 *	ARP & unsolicited NA
 * @IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_ADD: add a peer to the TDLS peer cache
 * @IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_DEL: delete a peer from the TDLS peer
 *	cache
 * @IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_QUERY: query traffic statistics for a
 *	peer in the TDLS cache
 * @IWL_MVM_VENDOR_CMD_SET_NIC_TXPOWER_LIMIT: set the NIC's (SAR) TX power limit
 * @IWL_MVM_VENDOR_CMD_OPPPS_WA: wa to pass Sigma test - applicable code is
 *	claused under CPTCFG_IWLMVM_P2P_OPPPS_TEST_WA
 * @IWL_MVM_VENDOR_CMD_GSCAN_GET_CAPABILITIES: get driver gscan capabilities as
 *	specified in %IWL_MVM_VENDOR_ATTR_GSCAN_*
 * @IWL_MVM_VENDOR_CMD_GSCAN_START: set gscan parameters and start gscan
 * @IWL_MVM_VENDOR_CMD_GSCAN_STOP: stop a previously started gscan
 * @IWL_MVM_VENDOR_CMD_GSCAN_RESULTS_EVENT: event that reports scan results
 *	from gscan. This event is sent when the scan results buffer has reached
 *	the report threshold, or when scanning a bucket with report mode
 *	%IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE was completed.
 * @IWL_MVM_VENDOR_CMD_GSCAN_SET_BSSID_HOTLIST: set a list of AP's to track
 *	changes in their RSSI and report scan results history when RSSI goes
 *	above/below threshold. Sending this command with an empty list of AP's
 *	will cancel previous set_bssid_hotlist request.
 * @IWL_MVM_VENDOR_CMD_GSCAN_SET_SIGNIFICANT_CHANGE: set a list of APs to track
 *	significant changes in their RSSI. Sending this command with an empty
 *	list of AP's will cancel previous set_significant_change request.
 * @IWL_MVM_VENDOR_CMD_GSCAN_HOTLIST_CHANGE_EVENT: event that indicates that an
 *	AP from the BSSID hotlist was lost or found.
 * @IWL_MVM_VENDOR_CMD_GSCAN_SIGNIFICANT_CHANGE_EVENT: event that indicates a
 *	significant change in the RSSI level of beacons received from a certain
 *	AP.
 * @IWL_MVM_VENDOR_CMD_RXFILTER: Set/clear rx filter.
 * @IWL_MVM_VENDOR_CMD_GSCAN_BEACON_EVENT: event that reports a
 *	beacon/probe response was received, and contains information from the
 *	beacon/probe response. This event is sent for buckets with report mode
 *	set to %IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE_RESULTS.
 * @IWL_MVM_VENDOR_CMD_DBG_COLLECT: collect debug data
 * @IWL_MVM_VENDOR_CMD_NAN_FAW_CONF: Configure post NAN further availability.
 */

enum iwl_mvm_vendor_cmd {
	IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY,
	IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY,
	IWL_MVM_VENDOR_CMD_TCM_EVENT,
	IWL_MVM_VENDOR_CMD_LTE_STATE,
	IWL_MVM_VENDOR_CMD_LTE_COEX_CONFIG_INFO,
	IWL_MVM_VENDOR_CMD_LTE_COEX_DYNAMIC_INFO,
	IWL_MVM_VENDOR_CMD_LTE_COEX_SPS_INFO,
	IWL_MVM_VENDOR_CMD_LTE_COEX_WIFI_RPRTD_CHAN,
	IWL_MVM_VENDOR_CMD_SET_COUNTRY,
	IWL_MVM_VENDOR_CMD_PROXY_FRAME_FILTERING,
	IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_ADD,
	IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_DEL,
	IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_QUERY,
	IWL_MVM_VENDOR_CMD_SET_NIC_TXPOWER_LIMIT,
	IWL_MVM_VENDOR_CMD_OPPPS_WA,
	IWL_MVM_VENDOR_CMD_GSCAN_GET_CAPABILITIES,
	IWL_MVM_VENDOR_CMD_GSCAN_START,
	IWL_MVM_VENDOR_CMD_GSCAN_STOP,
	IWL_MVM_VENDOR_CMD_GSCAN_RESULTS_EVENT,
	IWL_MVM_VENDOR_CMD_GSCAN_SET_BSSID_HOTLIST,
	IWL_MVM_VENDOR_CMD_GSCAN_SET_SIGNIFICANT_CHANGE,
	IWL_MVM_VENDOR_CMD_GSCAN_HOTLIST_CHANGE_EVENT,
	IWL_MVM_VENDOR_CMD_GSCAN_SIGNIFICANT_CHANGE_EVENT,
	IWL_MVM_VENDOR_CMD_RXFILTER,
	IWL_MVM_VENDOR_CMD_GSCAN_BEACON_EVENT,
	IWL_MVM_VENDOR_CMD_DBG_COLLECT,
	IWL_MVM_VENDOR_CMD_NAN_FAW_CONF,
};

/**
 * enum iwl_mvm_vendor_load - traffic load identifiers
 * @IWL_MVM_VENDOR_LOAD_LOW: low load: less than 10% airtime usage
 * @IWL_MVM_VENDOR_LOAD_MEDIUM: medium load: 10% or more, but less than 50%
 * @IWL_MVM_VENDOR_LOAD_HIGH: high load: 50% or more
 *
 * Traffic load is calculated based on the percentage of airtime used
 * (TX airtime is accounted as RTS+CTS+PPDU+ACK/BlockACK, RX airtime
 * is just the PPDU's time)
 */
enum iwl_mvm_vendor_load {
	IWL_MVM_VENDOR_LOAD_LOW,
	IWL_MVM_VENDOR_LOAD_MEDIUM,
	IWL_MVM_VENDOR_LOAD_HIGH,
};

/**
 * enum iwl_mvm_vendor_gscan_report_mode - gscan scan results report modes
 * @IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER: report that scan results are
 *	available only when the scan results buffer reaches the report
 *	threshold. The report threshold is set for each bucket. See
 *	%IWL_MVM_VENDOR_ATTR_GSCAN_START_REPORT_THRESHOLD.
 * @IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE: like
 *	%IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER + report that scan results are
 *	available when scanning of this bucket is complete.
 * @IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE_RESULTS: like
 *	%IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE + forward scan results
 *	(beacons/probe responses) in real time to userspace.
 * @NUM_IWL_MVM_VENDOR_GSCAN_REPORT: number of defined report modes for gscan.
 */
enum iwl_mvm_vendor_gscan_report_mode {
	IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER,
	IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE,
	IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE_RESULTS,
	NUM_IWL_MVM_VENDOR_GSCAN_REPORT,
};

/**
 * enum iwl_mvm_vendor_gscan_channel_spec - gscan channel specification
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL: channel number
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME: u16 attribute specifying dwell
 *	time on this channel.
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_PASSIVE: flag attribute. If set, passive
 *	scan should be performed on this channel.
 * @NUM_IWL_MVM_VENDOR_CHANNEL_SPEC: number of channel spec attributes.
 * @MAX_IWL_MVM_VENDOR_CHANNEL_SPEC: highest channel spec attribute number.
 */
enum iwl_mvm_vendor_gscan_channel_spec {
	IWL_MVM_VENDOR_CHANNEL_SPEC_INVALID,
	IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL,
	IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME,
	IWL_MVM_VENDOR_CHANNEL_SPEC_PASSIVE,
	NUM_IWL_MVM_VENDOR_CHANNEL_SPEC,
	MAX_IWL_MVM_VENDOR_CHANNEL_SPEC =
		NUM_IWL_MVM_VENDOR_CHANNEL_SPEC - 1,
};

/**
 * enum iwl_mvm_vendor_gscan_bucket_spec - gscan bucket specification
 * @IWL_MVM_VENDOR_BUCKET_SPEC_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_BUCKET_SPEC_INDEX: bucket index
 * @IWL_MVM_VENDOR_BUCKET_SPEC_BAND: band to scan as specified in
 *	&enum iwl_gscan_band. When not set, the channel list is used.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD: interval between this bucket scans,
 *	in msecs.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE: when to report scan results.
 *	Available modes are specified in &enum iwl_mvm_vendor_report_mode.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS: array of channels to scan for this
 *	bucket. Each channel is specified with a nested attribute of
 *	%IWL_MVM_VENDOR_CHANNEL_SPEC. This channel list is used when
 *	%IWL_MVM_VENDOR_BUCKET_SPEC_BAND is set to
 *	%IWL_MVM_VENDOR_BAND_UNSPECIFIED.
 * @NUM_IWL_MVM_VENDOR_BUCKET_SPEC: number of bucket spec attributes.
 * @MAX_IWL_MVM_VENDOR_BUCKET_SPEC: highest bucket spec attribute number.
 */
enum iwl_mvm_vendor_gscan_bucket_spec {
	IWL_MVM_VENDOR_BUCKET_SPEC_INVALID,
	IWL_MVM_VENDOR_BUCKET_SPEC_INDEX,
	IWL_MVM_VENDOR_BUCKET_SPEC_BAND,
	IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD,
	IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE,
	IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS,
	NUM_IWL_MVM_VENDOR_BUCKET_SPEC,
	MAX_IWL_MVM_VENDOR_BUCKET_SPEC =
		NUM_IWL_MVM_VENDOR_BUCKET_SPEC - 1,
};

/**
 * enum iwl_mvm_vendor_results_event_type - scan results available event type
 * @IWL_MVM_VENDOR_RESULTS_NOTIF_BUFFER_FULL: scan results available was
 *	reported because scan results buffer has reached the report threshold.
 * @IWL_MVM_VENDOR_RESULTS_NOTIF_BUCKET_END: scan results available was reported
 *	because scan of a bucket was completed.
 * @NUM_IWL_VENDOR_RESULTS_NOTIF_EVENT_TYPE: number of defined gscan results
 *	notification event types.
 */
enum iwl_mvm_vendor_results_event_type {
	IWL_MVM_VENDOR_RESULTS_NOTIF_BUFFER_FULL,
	IWL_MVM_VENDOR_RESULTS_NOTIF_BUCKET_END,
	NUM_IWL_VENDOR_RESULTS_NOTIF_EVENT_TYPE,
};

/**
 * enum iwl_mvm_vendor_gscan_result - gscan scan result
 * @IWL_MVM_VENDOR_GSCAN_RESULT_INVALID: attribute number 0 is reserved.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_TIMESTAMP: time since boot (in usecs) when
 *	the result was retrieved.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_SSID: SSID.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_BSSID: BSSID of the BSS (6 octets).
 * @IWL_MVM_VENDOR_GSCAN_RESULT_CHANNEL: channel frequency in MHz.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_RSSI: signal strength in dB.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_FRAME: the whole beacon/probe response
 *	frame data including the header.
 * @NUM_IWL_MVM_VENDOR_GSCAN_RESULT: number of scan result attributes.
 * @MAX_IWL_MVM_VENDOR_GSCAN_RESULT: highest scan result attribute number.
 */
enum iwl_mvm_vendor_gscan_result {
	IWL_MVM_VENDOR_GSCAN_RESULT_INVALID,
	IWL_MVM_VENDOR_GSCAN_RESULT_TIMESTAMP,
	IWL_MVM_VENDOR_GSCAN_RESULT_SSID,
	IWL_MVM_VENDOR_GSCAN_RESULT_BSSID,
	IWL_MVM_VENDOR_GSCAN_RESULT_CHANNEL,
	IWL_MVM_VENDOR_GSCAN_RESULT_RSSI,
	IWL_MVM_VENDOR_GSCAN_RESULT_FRAME,
	NUM_IWL_MVM_VENDOR_GSCAN_RESULT,
	MAX_IWL_MVM_VENDOR_GSCAN_RESULT =
		NUM_IWL_MVM_VENDOR_GSCAN_RESULT - 1,
};

/**
 * enum iwl_mvm_vendor_ap_threshold_param - parameters for tracking AP's RSSI
 * @IWL_MVM_VENDOR_AP_THRESHOLD_PARAM_INVALID: attribute number 0 is reserved.
 * @IWL_MVM_VENDOR_AP_BSSID: BSSID of the BSS (6 octets)
 * @IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD: low RSSI threshold. in dB.
 * @IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD: high RSSI threshold. in dB.
 * @IWL_MVM_VENDOR_AP_CHANNEL_HINT: operating channel of the BSS.
 *	This is only a hint, the BSS may be operating on a different channel.
 * @NUM_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM: number of ap threshold param
 *	attributes.
 * @MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM: highest ap threshold param
 *	attribute number.
 */
enum iwl_mvm_vendor_ap_threshold_param {
	IWL_MVM_VENDOR_AP_THRESHOLD_PARAM_INVALID,
	IWL_MVM_VENDOR_AP_BSSID,
	IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD,
	IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD,
	IWL_MVM_VENDOR_AP_CHANNEL_HINT,
	NUM_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM,
	MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM =
		NUM_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM - 1,
};

/**
 * enum iwl_mvm_vendor_hotlist_ap_status - whether an AP was found or lost
 * @IWL_MVM_VENDOR_HOTLIST_AP_FOUND: beacon from this AP was received with RSSI
 *	above the configured high threshold.
 * @IWL_MVM_VENDOR_HOTLIST_AP_LOST: beacon from this AP was received with RSSI
 *	below the configured low threshold.
 * @NUM_IWL_MVM_VENDOR_HOTLIST_AP_STATUS: number of defined AP statuses.
 */
enum iwl_mvm_vendor_hotlist_ap_status {
	IWL_MVM_VENDOR_HOTLIST_AP_FOUND,
	IWL_MVM_VENDOR_HOTLIST_AP_LOST,
	NUM_IWL_MVM_VENDOR_HOTLIST_AP_STATUS,
};

/**
 * enum iwl_mvm_vendor_significant_change_result - significant change result
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_CHANNEL: channel number of the reported
 *	AP.
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_BSSID: BSSID.
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RSSI_HISTORY: array of RSSI samples for
 *	the reported AP. in dB.
 * @NUM_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT: number of significant change
 *	attriutes.
 * @MAX_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT: highest significant change
 *	result attribute number.
 */
enum iwl_mvm_vendor_significant_change_result {
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_INVALID,
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_CHANNEL,
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_BSSID,
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RSSI_HISTORY,
	NUM_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT,
	MAX_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT =
	NUM_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT - 1,
};

/**
 * enum iwl_mvm_vendor_rxfilter_flags - the type of request rxfilter
 *
 * @IWL_MVM_VENDOR_RXFILTER_UNICAST: control unicast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_BCAST: control broadcast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_MCAST4: control IPv4 multicast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_MCAST6: control IPv4 multicast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_EINVAL: no Rx filter command was set
 *
 */
enum iwl_mvm_vendor_rxfilter_flags {
	IWL_MVM_VENDOR_RXFILTER_UNICAST = 1 << 0,
	IWL_MVM_VENDOR_RXFILTER_BCAST = 1 << 1,
	IWL_MVM_VENDOR_RXFILTER_MCAST4 = 1 << 2,
	IWL_MVM_VENDOR_RXFILTER_MCAST6 = 1 << 3,
	IWL_MVM_VENDOR_RXFILTER_EINVAL = 1 << 7,
};

/**
 * enum iwl_mvm_vendor_rxfilter_op - the operation associated with a filter
 *
 * @IWL_MVM_VENDOR_RXFILTER_OP_PASS: pass frames matching the filter
 * @IWL_MVM_VENDOR_RXFILTER_OP_DROP: drop frames matching the filter
 */
enum iwl_mvm_vendor_rxfilter_op {
	IWL_MVM_VENDOR_RXFILTER_OP_PASS,
	IWL_MVM_VENDOR_RXFILTER_OP_DROP,
};

/**
 * enum iwl_mvm_vendor_attr - attributes used in vendor commands
 * @__IWL_MVM_VENDOR_ATTR_INVALID: attribute 0 is invalid
 * @IWL_MVM_VENDOR_ATTR_LOW_LATENCY: low-latency flag attribute
 * @IWL_MVM_VENDOR_ATTR_VIF_ADDR: interface MAC address
 * @IWL_MVM_VENDOR_ATTR_VIF_LL: vif-low-latency (u8, 0/1)
 * @IWL_MVM_VENDOR_ATTR_LL: global low-latency (u8, 0/1)
 * @IWL_MVM_VENDOR_ATTR_VIF_LOAD: vif traffic load (u8, see load enum)
 * @IWL_MVM_VENDOR_ATTR_LOAD: global traffic load (u8, see load enum)
 * @IWL_MVM_VENDOR_ATTR_COUNTRY: MCC to set, for regulatory information (u16)
 * IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA: filter gratuitous ARP and unsolicited
 *	Neighbor Advertisement frames
 * IWL_MVM_VENDOR_ATTR_FILTER_GTK: filter Filtering Frames Encrypted using
 *	the GTK
 * @IWL_MVM_VENDOR_ATTR_ADDR: MAC address
 * @IWL_MVM_VENDOR_ATTR_TX_BYTES: number of bytes transmitted to peer
 * @IWL_MVM_VENDOR_ATTR_RX_BYTES: number of bytes received from peer
 * @IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24: TX power limit for 2.4 GHz
 *	(s32 in units of 1/8 dBm)
 * @IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L: TX power limit for 5.2 GHz low (as 2.4)
 * @IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H: TX power limit for 5.2 GHz high (as 2.4)
 * @IWL_MVM_VENDOR_ATTR_OPPPS_WA: wa to pass Sigma test
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_CACHE_SIZE: scan cache size
 *	(in bytes)
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_BUCKETS: maximum number of channel
 *	buckets
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_CACHE_PER_SCAN: maximum number of AP's
 *	that can be stored per scan
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_RSSI_SAMPLE_SIZE: number of RSSI samples
 *	used for averaging RSSI
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_REPORTING_THRESHOLD: max possible
 *	report threshold. see %IWL_MVM_VENDOR_ATTR_GSCAN_START_REPORT_THRESHOLD
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_APS: maximum number of entries for
 *	hotlist AP's
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SIGNIFICANT_CHANGE_APS: maximum number of
 *	entries for significant change AP's
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_BSSID_HISTORY_ENTRIES: number of
 *	BSSID/RSSI entries that the device can hold
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR: mac address to be used on gscan scans
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK: mac address mask. Bits set to 0
 *	will be copied from %IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR. Bits set to 1
 *	will be randomized
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN: number of AP's to store in each
 *	scan in the BSSID/RSSI history buffer (keep the highest RSSI AP's)
 * @IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD: report that scan results
 *	are available when buffer is that much full. In percentage.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS: array of bucket specifications for
 *	this gscan start command. Each bucket spec is a nested attribute of
 *	&enum iwl_mvm_vendor_gscan_bucket_spec.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS_EVENT_TYPE: gscan results event type as
 *	specified in &enum iwl_mvm_vendor_results_event_type.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS: array of gscan results. Each result is a
 *	nested attribute of &enum iwl_mvm_vendor_gscan_result.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE: number of samples to confirm
 *	ap loss.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST: an array of nested attributes of
 *	&enum iwl_mvm_vendor_ap_threshold_param.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE: number of samples for averaging
 *	RSSI
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING: number of APs breaching threshold
 * @IWL_MVM_VENDOR_ATTR_GSCAN_HOTLIST_AP_STATUS: indicates if a reported AP was
 *	lost or found as specified in &enum iwl_mvm_vendor_hotlist_ap_status.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_SIG_CHANGE_RESULTS: array of significant
 *	change results. Each result is a nested attribute of &enum
 *	iwl_mvm_vendor_significant_change_result.
 * @IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ: u32 attribute. Frequency (in MHz) to be
 *	used for NAN further availability.
 * @IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS: u8 attribute. Number of 16TU slots
 *	the NAN device will be available on it's FAW between DWs.
 *
 * @NUM_IWL_MVM_VENDOR_ATTR: number of vendor attributes
 * @MAX_IWL_MVM_VENDOR_ATTR: highest vendor attribute number
 * @IWL_MVM_VENDOR_ATTR_RXFILTER: u32 attribute.
 *      See %iwl_mvm_vendor_rxfilter_flags.
 * @IWL_MVM_VENDOR_ATTR_RXFILTER_OP: u32 attribute.
 *      See %iwl_mvm_vendor_rxfilter_op.
 * @IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER: description of collect debug data
	trigger.
 */
enum iwl_mvm_vendor_attr {
	__IWL_MVM_VENDOR_ATTR_INVALID,
	IWL_MVM_VENDOR_ATTR_LOW_LATENCY,
	IWL_MVM_VENDOR_ATTR_VIF_ADDR,
	IWL_MVM_VENDOR_ATTR_VIF_LL,
	IWL_MVM_VENDOR_ATTR_LL,
	IWL_MVM_VENDOR_ATTR_VIF_LOAD,
	IWL_MVM_VENDOR_ATTR_LOAD,
	IWL_MVM_VENDOR_ATTR_COUNTRY,
	IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA,
	IWL_MVM_VENDOR_ATTR_FILTER_GTK,
	IWL_MVM_VENDOR_ATTR_ADDR,
	IWL_MVM_VENDOR_ATTR_TX_BYTES,
	IWL_MVM_VENDOR_ATTR_RX_BYTES,
	IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24,
	IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L,
	IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H,
	IWL_MVM_VENDOR_ATTR_OPPPS_WA,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_CACHE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_BUCKETS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_CACHE_PER_SCAN,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_RSSI_SAMPLE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_REPORTING_THRESHOLD,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_APS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SIGNIFICANT_CHANGE_APS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_BSSID_HISTORY_ENTRIES,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN,
	IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD,
	IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS,
	IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS_EVENT_TYPE,
	IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS,
	IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST,
	IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING,
	IWL_MVM_VENDOR_ATTR_GSCAN_HOTLIST_AP_STATUS,
	IWL_MVM_VENDOR_ATTR_GSCAN_SIG_CHANGE_RESULTS,
	IWL_MVM_VENDOR_ATTR_RXFILTER,
	IWL_MVM_VENDOR_ATTR_RXFILTER_OP,
	IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER,
	IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ,
	IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS,

	NUM_IWL_MVM_VENDOR_ATTR,
	MAX_IWL_MVM_VENDOR_ATTR = NUM_IWL_MVM_VENDOR_ATTR - 1,
};
#define IWL_MVM_VENDOR_FILTER_ARP_NA IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA
#define IWL_MVM_VENDOR_FILTER_GTK IWL_MVM_VENDOR_ATTR_FILTER_GTK
#endif /* __VENDOR_CMD_H__ */
