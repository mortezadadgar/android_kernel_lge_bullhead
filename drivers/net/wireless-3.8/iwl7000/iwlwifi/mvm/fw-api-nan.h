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
 *  Intel Linux Wireless <linuxwifi@intel.com>
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

#ifndef __fw_api_nan_h__
#define __fw_api_nan_h__

#include "fw-api.h"

/* TODO: read it from tlv */
#define NAN_MAX_SUPPORTED_DE_ENTRIES 10

/* NAN commands */
enum iwl_nan_subcmd_ids {
	/* NaN commands */
	NAN_CONFIG_CMD = 0,
	NAN_DISCOVERY_FUNC_CMD = 0x1,
	NAN_FAW_CONFIG_CMD = 0x2,
	NAN_DISCOVERY_EVENT_NOTIF = 0xFD,
	NAN_DISCOVERY_TERMINATE_NOTIF = 0xFE,
	NAN_FAW_START_NOTIF = 0xFF,
};

/**
 * NAN configuration command. This command starts/stops/modifies NAN
 * sync engine.
 *
 * @action: one of the FW_CTXT_ACTION_*
 * @tsf_id: tsf id to use in beacons
 * @sta_id: STA used for NAN operations. Currently it is AUX STA.
 * @node_addr: our address
 * @master_pref: master preference value
 * @master_rand: random factor to override fw's decision (DEBUG)
 * @cluster_id: cluster id, if 0 the fw will choose one for us.
 * @dual_band: enables dual band operation.
 * @beacon_template_id: beacon template id for NAN
 * @nan_state: nan state (DEBUG)
 * @chan24: override default 2.4GHz channel (DEBUG)
 * @chan52: override default 5.2GHz channel (DEBUG)
 * @hop_count: fake hop count (DEBUG)
 * @op_bands: band bit mask (DEBUG)
 * @warmup_timer: warmup_timer in us (DEBUG)
 * @custom_tsf: fake tsf value (DEBUG)
 * @action_delay: usecs to delay SDFs (DEBUG)
 */
struct iwl_nan_cfg_cmd {
	__le32 action;

	/* _NAN_UMAC_CONFIG_CMD_API_S_VER_1 */
	__le32 tsf_id;
	__le32 sta_id;
	u8 node_addr[6];
	__le16 reserved1;
	u8 master_pref;
	u8 master_rand;
	__le16 cluster_id;
	__le32 dual_band;
	__le32 beacon_template_id;

	/* NAN_TEST_BED_SPECIFIC_CONFIG_S_VER_1 */
	u8 chan24;
	u8 chan52;
	u8 hop_count;
	u8 op_bands;
	__le32 warmup_timer;
	__le64 custom_tsf;
	__le32 action_delay;
} __packed; /* NAN_CONFIG_CMD_API_S_VER_1 */

/* NAN DE function type */
enum iwl_fw_nan_func_type {
	NAN_DE_FUNC_PUBLISH = 0,
	NAN_DE_FUNC_SUBSCRIBE = 1,
	NAN_DE_FUNC_FOLLOW_UP = 2,

	/* keep last */
	NAN_DE_FUNC_NOT_VALID,
};

/* NAN DE function flags */
enum iwl_fw_nan_func_flags {
	NAN_DE_FUNC_FLAG_UNSOLICITED_OR_ACTIVE = BIT(0),
	NAN_DE_FUNC_FLAG_SOLICITED = BIT(1),
	NAN_DE_FUNC_FLAG_UNICAST = BIT(2),
	NAN_DE_FUNC_FLAG_CLOSE_RANGE = BIT(3),
	NAN_DE_FUNC_FLAG_FAW_PRESENT = BIT(4),
	NAN_DE_FUNC_FLAG_FAW_TYPE = BIT(5),
	NAN_DE_FUNC_FLAG_FAW_NOTIFY = BIT(6),
	NAN_DE_FUNC_FLAG_RAISE_EVENTS = BIT(7),
};

/**
 * NAN add/remove function command
 *
 * @action: one of the FW_CTXT_ACTION_ADD/REMOVE
 * @instance_id: instance id of the DE function to remove
 * @type: enum %iwl_fw_nan_func_type
 * @service_id: service id
 * @flags: a combination of %iwl_fw_nan_func_flags
 * @flw_up_id: follow up id
 * @flw_up_req_id: follow up requestor id
 * @flw_up_addr[6]: follow up destination address
 * @ttl: ttl in DW's or 0 for infinite
 * @faw_ci: struct %iwl_fw_channel_info for furher availability
 * @faw_attrtype: further availability bitmap
 * @serv_info_len: length of service specific information
 * @srf_len: length of the srf
 * @rx_filter_len: length of rx filter
 * @tx_filter_len: length of tx filter
 * data[0]: dw aligned fields -service_info, srf, rxFilter, txFilter
 */
struct iwl_nan_add_func_cmd {
	__le32 action;
	u8 instance_id;
	u8 type;
	u8 service_id[6];
	__le16 flags;
	u8 flw_up_id;
	u8 flw_up_req_id;
	u8 flw_up_addr[6];
	__le16 reserved1;
	__le32 ttl;
	struct iwl_fw_channel_info faw_ci;
	u8 fawAttrtype;
	u8 serv_info_len;
	u8 srf_len;
	u8 rx_filter_len;
	u8 tx_filter_len;
	u8 reserved[3];
	u8 data[0];
} __packed; /* NAN_DISCO_FUNC_FIXED_CMD_API_S_VER_1 */

enum iwl_nan_add_func_resp_status {
	NAN_DE_FUNC_STATUS_SUCCESSFUL,
	NAN_DE_FUNC_STATUS_INSUFFICIENT_ENTRIES,
	NAN_DE_FUNC_STATUS_INSUFFICIENT_MEMORY,
	NAN_DE_FUNC_STATUS_INVALID_INSTANCE,
	NAN_DE_FUNC_STATUS_UNSPECIFIED,
};

/**
 * Add NAN function response
 *
 * @instance_id: assigned instance_id (if added)
 * @status: status of the command (NAN_DE_FUNC_STATUS_*)
 */
struct iwl_nan_add_func_res {
	u8 instance_id;
	u8 status;
	__le16 reserved;
} __packed; /* NAN_DISCO_FUNC_CMD_API_S_VER_1 */

/**
 * NaN discovery event
 *
 * @peer_mac_addr: peer address
 * @type: Type of matching function
 * @instance_id: local matching instance id
 * @peer_instance: peer matching instance id
 * @service_info_len: service specific information length
 * @service_info: service specific information
 */
struct iwl_nan_disc_evt_notify {
	u8 peer_mac_addr[6];
	__le16 reserved1;
	u8 type;
	u8 instance_id;
	u8 peer_instance;
	u8 service_info_len;
	u8 service_info[0];
} __packed; /* NAN_DISCO_EVENT_NTFY_API_S_VER_1 */

/* NAN function termination reasons */
enum iwl_fw_nan_de_term_reason {
	NAN_DE_TERM_FAILURE = 0,
	NAN_DE_TERM_TTL_REACHED,
	NAN_DE_TERM_USER_REQUEST,
};

/**
 * NAN function termination event
 *
 * @type: type of terminated function (enum %iwl_fw_nan_func_type)
 * @instance_id: instance id
 * @reason: termination reason (enum %iwl_fw_nan_de_term_reason)
 */
struct iwl_nan_de_term {
	u8 type;
	u8 instance_id;
	u8 reason;
	u8 reserved1;
} __packed; /* NAN_DISCO_TERM_NTFY_API_S_VER_1 */

enum iwl_fw_post_nan_type {
	NAN_POST_NAN_ATTR_WLAN = 0,
	NAN_POST_NAN_ATTR_P2P,
	NAN_POST_NAN_ATTR_IBSS,
	NAN_POST_NAN_ATTR_MESH,
	NAN_POST_NAN_ATTR_FURTHER_NAN,
};

enum iwl_fw_config_flags {
	NAN_FAW_FLAG_NOTIFY_HOST = BIT(0),
};

/**
 * NAN further availability configuration command
 *
 * @id_n_color: id and color of the mac used for further availability
 * @faw_ci: channel to be available on
 * @type: type of post nan availability (enum %iwl_fw_post_nan_type)
 * @slots: number of 16TU slots to be available on (should be < 32)
 * @flags: NAN_FAW_FLAG_*
 * @op_class: operating class which corresponds to faw_ci
 */
struct iwl_nan_faw_config {
	__le32 id_n_color;
	struct iwl_fw_channel_info faw_ci;
	u8 type;
	u8 slots;
	u8 flags;
	u8 op_class;
} __packed; /* _NAN_DISCO_FAW_CMD_API_S_VER_1 */

#endif
