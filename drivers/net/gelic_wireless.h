/*
 * gelic_wireless.h: wireless extension for gelic_net
 *
 * Copyright (C) 2007 Sony Computer Entertainment Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _GELIC_WIRELESS_H_
#define _GELIC_WIRELESS_H_

#include <linux/wireless.h>
#include <net/ieee80211.h>

/* wireless */
#define GELICW_WIRELESS_NOT_EXIST	0
#define GELICW_WIRELESS_SUPPORTED	1
#define GELICW_WIRELESS_ON		2
#define GELICW_WIRELESS_SHUTDOWN	3
/* state */
#define GELICW_STATE_DOWN		0
#define GELICW_STATE_UP			1
#define GELICW_STATE_SCANNING		2
#define GELICW_STATE_SCAN_DONE		3
#define GELICW_STATE_ASSOCIATED		4

/* cmd_send_flg */
#define GELICW_CMD_SEND_NONE		0x00
#define GELICW_CMD_SEND_COMMON		0x01
#define GELICW_CMD_SEND_ENCODE		0x02
#define GELICW_CMD_SEND_SCAN		0x04
#define GELICW_CMD_SEND_ALL		(GELICW_CMD_SEND_COMMON \
					| GELICW_CMD_SEND_ENCODE \
					| GELICW_CMD_SEND_SCAN)
/* max scan list */
#define MAX_SCAN_BSS			16

#define GELICW_SCAN_INTERVAL		(HZ)

#ifdef DEBUG
#define CH_INFO_FAIL 0x0600 /* debug */
#else
#define CH_INFO_FAIL 0
#endif

struct gelicw_bss {
	u8 bssid[ETH_ALEN];
	u8 channel;
	u8 mode;
	u8 essid_len;
	u8 essid[IW_ESSID_MAX_SIZE + 1]; /* null terminated for debug msg */

	u16 capability;
	u16 beacon_interval;

	u8 rates_len;
	u8 rates[MAX_RATES_LENGTH];
	u8 rates_ex_len;
	u8 rates_ex[MAX_RATES_EX_LENGTH];
	u8 rssi;

	/* scan results have sec_info instead of rsn_ie or wpa_ie */
	u16 sec_info;
};

struct gelic_wireless {
	struct gelic_net_card *card;
	struct completion cmd_done, rssi_done;
	struct work_struct work_event, work_start_done;
	struct delayed_work work_rssi, work_scan_all, work_scan_essid;
	struct delayed_work work_common, work_encode;
	struct delayed_work work_start, work_stop, work_roam;
	wait_queue_head_t waitq_cmd, waitq_scan;

	u64 cmd_tag, cmd_id;
	u8 cmd_send_flg;

	struct iw_public_data wireless_data;
	u8 *data_buf; /* data buffer for lv1_net_control */

	u8 wireless; /* wireless support */
	u8 state;
	u8 scan_all; /* essid scan or all scan */
	u8 essid_search; /* essid background scan */
	u8 is_assoc;

	u16 ch_info; /* supoprted channels */
	u8 wireless_mode; /* 11b/g */
	u8 channel; /* current ch */
	u8 iw_mode; /* INFRA or Ad-hoc */
	u8 rssi;
	u8 essid_len;
	u8 essid[IW_ESSID_MAX_SIZE + 1]; /* null terminated for debug msg */
	u8 nick[IW_ESSID_MAX_SIZE + 1];
	u8 bssid[ETH_ALEN];

	u8 key_index;
	u8 key[WEP_KEYS][IW_ENCODING_TOKEN_MAX]; /* 4 * 64byte */
	u8 key_len[WEP_KEYS];
	u8 key_alg; /* key algorithm  */
	u8 auth_mode; /* authenticaton mode */

	u8 bss_index; /* current bss in bss_list */
	u8 num_bss_list;
	u8 bss_key_alg; /* key alg of bss */
	u8 wap_bssid[ETH_ALEN];
	unsigned long last_scan; /* last scan time */
	struct gelicw_bss current_bss;
	struct gelicw_bss bss_list[MAX_SCAN_BSS];
};

/* net_control command */
#define GELICW_SET_PORT			3 /* control Ether port */
#define GELICW_GET_INFO			6 /* get supported channels */
#define GELICW_SET_CMD			9 /* set configuration */
#define GELICW_GET_RES			10 /* get command response */
#define GELICW_GET_EVENT		11 /* get event from device */
/* net_control command data buffer */
#define GELICW_DATA_BUF_SIZE		0x1000

/* GELICW_SET_CMD params */
#define GELICW_CMD_START		1
#define GELICW_CMD_STOP			2
#define GELICW_CMD_SCAN			3
#define GELICW_CMD_GET_SCAN		4
#define GELICW_CMD_SET_CONFIG		5
#define GELICW_CMD_GET_CONFIG		6
#define GELICW_CMD_SET_WEP		7
#define GELICW_CMD_GET_WEP		8
#define GELICW_CMD_SET_WPA		9
#define GELICW_CMD_GET_WPA		10
#define GELICW_CMD_GET_RSSI		11

/* GELICW_SET_PORT params */
#define GELICW_ETHER_PORT		2
#define GELICW_PORT_DOWN		0 /* Ether port off */
#define GELICW_PORT_UP			4 /* Ether port on (auto neg) */

/* interrupt status bit */
#define GELICW_DEVICE_CMD_COMP		0x0000000080000000UL
#define GELICW_DEVICE_EVENT_RECV	0x0000000040000000UL

/* GELICW_GET_EVENT ID */
#define GELICW_EVENT_UNKNOWN		0x00
#define GELICW_EVENT_DEVICE_READY	0x01
#define GELICW_EVENT_SCAN_COMPLETED	0x02
#define GELICW_EVENT_DEAUTH		0x04
#define GELICW_EVENT_BEACON_LOST	0x08
#define GELICW_EVENT_CONNECTED		0x10
#define GELICW_EVENT_WPA_CONNECTED	0x20
#define GELICW_EVENT_WPA_ERROR		0x40
#define GELICW_EVENT_NO_ENTRY		(-6)

#define MAX_IW_PRIV_SIZE		32

/* structure of data buffer for lv1_net_contol */
/* wep_config: sec */
#define GELICW_WEP_SEC_NONE		0
#define GELICW_WEP_SEC_40BIT		1
#define GELICW_WEP_SEC_104BIT		2
struct wep_config {
	u16 sec;
	u8  key[4][16];
} __attribute__ ((packed));

/* wpa_config: sec */
#define GELICW_WPA_SEC_NONE		0
#define GELICW_WPA_SEC_TKIP		1
#define GELICW_WPA_SEC_AES		2
/* wpa_config: psk_type */
#define GELICW_PSK_PASSPHRASE		0
#define GELICW_PSK_64HEX		1
struct wpa_config {
	u16 sec;
	u16 psk_type;
	u8  psk_material[64]; /* key */
} __attribute__ ((packed));

/* common_config: bss_type */
#define GELICW_BSS_INFRA		0
#define GELICW_BSS_ADHOC		1
/* common_config: auth_method */
#define GELICW_AUTH_OPEN		0
#define GELICW_AUTH_SHARED		1
/* common_config: op_mode */
#define GELICW_OP_MODE_11BG		0
#define GELICW_OP_MODE_11B		1
#define GELICW_OP_MODE_11G		2
struct common_config {
	u16 scan_index; /* index of scan_desc list */
	u16 bss_type;
	u16 auth_method;
	u16 op_mode;
} __attribute__ ((packed));

/* scan_descriptor: security */
#define GELICW_SEC_TYPE_NONE		0x0000
#define GELICW_SEC_TYPE_WEP		0x0100
#define GELICW_SEC_TYPE_WEP40		0x0101
#define GELICW_SEC_TYPE_WEP104		0x0102
#define GELICW_SEC_TYPE_TKIP		0x0201
#define GELICW_SEC_TYPE_AES		0x0202
#define GELICW_SEC_TYPE_WEP_MASK	0xFF00
struct scan_desc {
	u16 size;
	u16 rssi;
	u16 channel;
	u16 beacon_period;
	u16 capability;
	u16 security;
	u64 bssid;
	u8  essid[32];
	u8  rate[16];
	u8  ext_rate[16];
	u32 reserved1;
	u32 reserved2;
	u32 reserved3;
	u32 reserved4;
} __attribute__ ((packed));

/* rssi_descriptor */
struct rssi_desc {
	u16 rssi; /* max rssi = 100 */
} __attribute__ ((packed));


extern unsigned long p_to_lp(long pa);
extern int gelicw_setup_netdev(struct net_device *netdev, int wi);
extern void gelicw_up(struct net_device *netdev);
extern int gelicw_down(struct net_device *netdev);
extern void gelicw_remove(struct net_device *netdev);
extern void gelicw_interrupt(struct net_device *netdev, u32 status1);
extern int gelicw_is_associated(struct net_device *netdev);

#endif //  _GELIC_WIRELESS_H_
