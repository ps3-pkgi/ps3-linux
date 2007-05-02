/*
 *  PS3 Platfom gelic network driver.
 *
 * Copyright (C) 2006 Sony Computer Entertainment Inc.
 *
 *  this file is based on: spider_net.c
 *
 * Network device driver for Cell Processor-Based Blade
 *
 * (C) Copyright IBM Corp. 2005
 *
 * Authors : Utz Bacher <utz.bacher@de.ibm.com>
 *           Jens Osterkamp <Jens.Osterkamp@de.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef _GELIC_NET_H
#define _GELIC_NET_H

#define GELIC_NET_DRV_NAME "Gelic Network Driver"
#define GELIC_NET_DRV_VERSION "1.0"

#define GELIC_NET_DEBUG

#ifdef GELIC_NET_DEBUG
#define DPRINTK(fmt,arg...)   printk(KERN_ERR fmt ,##arg)
#define DPRINTKK(fmt,arg...)  printk(KERN_ERR fmt ,##arg)
#else
#define DPRINTK(fmt,arg...)
#define DPRINTKK(fmt,arg...)
#endif

#define GELIC_NET_ETHTOOL               /* use ethtool */

/* ioctl */
#define GELIC_NET_GET_MODE              (SIOCDEVPRIVATE + 0)
#define GELIC_NET_SET_MODE              (SIOCDEVPRIVATE + 1)

/* descriptors */
#define GELIC_NET_RX_DESCRIPTORS        128 /* num of descriptors */
#define GELIC_NET_TX_DESCRIPTORS        128 /* num of descriptors */

#define GELIC_NET_MAX_MTU               2308
#define GELIC_NET_MIN_MTU               64
#define GELIC_NET_RXBUF_ALIGN           128
#define GELIC_NET_RX_CSUM_DEFAULT       1 /* hw chksum */
#define GELIC_NET_WATCHDOG_TIMEOUT      5*HZ
#define GELIC_NET_NAPI_WEIGHT           64
#define GELIC_NET_BROADCAST_ADDR        0xffffffffffff
#define GELIC_NET_VLAN_POS              (VLAN_ETH_ALEN * 2)
#define GELIC_NET_VLAN_MAX              4
#define GELIC_NET_MC_COUNT_MAX          32 /* multicast address list */

enum gelic_net_int0_status {
	GELIC_NET_GDTDCEINT  = 24,
	GELIC_NET_GRFANMINT  = 28,
};

/* GHIINT1STS bits */
enum gelic_net_int1_status {
	GELIC_NET_GDADCEINT = 14,
};

/* interrupt mask */
#define GELIC_NET_TXINT                   (1L << (GELIC_NET_GDTDCEINT + 32))

#define GELIC_NET_RXINT0                  (1L << (GELIC_NET_GRFANMINT + 32))
#define GELIC_NET_RXINT1                  (1L << GELIC_NET_GDADCEINT)
#define GELIC_NET_RXINT                   (GELIC_NET_RXINT0 | GELIC_NET_RXINT1)

 /* descriptor data_status bits */
#define GELIC_NET_RXIPCHK                 29
#define GELIC_NET_TCPUDPIPCHK             28
#define GELIC_NET_DATA_STATUS_CHK_MASK    (1 << GELIC_NET_RXIPCHK | \
                                           1 << GELIC_NET_TCPUDPIPCHK)

/* descriptor data_error bits */
#define GELIC_NET_RXIPCHKERR              27
#define GELIC_NET_RXTCPCHKERR             26
#define GELIC_NET_DATA_ERROR_CHK_MASK     (1 << GELIC_NET_RXIPCHKERR | \
                                           1 << GELIC_NET_RXTCPCHKERR)

#define GELIC_NET_DMAC_CMDSTAT_NOCS       0xa0080000 /* middle of frame */
#define GELIC_NET_DMAC_CMDSTAT_TCPCS      0xa00a0000
#define GELIC_NET_DMAC_CMDSTAT_UDPCS      0xa00b0000
#define GELIC_NET_DMAC_CMDSTAT_END_FRAME  0x00040000 /* end of frame */

#define GELIC_NET_DMAC_CMDSTAT_CHAIN_END  0x00000002 /* RXDCEIS:DMA stopped */

#define GELIC_NET_DESCR_IND_PROC_SHIFT    28
#define GELIC_NET_DESCR_IND_PROC_MASKO    0x0fffffff

/* ignore ipsec ans multicast */
#define GELIC_NET_DATA_ERROR_MASK         0xfdefbfff
/* ignore unmatched sp on sp, drop_packet, multicast address frame*/
#define GELIC_NET_DATA_ERROR_FLG          0x7def8000

enum gelic_net_descr_status {
	GELIC_NET_DESCR_COMPLETE            = 0x00, /* used in rx and tx */
	GELIC_NET_DESCR_RESPONSE_ERROR      = 0x01, /* used in rx and tx */
	GELIC_NET_DESCR_PROTECTION_ERROR    = 0x02, /* used in rx and tx */
	GELIC_NET_DESCR_FRAME_END           = 0x04, /* used in rx */
	GELIC_NET_DESCR_FORCE_END           = 0x05, /* used in rx and tx */
	GELIC_NET_DESCR_CARDOWNED           = 0x0a, /* used in rx and tx */
	GELIC_NET_DESCR_NOT_IN_USE                  /* any other value */
};
#define GELIC_NET_DMAC_CMDSTAT_NOT_IN_USE 0xb0000000

#define GELIC_NET_DESCR_SIZE              32
struct gelic_net_descr {
	/* as defined by the hardware */
	uint32_t buf_addr;
	uint32_t buf_size;
	uint32_t next_descr_addr;
	uint32_t dmac_cmd_status;
	uint32_t result_size;
	uint32_t valid_size;	/* all zeroes for tx */
	uint32_t data_status;
	uint32_t data_error;	/* all zeroes for tx */

	/* used in the driver */
	struct sk_buff *skb;
	dma_addr_t bus_addr;
	struct gelic_net_descr *next;
	struct gelic_net_descr *prev;
	struct vlan_ethhdr vlan;
} __attribute__((aligned(32)));

struct gelic_net_descr_chain {
	/* we walk from tail to head */
	struct gelic_net_descr *head;
	struct gelic_net_descr *tail;
	spinlock_t lock;
};

struct gelic_net_card {
	struct net_device *netdev;
	uint64_t ghiintmask;
	struct ps3_system_bus_device *dev;
	uint32_t vlan_id[GELIC_NET_VLAN_MAX];
	int vlan_index;

	struct gelic_net_descr_chain tx_chain;
	struct gelic_net_descr_chain rx_chain;
	spinlock_t chain_lock;

	struct net_device_stats netdev_stats;
	int rx_csum;
	spinlock_t intmask_lock;

	struct work_struct tx_timeout_task;
	atomic_t tx_timeout_task_counter;
	wait_queue_head_t waitq;

	struct gelic_net_descr *tx_top, *rx_top;
#ifdef CONFIG_GELIC_WIRELESS
	struct gelic_wireless w;
#endif
	struct gelic_net_descr descr[0];
};

/* for lv1_net_control */
#define GELIC_NET_GET_MAC_ADDRESS               0x0000000000000001
#define GELIC_NET_GET_ETH_PORT_STATUS           0x0000000000000002
#define GELIC_NET_SET_NEGOTIATION_MODE          0x0000000000000003
#define GELIC_NET_GET_VLAN_ID                   0x0000000000000004

#define GELIC_NET_LINK_UP                       0x0000000000000001
#define GELIC_NET_FULL_DUPLEX                   0x0000000000000002
#define GELIC_NET_AUTO_NEG                      0x0000000000000004
#define GELIC_NET_SPEED_10                      0x0000000000000010
#define GELIC_NET_SPEED_100                     0x0000000000000020
#define GELIC_NET_SPEED_1000                    0x0000000000000040

#define GELIC_NET_VLAN_ALL                      0x0000000000000001
#define GELIC_NET_VLAN_WIRED                    0x0000000000000002
#define GELIC_NET_VLAN_WIRELESS                 0x0000000000000003
#define GELIC_NET_VLAN_PSP                      0x0000000000000004
#define GELIC_NET_VLAN_PORT0                    0x0000000000000010
#define GELIC_NET_VLAN_PORT1                    0x0000000000000011
#define GELIC_NET_VLAN_PORT2                    0x0000000000000012
#define GELIC_NET_VLAN_DAEMON_CLIENT_BSS        0x0000000000000013
#define GELIC_NET_VLAN_LIBERO_CLIENT_BSS        0x0000000000000014
#define GELIC_NET_VLAN_NO_ENTRY                 -6

#define GELIC_NET_PORT                          2 /* for port status */

#endif //_GELIC_NET_H
