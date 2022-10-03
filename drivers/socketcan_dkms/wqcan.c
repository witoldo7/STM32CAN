// SPDX-License-Identifier: GPL-2.0-only

/* SocketCAN driver for WQCAN.
 *
 * Copyright (C) 2022 Witold Olechowski
 *
 * This driver is inspired by the net/can/usb/mcba_usb.c
 */

#include <asm/unaligned.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/bitfield.h>

/* vendor and product id */
#define WQCAN_MODULE_NAME "wqcan"
#define WQCAN_VENDOR_ID 0xffff
#define WQCAN_PRODUCT_ID 0x0005

/* driver constants */
#define WQCAN_MAX_RX_URBS 20
#define WQCAN_MAX_TX_URBS 20
#define WQCAN_CTX_FREE WQCAN_MAX_TX_URBS

/* RX buffer must be bigger than msg size since at the
 * beginning USB messages are stacked.
 */
#define WQCAN_USB_RX_BUFF_SIZE 64 * 2

/* WQCAN endpoint numbers */
#define WQCAN_USB_EP_IN 2
#define WQCAN_USB_EP_OUT 2

/* WQCAN command id */
#define WQCAN_CMD_READ_FW_VERSION 0x20

#define WQCAN_CMD_SWCAN_OPEN 0x60
#define WQCAN_CMD_SWCAN_BIT_RATE 0x61
#define WQCAN_CMD_SWCAN_RECEIVE_MESSAGE 0x62
#define WQCAN_CMD_SWCAN_TRANSMIT_MESSAGE 0x63

#define WQCAN_CMD_CAN_OPEN 0x80
#define WQCAN_CMD_CAN_BIT_RATE 0x81
#define WQCAN_CMD_CAN_RECEIVE_MESSAGE 0x85
#define WQCAN_CMD_CAN_TRANSMIT_MESSAGE 0x86

/* Data Bit Timing & Prescaler Register (DBTP) */
#define DBTP_TDC		BIT(23)
#define DBTP_DBRP_MASK		GENMASK(20, 16)
#define DBTP_DTSEG1_MASK	GENMASK(12, 8)
#define DBTP_DTSEG2_MASK	GENMASK(7, 4)
#define DBTP_DSJW_MASK		GENMASK(3, 0)

/* Nominal Bit Timing & Prescaler Register (NBTP) */
#define NBTP_NSJW_MASK		GENMASK(31, 25)
#define NBTP_NBRP_MASK		GENMASK(24, 16)
#define NBTP_NTSEG1_MASK	GENMASK(15, 8)
#define NBTP_NTSEG2_MASK	GENMASK(6, 0)

/* Transmitter Delay Compensation Register (TDCR) */
#define TDCR_TDCO_MASK		GENMASK(14, 8)
#define TDCR_TDCF_MASK		GENMASK(6, 0)

static const struct can_bittiming_const wqcan_can_fd_bit_timing_max = {
	.name = "wqcan_can_fd",
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 256,
	.tseg2_min = 2,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 1,
};

static const struct can_bittiming_const wqcan_can_fd_bit_timing_data_max = {
	.name = "wqcan_can_fd",
	.tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 32,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

struct wqcan_usb_ctx {
	struct wqcan_priv *priv;
	u32 ndx;
	bool can;
};

struct wqcan_dev_priv {
  struct usb_device *udev;
  struct device *dev;
  struct usb_anchor rx_submitted;
  struct wqcan_priv *interfaces[2];
};

/* Structure to hold all of our device specific stuff */
struct wqcan_priv {
	struct can_priv can; /* must be the first member */
	struct wqcan_usb_ctx tx_context[WQCAN_MAX_TX_URBS];
	struct net_device *netdev;
	struct usb_anchor tx_submitted;
	struct can_berr_counter bec;
	atomic_t free_ctx_cnt;
	u8 interface_num;
	u8 mcu_can_ifnum;
	struct wqcan_dev_priv *priv_dev;
};

/* CAN RX frame */
/* data[0:3] - SID or EID
 * data[4]
 *    b0 - XTD
 *    b1 - RTR
 *    b2 - ESI
 *    b3 - BRS
 *    b4 - FDF
 *    b4 - ANMF
 * data[5] - FIDX
 * data[6:7] - RXTS
 * data[8] - DLC
 * data[9:9+DLC] - data
 */
struct __packed wqcan_usb_msg_can_rx {
	u8 cmd_id;
	u16 len;
	u8 SID[4];
	u8 CFG;
	u8 FIDX;
	u16 RXTS;
	u8 DLC;
	u8 data[64];
	u8 term;
};

/* CAN TX frame */
  /* data[0:3] - SID or EID
   * data[4]
   *    b0 - XTD
   *    b1 - RTR
   *    b2 - ESI
   *    b3 - BPS
   *    b4 - FDF
   *    b5 - EFC
   * data[5] - MM
   * data[6] - DLC
   * data[7:7+DLC] - data
   */
struct __packed wqcan_usb_msg_can_tx {
	u8 cmd_id;
	u16 len;
	u8 SID[4];
	u8 CFG;
	u8 MM;
	u8 DLC;
	u8 data[64];
	u8 term;
};

struct __packed wqcan_usb_msg_swcan_rx {
	u8 cmd_id;
	u16 len;
	u8 SID[4];
	u8 data[8];
	u8 DLC;
	u8 XTD;
	u8 RTR;
	u8 term;
};

struct __packed wqcan_usb_msg_swcan_tx {
	u8 cmd_id;
	u16 len;
	u8 SID[4];
	u8 data[8];
	u8 DLC;
	u8 XTD;
	u8 RTR;
	u8 term;
};

/* command frame */
struct __packed wqcan_usb_msg {
	u8 cmd_id;
	u8 unused[80];
};

struct __packed wqcan_usb_msg_ka_usb {
	u8 cmd_id;
	u16 len;
	u8 soft_ver_major;
	u8 soft_ver_minor;
	u8 term;
};

struct __packed wqcan_usb_msg_can_bitrate {
	u8 cmd_id;
	u16 len;
	u8 cmd;
	u32 reg0;
	u32 reg1;
	u8 term;
};

struct __packed wqcan_usb_msg_swcan_bitrate {
	u8 cmd_id;
	u16 len;
	u32 bitrate;
	u8 term;
};

struct __packed wqcan_usb_msg_fw_ver {
	u8 cmd_id;
	u16 len;
	u8 term;
};

struct __packed wqcan_usb_msg_can_open {
	u8 cmd_id;
	u16 len;
	u8 data;
	u32 mode;
	u8 term;
};

static const struct usb_device_id wqcan_usb_table[] = {
	{ USB_DEVICE(WQCAN_VENDOR_ID, WQCAN_PRODUCT_ID) },
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, wqcan_usb_table);

static const u32 wqcan_bitrate[] = { 33333, 83333 };

static inline void wqcan_init_ctx(struct wqcan_priv *priv)
{
	int i = 0;

	for (i = 0; i < WQCAN_MAX_TX_URBS; i++) {
		priv->tx_context[i].ndx = WQCAN_CTX_FREE;
		priv->tx_context[i].priv = priv;
	}

	atomic_set(&priv->free_ctx_cnt, ARRAY_SIZE(priv->tx_context));
}

static inline struct wqcan_usb_ctx *wqcan_usb_get_free_ctx(struct wqcan_priv *priv,
							 struct canfd_frame *cf)
{
	int i = 0;
	struct wqcan_usb_ctx *ctx = NULL;

	for (i = 0; i < WQCAN_MAX_TX_URBS; i++) {
		if (priv->tx_context[i].ndx == WQCAN_CTX_FREE) {
			ctx = &priv->tx_context[i];
			ctx->ndx = i;

			if (cf)
				ctx->can = true;
			else
				ctx->can = false;

			atomic_dec(&priv->free_ctx_cnt);
			break;
		}
	}

	if (!atomic_read(&priv->free_ctx_cnt))
		/* That was the last free ctx. Slow down tx path */
		netif_stop_queue(priv->netdev);

	return ctx;
}

/* wqcan_usb_free_ctx and wqcan_usb_get_free_ctx are executed by different
 * threads. The order of execution in below function is important.
 */
static inline void wqcan_usb_free_ctx(struct wqcan_usb_ctx *ctx)
{
	/* Increase number of free ctxs before freeing ctx */
	atomic_inc(&ctx->priv->free_ctx_cnt);

	ctx->ndx = WQCAN_CTX_FREE;

	/* Wake up the queue once ctx is marked free */
	netif_wake_queue(ctx->priv->netdev);
}

static void wqcan_usb_write_bulk_callback(struct urb *urb)
{
	struct wqcan_usb_ctx *ctx = urb->context;
	struct net_device *netdev;

	WARN_ON(!ctx);

	netdev = ctx->priv->netdev;

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);

	if (ctx->can) {
		if (!netif_device_present(netdev))
			return;

		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += can_get_echo_skb(netdev, ctx->ndx,
							   NULL);

	}

	if (urb->status)
		netdev_info(netdev, "Tx URB aborted (%d)\n", urb->status);

	/* Release the context */
	wqcan_usb_free_ctx(ctx);
}

/* Send data to device */
static netdev_tx_t wqcan_usb_xmit(struct wqcan_priv *priv,
				 struct wqcan_usb_msg *usb_msg,
				 struct wqcan_usb_ctx *ctx, u16 size)
{
	struct urb *urb;
	u8 *buf;
	int err;

	/* create a URB, and a buffer for it, and copy the data to the URB */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		return -ENOMEM;

	buf = usb_alloc_coherent(priv->priv_dev->udev, size, GFP_ATOMIC,
				 &urb->transfer_dma);
	if (!buf) {
		err = -ENOMEM;
		goto nomembuf;
	}

	memcpy(buf, usb_msg, size);

	usb_fill_bulk_urb(urb, priv->priv_dev->udev,
			  usb_sndbulkpipe(priv->priv_dev->udev, WQCAN_USB_EP_OUT), buf,
			  size, wqcan_usb_write_bulk_callback,
			  ctx);

	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &priv->tx_submitted);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (unlikely(err))
		goto failed;

	/* Release our reference to this URB, the USB core will eventually free
	 * it entirely.
	 */
	usb_free_urb(urb);

	return 0;

failed:
	usb_unanchor_urb(urb);
	usb_free_coherent(priv->priv_dev->udev, size, buf,
			  urb->transfer_dma);

	if (err == -ENODEV)
		netif_device_detach(priv->netdev);
	else
		netdev_warn(priv->netdev, "failed tx_urb %d\n", err);

nomembuf:
	usb_free_urb(urb);

	return err;
}

/* Send data to device */
static netdev_tx_t wqcan_usb_start_xmit(struct sk_buff *skb,
				       struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	struct wqcan_usb_ctx *ctx = NULL;
	struct net_device_stats *stats = &priv->netdev->stats;
	int err;
	u8 flags = 0; 
	
	struct wqcan_usb_msg_can_tx usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_TRANSMIT_MESSAGE,
	};

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	ctx = wqcan_usb_get_free_ctx(priv, cf);
	if (!ctx)
		return NETDEV_TX_BUSY;

	if (cf->can_id & CAN_EFF_FLAG) {
		u32 eid = cf->can_id & CAN_EFF_MASK;
		usb_msg.SID[0] = eid & 0xFF; 
		usb_msg.SID[1] = (eid >> 8) & 0xFF;
		usb_msg.SID[2] = (eid >> 16) & 0xFF;
		usb_msg.SID[3] = (eid >> 24) & 0xFF;
		flags |= 0b00000001;
	} else {
		u32 sid = cf->can_id & CAN_SFF_MASK;
		usb_msg.SID[0] = sid & 0xFF; 
		usb_msg.SID[1] = (sid >> 8) & 0xFF;
		usb_msg.SID[2] = (sid >> 16) & 0xFF;
		usb_msg.SID[3] = 0;
	}

	if (cf->can_id & CAN_RTR_FLAG)
	{
		flags |= 0b00000010;
	}
	
	if (cf->flags & CANFD_ESI)
	{
		flags |= 0b00010000;
	}
	
	if (can_is_canfd_skb(skb)) 
	{
		flags |= 0b00010000;
		if (cf->flags & CANFD_BRS)
		flags |= 0b00001000;
	}

	usb_msg.CFG = flags;
	usb_msg.DLC = can_fd_len2dlc(cf->len);
	memcpy(usb_msg.data, cf->data, cf->len);

	can_put_echo_skb(skb, priv->netdev, ctx->ndx, 0);

	err = wqcan_usb_xmit(priv, (struct wqcan_usb_msg *)&usb_msg, ctx, sizeof(usb_msg));
	if (err)
		goto xmit_failed;

	return NETDEV_TX_OK;

xmit_failed:
	can_free_echo_skb(priv->netdev, ctx->ndx, NULL);
	wqcan_usb_free_ctx(ctx);
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

/* Send data to device */
static netdev_tx_t wqcan_usb_start_xmit_sw(struct sk_buff *skb,
				       struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	struct wqcan_usb_ctx *ctx = NULL;
	struct net_device_stats *stats = &priv->netdev->stats;
	int err;

	struct wqcan_usb_msg_swcan_tx usb_msg = {
		.cmd_id = WQCAN_CMD_SWCAN_TRANSMIT_MESSAGE,
	};

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	ctx = wqcan_usb_get_free_ctx(priv, cf);
	if (!ctx)
		return NETDEV_TX_BUSY;

	if (cf->can_id & CAN_EFF_FLAG) {
		u32 eid = cf->can_id & CAN_EFF_MASK;
		usb_msg.SID[0] = eid & 0xFF;
		usb_msg.SID[1] = (eid >> 8) & 0xFF;
		usb_msg.SID[2] = (eid >> 16) & 0xFF;
		usb_msg.SID[3] = (eid >> 24) & 0xFF;
		usb_msg.XTD = 1;
	} else {
		u32 sid = cf->can_id & CAN_SFF_MASK;
		usb_msg.SID[0] = sid & 0xFF;
		usb_msg.SID[1] = (sid >> 8) & 0xFF;
		usb_msg.SID[2] = (sid >> 16) & 0xFF;
		usb_msg.SID[3] = 0;
	}

	if (cf->can_id & CAN_RTR_FLAG)
	{
		usb_msg.RTR = 1;
	}

	usb_msg.DLC = can_fd_len2dlc(cf->len);
	memcpy(usb_msg.data, cf->data, cf->len);

	can_put_echo_skb(skb, priv->netdev, ctx->ndx, 0);

	err = wqcan_usb_xmit(priv, (struct wqcan_usb_msg *)&usb_msg, ctx, sizeof(usb_msg));
	if (err)
		goto xmit_failed;

	return NETDEV_TX_OK;

xmit_failed:
	can_free_echo_skb(priv->netdev, ctx->ndx, NULL);
	wqcan_usb_free_ctx(ctx);
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

/* Send cmd to device */
static void wqcan_usb_xmit_cmd(struct wqcan_priv *priv,
			      struct wqcan_usb_msg *usb_msg, u16 size)
{
	struct wqcan_usb_ctx *ctx = NULL;
	int err;

	ctx = wqcan_usb_get_free_ctx(priv, NULL);
	if (!ctx) {
		netdev_err(priv->netdev,
			   "Lack of free ctx. Sending (%d) cmd aborted",
			   usb_msg->cmd_id);

		return;
	}

	err = wqcan_usb_xmit(priv, usb_msg, ctx, size);
	if (err)
		netdev_err(priv->netdev, "Failed to send cmd (%d)",
			   usb_msg->cmd_id);
}

static void wqcan_usb_xmit_change_bitrate(struct wqcan_priv *priv, u8 cmd, u32 reg_btp, u32 reg_tdcr)
{
	struct wqcan_usb_msg_can_bitrate usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_BIT_RATE,
		.cmd = cmd,
		.term = 0
	};
	if (cmd == 2) 
	{
		put_unaligned_be16(9, &usb_msg.len);
		put_unaligned_be32(reg_tdcr, &usb_msg.reg1);
	} 
	else 
	{
		put_unaligned_be16(5, &usb_msg.len);
	}

	put_unaligned_be32(reg_btp, &usb_msg.reg0);

	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_xmit_change_bitrate_sw(struct wqcan_priv *priv, u32 bitrate)
{
	struct wqcan_usb_msg_swcan_bitrate usb_msg = {
		.cmd_id = WQCAN_CMD_SWCAN_BIT_RATE,
		.term = 0
	};
	put_unaligned_be16(4, &usb_msg.len);

	put_unaligned_be32(bitrate, &usb_msg.bitrate);

	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_xmit_read_fw_ver(struct wqcan_priv *priv)
{
	struct wqcan_usb_msg_fw_ver usb_msg = {
		.cmd_id = WQCAN_CMD_READ_FW_VERSION,
		.len = 0,
		.term = 0
	};

	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_can_open(struct wqcan_priv *priv, u8 cmd)
{
	u32 mode = priv->can.ctrlmode;
	struct wqcan_usb_msg_can_open usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_OPEN,
		.data = cmd,
		.term = 0
	};
	put_unaligned_be16(5, &usb_msg.len);
	put_unaligned_be32(mode, &usb_msg.mode);

	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_can_open_sw(struct wqcan_priv *priv)
{
	u32 mode = priv->can.ctrlmode;
	struct wqcan_usb_msg_can_open usb_msg = {
		.cmd_id = WQCAN_CMD_SWCAN_OPEN,
		.data = 3,
		.term = 0
	};
	put_unaligned_be16(5, &usb_msg.len);
	put_unaligned_be32(mode, &usb_msg.mode);

	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_can_close(struct wqcan_priv *priv)
{
	struct wqcan_usb_msg_can_open usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_OPEN,
		.data = 0,
		.term = 0
	};
	put_unaligned_be16(1, &usb_msg.len);
	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_swcan_close(struct wqcan_priv *priv)
{
	struct wqcan_usb_msg_can_open usb_msg = {
		.cmd_id = WQCAN_CMD_SWCAN_OPEN,
		.data = 0,
		.term = 0
	};
	put_unaligned_be16(1, &usb_msg.len);
	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}


static void wqcan_usb_process_can(struct wqcan_priv *priv,
				 struct wqcan_usb_msg_can_rx *msg)
{
	struct canfd_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;

	if ((msg->CFG >> 4) & 0x1) 
		skb = alloc_canfd_skb(priv->netdev, &cf);
	else
		skb = alloc_can_skb(priv->netdev, (struct can_frame **)&cf);
	
	if (!skb)
	{
		stats->rx_dropped++;
		return;
	}

	if (msg->CFG & 0x1) 
	{
		u32 eid = msg->SID[0] | (u32)(msg->SID[1] << 8)
				| (u32)(msg->SID[2] << 16) | (u32)(msg->SID[3] << 24);
		cf->can_id = CAN_EFF_FLAG;
		cf->can_id |= eid;
	} 
	else 
	{
		u16 sid = msg->SID[0] | (u16)(msg->SID[1] << 8) | (u16)(msg->SID[2] << 16);
		cf->can_id = sid & CAN_SFF_MASK;
	}

	if ((msg->CFG >> 4) & 0x1) 
		cf->len = can_fd_dlc2len(msg->DLC);
	else
		cf->len = can_cc_dlc2len(msg->DLC);
	
	if ((msg->CFG >> 2) & 0x1) {	
		cf->flags |= CANFD_ESI;
		netdev_dbg(priv->netdev, "ESI Error\n");
	}

	if ((msg->CFG >> 1) & 0x1) 
	{
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		memcpy(cf->data, msg->data, cf->len);
		stats->rx_bytes += cf->len;
	}

	stats->rx_packets++;
	netif_rx(skb);
}

static void wqcan_usb_process_swcan(struct wqcan_priv *priv,
				 struct wqcan_usb_msg_swcan_rx *msg)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;

	skb = alloc_can_skb(priv->netdev, (struct can_frame **)&cf);

	if (!skb)
	{
		stats->rx_dropped++;
		return;
	}

	if (msg->XTD)
	{
		u32 eid = msg->SID[0] | (u32)(msg->SID[1] << 8)
				| (u32)(msg->SID[2] << 16) | (u32)(msg->SID[3] << 24);
		cf->can_id = CAN_EFF_FLAG;
		cf->can_id |= eid;
	}
	else
	{
		u16 sid = msg->SID[0] | (u16)(msg->SID[1] << 8) | (u16)(msg->SID[2] << 16);
		cf->can_id = sid & CAN_SFF_MASK;
	}

	cf->len = can_cc_dlc2len(msg->DLC);

	if (msg->RTR)
	{
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		memcpy(cf->data, msg->data, cf->len);
		stats->rx_bytes += cf->len;
	}

	stats->rx_packets++;
	netif_rx(skb);
}


static void wqcan_usb_process_ka_usb(struct wqcan_priv *priv,
				    struct wqcan_usb_msg_ka_usb *msg)
{
	netdev_info(priv->netdev, "WQCAN USB version %u.%u\n",
			    msg->soft_ver_major, msg->soft_ver_minor);
}

static void wqcan_usb_process_rx(struct wqcan_dev_priv *priv,
				struct wqcan_usb_msg *msg)
{
	switch (msg->cmd_id) {
	case WQCAN_CMD_READ_FW_VERSION:
		wqcan_usb_process_ka_usb(priv->interfaces[0],
					(struct wqcan_usb_msg_ka_usb *)msg);
		wqcan_usb_process_ka_usb(priv->interfaces[1],
					(struct wqcan_usb_msg_ka_usb *)msg);
		break;

	case WQCAN_CMD_CAN_RECEIVE_MESSAGE:
		wqcan_usb_process_can(priv->interfaces[0], (struct wqcan_usb_msg_can_rx *)msg);
		break;

	case WQCAN_CMD_SWCAN_RECEIVE_MESSAGE:
		wqcan_usb_process_swcan(priv->interfaces[1], (struct wqcan_usb_msg_swcan_rx *)msg);
		break;

	case WQCAN_CMD_SWCAN_OPEN:
	case WQCAN_CMD_SWCAN_BIT_RATE:
	case WQCAN_CMD_CAN_OPEN:
	case WQCAN_CMD_CAN_BIT_RATE:
		break;

	default:
		dev_err(priv->dev, "Unsupported msg (0x%X)", msg->cmd_id);
		break;
	}
}

/* Callback for reading data from device
 *
 * Check urb status, call read function and resubmit urb read operation.
 */
static void wqcan_usb_read_bulk_callback(struct urb *urb)
{
	struct wqcan_dev_priv *priv = urb->context;
	int retval;
	int pos = 0;

	switch (urb->status) {
	case 0: /* success */
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		return;

	default:
		dev_info(priv->dev, "Rx URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}

	while (pos < urb->actual_length) {
		struct wqcan_usb_msg *msg;

		msg = (struct wqcan_usb_msg *)(urb->transfer_buffer + pos);
		wqcan_usb_process_rx(priv, msg);

		pos += sizeof(struct wqcan_usb_msg);
	}

resubmit_urb:

	usb_fill_bulk_urb(urb, priv->udev,
			  usb_rcvbulkpipe(priv->udev, WQCAN_USB_EP_OUT),
			  urb->transfer_buffer, WQCAN_USB_RX_BUFF_SIZE,
			  wqcan_usb_read_bulk_callback, priv);

	retval = usb_submit_urb(urb, GFP_ATOMIC);

	if (retval == -ENODEV)
	{
		if (priv->interfaces[0])
			netif_device_detach(priv->interfaces[0]->netdev);
		else
			netif_device_detach(priv->interfaces[0]->netdev);
	}
	else if (retval)
		dev_err(priv->dev, "failed resubmitting read bulk urb: %d\n", retval);
}

/* Start USB device */
static int wqcan_usb_start(struct wqcan_dev_priv *priv)
{
	int err, i;

	wqcan_init_ctx(priv->interfaces[0]);
	wqcan_init_ctx(priv->interfaces[1]);

	for (i = 0; i < WQCAN_MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf;
		dma_addr_t buf_dma;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			err = -ENOMEM;
			break;
		}

		buf = usb_alloc_coherent(priv->udev, WQCAN_USB_RX_BUFF_SIZE,
					 GFP_KERNEL, &buf_dma);
		if (!buf) {
			dev_err(priv->dev, "No memory left for USB buffer\n");
			usb_free_urb(urb);
			err = -ENOMEM;
			break;
		}

		urb->transfer_dma = buf_dma;

		usb_fill_bulk_urb(urb, priv->udev,
				  usb_rcvbulkpipe(priv->udev, WQCAN_USB_EP_IN),
				  buf, WQCAN_USB_RX_BUFF_SIZE,
				  wqcan_usb_read_bulk_callback, priv);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &priv->rx_submitted);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			usb_unanchor_urb(urb);
			usb_free_coherent(priv->udev, WQCAN_USB_RX_BUFF_SIZE,
					  buf, buf_dma);
			usb_free_urb(urb);
			break;
		}

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
	}

	/* Did we submit any URBs */
	if (i == 0) {
		dev_err(priv->dev, "couldn't setup read URBs\n");
		return err;
	}

	/* Warn if we've couldn't transmit all the URBs */
	if (i < WQCAN_MAX_RX_URBS)
		dev_err(priv->dev, "rx performance may be slow\n");

	wqcan_usb_xmit_read_fw_ver(priv->interfaces[0]);

	return err;
}

/* Open USB device */
static int wqcan_usb_open(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	int err;
	wqcan_usb_can_open(priv, 2);
	/* common open */

	err = open_candev(netdev);
	if (err)
		return err;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	netif_start_queue(netdev);

	return 0;
}

static int wqcan_usb_open_sw(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	int err;
	wqcan_usb_can_open_sw(priv);

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	netif_start_queue(netdev);

	return 0;
}

static void wqcan_urb_unlink(struct wqcan_priv *priv)
{
	usb_kill_anchored_urbs(&priv->priv_dev->rx_submitted);
	usb_kill_anchored_urbs(&priv->tx_submitted);
}

/* Close USB device */
static int wqcan_usb_close(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	wqcan_usb_can_close(priv);
	priv->can.state = CAN_STATE_STOPPED;

	netif_stop_queue(netdev);

	/* Stop polling */
	wqcan_urb_unlink(priv);

	close_candev(netdev);

	return 0;
}

/* Close USB device */
static int wqcan_usb_close_sw(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	wqcan_usb_swcan_close(priv);
	priv->can.state = CAN_STATE_STOPPED;

	netif_stop_queue(netdev);

	/* Stop polling */
	wqcan_urb_unlink(priv);

	close_candev(netdev);

	return 0;
}


static int wqcan_net_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		wqcan_usb_can_open(priv, 2);
		break;
	case CAN_MODE_STOP:
		wqcan_usb_can_close(priv);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

static int wqcan_net_get_berr_counter(const struct net_device *netdev,
				     struct can_berr_counter *bec)
{
	struct wqcan_priv *priv = netdev_priv(netdev);

	bec->txerr = priv->bec.txerr;
	bec->rxerr = priv->bec.rxerr;

	return 0;
}

static const struct net_device_ops wqcan_netdev_ops = {
	.ndo_open = wqcan_usb_open,
	.ndo_stop = wqcan_usb_close,
	.ndo_start_xmit = wqcan_usb_start_xmit,
};

static const struct net_device_ops wqcan_netdev_ops_sw = {
	.ndo_open = wqcan_usb_open_sw,
	.ndo_stop = wqcan_usb_close_sw,
	.ndo_start_xmit = wqcan_usb_start_xmit_sw,
};

static int wqcan_net_set_bittiming(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u16 brp, sjw, tseg1, tseg2;
	u32 reg_btp;

	brp = bt->brp - 1;
	sjw = bt->sjw - 1;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;
	reg_btp = FIELD_PREP(NBTP_NBRP_MASK, brp) |
		  FIELD_PREP(NBTP_NSJW_MASK, sjw) |
		  FIELD_PREP(NBTP_NTSEG1_MASK, tseg1) |
		  FIELD_PREP(NBTP_NTSEG2_MASK, tseg2);

	wqcan_usb_xmit_change_bitrate(priv, 0, reg_btp, 0);

	return 0;
}

static int wqcan_net_set_data_bittiming(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	const struct can_bittiming *dbt = &priv->can.data_bittiming;
	u16 brp, sjw, tseg1, tseg2;
	u32 reg_btp, reg_tdcr;
	u8 cmd = 1;

	brp = dbt->brp - 1;
	sjw = dbt->sjw - 1;
	tseg1 = dbt->prop_seg + dbt->phase_seg1 - 1;
	tseg2 = dbt->phase_seg2 - 1;

	/* TDC is only needed for bitrates beyond 2.5 MBit/s.
		* This is mentioned in the "Bit Time Requirements for CAN FD"
		* paper presented at the International CAN Conference 2013
		*/
	if (dbt->bitrate > 2500000) {
		u32 tdco, ssp;

		/* Use the same value of secondary sampling point
			* as the data sampling point
			*/
		ssp = dbt->sample_point;

		/* Equation based on Bosch's M_CAN User Manual's
			* Transmitter Delay Compensation Section
			*/
		tdco = (priv->can.clock.freq / 1000) *
			ssp / dbt->bitrate;

		/* Max valid TDCO value is 127 */
		if (tdco > 127) {
			netdev_warn(netdev, "TDCO value of %u is beyond maximum. Using maximum possible value\n",
					tdco);
			tdco = 127;
		}

		reg_btp |= DBTP_TDC;
		reg_tdcr = FIELD_PREP(TDCR_TDCO_MASK, tdco);
		cmd = 2;
	}

	reg_btp |= FIELD_PREP(DBTP_DBRP_MASK, brp) |
		FIELD_PREP(DBTP_DSJW_MASK, sjw) |
		FIELD_PREP(DBTP_DTSEG1_MASK, tseg1) |
		FIELD_PREP(DBTP_DTSEG2_MASK, tseg2);

	wqcan_usb_xmit_change_bitrate(priv, cmd, reg_btp, reg_tdcr);
	return 0;
}

static int wqcan_net_set_bittiming_sw(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	const u32 bitrate_kbps = priv->can.bittiming.bitrate;

	wqcan_usb_xmit_change_bitrate_sw(priv, bitrate_kbps);

	return 0;
}

static int wqcan_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct net_device *netdev, *netdev1;
	struct wqcan_priv *priv, *priv1;
	struct wqcan_dev_priv *priv_dev;
	int err;
	struct usb_device *usbdev = interface_to_usbdev(intf);
	dev_info(&intf->dev, "WQCAN probe\n");

	priv_dev = kzalloc(sizeof(struct wqcan_dev_priv), GFP_KERNEL);
	if (!priv_dev) {
		dev_err(&intf->dev, "Couldn't alloc priv_dev\n");
		return -ENOMEM;
	}
	priv_dev->udev = usbdev;
	priv_dev->dev = &intf->dev;
	usb_set_intfdata(intf, priv_dev);

	netdev = alloc_candev(sizeof(struct wqcan_priv), WQCAN_MAX_TX_URBS);
	netdev1 = alloc_candev(sizeof(struct wqcan_priv), WQCAN_MAX_TX_URBS);
	if (!netdev || !netdev1) {
		dev_err(&intf->dev, "Couldn't alloc candev\n");
		return -ENOMEM;
	}

	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->priv_dev = priv_dev;
	priv->interface_num = 0;
	init_usb_anchor(&priv_dev->rx_submitted);
	init_usb_anchor(&priv->tx_submitted);

	/* Init HSCAN device */
	priv->can.state = CAN_STATE_STOPPED;
	priv->can.bittiming_const = &wqcan_can_fd_bit_timing_max;
	priv->can.data_bittiming_const = &wqcan_can_fd_bit_timing_data_max;
	priv->can.do_set_mode = wqcan_net_set_mode;
	priv->can.do_get_berr_counter = wqcan_net_get_berr_counter;
	priv->can.do_set_bittiming = wqcan_net_set_bittiming;
	priv->can.do_set_data_bittiming = wqcan_net_set_data_bittiming;
	priv->can.clock.freq = 80000000;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK
					| CAN_CTRLMODE_LISTENONLY
					| CAN_CTRLMODE_FD
					| CAN_CTRLMODE_BERR_REPORTING
					| CAN_CTRLMODE_FD_NON_ISO
					| CAN_CTRLMODE_ONE_SHOT
					| CAN_CTRLMODE_TDC_AUTO;

	netdev->netdev_ops = &wqcan_netdev_ops;
	netdev->flags |= IFF_ECHO; /* we support local echo */

	SET_NETDEV_DEV(netdev, &intf->dev);

	err = register_candev(netdev);
	if (err) {
		netdev_err(netdev, "couldn't register CAN device: %d\n", err);
		goto cleanup_free_candev;
	}
	priv_dev->interfaces[0] = priv;

	priv1 = netdev_priv(netdev1);
	priv1->netdev = netdev1;
	priv1->priv_dev = priv_dev;
	priv1->interface_num = 1;
	init_usb_anchor(&priv_dev->rx_submitted);
	init_usb_anchor(&priv1->tx_submitted);

	/* Init SWCAN device */
	priv1->can.state = CAN_STATE_STOPPED;
	priv1->can.bitrate_const = wqcan_bitrate;
	priv1->can.bitrate_const_cnt = ARRAY_SIZE(wqcan_bitrate);
	priv1->can.do_set_bittiming = wqcan_net_set_bittiming_sw;
	priv1->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK
					| CAN_CTRLMODE_ONE_SHOT
					| CAN_CTRLMODE_LISTENONLY;

	netdev1->flags |= IFF_ECHO; /* we support local echo */
	netdev1->netdev_ops = &wqcan_netdev_ops_sw;

	SET_NETDEV_DEV(netdev1, &intf->dev);

	err = register_candev(netdev1);
	if (err) {
		netdev_err(netdev1, "couldn't register CAN device: %d\n", err);
		goto cleanup_free_candev;
	}
	priv_dev->interfaces[1] = priv1;

	/* Start USB dev only if we have successfully registered CAN device */
	err = wqcan_usb_start(priv_dev);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(priv->netdev);

		netdev_warn(netdev, "couldn't start device: %d\n", err);
		goto cleanup_unregister_candev;
	}

	dev_info(&intf->dev, "WQCAN connected\n");

	return 0;

cleanup_unregister_candev:
	unregister_candev(priv->netdev);

cleanup_free_candev:
	free_candev(netdev);
	kfree(priv_dev);
	return err;
}

/* Called by the usb core when driver is unloaded or device is removed */
static void wqcan_usb_disconnect(struct usb_interface *intf)
{
	struct wqcan_dev_priv *priv_dev = usb_get_intfdata(intf);
	struct wqcan_priv *priv;
	usb_set_intfdata(intf, NULL);
	for (u8 i = 0; i < 2; i++)
	{
		priv = priv_dev->interfaces[i];
		netdev_info(priv->netdev, "device disconnected\n");
		unregister_candev(priv->netdev);
		free_candev(priv->netdev);
	}
	wqcan_urb_unlink(priv);
	kfree(priv_dev);
}

static struct usb_driver wqcan_usb_driver = {
	.name = WQCAN_MODULE_NAME,
	.probe = wqcan_usb_probe,
	.disconnect = wqcan_usb_disconnect,
	.id_table = wqcan_usb_table,
};

module_usb_driver(wqcan_usb_driver);

MODULE_AUTHOR("Witold Olechowski <witusio@gmail.com>");
MODULE_DESCRIPTION("SocketCAN driver for WQCAN");
MODULE_LICENSE("GPL v2");
