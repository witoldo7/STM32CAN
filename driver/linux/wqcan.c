// SPDX-License-Identifier: GPL-2.0-only
/* SocketCAN driver for CombiAdapter/WQCAN.
 *
 * Copyright (C) 2022 Witold Olechowski
 *
 * This driver is inspired by the net/can/usb/wqcan_usb.c
 */

#include <asm/unaligned.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/usb.h>

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
#define WQCAN_USB_RX_BUFF_SIZE 64

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
#define WQCAN_CMD_CAN_RECEIVE_MESSAGE 0x82
#define WQCAN_CMD_CAN_TRANSMIT_MESSAGE 0x83

struct wqcan_usb_ctx {
	struct wqcan_priv *priv;
	u32 ndx;
	bool can;
};

/* Structure to hold all of our device specific stuff */
struct wqcan_priv {
	struct can_priv can; /* must be the first member */
	struct sk_buff *echo_skb[WQCAN_MAX_TX_URBS];
	struct wqcan_usb_ctx tx_context[WQCAN_MAX_TX_URBS];
	struct usb_device *udev;
	struct net_device *netdev;
	struct usb_anchor tx_submitted;
	struct usb_anchor rx_submitted;
	struct can_berr_counter bec;
	bool usb_ka_first_pass;
	bool can_ka_first_pass;
	bool can_speed_check;
	atomic_t free_ctx_cnt;
	void *rxbuf[WQCAN_MAX_RX_URBS];
	dma_addr_t rxbuf_dma[WQCAN_MAX_RX_URBS];
};

/* CAN frame */
struct __packed wqcan_usb_msg_can {
	u8 cmd_id;
	u16 len;
	__be16 sid;
	__be16 eid;
	u8 data[8];
	u8 dlc;
	u8 extended;
	u8 remote;
	u8 term;
};

/* command frame */
struct __packed wqcan_usb_msg {
	u8 cmd_id;
	u8 unused[18];
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
	u8 term;
};

static const struct usb_device_id wqcan_usb_table[] = {
	{ USB_DEVICE(WQCAN_VENDOR_ID, WQCAN_PRODUCT_ID) },
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, wqcan_usb_table);

static const u32 wqcan_bitrate[] = { 33333, 47619, 500000};

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
							 struct can_frame *cf)
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

		can_led_event(netdev, CAN_LED_EVENT_TX);
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

	buf = usb_alloc_coherent(priv->udev, size, GFP_ATOMIC,
				 &urb->transfer_dma);
	if (!buf) {
		err = -ENOMEM;
		goto nomembuf;
	}

	memcpy(buf, usb_msg, size);

	usb_fill_bulk_urb(urb, priv->udev,
			  usb_sndbulkpipe(priv->udev, WQCAN_USB_EP_OUT), buf,
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
	usb_free_coherent(priv->udev, size, buf,
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
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct wqcan_usb_ctx *ctx = NULL;
	struct net_device_stats *stats = &priv->netdev->stats;
	u16 sid;
	int err;
	struct wqcan_usb_msg_can usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_TRANSMIT_MESSAGE
	};

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	ctx = wqcan_usb_get_free_ctx(priv, cf);
	if (!ctx)
		return NETDEV_TX_BUSY;

	if (cf->can_id & CAN_EFF_FLAG) {
		/* SIDH    | SIDL                 | EIDH   | EIDL
		 * 28 - 21 | 20 19 18 x x x 17 16 | 15 - 8 | 7 - 0
		 */
		usb_msg.extended = 1;
		/* store 28-18 bits */
		sid |= (cf->can_id & 0x1ffc0000) >> 13;
		/* store 17-16 bits */
		sid |= (cf->can_id & 0x30000) >> 16;
		put_unaligned_be16(sid, &usb_msg.sid);

		/* store 15-0 bits */
		put_unaligned_be16(cf->can_id & 0xffff, &usb_msg.eid);
	} else {
		/* SIDH   | SIDL
		 * 10 - 3 | 2 1 0 x x x x x
		 */
		put_unaligned_be16((cf->can_id & CAN_SFF_MASK) << 5,
				   &usb_msg.sid);
		usb_msg.eid = 0;
	}

	usb_msg.dlc = cf->len;

	memcpy(usb_msg.data, cf->data, usb_msg.dlc);


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

static void wqcan_usb_xmit_change_bitrate(struct wqcan_priv *priv, u32 bitrate)
{
	struct wqcan_usb_msg_can_bitrate usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_BIT_RATE,
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

static void wqcan_usb_can_open(struct wqcan_priv *priv)
{
	struct wqcan_usb_msg_can_open usb_msg = {
		.cmd_id = WQCAN_CMD_CAN_OPEN,
		.data = 1,
		.term = 0
	};
	put_unaligned_be16(1, &usb_msg.len);
	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
	usb_msg.cmd_id = WQCAN_CMD_SWCAN_OPEN;
	wqcan_usb_xmit_cmd(priv, (struct wqcan_usb_msg *)&usb_msg, sizeof(usb_msg));
}

static void wqcan_usb_process_can(struct wqcan_priv *priv,
				 struct wqcan_usb_msg_can *msg)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;
	u16 sid;

	skb = alloc_can_skb(priv->netdev, &cf);
	if (!skb)
		return;

	sid = get_unaligned_be16(&msg->sid);

	if (&msg->extended == 1) {
		/* SIDH    | SIDL                 | EIDH   | EIDL
		 * 28 - 21 | 20 19 18 x x x 17 16 | 15 - 8 | 7 - 0
		 */
		cf->can_id = CAN_EFF_FLAG;

		/* store 28-18 bits */
		cf->can_id |= (sid & 0xffe0) << 13;
		/* store 17-16 bits */
		cf->can_id |= (sid & 3) << 16;
		/* store 15-0 bits */
		cf->can_id |= get_unaligned_be16(&msg->eid);
	} else {
		/* SIDH   | SIDL
		 * 10 - 3 | 2 1 0 x x x x x
		 */
		cf->can_id = (sid & 0xffe0) >> 5;
	}

	cf->len = can_cc_dlc2len(msg->dlc);

	if (&msg->remote == 1) {
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		memcpy(cf->data, msg->data, cf->len);

		stats->rx_bytes += cf->len;
	}
	stats->rx_packets++;

	can_led_event(priv->netdev, CAN_LED_EVENT_RX);
	netif_rx(skb);
}

static void wqcan_usb_process_ka_usb(struct wqcan_priv *priv,
				    struct wqcan_usb_msg_ka_usb *msg)
{
	if (unlikely(priv->usb_ka_first_pass)) {
		netdev_info(priv->netdev, "WQCAN USB version %u.%u\n",
			    msg->soft_ver_major, msg->soft_ver_minor);

		priv->usb_ka_first_pass = false;
	}
}

static void wqcan_usb_process_rx(struct wqcan_priv *priv,
				struct wqcan_usb_msg *msg)
{
	switch (msg->cmd_id) {
	case WQCAN_CMD_READ_FW_VERSION:
		wqcan_usb_process_ka_usb(priv,
					(struct wqcan_usb_msg_ka_usb *)msg);
		break;

	case WQCAN_CMD_CAN_RECEIVE_MESSAGE:
	case WQCAN_CMD_SWCAN_RECEIVE_MESSAGE:
		wqcan_usb_process_can(priv, (struct wqcan_usb_msg_can *)msg);
		break;

	default:
		netdev_warn(priv->netdev, "Unsupported msg (0x%X)",
			    msg->cmd_id);
		break;
	}
}

/* Callback for reading data from device
 *
 * Check urb status, call read function and resubmit urb read operation.
 */
static void wqcan_usb_read_bulk_callback(struct urb *urb)
{
	struct wqcan_priv *priv = urb->context;
	struct net_device *netdev;
	int retval;
	int pos = 0;

	netdev = priv->netdev;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		return;

	default:
		netdev_info(netdev, "Rx URB aborted (%d)\n", urb->status);

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
		netif_device_detach(netdev);
	else if (retval)
		netdev_err(netdev, "failed resubmitting read bulk urb: %d\n",
			   retval);
}

/* Start USB device */
static int wqcan_usb_start(struct wqcan_priv *priv)
{
	struct net_device *netdev = priv->netdev;
	int err, i;

	wqcan_init_ctx(priv);

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
			netdev_err(netdev, "No memory left for USB buffer\n");
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

		priv->rxbuf[i] = buf;
		priv->rxbuf_dma[i] = buf_dma;

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
	}

	/* Did we submit any URBs */
	if (i == 0) {
		netdev_warn(netdev, "couldn't setup read URBs\n");
		return err;
	}

	/* Warn if we've couldn't transmit all the URBs */
	if (i < WQCAN_MAX_RX_URBS)
		netdev_warn(netdev, "rx performance may be slow\n");

	wqcan_usb_xmit_read_fw_ver(priv);
	wqcan_usb_can_open(priv);
	return err;
}

/* Open USB device */
static int wqcan_usb_open(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	int err;

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

	priv->can_speed_check = true;
	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	can_led_event(netdev, CAN_LED_EVENT_OPEN);
	netif_start_queue(netdev);

	return 0;
}

static void wqcan_urb_unlink(struct wqcan_priv *priv)
{
	int i;

	usb_kill_anchored_urbs(&priv->rx_submitted);

	for (i = 0; i < WQCAN_MAX_RX_URBS; ++i)
		usb_free_coherent(priv->udev, WQCAN_USB_RX_BUFF_SIZE,
				  priv->rxbuf[i], priv->rxbuf_dma[i]);

	usb_kill_anchored_urbs(&priv->tx_submitted);
}

/* Close USB device */
static int wqcan_usb_close(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);

	priv->can.state = CAN_STATE_STOPPED;

	netif_stop_queue(netdev);

	/* Stop polling */
	wqcan_urb_unlink(priv);

	close_candev(netdev);
	can_led_event(netdev, CAN_LED_EVENT_STOP);

	return 0;
}

static int wqcan_net_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		wqcan_usb_can_open(priv);
		err = 0;
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

static int wqcan_net_set_bittiming(struct net_device *netdev)
{
	struct wqcan_priv *priv = netdev_priv(netdev);
	const u32 bitrate_kbps = priv->can.bittiming.bitrate;

	wqcan_usb_xmit_change_bitrate(priv, bitrate_kbps);

	return 0;
}

static int wqcan_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct net_device *netdev;
	struct wqcan_priv *priv;
	int err;
	struct usb_device *usbdev = interface_to_usbdev(intf);

	netdev = alloc_candev(sizeof(struct wqcan_priv), WQCAN_MAX_TX_URBS);
	if (!netdev) {
		dev_err(&intf->dev, "Couldn't alloc candev\n");
		return -ENOMEM;
	}

	priv = netdev_priv(netdev);

	priv->udev = usbdev;
	priv->netdev = netdev;
	priv->usb_ka_first_pass = true;
	priv->can_ka_first_pass = true;
	priv->can_speed_check = false;

	init_usb_anchor(&priv->rx_submitted);
	init_usb_anchor(&priv->tx_submitted);

	usb_set_intfdata(intf, priv);

	/* Init CAN device */
	priv->can.state = CAN_STATE_STOPPED;
	priv->can.bitrate_const = wqcan_bitrate;
	priv->can.bitrate_const_cnt = ARRAY_SIZE(wqcan_bitrate);
	priv->can.do_set_mode = wqcan_net_set_mode;
	priv->can.do_get_berr_counter = wqcan_net_get_berr_counter;
	priv->can.do_set_bittiming = wqcan_net_set_bittiming;

	netdev->netdev_ops = &wqcan_netdev_ops;

	netdev->flags |= IFF_ECHO; /* we support local echo */

	SET_NETDEV_DEV(netdev, &intf->dev);

	err = register_candev(netdev);
	if (err) {
		netdev_err(netdev, "couldn't register CAN device: %d\n", err);

		goto cleanup_free_candev;
	}

	devm_can_led_init(netdev);

	/* Start USB dev only if we have successfully registered CAN device */
	err = wqcan_usb_start(priv);
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

	return err;
}

/* Called by the usb core when driver is unloaded or device is removed */
static void wqcan_usb_disconnect(struct usb_interface *intf)
{
	struct wqcan_priv *priv = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	netdev_info(priv->netdev, "device disconnected\n");

	unregister_candev(priv->netdev);
	wqcan_urb_unlink(priv);
	free_candev(priv->netdev);
}

static struct usb_driver wqcan_usb_driver = {
	.name = WQCAN_MODULE_NAME,
	.probe = wqcan_usb_probe,
	.disconnect = wqcan_usb_disconnect,
	.id_table = wqcan_usb_table,
};

module_usb_driver(wqcan_usb_driver);

MODULE_AUTHOR("Witold Olechowski <witusio@gmail.com>");
MODULE_DESCRIPTION("SocketCAN driver for CombbiAdapter");
MODULE_LICENSE("GPL v2");
