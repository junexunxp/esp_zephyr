/* esp vhci based Bluetooth driver */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr.h>
#include <arch/cpu.h>

#include <init.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <string.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <drivers/bluetooth/hci_driver.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME bt_driver
#include "common/log.h"

#define VHCI_NONE 0x00
#define VHCI_CMD  0x01
#define VHCI_ACL  0x02
#define VHCI_SCO  0x03
#define VHCI_EVT  0x04

/* VHCI function interface */
typedef struct vhci_host_callback {
    void (*notify_host_send_available)(void);               /*!< callback used to notify that the host can send packet to controller */
    int (*notify_host_recv)(uint8_t *data, uint16_t len);   /*!< callback used to notify that the controller has a packet to send to the host*/
} vhci_host_callback_t;

static void controller_rcv_pkt_ready(void);
static int vhci_rcv_pkt(uint8_t *data, uint16_t len);

static const vhci_host_callback_t vhci_host_cb = {
    .notify_host_send_available = controller_rcv_pkt_ready,
    .notify_host_recv = vhci_rcv_pkt,
};

/* VHCI */
extern bool API_vhci_host_check_send_available(void);
extern void API_vhci_host_send_packet(uint8_t *data, uint16_t len);
extern int API_vhci_host_register_callback(const vhci_host_callback_t *callback);

static K_THREAD_STACK_DEFINE(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);
static struct k_thread rx_thread_data;

static struct {
	struct net_buf *buf;
	struct k_fifo   fifo;

	u16_t    remaining;
	u16_t    discard;

	bool     have_hdr;
	bool     discardable;

	u8_t     hdr_len;

	u8_t     type;
	union {
		struct bt_hci_evt_hdr evt;
		struct bt_hci_acl_hdr acl;
		u8_t hdr[4];
	};
} rx = {
	.fifo = Z_FIFO_INITIALIZER(rx.fifo),
};

static inline void copy_hdr(struct net_buf *buf)
{
	net_buf_add_mem(buf, rx.hdr, rx.hdr_len);
}

static void reset_rx(void)
{
	rx.type = VHCI_NONE;
	rx.remaining = 0U;
	rx.have_hdr = false;
	rx.hdr_len = 0U;
	rx.discardable = false;
}

static struct net_buf *get_rx(int timeout)
{
	BT_DBG("type 0x%02x, evt 0x%02x", rx.type, rx.evt.evt);

	if (rx.type == VHCI_EVT) {
		return bt_buf_get_evt(rx.evt.evt, rx.discardable, timeout);
	}

	return bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
}

static void rx_thread(void *p1, void *p2, void *p3)
{
	struct net_buf *buf;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	BT_DBG("started");

	while (1) {
		BT_DBG("rx.buf %p", rx.buf);

		/* We can only do the allocation if we know the initial
		 * header, since Command Complete/Status events must use the
		 * original command buffer (if available).
		 */
		if (rx.have_hdr && !rx.buf) {
			rx.buf = get_rx(K_FOREVER);
			BT_DBG("Got rx.buf %p", rx.buf);
			if (rx.remaining > net_buf_tailroom(rx.buf)) {
				BT_ERR("Not enough space in buffer");
				rx.discard = rx.remaining;
				reset_rx();
			} else {
				copy_hdr(rx.buf);
			}
		}

		buf = net_buf_get(&rx.fifo, K_FOREVER);
		do {
			BT_DBG("Calling bt_recv(%p)", buf);
			bt_recv(buf);

			/* Give other threads a chance to run if the ISR
			 * is receiving data so fast that rx.fifo never
			 * or very rarely goes empty.
			 */
			k_yield();
			buf = net_buf_get(&rx.fifo, K_NO_WAIT);
		} while (buf);
	}
}


static void controller_rcv_pkt_ready(void)
{

}


static inline u16_t vhci_extract_type(uint8_t *data, uint16_t len)
{
	
	u16_t consume_len = 0;
	if (!data || !len) {
		BT_WARN("Unable to read packet type");
		rx.type = VHCI_NONE;
		return consume_len;
	}
	rx.type = data[0];
	consume_len++;

	switch (rx.type) {
	case VHCI_EVT:
		rx.remaining = sizeof(rx.evt);
		rx.hdr_len = rx.remaining;
		break;
	case VHCI_ACL:
		rx.remaining = sizeof(rx.acl);
		rx.hdr_len = rx.remaining;
		break;
	default:
		BT_ERR("Unknown VHCI type 0x%02x", rx.type);
		rx.type = VHCI_NONE;
	}
	return consume_len;
}

static inline u16_t extract_data_to_buffer(uint8_t *data, uint16_t len, u8_t *rxbuffer, u16_t read_len)
{
	u16_t rlen = MIN(len,read_len);
	memcpy(rxbuffer,data,rlen);
	return rlen;

}

static inline u16_t vhci_extract_evt_hdr(uint8_t *data, uint16_t len)
{
	struct bt_hci_evt_hdr *hdr = &rx.evt;
	int to_read = rx.hdr_len - rx.remaining;
	u16_t clen = extract_data_to_buffer(data,len,(u8_t *)hdr + to_read, rx.remaining);
	rx.remaining -= clen;
	
	if (rx.hdr_len == sizeof(*hdr) && rx.remaining < sizeof(*hdr)) {
		switch (rx.evt.evt) {
		case BT_HCI_EVT_LE_META_EVENT:
			rx.remaining++;
			rx.hdr_len++;
			break;
#if defined(CONFIG_BT_BREDR)
		case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
		case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
			rx.discardable = true;
			break;
#endif
		}
	}

	if (!rx.remaining) {
		if (rx.evt.evt == BT_HCI_EVT_LE_META_EVENT &&
		    rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_ADVERTISING_REPORT) {
			BT_DBG("Marking adv report as discardable");
			rx.discardable = true;
		}

		rx.remaining = hdr->len - (rx.hdr_len - sizeof(*hdr));
		BT_DBG("Got event header. Payload %u bytes", hdr->len);
		rx.have_hdr = true;
	}
	return clen;
}

static inline u16_t vhci_extract_acl_hdr(uint8_t *data, uint16_t len)
{
	struct bt_hci_acl_hdr *hdr = &rx.acl;
	int to_read = sizeof(*hdr) - rx.remaining;
	u16_t clen = extract_data_to_buffer(data,len,(u8_t *)hdr + to_read,rx.remaining);

	rx.remaining -= clen;
	
	if (!rx.remaining) {
		rx.remaining = sys_le16_to_cpu(hdr->len);
		BT_DBG("Got ACL header. Payload %u bytes", rx.remaining);
		rx.have_hdr = true;
	}
}

static inline uint16_t extract_header(uint8_t *data, uint16_t len)
{
	u16_t consume_len = 0;
	switch (rx.type) {
	case VHCI_NONE:
		consume_len = vhci_extract_type(data,len);
		return consume_len;
	case VHCI_EVT:
		consume_len = vhci_extract_evt_hdr(data,len);
		break;
	case VHCI_ACL:
		consume_len = vhci_extract_acl_hdr(data,len);
		break;
	default:
		CODE_UNREACHABLE;
		return consume_len;
	}

	if (rx.have_hdr && rx.buf) {
		if (rx.remaining > net_buf_tailroom(rx.buf)) {
			BT_ERR("Not enough space in buffer");
			rx.discard = rx.remaining;
			reset_rx();
		} else {
			copy_hdr(rx.buf);
		}
	}
	return consume_len;
}


static inline uint16_t extract_payload(uint8_t *data, uint16_t len)
{
	struct net_buf *buf;
	bool prio;
	int read;
	u16_t clen=0;

	if (!rx.buf) {
		rx.buf = get_rx(K_NO_WAIT);
		if (!rx.buf) {
			if (rx.discardable) {
				BT_WARN("Discarding event 0x%02x", rx.evt.evt);
				rx.discard = rx.remaining;
				reset_rx();
				return clen;
			}

			BT_WARN("Failed to allocate, deferring to rx_thread");
			return clen;
		}

		BT_DBG("Allocated rx.buf %p", rx.buf);

		if (rx.remaining > net_buf_tailroom(rx.buf)) {
			BT_ERR("Not enough space in buffer");
			rx.discard = rx.remaining;
			reset_rx();
			return clen;
		}

		copy_hdr(rx.buf);
	}
	read = clen = extract_data_to_buffer(data,len,net_buf_tail(rx.buf), rx.remaining);
	net_buf_add(rx.buf, read);
	rx.remaining -= read;

	BT_DBG("got %d bytes, remaining %u", read, rx.remaining);
	BT_DBG("Payload (len %u): %s", rx.buf->len,
	       bt_hex(rx.buf->data, rx.buf->len));

	if (rx.remaining) {
		return clen;
	}

	prio = (rx.type == VHCI_EVT && bt_hci_evt_is_prio(rx.evt.evt));

	buf = rx.buf;
	rx.buf = NULL;

	if (rx.type == VHCI_EVT) {
		bt_buf_set_type(buf, BT_BUF_EVT);
	} else {
		bt_buf_set_type(buf, BT_BUF_ACL_IN);
	}

	reset_rx();

	if (prio) {
		BT_DBG("Calling bt_recv_prio(%p)", buf);
		bt_recv_prio(buf);
	} else {
		BT_DBG("Putting buf %p to rx fifo", buf);
		net_buf_put(&rx.fifo, buf);
	}
	return clen;
}




static int vhci_rcv_pkt(uint8_t *data, uint16_t len)
{
	BT_DBG("remaining %u discard %u have_hdr %u rx.buf %p len %u",
	       rx.remaining, rx.discard, rx.have_hdr, rx.buf,
	       rx.buf ? rx.buf->len : 0);
	uint8_t *ptr = data;
	u16_t rlen = len;
	u16_t clen = 0;
	if (rx.discard) {
		clen = MIN(rx.discard,len);
		rx.discard -= clen;
		if(len > clen){
			rlen -= clen;
			ptr += clen;
		}else{
			return 0;
		}
	}
	
	do{
		if (rx.have_hdr) {
			clen = extract_payload(ptr,rlen);
		} else {
			clen = extract_header(ptr,rlen);
		}
	
		ptr += clen;
		rlen -= clen;
	}while(rlen);
		
    return 0;
}


static int vhci_send(struct net_buf *buf)
{
	BT_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	u8_t type;

	BT_DBG("enter");

	if (!buf || !buf->len) {
		BT_ERR("Empty HCI packet");
		return -EINVAL;
	}

	type = bt_buf_get_type(buf);
	switch (type) {
	case BT_BUF_ACL_OUT:
	case BT_BUF_CMD:
		break;
	default:
		BT_ERR("Unknown HCI type %u", type);
		return -EINVAL;
	}
	API_vhci_host_send_packet(&type,1);
	API_vhci_host_send_packet(buf->data, buf->len);

	return 0;
}


static int vhci_open(void)
{
	int ret;

	BT_DBG("");
	API_vhci_host_register_callback(&vhci_host_cb);

	k_thread_create(&rx_thread_data, rx_thread_stack,
			K_THREAD_STACK_SIZEOF(rx_thread_stack),
			rx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "VHCI RX");

	return 0;
}

static const struct bt_hci_driver drv = {
	.name		= "vhci",
	.bus		= BT_HCI_DRIVER_BUS_VIRTUAL,
	.open		= vhci_open,
	.send		= vhci_send,
};

static int vhci_init(struct device *unused)
{
	ARG_UNUSED(unused);

	bt_hci_driver_register(&drv);

	return 0;
}

SYS_INIT(vhci_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
