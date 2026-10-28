/*
 * FTDI FT2232H-compatible vendor class (two UART channels) for the Zephyr
 * USB device stack (next generation).
 *
 * Ported from app/usb2dualuart/ftdi/usbd_ftdi.c of the bl_mcu_sdk project.
 *
 * Copyright (c) 2021 Sipeed team
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/common/sys_io.h>
#include <bflb_soc.h>
#include <glb_reg.h>

#include "usbd_ftdi.h"
#include "uart_bridge.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_ftdi, LOG_LEVEL_INF);

/* FTDI vendor requests */
#define SIO_RESET_REQUEST		0x00 /* Reset the port */
#define SIO_SET_MODEM_CTRL_REQUEST	0x01 /* Set the modem control register */
#define SIO_SET_FLOW_CTRL_REQUEST	0x02 /* Set flow control register */
#define SIO_SET_BAUDRATE_REQUEST	0x03 /* Set baud rate */
#define SIO_SET_DATA_REQUEST		0x04 /* Set data characteristics */
#define SIO_POLL_MODEM_STATUS_REQUEST	0x05
#define SIO_SET_EVENT_CHAR_REQUEST	0x06
#define SIO_SET_ERROR_CHAR_REQUEST	0x07
#define SIO_SET_LATENCY_TIMER_REQUEST	0x09
#define SIO_GET_LATENCY_TIMER_REQUEST	0x0A
#define SIO_SET_BITMODE_REQUEST		0x0B
#define SIO_READ_PINS_REQUEST		0x0C
#define SIO_READ_EEPROM_REQUEST		0x90
#define SIO_WRITE_EEPROM_REQUEST	0x91
#define SIO_ERASE_EEPROM_REQUEST	0x92

#define SIO_SET_DTR_MASK 0x1
#define SIO_SET_DTR_HIGH (1 | (SIO_SET_DTR_MASK << 8))
#define SIO_SET_DTR_LOW  (0 | (SIO_SET_DTR_MASK << 8))
#define SIO_SET_RTS_MASK 0x2
#define SIO_SET_RTS_HIGH (2 | (SIO_SET_RTS_MASK << 8))
#define SIO_SET_RTS_LOW  (0 | (SIO_SET_RTS_MASK << 8))

#define FTDI_NUM_PORTS		2
#define FTDI_EP_MPS		64
#define FTDI_MAX_PAYLOAD	(FTDI_EP_MPS - 2)
#define FTDI_STATUS_BYTE0	0x01
#define FTDI_STATUS_BYTE1	0x60
#define FTDI_DEFAULT_LATENCY	16

/* Initial endpoint addresses; the device stack may reassign them at
 * usbd_init() time (BL702 only supports IN on EP4..EP7, OUT on EP1..EP3).
 */
#define FTDI_IN_EP_0		(USB_EP_DIR_IN | 1)  /* 0x81 */
#define FTDI_OUT_EP_0		(USB_EP_DIR_OUT | 2) /* 0x02 */
#define FTDI_IN_EP_1		(USB_EP_DIR_IN | 3)  /* 0x83 */
#define FTDI_OUT_EP_1		(USB_EP_DIR_OUT | 4) /* 0x04 */

struct ftdi_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_if_descriptor if1;
	struct usb_ep_descriptor if1_in_ep;
	struct usb_ep_descriptor if1_out_ep;
	struct usb_desc_header nil_desc;
};

struct ftdi_class_data {
	struct ftdi_desc *desc;
	const struct usb_desc_header *const *fs_desc;
	uint8_t latency[FTDI_NUM_PORTS];
	/* Last baud rate requested by the host, initialised to the value
	 * used by the original firmware; 1200 means "not configured yet".
	 */
	uint32_t baudrate[FTDI_NUM_PORTS];
	atomic_t enabled;
	atomic_t in_busy;
	atomic_t out_busy;
};

static struct ftdi_desc ftdi_desc = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0xff,
		.bInterfaceProtocol = 0xff,
		.iInterface = 0,
	},
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = FTDI_IN_EP_0,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(FTDI_EP_MPS),
		.bInterval = 0x00,
	},
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = FTDI_OUT_EP_0,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(FTDI_EP_MPS),
		.bInterval = 0x00,
	},
	.if1 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0xff,
		.bInterfaceProtocol = 0xff,
		.iInterface = 0,
	},
	.if1_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = FTDI_IN_EP_1,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(FTDI_EP_MPS),
		.bInterval = 0x00,
	},
	.if1_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = FTDI_OUT_EP_1,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(FTDI_EP_MPS),
		.bInterval = 0x00,
	},
	.nil_desc = {
		.bLength = 0,
		.bDescriptorType = 0,
	},
};

static const struct usb_desc_header *const ftdi_fs_desc[] = {
	(struct usb_desc_header *)&ftdi_desc.if0,
	(struct usb_desc_header *)&ftdi_desc.if0_in_ep,
	(struct usb_desc_header *)&ftdi_desc.if0_out_ep,
	(struct usb_desc_header *)&ftdi_desc.if1,
	(struct usb_desc_header *)&ftdi_desc.if1_in_ep,
	(struct usb_desc_header *)&ftdi_desc.if1_out_ep,
	(struct usb_desc_header *)&ftdi_desc.nil_desc,
};

static struct ftdi_class_data ftdi_priv = {
	.desc = &ftdi_desc,
	.fs_desc = ftdi_fs_desc,
	.latency = { FTDI_DEFAULT_LATENCY, FTDI_DEFAULT_LATENCY },
	.baudrate = { 1200, 1200 },
};

/* Make every FTDI vendor request visible to the device stack, so they are
 * routed here even though the recipient is the device (not an interface).
 */
static const struct usbd_cctx_vendor_req ftdi_vreqs = USBD_VENDOR_REQ(
	SIO_RESET_REQUEST, SIO_SET_MODEM_CTRL_REQUEST,
	SIO_SET_FLOW_CTRL_REQUEST, SIO_SET_BAUDRATE_REQUEST,
	SIO_SET_DATA_REQUEST, SIO_POLL_MODEM_STATUS_REQUEST,
	SIO_SET_EVENT_CHAR_REQUEST, SIO_SET_ERROR_CHAR_REQUEST,
	SIO_SET_LATENCY_TIMER_REQUEST, SIO_GET_LATENCY_TIMER_REQUEST,
	SIO_SET_BITMODE_REQUEST, SIO_READ_PINS_REQUEST,
	SIO_READ_EEPROM_REQUEST, SIO_WRITE_EEPROM_REQUEST,
	SIO_ERASE_EEPROM_REQUEST);

/* ------------------------------------------------------------------------- */
/* Device / configuration descriptors                                        */
/* ------------------------------------------------------------------------- */
USBD_DEVICE_DEFINE(ftdi_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x0403, 0x6010);

USBD_DESC_LANG_DEFINE(ftdi_lang);
USBD_DESC_MANUFACTURER_DEFINE(ftdi_mfr, "SiPEED");
USBD_DESC_PRODUCT_DEFINE(ftdi_product, "USB TO DUALUART");
USBD_DESC_STRING_DEFINE(ftdi_sn, "SI8848", USBD_DUT_STRING_SERIAL_NUMBER);

USBD_DESC_CONFIG_DEFINE(ftdi_cfg_str, "FTDI dual UART");

/* Bus powered (the reserved bit is set by the macro), 90 * 2 mA = 180 mA,
 * same power value as the original firmware.
 *
 * The original descriptor also advertised remote wakeup, but the BL702 UDC
 * has no remote wakeup support (udc_bflb_v1 caps.rwup = false) and
 * usbd_add_configuration() rejects such a configuration with -ENOTSUP.
 */
USBD_CONFIGURATION_DEFINE(ftdi_fs_config, 0, 90, &ftdi_cfg_str);

/* ------------------------------------------------------------------------- */
/* Enumeration watchdog                                                      */
/* ------------------------------------------------------------------------- */

/*
 * The BL702 UDC driver occasionally misses the first control transfers of an
 * enumeration: the device stays attached but the host never configures it.
 * Re-attaching (releasing and re-asserting the D+ pull-up) makes the host
 * enumerate again; the stack is left untouched.
 */
#define FTDI_USB_RECONNECT_MS    1500
#define FTDI_USB_MAX_RECONNECTS 60

static struct k_work_delayable ftdi_reconnect_work;
static int ftdi_reconnect_count;

static void ftdi_reconnect_handler(struct k_work *work)
{
	if (ftdi_usb_port_ready(0) || ftdi_usb_port_ready(1)) {
		return;
	}

	if (ftdi_reconnect_count++ >= FTDI_USB_MAX_RECONNECTS) {
		LOG_ERR("USB enumeration keeps failing, giving up");
		return;
	}

	LOG_WRN("USB not configured, re-attaching (%d)", ftdi_reconnect_count);

	sys_clear_bits(GLB_BASE + GLB_USB_XCVR_OFFSET, GLB_USB_ENUM_MSK);
	k_msleep(100);
	sys_set_bits(GLB_BASE + GLB_USB_XCVR_OFFSET, GLB_USB_ENUM_MSK);

#if !defined(FTDI_NO_RECONNECT)
	k_work_reschedule(&ftdi_reconnect_work, K_MSEC(FTDI_USB_RECONNECT_MS));
#endif
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

/*
 * FTDI addresses its channels through the low byte of wIndex, 1-based
 * (Linux ftdi_sio: priv->channel = CHANNEL_A + bInterfaceNumber) for the
 * device-recipient vendor requests it normally uses.  Should a host address
 * the request to a specific interface instead, wIndex is the 0-based
 * interface number and can be used directly.
 * Returns the port index or -1 when the request is not port specific.
 */
static int request_port(const struct usb_setup_packet *setup)
{
	if (setup->RequestType.recipient == USB_REQTYPE_RECIPIENT_INTERFACE) {
		return (setup->wIndex < FTDI_NUM_PORTS) ? setup->wIndex : -1;
	}

	uint8_t channel = setup->wIndex & 0xff;

	/* FTDI uses 1-based channels (1, 2); some hosts (e.g. the macOS
	 * AppleUSBFTDI driver) use the 0-based interface number instead.
	 * Accept both: 0/1 -> port A, 2 -> port B.
	 */
	if (channel == 0) {
		return 0;
	}

	if (channel > FTDI_NUM_PORTS) {
		return -1;
	}

	return channel - 1;
}

static struct net_buf *ctrl_reply(struct usbd_class_data *const c_data,
				  const void *data, size_t len)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct net_buf *buf;

	if (len == 0) {
		return NULL;
	}

	buf = usbd_ep_ctrl_data_in_alloc(uds_ctx, len);
	if (buf == NULL) {
		return NULL;
	}

	net_buf_add_mem(buf, data, len);

	return buf;
}

/* FT2232H/FT232H baud encoding: bit 17 set, 120 MHz base, 10-bit sampling. */
static uint32_t ftdi_decode_baudrate_h(uint32_t itdf)
{
	static const uint8_t divfrac[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
	uint32_t num = divfrac[(itdf >> 14) & 0x07];
	uint32_t intpart = itdf & 0x3fff;
	uint32_t denom;

	if (intpart == 0 && num == 0) {
		/* Divisor 1.0 with the /2.5 divider disabled: 12 Mbaud. */
		return 12000000;
	}

	denom = 10 * (intpart * 8 + num);

	return (uint32_t)((uint64_t)120000000 * 8 / denom);
}

/* Translate the FTDI baud rate encoding into a real baud rate. */
static uint32_t ftdi_decode_baudrate(uint32_t itdf_divisor)
{
#define FTDI_USB_CLK 48000000
	uint32_t baudrate;
	static const uint8_t frac[] = { 0, 8, 4, 2, 6, 10, 12, 14 };
	uint32_t divisor = itdf_divisor & 0x3fff;

	divisor <<= 4;
	divisor |= frac[(itdf_divisor >> 14) & 0x07];

	if (itdf_divisor == 0x01) {
		baudrate = 2000000;
	} else if (itdf_divisor == 0x00) {
		baudrate = 3000000;
	} else {
		baudrate = (divisor != 0) ? (FTDI_USB_CLK / divisor) : 0;
	}

	return baudrate;
}

/*
 * Endpoint addresses are assigned by the USB device stack at usbd_init()
 * time: the BL702 controller only supports IN on EP4..EP7 and OUT on
 * EP1..EP3, so the descriptors are rewritten.  Always use the values from
 * the descriptors once the stack has been initialised.
 */
static uint8_t ftdi_ep_in(uint8_t port)
{
	return (port == 0) ? ftdi_priv.desc->if0_in_ep.bEndpointAddress
			   : ftdi_priv.desc->if1_in_ep.bEndpointAddress;
}

static uint8_t ftdi_ep_out(uint8_t port)
{
	return (port == 0) ? ftdi_priv.desc->if0_out_ep.bEndpointAddress
			   : ftdi_priv.desc->if1_out_ep.bEndpointAddress;
}

/* ------------------------------------------------------------------------- */
/* Control requests                                                          */
/* ------------------------------------------------------------------------- */

static struct net_buf *
ftdi_control_to_host(struct usbd_class_data *const c_data,
		     const struct usb_setup_packet *const setup)
{
	struct ftdi_class_data *priv = usbd_class_get_private(c_data);
	size_t len;

	switch (setup->bRequest) {
	case SIO_GET_LATENCY_TIMER_REQUEST: {
		int port = request_port(setup);
		uint8_t latency;

		if (port < 0) {
			return NULL;
		}

		latency = priv->latency[port];
		len = MIN(sizeof(latency), setup->wLength);

		return ctrl_reply(c_data, &latency, len);
	}
	case SIO_POLL_MODEM_STATUS_REQUEST: {
		/* Byte 0: CTS/DSR/RI/RLSD (no inputs wired);
		 * byte 1: THRE | TEMT, as sent by the original firmware.
		 */
		static const uint8_t status[2] = { 0x10, FTDI_STATUS_BYTE1 };

		len = MIN(sizeof(status), setup->wLength);

		return ctrl_reply(c_data, status, len);
	}
	case SIO_READ_EEPROM_REQUEST: {
		uint8_t word[2];
		size_t idx = setup->wIndex % FTDI_EEPROM_WORDS;

		sys_put_le16(ftdi_eeprom_info[idx], word);
		len = MIN(sizeof(word), setup->wLength);

		return ctrl_reply(c_data, word, len);
	}
	case SIO_READ_PINS_REQUEST: {
		/* No input lines are connected to the bridge. */
		static const uint8_t pins = 0x00;

		len = MIN(sizeof(pins), setup->wLength);

		return ctrl_reply(c_data, &pins, len);
	}
	default:
		LOG_DBG("Unsupported FTDI IN request 0x%02x", setup->bRequest);
		return NULL;
	}
}

static int ftdi_control_to_dev(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       const struct net_buf *const buf)
{
	struct ftdi_class_data *priv = usbd_class_get_private(c_data);
	int port = request_port(setup);

	ARG_UNUSED(buf);

	switch (setup->bRequest) {
	case SIO_RESET_REQUEST:
		if (port >= 0) {
			priv->latency[port] = FTDI_DEFAULT_LATENCY;
		} else {
			for (int i = 0; i < FTDI_NUM_PORTS; i++) {
				priv->latency[i] = FTDI_DEFAULT_LATENCY;
			}
		}
		break;

	case SIO_SET_MODEM_CTRL_REQUEST:
		if (port < 0) {
			return -ENOTSUP;
		}

		switch (setup->wValue) {
		case SIO_SET_DTR_HIGH:
			uart_bridge_set_dtr(port, true);
			break;
		case SIO_SET_DTR_LOW:
			uart_bridge_set_dtr(port, false);
			break;
		case SIO_SET_RTS_HIGH:
			uart_bridge_set_rts(port, true);
			break;
		case SIO_SET_RTS_LOW:
			uart_bridge_set_rts(port, false);
			break;
		default:
			break;
		}
		break;

	case SIO_SET_FLOW_CTRL_REQUEST:
		/* RTS/CTS and XON/XOFF are not implemented. */
		break;

	case SIO_SET_BAUDRATE_REQUEST: {
		uint32_t itdf_divisor;
		uint32_t baudrate;

		if (port < 0) {
			return -ENOTSUP;
		}

		/* wValue = divisor[15:0]
		 * wIndex[15:8] = divisor[23:16], wIndex[7:0] = channel
		 */
		itdf_divisor = setup->wValue |
			       ((uint32_t)(setup->wIndex & 0xff00) << 8);
		if (itdf_divisor & 0x20000) {
			baudrate = ftdi_decode_baudrate_h(itdf_divisor);
		} else {
			baudrate = ftdi_decode_baudrate(itdf_divisor);
		}
		priv->baudrate[port] = baudrate;

		if (baudrate >= 50 && baudrate <= 6000000 && baudrate != 1200) {
			uart_bridge_set_baudrate(port, baudrate);
		}
		break;
	}

	case SIO_SET_DATA_REQUEST:
		/* D0-D7   data bits (7, 8)
		 * D8-D10  parity (0 = none, 1 = odd, 2 = even, 3/4 = mark/space)
		 * D11-D12 stop bits (0 = 1, 1 = 1.5, 2 = 2)
		 */
		if (port < 0) {
			return -ENOTSUP;
		}

		if (priv->baudrate[port] != 1200) {
			uart_bridge_set_format(port, setup->wValue & 0xff,
					       (setup->wValue >> 8) & 0xff,
					       (setup->wValue >> 11) & 0x03);
		}
		break;

	case SIO_SET_LATENCY_TIMER_REQUEST:
		if (port < 0) {
			return -ENOTSUP;
		}

		/* The original firmware always used 16 ms; honouring the
		 * requested value reduces latency for hosts that ask for 1 ms.
		 */
		priv->latency[port] = CLAMP(setup->wValue & 0xff, 1, 255);
		break;

	case SIO_SET_EVENT_CHAR_REQUEST:
	case SIO_SET_ERROR_CHAR_REQUEST:
	case SIO_SET_BITMODE_REQUEST:
	case SIO_WRITE_EEPROM_REQUEST:
	case SIO_ERASE_EEPROM_REQUEST:
		/* Accepted but not implemented. */
		break;

	default:
		LOG_DBG("Unsupported FTDI OUT request 0x%02x", setup->bRequest);
		return -ENOTSUP;
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/* Data path                                                                 */
/* ------------------------------------------------------------------------- */

static int ftdi_request(struct usbd_class_data *const c_data,
			struct net_buf *buf, int err)
{
	struct ftdi_class_data *priv = usbd_class_get_private(c_data);
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	uint8_t ep = bi->ep;

	for (uint8_t port = 0; port < FTDI_NUM_PORTS; port++) {
		if (ep == ftdi_ep_in(port)) {
			atomic_clear_bit(&priv->in_busy, port);
			break;
		}

		if (ep == ftdi_ep_out(port)) {
			if (err == 0 && buf->len != 0) {
				uart_bridge_usb_out(port, buf->data, buf->len);
			}
			atomic_clear_bit(&priv->out_busy, port);
			break;
		}
	}

	net_buf_unref(buf);
	uart_bridge_wake();

	return 0;
}

static void ftdi_enable(struct usbd_class_data *const c_data)
{
	struct ftdi_class_data *priv = usbd_class_get_private(c_data);

	ftdi_reconnect_count = 0;
	(void)k_work_cancel_delayable(&ftdi_reconnect_work);
	atomic_set_bit(&priv->enabled, 0);
	atomic_set_bit(&priv->enabled, 1);
	uart_bridge_set_usb_configured(true);
	LOG_INF("FTDI function enabled");
}

static void ftdi_disable(struct usbd_class_data *const c_data)
{
	struct ftdi_class_data *priv = usbd_class_get_private(c_data);

	atomic_clear_bit(&priv->enabled, 0);
	atomic_clear_bit(&priv->enabled, 1);
	atomic_clear_bit(&priv->in_busy, 0);
	atomic_clear_bit(&priv->in_busy, 1);
	atomic_clear_bit(&priv->out_busy, 0);
	atomic_clear_bit(&priv->out_busy, 1);
	uart_bridge_set_usb_configured(false);
	LOG_INF("FTDI function disabled");
}

static const void *ftdi_get_desc(struct usbd_class_data *const c_data,
				 const enum usbd_speed speed)
{
	struct ftdi_class_data *priv = usbd_class_get_private(c_data);

	ARG_UNUSED(speed);

	/* The BL702 UDC is full-speed only. */
	return priv->fs_desc;
}

static int ftdi_init(struct usbd_class_data *const c_data)
{
	ARG_UNUSED(c_data);

	return 0;
}

static const struct usbd_class_api ftdi_api = {
	.control_to_host = ftdi_control_to_host,
	.control_to_dev = ftdi_control_to_dev,
	.request = ftdi_request,
	.get_desc = ftdi_get_desc,
	.enable = ftdi_enable,
	.disable = ftdi_disable,
	.init = ftdi_init,
};

USBD_DEFINE_CLASS(ftdi_usb_class, &ftdi_api, &ftdi_priv, &ftdi_vreqs);

/* ------------------------------------------------------------------------- */
/* Public data path API used by the bridge                                   */
/* ------------------------------------------------------------------------- */

bool ftdi_usb_port_ready(uint8_t port)
{
	if (port >= FTDI_NUM_PORTS) {
		return false;
	}

	return atomic_test_bit(&ftdi_priv.enabled, port);
}

uint32_t ftdi_usb_latency_get(uint8_t port)
{
	if (port >= FTDI_NUM_PORTS) {
		return FTDI_DEFAULT_LATENCY;
	}

	return ftdi_priv.latency[port];
}

int ftdi_usb_send(uint8_t port, const uint8_t *payload, size_t len)
{
	static const uint8_t status[2] = {
		FTDI_STATUS_BYTE0, FTDI_STATUS_BYTE1
	};
	struct net_buf *buf;
	int err;

	if (port >= FTDI_NUM_PORTS || len > FTDI_MAX_PAYLOAD) {
		return -EINVAL;
	}

	if (!ftdi_usb_port_ready(port)) {
		return -EPERM;
	}

	if (atomic_test_and_set_bit(&ftdi_priv.in_busy, port)) {
		return -EBUSY;
	}

	buf = usbd_ep_buf_alloc(&ftdi_usb_class, ftdi_ep_in(port),
				FTDI_EP_MPS);
	if (buf == NULL) {
		atomic_clear_bit(&ftdi_priv.in_busy, port);
		return -ENOMEM;
	}

	net_buf_add_mem(buf, status, sizeof(status));
	if (len != 0) {
		net_buf_add_mem(buf, payload, len);
	}

	err = usbd_ep_enqueue(&ftdi_usb_class, buf);
	if (err != 0) {
		net_buf_unref(buf);
		atomic_clear_bit(&ftdi_priv.in_busy, port);
	}

	return err;
}

void ftdi_usb_pump(uint8_t port)
{
	struct net_buf *buf;
	int err;

	if (port >= FTDI_NUM_PORTS || !ftdi_usb_port_ready(port)) {
		return;
	}

	if (uart_bridge_usb_out_space(port) < FTDI_EP_MPS) {
		return;
	}

	if (atomic_test_and_set_bit(&ftdi_priv.out_busy, port)) {
		return;
	}

	buf = usbd_ep_buf_alloc(&ftdi_usb_class, ftdi_ep_out(port),
				FTDI_EP_MPS);
	if (buf == NULL) {
		atomic_clear_bit(&ftdi_priv.out_busy, port);
		return;
	}

	err = usbd_ep_enqueue(&ftdi_usb_class, buf);
	if (err != 0) {
		net_buf_unref(buf);
		atomic_clear_bit(&ftdi_priv.out_busy, port);
	}
}

/* ------------------------------------------------------------------------- */
/* Initialisation                                                            */
/* ------------------------------------------------------------------------- */

static void ftdi_msg_cb(struct usbd_context *const uds_ctx,
			const struct usbd_msg *const msg)
{
	LOG_DBG("USBD message: %s", usbd_msg_type_string(msg->type));

	if (msg->type == USBD_MSG_RESET) {
			ftdi_priv.latency[0] = FTDI_DEFAULT_LATENCY;
		ftdi_priv.latency[1] = FTDI_DEFAULT_LATENCY;
	}

	if (usbd_can_detect_vbus(uds_ctx) &&
	    msg->type == USBD_MSG_VBUS_READY) {
		if (usbd_enable(uds_ctx) != 0) {
			LOG_ERR("Failed to enable USB device support");
		}
	}

	if (usbd_can_detect_vbus(uds_ctx) &&
	    msg->type == USBD_MSG_VBUS_REMOVED) {
		if (usbd_disable(uds_ctx) != 0) {
			LOG_ERR("Failed to disable USB device support");
		}
	}
}

int ftdi_usb_init(void)
{
	int err;

	k_work_init_delayable(&ftdi_reconnect_work, ftdi_reconnect_handler);

	err = usbd_add_descriptor(&ftdi_usbd, &ftdi_lang);
	if (err == 0) {
		err = usbd_add_descriptor(&ftdi_usbd, &ftdi_mfr);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&ftdi_usbd, &ftdi_product);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&ftdi_usbd, &ftdi_sn);
	}
	if (err == 0) {
		err = usbd_add_configuration(&ftdi_usbd, USBD_SPEED_FS,
					     &ftdi_fs_config);
	}
	if (err == 0) {
		err = usbd_register_all_classes(&ftdi_usbd, USBD_SPEED_FS, 1,
						NULL);
	}
	if (err == 0) {
		/* bcdDevice 0x0500 tells the FTDI host driver this is an
		 * FT2232H-class device (FTDI_SIO_SET_BAUDRATE support).
		 */
		err = usbd_device_set_bcd_device(&ftdi_usbd, 0x0500);
	}
	if (err == 0) {
		/* No device level class, like the original descriptor
		 * (bDeviceClass/SubClass/Protocol = 0/0/0).
		 */
		err = usbd_device_set_code_triple(&ftdi_usbd, USBD_SPEED_FS,
						  0, 0, 0);
	}
	if (err != 0) {
		LOG_ERR("Failed to set up USB descriptors (%d)", err);
		return err;
	}

	usbd_msg_register_cb(&ftdi_usbd, ftdi_msg_cb);

	err = usbd_init(&ftdi_usbd);
	if (err != 0) {
		LOG_ERR("Failed to initialise USB device support (%d)", err);
		return err;
	}

	err = usbd_enable(&ftdi_usbd);
	if (err != 0) {
		LOG_ERR("Failed to enable USB device support (%d)", err);
		return err;
	}

	k_work_reschedule(&ftdi_reconnect_work, K_MSEC(FTDI_USB_RECONNECT_MS));

	LOG_INF("USB FTDI dual UART bridge initialised");

	return 0;
}
