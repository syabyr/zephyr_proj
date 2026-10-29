/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IEEE 802.15.4 sniffer for the BL702 (dt_xt_zb1_devkit).
 *
 * The in-tree ieee802154_bflb driver runs without an L2: with
 * CONFIG_IEEE802154_RAW_MODE the lmac154 library hands every received frame
 * to net_recv_data() below. Frames are printed as IEEE 802.15.4 TAP hex
 * lines that zbsniff wraps into a LINKTYPE_IEEE802_15_4_TAP pcap file.
 *
 * The channel is changed with the BOOT key (D28) or from the console:
 * 'c' next channel, 'p' previous channel, 's' toggle sweeping, 'h' help.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ieee802154_pkt.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <bouffalolab/common/lmac154/lmac154.h>

#define CHANNEL_FIRST		11U
#define CHANNEL_LAST		26U
#define CHANNEL_START		15U
#define CHANNEL_COUNT		(CHANNEL_LAST - CHANNEL_FIRST + 1U)

#define SWEEP_INTERVAL_MS	500

/* D28, the BOOT key. Pressing it pulls the pin high; the same level at reset
 * selects the UART bootloader.
 */
#define KEY_DEBOUNCE_MS		30

#define PACKET_QUEUE_DEPTH	8U
#define PSDU_MAX		127U
#define TAP_HEADER_LEN		36U

struct sniffer_packet {
	/* MPDU without the FCS. */
	uint8_t data[PSDU_MAX];
	uint16_t len;
	int16_t rssi;
	uint8_t lqi;
	uint8_t channel;
};

K_MSGQ_DEFINE(packet_queue, sizeof(struct sniffer_packet), PACKET_QUEUE_DEPTH, 4);

static const struct device *const radio_dev = DEVICE_DT_GET(DT_NODELABEL(ieee802154));
static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static const struct ieee802154_radio_api *radio_api;

static const struct gpio_dt_spec boot_key =
	GPIO_DT_SPEC_GET(DT_NODELABEL(boot_key), gpios);

/* Raw and debounced key levels, both true while pressed. */
static bool key_raw;
static bool key_stable;
static int64_t key_raw_time;

/* Read in the radio interrupt, written by the main thread. */
static volatile uint8_t current_channel = CHANNEL_START;

int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	struct sniffer_packet out;
	size_t len = net_pkt_get_len(pkt);

	ARG_UNUSED(iface);

	if (len == 0U || len > PSDU_MAX) {
		/* The driver unrefs the packet when an error is returned. */
		return -EMSGSIZE;
	}

	out.len = (uint16_t)len;
	out.channel = current_channel;
	out.rssi = net_pkt_ieee802154_rssi_dbm(pkt);
	if (out.rssi == IEEE802154_MAC_RSSI_DBM_UNDEFINED) {
		out.rssi = 0;
	}
	out.lqi = net_pkt_ieee802154_lqi(pkt);

	/* net_pkt_write() in the driver left the cursor at the end. */
	net_pkt_cursor_init(pkt);
	if (net_pkt_read(pkt, out.data, len) != 0) {
		return -EIO;
	}

	if (k_msgq_put(&packet_queue, &out, K_NO_WAIT) != 0) {
		return -ENOMEM;
	}

	/* No L2 consumes the packet in raw mode, so the application owns it. */
	net_pkt_unref(pkt);

	return 0;
}

static void tap_header_build(uint8_t *header, uint16_t channel, int16_t rssi, uint8_t lqi)
{
	union {
		float f;
		uint8_t b[sizeof(float)];
	} rssi_value = { .f = (float)rssi };
	size_t i = 0U;

	header[i++] = 0x00; /* version */
	header[i++] = 0x00; /* reserved */
	header[i++] = TAP_HEADER_LEN & 0xffU;
	header[i++] = TAP_HEADER_LEN >> 8;

	/* FCS type TLV: no FCS is appended to the PSDU. */
	header[i++] = 0x00;
	header[i++] = 0x00;
	header[i++] = 0x01;
	header[i++] = 0x00;
	header[i++] = 0x00;
	header[i++] = 0x00;
	header[i++] = 0x00;
	header[i++] = 0x00;

	/* RSS TLV: IEEE-754 float in dBm. */
	header[i++] = 0x01;
	header[i++] = 0x00;
	header[i++] = 0x04;
	header[i++] = 0x00;
	memcpy(&header[i], rssi_value.b, sizeof(rssi_value.b));
	i += sizeof(rssi_value.b);

	/* Channel assignment TLV: channel number and page 0. */
	header[i++] = 0x03;
	header[i++] = 0x00;
	header[i++] = 0x03;
	header[i++] = 0x00;
	header[i++] = channel & 0xffU;
	header[i++] = channel >> 8;
	header[i++] = 0x00;
	header[i++] = 0x00;

	/* LQI TLV. */
	header[i++] = 0x0a;
	header[i++] = 0x00;
	header[i++] = 0x01;
	header[i++] = 0x00;
	header[i++] = lqi;
	header[i++] = 0x00;
	header[i++] = 0x00;
	header[i++] = 0x00;
}

static void packet_print(const struct sniffer_packet *pkt)
{
	char line[2U * (TAP_HEADER_LEN + PSDU_MAX) + 1U];
	uint8_t header[TAP_HEADER_LEN];
	size_t pos;

	tap_header_build(header, pkt->channel, pkt->rssi, pkt->lqi);

	pos = bin2hex(header, sizeof(header), line, sizeof(line));
	pos += bin2hex(pkt->data, pkt->len, &line[pos], sizeof(line) - pos);
	line[pos] = '\0';

	printk("%s\r\n", line);
}

static void channel_set(uint8_t channel)
{
	current_channel = channel;
	(void)radio_api->set_channel(radio_dev, channel);

	/* Trailing space: zbsniff forwards any run of hex digits, so it would
	 * otherwise turn the channel number into a bogus packet.
	 */
	printk("CH %u \r\n", channel);
}

static int radio_start(void)
{
	struct ieee802154_config config = { .promiscuous = true };
	int rc;

	if (!device_is_ready(radio_dev)) {
		return -ENODEV;
	}

	radio_api = (const struct ieee802154_radio_api *)radio_dev->api;

	rc = radio_api->configure(radio_dev, IEEE802154_CONFIG_PROMISCUOUS, &config);
	if (rc != 0) {
		return rc;
	}

	/* Sniffer: accept every frame, and never inject ACKs into the
	 * network the way a normal 802.15.4 node would.
	 */
	lmac154_disableFrameTypeFiltering();
	lmac154_setRxAcceptPolicy(LMAC154_RX_ACCEPT_ALL);
	lmac154_disableHwAutoTxAck();

	return radio_api->start(radio_dev);
}

static int boot_key_init(void)
{
	int val;

	if (!gpio_is_ready_dt(&boot_key)) {
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&boot_key, GPIO_INPUT) != 0) {
		return -EIO;
	}

	/* Start from the level at boot, so a key already held down does not
	 * step the channel when it is released.
	 */
	val = gpio_pin_get_dt(&boot_key);
	key_raw = val > 0;
	key_stable = key_raw;
	key_raw_time = k_uptime_get();

	return 0;
}

/* Returns true once per debounced press. */
static bool boot_key_pressed(void)
{
	int val = gpio_pin_get_dt(&boot_key);
	int64_t now = k_uptime_get();
	bool pressed = false;

	if (val < 0) {
		return false;
	}

	if ((bool)val != key_raw) {
		key_raw = val;
		key_raw_time = now;
	} else if (key_raw != key_stable && now - key_raw_time >= KEY_DEBOUNCE_MS) {
		key_stable = key_raw;
		pressed = key_stable;
	}

	return pressed;
}

int main(void)
{
	struct sniffer_packet pkt;
	unsigned char c;
	uint8_t channel_index = CHANNEL_START - CHANNEL_FIRST;
	int64_t next_sweep = 0;
	bool sweeping = false;
	int rc;

	printk("BL702 IEEE 802.15.4 sniffer\r\n");

	rc = radio_start();
	if (rc != 0) {
		printk("radio init error: %d\r\n", rc);
		return 0;
	}

	channel_set(CHANNEL_START);

	if (boot_key_init() != 0) {
		printk("BOOT key not available\r\n");
	}

	while (true) {
		while (k_msgq_get(&packet_queue, &pkt, K_NO_WAIT) == 0) {
			packet_print(&pkt);
		}

		if (uart_poll_in(console_dev, &c) == 0) {
			switch (c) {
			case 'c':
				sweeping = false;
				channel_index = (channel_index + 1U) % CHANNEL_COUNT;
				channel_set(CHANNEL_FIRST + channel_index);
				break;
			case 'p':
				sweeping = false;
				channel_index = (channel_index + CHANNEL_COUNT - 1U) %
						CHANNEL_COUNT;
				channel_set(CHANNEL_FIRST + channel_index);
				break;
			case 's':
				sweeping = !sweeping;
				printk("sweep %s\r\n", sweeping ? "on" : "off");
				next_sweep = k_uptime_get();
				break;
			case 'h':
				printk("c: next channel, p: previous channel, "
				       "s: sweep, h: help, BOOT key: next channel\r\n");
				break;
			default:
				break;
			}
		}

		if (boot_key_pressed()) {
			sweeping = false;
			channel_index = (channel_index + 1U) % CHANNEL_COUNT;
			channel_set(CHANNEL_FIRST + channel_index);
		}

		if (sweeping && k_uptime_get() >= next_sweep) {
			channel_index = (channel_index + 1U) % CHANNEL_COUNT;
			channel_set(CHANNEL_FIRST + channel_index);
			next_sweep = k_uptime_get() + SWEEP_INTERVAL_MS;
		}

		k_msleep(1);
	}

	return 0;
}
