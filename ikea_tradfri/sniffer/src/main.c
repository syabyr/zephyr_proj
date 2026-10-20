/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IEEE 802.15.4 sniffer for the EFR32MG1P (Series 1). The in-tree
 * ieee802154_silabs_efr32 driver targets Series 2/3, so this application
 * drives the Gecko SDK RAIL library directly.
 *
 * Received frames are printed as IEEE 802.15.4 TAP hex lines that zbsniff
 * can wrap into a LINKTYPE_IEEE802_15_4_TAP pcap. The PA0 button steps the
 * radio through channels 11..26 and wraps around.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <em_gpio.h>
#include <pa_conversions_efr32.h>
#include <rail.h>
#include <rail_ieee802154.h>
#include <sl_power_manager.h>

#define CHANNEL_FIRST		11U
#define CHANNEL_LAST		26U
#define CHANNEL_START		15U
#define CHANNEL_COUNT		(CHANNEL_LAST - CHANNEL_FIRST + 1U)

#define BUTTON_DEBOUNCE_MS	50
#define LED_HALF_MS		30

#define PACKET_QUEUE_DEPTH	8U
#define PSDU_MAX		127U
#define TAP_HEADER_LEN		36U

/* The RAIL library provides these handlers; the application only has to
 * connect and enable them.
 */
extern void FRC_PRI_IRQHandler(void);
extern void FRC_IRQHandler(void);
extern void MODEM_IRQHandler(void);
extern void RAC_SEQ_IRQHandler(void);
extern void RAC_RSM_IRQHandler(void);
extern void BUFC_IRQHandler(void);
extern void AGC_IRQHandler(void);
extern void PROTIMER_IRQHandler(void);
extern void SYNTH_IRQHandler(void);
extern void RFSENSE_IRQHandler(void);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);

static RAIL_Handle_t rail_handle;

static RAIL_Config_t rail_config;

static const RAIL_DataConfig_t rail_data_config = {
	.txSource = TX_PACKET_DATA,
	.rxSource = RX_PACKET_DATA,
	.txMethod = PACKET_MODE,
	.rxMethod = PACKET_MODE,
};

static const RAIL_IEEE802154_Config_t ieee802154_config = {
	.addresses = NULL,
	.ackConfig = {
		/* A sniffer must not inject ACKs into the network. */
		.enable = false,
		.ackTimeout = 672,
		.rxTransitions = {
			.success = RAIL_RF_STATE_RX,
			.error = RAIL_RF_STATE_RX,
		},
		.txTransitions = {
			.success = RAIL_RF_STATE_RX,
			.error = RAIL_RF_STATE_RX,
		},
	},
	.timings = {
		.idleToRx = 100,
		.idleToTx = 100,
		.rxToTx = 192,
		.txToRx = 192,
		.rxSearchTimeout = 0,
		.txToRxSearchTimeout = 0,
	},
	.framesMask = RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES,
	.promiscuousMode = true,
	.isPanCoordinator = false,
	.defaultFramePendingInOutgoingAcks = false,
};

struct sniffer_packet {
	/* PHR followed by the PSDU, as delivered by RAIL. */
	uint8_t data[PSDU_MAX + 1U];
	uint16_t len; /* PSDU length, excluding the PHR */
	int8_t rssi;
	uint8_t lqi;
	uint8_t channel;
};

K_MSGQ_DEFINE(packet_queue, sizeof(struct sniffer_packet), PACKET_QUEUE_DEPTH, 4);

/* Scratch packet for the radio interrupt: too large for the 2 kB ISR stack,
 * and callbacks from a single radio interrupt cannot nest.
 */
static struct sniffer_packet rx_packet;

static uint8_t channel_index = CHANNEL_START - CHANNEL_FIRST;
static volatile uint8_t current_channel = CHANNEL_START;

static volatile bool channel_changed;
static volatile bool switch_blink;
static volatile bool rx_blink;

static void button_debounce_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	(void)gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
}

/* Contact bounce is suppressed by disabling the pin interrupt on the first
 * edge and re-enabling it once the button has settled.
 */
K_TIMER_DEFINE(button_debounce, button_debounce_handler, NULL);

static void radio_irq_init(void)
{
	IRQ_CONNECT(FRC_PRI_IRQn, 0, FRC_PRI_IRQHandler, 0, 0);
	IRQ_CONNECT(FRC_IRQn, 0, FRC_IRQHandler, 0, 0);
	IRQ_CONNECT(MODEM_IRQn, 0, MODEM_IRQHandler, 0, 0);
	IRQ_CONNECT(RAC_SEQ_IRQn, 0, RAC_SEQ_IRQHandler, 0, 0);
	IRQ_CONNECT(RAC_RSM_IRQn, 0, RAC_RSM_IRQHandler, 0, 0);
	IRQ_CONNECT(BUFC_IRQn, 0, BUFC_IRQHandler, 0, 0);
	IRQ_CONNECT(AGC_IRQn, 0, AGC_IRQHandler, 0, 0);
	IRQ_CONNECT(PROTIMER_IRQn, 0, PROTIMER_IRQHandler, 0, 0);
	IRQ_CONNECT(SYNTH_IRQn, 0, SYNTH_IRQHandler, 0, 0);
	IRQ_CONNECT(RFSENSE_IRQn, 0, RFSENSE_IRQHandler, 0, 0);
	/* The RAIL blob uses the sleeptimer service, which runs off RTCC. RTCC
	 * is also the kernel's clock source, so its interrupt is already
	 * connected and enabled by the sleeptimer system timer driver.
	 */

	irq_enable(FRC_PRI_IRQn);
	irq_enable(FRC_IRQn);
	irq_enable(MODEM_IRQn);
	irq_enable(RAC_SEQ_IRQn);
	irq_enable(RAC_RSM_IRQn);
	irq_enable(BUFC_IRQn);
	irq_enable(AGC_IRQn);
	irq_enable(PROTIMER_IRQn);
	irq_enable(SYNTH_IRQn);
	irq_enable(RFSENSE_IRQn);
}

static void channel_set(uint8_t channel)
{
	(void)RAIL_Idle(rail_handle, RAIL_IDLE_FORCE_SHUTDOWN_CLEAR_FLAGS, true);
	current_channel = channel;
	(void)RAIL_StartRx(rail_handle, channel, NULL);
}

static void radio_event_handler(RAIL_Handle_t handle, RAIL_Events_t events)
{
	if ((events & RAIL_EVENT_RX_PACKET_RECEIVED) == 0U) {
		return;
	}

	RAIL_RxPacketInfo_t info;
	RAIL_RxPacketHandle_t packet;

	packet = RAIL_GetRxPacketInfo(handle, RAIL_RX_PACKET_HANDLE_NEWEST, &info);
	if (packet == RAIL_RX_PACKET_HANDLE_INVALID) {
		return;
	}

	if (info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS &&
	    info.packetBytes > 1U && info.packetBytes <= sizeof(rx_packet.data)) {
		RAIL_RxPacketDetails_t details = {
			.timeReceived = {
				.timePosition = RAIL_PACKET_TIME_DEFAULT,
				.totalPacketBytes = 0,
			},
		};

		RAIL_CopyRxPacket(rx_packet.data, &info);
		rx_packet.len = info.packetBytes - 1U;
		rx_packet.channel = current_channel;
		rx_packet.rssi = 0;
		rx_packet.lqi = 0;
		if (RAIL_GetRxPacketDetails(handle, packet, &details) == RAIL_STATUS_NO_ERROR) {
			rx_packet.rssi = details.rssi;
			rx_packet.lqi = details.lqi;
		}

		/* Print from the main thread: a TAP line takes ~30 ms on the
		 * 115200 baud console, far too long for the radio interrupt.
		 */
		(void)k_msgq_put(&packet_queue, &rx_packet, K_NO_WAIT);
		rx_blink = true;
	}

	(void)RAIL_ReleaseRxPacket(handle, packet);
}

static int radio_init(void)
{
	RAIL_Status_t status;
	RAIL_TxPowerConfig_t *tx_power_config;

	radio_irq_init();

	/* RAIL uses these services; the power manager also initializes the
	 * sleeptimer that RAIL schedules its internal timeouts with.
	 */
	if (sl_power_manager_init() != SL_STATUS_OK) {
		return -EIO;
	}

	rail_config.eventsCallback = radio_event_handler;
	rail_config.protocol = NULL;
	rail_config.scheduler = NULL;

	rail_handle = RAIL_Init(&rail_config, NULL);
	if (rail_handle == NULL) {
		return -EIO;
	}

	status = RAIL_ConfigData(rail_handle, &rail_data_config);
	if (status != RAIL_STATUS_NO_ERROR) {
		return -EIO;
	}

	sl_rail_util_pa_init();
	tx_power_config = sl_rail_util_pa_get_tx_power_config_2p4ghz();
	if (tx_power_config == NULL) {
		return -EIO;
	}
	status = RAIL_ConfigTxPower(rail_handle, tx_power_config);
	if (status != RAIL_STATUS_NO_ERROR) {
		return -EIO;
	}
	(void)RAIL_SetTxPowerDbm(rail_handle, 0);

	status = RAIL_IEEE802154_Config2p4GHzRadio(rail_handle);
	if (status != RAIL_STATUS_NO_ERROR) {
		return -EIO;
	}

	status = RAIL_IEEE802154_Init(rail_handle, &ieee802154_config);
	if (status != RAIL_STATUS_NO_ERROR) {
		return -EIO;
	}

	status = RAIL_ConfigEvents(rail_handle, RAIL_EVENTS_ALL,
				   RAIL_EVENT_RX_PACKET_RECEIVED);
	if (status != RAIL_STATUS_NO_ERROR) {
		return -EIO;
	}

	(void)RAIL_StartRx(rail_handle, current_channel, NULL);

	return 0;
}

static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	(void)gpio_pin_interrupt_configure_dt(&button, GPIO_INT_DISABLE);
	k_timer_start(&button_debounce, K_MSEC(BUTTON_DEBOUNCE_MS), K_NO_WAIT);

	channel_index = (channel_index + 1U) % CHANNEL_COUNT;
	channel_set(CHANNEL_FIRST + channel_index);
	channel_changed = true;
	switch_blink = true;
}

static int button_init(void)
{
	static struct gpio_callback button_cb;
	int rc;

	if (!gpio_is_ready_dt(&button)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (rc != 0) {
		return rc;
	}

	/* The Zephyr gecko GPIO driver has no flag for the pin glitch filter,
	 * so enable it through emlib.
	 */
	GPIO_PinModeSet((GPIO_Port_TypeDef)DT_PROP(DT_NODELABEL(gpioa), peripheral_id),
			button.pin, gpioModeInputPullFilter, 1);

	gpio_init_callback(&button_cb, button_isr, BIT(0));
	rc = gpio_add_callback_dt(&button, &button_cb);
	if (rc != 0) {
		return rc;
	}

	return gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
}

static void led_service(void)
{
	static uint8_t toggles_left;
	static int64_t next_toggle;
	int64_t now = k_uptime_get();

	if (toggles_left == 0U) {
		if (switch_blink) {
			switch_blink = false;
			toggles_left = 4U; /* two blinks */
		} else if (rx_blink) {
			rx_blink = false;
			toggles_left = 2U; /* one blink */
		} else {
			return;
		}
		next_toggle = now;
	}

	if (now < next_toggle) {
		return;
	}

	(void)gpio_pin_toggle_dt(&led);
	toggles_left--;
	next_toggle = now + LED_HALF_MS;
}

static void tap_header_build(uint8_t *header, uint16_t channel, int8_t rssi, uint8_t lqi)
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
	pos += bin2hex(&pkt->data[1], pkt->len, &line[pos], sizeof(line) - pos);
	line[pos] = '\0';

	printk("%s\r\n", line);
}

int main(void)
{
	struct sniffer_packet pkt;
	int rc;

	printk("EFR32MG1P IEEE 802.15.4 sniffer\r\n");

	rc = radio_init();
	if (rc != 0) {
		printk("radio init error\r\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		printk("LED init error\r\n");
	} else {
		/* gpio_gecko does not apply GPIO_ACTIVE_LOW to output levels,
		 * so set the physical level: the indicator LED is active low
		 * and starts off.
		 */
		(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_INIT_HIGH);
	}

	rc = button_init();
	if (rc != 0) {
		printk("button init error\r\n");
	}

	/* Trailing space: zbsniff forwards any run of hex digits, so it would
	 * otherwise turn the channel number into a bogus packet.
	 */
	printk("CH %u \r\n", current_channel);

	while (true) {
		if (channel_changed) {
			channel_changed = false;
			printk("CH %u \r\n", current_channel);
		}

		if (k_msgq_get(&packet_queue, &pkt, K_NO_WAIT) == 0) {
			packet_print(&pkt);
		} else {
			k_msleep(1);
		}

		led_service();
	}

	return 0;
}
