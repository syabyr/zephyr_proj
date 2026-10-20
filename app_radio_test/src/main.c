/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IEEE 802.15.4 TX/RX test for the EFR32MG1P radio using the Gecko SDK
 * RAIL library. The in-tree ieee802154_silabs_efr32 driver only supports
 * Series 2/3, so Series 1 has to drive RAIL directly.
 *
 * The RAIL library defines the radio interrupt handlers; the application
 * only has to connect them to the Zephyr interrupt table and enable them.
 */

#include <string.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <pa_conversions_efr32.h>
#include <rail.h>
#include <rail_ieee802154.h>
#include <sl_power_manager.h>

#define RADIO_CHANNEL		15U
#define TX_PERIOD_MS		2000
#define PAYLOAD_MAX		100

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
		.enable = true,
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

static uint8_t tx_fifo[PAYLOAD_MAX + 16];
static uint8_t rx_packet[PAYLOAD_MAX + 16];

static volatile bool rx_pending;
static volatile uint16_t rx_len;
static volatile int8_t rx_rssi;
static volatile uint8_t rx_lqi;
static volatile uint32_t rx_count;
static volatile uint32_t tx_done_count;
static uint8_t tx_seq;

static void radio_event_handler(RAIL_Handle_t handle, RAIL_Events_t events)
{
	if ((events & RAIL_EVENT_RX_PACKET_RECEIVED) != 0U) {
		RAIL_RxPacketInfo_t info;
		RAIL_RxPacketHandle_t packet;
		RAIL_RxPacketDetails_t details;

		packet = RAIL_GetRxPacketInfo(handle, RAIL_RX_PACKET_HANDLE_NEWEST, &info);
		if (packet != RAIL_RX_PACKET_HANDLE_INVALID) {
			if (info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS && !rx_pending) {
				rx_len = MIN(info.packetBytes, sizeof(rx_packet));
				RAIL_CopyRxPacket(rx_packet, &info);
				(void)RAIL_GetRxPacketDetails(handle, packet, &details);
				rx_rssi = details.rssi;
				rx_lqi = details.lqi;
				rx_count++;
				rx_pending = true;
			}
			(void)RAIL_ReleaseRxPacket(handle, packet);
		}
	}

	if ((events & RAIL_EVENTS_TX_COMPLETION) != 0U) {
		tx_done_count++;
	}
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
		printk("sl_power_manager_init failed\n");
		return -EIO;
	}

	rail_config.eventsCallback = radio_event_handler;
	rail_config.protocol = NULL;
	rail_config.scheduler = NULL;

	rail_handle = RAIL_Init(&rail_config, NULL);
	if (rail_handle == NULL) {
		printk("RAIL_Init failed\n");
		return -EIO;
	}

	status = RAIL_ConfigData(rail_handle, &rail_data_config);
	if (status != RAIL_STATUS_NO_ERROR) {
		printk("RAIL_ConfigData: %d\n", status);
		return -EIO;
	}

	sl_rail_util_pa_init();
	tx_power_config = sl_rail_util_pa_get_tx_power_config_2p4ghz();
	if (tx_power_config == NULL) {
		printk("no 2.4 GHz PA configuration\n");
		return -EIO;
	}
	status = RAIL_ConfigTxPower(rail_handle, tx_power_config);
	if (status != RAIL_STATUS_NO_ERROR) {
		printk("RAIL_ConfigTxPower: %d\n", status);
		return -EIO;
	}
	(void)RAIL_SetTxPowerDbm(rail_handle, 0);

	status = RAIL_IEEE802154_Config2p4GHzRadio(rail_handle);
	if (status != RAIL_STATUS_NO_ERROR) {
		printk("RAIL_IEEE802154_Config2p4GHzRadio: %d\n", status);
		return -EIO;
	}

	status = RAIL_IEEE802154_Init(rail_handle, &ieee802154_config);
	if (status != RAIL_STATUS_NO_ERROR) {
		printk("RAIL_IEEE802154_Init: %d\n", status);
		return -EIO;
	}

	status = RAIL_ConfigEvents(rail_handle, RAIL_EVENTS_ALL,
				   RAIL_EVENT_RX_PACKET_RECEIVED |
				   RAIL_EVENTS_TX_COMPLETION);

	return status;
}

static int radio_send(void)
{
	static const uint8_t payload[] = "hello tradfri";
	size_t psdu_len = 9U + sizeof(payload) - 1U;
	uint16_t written;

	/* PHR: PSDU length plus the two FCS bytes appended by RAIL. */
	tx_fifo[0] = psdu_len + 2U;
	/* FCF: data frame, PAN ID compression, short destination/source. */
	tx_fifo[1] = 0x41;
	tx_fifo[2] = 0x88;
	tx_fifo[3] = tx_seq++;
	/* Destination PAN and short address: broadcast. */
	tx_fifo[4] = 0xFF;
	tx_fifo[5] = 0xFF;
	tx_fifo[6] = 0xFF;
	tx_fifo[7] = 0xFF;
	/* Source short address 0x1234. */
	tx_fifo[8] = 0x34;
	tx_fifo[9] = 0x12;
	memcpy(&tx_fifo[10], payload, sizeof(payload) - 1U);

	written = RAIL_SetTxFifo(rail_handle, tx_fifo, psdu_len + 1U, sizeof(tx_fifo));
	if (written < psdu_len + 1U) {
		printk("TX FIFO too small: %u\n", written);
		return -ENOMEM;
	}

	return RAIL_StartTx(rail_handle, RADIO_CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
}

static void print_rx_packet(void)
{
	uint16_t len = rx_len;

	rx_pending = false;

	printk("RX %u bytes, RSSI %d dBm, LQI %u:", len, rx_rssi, rx_lqi);
	for (uint16_t i = 0; i < len; i++) {
		printk(" %02x", rx_packet[i]);
	}
	printk("\n");
}

int main(void)
{
	RAIL_Version_t version;
	int rc;

	printk("EFR32MG1P IEEE 802.15.4 RAIL test\n");

	rc = radio_init();
	if (rc != 0) {
		printk("radio init failed (%d)\n", rc);
		return 0;
	}

	RAIL_GetVersion(&version, false);
	printk("RAIL %u.%u.%u, channel %u\n", version.major, version.minor, version.rev,
	       RADIO_CHANNEL);

	rc = RAIL_StartRx(rail_handle, RADIO_CHANNEL, NULL);
	printk("RAIL_StartRx: %d\n", rc);

	while (true) {
		if (rx_pending) {
			print_rx_packet();
		}

		rc = radio_send();
		printk("TX seq %u: rc %d, rx %u, tx done %u\n", tx_seq - 1, rc, rx_count,
		       tx_done_count);

		k_msleep(TX_PERIOD_MS);
	}

	return 0;
}
