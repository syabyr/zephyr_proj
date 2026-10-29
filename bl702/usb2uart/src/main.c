/*
 * USB CDC ACM to UART bridge (port of bl_mcu_sdk examples/usb/usb2uart).
 *
 * The device enumerates as a CDC ACM serial port ("Bouffalo Serial",
 * 0xFFFF:0xFFFF) and bridges it to UART1 on GPIO0 (TX) / GPIO1 (RX).
 *
 * The original firmware also implements an auto-reset sequence: when the
 * host sends the exact string "BOUFFALOLAB5555RESET" followed by two
 * characters, the bytes are consumed instead of being forwarded and DTR/RTS
 * are toggled ('0' drives the pin high, '1' low, as in the original).
 *
 * Copyright (c) 2021 Bouffalolab team
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/arch/common/sys_io.h>

#include <bflb_soc.h>
#include <glb_reg.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb2uart, LOG_LEVEL_INF);

#define BRIDGE_NODE DT_NODELABEL(usb2uart)

static const struct device *const cdc_dev =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

static const struct gpio_dt_spec dtr_gpio = GPIO_DT_SPEC_GET(BRIDGE_NODE, dtr_gpios);
static const struct gpio_dt_spec rts_gpio = GPIO_DT_SPEC_GET(BRIDGE_NODE, rts_gpios);

#define RING_SIZE 4096
#define BRIDGE_STACK_SIZE 2048
#define BRIDGE_PRIORITY 5

/* "BOUFFALOLAB5555RESET" plus the two DTR/RTS characters. */
static const char reset_pattern[] = "BOUFFALOLAB5555RESET";
#define RESET_PATTERN_LEN (sizeof(reset_pattern) - 1)
#define RESET_TOTAL_LEN   (RESET_PATTERN_LEN + 2)

/* Host -> target bytes are staged here before the reset filter runs. */
static uint8_t host_mem[RING_SIZE];
static struct ring_buf host_ring;
/* Target -> host bytes. */
static uint8_t target_mem[RING_SIZE];
static struct ring_buf target_ring;

K_EVENT_DEFINE(bridge_event);
#define BRIDGE_WAKE BIT(0)

static K_THREAD_STACK_DEFINE(bridge_stack, BRIDGE_STACK_SIZE);
static struct k_thread bridge_thread_data;

static volatile bool usb_configured;
static atomic_t line_coding_pending;


/* ------------------------------------------------------------------------- */
/* USB descriptors                                                           */
/* ------------------------------------------------------------------------- */

/* Development IDs: 0xffff/0xffff is the Bouffalo ISP bootloader ID and must
 * not be used by the application, otherwise flashing tools (blisp) would
 * mistake the running device for the bootloader.  Replace with your own
 * licensed VID/PID for a product.
 */
USBD_DEVICE_DEFINE(usb2uart_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0002);

USBD_DESC_LANG_DEFINE(usb2uart_lang);
USBD_DESC_MANUFACTURER_DEFINE(usb2uart_mfr, "Bouffalo");
USBD_DESC_PRODUCT_DEFINE(usb2uart_product, "Bouffalo Serial");
USBD_DESC_SERIAL_NUMBER_DEFINE(usb2uart_sn);

/* Bus powered, 100 * 2 mA = 200 mA, as in the original firmware. */
USBD_CONFIGURATION_DEFINE(usb2uart_fs_config, 0, 100, NULL);

/* ------------------------------------------------------------------------- */
/* Enumeration watchdog                                                      */
/* ------------------------------------------------------------------------- */

#define USB_RECONNECT_MS     2000
#define USB_MAX_RECONNECTS   200

static struct k_work_delayable reconnect_work;
static int reconnect_count;

static void reconnect_handler(struct k_work *work)
{
	if (usb_configured) {
		return;
	}

	if (reconnect_count++ >= USB_MAX_RECONNECTS) {
		LOG_ERR("USB enumeration keeps failing, giving up");
		return;
	}

	LOG_WRN("USB not configured, re-attaching (%d)", reconnect_count);

	sys_clear_bits(GLB_BASE + GLB_USB_XCVR_OFFSET, GLB_USB_ENUM_MSK);
	k_msleep(100);
	sys_set_bits(GLB_BASE + GLB_USB_XCVR_OFFSET, GLB_USB_ENUM_MSK);

	k_work_reschedule(&reconnect_work, K_MSEC(USB_RECONNECT_MS));
}

/* ------------------------------------------------------------------------- */
/* UART plumbing                                                             */
/* ------------------------------------------------------------------------- */

static void uart_write_all(const struct device *dev, const uint8_t *data,
			   size_t len)
{
	int64_t deadline = k_uptime_get() + 1000;
	size_t off = 0;

	while (off < len) {
		int sent = uart_fifo_fill(dev, data + off, len - off);

		if (sent > 0) {
			off += sent;
			continue;
		}

		if (k_uptime_get() > deadline) {
			LOG_WRN("UART TX timeout, dropped %zu bytes", len - off);
			return;
		}

		k_usleep(50);
	}
}

static void cdc_isr(const struct device *dev, void *user_data)
{
	uint8_t buf[32];

	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) > 0) {
		int len = uart_fifo_read(dev, buf, sizeof(buf));

		if (len <= 0) {
			break;
		}

		(void)ring_buf_put(&host_ring, buf, len);
	}

	if (uart_irq_tx_ready(dev) > 0) {
		uint32_t len = ring_buf_peek(&target_ring, buf, sizeof(buf));

		if (len != 0) {
			int sent = uart_fifo_fill(dev, buf, len);

			if (sent > 0) {
				ring_buf_consume(&target_ring, sent);
			}
		}

		if (ring_buf_is_empty(&target_ring)) {
			uart_irq_tx_disable(dev);
		}
	}

	k_event_post(&bridge_event, BRIDGE_WAKE);
}

static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t buf[32];

	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) > 0) {
		int len = uart_fifo_read(dev, buf, sizeof(buf));

		if (len <= 0) {
			break;
		}

		(void)ring_buf_put(&target_ring, buf, len);
	}

	k_event_post(&bridge_event, BRIDGE_WAKE);
}

/* ------------------------------------------------------------------------- */
/* Auto-reset filter                                                         */
/* ------------------------------------------------------------------------- */

static uint8_t filter_buf[RESET_TOTAL_LEN];
static size_t filter_len;
static int64_t filter_start;

static bool gpio_ready(const struct gpio_dt_spec *spec)
{
	return spec->port != NULL && gpio_is_ready_dt(spec);
}

/*
 * Run the same DTR/RTS sequence as the original firmware: '0' drives the pin
 * high, '1' drives it low, then RTS is toggled and both pins are released.
 */
static void reset_sequence(uint8_t dtr_ch, uint8_t rts_ch)
{
	bool dtr_high = (dtr_ch == '0' || dtr_ch == 0);
	bool rts_high = (rts_ch == '0' || rts_ch == 0);

	if (!gpio_ready(&dtr_gpio) || !gpio_ready(&rts_gpio)) {
		LOG_WRN("DTR/RTS GPIOs not available, reset ignored");
		return;
	}

	LOG_INF("auto-reset sequence received");

	(void)gpio_pin_configure_dt(&dtr_gpio, GPIO_OUTPUT);
	(void)gpio_pin_configure_dt(&rts_gpio, GPIO_OUTPUT);

	gpio_pin_set_dt(&dtr_gpio, dtr_high);
	gpio_pin_set_dt(&rts_gpio, rts_high);
	k_msleep(10);

	gpio_pin_set_dt(&rts_gpio, !rts_high);
	k_msleep(10);

	(void)gpio_pin_configure_dt(&rts_gpio, GPIO_INPUT);
	k_msleep(5);
	(void)gpio_pin_configure_dt(&dtr_gpio, GPIO_INPUT);
}

static void filter_flush(void)
{
	if (filter_len != 0) {
		uart_write_all(uart_dev, filter_buf, filter_len);
		filter_len = 0;
	}
}

static void filter_byte(uint8_t byte)
{
	filter_buf[filter_len++] = byte;

	if (memcmp(filter_buf, reset_pattern,
		   MIN(filter_len, RESET_PATTERN_LEN)) != 0) {
		filter_flush();
		return;
	}

	if (filter_len == 1) {
		filter_start = k_uptime_get();
	}

	if (filter_len == RESET_TOTAL_LEN) {
		reset_sequence(filter_buf[RESET_PATTERN_LEN],
				filter_buf[RESET_PATTERN_LEN + 1]);
		filter_len = 0;
	}
}

/* ------------------------------------------------------------------------- */
/* Bridge thread                                                             */
/* ------------------------------------------------------------------------- */

static void pump_host_to_target(void)
{
	uint8_t buf[64];
	uint32_t len;

	while ((len = ring_buf_get(&host_ring, buf, sizeof(buf))) > 0) {
		for (uint32_t i = 0; i < len; i++) {
			filter_byte(buf[i]);
		}
	}

	/* Do not keep a partial pattern prefix buffered forever. */
	if (filter_len != 0 && (k_uptime_get() - filter_start) > 20) {
		filter_flush();
	}
}

static void pump_target_to_host(void)
{
	/* The CDC ACM class expects its UART TX path to be driven from the
	 * UART IRQ callback; just kick it when there is data waiting.
	 */
	if (!ring_buf_is_empty(&target_ring)) {
		uart_irq_tx_enable(cdc_dev);
	}
}

static void apply_line_coding(void)
{
	struct uart_config cfg;

	if (uart_config_get(cdc_dev, &cfg) != 0) {
		return;
	}

	if (uart_configure(uart_dev, &cfg) != 0) {
		LOG_ERR("Failed to configure UART1");
		return;
	}

	LOG_INF("UART1 -> %u baud, %u data bits, parity %u, stop %u",
		cfg.baudrate, cfg.data_bits, cfg.parity, cfg.stop_bits);

	LOG_DBG("line coding: %u baud", cfg.baudrate);
}

static void bridge_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_event_wait(&bridge_event, BRIDGE_WAKE, true, K_MSEC(20));

		if (atomic_test_and_clear_bit(&line_coding_pending, 0)) {
			apply_line_coding();
		}

		pump_host_to_target();
		pump_target_to_host();
	}
}

/* ------------------------------------------------------------------------- */
/* USB setup                                                                 */
/* ------------------------------------------------------------------------- */

static void usb_msg_cb(struct usbd_context *const uds_ctx,
		       const struct usbd_msg *const msg)
{
	switch (msg->type) {
	case USBD_MSG_CONFIGURATION:
		usb_configured = true;
		(void)k_work_cancel_delayable(&reconnect_work);
		LOG_INF("USB configured");
		break;
	case USBD_MSG_RESET:
		usb_configured = false;
		break;
	case USBD_MSG_CDC_ACM_LINE_CODING:
		atomic_set_bit(&line_coding_pending, 0);
		k_event_post(&bridge_event, BRIDGE_WAKE);
		break;
	case USBD_MSG_VBUS_READY:
		if (usbd_can_detect_vbus(uds_ctx) && usbd_enable(uds_ctx) != 0) {
			LOG_ERR("Failed to enable USB device support");
		}
		break;
	case USBD_MSG_VBUS_REMOVED:
		if (usbd_can_detect_vbus(uds_ctx) && usbd_disable(uds_ctx) != 0) {
			LOG_ERR("Failed to disable USB device support");
		}
		break;
	default:
		break;
	}
}

static int usb_setup(void)
{
	int err;

	err = usbd_add_descriptor(&usb2uart_usbd, &usb2uart_lang);
	if (err == 0) {
		err = usbd_add_descriptor(&usb2uart_usbd, &usb2uart_mfr);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&usb2uart_usbd, &usb2uart_product);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&usb2uart_usbd, &usb2uart_sn);
	}
	if (err == 0) {
		err = usbd_add_configuration(&usb2uart_usbd, USBD_SPEED_FS,
					     &usb2uart_fs_config);
	}
	if (err == 0) {
		err = usbd_register_all_classes(&usb2uart_usbd, USBD_SPEED_FS, 1,
						NULL);
	}
	if (err == 0) {
		err = usbd_device_set_bcd_device(&usb2uart_usbd, 0x0100);
	}
	if (err != 0) {
		LOG_ERR("Failed to configure USB descriptors (%d)", err);
		return err;
	}

	usbd_msg_register_cb(&usb2uart_usbd, usb_msg_cb);

	err = usbd_init(&usb2uart_usbd);
	if (err == 0) {
		err = usbd_enable(&usb2uart_usbd);
	}
	if (err != 0) {
		LOG_ERR("Failed to start USB device support (%d)", err);
		return err;
	}

	k_work_reschedule(&reconnect_work, K_MSEC(USB_RECONNECT_MS));

	return 0;
}

/* ------------------------------------------------------------------------- */
/* Initialisation                                                            */
/* ------------------------------------------------------------------------- */

int main(void)
{
	int err;

	ring_buf_init(&host_ring, sizeof(host_mem), host_mem);
	ring_buf_init(&target_ring, sizeof(target_mem), target_mem);

	k_work_init_delayable(&reconnect_work, reconnect_handler);

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART1 is not ready");
		return 0;
	}

	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM UART is not ready");
		return 0;
	}

	/* Leave DTR/RTS released until the reset sequence needs them. */
	if (gpio_ready(&rts_gpio)) {
		(void)gpio_pin_configure_dt(&rts_gpio, GPIO_INPUT);
	}
	if (gpio_ready(&dtr_gpio)) {
		(void)gpio_pin_configure_dt(&dtr_gpio, GPIO_INPUT);
	}

	k_thread_create(&bridge_thread_data, bridge_stack,
			K_THREAD_STACK_SIZEOF(bridge_stack), bridge_thread,
			NULL, NULL, NULL, BRIDGE_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&bridge_thread_data, "usb2uart");

	apply_line_coding();

	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	(void)uart_irq_callback_user_data_set(cdc_dev, cdc_isr, NULL);
	uart_irq_rx_enable(cdc_dev);

	err = usb_setup();
	if (err != 0) {
		LOG_ERR("USB initialisation failed: %d", err);
	}

	return 0;
}
