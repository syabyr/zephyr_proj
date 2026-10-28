/*
 * UART side of the FTDI dual UART bridge.
 *
 * USB bulk OUT  -> UART TX : done from the UART interrupt handler; the host
 *                            data is queued in a ring buffer by the USB class.
 * UART RX       -> USB bulk IN : queued by the UART interrupt handler and
 *                            drained by the bridge thread, which asks the
 *                            USB class to send when the FTDI latency timer
 *                            expires or a full 62-byte payload accumulated.
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "uart_bridge.h"
#include "usbd_ftdi.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_bridge, LOG_LEVEL_INF);

#define BRIDGE_NODE DT_NODELABEL(ftdi_bridge)

/* FTDI bulk endpoint is 64 bytes, two are taken by the status header. */
#define FTDI_MAX_PAYLOAD 62

#define BRIDGE_THREAD_STACK_SIZE 2048
#define BRIDGE_THREAD_PRIORITY 5

static const struct device *const uart_dev[FTDI_BRIDGE_NUM_PORTS] = {
	DEVICE_DT_GET(DT_NODELABEL(uart0)),
	DEVICE_DT_GET(DT_NODELABEL(uart1)),
};

static const struct gpio_dt_spec dtr_gpio[FTDI_BRIDGE_NUM_PORTS] = {
#if DT_NODE_HAS_PROP(BRIDGE_NODE, uart0_dtr_gpios)
	GPIO_DT_SPEC_GET(BRIDGE_NODE, uart0_dtr_gpios),
#else
	{ 0 },
#endif
#if DT_NODE_HAS_PROP(BRIDGE_NODE, uart1_dtr_gpios)
	GPIO_DT_SPEC_GET(BRIDGE_NODE, uart1_dtr_gpios),
#else
	{ 0 },
#endif
};

static const struct gpio_dt_spec rts_gpio[FTDI_BRIDGE_NUM_PORTS] = {
#if DT_NODE_HAS_PROP(BRIDGE_NODE, uart0_rts_gpios)
	GPIO_DT_SPEC_GET(BRIDGE_NODE, uart0_rts_gpios),
#else
	{ 0 },
#endif
#if DT_NODE_HAS_PROP(BRIDGE_NODE, uart1_rts_gpios)
	GPIO_DT_SPEC_GET(BRIDGE_NODE, uart1_rts_gpios),
#else
	{ 0 },
#endif
};

static const struct gpio_dt_spec led_gpio[FTDI_BRIDGE_NUM_PORTS] = {
#if DT_NODE_HAS_PROP(BRIDGE_NODE, led0_gpios)
	GPIO_DT_SPEC_GET(BRIDGE_NODE, led0_gpios),
#else
	{ 0 },
#endif
#if DT_NODE_HAS_PROP(BRIDGE_NODE, led1_gpios)
	GPIO_DT_SPEC_GET(BRIDGE_NODE, led1_gpios),
#else
	{ 0 },
#endif
};

/* UART -> USB ring: written by the UART ISR, read by the bridge thread. */
static uint8_t uart_to_usb_mem[FTDI_BRIDGE_NUM_PORTS][FTDI_BRIDGE_UART_TO_USB_SIZE];
static struct ring_buf uart_to_usb[FTDI_BRIDGE_NUM_PORTS];
/* Bit per port: discard queued UART data on the next bridge iteration. */
static atomic_t uart_to_usb_flush;

/* USB -> UART ring: written by the USB class, read by the UART ISR. */
static uint8_t usb_to_uart_mem[FTDI_BRIDGE_NUM_PORTS][FTDI_BRIDGE_USB_TO_UART_SIZE];
static struct ring_buf usb_to_uart[FTDI_BRIDGE_NUM_PORTS];
static struct k_spinlock usb_to_uart_lock[FTDI_BRIDGE_NUM_PORTS];

/* Requested line coding, kept so either SET_BAUDRATE or SET_DATA can come
 * first without losing the other half of the configuration.
 */
static uint32_t line_baudrate[FTDI_BRIDGE_NUM_PORTS] = { 115200, 115200 };
static uint8_t line_data_bits[FTDI_BRIDGE_NUM_PORTS] = { 8, 8 };
static uint8_t line_parity[FTDI_BRIDGE_NUM_PORTS] = { 0, 0 };
static uint8_t line_stop_bits[FTDI_BRIDGE_NUM_PORTS] = { 0, 0 };

static int64_t last_in_ms[FTDI_BRIDGE_NUM_PORTS];

K_EVENT_DEFINE(bridge_event);
#define BRIDGE_WAKE_EVENT BIT(0)

static K_THREAD_STACK_DEFINE(bridge_stack, BRIDGE_THREAD_STACK_SIZE);
static struct k_thread bridge_thread_data;

static bool gpio_ready(const struct gpio_dt_spec *spec)
{
	return spec->port != NULL && gpio_is_ready_dt(spec);
}

static void led_set(uint8_t port, bool on)
{
	if (gpio_ready(&led_gpio[port])) {
		gpio_pin_set_dt(&led_gpio[port], on);
	}
}

static void led_toggle(uint8_t port)
{
	if (gpio_ready(&led_gpio[port])) {
		gpio_pin_toggle_dt(&led_gpio[port]);
	}
}

void uart_bridge_wake(void)
{
	k_event_post(&bridge_event, BRIDGE_WAKE_EVENT);
}

void uart_bridge_set_usb_configured(bool configured)
{
	led_set(1, configured);
	uart_bridge_wake();
}

void uart_bridge_indicate_error(void)
{
	for (;;) {
		for (uint8_t port = 0; port < FTDI_BRIDGE_NUM_PORTS; port++) {
			led_set(port, true);
		}
		k_msleep(200);
		for (uint8_t port = 0; port < FTDI_BRIDGE_NUM_PORTS; port++) {
			led_set(port, false);
		}
		k_msleep(1000);
	}
}

static void uart_apply_line_coding(uint8_t port)
{
	struct uart_config cfg = {
		.baudrate = line_baudrate[port],
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	switch (line_data_bits[port]) {
	case 7:
		cfg.data_bits = UART_CFG_DATA_BITS_7;
		break;
	default:
		cfg.data_bits = UART_CFG_DATA_BITS_8;
		break;
	}

	switch (line_parity[port]) {
	case 1: /* FTDI ODD */
		cfg.parity = UART_CFG_PARITY_ODD;
		break;
	case 2: /* FTDI EVEN */
		cfg.parity = UART_CFG_PARITY_EVEN;
		break;
	default: /* NONE; MARK/SPACE are not supported by the UART */
		cfg.parity = UART_CFG_PARITY_NONE;
		break;
	}

	switch (line_stop_bits[port]) {
	case 1:
		cfg.stop_bits = UART_CFG_STOP_BITS_1_5;
		break;
	case 2:
		cfg.stop_bits = UART_CFG_STOP_BITS_2;
		break;
	default:
		cfg.stop_bits = UART_CFG_STOP_BITS_1;
		break;
	}

	if (uart_configure(uart_dev[port], &cfg) != 0) {
		LOG_ERR("Failed to configure UART%u at %u baud", port,
			line_baudrate[port]);
	}
}

void uart_bridge_set_baudrate(uint8_t port, uint32_t baudrate)
{
	if (port >= FTDI_BRIDGE_NUM_PORTS) {
		return;
	}

	line_baudrate[port] = baudrate;
	uart_apply_line_coding(port);
	uart_bridge_flush(port);
}

void uart_bridge_set_format(uint8_t port, uint8_t data_bits, uint8_t parity,
			    uint8_t stop_bits)
{
	if (port >= FTDI_BRIDGE_NUM_PORTS) {
		return;
	}

	line_data_bits[port] = data_bits;
	line_parity[port] = parity;
	line_stop_bits[port] = stop_bits;
	uart_apply_line_coding(port);
	uart_bridge_flush(port);
}

void uart_bridge_set_dtr(uint8_t port, bool dtr)
{
	/* DTR is active low on the wire (GPIO_ACTIVE_LOW in DT). */
	if (port < FTDI_BRIDGE_NUM_PORTS && gpio_ready(&dtr_gpio[port])) {
		gpio_pin_set_dt(&dtr_gpio[port], dtr);
	}
}

void uart_bridge_set_rts(uint8_t port, bool rts)
{
	if (port < FTDI_BRIDGE_NUM_PORTS && gpio_ready(&rts_gpio[port])) {
		gpio_pin_set_dt(&rts_gpio[port], rts);
	}
}

void uart_bridge_flush(uint8_t port)
{
	k_spinlock_key_t key;

	if (port >= FTDI_BRIDGE_NUM_PORTS) {
		return;
	}

	/* The UART->USB ring is only ever reset by its consumer (the bridge
	 * thread), otherwise it would race with the UART ISR producer.
	 */
	atomic_set_bit(&uart_to_usb_flush, port);

	key = k_spin_lock(&usb_to_uart_lock[port]);
	ring_buf_reset(&usb_to_uart[port]);
	uart_irq_tx_disable(uart_dev[port]);
	k_spin_unlock(&usb_to_uart_lock[port], key);

	uart_bridge_wake();
}

size_t uart_bridge_usb_out_space(uint8_t port)
{
	k_spinlock_key_t key;
	size_t space;

	if (port >= FTDI_BRIDGE_NUM_PORTS) {
		return 0;
	}

	key = k_spin_lock(&usb_to_uart_lock[port]);
	space = ring_buf_space_get(&usb_to_uart[port]);
	k_spin_unlock(&usb_to_uart_lock[port], key);

	return space;
}

void uart_bridge_usb_out(uint8_t port, const uint8_t *data, size_t len)
{
	k_spinlock_key_t key;
	size_t copied;

	if (port >= FTDI_BRIDGE_NUM_PORTS || len == 0) {
		return;
	}

	key = k_spin_lock(&usb_to_uart_lock[port]);
	copied = ring_buf_put(&usb_to_uart[port], data, len);
	uart_irq_tx_enable(uart_dev[port]);
	k_spin_unlock(&usb_to_uart_lock[port], key);

	if (copied == 0) {
		LOG_WRN("UART%u TX ring full, dropped %zu bytes", port, len);
	}
}

/*
 * UART interrupt handler.  Runs for both the TX FIFO request (data from the
 * host) and the RX FIFO request (data for the host).
 */
static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t port = (uint8_t)(uintptr_t)user_data;

	uart_irq_update(dev);

	while (uart_irq_is_pending(dev)) {
		k_spinlock_key_t key;
		bool progressed = false;

		key = k_spin_lock(&usb_to_uart_lock[port]);

		if (uart_irq_tx_ready(dev) > 0 &&
		    !ring_buf_is_empty(&usb_to_uart[port])) {
			uint8_t tmp[32];
			size_t len = ring_buf_peek(&usb_to_uart[port], tmp,
						   sizeof(tmp));
			int sent = uart_fifo_fill(dev, tmp, len);

			if (sent > 0) {
				ring_buf_consume(&usb_to_uart[port], sent);
				progressed = true;
			}
		}

		if (ring_buf_is_empty(&usb_to_uart[port])) {
			uart_irq_tx_disable(dev);
		}

		k_spin_unlock(&usb_to_uart_lock[port], key);

		if (uart_irq_rx_ready(dev) > 0) {
			uint8_t tmp[32];
			int len = uart_fifo_read(dev, tmp, sizeof(tmp));

			if (len > 0) {
				uint32_t put = ring_buf_put(&uart_to_usb[port],
							    tmp, len);

				if (put < (uint32_t)len) {
					LOG_WRN("UART%u RX ring full, dropped %u "
						"bytes", port, len - put);
				}
				if (put > 0) {
					led_toggle(port);
				}
				progressed = true;
				uart_bridge_wake();
			}
		}

		if (!progressed) {
			break;
		}
	}
}

/*
 * Send whatever the UART received to the host, honouring the FTDI latency
 * timer: a transfer is started when either a full 62-byte payload is
 * available or the latency timer expired.  With an empty buffer a bare status
 * header is sent, just like a real FTDI device does while idle.
 */
static void uart_to_usb_feed(uint8_t port)
{
	uint8_t payload[FTDI_MAX_PAYLOAD];
	uint32_t latency = ftdi_usb_latency_get(port);
	size_t available = ring_buf_size_get(&uart_to_usb[port]);
	int64_t now = k_uptime_get();
	bool due;
	size_t len;

	/* Do not send bare status packets: leaving an IN transfer queued on an
	 * idle endpoint makes host drivers' open/line-coding requests fail.
	 */
	if (available == 0) {
		return;
	}

	due = (available >= FTDI_MAX_PAYLOAD) ||
	      ((now - last_in_ms[port]) >= (int64_t)latency);
	if (!due) {
		return;
	}

	len = MIN(available, sizeof(payload));
	len = ring_buf_peek(&uart_to_usb[port], payload, len);
	if (len == 0) {
		return;
	}

	if (ftdi_usb_send(port, payload, len) == 0) {
		ring_buf_consume(&uart_to_usb[port], len);
		last_in_ms[port] = now;
	}
}

static void uart_to_usb_discard_pending(uint8_t port)
{
	uint8_t *ptr;
	uint32_t len;

	/* Only the consumer may reset the ring, so drop the data explicitly
	 * instead of touching the indices from the USB control path.
	 */
	while ((len = ring_buf_get_ptr(&uart_to_usb[port], &ptr, 0)) != 0) {
		ring_buf_consume(&uart_to_usb[port], len);
	}
}

static void bridge_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_event_wait(&bridge_event, BRIDGE_WAKE_EVENT, true,
			     K_MSEC(1));

		for (uint8_t port = 0; port < FTDI_BRIDGE_NUM_PORTS; port++) {
			if (atomic_test_and_clear_bit(&uart_to_usb_flush,
						      port)) {
				uart_to_usb_discard_pending(port);
			}

			ftdi_usb_pump(port);
			uart_to_usb_feed(port);
		}
	}
}

static void uart_port_init(uint8_t port)
{
	const struct device *dev = uart_dev[port];

	ring_buf_init(&uart_to_usb[port], sizeof(uart_to_usb_mem[port]),
		      uart_to_usb_mem[port]);
	ring_buf_init(&usb_to_uart[port], sizeof(usb_to_uart_mem[port]),
		      usb_to_uart_mem[port]);

	if (!device_is_ready(dev)) {
		LOG_ERR("UART%u is not ready", port);
		return;
	}

	uart_irq_rx_disable(dev);
	uart_irq_tx_disable(dev);
	uart_irq_callback_user_data_set(dev, uart_isr, (void *)(uintptr_t)port);

	uart_apply_line_coding(port);
	uart_irq_rx_enable(dev);
}

static void gpio_port_init(const struct gpio_dt_spec *spec, bool init_high)
{
	if (!gpio_ready(spec)) {
		return;
	}

	gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	gpio_pin_set_dt(spec, init_high);
}

void uart_bridge_init(void)
{
	for (uint8_t port = 0; port < FTDI_BRIDGE_NUM_PORTS; port++) {
		gpio_port_init(&dtr_gpio[port], false);
		gpio_port_init(&rts_gpio[port], false);
		gpio_port_init(&led_gpio[port], false);
		uart_port_init(port);
		last_in_ms[port] = k_uptime_get();
	}

	k_thread_create(&bridge_thread_data, bridge_stack,
			K_THREAD_STACK_SIZEOF(bridge_stack), bridge_thread,
			NULL, NULL, NULL, BRIDGE_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&bridge_thread_data, "ftdi_bridge");
}
