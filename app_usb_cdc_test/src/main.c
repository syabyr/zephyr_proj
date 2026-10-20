/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>

/*
 * CDC ACM echo + periodic transmit test.
 * The zephyr_cdc_acm_uart0 node is created by the app.overlay.
 */

static const struct device *const cdc_dev =
	DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

int main(void)
{
	int ret;
	uint32_t count = 0;
	unsigned char ch;

	printk("=== USB CDC ACM Test (Tiva C) ===\n");

	if (!device_is_ready(cdc_dev)) {
		printk("CDC ACM device not ready\n");
		return 0;
	}

	printk("CDC ACM device: %s\n", cdc_dev->name);

	/* Wait for the host to open the port (DTR) */
	printk("Waiting for host to open port...\n");

	while (1) {
		ret = uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &count);
		if (ret == 0 && count != 0) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	printk("Port opened, starting echo + beacon\n");

	/* Line coding: 115200 8N1 (virtual, informational only) */
	uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_BAUD_RATE, 115200);

	count = 0;

	while (1) {
		/* Echo any received character */
		while (uart_fifo_read(cdc_dev, &ch, 1) == 1) {
			uart_fifo_fill(cdc_dev, &ch, 1);
		}

		/* Periodic test pattern: 0x00-0x3F ramp so bit alignment
		 * errors are visible in the received stream */
		for (int i = 0; i < 64; i++) {
			uart_poll_out(cdc_dev, (unsigned char)(i & 0x3F));
		}
		count++;

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
