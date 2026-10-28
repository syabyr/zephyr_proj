/*
 * USB-to-dual-UART bridge (FTDI FT2232H compatible) for the BL702.
 *
 * Ported from bl-riscv/bl702_proj/app/usb2dualuart.
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "uart_bridge.h"
#include "usbd_ftdi.h"

int main(void)
{
	int ret;

	/* UARTs, GPIOs and the bridge thread first: the USB class starts
	 * feeding endpoints as soon as the host configures the device.
	 */
	uart_bridge_init();

	ret = ftdi_usb_init();

	/* There is no console (UART0 is FTDI channel A).  If USB cannot be
	 * brought up, blink both LEDs so the failure is visible.
	 */
	if (ret != 0) {
		uart_bridge_indicate_error();
	}

	return 0;
}
