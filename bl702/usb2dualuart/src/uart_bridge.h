/*
 * UART side of the FTDI dual UART bridge.
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UART_BRIDGE_H_
#define UART_BRIDGE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/toolchain.h>

#define FTDI_BRIDGE_NUM_PORTS 2

/*
 * Ring buffer sizes, per direction and per port.  The original bl_mcu_sdk
 * application used 8 KiB; 4 KiB per direction keeps static RAM usage at
 * 16 KiB while still buffering ~350 ms at 115200 baud.
 */
#define FTDI_BRIDGE_USB_TO_UART_SIZE 4096
#define FTDI_BRIDGE_UART_TO_USB_SIZE 4096

/* Initialise GPIOs, UARTs, ring buffers and start the bridge thread. */
void uart_bridge_init(void);

/* Called from the USB class when the host changes the line coding. */
void uart_bridge_set_baudrate(uint8_t port, uint32_t baudrate);
void uart_bridge_set_format(uint8_t port, uint8_t data_bits,
			    uint8_t parity, uint8_t stop_bits);
void uart_bridge_set_dtr(uint8_t port, bool dtr);
void uart_bridge_set_rts(uint8_t port, bool rts);

/* Data received from the host (USB bulk OUT), copied into the TX ring. */
void uart_bridge_usb_out(uint8_t port, const uint8_t *data, size_t len);
/* Free space in the USB->UART ring. */
size_t uart_bridge_usb_out_space(uint8_t port);
/* Drop whatever is queued in both directions after a line coding change. */
void uart_bridge_flush(uint8_t port);

/* Wake the bridge thread (e.g. after a USB transfer completed). */
void uart_bridge_wake(void);

/* Reflect USB configured state on the status LED. */
void uart_bridge_set_usb_configured(bool configured);

/*
 * USB initialisation failed and there is no console to report it: blink both
 * LEDs forever so the failure is at least visible.
 */
void uart_bridge_indicate_error(void) FUNC_NORETURN;

#endif /* UART_BRIDGE_H_ */
