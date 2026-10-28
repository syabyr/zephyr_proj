/*
 * FTDI FT2232H-compatible class for the Zephyr USB device stack (next).
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USBD_FTDI_H_
#define USBD_FTDI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Initialise descriptors, register the class and enable the USB device. */
int ftdi_usb_init(void);

/* True when the device is configured and the port accepts data. */
bool ftdi_usb_port_ready(uint8_t port);

/*
 * Queue one bulk-IN transfer carrying the two FTDI status bytes followed by
 * up to 62 payload bytes.  Returns -EBUSY when a transfer is already in
 * flight, -EPERM when the port is not configured.
 */
int ftdi_usb_send(uint8_t port, const uint8_t *payload, size_t len);

/* Keep the bulk-OUT endpoint armed (called from the bridge thread). */
void ftdi_usb_pump(uint8_t port);

/* Latency timer of a port, in milliseconds. */
uint32_t ftdi_usb_latency_get(uint8_t port);

/* EEPROM image returned to the host by SIO_READ_EEPROM. */
#define FTDI_EEPROM_WORDS 64
extern const uint16_t ftdi_eeprom_info[FTDI_EEPROM_WORDS];

#endif /* USBD_FTDI_H_ */
