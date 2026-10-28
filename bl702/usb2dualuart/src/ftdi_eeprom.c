/*
 * FTDI EEPROM image returned to the host through SIO_READ_EEPROM.
 *
 * Values are kept byte-for-byte identical to the original bl_mcu_sdk
 * usb2dualuart application (SiPEED USB TO DUALUART, 0403:6010, bcd 0x0500).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

const uint16_t ftdi_eeprom_info[] = {
	0x0808, 0x0403, 0x6010, 0x0500, 0x32A0, 0x0008, 0x0200, 0x0e96,
	0x20a4, 0x0ec4, 0x0046, 0x030e, 'S',    'I',    'P',    'E',
	'E',    'D',    0x0320, 'U',    'S',    'B',    ' ',    'T',
	'O',    ' ',    'D',    'U',    'A',    'L',    'U',    'A',
	'R',    'T',    0x030e, 'S',    'I',    '8',    '8',    '4',
	'8',    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x827E,
};
