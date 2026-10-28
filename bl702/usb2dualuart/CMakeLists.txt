# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb2dualuart)

target_sources(app PRIVATE
	src/main.c
	src/ftdi_eeprom.c
	src/usbd_ftdi.c
	src/uart_bridge.c
)
