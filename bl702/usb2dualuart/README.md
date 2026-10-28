# usb2dualuart for Zephyr / BL702

USB-to-dual-UART bridge, a Zephyr port of
`bl-riscv/bl702_proj/app/usb2dualuart` (SiPEED USB2DUAL, bl_mcu_sdk).

The device enumerates as an FTDI FT2232H (`0403:6010`, bcdDevice `0x0500`) so
that the standard FTDI host drivers bind to it on Linux, macOS and Windows.
Each of the two channels is a normal serial port on the host.

| FTDI channel | USB interface | Bulk IN / OUT | UART  |
| ------------ | ------------- | ------------- | ----- |
| A            | 0             | EP1 IN `0x81` / EP2 OUT `0x02` | UART0 |
| B            | 1             | EP3 IN `0x83` / EP4 OUT `0x04` | UART1 |

## Difference from the original firmware

The original used the bl_mcu_sdk `usb_stack` + `hal_uart` + DMA. This port uses:

* Zephyr's next-generation USB device stack (`CONFIG_USB_DEVICE_STACK_NEXT`)
  with the in-tree BL702 UDC driver (`udc_bflb_v1.c`).
* The Zephyr UART driver with interrupt driven RX/TX instead of DMA.
  USB→UART data is pushed to the UART TX FIFO from the UART ISR; UART→USB
  data is queued by the UART ISR and sent by a small bridge thread when
  either 62 bytes accumulated or the FTDI latency timer expired.
* GPIOs from devicetree (`app.overlay`) instead of `dev_cfg.h` macros.

The FTDI protocol itself is unchanged: vendor requests (`SIO_*`), the 2-byte
status header on every bulk-IN packet, EEPROM image, baud rate divisor
decoding, DTR/RTS handling and per-port latency timers.

## BL702 UDC driver fixes (local Zephyr tree changes)

The in-tree `drivers/usb/udc/udc_bflb_v1.c` needs three fixes for reliable
operation on this board; they are applied as local changes to the Zephyr
checkout (revert with `git -C zephyr checkout -- drivers/usb/udc/udc_bflb_v1.c`):

1. The PHY is re-initialized in `udc_bflb_v1_enable()` instead of only from
   `PRE_KERNEL_1`.  Configuring it before the 48 MHz DLL output is stable
   latched the transceiver in a `UTMI_RX_ERR` state and no control transfer
   was received.
2. The USB error interrupt is not enabled (the vendor SDK does not use it
   either); the continuous `UTMI_RX_ERR` interrupt storm otherwise starves
   the UDC thread.
3. A completion of an EP0 setup buffer whose length is not 8 is put back on
   the queue instead of being submitted, which used to be reported as
   `Malformed setup packet` and made the host's request fail.

With these changes enumeration completes in about a second with the correct
strings and both ports, and opening a port succeeds most of the time.

## Application-side mitigation

As a safety net the application also runs a watchdog: if the host has not
configured the device within 1.5 s, it drops and re-asserts the USB D+ pull-up
(`GLB_USB_ENUM`) so the host enumerates again (up to 60 attempts).  No idle
status-only packets are sent, and `wIndex` 0 (0-based channel numbering used
by some hosts) is accepted as channel A.

## Pins (app.overlay, same wiring as the original firmware)

| Signal        | BL702 pin |
| ------------- | --------- |
| UART0 TX / RX | GPIO14 / GPIO15 |
| UART1 TX / RX | GPIO0 / GPIO1 |
| UART0 DTR/RTS | GPIO23 / GPIO25 |
| UART1 DTR/RTS | GPIO28 / GPIO24 |
| LED0 / LED1   | GPIO9 / GPIO17 |

All GPIO outputs are active low, as on the original hardware. LED0 toggles on
UART0 activity, LED1 on UART1 activity; LED1 is on while USB is configured.
Change the pin numbers (or delete the properties) in `app.overlay`; the
driver code reads them with `GPIO_DT_SPEC_GET()` and skips missing entries.

The overlay also disables `i2c0` (GPIO0/1) and `spi0` (GPIO7/8/9) because the
board definition enables them by default and they collide with UART1, the USB
PHY pins and LED0.

## Console

UART0 is FTDI channel A, therefore the Zephyr console **must not** use it in
normal operation. `prj.conf` disables `CONFIG_CONSOLE`.

For debugging, `debug.conf` (build with `-DEXTRA_CONF_FILE=debug.conf`) enables
the console and USB stack logs on UART0 at 115200; connect a USB-UART or a
J-Link VCOM to GPIO14/GPIO15 and capture the output. Channel A is unusable
while that build is running, and the heavy logging distorts USB timing.

## Build, flash, run

```sh
west build -p always -b dt_xt_zb1_devkit \
    my_zephyr_app/bl702/usb2dualuart -d my_zephyr_app/bl702/usb2dualuart/build
west flash -d my_zephyr_app/bl702/usb2dualuart/build
```

Host side (Linux):

```sh
dmesg | tail                 # "FTDI USB Serial Device converter now attached to ttyUSB0/1"
picocom -b 115200 /dev/ttyUSB0
```

## Notes / limitations

* Remote wakeup is not supported by the BL702 USB controller, so unlike the
  original firmware the configuration is bus powered without the remote
  wakeup attribute (the Zephyr stack rejects it otherwise).
* Endpoint addresses are assigned by the Zephyr USB stack: on the BL702 only
  EP1..EP3 can be OUT and EP4..EP7 IN, so the host sees IN `0x84`/`0x85` and
  OUT `0x01`/`0x02`. This is transparent to the FTDI driver.
* Hardware flow control (RTS/CTS) and XON/XOFF from the host are accepted but
  not implemented; DTR and RTS are only driven as GPIO outputs.
* Parity MARK/SPACE fall back to no parity (the BL702 UART only supports
  none/odd/even); 5/6 data bits fall back to 8.
* Buffer sizes are `FTDI_BRIDGE_*_SIZE` in `src/uart_bridge.h`
  (4 KiB per direction per port, 16 KiB total).
* The latency timer honours the value requested by the host (default 16 ms);
  the original firmware always forced 16 ms.
* A 1200 baud request is ignored, as in the original firmware.
* There is no console output (UART0 is channel A). If USB initialisation
  fails, `main()` blinks both LEDs instead.
