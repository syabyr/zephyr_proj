# usb2uart for Zephyr / BL702

USB CDC ACM to UART bridge, a Zephyr port of
`bl_mcu_sdk/examples/usb/usb2uart` for the `dt_xt_zb1_devkit` board.

The device enumerates as a CDC ACM serial port (`0x2FE3:0x0002`, product
"Bouffalo Serial", serial number from the chip ID) and bridges it to a
hardware UART:

| USB            | Target UART |
| -------------- | ----------- |
| CDC ACM data   | UART1       |
| TX             | GPIO0       |
| RX             | GPIO1       |

The UART line coding set by the host (baud rate, data bits, parity, stop
bits) is applied to UART1.

## Auto-reset sequence

As in the original firmware, when the host sends the exact string
`BOUFFALOLAB5555RESET` followed by two characters, the message is consumed
instead of being forwarded to UART1 and the DTR/RTS pins are toggled:
`'0'` drives the pin high, `'1'` drives it low, then RTS is toggled and both
pins are released. This can be used by the host tool to reset a target
MCU in bootloader mode.

DTR/RTS use GPIO22 / GPIO21 (same pins as the vendor `dev_cfg.h`); change
them in `app.overlay`.

## Driver requirement

The BL702 UDC driver fixes for reliable enumeration live on the `bl702`
branch of the Zephyr checkout (commit
`drivers: usb: udc: bflb: fix unreliable enumeration`). Build from that
branch:

```sh
git -C zephyr checkout bl702
west build -p always -b dt_xt_zb1_devkit \
    my_zephyr_app/bl702/usb2uart -d my_zephyr_app/bl702/usb2uart/build
west flash -d my_zephyr_app/bl702/usb2uart/build
```

## Notes

* The application keeps UART0 as the board console; it is not bridged.
* `i2c0` (GPIO0/GPIO1) and `spi0` (GPIO7/GPIO8, the USB PHY pins) are
  disabled in `app.overlay`.
* 4 KiB ring buffers in each direction. The host->target direction is
  filtered for the auto-reset pattern before being written to UART1.
* An enumeration watchdog re-attaches the USB D+ pull-up if the host does
  not configure the device within 2 s (up to 200 times), as a safety net
  for the BL702 UDC driver's intermittent enumeration failures.

## Verified

On a dt_xt_zb1_devkit with GPIO0 shorted to GPIO1:

* CDC ACM echo at 115200 and after switching the port to 9600 and back,
  i.e. the host line coding is applied to UART1.
* `BOUFFALOLAB5555RESET01` is consumed and not echoed; data sent afterwards
  is bridged normally.
