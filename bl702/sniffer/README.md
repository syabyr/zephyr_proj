# IEEE 802.15.4 sniffer for the BL702

Promiscuous 802.15.4 sniffer for `dt_xt_zb1_devkit` (Bouffalo Lab BL702),
modelled on `zephyr_proj/ikea_tradfri/sniffer`. Frames are printed on the
console UART as IEEE 802.15.4 TAP hex lines, which `zbsniff` wraps into a
`LINKTYPE_IEEE802_15_4_TAP` pcap.

The application uses the in-tree `ieee802154_bflb` driver in raw mode
(`CONFIG_IEEE802154_RAW_MODE`): there is no MAC stack, `lmac154` delivers every
received frame to the application's `net_recv_data()`, and the application
prints it. Hardware auto-ACK is disabled so the sniffer never answers frames.

## Build

The `lmac154` and BL702 RF libraries are prebuilt blobs:

```sh
cd /path/to/zephyrproject
west blobs fetch hal_bouffalolab
west build -p always -b dt_xt_zb1_devkit zephyr_proj/bl702/sniffer
```

Do not run plain `west update` in this workspace (it resets the forked module
checkouts).

## Flash

```sh
west flash                       # from the build directory
```

or, with the Doiting/Bouffalo `blisp` tool:

```sh
blisp write -c bl70x --reset build/zephyr/zephyr.bin
```

## Capture

The console runs at 115200 baud on the board's USB serial port. `zbsniff`
(tcpdump's `zbsniff.py`, or the copy shipped with this project) reads the hex
lines and writes a pcap:

```sh
python3 zbsniff.py -d /dev/ttyUSB0 -o capture.pcap
```

The pcap can be opened in Wireshark, which dissects TAP with RSSI, channel and
LQI per frame.

## Channel control

The channel can be changed from the console, or with the BOOT key (D28): each
press stops sweeping and advances to the next channel.

| Input       | Action                         |
|-------------|--------------------------------|
| `c`         | next channel (11..26, wraps)   |
| `p`         | previous channel               |
| `s`         | toggle 500 ms/channel sweeping |
| `h`         | print the key help             |
| BOOT (D28)  | next channel, stops sweeping   |

D28 is the same pin the BL702 samples at reset to enter the UART bootloader,
so it reads high while pressed. The key is configured in `app.overlay` as
`GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN`; holding it while resetting the board
enters the bootloader instead of running the sniffer.

The sniffer starts on channel 15. Every channel change is announced with
`CH <n> `, which `zbsniff` records in the pcap.

## Output format

Each frame is one line of hex digits:

- 36-byte TAP header: version/length, FCS-type TLV (`0x0000`, no FCS),
  RSS TLV (`0x0001`, IEEE-754 float dBm), channel-assignment TLV (`0x0003`,
  channel + page 0) and LQI TLV (`0x000a`).
- The MPDU, without the FCS and without the PHR.
