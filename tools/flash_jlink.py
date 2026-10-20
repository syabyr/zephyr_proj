#!/usr/bin/env python3
"""Flash an EFR32MG1P target over J-Link using only pylink + libjlinkarm.dylib.

No JLinkExe / full SEGGER install is required: pylink loads the self-contained
J-Link SDK shared library through ctypes and drives it directly.

Usage:
    python flash_jlink.py /tmp/ikea_tradfri_build/zephyr/zephyr.hex
    python flash_jlink.py --addr 0x0 firmware.bin
    python flash_jlink.py --list
"""
import argparse
import sys

import pylink

# The single self-contained J-Link SDK shared library. This one file is all the
# "binary installation" that pylink needs; it can be copied to any machine.
JLINK_LIB = "/Users/mybays/.platformio/packages/tool-jlink/libjlinkarm.9.20.0.dylib"

DEVICE = "EFR32MG1PxxxF256"
SERIAL = "190173"          # J-Link OB S/N; use None to auto-pick the first probe


def list_probes(jlink):
    for i, info in enumerate(jlink.connected_emulators()):
        print(f"  [{i}] {info.SerialNumber}  {info.acProduct}  {info.acNickname or ''}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", nargs="?", help="path to .hex/.bin/.elf to flash")
    ap.add_argument("--addr", default="0x0", help="load address for .bin (hex/elf ignore it)")
    ap.add_argument("--serial", default=SERIAL, help="J-Link serial number")
    ap.add_argument("--lib", default=JLINK_LIB, help="libjlinkarm .dylib path")
    ap.add_argument("--speed", default=4000, help="SWD clock in kHz")
    ap.add_argument("--list", action="store_true", help="list probes and exit")
    ap.add_argument("--norun", action="store_true", help="leave CPU halted after flash")
    args = ap.parse_args()

    lib = pylink.Library(dllpath=args.lib)
    jlink = pylink.JLink(lib=lib)
    jlink.open(serial_no=args.serial or None)

    if args.list:
        list_probes(jlink)
        return

    if not args.firmware:
        ap.error("firmware path required (or use --list)")

    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(DEVICE, speed=args.speed)
    print(f"connected: {DEVICE}  core={jlink.core_id():#x}")

    jlink.reset(halt=True)

    addr = int(args.addr, 0)
    print(f"flashing {args.firmware} (addr={addr:#x}) ...")
    # JLINK_DownloadFile auto-erases sectors as needed and verifies; the
    # callback is (action, progress_string, percentage).
    jlink.flash_file(args.firmware, addr, on_progress=lambda action, pstr, pct: print(
        f"    {action}: {pct:3d}%", flush=True))

    if args.norun:
        print("done (CPU halted)")
    else:
        jlink.reset(halt=False)   # reset and run
        print("done (reset & run)")

    jlink.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except pylink.errors.JLinkException as exc:
        print(f"J-Link error: {exc}", file=sys.stderr)
        sys.exit(1)