#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial>=3.5"]
# ///
"""Force a u-blox receiver into a true cold start.

A u-blox receiver retains session state in two places:

- BBR (battery-backed RAM): ephemeris, almanac, health, Klobuchar/iono,
  last position, clock drift, oscillator parameter, UTC/leap, RTC,
  SFDR, AOP.
- SOS (save-on-shutdown) backup in flash: a snapshot the receiver
  restores on boot so it can skip the cold path.

This tool clears both. It sends UBX-UPD-SOS (cmd=1) to wipe the flash
backup, then UBX-CFG-RST with navBbrMask=0xFFFF and a controlled
software reset to clear every BBR section and reboot the firmware.

Order matters: SOS is cleared *before* the reset so nothing is
restored on next boot. UBX-CFG-RST is intentionally not acknowledged
by the receiver (it just resets), so the script does not wait for an
ACK on that step.

Examples (run via uv so dependencies are resolved automatically):

    uv run tools/ubx_cold_start.py --port COM3
    uv run tools/ubx_cold_start.py --port COM4 --baud 38400
    uv run tools/ubx_cold_start.py --port COM3 --scope warm
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print(
        "ERROR: pyserial is missing. Run this script via uv"
        " (`uv run tools/ubx_cold_start.py ...`) or install pyserial manually.",
        file=sys.stderr,
    )
    sys.exit(1)


UBX_SYNC = b"\xB5\x62"

CFG_RST_CLASS = 0x06
CFG_RST_ID = 0x04
UPD_SOS_CLASS = 0x09
UPD_SOS_ID = 0x14
ACK_CLASS = 0x05
ACK_ACK_ID = 0x01
ACK_NAK_ID = 0x00

NAV_BBR_COLD = 0xFFFF
NAV_BBR_WARM = 0x0001
NAV_BBR_HOT = 0x0000

RESET_MODE_HW = 0x00
RESET_MODE_SW = 0x01
RESET_MODE_SW_GNSS = 0x02

UPD_SOS_CMD_CLEAR = 0x01
UPD_SOS_CMD_ACK_CLEAR = 0x03


def ubx_checksum(buffer: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for byte in buffer:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def build_ubx_frame(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    header = struct.pack("<BBH", msg_class, msg_id, len(payload))
    ck_a, ck_b = ubx_checksum(header + payload)
    return UBX_SYNC + header + payload + bytes((ck_a, ck_b))


def build_cfg_rst(nav_bbr_mask: int, reset_mode: int) -> bytes:
    payload = struct.pack("<HBB", nav_bbr_mask, reset_mode, 0x00)
    return build_ubx_frame(CFG_RST_CLASS, CFG_RST_ID, payload)


def build_upd_sos_clear() -> bytes:
    payload = struct.pack("<BBBB", UPD_SOS_CMD_CLEAR, 0, 0, 0)
    return build_ubx_frame(UPD_SOS_CLASS, UPD_SOS_ID, payload)


def iter_ubx_frames(buffer: bytearray):
    while True:
        sync_index = buffer.find(UBX_SYNC)
        if sync_index < 0:
            buffer.clear()
            return
        if sync_index > 0:
            del buffer[:sync_index]
        if len(buffer) < 8:
            return
        length = struct.unpack_from("<H", buffer, 4)[0]
        frame_size = 6 + length + 2
        if len(buffer) < frame_size:
            return
        msg_class = buffer[2]
        msg_id = buffer[3]
        payload = bytes(buffer[6 : 6 + length])
        ck_a_expected, ck_b_expected = ubx_checksum(bytes(buffer[2 : 6 + length]))
        ck_a = buffer[6 + length]
        ck_b = buffer[6 + length + 1]
        del buffer[:frame_size]
        if (ck_a, ck_b) != (ck_a_expected, ck_b_expected):
            continue
        yield msg_class, msg_id, payload


def wait_for_sos_clear_response(ser, timeout_s: float) -> str:
    """Returns 'ACK', 'NAK', or 'TIMEOUT'.

    A receiver may answer the SOS clear with either UBX-UPD-SOS
    cmd=3 (Acknowledge clearing, response field 1=ACK / 0=NAK) or
    a generic UBX-ACK-ACK / UBX-ACK-NAK referencing (0x09, 0x14).
    Both paths are accepted.
    """
    deadline = time.monotonic() + timeout_s
    buffer = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(512)
        if chunk:
            buffer.extend(chunk)
            for msg_class, msg_id, payload in iter_ubx_frames(buffer):
                if (
                    msg_class == UPD_SOS_CLASS
                    and msg_id == UPD_SOS_ID
                    and len(payload) >= 8
                    and payload[0] == UPD_SOS_CMD_ACK_CLEAR
                ):
                    return "ACK" if payload[4] == 1 else "NAK"
                if (
                    msg_class == ACK_CLASS
                    and len(payload) >= 2
                    and payload[0] == UPD_SOS_CLASS
                    and payload[1] == UPD_SOS_ID
                ):
                    return "ACK" if msg_id == ACK_ACK_ID else "NAK"
        else:
            time.sleep(0.01)
    return "TIMEOUT"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Force a u-blox receiver into a true cold start.",
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3 or COM4")
    parser.add_argument(
        "--baud",
        type=int,
        default=9600,
        help="Baud rate. u-blox factory default is 9600; ZED-F9P UART default is 38400.",
    )
    parser.add_argument(
        "--scope",
        choices=["cold", "warm", "hot"],
        default="cold",
        help="cold=clear everything (default), warm=clear ephemeris only, hot=clear nothing",
    )
    parser.add_argument(
        "--reset-mode",
        choices=["sw", "hw", "sw-gnss"],
        default="sw",
        help="sw=controlled software reset (default), hw=hardware watchdog, sw-gnss=GNSS only",
    )
    parser.add_argument(
        "--no-clear-backup",
        action="store_true",
        help="Skip the UBX-UPD-SOS clear step (by default the flash backup is wiped first).",
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=1.5,
        help="Seconds to wait for the SOS clear acknowledgment (default: 1.5)",
    )
    parser.add_argument(
        "--post-reset-wait",
        type=float,
        default=0.5,
        help="Seconds to linger after issuing the cold-start command (default: 0.5)",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    bbr_mask = {"cold": NAV_BBR_COLD, "warm": NAV_BBR_WARM, "hot": NAV_BBR_HOT}[args.scope]
    reset_mode = {
        "sw": RESET_MODE_SW,
        "hw": RESET_MODE_HW,
        "sw-gnss": RESET_MODE_SW_GNSS,
    }[args.reset_mode]

    print(f"Opening {args.port} @ {args.baud} baud")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"ERROR: cannot open {args.port}: {exc}", file=sys.stderr)
        print(
            "  Check the port name, that nothing else (u-center, gpsd) holds it,"
            " and that the baud rate matches the receiver.",
            file=sys.stderr,
        )
        return 2

    with ser:
        ser.reset_input_buffer()

        if not args.no_clear_backup:
            print("Step 1/2: clearing save-on-shutdown backup (UBX-UPD-SOS cmd=1)")
            ser.write(build_upd_sos_clear())
            ser.flush()
            status = wait_for_sos_clear_response(ser, args.ack_timeout)
            if status == "ACK":
                print("  -> acknowledged (flash backup cleared)")
            elif status == "NAK":
                print("  -> NAK (receiver may not support SOS, or had no backup to clear)")
            else:
                print("  -> no response within timeout; continuing")
        else:
            print("Step 1/2: skipped (--no-clear-backup)")

        print(
            f"Step 2/2: cold reset — UBX-CFG-RST navBbrMask=0x{bbr_mask:04X} ({args.scope}),"
            f" resetMode=0x{reset_mode:02X} ({args.reset_mode})"
        )
        ser.write(build_cfg_rst(bbr_mask, reset_mode))
        ser.flush()
        time.sleep(args.post_reset_wait)

    print()
    print("Done. Receiver has been told to forget:")
    if args.scope == "cold":
        print(
            "  ephemeris, almanac, health, Klobuchar/iono, position, clock drift,"
        )
        print(
            "  oscillator, UTC/leap, RTC, SFDR, AOP — every BBR section."
        )
        print("Expect TTFF ~30s or longer (GPS L1) on next acquisition.")
    elif args.scope == "warm":
        print("  ephemeris (almanac, position, time retained).")
    else:
        print("  nothing (firmware restarted; aiding data preserved).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
