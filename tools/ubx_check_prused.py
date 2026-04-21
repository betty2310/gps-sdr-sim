#!/usr/bin/env python3
"""Inspect one satellite's usage flags in a u-blox UBX log.

This tool focuses on the two messages that matter most for "is the receiver
actually using this satellite?":

- UBX-NAV-SIG: per-signal usage (`prUsed`, `crUsed`, `doUsed`)
- UBX-NAV-SAT: per-satellite usage (`svUsed`) and pseudorange residual (`prRes`)

It also reports RXM-RAWX pseudorange samples for the same satellite so it is
easy to distinguish "not tracked" from "tracked but rejected".

Examples:
  python3 tools/ubx_check_prused.py prn8.ubx 8
  python3 tools/ubx_check_prused.py prn8.ubx 8 --sig-id 0
  python3 tools/ubx_check_prused.py prn8.ubx 8 --show-epochs --limit 20
"""
from __future__ import annotations

import argparse
import struct
import sys
from collections import Counter
from pathlib import Path
from statistics import median

try:
    from pyubx2 import UBXReader
except ImportError:  # pragma: no cover - handled at runtime
    UBXReader = None

NAV_SAT_PRRES_CLAMP_M = 3276.8
PRRES_CLAMP_TOL_M = 0.11


def iter_ubx(data: bytes):
    index = 0
    end = len(data) - 5
    while index < end:
        if data[index] == 0xB5 and data[index + 1] == 0x62:
            msg_class = data[index + 2]
            msg_id = data[index + 3]
            length = struct.unpack_from("<H", data, index + 4)[0]
            frame_end = index + 6 + length + 2
            if frame_end <= len(data):
                yield msg_class, msg_id, data[index + 6 : index + 6 + length]
                index = frame_end
                continue
        index += 1


def parse_rawx(payload: bytes):
    if len(payload) < 16:
        return None
    rcv_tow, week, leap_s, num_meas, rec_stat, version = struct.unpack_from(
        "<dHbBBB", payload, 0
    )
    measurements = []
    for index in range(num_meas):
        offset = 16 + index * 32
        if offset + 32 > len(payload):
            break
        pr_mes, cp_mes, do_mes = struct.unpack_from("<ddf", payload, offset)
        gnss_id = payload[offset + 20]
        sv_id = payload[offset + 21]
        sig_id = payload[offset + 22]
        freq_id = payload[offset + 23]
        locktime, cno = struct.unpack_from("<HB", payload, offset + 24)
        trk_stat = payload[offset + 30]
        measurements.append(
            {
                "gnss_id": gnss_id,
                "sv_id": sv_id,
                "sig_id": sig_id,
                "freq_id": freq_id,
                "pr_mes": pr_mes,
                "cp_mes": cp_mes,
                "do_mes": do_mes,
                "locktime": locktime,
                "cno": cno,
                "trk_stat": trk_stat,
            }
        )
    return {
        "rcv_tow": rcv_tow,
        "week": week,
        "leap_s": leap_s,
        "rec_stat": rec_stat,
        "version": version,
        "measurements": measurements,
    }


def parse_nav_sat(payload: bytes):
    if len(payload) < 8:
        return None
    itow_ms = struct.unpack_from("<I", payload, 0)[0]
    num_svs = payload[5]
    sats = []
    for index in range(num_svs):
        offset = 8 + index * 12
        if offset + 12 > len(payload):
            break
        gnss_id, sv_id, cno, elev = struct.unpack_from("<BBBb", payload, offset)
        azim, pr_res = struct.unpack_from("<hh", payload, offset + 4)
        flags = struct.unpack_from("<I", payload, offset + 8)[0]
        sats.append(
            {
                "gnss_id": gnss_id,
                "sv_id": sv_id,
                "cno": cno,
                "elev": elev,
                "azim": azim,
                "pr_res": pr_res / 10.0,
                "quality_ind": flags & 0x07,
                "sv_used": bool(flags & 0x08),
                "health": (flags >> 4) & 0x03,
                "orbit_source": (flags >> 8) & 0x07,
                "eph_avail": bool((flags >> 11) & 0x01),
                "alm_avail": bool((flags >> 12) & 0x01),
                "ano_avail": bool((flags >> 13) & 0x01),
                "aop_avail": bool((flags >> 14) & 0x01),
                "smoothed": bool((flags >> 15) & 0x01),
                "pr_corr_used": bool((flags >> 16) & 0x01),
                "cr_corr_used": bool((flags >> 17) & 0x01),
                "do_corr_used": bool((flags >> 18) & 0x01),
            }
        )
    return {"itow_ms": itow_ms, "sats": sats}


def parse_nav_pvt(payload: bytes):
    if len(payload) < 92:
        return None
    itow_ms, year, month, day, hour, minute, second = struct.unpack_from(
        "<IHBBBBBx", payload, 0
    )
    fix_type = payload[20]
    num_sv = payload[23]
    lon, lat, height, _ = struct.unpack_from("<iiII", payload, 24)
    return {
        "itow_ms": itow_ms,
        "year": year,
        "month": month,
        "day": day,
        "hour": hour,
        "minute": minute,
        "second": second,
        "fix_type": fix_type,
        "num_sv": num_sv,
        "lat": lat / 1e7,
        "lon": lon / 1e7,
        "height_m": height / 1000.0,
    }


def _msg_attr(msg, parsed, name: str):
    if name in msg:
        return msg[name]
    return getattr(parsed, name, None)


def parse_nav_sig(path: Path, gnss_id: int, sv_id: int):
    if UBXReader is None:
        raise RuntimeError(
            "pyubx2 is required for NAV-SIG parsing. Install it with `pip install pyubx2`."
        )

    rows = []
    with path.open("rb") as stream:
        reader = UBXReader(stream, protfilter=2)
        for _, parsed in reader:
            if parsed is None or parsed.identity != "NAV-SIG":
                continue

            msg = parsed.__dict__ if not hasattr(parsed, "val") else parsed.val
            itow_ms = _msg_attr(msg, parsed, "iTOW")
            num_sigs = _msg_attr(msg, parsed, "numSigs")
            if itow_ms is None or num_sigs is None:
                continue

            for index in range(1, int(num_sigs) + 1):
                suffix = f"_{index:02d}"
                row_gnss_id = _msg_attr(msg, parsed, f"gnssId{suffix}")
                row_sv_id = _msg_attr(msg, parsed, f"svId{suffix}")
                if row_gnss_id != gnss_id or row_sv_id != sv_id:
                    continue

                rows.append(
                    {
                        "itow_ms": int(itow_ms),
                        "sig_id": _msg_attr(msg, parsed, f"sigId{suffix}"),
                        "freq_id": _msg_attr(msg, parsed, f"freqId{suffix}"),
                        "pr_res": _msg_attr(msg, parsed, f"prRes{suffix}"),
                        "cno": _msg_attr(msg, parsed, f"cno{suffix}"),
                        "quality_ind": _msg_attr(msg, parsed, f"qualityInd{suffix}"),
                        "health": _msg_attr(msg, parsed, f"health{suffix}"),
                        "pr_smoothed": _msg_attr(msg, parsed, f"prSmoothed{suffix}"),
                        "pr_used": bool(_msg_attr(msg, parsed, f"prUsed{suffix}")),
                        "cr_used": bool(_msg_attr(msg, parsed, f"crUsed{suffix}")),
                        "do_used": bool(_msg_attr(msg, parsed, f"doUsed{suffix}")),
                        "pr_corr_used": bool(
                            _msg_attr(msg, parsed, f"prCorrUsed{suffix}")
                        ),
                        "cr_corr_used": bool(
                            _msg_attr(msg, parsed, f"crCorrUsed{suffix}")
                        ),
                        "do_corr_used": bool(
                            _msg_attr(msg, parsed, f"doCorrUsed{suffix}")
                        ),
                    }
                )
    return rows


def parse_binary_messages(path: Path, gnss_id: int, sv_id: int, rawx_sig_id: int):
    rawx_rows = []
    nav_sat_rows = []
    pvt_by_itow = {}

    data = path.read_bytes()
    for msg_class, msg_id, payload in iter_ubx(data):
        if (msg_class, msg_id) == (0x02, 0x15):
            epoch = parse_rawx(payload)
            if epoch is None:
                continue
            for measurement in epoch["measurements"]:
                if (
                    measurement["gnss_id"] == gnss_id
                    and measurement["sv_id"] == sv_id
                    and measurement["sig_id"] == rawx_sig_id
                ):
                    rawx_rows.append(
                        {
                            "rcv_tow": epoch["rcv_tow"],
                            "week": epoch["week"],
                            **measurement,
                        }
                    )
        elif (msg_class, msg_id) == (0x01, 0x35):
            epoch = parse_nav_sat(payload)
            if epoch is None:
                continue
            for sat in epoch["sats"]:
                if sat["gnss_id"] == gnss_id and sat["sv_id"] == sv_id:
                    nav_sat_rows.append({"itow_ms": epoch["itow_ms"], **sat})
        elif (msg_class, msg_id) == (0x01, 0x07):
            epoch = parse_nav_pvt(payload)
            if epoch is not None:
                pvt_by_itow[epoch["itow_ms"]] = epoch

    return rawx_rows, nav_sat_rows, pvt_by_itow


def parse_sat_label(text: str) -> tuple[int, int]:
    value = text.strip().upper()
    if not value:
        raise argparse.ArgumentTypeError("Satellite must not be empty.")

    if value.isdigit():
        prn = int(value)
        if prn < 1 or prn > 255:
            raise argparse.ArgumentTypeError(f"Satellite id out of range: {prn}")
        return 0, prn

    prefixes = {
        "G": 0,
        "S": 1,
        "E": 2,
        "C": 3,
        "I": 4,
        "J": 5,
        "R": 6,
    }
    prefix = value[0]
    if prefix not in prefixes or not value[1:].isdigit():
        raise argparse.ArgumentTypeError(
            "Satellite must be either a PRN like `8` or a label like `G08`."
        )
    return prefixes[prefix], int(value[1:])


def gnss_name(gnss_id: int) -> str:
    return {
        0: "GPS",
        1: "SBAS",
        2: "Galileo",
        3: "BeiDou",
        4: "IMES",
        5: "QZSS",
        6: "GLONASS",
    }.get(gnss_id, f"GNSS{gnss_id}")


def sat_label(gnss_id: int, sv_id: int) -> str:
    return f"{gnss_name(gnss_id)}{sv_id:02d}"


def median_or_none(values):
    if not values:
        return None
    return float(median(values))


def format_counter(counter: Counter) -> str:
    if not counter:
        return "none"
    return ", ".join(f"{key}={counter[key]}" for key in sorted(counter))


def is_prres_clamped(pr_res_m: float) -> bool:
    return abs(abs(pr_res_m) - NAV_SAT_PRRES_CLAMP_M) <= PRRES_CLAMP_TOL_M


def utc_label(pvt_by_itow: dict[int, dict], itow_ms: int) -> str:
    pvt = pvt_by_itow.get(itow_ms)
    if pvt is None:
        return f"iTOW={itow_ms}"
    return (
        f"{pvt['year']:04d}-{pvt['month']:02d}-{pvt['day']:02d} "
        f"{pvt['hour']:02d}:{pvt['minute']:02d}:{pvt['second']:02d}"
    )


def summarize_nav_sig(rows):
    epochs = sorted({row["itow_ms"] for row in rows})
    pr_used_epochs = {row["itow_ms"] for row in rows if row["pr_used"]}
    cr_used_epochs = {row["itow_ms"] for row in rows if row["cr_used"]}
    do_used_epochs = {row["itow_ms"] for row in rows if row["do_used"]}
    pr_res_values = [float(row["pr_res"]) for row in rows if row["pr_res"] is not None]
    cno_values = [float(row["cno"]) for row in rows if row["cno"] is not None]
    return {
        "rows": len(rows),
        "epochs": len(epochs),
        "sig_ids": Counter(row["sig_id"] for row in rows),
        "quality_ind": Counter(row["quality_ind"] for row in rows),
        "health": Counter(row["health"] for row in rows),
        "pr_used_rows": sum(row["pr_used"] for row in rows),
        "pr_used_epochs": len(pr_used_epochs),
        "cr_used_rows": sum(row["cr_used"] for row in rows),
        "cr_used_epochs": len(cr_used_epochs),
        "do_used_rows": sum(row["do_used"] for row in rows),
        "do_used_epochs": len(do_used_epochs),
        "median_pr_res_m": median_or_none(pr_res_values),
        "min_pr_res_m": None if not pr_res_values else min(pr_res_values),
        "max_pr_res_m": None if not pr_res_values else max(pr_res_values),
        "median_cno": median_or_none(cno_values),
        "min_cno": None if not cno_values else min(cno_values),
        "max_cno": None if not cno_values else max(cno_values),
    }


def summarize_nav_sat(rows):
    epochs = sorted({row["itow_ms"] for row in rows})
    sv_used_epochs = {row["itow_ms"] for row in rows if row["sv_used"]}
    pr_res_values = [float(row["pr_res"]) for row in rows if row["pr_res"] is not None]
    cno_values = [float(row["cno"]) for row in rows if row["cno"] is not None]
    return {
        "rows": len(rows),
        "epochs": len(epochs),
        "quality_ind": Counter(row["quality_ind"] for row in rows),
        "health": Counter(row["health"] for row in rows),
        "orbit_source": Counter(row["orbit_source"] for row in rows),
        "eph_avail_rows": sum(row["eph_avail"] for row in rows),
        "sv_used_rows": sum(row["sv_used"] for row in rows),
        "sv_used_epochs": len(sv_used_epochs),
        "median_pr_res_m": median_or_none(pr_res_values),
        "min_pr_res_m": None if not pr_res_values else min(pr_res_values),
        "max_pr_res_m": None if not pr_res_values else max(pr_res_values),
        "clamped_rows": sum(is_prres_clamped(value) for value in pr_res_values),
        "median_cno": median_or_none(cno_values),
        "min_cno": None if not cno_values else min(cno_values),
        "max_cno": None if not cno_values else max(cno_values),
    }


def summarize_rawx(rows):
    pr_values = [row["pr_mes"] for row in rows]
    cno_values = [row["cno"] for row in rows]
    do_values = [row["do_mes"] for row in rows]
    lock_values = [row["locktime"] for row in rows]
    return {
        "rows": len(rows),
        "week_counts": Counter(row["week"] for row in rows),
        "trk_stat": Counter(row["trk_stat"] for row in rows),
        "first_tow": None if not rows else rows[0]["rcv_tow"],
        "last_tow": None if not rows else rows[-1]["rcv_tow"],
        "median_pr_m": median_or_none(pr_values),
        "min_pr_m": None if not pr_values else min(pr_values),
        "max_pr_m": None if not pr_values else max(pr_values),
        "median_cno": median_or_none(cno_values),
        "min_cno": None if not cno_values else min(cno_values),
        "max_cno": None if not cno_values else max(cno_values),
        "median_do_hz": median_or_none(do_values),
        "min_do_hz": None if not do_values else min(do_values),
        "max_do_hz": None if not do_values else max(do_values),
        "median_lock_ms": median_or_none(lock_values),
        "max_lock_ms": None if not lock_values else max(lock_values),
    }


def print_range(label: str, median_value, min_value, max_value, unit: str):
    if median_value is None:
        print(f"  {label}: none")
        return
    print(
        f"  {label}: median={median_value:.1f} {unit}"
        f", min={min_value:.1f} {unit}, max={max_value:.1f} {unit}"
    )


def print_report(
    ubx_file: Path,
    gnss_id: int,
    sv_id: int,
    nav_sig_rows,
    nav_sat_rows,
    rawx_rows,
    pvt_by_itow,
    nav_sig_filter_sig_id: int | None,
    rawx_sig_id: int,
):
    print(f"UBX file: {ubx_file}")
    print(f"Satellite: {sat_label(gnss_id, sv_id)}")
    if nav_sig_filter_sig_id is None:
        print("NAV-SIG filter: all signals")
    else:
        print(f"NAV-SIG filter: sigId={nav_sig_filter_sig_id}")
    print(f"RAWX filter: sigId={rawx_sig_id}")
    print()

    print("NAV-SIG")
    if not nav_sig_rows:
        print("  no rows for this satellite")
    else:
        stats = summarize_nav_sig(nav_sig_rows)
        print(f"  rows={stats['rows']} epochs={stats['epochs']}")
        print(f"  sigId counts: {format_counter(stats['sig_ids'])}")
        print(
            f"  prUsed: rows={stats['pr_used_rows']} epochs={stats['pr_used_epochs']}"
        )
        print(
            f"  crUsed: rows={stats['cr_used_rows']} epochs={stats['cr_used_epochs']}"
        )
        print(
            f"  doUsed: rows={stats['do_used_rows']} epochs={stats['do_used_epochs']}"
        )
        print_range(
            "prRes",
            stats["median_pr_res_m"],
            stats["min_pr_res_m"],
            stats["max_pr_res_m"],
            "m",
        )
        print_range(
            "C/N0",
            stats["median_cno"],
            stats["min_cno"],
            stats["max_cno"],
            "dB-Hz",
        )
        print(f"  qualityInd counts: {format_counter(stats['quality_ind'])}")
        print(f"  health counts: {format_counter(stats['health'])}")
    print()

    print("NAV-SAT")
    if not nav_sat_rows:
        print("  no rows for this satellite")
    else:
        stats = summarize_nav_sat(nav_sat_rows)
        print(f"  rows={stats['rows']} epochs={stats['epochs']}")
        print(
            f"  svUsed: rows={stats['sv_used_rows']} epochs={stats['sv_used_epochs']}"
        )
        print(
            f"  ephAvail rows: {stats['eph_avail_rows']}/{stats['rows']}"
        )
        print_range(
            "prRes",
            stats["median_pr_res_m"],
            stats["min_pr_res_m"],
            stats["max_pr_res_m"],
            "m",
        )
        print(f"  clamped prRes rows: {stats['clamped_rows']}")
        print_range(
            "C/N0",
            stats["median_cno"],
            stats["min_cno"],
            stats["max_cno"],
            "dB-Hz",
        )
        print(f"  qualityInd counts: {format_counter(stats['quality_ind'])}")
        print(f"  orbitSource counts: {format_counter(stats['orbit_source'])}")
        print(f"  health counts: {format_counter(stats['health'])}")
    print()

    print("RXM-RAWX")
    if not rawx_rows:
        print("  no rows for this satellite/signal")
    else:
        stats = summarize_rawx(rawx_rows)
        print(f"  rows={stats['rows']}")
        print(
            f"  rcvTow window: {stats['first_tow']:.3f} -> {stats['last_tow']:.3f} s"
        )
        print(f"  GPS week counts: {format_counter(stats['week_counts'])}")
        print_range(
            "pseudorange",
            stats["median_pr_m"],
            stats["min_pr_m"],
            stats["max_pr_m"],
            "m",
        )
        print_range(
            "C/N0",
            stats["median_cno"],
            stats["min_cno"],
            stats["max_cno"],
            "dB-Hz",
        )
        print_range(
            "doppler",
            stats["median_do_hz"],
            stats["min_do_hz"],
            stats["max_do_hz"],
            "Hz",
        )
        print(
            f"  locktime: median={stats['median_lock_ms']:.1f} ms"
            f", max={stats['max_lock_ms']} ms"
        )
        print(f"  trkStat counts: {format_counter(stats['trk_stat'])}")

    if nav_sig_rows or nav_sat_rows:
        print()
        if any(row["pr_used"] for row in nav_sig_rows):
            print("Result: this signal is used in the navigation solution.")
        elif any(row["sv_used"] for row in nav_sat_rows):
            print("Result: the satellite is used at NAV-SAT level but this NAV-SIG filter is not prUsed.")
        elif rawx_rows:
            print("Result: the satellite is tracked in RAWX but not used in the navigation solution.")
        else:
            print("Result: the satellite is not visible in RAWX and not used in the navigation solution.")


def print_epoch_rows(nav_sig_rows, nav_sat_rows, pvt_by_itow, limit: int):
    print()
    print("Epoch Rows")
    print(
        "  utc                       iTOW_ms    type      sigId  cno   prRes(m)   used  quality"
    )

    rows = []
    for row in nav_sig_rows:
        rows.append(
            (
                row["itow_ms"],
                (
                    f"  {utc_label(pvt_by_itow, row['itow_ms']):23s}"
                    f" {row['itow_ms']:9d}  NAV-SIG"
                    f"  {str(row['sig_id']):>5s}"
                    f" {str(row['cno']):>4s}"
                    f" {float(row['pr_res']):9.1f}"
                    f"  {'Y' if row['pr_used'] else 'N':>4s}"
                    f" {str(row['quality_ind']):>7s}"
                )
            )
        )
    for row in nav_sat_rows:
        rows.append(
            (
                row["itow_ms"],
                (
                    f"  {utc_label(pvt_by_itow, row['itow_ms']):23s}"
                    f" {row['itow_ms']:9d}  NAV-SAT"
                    f"  {'-':>5s}"
                    f" {str(row['cno']):>4s}"
                    f" {float(row['pr_res']):9.1f}"
                    f"  {'Y' if row['sv_used'] else 'N':>4s}"
                    f" {str(row['quality_ind']):>7s}"
                )
            )
        )

    rows.sort(key=lambda item: (item[0], item[1]))
    if limit > 0:
        rows = rows[:limit]
    for _, line in rows:
        print(line)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check NAV-SIG prUsed for one satellite in a UBX log."
    )
    parser.add_argument("ubx_file", type=Path, help="Path to the .ubx log file")
    parser.add_argument(
        "satellite",
        type=parse_sat_label,
        help="Target satellite, e.g. `8` or `G08`",
    )
    parser.add_argument(
        "--sig-id",
        type=int,
        default=None,
        help="Optional NAV-SIG signal filter. Example: `0` for GPS L1 C/A.",
    )
    parser.add_argument(
        "--rawx-sig-id",
        type=int,
        default=0,
        help="RXM-RAWX signal id to inspect. Default: 0 (GPS L1 C/A).",
    )
    parser.add_argument(
        "--show-epochs",
        action="store_true",
        help="Print per-epoch NAV-SIG and NAV-SAT rows after the summary.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=50,
        help="Maximum number of per-epoch rows to print with --show-epochs. Default: 50.",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    ubx_file: Path = args.ubx_file
    if not ubx_file.is_file():
        parser.error(f"UBX file does not exist: {ubx_file}")

    gnss_id, sv_id = args.satellite
    rawx_rows, nav_sat_rows, pvt_by_itow = parse_binary_messages(
        ubx_file, gnss_id, sv_id, args.rawx_sig_id
    )
    nav_sig_rows = parse_nav_sig(ubx_file, gnss_id, sv_id)
    if args.sig_id is not None:
        nav_sig_rows = [row for row in nav_sig_rows if row["sig_id"] == args.sig_id]

    print_report(
        ubx_file=ubx_file,
        gnss_id=gnss_id,
        sv_id=sv_id,
        nav_sig_rows=nav_sig_rows,
        nav_sat_rows=nav_sat_rows,
        rawx_rows=rawx_rows,
        pvt_by_itow=pvt_by_itow,
        nav_sig_filter_sig_id=args.sig_id,
        rawx_sig_id=args.rawx_sig_id,
    )

    if args.show_epochs:
        print_epoch_rows(nav_sig_rows, nav_sat_rows, pvt_by_itow, args.limit)

    return 0


if __name__ == "__main__":
    sys.exit(main())
