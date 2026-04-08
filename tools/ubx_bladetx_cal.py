#!/usr/bin/env python3
"""Estimate bladeTX timing calibration from u-blox UBX logs.

This tool is built for mixed real-sky plus injected-satellite validation runs.
It looks for a common pseudorange residual between injected PRNs and the real
PRNs that the receiver actually used in the navigation solution, then converts
that residual into a recommended `--trimble-tx-cal-ns` adjustment.

The strongest path uses `UBX-NAV-SAT`:
- real anchor group: GPS PRNs with `svUsed=Y` and not in `--inject`
- injected group: GPS PRNs with `svUsed=Y` and in `--inject`
- common residual per epoch: median(real prRes) - median(injected prRes)
- recommended delta: `--trimble-tx-cal-ns += -residual_sec * 1e9`

If no injected PRN is ever used in navigation, the tool still reports tracking
and per-PRN NAV-SAT status, but it will not emit a high-confidence calibration.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import median

C_LIGHT = 299792458.0
NAV_SAT_PRRES_CLAMP_M = 3276.8
PRRES_CLAMP_TOL_M = 0.11


@dataclass
class RawxMeasurement:
    sv_id: int
    gnss_id: int
    sig_id: int
    pr_mes: float
    cno: int


@dataclass
class RawxEpoch:
    rcv_tow: float
    week: int
    measurements: list[RawxMeasurement]


@dataclass
class NavSatMeasurement:
    sv_id: int
    gnss_id: int
    cno: int
    elev: int
    azim: int
    pr_res: float
    quality_ind: int
    sv_used: bool


@dataclass
class NavSatEpoch:
    itow_sec: float
    sats: list[NavSatMeasurement]


@dataclass
class NavStatusEpoch:
    itow_sec: float
    spoof_det_state: int


@dataclass
class NavPvtEpoch:
    itow_sec: float
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int
    fix_type: int
    num_sv: int
    lat: float
    lon: float
    height_m: float


@dataclass
class ResidualEpoch:
    itow_sec: float
    utc_label: str
    synth_prns: tuple[int, ...]
    real_prns: tuple[int, ...]
    synth_pr_res_m: float
    real_pr_res_m: float
    real_minus_injected_m: float
    recommended_delta_cal_ns: int


@dataclass
class CaptureEstimate:
    path: Path
    rawx_epochs: int
    nav_sat_epochs: int
    nav_status_epochs: int
    nav_pvt_epochs: int
    residual_epochs: list[ResidualEpoch]
    tracked_prns: dict[int, dict[str, float | int]]
    nav_sat_prns: dict[int, dict[str, object]]
    spoof_counts: dict[int, int]
    fix_counts: dict[int, int]
    rawx_frac_counts: dict[float, int]

    @property
    def usable(self) -> bool:
        return bool(self.residual_epochs)


SPOOF_LABELS = {0: "unknown", 1: "no_spoof", 2: "spoof_indicated", 3: "multi_spoof"}
FIX_LABELS = {0: "no_fix", 1: "DR", 2: "2D", 3: "3D", 4: "3D+DR", 5: "time"}


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


def parse_rawx(payload: bytes) -> RawxEpoch | None:
    if len(payload) < 16:
        return None
    rcv_tow, week, _, num_meas, _, _ = struct.unpack_from("<dHbBBB", payload, 0)
    measurements: list[RawxMeasurement] = []
    for index in range(num_meas):
        offset = 16 + index * 32
        if offset + 32 > len(payload):
            break
        pr_mes = struct.unpack_from("<d", payload, offset)[0]
        gnss_id = payload[offset + 20]
        sv_id = payload[offset + 21]
        sig_id = payload[offset + 22]
        cno = payload[offset + 26]
        measurements.append(
            RawxMeasurement(
                sv_id=sv_id,
                gnss_id=gnss_id,
                sig_id=sig_id,
                pr_mes=pr_mes,
                cno=cno,
            )
        )
    return RawxEpoch(rcv_tow=rcv_tow, week=week, measurements=measurements)


def parse_nav_sat(payload: bytes) -> NavSatEpoch | None:
    if len(payload) < 8:
        return None
    itow_ms = struct.unpack_from("<I", payload, 0)[0]
    num_svs = payload[5]
    sats: list[NavSatMeasurement] = []
    for index in range(num_svs):
        offset = 8 + index * 12
        if offset + 12 > len(payload):
            break
        gnss_id, sv_id, cno, elev = struct.unpack_from("<BBBb", payload, offset)
        azim, pr_res_raw = struct.unpack_from("<hh", payload, offset + 4)
        flags = struct.unpack_from("<I", payload, offset + 8)[0]
        sats.append(
            NavSatMeasurement(
                sv_id=sv_id,
                gnss_id=gnss_id,
                cno=cno,
                elev=elev,
                azim=azim,
                pr_res=pr_res_raw / 10.0,
                quality_ind=flags & 0x07,
                sv_used=bool(flags & 0x08),
            )
        )
    return NavSatEpoch(itow_sec=itow_ms / 1000.0, sats=sats)


def parse_nav_status(payload: bytes) -> NavStatusEpoch | None:
    if len(payload) < 16:
        return None
    itow_ms, _, _, _, flags2 = struct.unpack_from("<IBBBB", payload, 0)
    return NavStatusEpoch(
        itow_sec=itow_ms / 1000.0,
        spoof_det_state=(flags2 >> 3) & 0x03,
    )


def parse_nav_pvt(payload: bytes) -> NavPvtEpoch | None:
    if len(payload) < 92:
        return None
    itow_ms, year, month, day, hour, minute, second = struct.unpack_from(
        "<IHBBBBBx", payload, 0
    )
    fix_type = payload[20]
    num_sv = payload[23]
    lon, lat, height, _ = struct.unpack_from("<iiII", payload, 24)
    return NavPvtEpoch(
        itow_sec=itow_ms / 1000.0,
        year=year,
        month=month,
        day=day,
        hour=hour,
        minute=minute,
        second=second,
        fix_type=fix_type,
        num_sv=num_sv,
        lat=lat / 1e7,
        lon=lon / 1e7,
        height_m=height / 1000.0,
    )


def parse_log(path: Path):
    data = path.read_bytes()
    rawx: list[RawxEpoch] = []
    nav_sat: list[NavSatEpoch] = []
    nav_status: list[NavStatusEpoch] = []
    nav_pvt: list[NavPvtEpoch] = []

    for msg_class, msg_id, payload in iter_ubx(data):
        if (msg_class, msg_id) == (0x02, 0x15):
            epoch = parse_rawx(payload)
            if epoch is not None:
                rawx.append(epoch)
        elif (msg_class, msg_id) == (0x01, 0x35):
            epoch = parse_nav_sat(payload)
            if epoch is not None:
                nav_sat.append(epoch)
        elif (msg_class, msg_id) == (0x01, 0x03):
            epoch = parse_nav_status(payload)
            if epoch is not None:
                nav_status.append(epoch)
        elif (msg_class, msg_id) == (0x01, 0x07):
            epoch = parse_nav_pvt(payload)
            if epoch is not None:
                nav_pvt.append(epoch)

    return rawx, nav_sat, nav_status, nav_pvt


def parse_prn_list(text: str) -> list[int]:
    prns: list[int] = []
    for chunk in text.split(","):
        value = chunk.strip()
        if not value:
            continue
        prn = int(value)
        if prn < 1 or prn > 32:
            raise argparse.ArgumentTypeError(f"GPS PRN must be 1..32, got {prn}")
        prns.append(prn)
    if not prns:
        raise argparse.ArgumentTypeError("At least one injected PRN is required")
    return sorted(set(prns))


def median_or_none(values: list[float]) -> float | None:
    return None if not values else float(median(values))


def utc_label(pvt_by_itow: dict[float, NavPvtEpoch], itow_sec: float) -> str:
    pvt = pvt_by_itow.get(itow_sec)
    if pvt is None:
        return f"iTOW={itow_sec:.3f}s"
    return (
        f"{pvt.year:04d}-{pvt.month:02d}-{pvt.day:02d} "
        f"{pvt.hour:02d}:{pvt.minute:02d}:{pvt.second:02d}"
    )


def is_prres_clamped(pr_res_m: float) -> bool:
    return abs(abs(pr_res_m) - NAV_SAT_PRRES_CLAMP_M) <= PRRES_CLAMP_TOL_M


def recommendation_from_real_minus_injected_m(real_minus_injected_m: float) -> int:
    residual_sec = real_minus_injected_m / C_LIGHT
    return int(round(-residual_sec * 1.0e9))


def classify_confidence(residual_epochs: list[ResidualEpoch], used_injected_prns: set[int]) -> str:
    count = len(residual_epochs)
    if count >= 50 and len(used_injected_prns) >= 2:
        return "high"
    if count >= 10 and len(used_injected_prns) >= 1:
        return "medium"
    if count >= 3:
        return "low"
    return "insufficient"


def analyze_capture(path: Path, inject_prns: set[int]) -> CaptureEstimate:
    rawx, nav_sat, nav_status, nav_pvt = parse_log(path)

    tracked_prns: dict[int, dict[str, float | int]] = defaultdict(
        lambda: {"epochs": 0, "first_tow": math.nan, "last_tow": math.nan, "cno": [], "pr": []}
    )
    rawx_frac_counts: Counter[float] = Counter()
    for epoch in rawx:
        rawx_frac_counts[round(epoch.rcv_tow - int(epoch.rcv_tow), 3)] += 1
        for meas in epoch.measurements:
            if meas.gnss_id != 0 or meas.sig_id != 0:
                continue
            stats = tracked_prns[meas.sv_id]
            stats["epochs"] = int(stats["epochs"]) + 1
            if math.isnan(float(stats["first_tow"])):
                stats["first_tow"] = epoch.rcv_tow
            stats["last_tow"] = epoch.rcv_tow
            stats["cno"].append(meas.cno)
            stats["pr"].append(meas.pr_mes)

    pvt_by_itow = {epoch.itow_sec: epoch for epoch in nav_pvt}
    residual_epochs: list[ResidualEpoch] = []
    nav_sat_prns: dict[int, dict[str, object]] = defaultdict(
        lambda: {
            "seen": 0,
            "used": 0,
            "cno": [],
            "pr_res": [],
            "used_pr_res": [],
            "quality_ind": Counter(),
        }
    )

    for epoch in nav_sat:
        gps_sats = [sat for sat in epoch.sats if sat.gnss_id == 0]
        if not gps_sats:
            continue

        for sat in gps_sats:
            stats = nav_sat_prns[sat.sv_id]
            stats["seen"] = int(stats["seen"]) + 1
            stats["cno"].append(sat.cno)
            stats["pr_res"].append(sat.pr_res)
            stats["quality_ind"][sat.quality_ind] += 1
            if sat.sv_used:
                stats["used"] = int(stats["used"]) + 1
                stats["used_pr_res"].append(sat.pr_res)

        synth_used = [sat for sat in gps_sats if sat.sv_used and sat.sv_id in inject_prns]
        real_used = [sat for sat in gps_sats if sat.sv_used and sat.sv_id not in inject_prns]
        if not synth_used or not real_used:
            continue

        synth_pr_res_m = float(median(sat.pr_res for sat in synth_used))
        real_pr_res_m = float(median(sat.pr_res for sat in real_used))
        real_minus_injected_m = real_pr_res_m - synth_pr_res_m
        residual_epochs.append(
            ResidualEpoch(
                itow_sec=epoch.itow_sec,
                utc_label=utc_label(pvt_by_itow, epoch.itow_sec),
                synth_prns=tuple(sorted(sat.sv_id for sat in synth_used)),
                real_prns=tuple(sorted(sat.sv_id for sat in real_used)),
                synth_pr_res_m=synth_pr_res_m,
                real_pr_res_m=real_pr_res_m,
                real_minus_injected_m=real_minus_injected_m,
                recommended_delta_cal_ns=recommendation_from_real_minus_injected_m(
                    real_minus_injected_m
                ),
            )
        )

    spoof_counts = Counter(epoch.spoof_det_state for epoch in nav_status)
    fix_counts = Counter(epoch.fix_type for epoch in nav_pvt)

    compact_tracked: dict[int, dict[str, float | int]] = {}
    for sv_id, stats in tracked_prns.items():
        compact_tracked[sv_id] = {
            "epochs": int(stats["epochs"]),
            "first_tow": float(stats["first_tow"]),
            "last_tow": float(stats["last_tow"]),
            "median_cno": median_or_none(stats["cno"]),
            "median_pr_m": median_or_none(stats["pr"]),
        }

    compact_nav_sat: dict[int, dict[str, object]] = {}
    for sv_id, stats in nav_sat_prns.items():
        compact_nav_sat[sv_id] = {
            "seen": int(stats["seen"]),
            "used": int(stats["used"]),
            "median_cno": median_or_none(stats["cno"]),
            "median_pr_res_m": median_or_none(stats["pr_res"]),
            "median_used_pr_res_m": median_or_none(stats["used_pr_res"]),
            "quality_ind": dict(stats["quality_ind"]),
        }

    return CaptureEstimate(
        path=path,
        rawx_epochs=len(rawx),
        nav_sat_epochs=len(nav_sat),
        nav_status_epochs=len(nav_status),
        nav_pvt_epochs=len(nav_pvt),
        residual_epochs=residual_epochs,
        tracked_prns=compact_tracked,
        nav_sat_prns=compact_nav_sat,
        spoof_counts=dict(spoof_counts),
        fix_counts=dict(fix_counts),
        rawx_frac_counts=dict(rawx_frac_counts),
    )


def summarize_used_prns(nav_sat_prns: dict[int, dict[str, object]], inject_prns: set[int]) -> tuple[set[int], set[int]]:
    used_injected = {
        sv_id
        for sv_id, stats in nav_sat_prns.items()
        if sv_id in inject_prns and int(stats["used"]) > 0
    }
    used_real = {
        sv_id
        for sv_id, stats in nav_sat_prns.items()
        if sv_id not in inject_prns and int(stats["used"]) > 0
    }
    return used_injected, used_real


def format_counter(counter_dict: dict[int | float, int], labels: dict[int, str] | None = None) -> str:
    parts = []
    for key in sorted(counter_dict):
        label = labels.get(int(key), str(key)) if labels else str(key)
        parts.append(f"{label}={counter_dict[key]}")
    return ", ".join(parts) if parts else "none"


def print_capture_report(
    estimate: CaptureEstimate,
    inject_prns: set[int],
    current_trimble_tx_cal_ns: int,
    trimble_tag_lead_ms: int | None,
    trimble_start_offset_sec: int | None,
) -> list[int]:
    print(f"Capture: {estimate.path}")
    print(
        "  epochs:"
        f" RAWX={estimate.rawx_epochs}"
        f" NAV-SAT={estimate.nav_sat_epochs}"
        f" NAV-STATUS={estimate.nav_status_epochs}"
        f" NAV-PVT={estimate.nav_pvt_epochs}"
    )
    print(f"  RAWX fractional TOW: {format_counter(estimate.rawx_frac_counts)}")
    print(f"  spoof states: {format_counter(estimate.spoof_counts, SPOOF_LABELS)}")
    print(f"  fix states: {format_counter(estimate.fix_counts, FIX_LABELS)}")

    print("  injected tracking:")
    for sv_id in sorted(inject_prns):
        tracked = estimate.tracked_prns.get(sv_id)
        nav_sat = estimate.nav_sat_prns.get(sv_id)
        if tracked is None and nav_sat is None:
            print(f"    PRN{sv_id:02d}: not present")
            continue

        tracked_bits = []
        if tracked is not None:
            tracked_bits.append(f"RAWX epochs={tracked['epochs']}")
            tracked_bits.append(
                f"tow={tracked['first_tow']:.3f}->{tracked['last_tow']:.3f}"
            )
            tracked_bits.append(f"median C/N0={tracked['median_cno']:.1f} dB-Hz")

        if nav_sat is not None:
            used = int(nav_sat["used"])
            seen = int(nav_sat["seen"])
            tracked_bits.append(f"NAV-SAT seen={seen} used={used}")
            med_used = nav_sat["median_used_pr_res_m"]
            med_seen = nav_sat["median_pr_res_m"]
            if med_used is not None:
                tracked_bits.append(f"median used prRes={med_used:+.1f} m")
            elif med_seen is not None:
                suffix = " (clamped)" if is_prres_clamped(float(med_seen)) else ""
                tracked_bits.append(f"median seen prRes={float(med_seen):+.1f} m{suffix}")

        print(f"    PRN{sv_id:02d}: " + ", ".join(tracked_bits))

    used_injected, used_real = summarize_used_prns(estimate.nav_sat_prns, inject_prns)
    if used_real:
        print(f"  real anchor PRNs used: {','.join(str(prn) for prn in sorted(used_real))}")
    elif estimate.nav_sat_epochs:
        print("  real anchor PRNs used: none")

    if not estimate.residual_epochs:
        print("  calibration: unavailable")
        if estimate.nav_sat_epochs == 0:
            print("    reason: no NAV-SAT in this log, so svUsed/prRes calibration is impossible")
        elif not used_injected:
            print("    reason: injected PRNs were tracked but never svUsed in navigation")
        else:
            print("    reason: no epoch had both injected and real PRNs svUsed together")
        print()
        return []

    residual_m = [epoch.real_minus_injected_m for epoch in estimate.residual_epochs]
    residual_ns = [epoch.real_minus_injected_m / C_LIGHT * 1.0e9 for epoch in estimate.residual_epochs]
    delta_ns = [epoch.recommended_delta_cal_ns for epoch in estimate.residual_epochs]
    best_delta_ns = int(round(median(delta_ns)))
    next_trimble_tx_cal_ns = current_trimble_tx_cal_ns + best_delta_ns
    confidence = classify_confidence(estimate.residual_epochs, used_injected)

    print("  calibration:")
    print(f"    usable epochs={len(estimate.residual_epochs)}")
    print(
        "    common residual (real minus injected):"
        f" median={median(residual_m):+.2f} m / {median(residual_ns):+.2f} ns"
    )
    if len(residual_ns) > 1:
        print(
            "    residual spread:"
            f" min={min(residual_ns):+.2f} ns"
            f" max={max(residual_ns):+.2f} ns"
        )
    print(f"    recommended delta --trimble-tx-cal-ns: {best_delta_ns:+d}")
    print(f"    recommended next --trimble-tx-cal-ns: {next_trimble_tx_cal_ns}")
    if trimble_tag_lead_ms is not None:
        print(f"    keep --trimble-tag-lead-ms: {trimble_tag_lead_ms}")
        equivalent_ms = best_delta_ns / 1.0e6
        print(
            "    equivalent lead adjustment:"
            f" {equivalent_ms:+.6f} ms"
            " (not the preferred knob)"
        )
    if trimble_start_offset_sec is not None:
        print(f"    keep --trimble-start-offset-sec: {trimble_start_offset_sec}")
    print(f"    confidence: {confidence}")

    first = estimate.residual_epochs[0]
    last = estimate.residual_epochs[-1]
    print(
        "    usable window:"
        f" {first.utc_label} -> {last.utc_label}"
        f" (iTOW {first.itow_sec:.3f}s -> {last.itow_sec:.3f}s)"
    )
    print()
    return delta_ns


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Estimate bladeTX timing calibration from u-blox UBX logs."
    )
    parser.add_argument("ubx_logs", nargs="+", help="Path(s) to .ubx log files")
    parser.add_argument(
        "--inject",
        required=True,
        type=parse_prn_list,
        help="Comma-separated injected GPS PRN list, for example 3,4,7,8",
    )
    parser.add_argument(
        "--current-trimble-tx-cal-ns",
        type=int,
        default=0,
        help="Current --trimble-tx-cal-ns value in your command (default: 0)",
    )
    parser.add_argument(
        "--trimble-tag-lead-ms",
        type=int,
        default=None,
        help="Current --trimble-tag-lead-ms value to echo in the recommendation",
    )
    parser.add_argument(
        "--trimble-start-offset-sec",
        type=int,
        default=None,
        help="Current --trimble-start-offset-sec value to echo in the recommendation",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    inject_prns = set(args.inject)
    all_delta_ns: list[int] = []

    for raw_path in args.ubx_logs:
        path = Path(raw_path)
        if not path.exists():
            parser.error(f"{path} does not exist")
        estimate = analyze_capture(path, inject_prns)
        all_delta_ns.extend(
            print_capture_report(
                estimate,
                inject_prns,
                current_trimble_tx_cal_ns=args.current_trimble_tx_cal_ns,
                trimble_tag_lead_ms=args.trimble_tag_lead_ms,
                trimble_start_offset_sec=args.trimble_start_offset_sec,
            )
        )

    if not all_delta_ns:
        print("Overall recommendation: no high-confidence timing calibration was derivable.")
        print("Need: UBX-NAV-SAT plus at least one injected PRN marked svUsed=Y.")
        return 2

    overall_delta_ns = int(round(median(all_delta_ns)))
    overall_next_tx_cal_ns = args.current_trimble_tx_cal_ns + overall_delta_ns
    print("Overall recommendation:")
    print(f"  best delta --trimble-tx-cal-ns: {overall_delta_ns:+d}")
    print(f"  best next --trimble-tx-cal-ns: {overall_next_tx_cal_ns}")
    if args.trimble_tag_lead_ms is not None:
        print(f"  keep --trimble-tag-lead-ms: {args.trimble_tag_lead_ms}")
    if args.trimble_start_offset_sec is not None:
        print(f"  keep --trimble-start-offset-sec: {args.trimble_start_offset_sec}")
    print("  sign convention: positive trimble-tx-cal-ns advances the injected epoch")
    return 0


if __name__ == "__main__":
    sys.exit(main())
