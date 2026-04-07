#!/usr/bin/env python3
"""Read a ZED-F9P UBX log and emit best-available GPS tx/rx tuples.

Input:  one .ubx binary log file containing RXM-RAWX, NAV-TIMEGPS, NAV-PVT, and RXM-SFRBX.
Output: CSV rows with sv_id,sig_id,tx_week,tx_tow,rx_week,rx_tow.
"""
from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

C_LIGHT = 299792458.0
SECONDS_PER_WEEK = 604800.0
MU_GPS = 3.986005e14
OMEGA_E_DOT = 7.2921151467e-5
F_REL = -4.442807633e-10
TIME_MATCH_TOLERANCE_SECONDS = 0.010


@dataclass
class GpsClockTerms:
    toc: float
    af0: float
    af1: float
    af2: float
    tgd: float
    sqrt_a: float = 5153.7954905
    e: float = 0.0
    m0: float = 0.0
    delta_n: float = 0.0


@dataclass
class TupleRow:
    sv_id: int
    sig_id: int
    tx_week: int
    tx_tow: float
    rx_week: int
    rx_tow: float


def iter_ubx(data: bytes):
    index = 0
    end = len(data) - 5
    while index < end:
        if data[index] == 0xB5 and data[index + 1] == 0x62:
            msg_class = data[index + 2]
            msg_id = data[index + 3]
            length = struct.unpack_from('<H', data, index + 4)[0]
            frame_end = index + 6 + length + 2
            if frame_end <= len(data):
                yield msg_class, msg_id, data[index + 6:index + 6 + length]
                index = frame_end
                continue
        index += 1


def wrap_week_tow(week: int, tow: float) -> tuple[int, float]:
    while tow < 0.0:
        week -= 1
        tow += SECONDS_PER_WEEK
    while tow >= SECONDS_PER_WEEK:
        week += 1
        tow -= SECONDS_PER_WEEK
    return week, tow


def parse_rawx(payload: bytes):
    if len(payload) < 16:
        return None
    rcv_tow, week, leap_s, num_meas, rec_stat, version = struct.unpack_from('<dHbBBB', payload, 0)
    measurements = []
    for index in range(num_meas):
        offset = 16 + index * 32
        if offset + 32 > len(payload):
            break
        pr_mes, cp_mes, do_mes = struct.unpack_from('<ddf', payload, offset)
        gnss_id = payload[offset + 20]
        sv_id = payload[offset + 21]
        sig_id = payload[offset + 22]
        trk_stat = payload[offset + 30]
        measurements.append(
            {
                'pr_mes': pr_mes,
                'cp_mes': cp_mes,
                'do_mes': do_mes,
                'gnss_id': gnss_id,
                'sv_id': sv_id,
                'sig_id': sig_id,
                'trk_stat': trk_stat,
            }
        )
    return {
        'rcv_tow': rcv_tow,
        'week': week,
        'leap_s': leap_s,
        'rec_stat': rec_stat,
        'measurements': measurements,
    }


def parse_timegps(payload: bytes):
    if len(payload) != 16:
        return None
    itow_ms, ftow_ns, week, leap_s, valid, t_acc = struct.unpack('<IiHbBI', payload)
    return {
        'itow_ms': itow_ms,
        'ftow_ns': ftow_ns,
        'week': week,
        'leap_s': leap_s,
        'valid': valid,
        't_acc': t_acc,
    }


def parse_pvt(payload: bytes):
    if len(payload) < 92:
        return None
    return {
        'itow_ms': struct.unpack_from('<I', payload, 0)[0],
        'valid_flags': payload[11],
        'flags2': payload[22],
        'fix_type': payload[20],
    }


def timegps_is_valid(timegps: dict | None) -> bool:
    if timegps is None:
        return False
    return bool(timegps['valid'] & 0x01) and bool(timegps['valid'] & 0x02)


def pvt_time_is_valid(pvt: dict | None) -> bool:
    if pvt is None:
        return False
    valid_time = bool(pvt['valid_flags'] & 0x02)
    confirmed_time = bool(pvt['flags2'] & 0x80)
    return valid_time and confirmed_time and pvt['fix_type'] >= 3


def solve_kepler(mean_anomaly: float, eccentricity: float) -> float:
    eccentric_anomaly = mean_anomaly
    for _ in range(8):
        numerator = eccentric_anomaly - eccentricity * math.sin(eccentric_anomaly) - mean_anomaly
        denominator = 1.0 - eccentricity * math.cos(eccentric_anomaly)
        eccentric_anomaly -= numerator / denominator
    return eccentric_anomaly


def satellite_clock_bias(tx_tow: float, clock_terms: GpsClockTerms) -> float:
    dt = tx_tow - clock_terms.toc
    if dt > 302400.0:
        dt -= SECONDS_PER_WEEK
    elif dt < -302400.0:
        dt += SECONDS_PER_WEEK
    semi_major_axis = clock_terms.sqrt_a ** 2
    mean_motion = math.sqrt(MU_GPS / (semi_major_axis ** 3)) + clock_terms.delta_n
    mean_anomaly = clock_terms.m0 + mean_motion * dt
    eccentric_anomaly = solve_kepler(mean_anomaly, clock_terms.e)
    relativistic = F_REL * clock_terms.e * clock_terms.sqrt_a * math.sin(eccentric_anomaly)
    return clock_terms.af0 + clock_terms.af1 * dt + clock_terms.af2 * dt * dt + relativistic - clock_terms.tgd


def corrected_transmit_time(*, rx_week: int, rx_tow: float, pseudorange_m: float, clock_terms: GpsClockTerms) -> tuple[int, float]:
    tx_week, tx_tow = wrap_week_tow(rx_week, rx_tow - pseudorange_m / C_LIGHT)
    for _ in range(3):
        bias = satellite_clock_bias(tx_tow, clock_terms)
        tx_week, tx_tow = wrap_week_tow(rx_week, rx_tow - pseudorange_m / C_LIGHT - bias)
    return tx_week, tx_tow


@dataclass
class GpsEphemeris:
    sv_id: int
    clock_terms: GpsClockTerms


def parse_sfrbx(payload: bytes):
    if len(payload) < 8:
        return None
    gnss_id, sv_id, _, freq_id, num_words, chn, version, _ = struct.unpack_from('<BBBBBBBB', payload, 0)
    if len(payload) < 8 + num_words * 4:
        return None
    words = list(struct.unpack_from(f'<{num_words}I', payload, 8))
    return {
        'gnss_id': gnss_id,
        'sv_id': sv_id,
        'freq_id': freq_id,
        'num_words': num_words,
        'chn': chn,
        'version': version,
        'words': words,
    }


def get_bits(word_list: list, start: int, width: int) -> int:
    value = 0
    for bit_index in range(width):
        absolute = start + bit_index
        word = absolute // 30
        bit = 29 - (absolute % 30)
        value = (value << 1) | ((word_list[word] >> bit) & 1)
    return value


def signed(value: int, width: int) -> int:
    sign_bit = 1 << (width - 1)
    return (value ^ sign_bit) - sign_bit


def decode_subframe_1_clock_terms(words: list, sv_id: int) -> GpsEphemeris:
    toc = get_bits(words, 218, 16) * 16.0
    af2 = signed(get_bits(words, 240, 8), 8) * 2.0 ** -55
    af1 = signed(get_bits(words, 248, 16), 16) * 2.0 ** -43
    af0 = signed(get_bits(words, 270, 22), 22) * 2.0 ** -31
    tgd = signed(get_bits(words, 196, 8), 8) * 2.0 ** -31
    return GpsEphemeris(
        sv_id=sv_id,
        clock_terms=GpsClockTerms(toc=toc, af0=af0, af1=af1, af2=af2, tgd=tgd),
    )


def timegps_tow_seconds(timegps: dict | None) -> float | None:
    if timegps is None:
        return None
    return timegps['itow_ms'] * 1e-3 + timegps['ftow_ns'] * 1e-9


def find_matching_timegps(epoch: dict, timegps_by_week: dict[int, list[dict]]) -> dict | None:
    candidates = timegps_by_week.get(epoch['week'])
    if not candidates:
        return None
    best = min(
        candidates,
        key=lambda candidate: abs(timegps_tow_seconds(candidate) - epoch['rcv_tow']),
    )
    if abs(timegps_tow_seconds(best) - epoch['rcv_tow']) > TIME_MATCH_TOLERANCE_SECONDS:
        return None
    return best


def extract_rows(path: Path) -> list[TupleRow]:
    data = path.read_bytes()
    rows: list[TupleRow] = []
    timegps_by_week: dict[int, list[dict]] = {}
    pvt_by_itow_ms: dict[int, dict] = {}
    ephemeris_by_sv: dict[int, GpsEphemeris] = {}
    rawx_epochs: list[dict] = []

    for msg_class, msg_id, payload in iter_ubx(data):
        if (msg_class, msg_id) == (0x01, 0x20):
            timegps = parse_timegps(payload)
            if timegps is not None:
                timegps_by_week.setdefault(timegps['week'], []).append(timegps)
            continue
        if (msg_class, msg_id) == (0x01, 0x07):
            pvt = parse_pvt(payload)
            if pvt is not None:
                pvt_by_itow_ms[pvt['itow_ms']] = pvt
            continue
        if (msg_class, msg_id) == (0x02, 0x13):
            sfrbx = parse_sfrbx(payload)
            if sfrbx and sfrbx['gnss_id'] == 0 and sfrbx['num_words'] >= 10:
                ephemeris_by_sv[sfrbx['sv_id']] = decode_subframe_1_clock_terms(sfrbx['words'], sfrbx['sv_id'])
            continue
        if (msg_class, msg_id) == (0x02, 0x15):
            epoch = parse_rawx(payload)
            if epoch is not None:
                rawx_epochs.append(epoch)

    for epoch in rawx_epochs:
        if epoch['rec_stat'] & 0x02:
            continue
        timegps = find_matching_timegps(epoch, timegps_by_week)
        if not timegps_is_valid(timegps):
            continue
        pvt = pvt_by_itow_ms.get(timegps['itow_ms'])
        if not pvt_time_is_valid(pvt):
            continue

        for measurement in epoch['measurements']:
            if measurement['gnss_id'] != 0 or not (measurement['trk_stat'] & 0x01):
                continue
            ephemeris = ephemeris_by_sv.get(measurement['sv_id'])
            if ephemeris is None:
                continue
            tx_week, tx_tow = corrected_transmit_time(
                rx_week=epoch['week'],
                rx_tow=epoch['rcv_tow'],
                pseudorange_m=measurement['pr_mes'],
                clock_terms=ephemeris.clock_terms,
            )
            rows.append(TupleRow(measurement['sv_id'], measurement['sig_id'], tx_week, tx_tow, epoch['week'], epoch['rcv_tow']))

    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('ubx_file', type=Path)
    args = parser.parse_args()

    writer = csv.DictWriter(
        sys.stdout,
        fieldnames=['sv_id', 'sig_id', 'tx_week', 'tx_tow', 'rx_week', 'rx_tow'],
    )
    writer.writeheader()
    rows = sorted(
        extract_rows(args.ubx_file),
        key=lambda row: (row.rx_week, row.rx_tow, row.sv_id, row.sig_id),
    )
    for row in rows:
        writer.writerow(
            {
                'sv_id': row.sv_id,
                'sig_id': row.sig_id,
                'tx_week': row.tx_week,
                'tx_tow': f'{row.tx_tow:.9f}',
                'rx_week': row.rx_week,
                'rx_tow': f'{row.rx_tow:.9f}',
            }
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
