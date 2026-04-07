# ZED-F9P UBX Best Tuple Extractor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python CLI that reads a `.ubx` file from a u-blox ZED-F9P and emits the best-available per-satellite GPS tuple `{tx_week, tx_tow, rx_week, rx_tow}`.

**Architecture:** Reuse the proven low-level UBX frame walking and `RXM-RAWX` parsing approach already present in `docs/issues/parse_ubx.py`, but implement a clean standalone tool in `tools/ubx_best_tuple.py`. The tool will treat `RXM-RAWX` as the canonical observation epoch, validate receive time with `NAV-TIMEGPS`/`NAV-PVT`/`NAV-CLOCK`, and improve transmit time by decoding `RXM-SFRBX` GPS broadcast data and applying broadcast satellite clock correction.

**Tech Stack:** Python 3 standard library only (`argparse`, `csv`, `dataclasses`, `json`, `math`, `struct`, `subprocess`, `tempfile`, `unittest`).

---

## File Structure

- Create: `tools/ubx_best_tuple.py`
- Create: `tests/test_ubx_best_tuple.py`
- Create: `docs/ubx-best-tuple.md`
- Reference only: `docs/issues/parse_ubx.py`

### Responsibility Map

- `tools/ubx_best_tuple.py`
  - Own the full CLI.
  - Parse UBX frames from a `.ubx` file.
  - Parse `RXM-RAWX`, `NAV-TIMEGPS`, `NAV-CLOCK`, `NAV-PVT`, `NAV-EOE`, `TIM-TM2`, and `RXM-SFRBX`.
  - Build observation epochs keyed by `RXM-RAWX`.
  - Compute `rx_week` and `rx_tow` from the raw observation epoch, gated by timing-validity messages.
  - Compute `tx_week` and `tx_tow` first from `prMes / c`, then refine with decoded GPS broadcast clock terms.
  - Emit CSV to stdout with columns `sv_id,sig_id,tx_week,tx_tow,rx_week,rx_tow`.

- `tests/test_ubx_best_tuple.py`
  - Build tiny synthetic UBX logs in-memory and write them to temp files.
  - Verify CLI output for raw-only extraction, timing gating, week rollover, and broadcast-corrected transmit time.

- `docs/ubx-best-tuple.md`
  - Document the input/output contract, message requirements, limitations, and validation rules.

## Output Contract

The tool should emit one CSV row per accepted GPS observation:

```csv
sv_id,sig_id,tx_week,tx_tow,rx_week,rx_tow
7,0,2400,345600.129976033,2400,345600.200000000
8,0,2400,345600.131421882,2400,345600.200000000
```

Rules:

- Output GPS observations only: `gnssId == 0`.
- Use `RXM-RAWX.week` and `RXM-RAWX.rcvTow` as the observation receive tuple.
- Reject rows when timing validity is not established.
- Reject rows when no usable GPS broadcast clock terms are available.
- Normalize week rollover for both receive and transmit tuples.

## Task 1: Stand Up A RAWX-Only CLI

**Files:**
- Create: `tools/ubx_best_tuple.py`
- Create: `tests/test_ubx_best_tuple.py`
- Reference: `docs/issues/parse_ubx.py`

- [ ] **Step 1: Write the failing CLI smoke test**

```python
import csv
import io
import math
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

C_LIGHT = 299792458.0
REPO = Path(__file__).resolve().parents[1]


def ubx_frame(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    header = bytes([0xB5, 0x62, msg_class, msg_id]) + struct.pack('<H', len(payload))
    ck_a = 0
    ck_b = 0
    for byte in header[2:] + payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + payload + bytes([ck_a, ck_b])


def rawx_frame(*, week: int, rcv_tow: float, pr_mes: float, sv_id: int, sig_id: int = 0) -> bytes:
    payload = struct.pack('<dHbBBB', rcv_tow, week, 18, 1, 0, 1)
    payload += b'\x00\x00'
    payload += struct.pack('<ddfBBBBHBBBB', pr_mes, 0.0, 0.0, 0, sv_id, sig_id, 0, 1, 45, 0, 0, 1)
    return ubx_frame(0x02, 0x15, payload)


class RawOnlyCliTest(unittest.TestCase):
    def test_emits_one_gps_tuple_from_rawx_only_file(self) -> None:
        pr_mes = 20985472.123
        ubx_bytes = rawx_frame(week=2400, rcv_tow=345600.2, pr_mes=pr_mes, sv_id=7)
        with tempfile.TemporaryDirectory() as tmp_dir:
            ubx_path = Path(tmp_dir) / 'input.ubx'
            ubx_path.write_bytes(ubx_bytes)
            result = subprocess.run(
                [sys.executable, 'tools/ubx_best_tuple.py', str(ubx_path)],
                cwd=REPO,
                check=True,
                capture_output=True,
                text=True,
            )

        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['sv_id'], '7')
        self.assertEqual(rows[0]['sig_id'], '0')
        self.assertEqual(rows[0]['rx_week'], '2400')
        self.assertAlmostEqual(float(rows[0]['rx_tow']), 345600.2, places=9)
        self.assertAlmostEqual(
            float(rows[0]['tx_tow']),
            345600.2 - pr_mes / C_LIGHT,
            places=9,
        )


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run the smoke test to verify it fails**

Run: `python3 -m unittest tests.test_ubx_best_tuple.RawOnlyCliTest.test_emits_one_gps_tuple_from_rawx_only_file -v`

Expected: FAIL with `can't open file 'tools/ubx_best_tuple.py'`.

- [ ] **Step 3: Write the minimal RAWX-only implementation**

```python
#!/usr/bin/env python3
import argparse
import csv
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

C_LIGHT = 299792458.0
SECONDS_PER_WEEK = 604800.0


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


def extract_rows(path: Path) -> list[TupleRow]:
    rows: list[TupleRow] = []
    for msg_class, msg_id, payload in iter_ubx(path.read_bytes()):
        if (msg_class, msg_id) != (0x02, 0x15):
            continue
        epoch = parse_rawx(payload)
        if epoch is None:
            continue
        for measurement in epoch['measurements']:
            if measurement['gnss_id'] != 0:
                continue
            if not (measurement['trk_stat'] & 0x01):
                continue
            tx_week, tx_tow = wrap_week_tow(
                epoch['week'],
                epoch['rcv_tow'] - measurement['pr_mes'] / C_LIGHT,
            )
            rows.append(
                TupleRow(
                    sv_id=measurement['sv_id'],
                    sig_id=measurement['sig_id'],
                    tx_week=tx_week,
                    tx_tow=tx_tow,
                    rx_week=epoch['week'],
                    rx_tow=epoch['rcv_tow'],
                )
            )
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
    for row in extract_rows(args.ubx_file):
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
```

- [ ] **Step 4: Run the smoke test to verify it passes**

Run: `python3 -m unittest tests.test_ubx_best_tuple.RawOnlyCliTest.test_emits_one_gps_tuple_from_rawx_only_file -v`

Expected: PASS.

- [ ] **Step 5: Commit the RAWX-only baseline**

```bash
git add tools/ubx_best_tuple.py tests/test_ubx_best_tuple.py
git commit -m "feat: add rawx-only ubx best tuple cli"
```

## Task 2: Validate Receive Time With GPS Timing Messages

**Files:**
- Modify: `tools/ubx_best_tuple.py`
- Modify: `tests/test_ubx_best_tuple.py`

- [ ] **Step 1: Add a failing test for timing validity gating**

```python
def timegps_frame(*, week: int, itow_ms: int, ftow_ns: int, valid: int, t_acc: int = 25) -> bytes:
    payload = struct.pack('<IiHbBI', itow_ms, ftow_ns, week, 18, valid, t_acc)
    return ubx_frame(0x01, 0x20, payload)


def pvt_frame(*, itow_ms: int, valid_flags: int) -> bytes:
    payload = bytearray(92)
    struct.pack_into('<I', payload, 0, itow_ms)
    payload[11] = valid_flags
    payload[20] = 3
    return ubx_frame(0x01, 0x07, bytes(payload))


class TimingValidityTest(unittest.TestCase):
    def test_rejects_rawx_epoch_when_timegps_is_invalid(self) -> None:
        ubx_bytes = b''.join(
            [
                rawx_frame(week=2400, rcv_tow=345600.2, pr_mes=20985472.123, sv_id=7),
                timegps_frame(week=2400, itow_ms=345600200, ftow_ns=0, valid=0),
                pvt_frame(itow_ms=345600200, valid_flags=0),
            ]
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            ubx_path = Path(tmp_dir) / 'input.ubx'
            ubx_path.write_bytes(ubx_bytes)
            result = subprocess.run(
                [sys.executable, 'tools/ubx_best_tuple.py', str(ubx_path)],
                cwd=REPO,
                check=True,
                capture_output=True,
                text=True,
            )
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual(rows, [])
```

- [ ] **Step 2: Run the new gating test to verify it fails**

Run: `python3 -m unittest tests.test_ubx_best_tuple.TimingValidityTest.test_rejects_rawx_epoch_when_timegps_is_invalid -v`

Expected: FAIL because the current implementation still outputs one row.

- [ ] **Step 3: Implement `NAV-TIMEGPS` and `NAV-PVT` validation and reject clock-reset RAWX epochs**

```python
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
    confirmed_time = bool(pvt['valid_flags'] & 0x80)
    return valid_time and confirmed_time and pvt['fix_type'] >= 3


def extract_rows(path: Path) -> list[TupleRow]:
    data = path.read_bytes()
    rows: list[TupleRow] = []
    last_timegps = None
    last_pvt = None

    for msg_class, msg_id, payload in iter_ubx(data):
        if (msg_class, msg_id) == (0x01, 0x20):
            last_timegps = parse_timegps(payload)
            continue
        if (msg_class, msg_id) == (0x01, 0x07):
            last_pvt = parse_pvt(payload)
            continue
        if (msg_class, msg_id) != (0x02, 0x15):
            continue

        epoch = parse_rawx(payload)
        if epoch is None:
            continue
        if epoch['rec_stat'] & 0x02:
            continue
        if not timegps_is_valid(last_timegps):
            continue
        if not pvt_time_is_valid(last_pvt):
            continue
        if last_timegps['week'] != epoch['week']:
            continue
        if abs((last_timegps['itow_ms'] * 1e-3 + last_timegps['ftow_ns'] * 1e-9) - epoch['rcv_tow']) > 0.002:
            continue

        for measurement in epoch['measurements']:
            if measurement['gnss_id'] != 0 or not (measurement['trk_stat'] & 0x01):
                continue
            tx_week, tx_tow = wrap_week_tow(epoch['week'], epoch['rcv_tow'] - measurement['pr_mes'] / C_LIGHT)
            rows.append(TupleRow(measurement['sv_id'], measurement['sig_id'], tx_week, tx_tow, epoch['week'], epoch['rcv_tow']))

    return rows
```

- [ ] **Step 4: Run the tests for valid and invalid timing cases**

Run: `python3 -m unittest tests.test_ubx_best_tuple.RawOnlyCliTest tests.test_ubx_best_tuple.TimingValidityTest -v`

Expected: PASS.

- [ ] **Step 5: Commit the timing validation layer**

```bash
git add tools/ubx_best_tuple.py tests/test_ubx_best_tuple.py
git commit -m "feat: validate rx tuple with nav timegps and pvt"
```

## Task 3: Add Broadcast Clock Correction Math

**Files:**
- Modify: `tools/ubx_best_tuple.py`
- Modify: `tests/test_ubx_best_tuple.py`

- [ ] **Step 1: Add a failing unit test for week rollover and satellite clock correction**

```python
from tools.ubx_best_tuple import GpsClockTerms, corrected_transmit_time


class BroadcastClockMathTest(unittest.TestCase):
    def test_corrected_transmit_time_wraps_to_previous_week(self) -> None:
        clock = GpsClockTerms(
            toc=16.0,
            af0=2.3e-5,
            af1=0.0,
            af2=0.0,
            tgd=-5.0e-9,
        )
        tx_week, tx_tow = corrected_transmit_time(
            rx_week=2400,
            rx_tow=0.010,
            pseudorange_m=2997924.58,
            clock_terms=clock,
        )
        self.assertEqual(tx_week, 2399)
        self.assertGreater(tx_tow, 604799.9)
```

- [ ] **Step 2: Run the new unit test to verify it fails**

Run: `python3 -m unittest tests.test_ubx_best_tuple.BroadcastClockMathTest.test_corrected_transmit_time_wraps_to_previous_week -v`

Expected: FAIL with `ImportError` because `GpsClockTerms` and `corrected_transmit_time` do not exist yet.

- [ ] **Step 3: Add the broadcast clock math helpers**

```python
import math

MU_GPS = 3.986005e14
OMEGA_E_DOT = 7.2921151467e-5
F_REL = -4.442807633e-10


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
```

- [ ] **Step 4: Run the raw CLI tests and the new clock math test**

Run: `python3 -m unittest tests.test_ubx_best_tuple.RawOnlyCliTest tests.test_ubx_best_tuple.TimingValidityTest tests.test_ubx_best_tuple.BroadcastClockMathTest -v`

Expected: PASS.

- [ ] **Step 5: Commit the broadcast correction math**

```bash
git add tools/ubx_best_tuple.py tests/test_ubx_best_tuple.py
git commit -m "feat: add broadcast clock correction for tx tuple"
```

## Task 4: Decode GPS `RXM-SFRBX` And Use It In The CLI

**Files:**
- Modify: `tools/ubx_best_tuple.py`
- Modify: `tests/test_ubx_best_tuple.py`

- [ ] **Step 1: Add a failing unit test for `RXM-SFRBX`-backed correction selection**

```python
def sfrbx_frame(*, sv_id: int, words: list[int]) -> bytes:
    payload = struct.pack('<BBBBBBBB', 0, sv_id, 0, 10, 0, 1, 0, 0)
    payload += struct.pack('<10I', *words)
    return ubx_frame(0x02, 0x13, payload)


class BroadcastSelectionTest(unittest.TestCase):
    def test_cli_prefers_broadcast_corrected_tx_when_ephemeris_exists(self) -> None:
        ubx_bytes = b''.join(
            [
                timegps_frame(week=2400, itow_ms=345600200, ftow_ns=0, valid=0x03),
                pvt_frame(itow_ms=345600200, valid_flags=0x82),
                sfrbx_frame(sv_id=7, words=[0x8B0000, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
                rawx_frame(week=2400, rcv_tow=345600.2, pr_mes=20985472.123, sv_id=7),
            ]
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            ubx_path = Path(tmp_dir) / 'input.ubx'
            ubx_path.write_bytes(ubx_bytes)
            result = subprocess.run(
                [sys.executable, 'tools/ubx_best_tuple.py', str(ubx_path)],
                cwd=REPO,
                check=True,
                capture_output=True,
                text=True,
            )
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['sv_id'], '7')
```

- [ ] **Step 2: Run the new SFRBX-backed test to verify it fails**

Run: `python3 -m unittest tests.test_ubx_best_tuple.BroadcastSelectionTest.test_cli_prefers_broadcast_corrected_tx_when_ephemeris_exists -v`

Expected: FAIL because the current CLI ignores `RXM-SFRBX` entirely.

- [ ] **Step 3: Add `RXM-SFRBX` parsing and a small GPS ephemeris store**

```python
@dataclass
class GpsEphemeris:
    sv_id: int
    clock_terms: GpsClockTerms


def parse_sfrbx(payload: bytes):
    if len(payload) < 8:
        return None
    gnss_id, sv_id, _, num_words, _, version, _, _ = struct.unpack_from('<BBBBBBBB', payload, 0)
    words = list(struct.unpack_from(f'<{num_words}I', payload, 8))
    return {
        'gnss_id': gnss_id,
        'sv_id': sv_id,
        'num_words': num_words,
        'version': version,
        'words': words,
    }


def get_bits(word_list: list[int], start: int, width: int) -> int:
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


def decode_subframe_1_clock_terms(words: list[int], sv_id: int) -> GpsEphemeris:
    toc = get_bits(words, 218, 16) * 16.0
    af2 = signed(get_bits(words, 240, 8), 8) * 2.0 ** -55
    af1 = signed(get_bits(words, 248, 16), 16) * 2.0 ** -43
    af0 = signed(get_bits(words, 270, 22), 22) * 2.0 ** -31
    tgd = signed(get_bits(words, 196, 8), 8) * 2.0 ** -31
    return GpsEphemeris(
        sv_id=sv_id,
        clock_terms=GpsClockTerms(toc=toc, af0=af0, af1=af1, af2=af2, tgd=tgd),
    )


def extract_rows(path: Path) -> list[TupleRow]:
    data = path.read_bytes()
    rows: list[TupleRow] = []
    last_timegps = None
    last_pvt = None
    ephemeris_by_sv: dict[int, GpsEphemeris] = {}

    for msg_class, msg_id, payload in iter_ubx(data):
        if (msg_class, msg_id) == (0x01, 0x20):
            last_timegps = parse_timegps(payload)
            continue
        if (msg_class, msg_id) == (0x01, 0x07):
            last_pvt = parse_pvt(payload)
            continue
        if (msg_class, msg_id) == (0x02, 0x13):
            sfrbx = parse_sfrbx(payload)
            if sfrbx and sfrbx['gnss_id'] == 0 and sfrbx['num_words'] >= 10:
                ephemeris_by_sv[sfrbx['sv_id']] = decode_subframe_1_clock_terms(sfrbx['words'], sfrbx['sv_id'])
            continue
        if (msg_class, msg_id) != (0x02, 0x15):
            continue

        epoch = parse_rawx(payload)
        if epoch is None or epoch['rec_stat'] & 0x02:
            continue
        if not timegps_is_valid(last_timegps) or not pvt_time_is_valid(last_pvt):
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
```

- [ ] **Step 4: Run the full test suite**

Run: `python3 -m unittest discover -s tests -p 'test_*.py' -v`

Expected: PASS.

- [ ] **Step 5: Commit the SFRBX-driven transmit refinement**

```bash
git add tools/ubx_best_tuple.py tests/test_ubx_best_tuple.py
git commit -m "feat: refine tx tuple with sfrbx broadcast clock terms"
```

## Task 5: Finish The CLI Contract And Documentation

**Files:**
- Modify: `tools/ubx_best_tuple.py`
- Create: `docs/ubx-best-tuple.md`
- Modify: `tests/test_ubx_best_tuple.py`

- [ ] **Step 1: Add a failing test for deterministic CSV formatting**

```python
class CsvContractTest(unittest.TestCase):
    def test_cli_outputs_expected_header_order(self) -> None:
        ubx_bytes = b''.join(
            [
                timegps_frame(week=2400, itow_ms=345600200, ftow_ns=0, valid=0x03),
                pvt_frame(itow_ms=345600200, valid_flags=0x82),
                sfrbx_frame(sv_id=7, words=[0x8B0000, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
                rawx_frame(week=2400, rcv_tow=345600.2, pr_mes=20985472.123, sv_id=7),
            ]
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            ubx_path = Path(tmp_dir) / 'input.ubx'
            ubx_path.write_bytes(ubx_bytes)
            result = subprocess.run(
                [sys.executable, 'tools/ubx_best_tuple.py', str(ubx_path)],
                cwd=REPO,
                check=True,
                capture_output=True,
                text=True,
            )
        self.assertTrue(result.stdout.startswith('sv_id,sig_id,tx_week,tx_tow,rx_week,rx_tow\n'))
```

- [ ] **Step 2: Run the CSV contract test to verify it fails if formatting drifts**

Run: `python3 -m unittest tests.test_ubx_best_tuple.CsvContractTest.test_cli_outputs_expected_header_order -v`

Expected: PASS now or FAIL after any accidental field-order drift. Keep this test in place either way.

- [ ] **Step 3: Add a short module docstring, sorted output, and usage docs**

```python
"""Read a ZED-F9P UBX log and emit best-available GPS tx/rx tuples.

Input:  one .ubx binary log file containing RXM-RAWX, NAV-TIMEGPS, NAV-PVT, and RXM-SFRBX.
Output: CSV rows with sv_id,sig_id,tx_week,tx_tow,rx_week,rx_tow.
"""


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
```

```markdown
# UBX Best Tuple Tool

## Purpose

`tools/ubx_best_tuple.py` reads a ZED-F9P `.ubx` log and emits the best GPS tuple the log supports:

- `tx_week`
- `tx_tow`
- `rx_week`
- `rx_tow`

## Required UBX Messages

- `UBX-RXM-RAWX`
- `UBX-NAV-TIMEGPS`
- `UBX-NAV-PVT`
- `UBX-RXM-SFRBX`

## Invocation

```bash
python3 tools/ubx_best_tuple.py sample.ubx > tuples.csv
```

## Output Columns

- `sv_id`
- `sig_id`
- `tx_week`
- `tx_tow`
- `rx_week`
- `rx_tow`

## Acceptance Rules

- GPS measurements only (`gnssId == 0`)
- pseudorange valid (`trkStat & 0x01`)
- `NAV-TIMEGPS` week and TOW valid
- `NAV-PVT` valid and confirmed time
- `RAWX.recStat` must not report clock reset
- GPS broadcast clock terms must exist for the satellite

## Limitations

- `tx_*` is still a derived value, not a direct receiver field
- current plan uses GPS broadcast clock correction, not precise external orbit/clock products
- `TIM-TM2` is not used to move the observation epoch; it is only useful later for receiver timing validation
```

- [ ] **Step 4: Run the full verification commands**

Run: `python3 -m unittest discover -s tests -p 'test_*.py' -v`

Expected: PASS.

Run: `python3 tools/ubx_best_tuple.py /tmp/nonexistent.ubx`

Expected: non-zero exit with a Python file-not-found error. This is acceptable for the first version because the contract assumes a valid `.ubx` path.

- [ ] **Step 5: Commit the final CLI contract and docs**

```bash
git add tools/ubx_best_tuple.py tests/test_ubx_best_tuple.py docs/ubx-best-tuple.md
git commit -m "docs: describe zed-f9p ubx best tuple extractor"
```

## Spec Coverage Check

- Input is a `.ubx` file: covered by `tools/ubx_best_tuple.py` CLI in Tasks 1-5.
- Output is the tuple only: covered by the fixed CSV contract in Task 5.
- Best value possible on ZED-F9P: covered by `RXM-RAWX` receive time plus `NAV-TIMEGPS`/`NAV-PVT` validation and `RXM-SFRBX` broadcast clock correction in Tasks 2-4.
- GPS week/TOW tuples for both tx and rx: covered in all tasks via `wrap_week_tow()` and the final CSV schema.

## Placeholder Scan

- No `TODO` markers.
- No “implement later” steps.
- No implicit files; every created or modified file is named explicitly.
- No unspecified test commands.

## Type Consistency Check

- `TupleRow` stays the single output record type through all tasks.
- CLI output fields stay `sv_id,sig_id,tx_week,tx_tow,rx_week,rx_tow` through all tasks.
- `GpsClockTerms` is the only clock-correction input type introduced for `tx_*` refinement.
