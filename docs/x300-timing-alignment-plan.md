# X300 Timing Alignment Plan

## Goal

Improve software-only timing alignment between generated GPS data time and the
actual RF transmission time from `x300tx`, so injected PRNs have the best chance
to be accepted with real-sky PRNs in u-blox PVT (`svUsed=Y`).

This plan assumes no external 10 MHz/PPS reference is available. It does not
replace a hardware clock reference; it makes the code path deterministic,
observable, and calibratable.

## Current Status

- [x] X300 sample-rate mismatch addressed by defaulting `x300tx` to `2.5 MHz`.
- [x] Generator still locks to UHD's reported actual TX rate before producing IQ.
- [x] `--trimble-tx-cal-ns` uses sub-millisecond GPS-time handling in
  `player/x300tx.cpp`.
- [x] Generated GPS time is derived from cumulative sample counts.
- [x] Software drift compensation is available with `--gps-time-ppm`.

## Timing Model

The system has four relevant time bases:

1. Real GPS time from live-sky satellites.
2. Trimble time-tag UTC text and its configured lead estimate.
3. Host monotonic time used to measure elapsed preparation delay.
4. X300 hardware time used by UHD timed TX metadata.

The software goal is to define a clear contract:

```text
first RF sample emitted by X300 == generated GPS time g0 + calibration
sample N emitted by X300        == generated GPS time g0 + N / tx_rate
```

If this contract is true, remaining receiver residuals can be interpreted as:

- constant offset: adjust `--trimble-tx-cal-ns`
- time drift: adjust a future ppm correction knob
- geometry/nav mismatch: fix PRN selection, RINEX, or receiver location

## Phase 1: Preserve Exact Rate Contract

Status: complete.

Implementation target:

- `player/x300tx.cpp`

Requirements:

- Default requested X300 TX rate should be an exact hardware rate (`2500000 Hz`).
- `--rate <Hz>` should allow explicit override.
- After `usrp->set_tx_rate(...)`, read back `usrp->get_tx_rate(...)`.
- Use the actual UHD rate for IQ generation, epoch sizing, and timing logs.

Expected log:

```text
[UHD] TX rate:    2500000 Hz (actual 2500000 Hz)
[TIMING] Generator sample rate locked to actual TX rate: 2500000.000000 Hz
[TIMING] Epoch sample count: 250000 samples every 100.0 ms
```

## Phase 2: Remove Millisecond Quantization

Status: complete.

Problem:

- `--trimble-tx-cal-ns` is parsed as nanoseconds.
- The value is converted to seconds and applied through `incGpsTime(...)`.
- `incGpsTime(...)` rounds GPS seconds to 1 ms.
- This makes sub-millisecond calibration ineffective.

Implementation target:

- Add a precise helper in `gpssim.c` / `gpssim.h`, or a local helper in
  `player/x300tx.cpp` if scope must stay X300-only.

Candidate helper:

```c
gpstime_t incGpsTimePrecise(gpstime_t g0, double dt);
```

Rules:

- Do not round to milliseconds.
- Keep week rollover behavior.
- Use this helper for `--trimble-tx-cal-ns`.
- Use it for X300 streaming time advancement if the broader codebase can accept
  sub-millisecond `gpstime_t.sec`.

Verification:

- Run with `--trimble-tx-cal-ns 580000`.
- Expected target TOW should shift by `0.000580 s`, not `0.001000 s`.
- `make x300tx` must pass.

## Phase 3: Drive GPS Time From Emitted Sample Count

Status: complete.

Problem:

The current loop advances GPS time by repeatedly adding epoch duration. That is
close, but the strongest software contract is to derive generation time from the
number of samples that will be emitted.

Implementation target:

- `player/x300tx.cpp`

Proposed state:

```c
long long emitted_samples;
gpstime_t first_sample_gps_time;
```

For each generated block:

```text
block_start_gps_time = first_sample_gps_time + emitted_samples / actual_tx_rate
block_end_gps_time   = block_start_gps_time + sample_count / actual_tx_rate
```

Use the time expected by existing `generateEpoch(...)` semantics. If
`generateEpoch(...)` expects the epoch end time, pass `block_end_gps_time`.
Document this explicitly in the code and log.

Requirements:

- The first block sent with timed TX must have a logged GPS time contract.
- Dithered epoch sample counts must not accumulate timing error.
- `Time into run` should derive from sample count, not loop count.

Suggested log:

```text
[TIMING] First RF sample GPS epoch: week 2421 tow 441231.000580000
[TIMING] First generated epoch end: week 2421 tow 441231.100580000
[TIMING] TX sample clock: 2500000.000000 Hz
```

## Phase 4: Add Software Drift Compensation

Status: complete.

Problem:

Without external 10 MHz/PPS, the X300 internal oscillator can be slightly fast or
slow relative to true GPS time. A constant `--trimble-tx-cal-ns` offset cannot
fix residual slope over a long capture.

Implementation target:

- `player/x300tx.cpp`

Add one knob:

```bash
--gps-time-ppm <ppm>
```

or:

```bash
--tx-time-scale-ppm <ppm>
```

Generation equation:

```text
gps_elapsed = emitted_samples / actual_tx_rate * (1.0 + ppm * 1e-6)
```

Calibration method:

- Capture 90-180 s of u-blox UBX.
- Compare `NAV-SAT.prRes` for injected PRNs versus real-sky `svUsed` anchor PRNs.
- Constant median residual updates `--trimble-tx-cal-ns`.
- Residual slope over time updates `--gps-time-ppm`.
- `tools/ubx_bladetx_cal.py` reports both recommendations when residual epochs
  are available.

## Phase 5: Receiver Validation Loop

Status: active.

Capture requirements:

- Enable `UBX-RXM-RAWX`.
- Enable `UBX-NAV-SAT`.
- Enable `UBX-NAV-PVT`.
- Prefer `UBX-NAV-SIG` if available.
- Run long enough for receiver use decisions: `90-180 s`.

Per-PRN checks:

```bash
uv run tools/ubx_check_prused.py <log.ubx> 3 --show-epochs --limit 20
uv run tools/ubx_check_prused.py <log.ubx> 30 --show-epochs --limit 20
```

Calibration check:

```bash
uv run tools/ubx_bladetx_cal.py <log.ubx> \
  --inject 3,30 \
  --current-trimble-tx-cal-ns <current_ns> \
  --current-gps-time-ppm <current_ppm> \
  --trimble-tag-lead-ms 788 \
  --trimble-start-offset-sec 2
```

Success criteria:

- Injected PRNs appear in `NAV-SAT`.
- Injected PRNs have stable `qualityInd`.
- `NAV-SAT.flags.svUsed` becomes true for at least some injected PRNs.
- `NAV-SAT.prRes` for injected PRNs is not separated from real-sky anchors by a
  large common bias.
- `NAV-PVT.numSV` includes both real-sky and injected PRNs in stable epochs.

Dataset reference:

- `processing/dataset/05-06/ver1`, with injected PRNs `3,30`, gave a usable
  NAV-SAT calibration window only in `COM3___9600_260605_025844.ubx`.
- With `--current-trimble-tx-cal-ns 580000` and
  `--current-gps-time-ppm 0`, the tool reported:
  `--trimble-tx-cal-ns 580047` and `--gps-time-ppm +0.002894118`.
- `COM4___9600_260605_025726.ubx` tracked PRNs `3,30`, but they were not
  `svUsed` at NAV-SAT level, so it was not usable for timing calibration.

## Implementation Order

1. Patch precise GPS-time addition for `--trimble-tx-cal-ns`.
2. Add first-RF-sample GPS timing logs.
3. Change X300 streaming time to derive from emitted sample count.
4. Capture UBX and estimate constant offset.
5. Add ppm drift knob only if residuals show a clear slope.
6. Capture UBX again and verify `svUsed`.

## Open Questions

- Does `generateEpoch(...)` conceptually generate the epoch ending at `grx` or
  beginning at `grx` in the X300 path? Confirm before the sample-count rewrite.
- Should precise GPS-time addition be global in `gpssim.c`, or should X300 use a
  local helper to avoid disturbing legacy file-generation behavior?
- What is the measured residual slope of the X300 internal clock over 90-180 s?
- Is the current `--trimble-tag-lead-ms 788` stable across multiple Trimble tag
  captures, or should it be estimated from logs each run?

## Notes

- Hardware clock reference is still the correct long-term solution.
- Software-only alignment can improve offset and short-term drift, but it cannot
  guarantee GPS-disciplined stability.
- Receiver proof must use `svUsed` / `prUsed`, not tracking alone.
