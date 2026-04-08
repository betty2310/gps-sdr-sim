# bladetx Timing Model & Timeline Analysis

## The Three Time Domains

There are three independent clocks in the system. The fundamental challenge is
aligning them so the receiver sees synthetic pseudoranges on the same time basis
as live-sky signals.

```
 ┌──────────────┐    ┌──────────────┐    ┌───────────────────┐
 │  GPS TIME    │    │  HOST CLOCK  │    │  bladeRF COUNTER  │
 │  (true)      │    │  (MONOTONIC) │    │  (VCTCXO)         │
 │              │    │              │    │                   │
 │  ns-accurate │    │  ~1ms NTP    │    │  free-running     │
 │  from sats   │    │  monotonic   │    │  from module      │
 │              │    │  kernel      │    │  enable, no epoch │
 └──────┬───────┘    └──────┬───────┘    └────────┬──────────┘
        │                   │                     │
   Trimble PPS         nanosleep()         bladerf_get_
   time tag            getMonotonic()      timestamp()
        │                   │                     │
        └───────┬───────────┘                     │
           tag_lead_ms                            │
           (~500ms before PPS)                    │
                │                                 │
                └─────── linked via ──────────────┘
                   blade_lead_sec calculation
```

- **GPS TIME**: nanosecond-accurate, carried by satellite signals and delivered
  to the Trimble receiver. This is the ground truth.
- **HOST CLOCK**: `CLOCK_MONOTONIC` on the control computer. Used to track
  elapsed time between events (tag receipt, sleep, device timestamp read).
  Absolute accuracy depends on NTP (~1-50 ms), but relative accuracy over
  short intervals (< 5 s) is sub-millisecond.
- **bladeRF COUNTER**: a free-running sample counter driven by the on-board
  VCTCXO. Starts at zero when the TX module is enabled. Has no concept of
  wall-clock or GPS time. Advances at the configured sample rate (2.6 Msps).

## Event-by-Event Timeline

Example from an actual run with `--trimble-start-offset-sec 2`,
`--trimble-tag-lead-ms 500`, `--trimble-leap-sec 18`:

```
REAL TIME (UTC)  EVENT                                     CLOCK DOMAIN
─────────────────────────────────────────────────────────────────────────

09:26:30.500     Trimble sends "UTC 26.04.07 09:26:31"     GPS → TCP
  (estimated)    tag_lead_ms = 500ms before the PPS edge
                 that marks 09:26:31.000 UTC

09:26:30.5xx     trimbleReadTag() returns                  HOST
                 trimble_tag_mono = CLOCK_MONOTONIC latch
                 ├─ TAG SAYS: "09:26:31" (integer second)
                 └─ MONO LATCH marks when we received it

09:26:30.5xx     trimbleUtcToGpsEpoch():                   COMPUTATION
                 target UTC = 09:26:31 + 2s = 09:26:33
                 g0 = GPS(09:26:33) + 18s leap
                    = week 2413  tow 206811.000

                 The FIRST SAMPLE we generate represents
                 GPS time g0 = tow 206811.000

09:26:30.5xx     Pre-buffer: generate 5 epochs             COMPUTATION
  ~30ms          ring[0..4] = IQ for g0+0.0 .. g0+0.5s
                 (runs at CPU speed, ~30ms wall time)

09:26:30.530     Phase 1: total_remaining_sec              HOST
                 = (tag_lead + start_offset) - prep_elapsed
                 = (0.5 + 2.0) - 0.030
                 = 2.470 seconds until g0 should be on air

09:26:30.530     Phase 2: sleep for (2.470 - 0.300) = 2.17s HOST
                 ── sleeping ──

09:26:32.700     Phase 3: wake up, recompute               HOST
                 blade_lead_sec ≈ 0.300s actual remaining

09:26:32.700     Phase 4: bladerf_get_timestamp()          bladeRF
                 blade_now = N samples
                 tx_start_ts = N + (0.300 × 2,600,000)
                             = N + 780,000

09:26:32.700     Phase 5: send ring[0] with                HOST → USB → FPGA
                 BLADERF_META_FLAG_TX_BURST_START
                 meta.timestamp = tx_start_ts
                 ├─ 260,000 samples queued (~8 buffers)
                 └─ FPGA waits for counter to reach tx_start_ts

09:26:32.7xx     Drain ring[1..4] (continuation data)      HOST → USB → FPGA
                 4 × 260,000 more samples queued

09:26:33.000     ★ FPGA counter reaches tx_start_ts        bladeRF FPGA
                 First sample exits DAC → RF
                 This sample represents GPS time g0

                 ← THIS IS THE CRITICAL ALIGNMENT MOMENT →

09:26:33.0xx     Main loop: generate + send epoch by epoch  HOST + FPGA
  ongoing...     FPGA consumes at 2.6 Msps (real-time)
                 Generator runs ahead, ring stays ~1 epoch deep
```

## Where Timing Errors Enter

| #   | Error source                                                                                                                                               | Magnitude    | Affects what                                                   |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ | -------------------------------------------------------------- |
| 1   | `tag_lead_ms` estimate — "tag arrives ~500ms before PPS" is a guess; actual lead varies with Trimble internal processing, TCP buffering, kernel scheduling | ±100-500 ms  | Initial `g0` alignment with true GPS time. **Dominant error.** |
| 2   | `CLOCK_MONOTONIC` latch after `trimbleReadTag()` returns — TCP + kernel scheduling delay                                                                   | ~0.1-1 ms    | Same                                                           |
| 3   | `nanosleep()` wakeup jitter — OS scheduler granularity                                                                                                     | ~1-5 ms      | Shifts `blade_lead_sec`, but compensated by Phase 3 recompute  |
| 4   | `bladerf_get_timestamp()` to `bladerf_sync_tx()` gap — USB round-trip                                                                                      | ~0.01-0.1 ms | Slight shift in `tx_start_ts` accuracy                         |
| 5   | bladeRF FPGA timestamp resolution = 1 sample at 2.6 Msps                                                                                                   | ~384 ns      | Sample-grid quantization (sub-microsecond)                     |
| 6   | bladeRF VCTCXO drift — no external reference, ~1-2 ppm                                                                                                     | 1-2 us/sec   | Accumulating drift: +60 us after 60s, +600 us after 10 min     |

## The Dominant Error: tag_lead_ms

This is the biggest uncertainty in the current model.

```
Trimble PPS:      ──────┤ PPS edge (exact second boundary)
                        ↑
                   09:26:31.000 UTC (nanosecond-accurate)

Tag arrives:   ──┤ "UTC 26.04.07 09:26:31"
                 ↑
            09:26:30.5?? UTC
            (tag_lead_ms = 500ms is a GUESS)
```

The code assumes the tag arrives exactly `tag_lead_ms` before the PPS edge it
describes. But the actual lead varies with Trimble internal processing time,
TCP buffering, and whether the tag corresponds to the upcoming or just-passed
PPS.

The planned delay calculation:

```c
planned_delay = tag_lead_sec + start_offset;
//            = 0.500       + 2.0
//            = 2.500 seconds from tag receipt to target TX
```

If the actual lead was 300ms instead of 500ms, the GPS epoch is off by 200ms —
the same order of magnitude as the original 233ms mismatch.

The calibration term (`--trimble-tx-cal-ns`) can absorb this as a fixed offset
once measured empirically.

## The Time Binding: How g0 Maps to the First RF Sample

```
g0 (GPS time)  ←──computed from──  Trimble tag + offset + leap
     │
     ↓
generateEpoch(grx = g0 + 0.1s)
  └─ computeRange(eph, grx, xyz)
       └─ satpos(eph, grx) → satellite ECEF position at time grx
       └─ pseudorange = |sat_pos - rx_pos| + clock_correction
  └─ computeCodePhase(chan, rho, dt)
       └─ code_phase = f(grx - chan.g0, pseudorange)
       └─ Sets C/A code chip position and nav data bit
          position in the IQ samples
     │
     ↓
ring[0] = 260,000 IQ samples
  Sample[0] represents GPS time g0 + 0.1s
  Sample[N] represents GPS time g0 + 0.1s + N/2,600,000
     │
     ↓
bladerf_sync_tx(ring[0], meta.timestamp = tx_start_ts)
     │
     ↓
FPGA DAC emits Sample[0] when device counter = tx_start_ts
     │
     ↓
RF signal arrives at receiver antenna
  └─ Receiver correlates the C/A code
  └─ Measures pseudorange from code phase
  └─ Compares with live-sky pseudoranges
  └─ If g0 is wrong: pseudorange split → spoof detection
```

The receiver sees the pseudorange encoded in the C/A code phase. If `g0` is off
from true GPS time by `Δt`, the pseudorange error is `Δt × c` (speed of light):

| `Δt`    | Pseudorange error | Receiver behavior                 |
| ------- | ----------------- | --------------------------------- |
| 1 ms    | 300 km            | Rejected, spoof flagged           |
| 100 us  | 30 km             | Rejected, spoof flagged           |
| 10 us   | 3 km              | Marginal, de-weighted             |
| 1 us    | 300 m             | Marginal                          |
| 100 ns  | 30 m              | Accepted with minor residuals     |
| < 50 ns | < 15 m            | Transparent — within normal noise |

## Time Error Budget for bladetx

| Source                          | Uncalibrated  | With `--trimble-tx-cal-ns` | Notes                      |
| ------------------------------- | ------------- | -------------------------- | -------------------------- |
| `tag_lead_ms` accuracy          | ±200 ms       | absorbed                   | Fixed bias, measured once  |
| `CLOCK_MONOTONIC` jitter        | ±1 ms         | ±1 ms                      | Small, per-run variation   |
| `nanosleep` + Phase 3 recompute | ~0 ms         | ~0 ms                      | Compensated in code        |
| bladeRF timed TX                | ±0.4 us       | ±0.4 us                    | Sample-grid quantization   |
| USB pipeline + DAC latency      | fixed bias    | absorbed                   | Measured into cal term     |
| Cable + air path delay          | fixed bias    | absorbed                   | Measured into cal term     |
| VCTCXO drift                    | +1 us/sec     | +1 us/sec                  | Accumulates, uncorrectable |
| **Total initial**               | **±200 ms**   | **±few ms**                | After calibration          |
| **After 60s**                   | +60 us added  | +60 us added               | Drift on top of initial    |
| **After 10 min**                | +600 us added | +600 us added              | Drift on top of initial    |

## Comparison with x300tx.cpp

The `x300tx` path shares the same Trimble timing model. The key difference is
in TX scheduling:

```
x300tx:   usrp->set_time_now(0.0)     ← resets USRP clock to zero
          md.time_spec = remaining     ← schedule relative to reset

bladetx:  bladerf_get_timestamp()      ← reads free-running counter
          meta.timestamp = now + lead  ← schedule relative to read
```

Both have the same `tag_lead_ms` uncertainty. The X300 has two additional
capabilities that the bladeRF 1.0 lacks:

| Capability                | X300                                      | bladeRF 1.0       |
| ------------------------- | ----------------------------------------- | ----------------- |
| External PPS input        | Yes — can discipline TX start to PPS edge | No                |
| External 10 MHz reference | Yes — eliminates clock drift              | No                |
| Clock accuracy (internal) | ~1 ppm (TCXO)                             | ~1-2 ppm (VCTCXO) |
| Clock accuracy (external) | ~0.01 ppm                                 | N/A               |

With external PPS + 10 MHz, the X300 can reach < 1 us alignment. The bladeRF
is limited to whatever accuracy the Trimble tag + host clock chain can achieve,
plus ongoing VCTCXO drift.

## bladeRF Buffer Flow Model

The bladeRF sync interface manages an internal buffer pool. Understanding its
flow explains the sleep-before-send strategy in `bladetx.cpp`:

```
Host memory                USB bus              FPGA
┌──────────────┐                            ┌────────────┐
│ Buffer pool  │──── USB transfers ────────→│ Sample     │
│ 32 buffers   │    (16 in flight max)      │ FIFO       │
│ × 32K samps  │                            │            │
│ = 1M samples │                            │  ┌──────┐  │
│ = 0.4 sec    │                            │  │ DAC  │──→ RF out
│              │                            │  └──────┘  │
└──────────────┘                            └────────────┘

bladerf_sync_tx() copies samples into the buffer pool.
When a buffer is full, it is submitted as a USB transfer.
The FPGA receives transfers into its sample FIFO.
In timed TX mode, the FPGA does NOT drain the FIFO to the DAC
until the device counter reaches meta.timestamp.
```

If we schedule TX 2.5 seconds ahead and immediately start queuing, the chain
backs up:

1. FPGA FIFO fills (FPGA won't consume yet)
2. USB transfers stall (FPGA can't accept more)
3. Buffer pool fills (USB transfers stuck)
4. `bladerf_sync_tx()` blocks (no free buffers)
5. After `TIMEOUT_MS` (1000ms) → timeout error

The fix in `bladetx.cpp`: sleep for `total_delay - 300ms`, then read the device
timestamp and schedule with only a 300ms lead. The FPGA starts consuming within
300ms, before the 32-buffer pool can fill.

## Calibration Procedure

1. Measure `tag_lead_ms` empirically: connect a scope to the Trimble PPS output,
   monitor TCP for the tag, measure the actual lead time. Update
   `--trimble-tag-lead-ms` to the measured value.

2. Run a test capture with `--trimble-tx-cal-ns 0`. Log `UBX-RXM-RAWX`,
   `UBX-NAV-SAT`, `UBX-NAV-PVT`, and spoof status on the receiver.

3. Measure the common pseudorange residual between synthetic and real-sky groups.

4. Convert the residual to nanoseconds: `cal_ns = -residual_sec × 1e9`.

5. Apply as `--trimble-tx-cal-ns <cal_ns>` on subsequent runs.

6. Repeat until the residual reaches the best repeatable level.

The calibration term absorbs all fixed biases in one number: `tag_lead_ms`
error, USB pipeline delay, DAC latency, cable delay, and air-path delay.

## Realistic Targets

| Configuration                       | Initial alignment | After 60s       | After 10 min     |
| ----------------------------------- | ----------------- | --------------- | ---------------- |
| Uncalibrated                        | ±200 ms           | ±200 ms + 60 us | ±200 ms + 600 us |
| Calibrated (`--trimble-tx-cal-ns`)  | ±few ms           | ±few ms + 60 us | ±few ms + 600 us |
| Calibrated + measured `tag_lead_ms` | ±1-2 ms           | ±1-2 ms + 60 us | ±1-2 ms + 600 us |

Compared to the original 233 ms (70,000 km) mismatch from the pipe-based path,
a calibrated `bladetx` should reduce the initial error by ~100x. The remaining
floor is set by the `tag_lead_ms` per-run jitter (~1-2 ms) and the VCTCXO drift
(~1 us/sec).

For runs under 5 minutes, the VCTCXO drift stays below 300 us and should not
trigger spoof detection on its own. For longer runs, the drift will eventually
become the dominant error source, and there is no correction available on
bladeRF 1.0 hardware.
