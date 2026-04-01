# Synthetic Satellite Time-Basis Mismatch: Solution Plan

## Executive Summary

The issue is most likely not in synthetic satellite geometry itself. It is the combination of:

1. a GNSS time-coded I/Q file generated at one absolute epoch,
2. delayed RF playback at a later epoch, and
3. file looping, which rewinds GNSS time by the file duration on every repeat.

For mixed live-sky plus injected-satellite tests, the solution should move away from offline replay and into a direct stream pipeline. The recommended path is:

- First, implement a live streaming path on **USRP X300**.
- Then, implement the same streaming model for **bladeRF 1.0** using [player/bladeplayer.c](/Users/betty/Developer/gnss-tools/gps-sdr-sim/player/bladeplayer.c) as the reference transmit loop.
- Stop using offline replay and raw-file looping for mixed-sky tests.

## Required Outcome

After the fix:

- synthetic PRNs and real-sky PRNs should stay on one shared pseudorange time basis,
- the two-cluster RAWX behavior should disappear,
- synthetic PRNs should begin appearing in the receiver's final solution set instead of only in tracking state.

## Root Cause Model

`gps-sdr-sim` does not generate a timeless waveform. It generates a waveform tied to the scenario epoch `g0`.

- `computeRange()` computes pseudorange at that epoch.
- `computeCodePhase()` sets code/data phase from that pseudorange.
- `generateNavMsg()` writes TOW derived from the same running time base.

If the waveform is transmitted later, the synthetic cluster is shifted by `Delta_t * c` relative to live sky. If the file loops, GNSS time jumps backwards by the file duration at every wrap. Both effects are fatal for mixed-sky coherence.

## Hard Rules Going Forward

1. Do not use raw-file looping for mixed-sky spoofing tests.
2. Do not use `-T now` plus unscheduled offline replay for mixed-sky tests.
3. Prefer direct streaming from `gps-sdr-sim` into the SDR transmitter.
4. If "start now" is used, the generator must latch its time basis when TX is actually ready, not long before.
5. If a file is ever kept for debugging, it must be single-pass only and must not be the default mixed-sky path.

## Phase 1: Build A Streaming Architecture

Purpose: remove the delayed replay path from mixed-sky experiments.

Work:

- Add a stream mode for mixed-sky runs:
  - `gps-sdr-sim` writes SC16 to stdout or a FIFO,
  - the transmitter process consumes samples continuously,
  - no intermediate `gpssim.bin` is required for the main workflow.
- Add a start handshake between generator and transmitter so the generator's initial GPS time is latched when the SDR side is ready.
- Ensure the streaming path runs at real time:
  - generator produces samples at the configured sample rate,
  - transmitter consumes at the same rate,
  - no rewind and no loop semantics.

Acceptance:

- Mixed-sky mode no longer depends on pre-generated files or repeat counts.

## Phase 2: Generator Changes

Purpose: make `gps-sdr-sim` suitable for direct live TX.

Work:

- Add a streaming-oriented start mode for mixed-sky use.
- Do not rely on existing `-T now` semantics if they snapshot time before the TX chain is actually live.
- Preferred behavior:
  - initialize SDR path first,
  - signal generator readiness,
  - generator latches "GPS now",
  - generator starts continuous output immediately.
- Keep `-o -` as the primary transport interface.

Recommended interfaces:

- `--stream-now`
  - latches GPS time at stream start
  - outputs SC16 to stdout
- optional FIFO mode
  - same semantics as stdout, easier debugging on some hosts

## Phase 3: USRP X300 Streaming Path

Purpose: build the first robust transmitter path on the platform most likely to succeed.

### Why X300 First

- External `10 MHz` and PPS support.
- Lower clock error and lower phase noise.
- Better fit for deterministic timed transmission.

### Implementation Plan

1. Replace `tx_samples_from_file` for mixed-sky mode with a live stream TX program.
2. Read SC16 from stdin or FIFO instead of from a pre-generated file.
3. Use external `clock_source=external,time_source=external` whenever a GPSDO/PPS is available.
4. Start the UHD sink first, then trigger generator stream start.
5. Run continuously for as long as the experiment needs. No repeat logic.

Preferred engineering direction:

- Add a new X300 launcher, for example `player/usrp_stream_player.py` or a small UHD C++ tool, with:
  - stdin/FIFO sample input,
  - `--clock-source`,
  - `--time-source`,
  - `--gain`,
  - optional `--rate`,
  - optional `--freq`.
- Primary invocation shape:

```bash
./gps-sdr-sim ... --stream-now -o - | player/usrp_stream_player.py ...
```

Operational target:

- X300 becomes the reference platform for all mixed-sky synthetic-satellite experiments.

## Phase 4: bladeRF 1.0 Streaming Path

Purpose: provide a workable bladeRF route, with clear limits.

### Constraint

The current `bladeRF-cli` script path is not suitable for mixed-sky time-coherent injection because it replays a file without any explicit start-time control. The correct code reference is `player/bladeplayer.c`, not `bladerf.script`.

### Implementation Plan

1. Do not use `bladerf.script` for mixed-sky tests except as a legacy baseline.
2. Extend [bladeplayer.c](/Users/betty/Developer/gnss-tools/gps-sdr-sim/player/bladeplayer.c) instead of the CLI script path.
3. Reuse the existing buffer-fill and `bladerf_sync_tx()` structure in `player/bladeplayer.c:217` to `player/bladeplayer.c:295`.
4. Replace file-based `fread()` input with stdin or FIFO streaming input.
5. Remove EOF-driven zero-padding behavior for normal stream mode; instead block waiting for more samples until shutdown.
6. Start bladeRF first, then trigger generator stream start.
7. Keep a file-input fallback only for debugging, not for the normal mixed-sky path.

Operational target:

- bladeRF supports the same live streaming model as X300,
- X300 remains the preferred platform for stable publication-grade measurements.

## Phase 5: Validation Campaign

Purpose: prove that the timing fix solved the actual issue.

Test ladder:

1. Conducted, synthetic-only, live stream
2. OTA, synthetic-only, live stream
3. OTA, mixed sky + 1 synthetic PRN, live stream
4. OTA, mixed sky + 2 synthetic PRNs, live stream
5. Long-duration live stream with no file artifact in the path

For each run, collect:

- `UBX-RXM-RAWX`
- `UBX-NAV-SAT`
- `UBX-NAV-SIG`
- `UBX-NAV-STATUS`
- transmitter start logs

Success criteria:

- no persistent synthetic-versus-real pseudorange cluster split,
- no bias step consistent with replay delay or file wrap,
- synthetic PRNs move from tracked-only to used-in-solution,
- `spoofDetState` does not increase solely because of timing incoherence.

## Recommended Work Order

1. Add generator live stream mode.
2. Build the X300 stream player and validate it first.
3. Convert `player/bladeplayer.c` into a stream-capable bladeRF player.
4. Update the docs so mixed-sky mode is stream-first and replay is debug-only.
5. Re-run the PRN 7/8 style experiment through the live stream path.

## What I Would Not Do First

- I would not start by changing synthetic orbit generation.
- I would not start by tuning RF power again; the symptoms are too structured for a pure power problem.
- I would not rely on `-T now` plus offline replay as the long-term method.
- I would not allow `repeat=0` or repeated 300 s files in mixed-sky mode.

## Deliverables To Build Next

- generator streaming updates in `gpssim.c`
- `player/usrp_stream_player.py` or equivalent UHD stream TX tool
- streaming extensions to `player/bladeplayer.c`
- doc updates in:
  - `docs/synthetic-satellite-spoofing-setup.md`
  - `docs/x300-spoofing-setup.md`
  - `docs/issues/synthetic-satellite-time-basis-mismatch.md`
