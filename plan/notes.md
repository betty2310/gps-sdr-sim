# Notes: Synthetic Satellite Time-Basis Mismatch

## Relevant Sources

### `docs/issues/synthetic-satellite-time-basis-mismatch.md`
- The receiver can track synthetic PRNs and real PRNs at the same time.
- `UBX-RXM-RAWX` shows two pseudorange clusters separated by about `617.7511717728833 s`.
- The active timing reference appears to flip between the synthetic group and the real-sky group.
- The issue note points at `-T now`, `computeRange()`, `generateNavMsg()`, and plain file replay as relevant context.

### `docs/synthetic-satellite-spoofing-setup.md`
- bladeRF workflow is file generation followed by offline replay through `bladerf.script`.
- The setup document already fixes the sample-rate mismatch, so this issue is not the earlier 2.6 MHz vs 4 MHz problem.
- bladeRF playback examples use `repeat=0`, which means indefinite file replay.

### `docs/x300-spoofing-setup.md`
- X300 workflow is also file generation followed by offline replay.
- X300 has cleaner gain control, better isolation, and support for external `10 MHz` / `time_source=external`.
- Current examples still use `tx_samples_from_file` on a pre-generated file and do not bind TX start to the scenario epoch inside that file.

### `docs/signal-generation-pipeline.md`
- `computeCodePhase()` derives code/data timing directly from pseudorange and channel reference time.
- `generateNavMsg()` derives TOW from the scenario time base.
- The output file is therefore not "generic GPS-like IQ"; it is tied to one specific scenario epoch.

## Code Findings

### `gpssim.c`
- `-T now` sets `g0` from wall-clock GPS time and `date2gps()` converts it into the scenario start.
- With `timeoverwrite == TRUE`, the code shifts `TOC` and `TOE` to the scenario start epoch.
- `computeRange()` uses that scenario GPS time directly for pseudorange generation.
- `generateNavMsg()` rounds that same running time to 30-second frame boundaries and writes TOW fields from it.
- Output can go to `stdout` with `-o -`, so real-time streaming is possible in principle.

### `player/bladeplayer.c`
- The repo already has a native bladeRF player.
- The current implementation is synchronous untimed replay from a file.
- The main transmit loop at `player/bladeplayer.c:217` to `player/bladeplayer.c:295` already has the right buffer-fill then `bladerf_sync_tx()` structure for a streaming player.
- The current limitations are:
  - it opens a file with `fopen()` instead of stdin/FIFO/socket input,
  - EOF switches it to zero padding and stop,
  - there is no explicit stream-start handshake with the generator.

### `gps-sdr-sim-uhd.py`
- The UHD script is also untimed replay from a file.
- It sets sample rate, frequency, gain, and clock source, but not a start time.

## Updated Working Hypothesis

The dominant problem is that the generated I/Q file embeds one GNSS time basis, but playback begins later on the air. The synthetic satellites are internally self-consistent because all of them share the same stale scenario epoch, while the real-sky satellites share the current epoch. The receiver therefore sees two internally valid but mutually inconsistent measurement clusters.

The most direct fix is to stop materializing a replay file for mixed-sky mode and instead stream samples continuously from the generator into the transmitter so GNSS time never rewinds.

## Strong Supporting Clue

- `gpssim.bin` in this repo is about `299.9 s` long at `2.6 Msps` SC16.
- The measured cluster offset is `617.7511717728833 s`.
- `617.751... = 2 * 300 + 17.751...`.

This is exactly the pattern expected if:
1. the file was generated around `17.75 s` before RF playback started, and
2. the capture happened after two full file loops.

That strongly suggests the mismatch is dominated by delayed offline replay plus invalid looping for a time-coded GNSS file.

## Implications

- `repeat=0` on bladeRF and `--repeat N` on X300 are unsafe for mixed real-plus-synthetic tests unless the file duration covers the whole experiment and the replay never wraps.
- Even a perfect sample rate is not enough. The stream start must be coupled to the generator time base.
- For mixed-sky experiments, transmitter control is part of the timing model, not just the RF plumbing.

## Streaming Design Direction

- Prefer `gps-sdr-sim -o -` or a named FIFO for mixed-sky mode.
- Do not use offline `gpssim.bin` as the primary transport for mixed-sky tests.
- Replace "generate file, then replay later" with "start SDR pipeline, then begin live sample production".
- Treat current `-T now` as insufficient for this mode if it latches time too early relative to actual TX start.
- Add a stream-start handshake so the generator latches its initial GPS time only when the transmitter is ready to consume samples.

## Platform Outlook

### USRP X300
- Best first target for a robust fix.
- External `10 MHz` and PPS support make absolute start-time alignment realistic.
- A live stream TX launcher can be built around UHD more cleanly than around the current bladeRF CLI script flow.

### bladeRF 1.0
- Still usable, but not with plain `bladeRF-cli -s bladerf.script` for this problem.
- Needs a custom transmitter path built from `player/bladeplayer.c`, with streamed input replacing file replay.
- Expect higher residual risk because the platform clocking and idle leakage are worse than X300.
