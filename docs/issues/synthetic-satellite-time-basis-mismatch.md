# Synthetic Satellite Time-Basis Mismatch With Live Sky Signals

## Summary

When `gps-sdr-sim` generated satellites are transmitted alongside real sky signals, the receiver can track both sets of PRNs, but the pseudorange domain is not consistently aligned between them.

In the original capture that motivated this issue, synthetic PRNs `7` and `8` were received from the bladeRF transmission while PRN `5` and other satellites were still received from the sky. The receiver alternated between two internally consistent measurement groups instead of maintaining one shared GNSS time basis.

This note records the issue, the partial mitigation that has since been implemented,
and the remaining work still needed for deterministic mixed real-plus-synthetic timing.

## Test Context

- Simulator source: [gpssim.c](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c)
- SDR playback script: [bladerf.script](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/bladerf.script)
- Setup reference: [docs/synthetic-satellite-spoofing-setup.md](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/docs/synthetic-satellite-spoofing-setup.md)
- Latest validation captures:
  - [COM3___9600_260403_024818.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260403_024818.ubx)
  - [COM4___9600_260403_024827.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM4___9600_260403_024827.ubx)

Scenario characteristics:

- Synthetic GPS signals were generated with `gps-sdr-sim` and transmitted via bladeRF.
- Real sky GPS signals were still present at the receiver.
- Synthetic PRNs from the actual transmit list were `3`, `4`, `7`, and `8`.
- The goal was to let the receiver track injected non-visible satellites while still receiving real sky signals.

Latest field-test command:

```bash
gps-sdr-sim -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 8,3,4,7 -S 3:0/60,5:45.0/55.0,4:90/45,7:180/30 \
  -n -o - | ./player/bladeplayer -f - -b 16 -g -35
```

Important note about this command:

- `-P` is the actual synthetic PRN transmit list, so the injected set was `3,4,7,8`.
- `-S` in this run still mismatched the transmit list by naming PRN `5` and omitting PRN `8`.
- Any later analysis of this run must therefore treat PRN `5` as real sky unless independent evidence shows otherwise.

## Observed Receiver Behavior

The receiver does not behave as if all measurements belong to one common time reference.

Observed symptoms:

- PRNs `7` and `8` appear as valid tracked GPS signals.
- Real satellites remain visible at the same time.
- Raw pseudoranges split into two groups with a large common offset between them.
- At some epochs, synthetic PRNs have plausible GPS pseudoranges while real-sky PRNs carry the large offset.
- At later epochs, the opposite occurs: real-sky PRNs return to plausible ranges while synthetic PRNs carry the large offset.
- The receiver appears to choose one group as the active timing reference and treat the other group as inconsistent.

This creates an unstable mixed-constellation condition where synthetic and real satellites cannot be treated as one coherent measurement set.

## Measurement Evidence

Analysis of `UBX-RXM-RAWX` from [COM3___9600_260401_091104.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260401_091104.ubx) shows a stable synthetic-versus-real pseudorange bias of approximately:

- `617.7511717728833 s`
- Equivalent range bias: about `185,197,142,218 m`

Examples from the capture:

- Early epoch: PRN `7` is near `20,691,936.69 m`, PRN `8` is near `21,495,371.23 m`, while PRN `5` is near `-185,176,417,723.45 m`.
- Later epoch: PRN `5` is near `20,095,881.90 m`, while PRN `7` is near `23,394,297.02 m` and PRN `8` is near `18,211,536.78 m`.

The important property is not the sign of the large values, but that the mismatch behaves like a common time-basis error shared across one satellite group at a time.

## Receiver Solution Impact

The issue is visible not only in raw measurements, but also in which signals contribute to the navigation solution.

From the same capture:

- Synthetic PRNs `7` and `8` are tracked as GPS L1 signals.
- During the first stable 3D fix, the receiver solution uses real satellites and does not use PRNs `7` and `8` in the final pseudorange set.
- This indicates that tracking alone is not enough; the mixed real-plus-synthetic set still fails consistency checks at the solution level.

So the problem is not simply acquisition failure. The core issue is that tracked synthetic satellites do not remain time-consistent with real satellites strongly enough to participate reliably in the same position solution.

## Code Context

The timing behavior is consistent with how the simulator constructs signal time:

- `-T now` sets the scenario start time from the current GPS time in [gpssim.c](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c#L2467-L2485).
- Ephemeris clock and orbit reference times are shifted to that scenario time in [gpssim.c](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c#L2689-L2718).
- Synthetic ephemerides are also anchored from the same scenario start in [gpssim.c](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c#L2789-L2825).
- Pseudorange generation uses that scenario time inside [computeRange](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c#L1795-L1849).
- Navigation message timing and TOW fields are generated from the same running time base in [generateNavMsg](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c#L2049-L2129).
- The bladeRF script simply replays a previously generated file in [bladerf.script](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/bladerf.script#L8-L10).

This documents the timing relationship that must be understood when reading the issue. It is included here as context, not as a proposed resolution.

## Problem Statement

The current workflow can produce a state where:

- synthetic satellites are trackable,
- real sky satellites are simultaneously trackable,
- but both sets do not share one consistent pseudorange time basis at the receiver.

As a result, the receiver may alternate between measurement clusters or exclude one cluster from the final navigation solution. This prevents stable combined use of real and injected satellites in the same GNSS fix.

## Scope Note

The first part of this note preserves the original problem statement and captured evidence.
The sections below summarize the current status after the first timing fix and define
the remaining work.

## Latest Validation Results

The April 3 captures do **not** reproduce the original catastrophic `617.751 s`
time-basis failure from [COM3___9600_260401_091104.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260401_091104.ubx).

Instead, they show a smaller but still very large residual mixed-set timing split:

- dominant residual bias on PRNs `4`, `5`, and `8`: about `0.233 s`
- equivalent range error: about `69,900 km`
- PRN `7` remained in a normal GPS pseudorange range throughout both captures
- PRN `3` still showed a separate large ambiguity of about `1.307 s` before later
  returning near the normal range

Capture-specific behavior:

- [COM3___9600_260403_024818.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260403_024818.ubx)
  contains `80` RAWX epochs over about `84 s`
- [COM4___9600_260403_024827.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM4___9600_260403_024827.ubx)
  contains `148` RAWX epochs over about `172 s`
- both captures keep a `3D` fix in all `NAV-PVT` epochs
- both captures keep the same RAWX `rcvTow` fractional part of `0.991`, so the old
  multi-hundred-millisecond epoch-family split seen in earlier testing is not present

Mixed real-plus-synthetic stability differs between the two receivers:

- `COM3___9600_260403_024818.ubx` remains mostly unstable, with only a short late
  interval where the tracked real and synthetic subsets simultaneously sit in a
  plausible pseudorange range
- `COM4___9600_260403_024827.ubx` improves substantially after startup and spends
  most of its later RAWX span in a mixed-normal state

Spoof-detection status from `NAV-STATUS`:

- `COM3___9600_260403_024818.ubx`: `no_spoof` at the start, then `spoof_indicated`
  for `90` epochs, then back to `no_spoof`
- `COM4___9600_260403_024827.ubx`: `multi_spoof` first, then `spoof_indicated` for
  the remainder of the capture; there are no `no_spoof` epochs in this file

One important limitation remains:

- these April 3 captures do not include `UBX-NAV-SAT`, so they cannot prove which
  PRNs were finally marked `svUsed=Y` by the navigation engine

## Status Update

A first timing mitigation has been implemented in the simulator and bladeRF player:

- `gps-sdr-sim -n` now latches GPS time with fractional-second precision instead of
  whole-second `time()` resolution.
- `gps-sdr-sim` now accepts `-r <lead_sec>` to choose a future stream start epoch.
- `player/bladeplayer` now supports timed TX start for stdin/FIFO streaming using
  bladeRF metadata timestamps instead of pure "transmit as soon as queued".

This removes the largest previously identified source of delay uncertainty:

- asynchronous queue-driven bladeRF start
- whole-second truncation when the stream start time is chosen

However, the April 3 validation shows that this does **not** fully solve the mixed
real-plus-synthetic timing problem.

## What Was Likely Fixed

The original streamed bladeRF path had a strong candidate for the dominant delay:

- `bladeplayer` was configured with `32` buffers of `32768` samples at `2.6 Msps`
- that queue depth corresponds to about `0.403 s`

That value is close to the `~0.378 s` fractional `rcvTow` separation seen in
`sieungon.ubx`, so timed TX start should reduce or remove this specific
"queue decides start time" error mode.

## Remaining Issues

The system still does not have one single authoritative start instant shared by
both the IQ generator and the transmitter.

Remaining technical issues:

- `gps-sdr-sim` chooses its future GPS epoch from host wall-clock time.
- `bladeplayer` chooses its future TX timestamp from bladeRF device time.
- Those two `now` measurements are taken in different processes at different moments.
- The configured lead time such as `-r 1.0` is only an arm margin, not a calibrated
  physical delay estimate.
- Process launch order, bladeRF initialization time, and host scheduling can still
  create residual skew between the intended GPS epoch and the actual TX epoch.
- No PPS or GPSDO-based common timing reference is used yet.
- No closed-loop verification has yet confirmed that the remaining skew is small
  enough for the receiver to merge real and synthetic satellites into one solution.

Operational issues still present:

- If the synthetic PRN list in `-P` and `-S` do not match, experiment logs can be
  misleading. The April 3 validation command transmitted PRNs `3,4,7,8` but defined
  synthetic geometry for `3,4,5,7`.
- The older capture `COM3___9600_260401_091104.ubx`, the later capture
  `sieungon.ubx`, and the April 3 captures do not show the same mismatch magnitude.
  They represent different timing-failure regimes and should not be treated as one
  single numeric bias.
- The April 3 captures did not log `UBX-NAV-SAT`, so solution participation cannot
  yet be confirmed directly.

## Interpretation Of Lead Time

The current `-r 1.0` default should be interpreted as:

- enough future margin for the host to generate data and arm the transmitter
- not proof that the true physical pipeline delay is exactly one second
- not a deterministic guarantee of sub-millisecond alignment

So `1.0 s` is a safe startup cushion, but it is not the final timing solution.

## Next Things To Do

Priority 1: validate the new timed-start path on hardware

- Run a fresh bladeRF OTA capture using matching PRN definitions in both `-P` and `-S`.
- Use the new timed stream command with the same `-r` value on both tools.
- Record a new UBX log and compare:
  - RAWX pseudorange split
  - `rcvTow` fractional separation by state
  - spoof-detection state
  - whether synthetic PRNs become `svUsed=Y` in the final solution from `UBX-NAV-SAT`

Priority 2: measure residual cross-process skew directly

- Add debug logging of:
  - host wall-clock at `gps-sdr-sim` start-time latch
  - chosen GPS start epoch in `gps-sdr-sim`
  - bladeRF timestamp read in `bladeplayer`
  - scheduled TX timestamp in `bladeplayer`
- From these logs, estimate the residual skew between generator epoch selection
  and transmitter start scheduling.

Priority 3: make both processes share one start authority

- Implement a controller process or handshake so one component chooses a single
  future start time and passes that exact target to both the generator and the
  transmitter.
- This should replace the current "each process samples its own now" behavior.

Priority 4: add a real common timing reference

- Use u-blox time plus PPS, or a GPSDO / external reference, to anchor the start.
- Longer term, this is the correct path if the goal is stable coexistence with
  real sky signals instead of only approximate startup alignment.

## Recommended Next Experiment Command

For bladeRF stdin streaming, use matching PRNs and timed start on both tools:

```bash
./gps-sdr-sim -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \
  -n -r 1.0 -o - | ./player/bladeplayer -f - -b 16 -g -35 -r 1.0
```

If this still leaves a measurable split, the next implementation target should be
a single-controller start-time handshake rather than further increasing the lead time.
