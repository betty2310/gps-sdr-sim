# Synthetic Satellite Time-Basis Mismatch With Live Sky Signals

## Summary

When `gps-sdr-sim` generated satellites are transmitted alongside real sky signals, the receiver can track both sets of PRNs, but the pseudorange domain is not consistently aligned between them.

In the observed test case, synthetic PRNs `7` and `8` were received from the bladeRF transmission while PRN `5` and other satellites were still received from the sky. The receiver alternated between two internally consistent measurement groups instead of maintaining one shared GNSS time basis.

This note records the issue only. It does not propose mitigations or fixes.

## Test Context

- Simulator source: [gpssim.c](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/gpssim.c)
- SDR playback script: [bladerf.script](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/bladerf.script)
- Setup reference: [docs/synthetic-satellite-spoofing-setup.md](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/docs/synthetic-satellite-spoofing-setup.md)
- Receiver capture: [COM3___9600_260401_091104.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260401_091104.ubx)

Scenario characteristics:

- Synthetic GPS signals were generated with `gps-sdr-sim` and transmitted via bladeRF.
- Real sky GPS signals were still present at the receiver.
- Synthetic PRNs `7` and `8` were observed together with real satellites such as PRN `5`.
- The goal was to let the receiver track injected non-visible satellites while still receiving real sky signals.

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

This issue document intentionally stops at problem description and recorded evidence.
