# SYNTH_REVIVE Current State After Dataset 22-4/ver3

## Status

Open — field-tested on 2026-04-22, implementation patched after analysis.

Related docs:

- `docs/synth-revive-mode.md` — implementation design and transform model
- `docs/issues/synth-azel-residual-divergence.md` — earlier AZEL residual failure
- `processing/dataset/22-4/ver3/` — receiver captures analyzed here

## Research Target

The research goal is to inject GPS PRNs that are **not currently visible in the
real sky** while a u-blox receiver is also receiving live real-sky satellites, and
make the receiver use both groups in one PVT solution.

Receiver-side success criterion:

- Revived PRNs are tracked.
- Revived PRNs have decoded usable ephemeris.
- Revived PRNs are marked `svUsed` in `UBX-NAV-SAT`.
- At least one revived PRN and at least one real-sky PRN are `svUsed` in the same
  epoch.
- The receiver does not switch into an all-synthetic-only solution.

The practical calibration objective is to create mixed real-plus-revived PVT
epochs so tools such as `ubx_bladetx_cal.py` can compare the synthetic group
against real-sky anchors.

## Current Implementation

`SYNTH_REVIVE` is implemented as `-S <prn>:revive`.

The mode uses the target PRN's own RINEX broadcast ephemeris from a past time.
For each target PRN, the scanner searches a lookback window and chooses a past
TOE where that PRN was above the receiver elevation threshold. The transformed
ephemeris is then broadcast at the current transmit time so the receiver should
compute the satellite at the old above-horizon ECEF position.

The intended invariant is:

```text
satpos(revived_ephemeris, t_now) == satpos(template_ephemeris, template_toe)
```

This should avoid the earlier `SYNTH_AZEL` failure where the receiver evaluates
measurements against an orbit that does not match the transmitted code geometry.

## Experiment 22-4/ver3

Transmit command:

```bash
bladetx -l 21.0047844,105.8460541,22 -e hour1120.26n -P 22,14,30 \
  -S 22:revive,14:revive,30:revive \
  --trimble-time-tag-host 192.168.5.245 \
  --trimble-time-tag-port 5017 \
  --trimble-start-offset-sec 2 \
  --txvga1 -35 \
  --trimble-tag-lead-ms 788 \
  --trimble-tx-cal-ns 580000
```

Dataset:

| Receiver | File |
|---|---|
| COM3 | `processing/dataset/22-4/ver3/COM3___9600_260422_082740.ubx` |
| COM4 | `processing/dataset/22-4/ver3/COM4___9600_260422_082748.ubx` |

Injected revived PRNs: `14`, `22`, `30`.

## Field Findings

### COM4 Partially Achieved The Research Target

COM4 produced mixed real-plus-revived PVT epochs:

- `NAV-SAT` epochs: 279
- Epochs with at least one revived PRN `svUsed`: 52
- Epochs with revived and real PRNs both `svUsed`: 52
- Epochs with all three revived PRNs `svUsed`: 0

Per-PRN use:

| PRN | NAV-SAT epochs | `svUsed` epochs | Used fraction | All-row mean abs `prRes` | Used-only median `prRes` |
|---:|---:|---:|---:|---:|---:|
| 14 | 279 | 2 | 0.01 | 1003.8 m | 99.2 m |
| 22 | 279 | 20 | 0.07 | 935.8 m | 11.1 m |
| 30 | 279 | 37 | 0.13 | 854.2 m | 10.8 m |

Interpretation: COM4 can use revived PRNs, but only when the receiver's residual
gate temporarily accepts them. Most revived measurements still have large or
saturated residuals.

### COM3 Did Not Use Revived PRNs In Available NAV-SAT/NAV-SIG Logs

COM3 stayed in 3D fix, but the capture only contains `NAV-SAT` for the first
32 epochs and `NAV-SIG` for the first 21 epochs, while `NAV-PVT` and `RXM-RAWX`
continue for 255 epochs. Within the available NAV-SAT/NAV-SIG window:

- PRN14 `svUsed`: 0
- PRN22 `svUsed`: 0
- PRN30 `svUsed`: 0

Per-PRN residuals in the available COM3 NAV-SAT window:

| PRN | NAV-SAT epochs | `svUsed` epochs | Mean abs `prRes` | Max abs `prRes` | Mean C/N0 |
|---:|---:|---:|---:|---:|---:|
| 14 | 31 | 0 | 1713.9 m | 3276.7 m | 23.4 dB-Hz |
| 22 | 31 | 0 | 1958.0 m | 3276.8 m | 27.8 dB-Hz |
| 30 | 31 | 0 | 3276.7 m | 3276.7 m | 38.7 dB-Hz |

COM3 did track the revived PRNs intermittently in RAWX:

| PRN | RAWX epochs |
|---:|---:|
| 14 | 26 |
| 22 | 41 |
| 30 | 48 |

Interpretation: COM3's early NAV-SAT residuals are too large for use. Because
NAV-SAT/NAV-SIG logging stops early, this capture cannot prove what COM3 did
later in the run.

### Ephemeris Was Available, But Residuals Were Still Large

The revived PRNs were not simply missing ephemeris:

- COM3 reports `ephAvail=1` for the revived PRNs in all available NAV-SAT epochs.
- COM4 reports `ephAvail=1` for 271 of 279 NAV-SAT epochs for each revived PRN.
- `RXM-SFRBX` was decoded for the revived PRNs:
  - COM3: PRN14=3, PRN22=3, PRN30=4 messages
  - COM4: PRN14=7, PRN22=7, PRN30=7 messages

The problem is therefore not only acquisition or ephemeris availability. The main
observed blocker is the receiver residual gate: most revived observations still
look inconsistent with the receiver's predicted pseudorange.

## Root Cause Found In Implementation

The revive transform re-stamps TOE/TOC to the current simulation time, rounded to
the 16-second broadcast quantum. Before the fix, the transform compensated
`Omega0` using:

```text
delta_sec = t_now - template_toe
```

but `toe_new` was actually:

```text
toe_new = round_to_16s(t_now)
```

If `t_now` is not exactly on the 16-second boundary, the receiver evaluates the
ephemeris at:

```text
tk_at_now = t_now - toe_new
```

The old transform assumed `tk_at_now == 0`. A local invariant check showed the
old code created about **3.9 km satellite-position error per second of TOE
mismatch**. For a transmit epoch around 5 seconds away from the rounded TOE, this
can be a kilometers-scale geometry error.

That bug does not fully explain every receiver behavior by itself, because if the
simulator and receiver both used the exact same wrong ephemeris, they would still
be internally consistent. It does explain why the implementation did not satisfy
the documented revive invariant and why the intended past az/el could be wrong by
kilometers.

## Fix Implemented

The transform in `reviveEphemerisFromTemplate()` now compensates the nonzero
`tk_at_now` caused by 16-second TOE rounding:

```text
toe_delta_sec = toe_new - template_toe
tk_at_now     = t_now - toe_new

M0_new    = M0_old - n * tk_at_now
inc0_new  = inc0_old - idot * tk_at_now
Omega0_new = Omega0_old
           + OMEGA_EARTH * toe_delta_sec
           - (OmegaDot - OMEGA_EARTH) * tk_at_now
```

Files changed:

- `gpssim.c` — fixed phase and node compensation for non-boundary `t_now`
- `tests/test_revive_transform.c` — changed the invariant test to use
  `delta_sec = 7205`, which exercises nonzero `tk_at_now`
- `docs/synth-revive-mode.md` — updated the transform derivation

Verification:

```bash
make test
make bladetx
```

Both passed. Existing build warnings remain in `gpssim.c` for unused variables and
unused static helpers.

## Current State

The revive idea is still valid, and COM4 has proven the important qualitative
target: a u-blox receiver can mark both real and revived PRNs as `svUsed` in the
same PVT epoch.

The implementation before the patch had a hard transform bug around TOE
quantization. New experiments should use the patched `bladetx`; old ver3
residuals should not be treated as the final revive-mode result.

The next experiment should determine whether the patched transform reduces revived
PRN residuals enough for both receivers, especially COM3, to use revived PRNs
reliably.

## Next Experiment Requirements

1. Use the rebuilt patched `bladetx`.
2. Keep `NAV-SAT` and `NAV-SIG` enabled for the full COM3 and COM4 run.
3. Record RAWX, SFRBX, NAV-SAT, NAV-SIG, NAV-PVT, NAV-STATUS, NAV-ORB, and SEC-SIG.
4. Reuse a target set like `22,14,30`, or choose candidates with higher revive
   elevation and no current live-sky collision.
5. Analyze:
   - per-PRN `svUsed` count
   - mixed real-plus-revived `svUsed` epochs
   - all-row and used-only `prRes`
   - `ephAvail`, `orbitSource`, `qualityInd`
   - SFRBX subframe coverage for each revived PRN

Success for the next run means COM3 and COM4 both produce sustained epochs where
at least one real PRN and at least one revived PRN are simultaneously `svUsed`,
with revived PRN residuals no longer saturating near `3276.8 m`.
