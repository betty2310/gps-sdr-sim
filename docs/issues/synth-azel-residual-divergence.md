# Synthetic AZEL Orbit Model Causes Large Pseudorange Residuals

## Status

Open — 2026-04-09

## Summary

When using `SYNTH_AZEL` mode to inject synthetic PRNs into a real-sky environment,
the u-blox ZED-F9P reports pseudorange residuals of ~1000 m (clamped at 3276.8 m)
for the synthetic satellites. Two receivers exhibit opposite behavior: the higher-
quality receiver rejects all synthetic PRNs from navigation, while the lower-quality
receiver adopts the synthetic constellation and rejects all real-sky PRNs.

The root cause is **not** a gross timing alignment problem (the Trimble time-tag
system reduced that from ~233 ms to the microsecond range). The dominant cause is
a **geometric and ephemeris mismatch**: the receiver evaluates synthetic pseudoranges
against real cached ephemeris data, producing range residuals that reflect the
physical distance between the synthetic satellite position (at user-specified az/el)
and the real orbital position.

## Test Setup — Dataset 9 / ver2

### Transmit command

```bash
bladetx -e hour0990.26n -l 21.0047844,105.8460541,5 \
  -P 4,5,10,16,11,12,15 \
  -S 4:20/60,5:90/45,10:60/30,16:45/55,11:30/45,12:27/35,15:48/54 \
  --trimble-time-tag-host 192.168.5.245 \
  --trimble-time-tag-port 5017 \
  --trimble-tag-lead-ms 790 \
  --trimble-start-offset-sec 2 \
  --txvga1 -35
```

### Key parameters

| Parameter       | Value                                                     |
| --------------- | --------------------------------------------------------- |
| Ephemeris       | hour0990.26n                                              |
| Target position | 21.0047844, 105.8460541, 5 m                              |
| Injected PRNs   | 4, 5, 10, 11, 12, 15, 16                                  |
| Mode            | SYNTH_AZEL (custom azimuth/elevation per PRN)             |
| Timing          | Trimble time-tag, lead=790 ms, start-offset=2 s, cal=0 ns |
| TX power        | txvga1=-35 dB                                             |
| Run duration    | ~766 s before bladeRF TX error                            |

### Receiver captures

| File                            | Receiver                  | Role                               |
| ------------------------------- | ------------------------- | ---------------------------------- |
| `COM3___9600_260409_023906.ubx` | Higher-quality u-blox F9P | Rejects synthetic, uses real sky   |
| `COM4___9600_260409_024636.ubx` | Lower-quality u-blox F9P  | Adopts synthetic, rejects real sky |

Both receivers at the same antenna location, receiving both real sky and synthetic
signals simultaneously.

## Findings

### COM3 — Good receiver rejects synthetic PRNs

From `ubx_bladetx_cal.py`:

```
epochs: RAWX=391 NAV-SAT=426 NAV-STATUS=426 NAV-PVT=426
spoof states: no_spoof=341, spoof_indicated=85
fix states: 3D=426
real anchor PRNs used: 1,2,7,14,17,19,22,30
```

All 7 injected PRNs are seen in NAV-SAT (426 epochs each) but **never svUsed**.
RAWX tracking is intermittent (36-109 epochs out of 391) — the receiver acquires
the synthetic signal briefly then drops it.

NAV-SAT residuals for the unused synthetic PRNs (from notebook analysis):

| Satellite | Epochs | Mean prRes (m) | Mean   | prRes  | (m) | Max | prRes | (m) | svUsed epochs |
| --------- | ------ | -------------- | ------ | ------ | --- | --- | ----- | --- | ------------- |
| GPS04     | 426    | -1104.8        | 1212.5 | 3276.8 | 0   |
| GPS05     | 426    | -1001.4        | 1155.7 | 3276.8 | 0   |
| GPS10     | 426    | -1010.4        | 1125.2 | 3276.8 | 0   |
| GPS11     | 426    | -969.3         | 1023.8 | 3276.8 | 0   |
| GPS12     | 426    | -1104.1        | 1204.4 | 3276.8 | 0   |
| GPS15     | 426    | -1011.2        | 1112.0 | 3276.8 | 0   |
| GPS16     | 426    | -1053.8        | 1122.0 | 3276.8 | 0   |

All mean residuals cluster around -1000 m with max values hitting the I16 saturation
limit of 3276.8 m (= 32768 \* 0.1 m).

### COM4 — Basic receiver adopts synthetic PRNs

From `ubx_bladetx_cal.py`:

```
epochs: RAWX=167 NAV-SAT=168 NAV-STATUS=168 NAV-PVT=167
spoof states: multi_spoof=168
fix states: 3D=167
real anchor PRNs used: none
```

All 7 injected PRNs svUsed in all epochs. No real-sky PRN is ever used.
Spoof detection permanently flagged as `multi_spoof`.

Post-fit residuals for used synthetic PRNs (median from cal tool):

| PRN   | Median prRes (m) | Notes                    |
| ----- | ---------------- | ------------------------ |
| GPS04 | +6.9             | small                    |
| GPS05 | -7.2             | small                    |
| GPS10 | -5.0             | small                    |
| GPS11 | -16.0            | moderate                 |
| GPS12 | -0.3             | excellent                |
| GPS15 | +31.7            | large, growing over time |
| GPS16 | -54.3            | large, growing over time |

These are post-fit residuals (PVT absorbs common-mode clock bias). The residuals
diverge over time from near-zero to +/-60 m over ~175 seconds, indicating the
synthetic orbit model drifts from the receiver's internal propagation.

### Calibration tool result

The `ubx_bladetx_cal.py` tool **cannot derive a timing calibration** from either
capture because:

- COM3: no injected PRN was ever svUsed (no injected group in PVT)
- COM4: no real PRN was ever svUsed (no real anchor group in PVT)

The tool requires both groups used simultaneously in the same PVT solution.

## Root Cause Analysis

### How prRes is computed

For each tracked satellite, the receiver computes:

```
prRes = rho_measured - rho_predicted
```

Where:

- `rho_measured` = pseudorange from tracking loop (code phase \* c)
- `rho_predicted` = |pos_sat - pos_rx| + c*dt_rx - c*dt_sat + iono + tropo
  - `pos_sat` from decoded/cached ephemeris
  - `pos_rx`, `dt_rx` from current PVT solution
  - `dt_sat` from ephemeris clock parameters (af0, af1, af2)

The prRes field is signed I16 scaled by 0.1 m, so it saturates at +/-3276.8 m.

### Why COM3 synthetic residuals are ~1000 m

Three factors contribute, in order of magnitude:

**1. Ephemeris source conflict (dominant, ~hundreds to thousands of meters)**

The receiver caches the **real broadcast ephemeris** for each PRN from the live-sky
signal. When the synthetic signal arrives with a SYNTH_AZEL ephemeris (IODE=1,
ecc=0, all perturbation terms zero, af0=0), the receiver likely:

- Retains the already-validated real ephemeris
- Rejects the anomalous synthetic ephemeris (or never fully decodes it from the
  intermittently-tracked signal)

Result: `rho_predicted` uses the **real satellite position** (from real ephemeris),
while `rho_measured` reflects the pseudorange from the **synthetic position** (at the
user-specified az/el). The slant-range difference between these two positions on the
GPS orbit sphere drives the residual.

Example: real GPS04 might be at az=280 el=40, synthetic is at az=20 el=60. At GPS
orbit altitude (~26,560 km radius), different positions on the orbit sphere produce
range differences of hundreds to thousands of meters depending on geometry.

**2. Satellite clock correction mismatch (~10-30 m)**

`synthEphemeris()` sets `af0 = af1 = af2 = 0` (zero clock correction). The real
satellite has non-zero clock corrections (typically tens of nanoseconds = ~10 m in
pseudorange). If the receiver uses the real ephemeris clock model, this creates an
additional per-satellite offset.

**3. Atmospheric correction mismatch (~5-15 m)**

The receiver applies Klobuchar ionospheric and tropospheric model corrections based
on the satellite's elevation angle. The synthetic signal carries no real atmospheric
delay, but the receiver still subtracts model corrections from `rho_predicted`. This
produces an elevation-dependent bias.

### Why COM4 post-fit residuals diverge over time

Since all synthetic PRNs are used in PVT, the common-mode clock offset is absorbed
by the receiver clock estimate. The remaining per-satellite residuals come from:

1. **Orbit model drift**: The synthetic circular orbit (ecc=0, zero perturbations,
   `omgdot=-8e-9`) is a first-order approximation. The receiver propagates the
   decoded synthetic ephemeris forward from TOE using its own orbit model. Small
   numerical differences between the simulator's `satpos()` and the F9P firmware's
   orbit propagator accumulate over time.

2. **Stale TOE**: The synthetic ephemeris TOE is set once at scenario start
   (`synth_ref = g0`). As time progresses, the extrapolation from TOE becomes
   less accurate. GPS ephemerides are normally refreshed every 2 hours.

3. **Missing perturbation terms**: Real GPS orbits have non-zero cuc, cus, cic, cis,
   crc, crs corrections. Setting all to zero removes ~1-10 m of periodic orbital
   variation that the receiver's model cannot recover.

## Follow-On Fix

The mitigation path is `docs/synth-clone-mode.md`: transmit synthetic target
PRNs using live broadcast ephemeris copied from donor PRNs carried in RTCM `1019`.
In the measured lab setup on 2026-04-16, those `1019` messages were available on
the NAVIS NTRIP source at `192.168.5.245:5005` (`mount=NAVIS`), while the raw
`5018` stream exposed MSM/station messages but not GPS ephemeris.
