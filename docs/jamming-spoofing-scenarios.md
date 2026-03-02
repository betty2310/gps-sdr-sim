# GPS Jamming/Spoofing Scenario Catalog

## Purpose

This document defines attack scenario types for GPS-SDR-SIM test campaigns and maps each type to the per-channel parameters that can be controlled.

The `-A` option provides per-PRN method selection in the format:

`PRN:method[,PRN:method...]`

Supported method names:

- `normal`
- `jam_drop`
- `jam_noise`
- `spoof_delay` (planned)
- `spoof_nav` (planned)

Current implementation status:

- `jam_drop` is implemented by forcing selected PRN channel gain to zero.
- `jam_noise` is implemented by replacing selected PRN symbols with deterministic pseudo-random noise-like I/Q values.
- Other methods are placeholders for scenario planning and future implementation.

## Attack Methods

| Method | Channel Intent | Main Parameter Family |
|---|---|---|
| `normal` | No attack on this PRN | none |
| `jam_drop` | Remove satellite contribution from composite IQ | power/gain |
| `jam_noise` | Degrade tracking by adding interference-like energy | power/noise/spectral |
| `spoof_delay` | Shift pseudorange and Doppler coherently | code phase + carrier |
| `spoof_nav` | Mislead by navigation message manipulation | nav data + timing |

## All Practical Scenario Classes

### 1) Baseline and Sanity Scenarios

1. Clean sky reference: all PRNs `normal`.
2. Single PRN off: one PRN `jam_drop`.
3. Dual PRN off: two low-elevation PRNs `jam_drop`.
4. Random PRN drop set: rotating `jam_drop` PRN list every 30 seconds.

### 2) Partial Constellation Jamming Scenarios

1. Low-elevation denial: jam PRNs below a chosen elevation threshold.
2. Geometry attack: jam PRNs that improve HDOP/VDOP the most.
3. Sector attack: jam PRNs in one azimuth sector only.
4. Dynamic denial: increase jammed PRN count over time (1 -> 2 -> 4 -> 6).
5. Intermittent denial: periodic ON/OFF jamming windows to stress tracking loops.

### 3) Power-Domain Jamming Scenarios

1. Soft attenuation: lower attacked PRN gain but not full drop.
2. Hard attenuation: full `jam_drop`.
3. Mixed gain attack: some PRNs dropped, others attenuated.
4. Time-ramped power attack: gradually move from normal power to denial.

### 4) Noise-Like Jamming Scenarios

1. Wideband-like additive interference over L1 baseband.
2. Pseudo-random code jammer with no valid nav bits.
3. Burst-noise attack synchronized to receiver update cycles.
4. Multi-tone/CW-like narrowband interferers.

### 5) Spoofing Delay/Carry-Off Scenarios (Planned)

1. Static range bias: constant pseudorange offset on selected PRNs.
2. Slow carry-off: small ramp in range bias and Doppler bias.
3. Fast carry-off: aggressive ramp to trigger integrity checks.
4. Partial spoofing: only subset of tracked PRNs spoofed.
5. Full spoofing: all tracked PRNs spoofed coherently.

### 6) Navigation Message Spoofing Scenarios (Planned)

1. TOW offset injection.
2. Clock term bias (`af0`) perturbation.
3. Clock drift (`af1`) perturbation.
4. Ephemeris distortion (orbital element perturbations).
5. SV health flag misuse (healthy/unhealthy toggling).
6. Leap-second/UTC parameter inconsistency.

### 7) Hybrid Attack Scenarios

1. Jam-then-spoof takeover: deny selected PRNs, then replace with spoofed channels.
2. Spoof-under-jam cover: spoof while additional PRNs are jammed.
3. Split strategy: one set `jam_drop`, one set `spoof_delay`.
4. Adaptive strategy: attacker method depends on receiver lock status.

### 8) Temporal Pattern Scenarios

1. Step attack: abrupt method changes at fixed times.
2. Ramp attack: smooth transition of attack intensity.
3. Pulsed attack: periodic short bursts.
4. Randomized attack windows: stochastic timing and PRN selection.

### 9) Receiver Stress Scenarios

1. Acquisition stress: start with attack active from t=0.
2. Tracking stress: start attack after stable lock.
3. Reacquisition stress: alternate drop/recover cycles.
4. Edge-of-fix stress: keep exactly 4 usable PRNs.

### 10) Validation/Measurement Scenarios

1. C/N0 degradation mapping vs jammed PRN count.
2. Position error growth vs spoof ramp slope.
3. Time-to-alarm and integrity flag triggering.
4. Recovery time after attack stop.

## Example `-A` Inputs

```bash
# Two PRNs dropped (implemented today)
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,11:jam_drop

# Mixed catalog (jam_drop + jam_noise active today)
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,7:jam_noise,11:spoof_nav
```

## Recommended Next Implementation Order

1. `spoof_delay`: per-PRN range/range-rate bias injection before `computeCodePhase`.
2. `spoof_nav`: per-PRN nav-bit and ephemeris field perturbation path.
