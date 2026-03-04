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
- `jam_noise` is implemented as additive interference on top of the real satellite signal with configurable J/S ratio (see [Noise Generation Model](#noise-generation-model) below).
- Other methods are placeholders for scenario planning and future implementation.

## Attack Methods

| Method | Channel Intent | Main Parameter Family |
|---|---|---|
| `normal` | No attack on this PRN | none |
| `jam_drop` | Remove satellite contribution from composite IQ | power/gain |
| `jam_noise` | Additive interference that buries satellite signal | power/J/S ratio |
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

## Noise Generation Model

### Overview

The `jam_noise` method models a **per-PRN additive interference jammer**. Unlike the previous implementation which replaced the satellite signal with noise, the current model keeps the real satellite signal intact and adds interference on top of it. This simulates a directional or matched-code jammer that selectively degrades targeted PRNs while leaving others unaffected.

### Signal Model

For a normal (unjammed) PRN, each I/Q sample is:

```
I = dataBit × codeCA × cos(carrier_phase) × gain
Q = dataBit × codeCA × sin(carrier_phase) × gain
```

For a `jam_noise` PRN, the interference is added:

```
I = signal_I + noise_I × (gain × js_linear)
Q = signal_Q + noise_Q × (gain × js_linear)
```

Where `js_linear = 10^(J/S_dB / 20)` converts the user-specified J/S ratio from dB to a linear amplitude scale relative to the satellite signal.

### Noise Distribution

The interference samples are generated using an **approximate Gaussian distribution** via the Central Limit Theorem (CLT):

1. **Base PRNG:** A xorshift32 generator (`nextNoiseValue()`) produces uniform integers in [-250, +250]. Each PRN has an independent PRNG state seeded deterministically from the PRN number, ensuring reproducible results.

2. **CLT approximation:** `nextGaussianNoise()` sums 4 independent uniform samples and divides by 4. By the CLT, the sum of independent uniform random variables converges toward a Gaussian distribution. With 4 terms the result has:
   - Mean: 0
   - Reduced tail probability compared to uniform (more realistic for thermal/broadband noise)
   - Output range: [-250, +250] (integer)

3. **Why not true Gaussian?** Box-Muller or Ziggurat methods require floating-point math and `log()`/`sin()` calls per sample at MHz rates. The CLT sum of 4 uniforms is fast (integer-only), deterministic, and sufficient for baseband interference simulation.

### J/S Ratio Parameter

The `-J <dB>` option controls the jammer-to-signal amplitude ratio:

| J/S (dB) | Linear amplitude | Effect |
|---|---|---|
| 0 | 1.0 | Noise power equals signal — marginal tracking |
| 10 | 3.16 | Noise ~10× signal power — likely loss of lock |
| 20 | 10.0 | Noise ~100× signal power — total denial (default) |
| 40 | 100.0 | Extreme jamming scenario |

The noise amplitude is scaled relative to each PRN's own `gain` value (which includes path loss and antenna pattern), so the J/S ratio is consistent regardless of satellite elevation or distance.

### Design Rationale

- **Additive, not substitutive:** Real jammers transmit energy; they cannot subtract a satellite signal. The real signal is always present underneath, matching physical reality.
- **Per-PRN scoping:** Enables partial constellation jamming scenarios (e.g., jam low-elevation PRNs only) without affecting unjammed satellites. This models directional or structured jammers.
- **Deterministic:** Seeded PRNG ensures identical output for the same parameters, enabling repeatable test campaigns.

## Example `-A` Inputs

```bash
# Two PRNs dropped
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,11:jam_drop

# Noise jamming on PRN 7 with default J/S = 20 dB
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 7:jam_noise

# Noise jamming with custom J/S = 30 dB
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_noise,7:jam_noise -J 30

# Mixed catalog (jam_drop + jam_noise)
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,7:jam_noise,11:spoof_nav
```

## Recommended Next Implementation Order

1. `spoof_delay`: per-PRN range/range-rate bias injection before `computeCodePhase`.
2. `spoof_nav`: per-PRN nav-bit and ephemeris field perturbation path.
