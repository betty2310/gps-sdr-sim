# GPS Jamming/Spoofing Scenario Catalog

## Purpose

This document defines attack scenario types for GPS-SDR-SIM test campaigns and maps each type to the per-channel parameters that can be controlled.

Two main interfaces control attack behavior:

1. **`-P <prn_list>`** — Partial constellation mode. Only the listed PRNs are rendered in the IQ output; all other visible satellites are excluded. The attack method for each PRN is controlled separately via `-A`.

2. **`-A <spec>`** — Per-PRN method selection in the format `PRN:method[,PRN:method...]` for fine-grained control.

Supported method names:

- `normal`
- `jam_drop`
- `jam_noise`
- `spoof_delay` (planned)
- `spoof_nav` (planned)

Additional options:

- **`-J <dB>`** — Jammer-to-signal ratio for `jam_noise` (default: 20 dB)
- **`-G <dB>`** — Power boost for partial-mode PRNs (default: 0 dB)

Current implementation status:

- `jam_drop` is implemented by forcing selected PRN channel gain to zero.
- `jam_noise` is implemented as **matched-code interference** — noise modulated with the target PRN's C/A code and carrier, ensuring interference concentrates on the target correlator while being rejected (~30 dB processing gain) by all other PRNs. See [Noise Generation Model](#noise-generation-model) below.
- `-P` partial constellation mode is implemented. It filters `allocateChannel()` so only selected PRNs enter the output. Use `-A` to assign an attack method to each.
- Other methods are placeholders for scenario planning and future implementation.

## Attack Methods

| Method        | Channel Intent                                              | Main Parameter Family |
| ------------- | ----------------------------------------------------------- | --------------------- |
| `normal`      | No attack on this PRN                                       | none                  |
| `jam_drop`    | Remove satellite contribution from composite IQ             | power/gain            |
| `jam_noise`   | Matched-code interference targeting specific PRN correlator | power/J/S ratio       |
| `spoof_delay` | Shift pseudorange and Doppler coherently                    | code phase + carrier  |
| `spoof_nav`   | Mislead by navigation message manipulation                  | nav data + timing     |

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

## Partial Constellation Mode (`-P`)

### Overview

The `-P` flag enables **targeted OTA jamming** of a subset of the GPS constellation. Only the listed PRNs are rendered in the IQ output file; all other visible satellites are excluded at the channel allocation stage (they consume no channel slots and contribute nothing to the output).

When `-P` is used, only the listed PRNs are rendered. The attack method for each PRN is set separately via `-A`. The intended OTA workflow is:

```
Real Sky (all satellites) ──────────────────────────► Receiver antenna
                                                           ▲
gps-sdr-sim -P 5,14,21 ──► USRP TX (matched-code noise) ──┘
```

The receiver tracks the remaining constellation normally from the sky, while the targeted PRNs are jammed by the transmitted matched-code interference.

### How It Works

1. **Channel filter:** `allocateChannel()` skips any PRN not in the `-P` list, even if visible. No channel slot is wasted.
2. **Method via `-A`:** Use `-A` to assign any attack method (e.g., `jam_noise`, `spoof_delay`) to the selected PRNs. Without `-A`, selected PRNs generate normal signals.
3. **Power boost (`-G`):** Optional dB boost applied to all selected PRNs on top of the natural path-loss/antenna-gain model.

### Why PRN Filtering Is Necessary for OTA

Without `-P`, the IQ output contains valid signals for **all** visible satellites. When transmitted via SDR alongside real sky signals, the receiver sees **double signals** for every non-targeted satellite (one from sky + one from USRP), corrupting tracking across the entire constellation.

## Noise Generation Model

### Overview

The `jam_noise` method models a **per-PRN matched-code interference jammer**. The noise is modulated with the target PRN's C/A code and carrier phase, so that after despreading in the receiver the interference concentrates on the target PRN's correlator while being rejected by all other PRNs (~30 dB processing gain from the 1023-chip C/A code).

No valid navigation data is transmitted — only noise-modulated spreading code. This denies the target PRN without providing any usable signal the receiver could track.

### Signal Model

For a normal (unjammed) PRN, each I/Q sample is:

```
I = dataBit × codeCA × cos(carrier_phase) × gain
Q = dataBit × codeCA × sin(carrier_phase) × gain
```

For a `jam_noise` PRN, the output is **replaced** with matched-code noise (no valid signal component):

```
I = gaussianNoise × codeCA × cos(carrier_phase) × noise_amp
Q = gaussianNoise × codeCA × sin(carrier_phase) × noise_amp
```

Where `noise_amp = gain × js_linear` and `js_linear = 10^(J/S_dB / 20)`.

### Why Matched-Code, Not Broadband

GPS receivers despread incoming signals by correlating with each PRN's known C/A code. This provides ~30 dB of processing gain (10 × log10(1023)):

| Noise Type                         | Effect on target PRN               | Effect on other PRNs                                   |
| ---------------------------------- | ---------------------------------- | ------------------------------------------------------ |
| **Broadband** (old implementation) | Attenuated ~30 dB by despreading   | Attenuated ~30 dB equally — jams everything or nothing |
| **Matched-code** (current)         | Full noise power after despreading | Attenuated ~30 dB — effectively invisible              |

Matched-code noise is the only way to achieve **selective** jamming when transmitting OTA alongside real sky signals.

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

| J/S (dB) | Linear amplitude | Effect                                            |
| -------- | ---------------- | ------------------------------------------------- |
| 0        | 1.0              | Noise power equals signal — marginal tracking     |
| 10       | 3.16             | Noise ~10× signal power — likely loss of lock     |
| 20       | 10.0             | Noise ~100× signal power — total denial (default) |
| 40       | 100.0            | Extreme jamming scenario                          |

The noise amplitude is scaled relative to each PRN's own `gain` value (which includes path loss and antenna pattern), so the J/S ratio is consistent regardless of satellite elevation or distance.

### Design Rationale

- **Matched-code, not broadband:** Noise is modulated with the target C/A code and carrier, concentrating interference on the target correlator after despreading. This enables selective jamming of individual PRNs when transmitting OTA.
- **No valid signal component:** Only noise is transmitted for jammed PRNs. Including a valid signal would help the receiver track (counterproductive for jamming).
- **Per-PRN scoping:** Enables partial constellation jamming scenarios (e.g., jam low-elevation PRNs only) without affecting unjammed satellites.
- **Deterministic:** Seeded PRNG ensures identical output for the same parameters, enabling repeatable test campaigns.

## Examples

### Partial Constellation Targeted Jamming (`-P` + `-A`)

Start low jammer strength and ramp slowly (-J 0 -> 3 -> 6 -> 10 -> 15 -> 20).

```bash
# Render only PRNs 5 and 14 with matched-code noise jamming
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -P 5,14 -A 5:jam_noise,14:jam_nois2.2.66e

# Same with custom J/S = 30 dB
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -P 5,14,21 -A 5:jam_noise,14:jam_noise,21:jam_noise -J 3

# With +5 dB power boost to ensure jamming overcomes real sky signal
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -P 5,14,21 -A 5:jam_noise,14:jam_noise,21:jam_noise -G 5

# Render only selected PRNs with normal signal (no attack) — useful for future spoofing
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -P 5,14,21
```

### Per-PRN Attack Config (`-A`)

```bash
# Two PRNs dropped (gain zeroed — for offline/simulation use)
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,11:jam_drop

# Matched-code noise jamming on PRN 7 with default J/S = 20 dB
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 7:jam_noise

# Mixed catalog (jam_drop + jam_noise)
./gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,7:jam_noise
```

## Recommended Next Implementation Order

1. `spoof_delay`: per-PRN range/range-rate bias injection before `computeCodePhase`.
2. `spoof_nav`: per-PRN nav-bit and ephemeris field perturbation path.
