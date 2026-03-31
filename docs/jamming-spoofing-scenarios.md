# GPS Jamming/Spoofing Scenario Catalog

## Hardware Assumptions

All configurations in this document assume:

- **SDR:** bladeRF 1.0 (x40/x115), LMS6002D, minimum TX gain (txvga1=-35, txvga2=0, overall=17 dB)
- **TX output power at minimum gain:** ~-10 dBm
- **TX antenna:** Quarter-wave whip (~48 mm), 0 dBi, hanging downward above receiver
- **Receiver:** u-blox ZED-F9P with multi-band L1+L2 active patch antenna, facing up
- **RX antenna gain at boresight (TX directly above):** +5 dBi
- **Polarization loss (linear TX to RHCP RX):** 3 dB

**Verify your hardware:** Run `set gain tx1 0` in bladeRF-cli. If you see `txvga1`/`txvga2` in the output, you have a bladeRF 1.0. The bladeRF 2.0 micro (AD9361) has a different gain model and lower minimum output (~-44 dBm at gain -24).

---

## Interfaces

Two main interfaces control attack behavior:

1. **`-P <prn_list>`** -- Partial constellation mode. Only the listed PRNs are rendered in the IQ output; all other visible satellites are excluded. **Mandatory for OTA** to avoid doubling real sky signals.

2. **`-A <spec>`** -- Per-PRN method selection: `PRN:method[,PRN:method...]`.

Supported methods:

| Method | Effect | Status |
|--------|--------|--------|
| `normal` | No attack, clean signal | Implemented |
| `jam_drop` | Zero the PRN's gain (silent denial) | Implemented |
| `jam_noise` | Matched-code interference on target correlator | Implemented |
| `spoof_delay` | Pseudorange/Doppler bias injection | Planned |
| `spoof_nav` | Navigation message manipulation | Planned |

Additional options:

| Flag | Default | Purpose |
|------|---------|---------|
| `-J <dB>` | 20 | Jammer-to-signal ratio for `jam_noise` |
| `-G <dB>` | 0 | Power boost for partial-mode PRNs |

---

## OTA Link Budget: bladeRF 1.0

### Constants

```
P_tx   = -10 dBm  (bladeRF 1.0, txvga1=-35, txvga2=0)
G_tx   =   0 dBi  (whip)
G_rx   =  +5 dBi  (F9P patch at boresight, TX directly above)
Pol    =   3 dB   (linear to RHCP)

P_rx = -10 - Atten - FSPL + 0 + 5 - 3 = -8 - Atten - FSPL
```

### Free-space path loss at 1575.42 MHz

| Height | FSPL |
|--------|------|
| 0.5 m | 30 dB |
| 1.0 m | 36 dB |
| 1.5 m | 40 dB |

### Target power at receiver

Real GPS at ground: **-130 dBm**. ZED-F9P spoofing detector compares C/N0 across all tracked PRNs.

| Target | P_rx | Margin above real GPS | F9P reaction |
|--------|------|-----------------------|-------------|
| Too weak | < -133 dBm | < -3 dB | Not acquired or unstable |
| Ideal | -127 to -124 dBm | +3 to +6 dB | Accepted, spoofDet=1 |
| Marginal | -120 to -115 dBm | +10 to +15 dB | May trigger spoofDet=2 |
| Too strong | > -110 dBm | > +20 dB | spoofDet=2/3, AGC issues |

### Attenuation table by height

| Height | FSPL | Atten for -124 dBm | Atten for -127 dBm | Atten for -130 dBm |
|--------|------|---------------------|---------------------|---------------------|
| **0.5 m** | 30 dB | **86 dB** | **89 dB** | **92 dB** |
| **1.0 m** | 36 dB | **80 dB** | **83 dB** | **86 dB** |
| **1.5 m** | 40 dB | **76 dB** | **79 dB** | **82 dB** |

### Practical attenuator combinations

SMA fixed attenuators (Mini-Circuits VAT series or equivalent):

| Target atten | Combination | P_rx at 0.5 m | P_rx at 1.0 m | P_rx at 1.5 m |
|-------------|-------------|----------------|----------------|----------------|
| 60 dB | 3x VAT-20+ | -98 dBm | -104 dBm | -108 dBm |
| 70 dB | 3x VAT-20+ + VAT-10+ | -108 dBm | -114 dBm | -118 dBm |
| 76 dB | 3x VAT-20+ + VAT-10+ + VAT-6+ | -114 dBm | -120 dBm | **-124 dBm** |
| 80 dB | 4x VAT-20+ | -118 dBm | **-124 dBm** | -128 dBm |
| 83 dB | 4x VAT-20+ + VAT-3+ | -121 dBm | **-127 dBm** | -131 dBm |
| 86 dB | 4x VAT-20+ + VAT-6+ | **-124 dBm** | -130 dBm | -134 dBm |
| 90 dB | 4x VAT-20+ + VAT-10+ | -128 dBm | -134 dBm | -138 dBm |

**Recommended starting points (bold = ideal range for F9P):**

| Height | Attenuator chain | Expected P_rx | Margin vs real GPS |
|--------|-----------------|---------------|-------------------|
| **0.5 m** | 4x VAT-20+ + VAT-6+ (86 dB) | -124 dBm | +6 dB |
| **1.0 m** | 4x VAT-20+ (80 dB) | -124 dBm | +6 dB |
| **1.5 m** | 3x VAT-20+ + VAT-10+ + VAT-6+ (76 dB) | -124 dBm | +6 dB |

Fine-tune by swapping/adding VAT-3+ or VAT-6+ attenuators while monitoring C/N0 in u-center.

---

## bladerf.script (bladeRF 1.0)

```
set frequency tx1 1575420000
set samplerate tx1 2600000
set bandwidth tx1 2500000
set txvga1 -35
set txvga2 0
cal lms
cal dc tx
tx config file=gpssim.bin format=bin repeat=0
tx start
tx wait
```

**Critical notes:**
- `set gain tx1 -10` is silently clamped to 17 dB on bladeRF 1.0. Always use `set txvga1`/`set txvga2` directly.
- `cal lms` and `cal dc tx` calibrate the LMS6002D. Without them, LO leakage and DC offset are worse.
- `samplerate tx1` **must** match gps-sdr-sim output (default 2.6 MHz). A mismatch breaks the C/A code rate.

---

## LO Leakage Warning (bladeRF 1.0)

The LMS6002D leaks local oscillator energy through the TX port **even when not transmitting**. If the frequency was previously set to 1575.42 MHz, this leakage can degrade u-blox tracking across all satellites (5-15 dB C/N0 drop).

**Mitigations:**
1. Keep attenuators in the RF chain at all times
2. Cap the TX SMA with a 50-ohm terminator when not transmitting
3. Unplug the bladeRF during baseline GPS measurements
4. Run `cal dc tx` before each session

---

## Scenario Configurations

### Preparation (all scenarios)

```bash
# 1. Download current ephemeris
bash download_nasa_ephemeris.sh

# 2. Identify visible PRNs at your location
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT -d 10 -v -o /dev/null

# 3. Note visible PRNs. For jam_noise OTA, choose PRNs FROM this list.
#    For synthetic satellite injection (-S), choose PRNs NOT in this list.
```

### 1. Targeted PRN Jamming (jam_noise, OTA)

Jam specific real-sky satellites with matched-code interference while the receiver tracks the rest normally.

**Pick target PRNs from the visible list.** These are real satellites the receiver is tracking; the bladeRF transmits interference that denies them.

#### 1a. Single PRN jam at 1.0 m height

```bash
# Generate matched-code noise for PRN 5
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -P 5 -A 5:jam_noise -J 20 -G 0 \
    -d 300 -v -o gpssim.bin
```

```
RF chain: bladeRF TX --> [20dB] --> [20dB] --> [20dB] --> [20dB] --> whip (1.0 m above F9P)
                         -------------------- 80 dB --------------------
Expected P_rx: -124 dBm (+6 dB above real GPS)
```

**Expected receiver behavior:**
- PRN 5 C/N0 drops from ~40 dB-Hz to <20 dB-Hz, then lost
- All other PRNs unaffected (matched-code noise rejected by ~30 dB processing gain)
- Position solution degrades slightly (one fewer satellite)

#### 1b. Multi-PRN jam at 0.5 m height

```bash
# Jam PRNs 5, 14, 21 with matched-code noise
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -P 5,14,21 -A 5:jam_noise,14:jam_noise,21:jam_noise -J 20 -G 0 \
    -d 300 -v -o gpssim.bin
```

```
RF chain: bladeRF TX --> [20dB] --> [20dB] --> [20dB] --> [20dB] --> [6dB] --> whip (0.5 m above F9P)
                         ----------------------- 86 dB -----------------------
Expected P_rx: -124 dBm (+6 dB above real GPS)
```

#### 1c. Geometry attack at 1.5 m height

Jam PRNs that contribute most to vertical dilution of precision (VDOP) -- typically high-elevation satellites.

```bash
# Jam two high-elevation PRNs to degrade VDOP
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -P 8,22 -A 8:jam_noise,22:jam_noise -J 20 -G 0 \
    -d 300 -v -o gpssim.bin
```

```
RF chain: bladeRF TX --> [20dB] --> [20dB] --> [20dB] --> [10dB] --> [6dB] --> whip (1.5 m above F9P)
                         ----------------------- 76 dB -----------------------
Expected P_rx: -124 dBm (+6 dB above real GPS)
```

#### 1d. J/S ratio sweep (any height)

Start low and increase to find the ZED-F9P's denial threshold:

```bash
# Generate multiple files at different J/S levels
for JS in 0 3 6 10 15 20 30; do
    ./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
        -P 5 -A 5:jam_noise -J $JS -G 0 \
        -d 120 -v -o gpssim_j${JS}.bin
done
```

| J/S (dB) | Noise amplitude | Expected effect on target PRN |
|-----------|----------------|-------------------------------|
| 0 | x1.0 | C/N0 drops ~3 dB, marginal tracking |
| 3 | x1.4 | C/N0 drops ~6 dB, unstable |
| 6 | x2.0 | Tracking intermittent |
| 10 | x3.2 | Likely loss of lock |
| 15 | x5.6 | Denial probable |
| **20** | **x10.0** | **Total denial (default)** |
| 30 | x31.6 | Overkill but useful for margin testing |

### 2. Synthetic Satellite Injection (-S, OTA)

Inject fake GPS satellites that the receiver acquires alongside real sky signals.

**Pick synthetic PRNs NOT in the visible list.** These are satellites the receiver cannot see from the sky; the bladeRF provides them.

#### 2a. Two synthetic satellites at 1.0 m height

```bash
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v -o gpssim.bin
```

```
RF chain: bladeRF TX --> [20dB] --> [20dB] --> [20dB] --> [20dB] --> whip (1.0 m above F9P)
                         -------------------- 80 dB --------------------
Expected P_rx: -124 dBm (+6 dB above real GPS)
```

**Expected receiver behavior:**
- PRN 25 appears in UBX-NAV-SAT with L1 C/A signal, no L2 (gps-sdr-sim is L1-only)
- PRN 26 appears similarly
- `qualityInd` = 4 for synthetic (L1 only) vs 7 for real sky (L1+L2)
- `Used=Yes` if `-l` matches actual receiver location
- `spoofDetState` should be 1 (no spoofing) if C/N0 within +/-3 dB of real satellites

#### 2b. Four synthetic satellites for full position fix (conducted test)

For cable-connected test without sky signals. Needs 4+ satellites for a fix.

```bash
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0,27:90.0/30.0,29:270.0/60.0 \
    -P 25,26,27,29 \
    -p \
    -d 300 -v -o gpssim.bin
```

```
RF chain (conducted): bladeRF TX --> DC Block --> [20dB] --> [20dB] --> [20dB] --> [20dB] --> ZED-F9P RF IN
                                                  -------------------- 80 dB --------------------
```

**Note:** DC block prevents F9P active antenna bias from reaching bladeRF TX. The `-p` flag disables path loss modeling (appropriate for conducted, not for OTA).

### 3. Hybrid: Jam Real + Inject Synthetic

Deny selected real PRNs while injecting synthetic replacements. This is a jam-then-spoof scenario.

```bash
# Jam real PRN 5, inject synthetic PRN 25
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead \
    -P 5,25 -A 5:jam_noise \
    -J 20 -G 3 \
    -d 300 -v -o gpssim.bin
```

```
RF chain (1.0 m): bladeRF TX --> [20dB] --> [20dB] --> [20dB] --> [20dB] --> whip
                                  -------------------- 80 dB --------------------
```

PRN 5 is denied by matched-code noise. PRN 25 (synthetic) takes its constellation slot. The receiver's position solution shifts because one real measurement was replaced with a controlled one.

### 4. Power-Domain Scenarios

#### 4a. Soft attenuation via `-G`

Instead of full denial, reduce the effective signal strength of targeted PRNs.

```bash
# PRN 5 with -10 dB power (signal weakened but not denied)
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -P 5 -A 5:normal -G -10 \
    -d 300 -v -o gpssim.bin
```

#### 4b. Time-ramped J/S (manual)

Generate separate IQ files with increasing J/S and play sequentially:

```bash
for JS in 0 6 12 20; do
    ./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
        -P 5 -A 5:jam_noise -J $JS \
        -d 60 -v -o gpssim_ramp_j${JS}.bin
done
# Concatenate: cat gpssim_ramp_j*.bin > gpssim_ramp.bin
```

### 5. Baseline / Sanity Checks

Run these first to establish reference measurements.

#### 5a. Clean sky reference (no bladeRF transmission)

```bash
# Record 5 minutes of UBX-NAV-SAT, UBX-NAV-PVT, UBX-MON-RF in u-center 2
# This is your baseline C/N0, satellite count, and position accuracy
# IMPORTANT: unplug or cap bladeRF to avoid LO leakage interference
```

#### 5b. Partial constellation, no attack (verify -P isolation)

```bash
# Render PRN 5 with normal signal only -- verify other PRNs are not affected
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -P 5 -A 5:normal -G 0 \
    -d 120 -v -o gpssim.bin
```

Transmit via bladeRF. All PRNs except 5 should show identical C/N0 to baseline. PRN 5 may show slight C/N0 change from the doubled signal.

---

## Measurement Checklist

For each scenario, record in u-center 2:

| Measurement | UBX Message | What to look for |
|-------------|-------------|------------------|
| Per-PRN C/N0 | UBX-NAV-SAT | Target vs non-target PRN comparison |
| Per-signal L1/L2 status | UBX-NAV-SIG | Synthetic PRNs: L1 tracked, L2 absent |
| Position solution | UBX-NAV-PVT | Accuracy before/during/after attack |
| Spoofing detection | UBX-NAV-STATUS | `spoofDetState`: 1=clean, 2=flagged, 3=multiple flags |
| RF front-end status | UBX-MON-RF | Per-block AGC, noise floor, jamming indicator |
| Raw pseudorange | UBX-RXM-RAWX | Carrier phase, Doppler for research analysis |
| Quality indicator | UBX-NAV-SAT | `qualityInd`: 4=L1 only (synthetic), 7=L1+L2 (real) |

**ZED-F9P spoofing detection states:**

| `spoofDetState` | Meaning | Action |
|-----------------|---------|--------|
| 0 | Deactivated | Check F9P firmware supports spoofing detection |
| **1** | **No spoofing** | **Success** |
| 2 | Spoofing indicated | Reduce TX power (add attenuation), verify `-l` matches location |
| 3 | Multiple indications | Significant mismatch -- recheck entire RF chain and position |

---

## Gain Tuning by C/N0 Observation

After transmitting, monitor synthetic/jammed PRN C/N0 in UBX-NAV-SAT:

| Observed C/N0 (dB-Hz) | Meaning | Action |
|------------------------|---------|--------|
| 0 (not acquired) | Signal too weak | Remove one VAT-20+ or swap for VAT-10+ |
| < 25 | Weak, unstable | Remove VAT-3+ or VAT-6+ |
| 25-35 | Marginal | Remove VAT-3+ for reliability |
| **35-42** | **Matches real L1 GPS** | **Optimal for F9P** |
| 42-48 | Slightly above real | F9P may flag C/N0 anomaly |
| > 48 | Too strong | Add VAT-10+ or VAT-3+ |
| > 50 | Way too strong | Add VAT-20+, check for front-end saturation |

**Key rule for ZED-F9P:** Keep synthetic/jammed PRN C/N0 within **+/- 3 dB** of real satellite C/N0.

---

## Physical Setup Reference

```
GOOD (TX directly above RX):

     bladeRF 1.0 [whip hanging down]
                  |
                  | 0.5 / 1.0 / 1.5 m
                  |
                  v
          [F9P patch antenna, facing up]

     Signal at 0 deg boresight = 0 dB antenna loss
     Mimics overhead satellite geometry

BAD (TX beside RX, same height):

     bladeRF [whip] ---- 1 m ---- [F9P patch]

     Signal at 90 deg boresight = 8.9 dB antenna loss
     Requires more TX power, unrealistic geometry
```

---

## Noise Generation Model

### How jam_noise Works

The `jam_noise` method generates **matched-code interference**: noise modulated with the target PRN's C/A code and carrier phase. After despreading in the receiver, the interference concentrates on the target correlator while being rejected (~30 dB processing gain) by all other PRNs.

No valid navigation data is transmitted -- only noise-modulated spreading code.

### Signal Model

Normal PRN sample:
```
I = dataBit * codeCA * cos(carrier_phase) * gain
Q = dataBit * codeCA * sin(carrier_phase) * gain
```

jam_noise PRN sample (replaces normal signal):
```
I = gaussianNoise * codeCA * cos(carrier_phase) * noise_amp
Q = gaussianNoise * codeCA * sin(carrier_phase) * noise_amp
```

Where `noise_amp = gain * 10^(J/S_dB / 20)`.

### Why Matched-Code, Not Broadband

| Noise type | Effect on target PRN | Effect on other PRNs |
|-----------|---------------------|---------------------|
| Broadband | Attenuated ~30 dB by despreading | Attenuated ~30 dB equally -- jams everything or nothing |
| **Matched-code** | **Full noise power after despreading** | **Attenuated ~30 dB -- invisible** |

Matched-code noise is the only way to achieve selective jamming when transmitting OTA alongside real sky signals.

### Noise Distribution

Approximate Gaussian via Central Limit Theorem: `nextGaussianNoise()` sums 4 independent xorshift32 uniform samples in [-250, +250] and divides by 4. Each PRN has an independent PRNG state seeded from the PRN number for reproducibility.

---

## Quick Reference: Complete Workflow

```
1. Download ephemeris         bash download_nasa_ephemeris.sh
2. Find visible PRNs          ./gps-sdr-sim -e brdc.nav -l LAT,LON,HGT -d 10 -v -o /dev/null
3. Generate IQ file           ./gps-sdr-sim -e brdc.nav -l LAT,LON,HGT -P ... -A ... -d 300 -v
4. Assemble RF chain          bladeRF TX --> [attenuators] --> whip antenna above F9P
5. Cold start receiver        u-center 2 > Device > Reset > Cold Start
6. Transmit                   bladeRF-cli -s bladerf.script
7. Monitor C/N0               UBX-NAV-SAT in u-center 2, tune attenuation
8. Record data                u-center 2 > Record > Start recording (UBX format)
```

---

## Scenario Catalog (Full List)

### Baseline & Sanity
1. Clean sky reference (no TX, bladeRF unplugged)
2. Single PRN normal signal via `-P` (verify isolation)
3. Single PRN `jam_drop` (verify signal disappears)

### Partial Constellation Jamming
4. Single PRN `jam_noise` (selective denial)
5. Multi-PRN `jam_noise` (2-3 PRNs)
6. Low-elevation denial (jam PRNs below chosen elevation)
7. Geometry attack (jam PRNs that maximize DOP degradation)
8. Sector attack (jam PRNs in one azimuth quadrant)

### Power-Domain Jamming
9. J/S sweep: 0, 3, 6, 10, 15, 20, 30 dB
10. Soft attenuation via `-G` negative values
11. Time-ramped J/S (concatenated IQ files)

### Synthetic Satellite Injection
12. Single synthetic PRN overhead
13. Two synthetic PRNs at different geometries
14. Four synthetic PRNs for full position fix (conducted)

### Hybrid Attack
15. Jam real PRN + inject synthetic replacement
16. Jam multiple + inject multiple
17. Jam-then-spoof sequential (two IQ files)

### Receiver Stress
18. Attack from t=0 (acquisition stress)
19. Attack after stable lock (tracking stress)
20. Alternating attack ON/OFF cycles (reacquisition stress)
21. Edge-of-fix: jam down to exactly 4 usable PRNs

### Measurement Campaigns
22. C/N0 degradation vs number of jammed PRNs
23. Position error vs J/S ratio
24. Time-to-alarm (spoofDetState transition latency)
25. Recovery time after attack stops

### Planned (Not Yet Implemented)
26. `spoof_delay`: per-PRN pseudorange/Doppler bias injection
27. `spoof_nav`: navigation message field perturbation
