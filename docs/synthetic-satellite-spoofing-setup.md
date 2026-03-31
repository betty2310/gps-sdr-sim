# Synthetic Satellite OTA Spoofing: bladeRF 1.0 + ZED-F9P Setup Guide

## Hardware

- **SDR:** bladeRF 1.0 (x40/x115), LMS6002D, bladeRF-cli 1.10.0, libbladeRF 2.6.0
- **Receiver:** u-blox ZED-F9P (dual-frequency L1/L2, advanced anti-spoofing)

### Confirm you have a bladeRF 1.0

Run any gain command. If you see `txvga1`/`txvga2` in the output, you have a bladeRF 1.0:

```
Setting TX1 overall gain to 17 dB
  Gain TX1 overall:   17 dB (Range: [17, 73])
            txvga1:  -35 dB (Range: [-35, -4])
            txvga2:    0 dB (Range: [0, 25])
```

The bladeRF 1.0 **cannot set overall gain below 17 dB**. `set gain tx1 -10` is silently clamped to 17. Always use `set txvga1`/`set txvga2` directly.

### bladeRF 1.0 TX gain stages

| Stage | Range | Description |
|-------|-------|-------------|
| `txvga1` | -35 to -4 dB | Pre-mixer analog gain |
| `txvga2` | 0 to 25 dB | Post-mixer analog gain |
| Overall | 17 to 73 dB | Combined. `set gain tx1 N` maps to both stages internally |

| txvga1 | txvga2 | Overall | Approx RF output | Use case |
|--------|--------|---------|-------------------|----------|
| **-35** | **0** | **17 (min)** | **~-10 dBm** | **Use this for GPS** |
| -25 | 0 | 27 | ~0 dBm | With 80+ dB attenuation only |
| -4 | 25 | 73 (max) | ~+10 dBm | **NEVER for GPS** |

## Context

Inject GPS signals for satellites **not currently visible** in the sky using gps-sdr-sim `-S`, transmit OTA via bladeRF 1.0, and have the ZED-F9P acquire and track them alongside real sky signals.

**ZED-F9P challenge:** The F9P is the hardest u-blox module to spoof. It has dual-frequency (L1+L2) tracking, cross-satellite consistency checks, position residual monitoring, and a dedicated spoofing detection flag. gps-sdr-sim generates L1-only signals, so the F9P will see L1 without matching L2 for synthetic PRNs. This document addresses these challenges.

---

## 1. Critical: Fix Sample Rate Mismatch

The current `bladerf.script` plays at 4 MHz but gps-sdr-sim generates at 2.6 MHz by default. This breaks the C/A code rate (1.023 MHz becomes 1.574 MHz) and **no receiver will ever acquire**.

**Rule:** The bladeRF sample rate MUST match the gps-sdr-sim `-s` value (default: 2.6 MHz).

---

## 2. LO Leakage Warning

The bladeRF 1.0's LMS6002D **leaks local oscillator energy through the TX port even when not transmitting**. If the frequency was previously set to 1575.42 MHz, this leakage can degrade u-blox tracking across all satellites -- you will see 5-15 dB C/N0 drop and satellites lost even though you are not transmitting anything.

**Mitigations:**
1. **Keep attenuators in the RF chain at all times** -- even idle LO leakage is killed by 80 dB inline attenuation
2. **Cap the TX SMA with a 50-ohm terminator** when not transmitting
3. **Unplug the bladeRF** during baseline GPS measurements
4. **Run `cal dc tx` before each session** -- reduces (but does not eliminate) LO leakage

---

## 3. RF Link Budget for OTA

### Real GPS signal at receiver

| Parameter | Value |
|-----------|-------|
| GPS satellite TX power | ~27 dBW (+57 dBm) EIRP |
| Free-space path loss (20,200 km) | ~182 dB |
| Signal at ground | -130 dBm (nominal) |
| ZED-F9P acquisition threshold | -148 dBm |
| ZED-F9P tracking threshold | -166 dBm |
| ZED-F9P L1 C/N0 typical | 35-45 dB-Hz |

### Free-space path loss at 1575.42 MHz

```
FSPL(dB) = 36.4 + 20*log10(d_meters)
```

| Height (vertical) | FSPL |
|-------------------|------|
| **0.5 m** | **30 dB** |
| **1.0 m** | **36 dB** |
| **1.5 m** | **40 dB** |

### u-blox patch antenna gain vs arrival angle

From `gpssim.c:107` (`ant_pat_db[]`), the receiver antenna attenuates signals arriving off-boresight:

| Boresight angle | Elevation equivalent | Attenuation |
|-----------------|---------------------|-------------|
| 0° (directly above) | 90° el | 0.0 dB |
| 15° | 75° el | 0.4 dB |
| 30° | 60° el | 1.6 dB |
| 45° | 45° el | 2.9 dB |
| 60° | 30° el | 4.9 dB |
| 75° | 15° el | 6.9 dB |
| 90° (horizon) | 0° el | 8.9 dB |

**Key insight:** TX antenna placed **directly above** the RX patch = 0 dB antenna loss (boresight). Placed at **same height** on a table = 8.9 dB loss. This 9 dB difference is significant. Always place TX above RX.

### Link budget formula

```
P_rx = P_tx - Atten_inline - FSPL + G_tx + G_rx - Polarization_loss

P_tx   = -10 dBm  (bladeRF 1.0, txvga1=-35, txvga2=0)
G_tx   =   0 dBi  (whip antenna)
G_rx   =  +5 dBi  (F9P patch at boresight, TX directly above)
Pol    =   3 dB   (linear TX to RHCP RX)

P_rx = -10 - Atten - FSPL + 0 + 5 - 3
P_rx = -8 - Atten - FSPL
```

### Target power at receiver

| Target | P_rx | Margin above real GPS | F9P reaction |
|--------|------|-----------------------|-------------|
| Too weak | < -133 dBm | < -3 dB | Not acquired or unstable tracking |
| **Ideal** | **-127 to -124 dBm** | **+3 to +6 dB** | **Accepted, spoofDet=1** |
| Marginal | -120 to -115 dBm | +10 to +15 dB | May trigger spoofDet=2 |
| Too strong | > -110 dBm | > +20 dB | spoofDet=2/3, AGC saturation risk |

### Required attenuation by height

| Height | FSPL | Atten for -124 dBm | Atten for -127 dBm | Atten for -130 dBm |
|--------|------|---------------------|---------------------|---------------------|
| **0.5 m** | 30 dB | **86 dB** | **89 dB** | **92 dB** |
| **1.0 m** | 36 dB | **80 dB** | **83 dB** | **86 dB** |
| **1.5 m** | 40 dB | **76 dB** | **79 dB** | **82 dB** |

### Practical attenuator combinations (Mini-Circuits VAT series, SMA)

| Inline atten | Combination | P_rx at 0.5 m | P_rx at 1.0 m | P_rx at 1.5 m |
|-------------|-------------|----------------|----------------|----------------|
| 60 dB | 3x VAT-20+ | -98 dBm | -104 dBm | -108 dBm |
| 70 dB | 3x VAT-20+ + VAT-10+ | -108 dBm | -114 dBm | -118 dBm |
| 76 dB | 3x VAT-20+ + VAT-10+ + VAT-6+ | -114 dBm | -120 dBm | **-124 dBm** |
| 80 dB | 4x VAT-20+ | -118 dBm | **-124 dBm** | -128 dBm |
| 83 dB | 4x VAT-20+ + VAT-3+ | -121 dBm | **-127 dBm** | -131 dBm |
| 86 dB | 4x VAT-20+ + VAT-6+ | **-124 dBm** | -130 dBm | -134 dBm |
| 90 dB | 4x VAT-20+ + VAT-10+ | -128 dBm | -134 dBm | -138 dBm |

---

## 4. Recommended OTA Configurations

### Configuration A: 0.5 m height

```
     bladeRF 1.0 [whip hanging down]
          |
          | 0.5 m
          v
     [F9P patch antenna, facing up]
```

**RF chain:**
```
bladeRF TX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-6+] --> whip antenna
                      20 dB         20 dB         20 dB         20 dB         6 dB
                      ----------------------- 86 dB total -----------------------
```

**Link budget:**
| Step | Value | Running total |
|------|-------|---------------|
| bladeRF 1.0 output (min gain) | -10 dBm | -10 dBm |
| Inline attenuator | -86 dB | -96 dBm |
| FSPL at 0.5 m | -30 dB | -126 dBm |
| TX antenna (whip) | +0 dBi | -126 dBm |
| RX antenna (F9P patch boresight) | +5 dBi | -121 dBm |
| Polarization loss | -3 dB | -124 dBm |
| **Signal at receiver** | | **-124 dBm (+6 dB above real GPS)** |

**bladerf.script:**
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

**gps-sdr-sim command:**
```bash
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v
```

**Pros:** Compact setup, fits on a desk. Strong signal margin.
**Cons:** Near-field effects possible at 0.5 m. Small positioning errors have larger dB impact.

### Configuration B: 1.0 m height (recommended)

```
     bladeRF 1.0 [whip hanging down]
          |
          | 1.0 m
          v
     [F9P patch antenna, facing up]
```

**RF chain:**
```
bladeRF TX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> whip antenna
                      20 dB         20 dB         20 dB         20 dB
                      -------------------- 80 dB total --------------------
```

**Link budget:**
| Step | Value | Running total |
|------|-------|---------------|
| bladeRF 1.0 output (min gain) | -10 dBm | -10 dBm |
| Inline attenuator | -80 dB | -90 dBm |
| FSPL at 1.0 m | -36 dB | -126 dBm |
| TX antenna (whip) | +0 dBi | -126 dBm |
| RX antenna (F9P patch boresight) | +5 dBi | -121 dBm |
| Polarization loss | -3 dB | -124 dBm |
| **Signal at receiver** | | **-124 dBm (+6 dB above real GPS)** |

**bladerf.script:** Same as Configuration A.

**gps-sdr-sim command:** Same as Configuration A.

**Pros:** Best balance of signal stability and practical setup. Clean 80 dB = exactly 4x VAT-20+ (no odd attenuator values). Recommended starting point.
**Cons:** Needs a tripod or shelf to hold the TX antenna 1 m above the table.

### Configuration C: 1.5 m height

```
     bladeRF 1.0 [whip hanging down]
          |
          | 1.5 m
          v
     [F9P patch antenna, facing up]
```

**RF chain:**
```
bladeRF TX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-10+] --> [VAT-6+] --> whip antenna
                      20 dB         20 dB         20 dB         10 dB         6 dB
                      ----------------------- 76 dB total -----------------------
```

**Link budget:**
| Step | Value | Running total |
|------|-------|---------------|
| bladeRF 1.0 output (min gain) | -10 dBm | -10 dBm |
| Inline attenuator | -76 dB | -86 dBm |
| FSPL at 1.5 m | -40 dB | -126 dBm |
| TX antenna (whip) | +0 dBi | -126 dBm |
| RX antenna (F9P patch boresight) | +5 dBi | -121 dBm |
| Polarization loss | -3 dB | -124 dBm |
| **Signal at receiver** | | **-124 dBm (+6 dB above real GPS)** |

**bladerf.script:** Same as Configuration A.

**gps-sdr-sim command:** Same as Configuration A.

**Pros:** Most separation from near-field effects. Extra FSPL means fewer attenuators needed.
**Cons:** Requires a tall tripod. Harder to keep whip antenna precisely aligned above the patch.

### Configuration D: Conducted test (cable, no antenna)

For initial debugging. Cable-connect bladeRF directly to ZED-F9P.

```
bladeRF TX (SMA) --> DC Block --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> ZED-F9P RF IN
                                   20 dB         20 dB         20 dB         20 dB
                                   -------------------- 80 dB total --------------------
```

**DC block is essential** -- prevents ZED-F9P active antenna bias voltage from reaching bladeRF TX.

```bash
# 4 synthetic satellites for full position fix (conducted, no sky signals)
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0,27:90.0/30.0,29:270.0/60.0 \
    -P 25,26,27,29 \
    -p \
    -d 300 -v
```

**Note:** `-p` disables path loss modeling (appropriate for conducted, not for OTA). With no real sky signals, the F9P sees L1-only for all PRNs. The `spoofDetState` may still be 1 because there is no L1+L2 reference to compare against.

**Start here.** Confirm basic synthetic satellite tracking works before attempting OTA.

---

## 5. Physical Placement

### Always place TX directly above RX

```
GOOD:       bladeRF [whip hanging down]
                    |
                    | 0.5 / 1.0 / 1.5 m
                    v
               [patch] u-blox
       Signal arrives at 0° boresight = 0 dB antenna loss
       Mimics overhead satellite geometry

BAD:   bladeRF [whip] ---- 1 m ---- [patch] u-blox    (same height)
       Signal arrives at 90° boresight = 8.9 dB antenna loss
       Requires more TX power, unrealistic satellite geometry
```

### Table/bench layout

```
+--------------------------------------------------------------------+
|  Lab bench                                                          |
|                                                                     |
|  [Laptop: u-center 2]   [bladeRF 1.0]---SMA---[80dB atten chain]  |
|       |                                              |              |
|       | USB-C                                   SMA cable           |
|       |                                              |              |
|  [ZED-F9P board]---SMA---[L1+L2 antenna facing up]   |              |
|                                              shelf/tripod arm       |
|                                                      |              |
|                                          [whip antenna, 1.0m up]    |
+--------------------------------------------------------------------+
```

### Hardware shopping list

| Item | Model example | Qty | Purpose |
|------|---------------|-----|---------|
| Fixed attenuator 20 dB | Mini-Circuits VAT-20+ | 4 | Inline attenuation (80 dB for 1.0 m config) |
| Fixed attenuator 10 dB | Mini-Circuits VAT-10+ | 1 | Fine power adjustment |
| Fixed attenuator 6 dB | Mini-Circuits VAT-6+ | 1 | Fine-tuning for 0.5 m / 1.5 m configs |
| Fixed attenuator 3 dB | Mini-Circuits VAT-3+ | 1 | Fine-tuning for F9P sweet spot |
| DC block | SMA inline | 1 | For conducted test (protects bladeRF from F9P antenna bias) |
| 50-ohm SMA terminator | Any SMA 50-ohm load | 1 | Cap bladeRF TX when idle (prevents LO leakage) |
| SMA cable | RG316, 0.5 m | 2 | Short, low-loss connections |
| Quarter-wave whip | 1575 MHz, SMA male | 1 | TX antenna |
| Multi-band GNSS antenna | Active, SMA, L1+L2 | 1 | RX antenna for ZED-F9P |
| Tripod or adjustable shelf | Camera tripod with SMA mount | 1 | Hold TX antenna above RX |

---

## 6. gps-sdr-sim Command for OTA

### Identify non-visible PRNs first

```bash
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT -d 10 -v -o /dev/null
```

Note which PRNs appear. Choose PRNs **NOT** in that list for synthetic injection.

### Generate the IQ file

```bash
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v \
    -o gpssim.bin
```

### Flag reference for OTA

| Flag | Value | Purpose |
|------|-------|---------|
| `-S 25:overhead` | Synthetic PRN 25 at zenith | Easiest to acquire (max antenna gain from above) |
| `-S 26:180.0/45.0` | Synthetic PRN 26 at az 180, el 45 | South, 45 deg elevation |
| `-P 25,26` | Partial constellation | **MANDATORY for OTA** -- only these PRNs in IQ output |
| `-G 3` | +3 dB boost | Compensates for SDR TX chain loss. Start at 0, increase if needed |
| `-p` | DO NOT USE for OTA | Path loss model must stay active for realistic relative amplitudes |
| `-d 300` | 5 min | Adjust as needed |
| `-v` | Verbose | Confirm synthetic PRNs appear with expected az/el/range |

**Why `-P` is mandatory for OTA:** Without `-P`, the IQ file contains signals for ALL visible satellites. Transmitting those alongside real sky signals creates doubled signals per PRN -- the receiver sees two copies of each satellite, corrupting tracking across the entire constellation.

### Expected verbose output

```
Synthetic PRN 25: az=0.0 el=90.0 deg
Synthetic PRN 26: az=180.0 el=45.0 deg
Partial constellation mode: rendering only PRNs 25 26

PRN   AZ    EL   RANGE(m)  IONO(m)
25    0.0  90.0  20188812.7   4.0    <-- overhead, GPS altitude range
26  180.0  45.0  21658399.0   5.6    <-- correct geometry
```

Ranges of 20,000-22,000 km confirm correct GPS orbit geometry.

---

## 7. ZED-F9P Receiver Setup

### ZED-F9P characteristics

| Parameter | Value |
|-----------|-------|
| Frequencies | L1 C/A + L2C (dual-frequency) |
| Acquisition sensitivity | -148 dBm |
| Tracking sensitivity | -166 dBm |
| Anti-spoofing | Advanced (spoofDetState, L1/L2 cross-check, C/N0 comparison) |

### The L1-only problem

gps-sdr-sim generates **L1 C/A only**. The ZED-F9P tracks both L1 and L2 for real satellites. For synthetic PRNs:

- L1 C/A signal tracked normally
- L2C search fails (no L2 in the transmitted signal)
- PRN flagged as "L1-only" (`qualityInd` 4 instead of 7)
- Still used in position solution, but with lower weight
- The missing L2 is a detectable anomaly but does NOT cause automatic rejection

### ZED-F9P anti-spoofing features

| Feature | Behavior with synthetic satellites | Mitigation |
|---------|-----------------------------------|------------|
| **Spoofing detection** (`spoofDetState`) | May trigger if multiple indicators fire | Keep power within +3 to +6 dB of real GPS |
| **L1/L2 consistency** | Synthetic PRNs have L1 but no L2 | Cannot avoid with L1-only simulator. Monitor but accept |
| **C/N0 cross-check** | Flags PRNs with C/N0 significantly different from others | Keep synthetic C/N0 within +/-3 dB of real satellites |
| **Position residual check** | Flags PRNs whose pseudorange doesn't fit | Ensure `-l` matches actual receiver location |
| **Almanac prediction** | Compares observed PRNs against expected | Cold start clears almanac, avoids this check initially |

### u-center 2 configuration

1. **Connect:** USB, auto-detect baud rate.

2. **Force cold start before each test:**
   - u-center 2: Device > Reset > Cold Start
   - UBX command: UBX-CFG-RST, navBbrMask=0xFFFF, resetMode=0x01

3. **Enable required messages:**

   | Message | Class/ID | Rate | Purpose |
   |---------|----------|------|---------|
   | UBX-NAV-SAT | 0x01 0x35 | 1 Hz | Per-PRN: C/N0, az, el, quality, used flag |
   | UBX-NAV-PVT | 0x01 0x07 | 1 Hz | Position, velocity, time, fix type |
   | UBX-NAV-STATUS | 0x01 0x03 | 1 Hz | Fix status + **spoofDetState** |
   | UBX-NAV-SIG | 0x01 0x43 | 1 Hz | Per-signal L1 vs L2 tracking status |
   | UBX-MON-RF | 0x0A 0x38 | 1 Hz | Per-RF-block AGC, noise, jamming indicator |
   | UBX-RXM-RAWX | 0x02 0x15 | 1 Hz | Raw pseudorange, carrier phase, Doppler |

4. **Monitor spoofing detection** in UBX-NAV-STATUS:
   - 0 = Unknown/deactivated
   - **1 = No spoofing indicated (success)**
   - 2 = Spoofing indicated (reduce power, check position)
   - 3 = Multiple indications (significant mismatch)

### What to look for

**Successful synthetic satellite acquisition:**

```
UBX-NAV-SIG output:
PRN  Signal   C/N0   Status
G25  L1 C/A   38.0   Acquired, Tracked, NavBit     <-- synthetic, L1 OK
G25  L2 CL    0.0    Not acquired                   <-- synthetic, NO L2 (expected)
G08  L1 C/A   42.0   Acquired, Tracked, NavBit     <-- real sky
G08  L2 CL    35.0   Acquired, Tracked, NavBit     <-- real sky, L2 OK
```

```
UBX-NAV-SAT output:
PRN  C/N0   El   Az    Used  Quality
G25  38.0   90   0     Yes   4         <-- synthetic (qualityInd 4 = L1 only)
G26  35.0   45   180   Yes   4         <-- synthetic
G08  42.0   55   120   Yes   7         <-- real sky (qualityInd 7 = L1+L2)
G13  39.0   30   240   Yes   7         <-- real sky
```

**Key checks:**
- Synthetic PRNs: `qualityInd` = 4, real PRNs: 7
- `Used=Yes` confirms F9P includes synthetic PRN in position solution
- Synthetic C/N0 within +/-3 dB of real satellite C/N0 (35-42 dB-Hz)

### ZED-F9P tips for research

```
Disable SBAS:       UBX-CFG-GNSS: disable SBAS (reduces early integrity flags)
Stationary mode:    UBX-CFG-NAV5: dynModel = 2 (reduces position jumps)
Raw measurements:   UBX-CFG-VALSET: CFG-MSGOUT-UBX_RXM_RAWX_USB = 1
```

---

## 8. Gain Tuning by C/N0 Observation

After transmitting, monitor synthetic PRN C/N0 in UBX-NAV-SAT:

| Observed C/N0 (dB-Hz) | Meaning | Action |
|------------------------|---------|--------|
| 0 (not acquired) | Too weak | Remove one VAT-20+ (drop 20 dB atten) |
| < 25 | Weak, unstable | Swap a VAT-20+ for VAT-10+ (drop 10 dB atten) |
| 25-35 | Marginal but tracking | Remove a VAT-3+ or increase `-G` by 3 |
| **35-42** | **Matches real L1 GPS** | **Optimal -- F9P spoofing detector least likely to trigger** |
| 42-48 | Slightly above real | Add VAT-3+ or VAT-6+ |
| > 48 | Too strong | Add VAT-10+ or VAT-20+ |
| > 50 | Way too strong | Front-end saturation risk, spoofDet will flag |

**Key rule:** Keep synthetic PRN C/N0 within **+/- 3 dB** of real satellite C/N0.

---

## 9. Step-by-Step OTA Workflow

### Phase 1: Preparation

```bash
# 1. Download current ephemeris
bash download_nasa_ephemeris.sh

# 2. Build gps-sdr-sim
make clean && make

# 3. Identify non-visible PRNs
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT -d 10 -v -o /dev/null
# Note which PRNs are listed. Choose PRNs NOT in the list.
```

### Phase 2: Generate IQ data

```bash
# 4. Generate synthetic satellite signal
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v
```

### Phase 3: Physical setup

```
5. Assemble RF chain (example: 1.0 m config):

   bladeRF TX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> whip antenna
                         20 dB         20 dB         20 dB         20 dB
                         -------------------- 80 dB total --------------------

   For 0.5 m: add VAT-6+ (86 dB total)
   For 1.5 m: replace one VAT-20+ with VAT-10+ + VAT-6+ (76 dB total)

6. Place ZED-F9P multi-band antenna on table, facing up (clear sky view for OTA)

7. Mount whip antenna on tripod/shelf, directly above F9P antenna

8. Connect ZED-F9P via USB to laptop running u-center 2

IMPORTANT: Keep attenuators connected at all times to suppress LO leakage.
```

### Phase 4: Transmit and observe

```
9.  u-center 2: Force cold start (Device > Reset > Cold Start)

10. Start bladeRF transmission:
    bladeRF-cli -s bladerf.script

11. Watch satellite view:
    - Real sky PRNs appear first (L1+L2, qualityInd 7)
    - Synthetic PRNs (25, 26) should appear within 30-45 sec (L1 only, qualityInd 4)
    - Wait for nav message decoded (~12.5 sec after acquisition)
    - UBX-NAV-SIG: synthetic PRNs show L1 tracked, L2 not acquired
    - UBX-NAV-SAT: synthetic PRNs show Used=Yes

12. Check C/N0 of synthetic PRNs:
    - Target: 35-42 dB-Hz (within +/-3 dB of real satellite L1 C/N0)
    - Too low (<25): swap one VAT-20+ for VAT-10+ (reduce atten by 10 dB)
    - Too high (>48): add VAT-10+ attenuator

13. Check spoofing detection:
    - UBX-NAV-STATUS: spoofDetState field
    - 1 = no spoofing detected (success)
    - 2 or 3 = detected, add more attenuation
    - UBX-MON-RF: jamming indicator for L1 RF block
```

### Phase 5: Data collection

```
14. Record UBX log: u-center 2 > Record > Start recording

15. Key data points to capture:
    - Time to first acquisition of synthetic PRNs
    - C/N0 stability over time (L1 only for synthetic, L1+L2 for real)
    - Position solution accuracy (UBX-NAV-PVT)
    - spoofDetState transitions (UBX-NAV-STATUS)
    - Per-RF-block jamming indicator (UBX-MON-RF)
    - L1 vs L2 signal status per PRN (UBX-NAV-SIG)
    - qualityInd differences: synthetic (4) vs real (7)
```

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Real satellites degrade when bladeRF plugged in (not transmitting) | LO leakage from LMS6002D | Cap SMA with 50-ohm terminator, keep attenuators in chain, or unplug bladeRF |
| `set gain tx1 -10` shows gain clamped to 17 | bladeRF 1.0 minimum overall gain is 17 dB | Use `set txvga1 -35` and `set txvga2 0` directly |
| No satellites at all (even real ones disappear) | TX output saturating F9P front-end | Add more attenuation. At min gain you need ~80 dB inline at 1.0 m |
| Real satellites OK but synthetic never appear | Sample rate mismatch or signal too weak | Check samplerate = 2600000. Then swap one VAT-20+ for VAT-10+ |
| Synthetic PRN acquired but C/N0 fluctuates wildly | Multipath from nearby surfaces | Move setup away from walls/metal |
| Synthetic PRN acquired but Used=N | F9P position residual check rejected it | Verify `-l` matches actual receiver location precisely |
| `spoofDetState` = 2 | F9P detected anomaly | Reduce power (add attenuation). Check `-l` position match |
| Synthetic PRN shows L1 but no L2 | Expected -- gps-sdr-sim is L1-only | Normal behavior. Research data point, not a problem |
| `qualityInd` = 4 for synthetic vs 7 for real | Missing L2 reduces quality indicator | Expected. F9P still uses L1-only PRNs in solution |
| Position jumps when synthetic PRN enters solution | Geometry mismatch | Ensure `-l` coordinates match to within a few meters |
| Lock lost after 30 seconds | IQ file looped, phase discontinuity | Generate longer file (`-d`), match to test duration |
| All real satellites disrupted during OTA | Missing `-P` flag | **Always use `-P` for OTA** |
| F9P MON-RF jamming indicator high on L1 | TX power too high | Add attenuation until C/N0 within +/-3 dB of real sats |

---

## 11. Key Source References

| File | Line | What |
|------|------|------|
| `gpssim.c` | 2329 | Default sample rate 2.6 MHz |
| `gpssim.c` | 107-111 | Antenna pattern `ant_pat_db[]` (37-point, 0-180 deg) |
| `gpssim.c` | 377 | `parseSynthConfig()` -- parses `-S` argument |
| `gpssim.c` | 457 | `azel2satpos()` -- az/el to satellite ECEF position |
| `gpssim.c` | 508 | `synthEphemeris()` -- builds synthetic `ephem_t` |
| `gpssim.c` | 2899-2917 | Gain/power computation with path loss and antenna pattern |
| `gpssim.h` | 14-16 | `MAX_SAT=32`, `MAX_CHAN=16` |
| `docs/synthetic-satellites.md` | - | Full `-S` flag documentation |
| `docs/jamming-spoofing-scenarios.md` | - | `-P` partial mode and attack methods |
