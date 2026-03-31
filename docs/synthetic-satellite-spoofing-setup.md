# Synthetic Satellite OTA Spoofing: bladeRF 2.0 + ZED-F9P Setup Guide

## Hardware

- **SDR:** bladeRF 2.0 micro (xA4/xA9), bladeRF-cli 1.10.0, libbladeRF 2.6.0
- **Receiver:** u-blox ZED-F9P (dual-frequency L1/L2, advanced anti-spoofing)

## Context

Inject GPS signals for satellites **not currently visible** in the sky using gps-sdr-sim `-S`, transmit OTA via bladeRF 2.0, and have the ZED-F9P acquire and track them alongside real sky signals.

**ZED-F9P challenge:** The F9P is the hardest u-blox module to spoof. It has dual-frequency (L1+L2) tracking, cross-satellite consistency checks, position residual monitoring, and a dedicated spoofing detection flag. gps-sdr-sim generates L1-only signals, so the F9P will see L1 without matching L2 for synthetic PRNs. This document addresses these challenges.

---

## 1. Critical: Fix Sample Rate Mismatch

The current `bladerf.script` plays at 4 MHz but gps-sdr-sim generates at 2.6 MHz by default. This breaks the C/A code rate (1.023 MHz becomes 1.574 MHz) and **no receiver will ever acquire**.

**Rule:** The bladeRF sample rate MUST match the gps-sdr-sim `-s` value (default: 2.6 MHz).

---

## 2. RF Link Budget for OTA

Understanding the power levels is essential before touching any hardware.

### Real GPS signal at receiver

| Parameter | Value |
|-----------|-------|
| GPS satellite TX power | ~27 dBW (+57 dBm) EIRP |
| Free-space path loss (20,200 km) | ~182 dB |
| Signal at ground | -130 dBm (nominal) |
| ZED-F9P acquisition threshold | -148 dBm |
| ZED-F9P tracking threshold | -166 dBm |
| ZED-F9P L1 C/N0 typical | 35-45 dB-Hz |
| ZED-F9P L2 C/N0 typical | 30-40 dB-Hz |

### Free-space path loss at 1575.42 MHz (short range)

```
FSPL(dB) = 36.4 + 20*log10(d_meters)
```

| Distance | FSPL |
|----------|------|
| 0.3 m | 26 dB |
| 0.5 m | 30 dB |
| 1.0 m | 36 dB |
| 2.0 m | 42 dB |
| 3.0 m | 46 dB |
| 5.0 m | 50 dB |
| 10.0 m | 56 dB |

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
| 120° | -30° el | 14.4 dB |
| 180° (from below) | n/a | 31.6 dB |

**Key insight:** A bladeRF antenna placed at the **same height** on a table hits the patch at ~90° boresight = 8.9 dB loss. Placed **directly above** = 0 dB loss. This 9 dB difference is significant.

### OTA link budget calculation

```
P_rx = P_tx - Atten_inline - FSPL + G_tx + G_rx - Polarization_loss
```

| Parameter | Typical value |
|-----------|---------------|
| P_tx (bladeRF 2.0 micro at gain -10) | ~-30 dBm |
| Atten_inline | 30 dB (adjustable) |
| FSPL at 1 m | 36 dB |
| G_tx (whip antenna) | 0 dBi |
| G_rx (ZED-F9P patch at boresight) | +5 dBi |
| Polarization loss (linear TX to RHCP RX) | 3 dB |
| **P_rx** | **-94 dBm** |

This is 36 dB above real GPS (-130 dBm). The ZED-F9P's spoofing detector will flag this immediately. For the F9P, you want the synthetic signal only 3-6 dB above real signals to stay below detection thresholds.

**Practical target:** P_rx = -127 to -124 dBm (3-6 dB above real GPS). The F9P is more sensitive to power anomalies than the M8 or u-blox 6.

---

## 3. Recommended OTA Configurations

### Configuration A: Screened Room (Best for Research)

The cleanest setup. A Faraday cage blocks real sky signals. You control everything.

```
                    Screened Room / Faraday Cage
    +-----------------------------------------------------------+
    |                                                           |
    |   GPS Repeater Antenna ----+                              |
    |   (brings real sky in)     |                              |
    |                            v                              |
    |                                                           |
    |        bladeRF 2.0 TX antenna (whip, facing down)         |
    |              |                                            |
    |              | 1.0 m vertical                             |
    |              v                                            |
    |     +------------------+                                  |
    |     | ZED-F9P patch    | (facing up, on table)            |
    |     | antenna          |                                  |
    |     +------------------+                                  |
    |              |                                            |
    |         USB to PC (outside cage)                          |
    +-----------------------------------------------------------+
```

**RF chain:**
```
bladeRF 2.0 TX (SMA) --> 30 dB atten --> whip antenna (inside cage)
```

**bladerf.script:**
```
set frequency tx1 1575420000
set samplerate tx1 2600000
set bandwidth tx1 2500000
set gain tx1 -10
tx config file=gpssim.bin format=bin repeat=0
tx start
tx wait
```

**gps-sdr-sim command:**
```bash
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 0 \
    -d 300 -v
```

**Link budget:**
| Step | Value | Running total |
|------|-------|---------------|
| bladeRF 2.0 output (gain -10) | -30 dBm | -30 dBm |
| Inline attenuator | -30 dB | -60 dBm |
| FSPL at 1.0 m | -36 dB | -96 dBm |
| TX antenna (whip) | +0 dBi | -96 dBm |
| RX antenna (F9P patch boresight) | +5 dBi | -91 dBm |
| Polarization loss | -3 dB | -94 dBm |
| **Signal at receiver** | | **-94 dBm** |

This is 36 dB above real GPS. If no GPS repeater is used (synthetic-only in the cage), this power level is fine -- the F9P will acquire easily. If a GPS repeater brings in real signals at -130 dBm, reduce power by increasing inline attenuation to 55-60 dB.

### Configuration B: Open-Air OTA (Requires Authorization)

Receiver antenna sees real sky. bladeRF 2.0 adds synthetic signals from above.

```
         Tripod or shelf (1.5 m above table)
              |
         bladeRF 2.0 TX antenna (whip, hanging down)
              |
              | 1.5 m vertical distance
              |
              v
     +------------------+
     | ZED-F9P patch    | (on table, facing up)
     | antenna          |
     +------------------+
```

**RF chain:**
```
bladeRF 2.0 TX (SMA) --> 50 dB atten --> whip antenna (on tripod above receiver)
```

**Why 50 dB:** At 1.5 m with 50 dB inline + 40 dB FSPL + 3 dB pol. loss = 93 dB total loss.
bladeRF 2.0 at gain -10 = -30 dBm output. Signal at RX = -30 - 93 + 5 = -118 dBm.
This is 12 dB above real GPS. For the F9P, this may still trigger spoofing detection.
Increase to 55-60 dB inline atten to bring signal closer to -125 dBm (5 dB above real GPS).

**bladerf.script:**
```
set frequency tx1 1575420000
set samplerate tx1 2600000
set bandwidth tx1 2500000
set gain tx1 -10
tx config file=gpssim.bin format=bin repeat=0
tx start
tx wait
```

**gps-sdr-sim command:**
```bash
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v
```

**Why `-P 25,26` is non-negotiable for OTA:** Without `-P`, the IQ file contains signals for ALL visible satellites. Transmitting those alongside real sky signals creates doubled signals per PRN -- the receiver sees two copies of each satellite signal with different delays, corrupting tracking across the entire constellation. `-P` restricts the IQ output to ONLY the synthetic PRNs.

---

## 4. Physical Placement Details

### TX antenna position and type

| Factor | Recommendation | Why |
|--------|---------------|-----|
| **Vertical vs horizontal** | Vertical (TX above RX) | Patch antennas have peak gain at zenith. A signal from above arrives at boresight = 0 dB antenna loss |
| **Height above RX** | 1.0-1.5 m | Close enough for reasonable link margin, far enough to avoid near-field coupling |
| **TX antenna type** | Quarter-wave whip (~48 mm) or small dipole | Omnidirectional in azimuth. A patch antenna concentrates too much power |
| **TX antenna orientation** | Hanging downward from tripod/shelf | Points toward the RX patch boresight |
| **Avoid** | Placing TX and RX on the same table at the same height | Signal arrives at 90° boresight = 8.9 dB antenna loss, requires more TX power, less realistic |

### Do NOT place the bladeRF antenna sideways

```
BAD:   bladeRF [whip] ---- 2 m ---- [patch] u-blox    (same height)
       Signal arrives at 90° boresight = 8.9 dB loss from patch antenna
       Requires higher TX power, appears as horizon signal

GOOD:       bladeRF [whip hanging down]
                    |
                    | 1.0-1.5 m
                    v
               [patch] u-blox
       Signal arrives at 0° boresight = 0 dB loss
       Appears as overhead signal, matches -S xx:overhead geometry
```

### Table/bench layout

```
+--------------------------------------------------------------------+
|  Lab bench                                                          |
|                                                                     |
|  [Laptop: u-center 2]   [bladeRF 2.0]---SMA---[60dB atten chain]  |
|       |                                              |              |
|       | USB-C                                   SMA cable           |
|       |                                              |              |
|  [ZED-F9P board]---SMA---[L1+L2 antenna facing up]   |              |
|                                              shelf/tripod arm       |
|                                                      |              |
|                                          [whip antenna, 1.5m up]    |
+--------------------------------------------------------------------+
```

### Distance vs required inline attenuation

Target for ZED-F9P: synthetic signal arrives 3-6 dB above real GPS (-127 to -124 dBm).
The F9P is more sensitive to power anomalies than simpler receivers, so tighter power control is needed.
Assumes bladeRF 2.0 micro at `gain tx1 -10` (~-30 dBm), whip TX (0 dBi), F9P patch RX (+5 dBi), 3 dB pol. loss.

| Distance (vertical) | FSPL | Required inline atten | Signal at RX |
|---------------------|------|-----------------------|-------------|
| 0.5 m | 30 dB | 65 dB | -123 dBm |
| 1.0 m | 36 dB | 60 dB | -124 dBm |
| 1.5 m | 40 dB | 55 dB | -123 dBm |
| 2.0 m | 42 dB | 55 dB | -125 dBm |
| 3.0 m | 46 dB | 50 dB | -124 dBm |

**Practical recommendation:** 1.0-1.5 m distance + 60 dB inline attenuation (e.g., three Mini-Circuits VAT-20+ = 60 dB, all SMA). The extra 10 dB compared to simpler receivers is critical for the F9P's spoofing detector.

### Hardware shopping list for OTA

| Item | Model example | Qty | Purpose |
|------|---------------|-----|---------|
| Fixed attenuator 20 dB | Mini-Circuits VAT-20+ | 3 | Inline attenuation (60 dB total) |
| Fixed attenuator 10 dB | Mini-Circuits VAT-10+ | 1 | Fine power adjustment |
| Fixed attenuator 3 dB | Mini-Circuits VAT-3+ | 1 | Fine-tuning for F9P sweet spot |
| SMA cable | RG316, 0.5 m | 2 | Short, low-loss connections |
| Quarter-wave whip | 1575 MHz, SMA male | 1 | TX antenna for bladeRF 2.0 |
| Multi-band GNSS antenna | Active, SMA, L1+L2 | 1 | RX antenna for ZED-F9P (must support L1+L2) |
| Tripod or adjustable shelf | Camera tripod with SMA mount | 1 | Hold TX antenna above RX |

---

## 5. bladeRF 2.0 Configuration Reference

The bladeRF 2.0 micro uses the AD9361 RFIC which handles calibration automatically -- no `cal lms` or `cal dc tx` commands needed (those are bladeRF 1.0 only).

### bladerf.script for OTA

```
set frequency tx1 1575420000
set samplerate tx1 2600000
set bandwidth tx1 2500000
set gain tx1 -10
tx config file=gpssim.bin format=bin repeat=0
tx start
tx wait
```

### Parameter reference

| Parameter | Value | Why |
|-----------|-------|-----|
| `frequency tx1` | 1575420000 | GPS L1 carrier (1575.42 MHz) |
| `samplerate tx1` | 2600000 | **Must match** gps-sdr-sim default (2.6 MHz) |
| `bandwidth tx1` | 2500000 | Slightly below sample rate, covers 2.046 MHz C/A null-to-null BW |
| `gain tx1` | -10 | Starting value for OTA. See tuning table below |
| `repeat=0` | - | Infinite loop. Use `repeat=1` for single-play |

### bladeRF 2.0 micro TX gain range

The `set gain tx1` command controls the combined analog + digital gain of the AD9361.

| Gain setting | Approximate RF output | Use case |
|--------------|----------------------|----------|
| -24 (min) | ~-44 dBm | Very close range or low attenuation |
| -15 | ~-35 dBm | OTA with 60 dB atten at 1 m |
| **-10** | **~-30 dBm** | **Recommended starting point** |
| 0 | ~-20 dBm | Conducted test with 60 dB atten |
| +10 | ~-10 dBm | Long range or heavy attenuation |
| +30 | ~+6 dBm | Maximum -- **never use for GPS without heavy attenuation** |

### Gain tuning by observation

Start at `gain tx1 -10` with 60 dB inline attenuation at 1.0 m. Monitor `UBX-NAV-SAT` C/N0 on the ZED-F9P:

| Observed C/N0 (dB-Hz) | Meaning | Action |
|------------------------|---------|--------|
| 0 (not acquired) | Signal too weak | Increase gain by 5 dB or reduce atten by 10 dB |
| < 25 | Weak, unstable tracking | Increase gain by 3 dB |
| 25-35 | Marginal but tracking | Increase 3 dB for reliability |
| **35-42** | **Matches real L1 GPS** | **Optimal -- F9P spoofing detector least likely to trigger** |
| 42-48 | Slightly above real sats | F9P may flag C/N0 anomaly |
| > 48 | Too strong | Decrease gain by 3 dB or add 10 dB attenuation |
| > 50 | Way too strong | F9P spoofDet will flag, AGC saturation risk |

**Note for ZED-F9P:** Keep synthetic PRN C/N0 within +/- 3 dB of real satellite C/N0. The F9P compares signal strengths across all tracked PRNs. A synthetic signal significantly stronger than real sky signals triggers the cross-consistency check.

---

## 6. gps-sdr-sim Command for OTA

### Identify non-visible PRNs first

```bash
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 -d 10 -v -o /dev/null
```

Note which PRNs appear. Choose PRNs NOT in that list.

### Generate the IQ file

```bash
# Two synthetic satellites, OTA mode
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v \
    -o gpssim.bin
```

### Flag reference for OTA

| Flag | Value | Purpose |
|------|-------|---------|
| `-S 25:overhead` | Synthetic PRN 25 at zenith | Easiest to acquire (shortest range, max antenna gain) |
| `-S 26:180.0/45.0` | Synthetic PRN 26 at az 180, el 45 | South, 45 deg elevation |
| `-P 25,26` | Partial constellation | **MANDATORY for OTA** -- only these PRNs in IQ output |
| `-G 3` | +3 dB boost | Compensates for SDR TX chain loss. Start at 0, increase if needed |
| `-p` | DO NOT USE for OTA | Path loss model must stay active for realistic relative amplitudes |
| `-d 300` | 5 min | Adjust as needed |
| `-v` | Verbose | Confirm synthetic PRNs appear with expected az/el/range |

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
| Position accuracy | 2.5 m (standalone), 1 cm (RTK) |
| Acquisition sensitivity | -148 dBm |
| Tracking sensitivity | -166 dBm |
| Default baud rate | 38400 (USB: auto) |
| Protocol | UBX + NMEA |
| Anti-spoofing | Advanced (see below) |

### The L1-only problem

gps-sdr-sim generates **L1 C/A only**. The ZED-F9P tracks both L1 and L2 for real satellites. For synthetic PRNs, the F9P will:
- Acquire and track the L1 C/A signal normally
- Search for L2C on the same PRN and **fail to find it**
- Flag the PRN as "L1-only" (visible in UBX-NAV-SAT `qualityInd` field)
- Still use L1-only PRNs in the position solution, but with lower weight
- The missing L2 is a detectable anomaly but does NOT cause automatic rejection

**Implication:** The synthetic satellites will track but the F9P knows they are L1-only. This is actually useful research data -- it shows how the F9P handles mixed L1+L2 and L1-only constellations.

### ZED-F9P anti-spoofing features

| Feature | Behavior with synthetic satellites | Mitigation |
|---------|-----------------------------------|------------|
| **Spoofing detection** (`spoofDetState` in UBX-NAV-STATUS) | May trigger if multiple indicators fire | Keep power close to real sat levels |
| **L1/L2 consistency** | Synthetic PRNs have L1 but no L2 | Cannot avoid with L1-only simulator. Monitor but accept. |
| **C/N0 cross-check** | Flags PRNs with C/N0 significantly different from others | Keep synthetic C/N0 within +/-3 dB of real satellites |
| **Position residual check** | Flags PRNs whose pseudorange doesn't fit the solution | Ensure `-l` in gps-sdr-sim matches actual receiver location |
| **Almanac prediction** | Compares observed PRNs against expected visibility | Cold start clears almanac, avoids this check initially |

### Physical connections

```
ZED-F9P (C099-F9P evaluation board or SparkFun GPS-RTK board):
  RF IN (SMA) --> Multi-band GNSS antenna (L1+L2, active, facing up on table)
  USB-C --> Laptop (u-center 2 or u-center)

Optional:
  PPS output --> Oscilloscope (verify timing solution)
  UART --> Serial logger (backup data capture)
```

**Important:** The ZED-F9P antenna must be a **multi-band L1+L2 antenna** (not a cheap L1-only patch). The F9P needs L2 signals from real sky satellites to maintain a healthy position solution. Using an L1-only antenna degrades the entire solution and makes it harder to distinguish synthetic satellite effects.

For OTA, the F9P uses its **own antenna** pointed at the sky. It receives both real sky signals (L1+L2) AND the bladeRF synthetic signal (L1 only) simultaneously.

### u-center 2 configuration (step by step)

u-center 2 is recommended for the ZED-F9P (u-center classic also works).

1. **Connect:** USB, auto-detect baud rate. Or set to 38400/115200 if using UART.

2. **Force cold start before each test:**
   - u-center 2: Device > Reset > Cold Start
   - u-center classic: Receiver > Action > Cold Start
   - UBX command: UBX-CFG-RST, navBbrMask=0xFFFF, resetMode=0x01
   - This clears almanac, position, time. The F9P searches all PRNs without almanac-based filtering.

3. **Enable required messages** (UBX > CFG > VALSET for F9P, or CFG > MSG for classic):

   | Message | Class/ID | Rate | Purpose |
   |---------|----------|------|---------|
   | UBX-NAV-SAT | 0x01 0x35 | 1 Hz | **Primary.** Per-PRN: C/N0, az, el, quality, used flag |
   | UBX-NAV-PVT | 0x01 0x07 | 1 Hz | Position, velocity, time, fix type |
   | UBX-NAV-STATUS | 0x01 0x03 | 1 Hz | Fix status + **spoofDetState** field |
   | UBX-RXM-RAWX | 0x02 0x15 | 1 Hz | Raw pseudorange, carrier phase, Doppler per signal |
   | UBX-MON-RF | 0x0A 0x38 | 1 Hz | Per-RF-block: AGC, noise, jamming indicator (F9P uses this, not MON-HW) |
   | UBX-NAV-SIG | 0x01 0x43 | 1 Hz | Per-signal info (L1 and L2 separately, shows which signals are tracked) |

   **F9P-specific:** Use `UBX-MON-RF` instead of `UBX-MON-HW`. The F9P reports per-RF-block status (one for L1, one for L2). Use `UBX-NAV-SIG` to see L1 vs L2 tracking separately.

4. **Monitor spoofing detection:**
   - In `UBX-NAV-STATUS`, field `spoofDetState`:
     - 0 = Unknown/deactivated
     - 1 = No spoofing indicated
     - 2 = Spoofing indicated
     - 3 = Multiple spoofing indications
   - This is the primary indicator of whether the F9P has detected the synthetic signals as spoofing.

5. **View satellite status:**
   - u-center 2: Satellite view panel
   - u-center classic: View > Docking Windows > Satellite Level + Sky View
   - Look for synthetic PRNs. They should show L1 signal but NO L2 signal.

6. **Log data:**
   - u-center 2: Record > Start recording (UBX format)
   - u-center classic: Receiver > Logfile > Record UBX

### What to look for

**Successful synthetic satellite acquisition on ZED-F9P:**
```
UBX-NAV-SIG output:
PRN  Signal   C/N0   Status
G25  L1 C/A   38.0   Acquired, Tracked, NavBit     <-- synthetic, L1 OK
G25  L2 CL    0.0    Not acquired                   <-- synthetic, NO L2
G08  L1 C/A   42.0   Acquired, Tracked, NavBit     <-- real sky
G08  L2 CL    35.0   Acquired, Tracked, NavBit     <-- real sky, L2 OK
```

```
UBX-NAV-SAT output:
PRN  C/N0   El   Az    Used  Quality
G25  38.0   90   0     Yes   4         <-- synthetic, used in solution
G26  35.0   45   180   Yes   4         <-- synthetic, used in solution
G08  42.0   55   120   Yes   7         <-- real sky (higher quality due to L2)
G13  39.0   30   240   Yes   7         <-- real sky
```

**Key observations:**
- Synthetic PRNs should have `qualityInd` 4 (code lock + carrier lock on L1), while real PRNs show 7 (both L1+L2)
- `Used=Yes` confirms the F9P includes the synthetic PRN in the position solution
- C/N0 of synthetic PRNs should be within +/-3 dB of real PRNs (35-42 dB-Hz)

**F9P spoofing detection states to monitor:**

| `spoofDetState` | Meaning | Implication |
|-----------------|---------|-------------|
| 0 | Deactivated | Spoofing detection not running (may need enable) |
| **1** | **No spoofing** | **Success -- F9P accepted synthetic signals** |
| 2 | Spoofing indicated | F9P flagged anomaly. Reduce power, check position match |
| 3 | Multiple indications | Strong detection. Significant power or geometry mismatch |

### ZED-F9P configuration tips for research

**Disable SBAS (optional):** SBAS integrity data can flag position inconsistencies earlier. Disabling it gives the synthetic signals a better chance of being accepted:
```
UBX-CFG-GNSS: disable SBAS
```

**Set dynamic model to Stationary:** Reduces position jumps when synthetic PRNs enter/exit solution:
```
UBX-CFG-NAV5: dynModel = 2 (Stationary)
```

**Enable raw measurements:** Essential for research-grade analysis:
```
UBX-CFG-VALSET: CFG-MSGOUT-UBX_RXM_RAWX_USB = 1
```

---

## 8. Step-by-Step OTA Workflow

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
5. Assemble RF chain:
   bladeRF 2.0 TX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> whip antenna
                              20 dB         20 dB         20 dB
                              --------- 60 dB total ---------

6. Place ZED-F9P multi-band antenna on table, facing up (clear sky view for OTA)

7. Mount whip antenna on tripod/shelf, 1.0-1.5 m directly above F9P antenna

8. Connect ZED-F9P via USB to laptop running u-center 2
```

### Phase 4: Transmit and observe

```
9.  u-center 2: Force cold start (Device > Reset > Cold Start)

10. Start bladeRF 2.0 transmission:
    bladeRF-cli -s bladerf.script

11. Watch satellite view:
    - Real sky PRNs appear first (L1+L2 signals, higher quality)
    - Synthetic PRNs (25, 26) should appear within 30-45 sec (L1 only)
    - Wait for nav message decoded (~12.5 sec after acquisition)
    - Check UBX-NAV-SIG: synthetic PRNs show L1 tracked, L2 not acquired
    - Check UBX-NAV-SAT: synthetic PRNs show Used=Yes

12. Check C/N0 of synthetic PRNs:
    - Target: 35-42 dB-Hz (within +/-3 dB of real satellite L1 C/N0)
    - Too low (<25): increase bladeRF gain by 3 dB or swap one VAT-20+ for VAT-10+
    - Too high (>48): add VAT-10+ attenuator or decrease bladeRF gain

13. Check spoofing detection:
    - Monitor UBX-NAV-STATUS: spoofDetState field
    - 1 = no spoofing detected (success)
    - 2 or 3 = detected, adjust power levels
    - Monitor UBX-MON-RF: check jamming indicator for L1 RF block
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
    - Raw pseudorange for synthetic vs real PRNs (UBX-RXM-RAWX)
    - qualityInd differences between synthetic (4) and real (7) PRNs
```

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No satellites at all (even real ones disappear) | bladeRF 2.0 output too strong, saturating F9P front-end | Add more attenuation, move TX antenna further |
| Real satellites OK but synthetic PRNs never appear | Signal too weak or sample rate mismatch | Check sample rate match first. Then increase gain by 3 dB |
| Synthetic PRN acquired but C/N0 fluctuates wildly | Multipath from nearby surfaces | Move setup away from walls/metal |
| Synthetic PRN acquired but Used=N | F9P consistency check rejected it (position residual too large) | Verify `-l` matches actual receiver location precisely |
| `spoofDetState` = 2 (spoofing indicated) | F9P detected anomaly | Reduce power to match real sat C/N0. Check position match. |
| Synthetic PRN shows L1 but no L2 in NAV-SIG | Expected -- gps-sdr-sim is L1-only | Normal behavior. The F9P still tracks L1-only signals. |
| `qualityInd` = 4 for synthetic vs 7 for real | Missing L2 reduces quality indicator | Expected. Research data point, not a problem. |
| Position jumps when synthetic PRN enters solution | Geometry mismatch between `-l` and actual location | Ensure `-l` coordinates match to within a few meters |
| Lock lost after 30 seconds | IQ file ended and looped, causing phase discontinuity | Generate longer file (`-d`), match to test duration |
| All real satellites disrupted during OTA | Missing `-P` flag | **Always use `-P` for OTA** |
| F9P MON-RF jamming indicator high on L1 block | TX power too high relative to real signals | Add attenuation. Target C/N0 within +/-3 dB of real sats |

---

## 10. Conducted Testing (Fallback/Comparison)

For initial debugging or controlled experiments, cable-connect the bladeRF 2.0 directly to the ZED-F9P:

```
bladeRF 2.0 TX (SMA) --> DC Block --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> ZED-F9P RF IN (SMA)
                                        20 dB         20 dB         20 dB
                                        --------- 60 dB total ---------
```

```bash
# Synthetic-only, 4 satellites for position fix
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 \
    -S 25:overhead,26:180.0/45.0,27:90.0/30.0,29:270.0/60.0 \
    -P 25,26,27,29 \
    -p \
    -d 300 -v
```

**bladerf.script for conducted:**
```
set frequency tx1 1575420000
set samplerate tx1 2600000
set bandwidth tx1 2500000
set gain tx1 -10
tx config file=gpssim.bin format=bin repeat=0
tx start
tx wait
```

**Notes for F9P conducted test:**
- DC block is essential to prevent ZED-F9P active antenna bias from reaching bladeRF 2.0 TX
- With no real sky signals, the F9P receives L1-only for ALL satellites. The `spoofDetState` may still be 1 (no spoofing) because the L1/L2 inconsistency is universal -- there is no reference L1+L2 satellite to compare against
- This is actually the easiest way to confirm basic synthetic satellite tracking before moving to OTA

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
| `docs/jamming-spoofing-scenarios.md` | - | `-P` partial mode rationale for OTA |
