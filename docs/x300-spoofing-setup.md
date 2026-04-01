# Synthetic Satellite OTA Spoofing: USRP X300 + ZED-F9P Setup Guide

## Hardware

- **SDR:** Ettus USRP X300, UBX-160 daughterboard, UHD 4.x
- **Receiver:** u-blox ZED-F9P (dual-frequency L1/L2, advanced anti-spoofing)
- **Host connection:** 10 GbE (SFP+) or PCIe

### Why X300 over bladeRF 1.0

| Parameter | bladeRF 1.0 | USRP X300 |
|-----------|-------------|-----------|
| DAC resolution | 12-bit | 16-bit |
| Min TX gain | Clamped at 17 dB overall (must use txvga1/txvga2 workaround) | Continuous 0-31.5 dB in 0.5 dB steps |
| LO leakage | Significant (LMS6002D leaks even when idle, degrades F9P tracking) | Negligible (far better TX isolation) |
| Clock | Internal VCTCXO (~1 ppm) | Internal OCXO or **external 10 MHz ref** (sub-ppb with GPSDO) |
| Streaming | USB 3.0 (underrun risk) | 10 GbE / PCIe (no underruns at 2.6 Msps) |
| Phase noise | Higher | Lower (better LO synthesis) |

**Key wins:** No LO leakage workaround needed. Fine gain control eliminates the bladeRF `txvga1`/`txvga2` hack. External 10 MHz reference makes carrier frequency indistinguishable from real GPS. No streaming underruns means no phase discontinuities.

## Context

Inject GPS signals for satellites **not currently visible** in the sky using gps-sdr-sim `-S`, transmit OTA via USRP X300, and have the ZED-F9P acquire and track them alongside real sky signals.

**ZED-F9P challenge:** The F9P is the hardest u-blox module to spoof. It has dual-frequency (L1+L2) tracking, cross-satellite consistency checks, position residual monitoring, and a dedicated spoofing detection flag. gps-sdr-sim generates L1-only signals, so the F9P will see L1 without matching L2 for synthetic PRNs. This document addresses these challenges.

---

## 1. USRP X300 Setup

### 1.1 Network configuration (10 GbE)

The X300 defaults to `192.168.10.2` on SFP+ port 0. Configure the host NIC:

```bash
# Set host NIC IP (replace eth0 with your 10 GbE interface)
sudo ip addr add 192.168.10.1/24 dev eth0
sudo ip link set eth0 mtu 9000

# Verify connectivity
uhd_find_devices --args "type=x300"
```

Expected output:
```
--------------------------------------------------
-- UHD Device 0
--------------------------------------------------
Device Address:
    type: x300
    addr: 192.168.10.2
    name:
    serial: ...
    product: X300
```

### 1.2 Verify UHD and daughterboard

```bash
uhd_usrp_probe --args "type=x300"
```

Confirm you see:
- `UBX-160` (or `SBX-120`) under TX Dboard
- TX frequency range covers 1575.42 MHz
- TX gain range: 0 to 31.5 dB (UBX)

### 1.3 Clock source

**Internal OCXO (default):**
```bash
tx_samples_from_file --args "type=x300" ...
# Uses internal clock, ~2.5 ppb stability (much better than bladeRF's ~1 ppm)
```

**External 10 MHz reference (recommended for GPS):**

Connect a GPSDO (e.g., Jackson Labs Fury, Leo Bodnar) to the X300 REF IN (SMA, 10 MHz, +10 dBm).

```bash
tx_samples_from_file --args "type=x300,clock_source=external,time_source=external" ...
```

Using an external GPSDO gives sub-ppb frequency accuracy -- the transmitted L1 carrier will be indistinguishable from real GPS in frequency. This makes it significantly harder for the F9P to detect spoofing via Doppler/frequency anomalies.

### 1.4 TX gain stages (UBX daughterboard)

| Gain (dB) | Approx RF output | Use case |
|-----------|-------------------|----------|
| **0** | **~-20 dBm** | **Start here for GPS** |
| 5 | ~-15 dBm | If 0 dB is too weak after attenuation |
| 10 | ~-10 dBm | Comparable to bladeRF 1.0 min |
| 20 | ~0 dBm | With 90+ dB attenuation only |
| 31.5 (max) | ~+10 dBm | **NEVER for GPS** |

Unlike the bladeRF 1.0, gain is set in a single `--gain` parameter with 0.5 dB resolution. No workaround needed.

---

## 2. Transmitting with tx_samples_from_file

### Basic command

```bash
tx_samples_from_file \
    --args "type=x300" \
    --freq 1575420000 \
    --rate 2600000 \
    --gain 0 \
    --type short \
    --file gpssim.bin \
    --repeat 10
```

### Parameter reference

| Parameter | Value | Notes |
|-----------|-------|-------|
| `--args "type=x300"` | Device selector | Add `,addr=192.168.10.2` if multiple USRPs |
| `--freq 1575420000` | L1 C/A center frequency | 1575.42 MHz |
| `--rate 2600000` | Sample rate | **Must match gps-sdr-sim `-s` (default 2.6 MHz)** |
| `--gain 0` | TX gain in dB | Start at 0, increase if signal too weak |
| `--type short` | Sample format | 16-bit signed int I/Q (matches gps-sdr-sim `-b 16`) |
| `--file gpssim.bin` | IQ data file | Output of gps-sdr-sim |
| `--repeat 10` | Loop count | Number of times to replay the file. Use large value for long tests |

### With external 10 MHz reference

```bash
tx_samples_from_file \
    --args "type=x300,clock_source=external,time_source=external" \
    --freq 1575420000 \
    --rate 2600000 \
    --gain 0 \
    --type short \
    --file gpssim.bin \
    --repeat 10
```

### With specific subdevice (if multiple daughterboards)

```bash
tx_samples_from_file \
    --args "type=x300" \
    --subdev "A:0" \
    --freq 1575420000 \
    --rate 2600000 \
    --gain 0 \
    --type short \
    --file gpssim.bin \
    --repeat 10
```

### Critical: sample rate must match

The `--rate` value **must** match the gps-sdr-sim `-s` value. Mismatch breaks the C/A code chipping rate and no receiver will acquire.

| gps-sdr-sim flag | tx_samples_from_file flag | Value |
|------------------|---------------------------|-------|
| `-s 2600000` (default) | `--rate 2600000` | 2.6 MHz |
| `-s 2500000` | `--rate 2500000` | 2.5 MHz (cleaner for X300 clock divider) |

**Note:** 2.5 MHz produces an even divider from the X300's 200 MHz master clock (200M / 80 = 2.5M). This may give marginally lower jitter than 2.6 MHz. If you use 2.5 MHz, generate IQ data with `-s 2500000`.

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
| 45° | 45° el | 2.9 dB |
| 90° (horizon) | 0° el | 8.9 dB |

**Always place TX directly above RX** (0 dB antenna loss at boresight).

### Link budget formula

```
P_rx = P_tx - Atten_inline - FSPL + G_tx + G_rx - Polarization_loss

P_tx   = -20 dBm  (USRP X300, UBX, gain=0)
G_tx   =   0 dBi  (whip antenna)
G_rx   =  +5 dBi  (F9P patch at boresight, TX directly above)
Pol    =   3 dB   (linear TX to RHCP RX)

P_rx = -20 - Atten - FSPL + 0 + 5 - 3
P_rx = -18 - Atten - FSPL
```

**Note:** The X300 at gain=0 outputs ~10 dB less than bladeRF 1.0 at min gain. This means you need **~10 dB less inline attenuation** for the same received power.

### Target power at receiver

| Target | P_rx | Margin above real GPS | F9P reaction |
|--------|------|-----------------------|-------------|
| Too weak | < -133 dBm | < -3 dB | Not acquired or unstable tracking |
| **Ideal** | **-127 to -124 dBm** | **+3 to +6 dB** | **Accepted, spoofDet=1** |
| Marginal | -120 to -115 dBm | +10 to +15 dB | May trigger spoofDet=2 |
| Too strong | > -110 dBm | > +20 dB | spoofDet=2/3, AGC saturation risk |

### Required attenuation by height (X300 gain=0, P_tx=-20 dBm)

| Height | FSPL | Atten for -124 dBm | Atten for -127 dBm | Atten for -130 dBm |
|--------|------|---------------------|---------------------|---------------------|
| **0.5 m** | 30 dB | **76 dB** | **79 dB** | **82 dB** |
| **1.0 m** | 36 dB | **70 dB** | **73 dB** | **76 dB** |
| **1.5 m** | 40 dB | **66 dB** | **69 dB** | **72 dB** |

Compare with bladeRF 1.0: the X300 needs ~10 dB less attenuation at the same height (one fewer VAT-20+ in many configurations).

### Practical attenuator combinations (Mini-Circuits VAT series, SMA)

| Inline atten | Combination | P_rx at 0.5 m | P_rx at 1.0 m | P_rx at 1.5 m |
|-------------|-------------|----------------|----------------|----------------|
| 60 dB | 3x VAT-20+ | -108 dBm | -114 dBm | -118 dBm |
| 66 dB | 3x VAT-20+ + VAT-6+ | -114 dBm | -120 dBm | **-124 dBm** |
| 70 dB | 3x VAT-20+ + VAT-10+ | -118 dBm | **-124 dBm** | -128 dBm |
| 73 dB | 3x VAT-20+ + VAT-10+ + VAT-3+ | -121 dBm | **-127 dBm** | -131 dBm |
| 76 dB | 3x VAT-20+ + VAT-10+ + VAT-6+ | **-124 dBm** | -130 dBm | -134 dBm |
| 80 dB | 4x VAT-20+ | -128 dBm | -134 dBm | -138 dBm |

### Fine-tuning with X300 gain instead of attenuators

Because the X300 has 0.5 dB gain steps, you can **replace attenuator swapping with gain adjustment**:

```
Need 3 dB more signal?  Increase --gain from 0 to 3.
Need 3 dB less signal?  Not possible below gain=0. Add VAT-3+.
```

This is a major workflow improvement over the bladeRF 1.0, where gain adjustment below the 17 dB clamp required physically swapping attenuators.

---

## 4. Recommended OTA Configurations

### Configuration A: 1.0 m height (recommended)

```
     USRP X300 [whip hanging down]
          |
          | 1.0 m
          v
     [F9P patch antenna, facing up]
```

**RF chain:**
```
X300 TX/RX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-10+] --> whip antenna
                      20 dB         20 dB         20 dB         10 dB
                      -------------------- 70 dB total --------------------
```

**Link budget:**
| Step | Value | Running total |
|------|-------|---------------|
| X300 output (gain=0) | -20 dBm | -20 dBm |
| Inline attenuator | -70 dB | -90 dBm |
| FSPL at 1.0 m | -36 dB | -126 dBm |
| TX antenna (whip) | +0 dBi | -126 dBm |
| RX antenna (F9P patch boresight) | +5 dBi | -121 dBm |
| Polarization loss | -3 dB | -124 dBm |
| **Signal at receiver** | | **-124 dBm (+6 dB above real GPS)** |

**TX command:**
```bash
tx_samples_from_file \
    --args "type=x300" \
    --freq 1575420000 \
    --rate 2600000 \
    --gain 0 \
    --type short \
    --file gpssim.bin \
    --repeat 10
```

**gps-sdr-sim command:**
```bash
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v
```

### Configuration B: 0.5 m height

**RF chain:**
```
X300 TX/RX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-10+] --> [VAT-6+] --> whip antenna
                      20 dB         20 dB         20 dB         10 dB         6 dB
                      ----------------------- 76 dB total -----------------------
```

**Link budget:** -20 - 76 - 30 + 0 + 5 - 3 = **-124 dBm (+6 dB above real GPS)**

**TX command:** Same as Configuration A.

### Configuration C: 1.5 m height

**RF chain:**
```
X300 TX/RX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-6+] --> whip antenna
                      20 dB         20 dB         20 dB         6 dB
                      -------------------- 66 dB total --------------------
```

**Link budget:** -20 - 66 - 40 + 0 + 5 - 3 = **-124 dBm (+6 dB above real GPS)**

**TX command:** Same as Configuration A.

### Configuration D: Conducted test (cable, no antenna)

For initial debugging. Cable-connect X300 directly to ZED-F9P.

```
X300 TX/RX (SMA) --> DC Block --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-10+] --> ZED-F9P RF IN
                                   20 dB         20 dB         20 dB         10 dB
                                   -------------------- 70 dB total --------------------
```

**DC block is essential** -- prevents ZED-F9P active antenna bias voltage from reaching X300 TX.

```bash
# Generate 4 synthetic satellites for full position fix
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0,27:90.0/30.0,29:270.0/60.0 \
    -P 25,26,27,29 \
    -p \
    -d 300 -v

# Transmit (conducted)
tx_samples_from_file \
    --args "type=x300" \
    --freq 1575420000 \
    --rate 2600000 \
    --gain 0 \
    --type short \
    --file gpssim.bin \
    --repeat 10
```

**Note:** `-p` disables path loss modeling (appropriate for conducted, not for OTA).

**Start here.** Confirm basic synthetic satellite tracking works before attempting OTA.

---

## 5. Physical Placement

### Always place TX directly above RX

```
GOOD:       X300 [whip hanging down]
                    |
                    | 0.5 / 1.0 / 1.5 m
                    v
               [patch] u-blox
       Signal arrives at 0 deg boresight = 0 dB antenna loss
       Mimics overhead satellite geometry

BAD:   X300 [whip] ---- 1 m ---- [patch] u-blox    (same height)
       Signal arrives at 90 deg boresight = 8.9 dB antenna loss
       Requires more TX power, unrealistic satellite geometry
```

### Table/bench layout

```
+--------------------------------------------------------------------+
|  Lab bench                                                          |
|                                                                     |
|  [Laptop: u-center 2]   [USRP X300]---SMA---[70dB atten chain]    |
|       |                      |                       |              |
|       | USB-C            10 GbE SFP+            SMA cable           |
|       |                      |                       |              |
|  [ZED-F9P board]---SMA  [Host PC]                shelf/tripod arm   |
|       |                                              |              |
|  [L1+L2 antenna facing up]               [whip antenna, 1.0m up]   |
+--------------------------------------------------------------------+
```

**Note:** The X300 requires a separate host PC with 10 GbE (or PCIe). This is different from the bladeRF which connects directly via USB. The laptop running u-center 2 can be the same host PC if it has both 10 GbE and USB.

### Hardware shopping list

| Item | Model example | Qty | Purpose |
|------|---------------|-----|---------|
| Fixed attenuator 20 dB | Mini-Circuits VAT-20+ | 3-4 | Inline attenuation |
| Fixed attenuator 10 dB | Mini-Circuits VAT-10+ | 1 | Fine power adjustment |
| Fixed attenuator 6 dB | Mini-Circuits VAT-6+ | 1 | Fine-tuning for 0.5 m / 1.5 m configs |
| Fixed attenuator 3 dB | Mini-Circuits VAT-3+ | 1 | Fine-tuning for F9P sweet spot |
| DC block | SMA inline | 1 | For conducted test (protects X300 from F9P antenna bias) |
| SMA cable | RG316, 0.5 m | 2 | Short, low-loss connections |
| Quarter-wave whip | 1575 MHz, SMA male | 1 | TX antenna |
| Multi-band GNSS antenna | Active, SMA, L1+L2 | 1 | RX antenna for ZED-F9P |
| Tripod or adjustable shelf | Camera tripod with SMA mount | 1 | Hold TX antenna above RX |
| 10 GbE NIC + SFP+ cable | Intel X520-DA1 + DAC cable | 1 | Host-to-X300 connection |
| 10 MHz GPSDO (optional) | Jackson Labs Fury, Leo Bodnar | 1 | External reference for sub-ppb accuracy |

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
| `-G 3` | +3 dB boost | Compensates for TX chain loss. Start at 0, increase if needed |
| `-p` | DO NOT USE for OTA | Path loss model must stay active for realistic relative amplitudes |
| `-d 300` | 5 min | Adjust to match `--repeat` loop duration |
| `-v` | Verbose | Confirm synthetic PRNs appear with expected az/el/range |

**Why `-P` is mandatory for OTA:** Without `-P`, the IQ file contains signals for ALL visible satellites. Transmitting those alongside real sky signals creates doubled signals per PRN -- the receiver sees two copies of each satellite, corrupting tracking across the entire constellation.

### Using 2.5 MHz sample rate (optional, cleaner clock divider)

```bash
# Generate at 2.5 MHz
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -s 2500000 \
    -d 300 -v

# Transmit at 2.5 MHz (must match)
tx_samples_from_file \
    --args "type=x300" \
    --freq 1575420000 \
    --rate 2500000 \
    --gain 0 \
    --type short \
    --file gpssim.bin \
    --repeat 10
```

---

## 7. ZED-F9P Receiver Setup

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

---

## 8. Gain Tuning by C/N0 Observation

After transmitting, monitor synthetic PRN C/N0 in UBX-NAV-SAT:

| Observed C/N0 (dB-Hz) | Meaning | Action |
|------------------------|---------|--------|
| 0 (not acquired) | Too weak | Increase `--gain` by 5, or remove one VAT-20+ |
| < 25 | Weak, unstable | Increase `--gain` by 3, or swap a VAT-20+ for VAT-10+ |
| 25-35 | Marginal but tracking | Increase `--gain` by 1-3 |
| **35-42** | **Matches real L1 GPS** | **Optimal** |
| 42-48 | Slightly above real | Decrease `--gain` by 3, or add VAT-3+ |
| > 48 | Too strong | Decrease `--gain` by 5, or add VAT-10+ |
| > 50 | Way too strong | Add VAT-20+ |

**X300 advantage:** With 0.5 dB gain steps, you can tune signal level from the command line without physically touching the attenuator chain. Set up attenuators once to get into the right ballpark, then fine-tune with `--gain`.

**Key rule:** Keep synthetic PRN C/N0 within **+/- 3 dB** of real satellite C/N0.

---

## 9. Step-by-Step OTA Workflow

### Phase 1: Preparation

```bash
# 1. Download current ephemeris
bash download_nasa_ephemeris.sh

# 2. Build gps-sdr-sim
make clean && make

# 3. Verify USRP X300 connectivity
uhd_find_devices --args "type=x300"

# 4. Identify non-visible PRNs
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT -d 10 -v -o /dev/null
# Note which PRNs are listed. Choose PRNs NOT in the list.
```

### Phase 2: Generate IQ data

```bash
# 5. Generate synthetic satellite signal
./gps-sdr-sim -e brdc.nav -l YOUR_LAT,YOUR_LON,YOUR_HGT \
    -S 25:overhead,26:180.0/45.0 \
    -P 25,26 \
    -G 3 \
    -d 300 -v
```

### Phase 3: Physical setup

```
6. Assemble RF chain (example: 1.0 m config):

   X300 TX/RX (SMA) --> [VAT-20+] --> [VAT-20+] --> [VAT-20+] --> [VAT-10+] --> whip antenna
                          20 dB         20 dB         20 dB         10 dB
                          -------------------- 70 dB total --------------------

   For 0.5 m: add VAT-6+ (76 dB total)
   For 1.5 m: remove VAT-10+, add VAT-6+ (66 dB total)

7. Place ZED-F9P multi-band antenna on table, facing up (clear sky view for OTA)

8. Mount whip antenna on tripod/shelf, directly above F9P antenna

9. Connect ZED-F9P via USB to laptop running u-center 2

10. Connect X300 to host PC via 10 GbE

11. (Optional) Connect GPSDO to X300 REF IN
```

### Phase 4: Transmit and observe

```
12. u-center 2: Force cold start (Device > Reset > Cold Start)

13. Start transmission:
    tx_samples_from_file \
        --args "type=x300" \
        --freq 1575420000 \
        --rate 2600000 \
        --gain 0 \
        --type short \
        --file gpssim.bin \
        --repeat 10

14. Watch satellite view:
    - Real sky PRNs appear first (L1+L2, qualityInd 7)
    - Synthetic PRNs (25, 26) should appear within 30-45 sec (L1 only, qualityInd 4)
    - Wait for nav message decoded (~12.5 sec after acquisition)
    - UBX-NAV-SIG: synthetic PRNs show L1 tracked, L2 not acquired
    - UBX-NAV-SAT: synthetic PRNs show Used=Yes

15. Check C/N0 of synthetic PRNs:
    - Target: 35-42 dB-Hz (within +/-3 dB of real satellite L1 C/N0)
    - Too low: increase --gain by 3 dB (or swap attenuators)
    - Too high: decrease --gain by 3 dB (or add attenuator)

16. Check spoofing detection:
    - UBX-NAV-STATUS: spoofDetState field
    - 1 = no spoofing detected (success)
    - 2 or 3 = detected, reduce --gain or add attenuation
```

### Phase 5: Data collection

```
17. Record UBX log: u-center 2 > Record > Start recording

18. Key data points to capture:
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
| `uhd_find_devices` shows nothing | Network config wrong | Check host NIC is 192.168.10.1/24, MTU 9000 |
| `No devices found for ---` | Wrong `--args` | Use `--args "type=x300,addr=192.168.10.2"` |
| `U` underrun warnings in terminal | Host can't keep up (rare at 2.6 Msps) | Check 10 GbE link, increase buffer size, close other apps |
| `L` late packet warnings | Network congestion | Ensure dedicated 10 GbE link, no other traffic |
| `ERROR: clock_source=external but no ref lock` | No 10 MHz signal on REF IN | Check GPSDO connection and output level (+10 dBm) |
| No satellites at all (real ones disappear) | TX output saturating F9P front-end | Decrease `--gain`, add more attenuation |
| Real satellites OK but synthetic never appear | Sample rate mismatch | Verify `--rate` matches gps-sdr-sim `-s` value |
| Synthetic PRN acquired but C/N0 fluctuates wildly | Multipath from nearby surfaces | Move setup away from walls/metal |
| Synthetic PRN acquired but Used=N | F9P position residual check rejected it | Verify `-l` matches actual receiver location |
| `spoofDetState` = 2 | F9P detected anomaly | Decrease `--gain` by 3. Check `-l` position match |
| Synthetic PRN shows L1 but no L2 | Expected -- gps-sdr-sim is L1-only | Normal behavior |
| `qualityInd` = 4 for synthetic vs 7 for real | Missing L2 reduces quality indicator | Expected |
| Position jumps when synthetic PRN enters solution | Geometry mismatch | Ensure `-l` coordinates match to within a few meters |
| All real satellites disrupted during OTA | Missing `-P` flag | **Always use `-P` for OTA** |

---

## 11. X300 vs bladeRF 1.0: Quick Migration Guide

If you have an existing bladeRF 1.0 setup and are switching to X300:

| bladeRF 1.0 | USRP X300 equivalent |
|-------------|---------------------|
| `bladeRF-cli -s bladerf.script` | `tx_samples_from_file --args "type=x300" ...` |
| `set frequency tx1 1575420000` | `--freq 1575420000` |
| `set samplerate tx1 2600000` | `--rate 2600000` |
| `set txvga1 -35` + `set txvga2 0` | `--gain 0` |
| `set bandwidth tx1 2500000` | Automatic (UHD sets appropriate filter) |
| `cal lms` / `cal dc tx` | Not needed (X300 has better factory cal) |
| `tx config file=gpssim.bin format=bin` | `--file gpssim.bin --type short` |
| `tx config repeat=0` (infinite) | `--repeat N` (set high number) |
| 50-ohm terminator when idle | Not needed (negligible LO leakage) |
| 86 dB attenuation at 0.5 m | 76 dB attenuation at 0.5 m (10 dB less) |
| 80 dB attenuation at 1.0 m | 70 dB attenuation at 1.0 m (10 dB less) |

---

## 12. Key Source References

| File | Line | What |
|------|------|------|
| `gpssim.c` | 2329 | Default sample rate 2.6 MHz |
| `gpssim.c` | 107-111 | Antenna pattern `ant_pat_db[]` (37-point, 0-180 deg) |
| `gpssim.c` | 377 | `parseSynthConfig()` -- parses `-S` argument |
| `gpssim.c` | 457 | `azel2satpos()` -- az/el to satellite ECEF position |
| `gpssim.c` | 508 | `synthEphemeris()` -- builds synthetic `ephem_t` |
| `gpssim.c` | 2899-2917 | Gain/power computation with path loss and antenna pattern |
| `gpssim.h` | 14-16 | `MAX_SAT=32`, `MAX_CHAN=16` |
| `gps-sdr-sim-uhd.py` | - | Alternative GNU Radio/UHD TX script (uses 2.5 MHz default) |
| `docs/synthetic-satellites.md` | - | Full `-S` flag documentation |
| `docs/jamming-spoofing-scenarios.md` | - | `-P` partial mode and attack methods |
| `docs/synthetic-satellite-spoofing-setup.md` | - | bladeRF 1.0 version of this guide |
