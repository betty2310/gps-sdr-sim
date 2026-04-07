# USRP X300 Integrated Transmitter

## Overview

`x300tx` is an integrated GPS signal generator and transmitter for the USRP X300.
It generates IQ samples in-process and sends them directly via the UHD C++ API
with timed TX metadata, eliminating the pipe-based latency of the old
`gps-sdr-sim | gps-sdr-sim-uhd.py` path.

## Build

```bash
make x300tx
```

Requires UHD library (`pkg-config --exists uhd`).

## Quick Start

```bash
# Stream-now mode (wall clock, quick testing)
./x300tx -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 1,2 -S 1:overhead,2:90.0/45.0 \
  -n -r 0.25 --gain 0

# Explicit GPS epoch (recommended for calibrated operation)
./x300tx -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \
  --gps-week 2361 --gps-tow 118800.0 --gain 0

# With external clock/time source
./x300tx -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \
  --gps-week 2361 --gps-tow 118800.0 \
  --clock-source external --time-source external --gain 0
```

## Options

### GPS Simulation Options

| Flag | Description |
|------|-------------|
| `-e <file>` | Navigation RINEX file (required) |
| `-l <lat,lon,alt>` | Static location in degrees and meters |
| `-c <x,y,z>` | Static ECEF position in meters |
| `-t <YYYY/MM/DD,hh:mm:ss>` | Explicit start time (UTC) |
| `-n` | Stream-now mode (derive GPS time from wall clock) |
| `-d <seconds>` | Duration (default: stream forever in static mode) |
| `-P <prn,prn,...>` | Partial constellation PRN list |
| `-S <spec>` | Synthetic satellite config (e.g. `1:overhead,2:90/45`) |
| `-A <spec>` | Attack config |
| `-J <dB>` | Jammer-to-signal ratio |
| `-G <dB>` | Power boost for partial constellation |
| `-r <sec>` | Future TX start lead for `-n` mode |
| `-p [gain]` | Disable path loss (optional fixed gain 1-128) |
| `-i` | Disable ionospheric correction |
| `-v` | Verbose output |

### USRP X300 Options

| Flag | Default | Description |
|------|---------|-------------|
| `--gps-week <N>` | - | GPS week number |
| `--gps-tow <sec>` | - | GPS time of week (seconds) |
| `--tx-advance-ns <ns>` | `250000000` | Future TX start lead in nanoseconds |
| `--addr <ip>` | `192.168.10.2` | USRP device IP address |
| `--gain <dB>` | `0` | TX gain |
| `--clock-source <src>` | `internal` | Clock source: `internal`, `external`, `gpsdo` |
| `--time-source <src>` | `internal` | Time source: `internal`, `external`, `gpsdo` |
| `--prebuffer <N>` | `5` | Pre-buffer epochs before TX start (1-50) |
| `--antenna <name>` | `TX/RX` | TX antenna port |

## How It Works

### Architecture

Unlike the old pipe-based path:

```
Old:  gps-sdr-sim -o - | gps-sdr-sim-uhd.py    (~233 ms timing error)
New:  x300tx                                     (timed TX, sub-ms capable)
```

`x300tx` links directly against `gpssim` signal generation functions and sends
SC16 IQ samples to the X300 via the UHD C++ API in a single process.

### Timed TX

1. Configures UHD first, then latches the wall clock for `-n` mode
2. Locks the IQ generator to the actual TX sample rate reported by UHD
3. Pre-generates N epochs (default 5 = 0.5 s) into a ring buffer
4. Measures how long startup preparation actually took
5. Schedules the first TX after the remaining lead time
6. Streams continuously, generating and sending one 0.1 s epoch at a time

### GPS Time Modes

**Explicit epoch** (`--gps-week` + `--gps-tow`): Sets the GPS start time
directly. No wall clock involved. Use this for calibrated mixed-sky operation.

**Stream-now** (`-n`): Derives GPS time from the PC wall clock and uses a
future timed start so the wall-clock latch and the first RF sample stay close
together. This is the current best mode for reducing mismatch without a u-blox
time source, but it is still limited by PC wall-clock accuracy (~1-50 ms via NTP).

**Fixed time** (`-t`): Sets a specific UTC date/time. Ephemeris TOC/TOE are
time-shifted to match.

### Start Lead

Use `-r` or `--tx-advance-ns` to control how far into the future the first
timed TX starts in `-n` mode:

1. The default is `250 ms`
2. If startup preparation exceeds the requested lead, `x300tx` warns and falls
   back to a small minimum timed-start margin
3. If that warning appears, increase `-r` / `--tx-advance-ns`

This option is currently a startup lead, not the final absolute-time calibration
term for the later u-blox-driven path.

### Sample Rate Locking

If the X300 cannot realize the requested nominal rate exactly, `x300tx` uses the
actual TX rate returned by UHD for sample generation. When needed, it alternates
epoch sample counts by one sample so the long-run average stays aligned to the
real hardware rate instead of drifting away from GPS time.

## Hardware Setup

- USRP X300/X310 with UBX daughterboard
- 1 GbE connection to host (sufficient for 2.6 Msps)
- TX antenna with appropriate attenuation chain
- Optional: external 10 MHz reference and/or 1 PPS input

Verify the device is reachable:

```bash
uhd_usrp_probe --args addr=192.168.10.2
```
