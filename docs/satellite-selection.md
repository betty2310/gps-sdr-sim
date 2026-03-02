# Satellite Selection and Channel Allocation

## Overview

GPS-SDR-SIM can model up to **32 satellites** (`MAX_SAT = 32`) but simulates a maximum of **16 simultaneously** (`MAX_CHAN = 16`). The tool dynamically selects which satellites to transmit based on visibility from the user's position.

## How Satellites Are Chosen

### 1. Ephemeris Parsing

The RINEX navigation file is parsed by `readRinexNavAll()`, which extracts orbital parameters (ephemeris) for each PRN (1–32). Only satellites with a valid ephemeris (`vflg == 1`) are considered.

### 2. Visibility Check — `checkSatVisibility()`

For each of the 32 PRNs, the tool:

1. Computes the satellite's ECEF position at the current time using `satpos()` (Keplerian orbit propagation from ephemeris).
2. Calculates the line-of-sight (LOS) vector from the user to the satellite.
3. Converts LOS to local North-East-Up (NEU) coordinates via `ecef2neu()`.
4. Derives azimuth and elevation angles via `neu2azel()`.
5. Compares the elevation angle against the **elevation mask** (default: **0°**).

A satellite is **visible** if its elevation is above the mask — i.e., above the horizon.

```
if (azel[1]*R2D > elvMask)
    return (1); // Visible
```

### 3. Channel Allocation — `allocateChannel()`

The allocation logic loops through PRN 1→32 in order:

| Condition | Action |
|---|---|
| Visible & not yet allocated | Assign to the first free channel (first-come, first-served) |
| Visible & already allocated | Keep the existing channel assignment |
| Not visible & was allocated | **Deallocate** — free the channel for reuse |
| Not visible & not allocated | Skip |

When a satellite is newly allocated to a channel, the tool:

- Generates the C/A code for that PRN (`codegen()`)
- Encodes the navigation message subframes (`eph2sbf()`)
- Initializes pseudorange and carrier phase

If all 16 channels are already occupied, additional visible satellites are skipped.

### 4. Dynamic Updates

Channel allocation is **refreshed every 30 seconds** of simulated time. As the simulation progresses:

- Satellites that rise above the horizon get assigned to free channels.
- Satellites that set below the horizon have their channels released.

This models the real-world behavior of a GPS constellation where satellites continuously rise and set.

## Typical Behavior

From any location on Earth, approximately **8–12 GPS satellites** are visible at a given time, well within the 16-channel limit. The `-v` flag can be used to print per-channel details (PRN, azimuth, elevation, range, ionospheric delay) during simulation.

## Key Constants

| Constant | Value | Description |
|---|---|---|
| `MAX_SAT` | 32 | Maximum number of GPS PRNs |
| `MAX_CHAN` | 16 | Maximum simultaneous channels (simulated satellites) |
| `elvmask` | 0.0° | Default elevation mask (horizon) |

## Source References

- Visibility check: `checkSatVisibility()` in `gpssim.c`
- Channel allocation: `allocateChannel()` in `gpssim.c`
- Tracking array: `allocatedSat[MAX_SAT]` maps PRN index → channel index (`-1` if unallocated)
