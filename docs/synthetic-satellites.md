# Synthetic Satellite Generation (`-S`)

## Purpose

By default, gps-sdr-sim only generates signals for satellites that have valid ephemeris **and** are above the horizon at the given location and time. The `-S` flag breaks this constraint, enabling signals from satellites that a real receiver could not normally see. This is the foundation for:

- Injecting unknown PRNs into a receiver's sky view
- Testing receiver response to satellites arriving from unexpected directions
- Combining with `-P` and `-A` for hybrid jamming/spoofing scenarios
- Validating integrity monitoring against out-of-view signal injection

## Usage

```
-S <spec>    Synthetic satellite specification (comma-separated)
             PRN:force           Force a below-horizon PRN (requires ephemeris in RINEX)
             PRN:overhead        Place a synthesized satellite directly at zenith
             PRN:az/el           Place a synthesized satellite at azimuth/elevation (degrees)
```

**Multiple specs** are comma-separated:

```bash
-S 17:force,25:overhead,30:180.0/45.0
```

**Coordinate convention**: azimuth is measured clockwise from North (0°–360°), elevation is measured from the horizon (0°–90°). Slightly negative elevations (down to –5°) are accepted.

## Two Cases

### Case A — Force (`PRN:force`)

The satellite **has valid ephemeris in the RINEX file** but its elevation is below the mask at the given time and location. Normally `allocateChannel()` would skip it. With `:force`, the elevation check is bypassed and the satellite is allocated to a channel using its real orbital parameters.

The transmitted signal is physically correct: the pseudorange, Doppler, and code phase all match where the satellite actually is — just below the horizon.

```bash
# Force PRN 5 even though it is at elevation -12°
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 -S 5:force -v
```

Output:
```
Forced PRN 05 (below-horizon bypass)
05    3.2 -12.4  27068184.8   6.9
```

**When to use:** Geometry attack scenarios where a receiver is not expecting a signal from a known PRN at a negative elevation. Also useful for testing acquisition sensitivity at very low elevations.

### Case B — Synthesized (`PRN:overhead` / `PRN:az/el`)

The satellite **has no ephemeris in the RINEX file** (or you want to place a signal at an exact sky position regardless of the real orbit). A complete `ephem_t` struct is synthesized from scratch and stored in a separate synthetic-ephemeris table before simulation begins. At runtime, the active ephemeris set is built by overlaying those synthetic entries onto the current real set, so the full downstream pipeline — `satpos()`, `computeRange()`, `eph2sbf()`, `generateNavMsg()` — still runs unmodified on a normal `ephem_t`.

```bash
# Synthesize PRN 25 directly overhead
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 -S 25:overhead -v

# Synthesize PRN 30 at azimuth 180°, elevation 45°
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 -S 30:180.0/45.0 -v
```

Output:
```
Synthetic PRN 25: az=0.0 el=90.0 deg
25  180.0  90.0  20188812.7   4.0

Synthetic PRN 30: az=180.0 el=45.0 deg
30  180.0  45.0  21658399.0   5.6
```

## Synthetic Orbit Model

The synthesized ephemeris represents a **circular GPS orbit**:

| Parameter       | Value                     | Notes                                      |
| --------------- | ------------------------- | ------------------------------------------ |
| Semi-major axis | 26,559,700 m              | Nominal GPS orbital radius (~20,200 km alt)|
| Eccentricity    | 0.0                       | Circular orbit (simplifies derivation)     |
| Inclination     | 55° (0.9599 rad)          | Standard GPS constellation inclination     |
| All corrections | 0                         | cuc, cus, cic, cis, crc, crs = 0          |
| Clock terms     | 0                         | af0, af1, af2, tgd = 0                     |
| RAAN rate       | −8×10⁻⁹ rad/s             | Approximate GPS secular RAAN drift         |

### Inverse Geometry: Az/El → Orbital Elements

To place the satellite at the requested sky position, the following derivation is used:

**Step 1 — Az/el to satellite ECEF position**

From the receiver position, build a NEU unit vector:
```
N = cos(el) × cos(az)
E = cos(el) × sin(az)
U = sin(el)
```

Transform NEU → ECEF using the transpose of the local tangent matrix (`ltcmat()`). Then compute the slant range `d` via the law of cosines in the triangle (Earth center, receiver, satellite at GPS altitude `R_sat`):

```
d = −R_rx × sin(el) + √(R_sat² − R_rx² × cos²(el))
```

`sat_ecef = rx_ecef + d × unit_ecef`

**Step 2 — ECEF position to orbital elements**

For a circular orbit with zero eccentricity and zero argument of perigee, `satpos()` at `t = toe` reduces to:

```
Z = A × sin(uk) × sin(inc0)   →   ypk = Z / sin(inc0)
                                   xpk = √(A² − ypk²)
```

where `uk = atan2(ypk, xpk)` is the argument of latitude. The ascending node `ok` is solved from the 2×2 linear system in X and Y:

```
ok = atan2(Y × xpk − X × ypk_ci,  X × xpk + Y × ypk_ci)
```
where `ypk_ci = ypk × cos(inc0)`.

The final elements stored in `ephem_t`:
```
m0   = uk        (mean anomaly = arg. of latitude for circular orbit)
aop  = 0
omg0 = ok + OMEGA_EARTH × toe.sec   (invert Earth rotation from satpos())
```

### GPS Inclination Limit

GPS satellites only cover latitudes up to ~55° N/S. If the receiver is at a latitude higher than the orbital inclination and requests an overhead satellite, the Z component of the required ECEF position would exceed what the orbit can reach. The tool clamps `ypk` to the maximum reachable value and prints a warning — the satellite will appear at a slightly lower elevation than requested.

```
WARNING: PRN 25 requested position exceeds GPS inclination band (~55 deg), clamped.
25  180.0  70.4  20479040.8   2.0
```

## Navigation Message Consistency

Because the synthetic ephemeris is a fully populated `ephem_t`, the navigation message generation path is unchanged:

1. `eph2sbf()` encodes the synthetic orbital parameters into GPS subframe bit structures
2. `generateNavMsg()` adds TOW and computes parity checksums
3. A receiver that acquires the signal and decodes subframes 1–3 will reconstruct orbital parameters that are consistent with the observed pseudorange and Doppler

The clock terms (af0, af1, af2, tgd) are zero, so there is no artificial clock bias in the pseudorange.

## Integration with Other Flags

Synthetic satellites are treated identically to real satellites by `-P` and `-A`:

```bash
# OTA scenario: transmit only synthetic PRN 25 (overhead) and jam PRN 8
./gps-sdr-sim -e brdc.nav -l 35.681298,139.766247,10 \
    -S 25:overhead \
    -P 8,25 \
    -A 8:jam_noise \
    -G 5 -v
```

In this example:
- `-S 25:overhead` — synthesize PRN 25 at zenith
- `-P 8,25` — render only PRNs 8 and 25 (partial constellation mode)
- `-A 8:jam_noise` — jam PRN 8 with matched-code noise
- `-G 5` — boost transmitted power by +5 dB

Refer to [jamming-spoofing-scenarios.md](jamming-spoofing-scenarios.md) for attack scenario catalog.

## Simulation Behavior Over Time

Synthetic satellites are stored separately from the real RINEX sets and overlaid onto the active set after each hourly ephemeris refresh. The orbit is fixed at the initial receiver position (`xyz[0]`). As the simulation progresses:

- The satellite moves along the synthetic circular orbit normally, so its az/el will drift slightly from the requested value over time (same as any real satellite would drift)
- For short runs (< 5 minutes) the drift is under 0.1°
- For longer runs or dynamic receiver motion, the satellite will move along the synthesized orbit trajectory

## Validation

Use `-v` to confirm the synthetic satellite appears in the channel list at the expected position:

```
PRN   AZ    EL   RANGE(m)  IONO(m)
25  180.0  90.0  20188812.7   4.0   ← overhead, GPS altitude
30  180.0  45.0  21658399.0   5.6   ← correct az/el
05    3.2 -12.4  27068184.8   6.9   ← forced, below horizon
```

Realistic range values (~20,000–28,000 km) confirm the orbit geometry is correctly computed.

## Error Cases

| Condition | Behavior |
|---|---|
| `PRN:force` but PRN has no ephemeris in RINEX | Warning printed, satellite skipped |
| Elevation outside −5° to 90° | Parse error, simulation exits |
| Azimuth outside 0° to 360° | Parse error, simulation exits |
| Receiver latitude > ~55° requesting overhead | Warning + clamping to max reachable elevation |
| More than 16 total channels (real + synthetic) | Excess silently not allocated (existing limit) |
| Invalid PRN format (`abc:overhead`) | Parse error, simulation exits |

## Source References

| Function             | Location           | Purpose                                           |
| -------------------- | ------------------ | ------------------------------------------------- |
| `parseSynthConfig()` | `gpssim.c`         | Parse `-S` argument into `synth_config_t`         |
| `azel2satpos()`      | `gpssim.c`         | Convert az/el + receiver position to ECEF         |
| `synthEphemeris()`   | `gpssim.c`         | Build complete `ephem_t` for a circular GPS orbit |
| `overlaySyntheticEphemerisSet()` | `gpssim.c` | Merge synthetic PRNs into the active ephemeris set |
| `allocateChannel()`  | `gpssim.c`         | Modified to bypass elevation check for synth PRNs |
| `synth_config_t` / `synth_ephem_store_t` | `gpssim.h` | Per-PRN synthetic config and stored ephemerides |
