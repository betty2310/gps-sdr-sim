# Synthetic Satellite Time-Basis Mismatch With Live Sky Signals

## Summary

Latest validation still shows a mixed real-plus-synthetic timing mismatch at the
receiver.

The receiver tracks both live-sky GPS and injected GPS PRNs, but the full set does
not stay on one consistent pseudorange time basis. The result is an unstable mixed
measurement set: the receiver keeps a `3D` fix, but spoof detection triggers and
some tracked satellites carry a large common pseudorange offset.

Current status: the issue is still open.

## Latest Test Setup

Validation captures:

- [COM3___9600_260403_024818.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260403_024818.ubx)
- [COM4___9600_260403_024827.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM4___9600_260403_024827.ubx)

Command used in this run:

```bash
gps-sdr-sim -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 8,3,4,7 -S 3:0/60,5:45.0/55.0,4:90/45,7:180/30 \
  -n -o - | ./player/bladeplayer -f - -b 16 -g -35
```

Important note:

- The actual injected PRN set is defined by `-P`, so this run transmitted `3,4,7,8`.
- The `-S` list does not match the transmit list because it names `5` and omits `8`.
- Analysis of this run therefore treats PRN `5` as live sky, not synthetic.

## Current Findings

Common behavior across both captures:

- Both logs contain tracked live-sky and injected GPS PRNs at the same time.
- Both logs keep a `3D` fix in all `NAV-PVT` epochs.
- Both logs keep the same RAWX `rcvTow` fractional part of `0.991`.
- Both logs show a persistent residual pseudorange split of about `0.233 s`
  on the mismatched group, equivalent to about `69,900 km`.
- PRN `7` remains in a normal GPS pseudorange range throughout both captures.
- PRN `3` still shows a separate large ambiguity of about `1.307 s` before later
  returning near the normal range.

Capture-specific summary:

- [COM3___9600_260403_024818.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM3___9600_260403_024818.ubx)
  has `80` RAWX epochs over about `84 s` and remains mostly unstable.
- [COM4___9600_260403_024827.ubx](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/COM4___9600_260403_024827.ubx)
  has `148` RAWX epochs over about `172 s` and becomes much more stable after
  startup, with most later epochs looking mixed-normal.

## Spoof Detection

`NAV-STATUS` shows that u-blox does detect the mixed condition:

- `COM3___9600_260403_024818.ubx`: starts as `no_spoof`, then switches to
  `spoof_indicated` for `90` epochs, then returns to `no_spoof`
- `COM4___9600_260403_024827.ubx`: starts as `multi_spoof`, then changes to
  `spoof_indicated` for the rest of the capture

So the receiver can continue producing a `3D` fix while still flagging the signal
environment as spoofed or inconsistent.

## Current Interpretation

The issue is no longer a total failure to track injected satellites. The current
problem is that live-sky and injected signals are not merged into one deterministic
measurement set.

In this state, the receiver appears able to:

- track injected satellites
- keep a navigation fix from a consistent subset
- reject or de-weight the inconsistent subset
- raise spoof-detection flags while continuing to operate

This means the receiver is still not behaving as if all visible PRNs share one
coherent GNSS time basis.

## Known Limitation Of These Captures

These logs do not include `UBX-NAV-SAT`, so they cannot show which satellites were
finally marked `svUsed=Y` in the navigation solution.

## Root Cause Analysis

The current `-n` mode derives GPS time from the PC wall clock via
`resolveCurrentGpsTime()` (`gpssim.c:143-180`). This reads `CLOCK_REALTIME`,
adds the hardcoded +18 s GPS-UTC leap offset and the `-r` lead time, and sets
`g0`. The bladeRF player independently latches its own wall-clock reading and
schedules TX against its free-running sample counter.

Neither component has any knowledge of actual GPS time. Both trust the PC wall
clock, which is typically accurate only to 1-50 ms via NTP. The receiver,
however, tracks live-sky pseudoranges anchored to true GPS time (accurate to
nanoseconds). The residual split of ~0.233 s (~70,000 km) is the combined
wall-clock error plus un-calibrated pipeline latency.

### Time error budget

For the receiver to treat all SVs as sharing one coherent time basis:

| Time error    | Range error   | Receiver behavior                                    |
|---------------|---------------|------------------------------------------------------|
| < 50 ns       | < 15 m        | Transparent — within normal pseudorange noise         |
| 50-200 ns     | 15-60 m       | Accepted, minor residuals, spoofing detection silent  |
| 200 ns - 1 us | 60-300 m      | Marginal — spoofing flags may trigger, de-weighting   |
| > 1 us        | > 300 m       | Flagged / excluded from navigation solution           |
| 233 ms (now)  | ~70,000 km    | ~2.6x GPS orbit radius — impossible to absorb         |

The practical target is **< 200 ns**. No PC-based clock source can reach this.

## Next Validation

The next capture should:

- use matching PRN definitions in both `-P` and `-S`
- include `UBX-NAV-SAT` logging
- repeat the same OTA test so the remaining residual split can be measured again

Recommended corrected PRN definition for the next run:

```bash
gps-sdr-sim -e hour0910.26n -l 21.0047844,105.8460541,5 \
  -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \
  -n -o - | ./player/bladeplayer -f - -b 16 -g -35
```

## Recommended Resolution: Best Plan With Current Hardware

The current best-available hardware path is no longer the full external-GPSDO design.
The practical plan for this repository is now:

- **Transmitter:** USRP X300 with UBX daughterboard
- **Host link:** `1 GbE` from the control computer to the X300
- **Time truth source:** a u-blox GNSS receiver used to read current GPS week/TOW
- **Optional timing pulse:** use the u-blox `1 PPS` only if the receiver exposes a timing pulse output and it can be wired safely to the X300 `PPS/TRIG IN`
- **Frequency reference:** X300 internal oscillator for now
- **Calibration model:** fixed end-to-end delay measured and compensated in software

### Realistic target with current hardware

The target must be relaxed to match the missing external `10 MHz` reference.

Two operating levels are realistic:

| Available timing source | Practical target | Stretch target |
|-------------------------|------------------|----------------|
| u-blox GPS week/TOW only | `< 1 ms` | `< 100 us` |
| u-blox GPS week/TOW + usable `1 PPS` into X300 | `< 10 us` | `< 1 us` |

The current `~233 ms` error is dominated by the bad start-time architecture, so reducing the mismatch to **well below `1 ms`** is realistic even without an external `10 MHz` source.

### Does `gpssim.c` need to change?

Yes.

The logic in `gpssim.c` still needs to change, even for this reduced target:

1. **Required change:** the current `-n` logic in `resolveCurrentGpsTime()` cannot remain the precise start path, because it derives `g0` from the PC wall clock.
2. **Required change:** the simulator must accept an explicit GPS epoch derived from the u-blox receiver.
3. **Optional later change:** if validation still stalls above the revised target, add finer timing support in the code-generation path.

The first two changes are mandatory for the current hardware plan.

## Detailed Implementation Plan

### Goal

Make the injected PRNs share a much tighter time basis with live-sky GPS using the X300 plus a u-blox-derived GPS epoch, and reduce the mixed-sky timing mismatch from the current `~233 ms` to the best level possible with the available hardware.

### Hardware Plan

Use the following hardware arrangement:

- USRP X300 or X310 with UBX-160 daughterboard
- control computer over `1 GbE`
- u-blox receiver as the GPS time truth source
- OTA transmit antenna and fixed RF attenuation chain
- fixed test geometry during calibration and validation

Base hardware behavior:

- X300 `clock_source=internal`
- X300 `time_source=internal` by default
- if the u-blox provides a usable timing pulse and wiring is safe, use `time_source=external` from u-blox `1 PPS`

Notes:

- `1 GbE` is sufficient for the current `2.6 Msps` path
- no external `10 MHz` is available, so the X300 cannot be fully GPS-disciplined in frequency
- this plan improves absolute start timing and common bias; it does not create a true nanosecond-class reference chain

### Software Plan

The software work should be split into four units.

The preferred software shape for the precision path is now:

- one **C/C++** X300 control program
- direct use of the UHD C++ API
- direct in-process sample generation
- direct timed transmit from memory to the USRP
- no pipe and no intermediate file in the final timing path

This removes one more source of startup uncertainty and keeps the generator epoch,
timed TX scheduling, and sample stream under one process and one control flow.

#### Unit 1: Replace wall-clock `-n` start with explicit GPS epoch control

**Files:**

- Modify: `gpssim.c`
- Modify if needed for declarations/constants: `gpssim.h`
- Update docs: `README.md`, `docs/x300-spoofing-setup.md`

**Required behavior:**

- Add a mode that accepts an explicit GPS epoch instead of deriving one from `CLOCK_REALTIME`
- Feed that epoch from a u-blox receiver message path, not from PC wall clock
- Keep `-n` only as a convenience/debug mode
- Document that `-n` is not the recommended path for mixed-sky alignment work

**Recommended CLI shape:**

```text
--gps-week <week>
--gps-tow <seconds>
--tx-advance-ns <nanoseconds>
```

`--tx-advance-ns` remains the fixed compensation term used to advance or delay the simulated epoch to compensate the constant end-to-end bias.

**Recommended u-blox inputs:**

- GPS week from a navigation/time message
- GPS TOW from a navigation/time message
- optional PPS-aligned trigger if the u-blox timing output is available

#### Unit 2: Build a single C/C++ X300 generator-plus-transmitter

**Files:**

- Modify: `gpssim.c`
- Modify if needed for shared interfaces: `gpssim.h`
- Create: `player/x300tx.cpp`
- Update build files: `Makefile` and/or `player/Makefile`
- Update docs: `docs/x300-spoofing-setup.md`, `README.md`

**Required behavior:**

- Refactor the sample-generation path so it can be called from a C/C++ transmitter program without shell piping
- Open the X300 with the UHD C++ API and `clock_source=internal`
- set `time_source=internal` by default
- support `time_source=external` only when a working u-blox `1 PPS` path is available
- schedule TX for a future USRP hardware time using timed metadata
- generate samples in memory and send them directly to UHD
- start the first sample at a known hardware time rather than immediate stream-now replay

**Important note:**

The old `gps-sdr-sim-uhd.py` path is no longer the preferred precision architecture because it still leaves the timing path split across separate tools and does not provide a clean integrated start model.

The new single-process path must:

- consume a GPS week/TOW start epoch
- set device time from a chosen start model
- schedule TX from an explicit hardware time plan
- distinguish the “u-blox week/TOW only” mode from the “u-blox week/TOW + PPS” mode
- eliminate pipe and file handoff from the final precision path
- keep enough buffered samples ready before the timed TX start to avoid underruns

#### Unit 3: Add fixed-delay calibration support

**Files:**

- Modify: `gpssim.c`
- Update docs: `docs/issues/synthetic-satellite-time-basis-mismatch.md`, `docs/x300-spoofing-setup.md`

**Required behavior:**

- support a calibrated common timing offset term in nanoseconds
- apply that term at scenario start so the simulated pseudoranges are advanced or delayed to match RF arrival at the DUT as closely as current hardware allows

**Delay sources to include in calibration:**

- USRP digital pipeline delay
- DAC / DUC / FPGA path delay
- coax and attenuator chain delay
- antenna separation air-path delay
- any fixed connector or filter group delay

**Calibration procedure:**

1. Read GPS week/TOW from the u-blox receiver.
2. If available, verify the u-blox timing pulse path before starting the X300.
3. Place the TX antenna at a fixed test geometry above the DUT antenna.
4. Generate a test capture with explicit GPS epoch and zero delay compensation.
5. Log `UBX-RXM-RAWX`, `UBX-NAV-SAT`, `UBX-NAV-PVT`, and spoof status.
6. Measure the common pseudorange residual between synthetic and real-sky groups.
7. Convert the common residual to nanoseconds.
8. Apply the opposite sign as `--tx-advance-ns`.
9. Repeat until the residual reaches the best repeatable level for the selected hardware mode.

#### Unit 4: Evaluate whether finer timing support is still needed

**Files:**

- Modify if needed: `gpssim.c`
- Update docs: `docs/signal-generation-pipeline.md`

**Reason this may still matter:**

At `2.6 Msps`, one sample period is about `384.6 ns`. That is no longer the first blocker, but it can still become the next floor once the gross start-time error is removed.

**Decision rule:**

- If the end-to-end residual already meets the revised target, stop there.
- If the residual stalls above the revised target and behaves like a quantization floor, add finer timing support.

**Preferred follow-up if needed:**

- keep code phase in higher-resolution form
- support a fractional common-delay term
- avoid tying the entire timing correction to integer sample boundaries

## File-Level Change Summary

### `gpssim.c`

Must change.

Required changes:

- add explicit GPS epoch input path
- separate “approximate stream-now mode” from the u-blox-driven absolute-time mode
- add fixed nanosecond delay compensation term
- document that PC wall clock is not the recommended timing source for the X300 path

Likely later changes if validation demands it:

- add fractional timing support to reduce sample-grid quantization effects

### `player/x300tx.cpp`

Must be created.

Required responsibilities:

- read GPS week/TOW from the selected control path
- configure the X300 through the UHD C++ API
- set the start model for the selected timing mode
- request samples directly from the integrated generator logic
- send samples directly to UHD with timed TX metadata
- support two operating modes:
  - X300 internal timing only
  - X300 internal frequency + external `1 PPS` time source, if u-blox timing pulse is available

### `gps-sdr-sim-uhd.py`

Can remain as a legacy or prototype path, but it should no longer be treated as the main precision implementation path.

### `docs/x300-spoofing-setup.md`

Must change.

Required updates:

- describe the current-hardware path based on u-blox GPS time truth
- explain that `1 GbE` is acceptable for the current sample rate
- distinguish the two timing modes:
  - week/TOW only
  - week/TOW + optional `1 PPS`
- document the revised target and calibration workflow

## Validation Plan

The validation plan should now be split by available timing mode.

### Mode A: u-blox GPS week/TOW only

Accept only if all of the following are true:

- synthetic PRN definitions match across `-P` and `-S`
- `UBX-NAV-SAT` confirms which PRNs are used in the navigation solution
- the common synthetic-versus-real pseudorange residual is measured directly
- the residual after calibration is below `1 ms`
- repeat runs stay in the same order of magnitude and do not jump back to the current `~233 ms` failure mode

### Mode B: u-blox GPS week/TOW + usable `1 PPS`

Accept only if all of the following are true:

- the X300 reports valid PPS input
- the common synthetic-versus-real pseudorange residual is measured directly
- the residual after calibration is below `10 us`
- repeated runs are stable enough to justify pursuing the `< 1 us` stretch goal

Recommended measurement sequence:

1. Run with the X300 path active but no synthetic PRNs, to verify the setup is stable.
2. Run with one synthetic PRN added and measure the common residual.
3. Increase to the target PRN set and confirm the residual remains tightly grouped.
4. Repeat across several independent runs to verify the result is repeatable.

## Success Criteria

This issue should be considered substantially improved when:

- the gross `~233 ms` split is removed
- synthetic and real-sky PRNs share one much tighter time basis than before
- the start path no longer depends on host wall clock timing
- the calibrated residual meets the best available target for the selected timing mode

## Recommendation

Use the X300 path with a u-blox-derived GPS epoch as the current working solution.

Treat the following as **mandatory** work now:

- change `gpssim.c` start-time logic
- build the integrated C/C++ UHD transmitter path
- add fixed-delay calibration support
- use the u-blox receiver to provide GPS week/TOW to the control path

Treat the following as **conditional** work:

- use u-blox `1 PPS` only if a safe and reliable timing-pulse path exists
- add sub-sample timing support in `gpssim.c` only if validation stalls above the revised target floor
