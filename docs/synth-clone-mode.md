# Synthetic satellite clone mode

Current reference, 2026-09-22. Clone is implemented in the shared synthetic
ephemeris layer. X300 consumes donor navigation from the reference ZED-F9P
UBX/TCP path; legacy bladeRF uses its own RTCM path.

Clone copies **another PRN's** ephemeris into a selected target slot. Revive
uses the **target PRN's own past** ephemeris. The F9P time-only revive
mode is documented separately in [revive mode](synth-revive-mode.md).

## Contract

`-S 4:clone=5,10:clone=11` defines target 4 from donor 5 and target 10 from donor
11. In normal X300 mode, `-P 4,10` selects the transmitted targets. Each PRN must
be in the valid GPS PRN range. Classic, clone and revive specifications cannot
be mixed in one `-S` argument.

[`cloneEphemerisFromDonor`](../gpssim.c) copies the donor's valid `ephem_t`.
The synthetic overlay lets the ordinary range/code/navigation renderer use that
state under the target's C/A code. It does not recreate the donor's independently
received carrier phase, authentic SF4/SF5 schedule or all broadcast bits.

## Current X300 input rules

Use `--ublox-tcp host:port` for live clone, without `-e` or an explicit scenario
epoch. The player maps selected targets to donor PRNs for readiness checks and
requires fresh, healthy, issue-consistent GPS L1 C/A SF1–SF3 for those donors,
plus valid reference time, position and RAWX.

The live reader continues independently. Stale required state or disconnect
fails the run; X300 does not indefinitely reuse cached donor ephemeris after
loss of the source. The fixed future deadline and unknown absolute GPS
uncertainty are the same as for other live UBX waveforms.

For a no-RF preparation check, choose donors from the new reference capture:

~~~bash
TARGET_PRNS='REPLACE_WITH_TARGET_PRNS'
CLONE_SPEC='REPLACE_WITH_TARGET:clone=DONOR'

rtk proxy ./x300tx --ublox-tcp 127.0.0.1:5019 --check-start \
  -P "$TARGET_PRNS" -S "$CLONE_SPEC" --ublox-warmup-sec 90 \
  --addr 192.168.10.2 --channel 1 --rate 2500000 \
  --start-lead-sec 2 --prebuffer 50 \
  --manifest clone-start-check.json
~~~

Use fresh artifact paths. This template has not been qualified as a current
hardware clone experiment. It creates no TX streamer. General receiver/clock
setup and controlled-RF requirements are in [command.md](../command.md).

## Interpretation and evidence

Duplicate modeled geometry, target-PRN conflicts and receiver consistency checks
can affect acquisition and PVT. Copying ephemeris does not guarantee a particular
spoofing flag, zero residuals, or mixed satellite use. Keep the authentic
reference independent and record target and donor PRNs explicitly.

The earlier April plan described Trimble/RTCM as the future X300 source. That
X300 dependency is obsolete. [Shared RTCM notes](trimble-rtcm-ephemeris-sync.md)
now apply only to legacy consumers; no new clone RF result is claimed here.

For the current bench, antennas are exposed and GPS TX remains off. Physical
GPS association, RF-path delay, timestamped IQ and per-PRN comparisons remain
in the [mixtracking plan](x300-live-sky-mixtracking-research-plan.md).
