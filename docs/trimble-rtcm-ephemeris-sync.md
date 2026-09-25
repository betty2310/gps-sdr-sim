# Shared RTCM navigation and the X300 migration

Updated 2026-09-22. The old plan to use Trimble for X300 time and navigation is
superseded. **Current X300 live input is ZED-F9P raw UBX over TCP.**
Use [X300 usage](x300-usage.md) and [command.md](../command.md).

## Remaining legacy consumers

`player/rtcm3_nav.hpp/.cpp` remains available to legacy bladeRF navigation
and candidate tooling. `tools/rtcm3_inspect.c` inspects RTCM. These consumers
are separate from the X300 UBX parser and its GPS LNAV readiness checks.

`revive_candidates` can use RTCM observations and ephemeris, or a manual
`--obs-prns` list. For the present F9P setup, pass a fresh reference observation
list rather than assuming the candidate tool can parse UBX/TCP. A RINEX remains
the historical template source for revive.

The legacy bladeRF workflow is described in the
[bladeRF matched-code guide](realtime-code-aligned-matched-code-bladerf.md).
Requalify its hardware, data source and timing before use. The bladeRF is an
IQ receiver in the current X300 setup; old Trimble lead/offset/ppm constants
are not current X300 calibration. Keep credentials out of checked-in commands.

## X300 command migration

| Old X300 input/control | Current behavior |
|---|---|
| `--trimble-rtcm-*` | Rejected; use `--ublox-tcp host:port` for live navigation |
| `--trimble-time-tag-*` and related calibration options | Rejected; local PPS plus explicit scenario or estimated UBX epoch |
| `--gps-time-ppm` / `--tx-time-scale-ppm` / `-n` | Removed |
| Frozen `-e` navigation | Supported scenario source; exclusive with UBX |
| Historical RINEX revive plus current F9P time | Not yet supported together |

RTCM ephemeris is not an entire received navigation bitstream. Similarly,
decoded UBX SFRBX ephemeris does not by itself establish identical transmitted
LNAV pages or physical GPS-to-RF timing. Those remain separate qualification
requirements in the [current plan](x300-live-sky-mixtracking-research-plan.md).
