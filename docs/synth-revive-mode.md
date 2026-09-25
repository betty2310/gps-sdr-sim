# Synthetic satellite revive mode

Current reference, 2026-09-22. The shared revive renderer is implemented.
X300 supports frozen-RINEX scenario revive and F9P-time revive with finite TX,
continuous `--stream`, no-TX `--check-start`, and offline `--dry-run`.
Use the [command templates](../command.md)
and [X300 mode contract](x300-usage.md).

## Meaning and implementation

`-S PRN:revive` constructs a synthetic satellite from that PRN's own past
broadcast ephemeris. `-P` selects the normal-mode constellation to render.
For example, selecting `-P 14,22,30` with
`-S 14:revive,22:revive,30:revive` requests those three revived channels.
These numbers are syntax examples, not a current-sky recommendation.

The intended geometry invariant is:

~~~text
satpos(revived_ephemeris, scenario_epoch)
    == satpos(template_ephemeris, template_toe)
~~~

This is a simulator/encoded-orbit consistency target, not a guarantee of zero
measured pseudorange residual, receiver satellite use, or GPS-aligned RF.

The current implementation is in [gpssim.c](../gpssim.c) and
[gpssim.h](../gpssim.h):

- `parseSynthConfig` accepts classic, clone or revive families; mixing families
  in one specification is rejected.
- `scanEphemerisForRevive` searches the target's own historical RINEX entries.
- `reviveEphemerisFromTemplate` copies the template, sets synthetic TOE/TOC on
  the broadcast time quantum, and adjusts mean anomaly, inclination and node
  longitude so evaluation at the scenario epoch retains the past geometry.
- The transformation propagates its clock term, changes IODE/IODC and marks the
  synthetic entry healthy. That is generated navigation, not a new authentic
  broadcast ephemeris.
- The overlay lifecycle feeds the ordinary range, C/A, navigation and IQ
  pipeline. The frozen input file remains separately identified by its hash.

## Current scan policy

| Setting | Current value |
|---|---:|
| Preferred lookback | 4 hours |
| Maximum lookback | 8 hours |
| Search step | 15 minutes |
| Minimum past elevation | 20 degrees |
| Revive refresh interval | 30 minutes |
| Issue increment scale | 2 hours, modulo 256 |

These are implementation constants, not user-specified per-PRN delays.
A requested `revive=-7200` form is unsupported. The scanner uses actual
template TOE and may select a different lookback from the preference.

Keep a RINEX with sufficient historical target coverage. In normal X300 mode,
when `-P` explicitly selects only PRNs configured as `revive`, no current
ephemeris set is required. Each target still needs an acceptable historical
template under the scan policy above. Runs that also generate non-revived
satellites retain the current-set check. Missing templates or no acceptable
past elevation abort startup; there is no fallback to fabricated azimuth/elevation.

## X300 compatibility

| Combination | Status |
|---|---|
| `-e FILE -P LIST -S ...:revive` with scenario epoch | Supported |
| Above plus `--dry-run -d SECONDS` | Supported, no UHD access |
| `--ublox-time-tcp HOST:PORT -e FILE -l ... -P ... -S ...:revive --dry-run` | Supported, automatically estimated GPS epoch; no UHD/RF |
| `--ublox-time-tcp` with `-d SECONDS` or `--stream` | Supported, fixed hardware start at an estimated current GPS epoch |
| `--ublox-time-tcp` with `--check-start` | Supported, actual rate/PPS and dated prebuffer; no TX streamer |
| Above plus external 10 MHz/PPS and RF B selection | Implemented TX path; RF acceptance unverified |
| Revive plus `--ublox-tcp` or `--ublox-replay` | Rejected |
| `-e` plus `--ublox-tcp` | Rejected |
| X300 Trimble time-tag, host-clock `-n` or ppm options | Removed |
| Matched-code with a synthetic revive alignment model | Separate navigation-free waveform; does not transmit revived navigation |

A scenario epoch labels sample zero; it does not schedule transmission at that
GPS instant. `--ublox-time-tcp` separates live TIMEGPS from frozen RINEX and
maps sample zero to a future X300 deadline without `-t`. Offline dry run uses
a future host instant instead. Target PRNs need no authentic live SFRBX.
It requires no F9P-to-X300 PPS cable. Minimum-delay filtering reduces arrival
jitter, while the remaining USB/TCP delay and RF-path corrections are explicit
estimates. Absolute GPS-to-RF alignment remains unverified. See the
[remaining implementation plan](x300-live-sky-mixtracking-research-plan.md#3-supported-modes-and-the-revive-gap).

## PRN selection and receiver interpretation

`revive_candidates` accepts a frozen RINEX, explicit GPS-calendar epoch,
position, and optional `--obs-prns` from the reference receiver. It still has
legacy RTCM input, but does not read UBX directly. The X300/F9P workflow uses the
manual observed-PRN list; Trimble is not required.

An `OK` row is a template/selection result. Without fresh reference observations
it does not establish absence from the live sky. Record the observation age,
predicted visibility through the run, selected templates and any collision
policy. Revive intentionally changes a satellite's modeled state, so a
controlled mixtracking experiment must measure the resulting consistency.

The [2026-04-22 receiver study](issues/synth-revive-ver3-current-state.md) is
historical bladeRF evidence and has unresolved receiver-use outcomes. Its
Trimble constants are not X300 calibration.

In the current X300 qualification, no RF has been sent. The offline fixture
rendered 2,812,500 samples without clipping on 2026-09-22, as recorded in the
[current plan](x300-live-sky-mixtracking-research-plan.md). Relevant existing tests are
[test_parse_synth_revive.c](../tests/test_parse_synth_revive.c),
[test_revive_transform.c](../tests/test_revive_transform.c), and
[test_revive_scan.c](../tests/test_revive_scan.c), included in `make test`.

For acceptance, distinguish tracking, code/carrier validity, half-cycle/lock,
decoded navigation, NAV-SAT satellite use, PVT and absolute RF timing.
Successful template transformation alone does not prove any receiver outcome.
