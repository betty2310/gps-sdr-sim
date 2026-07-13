# Real-Time Code-Aligned Matched-Code Transmission with `x300tx`

> **Status:** Implemented and transmitter-accepted on a USRP X300
>
> **Date:** 2026-07-13
>
> **Scope:** Finite, simulator-aligned GPS L1 C/A matched-code experiments in
> an authorized conducted path or verified shielded enclosure
>
> **Related:** [V2 specification](realtime-code-aligned-matched-code-x300-spec.md),
> [V1 offline matched-code dataset](code-aligned-matched-code-dataset-spec.md),
> and [general X300 usage](x300-usage.md)

## Start Here

`x300tx` can now generate these two signals in one process:

1. a synthetic clean GPS L1 C/A constellation; and
2. an independent, navigation-data-free matched-code source aligned to selected
   synthetic PRNs in code phase, carrier Doppler, and code-rate Doppler.

It mixes the sources sample-for-sample, quantizes the composite once to SC16,
and sends one finite hardware-timed burst through one X300 TX channel. There is
no intermediate IQ file in the live path.

This mode is for the following experiment:

```text
synthetic clean GPS ----\
                         +--> digital mixer --> one timed X300 TX burst
matched-code source ----/
```

It is not:

- alignment to authentic live-sky GPS;
- a coherent replica or spoofing source;
- a jammer-only `jammertx` mode;
- a continuous or operator-stopped transmission; or
- proof of RF waveform fidelity or receiver response.

The accepted V2 result is transmitter-only evidence: the X300 completed the
exact planned finite burst with zero UHD underflows, sequence errors, time
errors, and digital clipping.

## Controlled-RF Safety Boundary

GPS-band interference can disrupt safety-critical services and may be illegal
to radiate. Run the live command only after all of these conditions are true:

1. the X300 TX port is connected to an authorized conducted RF path or is
   inside a verified shielded enclosure;
2. the active setup has approved attenuation, isolation, a DC block where
   required, and receiver-input power protection;
3. leakage and the emergency stop procedure have been checked;
4. the selected X300 address, channel, antenna, and gain match the current RF
   power plan; and
5. the timing and RF setup have a current calibration record.

`--confirm-controlled-rf` is only an operator acknowledgement. The program
cannot detect an antenna, an unsafe power level, poor shielding, or RF leakage.
Do not use the examples in an open-air setup.

The live example deliberately contains non-numeric placeholders for gain and
timing calibration. It will fail before transmission until the operator
replaces them with values from the active laboratory record.

## Signal and Timing Contract

For each requested target PRN, the clean simulator publishes the state that
governs the next 100 ms epoch:

- first-sample code phase in chips;
- carrier Doppler in hertz;
- code rate in chips per second; and
- clean component gain.

The matched-code renderer consumes that state directly in memory. Its C/A code
and code/carrier NCOs advance from sample zero, including during the clean
prefix. It uses a constant positive data symbol and an independent,
deterministic carrier phase derived from `--matched-code-phase-seed` and the
target PRN. It does not copy LNAV data or clean carrier phase.

The finite burst has this sample-indexed timeline:

```text
sample 0          onset             offset             total samples
| clean prefix | rise--plateau--fall | clean recovery |
```

`--matched-code-ramp` is applied at both edges with a raised-cosine envelope.
Onset must precede offset, and the two ramps must leave a non-empty plateau.

`--matched-code-js-db` is a digital mixer ratio. The tool chooses one fixed
jammer scale before arming, then applies one common clean-and-jammer gain for
SC16 headroom. It does not measure or control TX-port power or receiver-input
J/S.

## Requirements

Software:

- a C/C++ compiler and GNU Make;
- UHD development headers and libraries; and
- Python 3 for the manifest check below.

Experiment inputs:

- a frozen RINEX navigation file available for the complete run;
- a fixed reference position supplied with `-l` or `-c`;
- an explicit ordered target list in GPS PRN range 1 through 32;
- a finite duration, onset, offset, ramp, digital J/S, and phase seed;
- a calibrated GPS start using the Trimble time-tag path, or an explicit GPS
  week/TOW with an approved external or GPSDO time source; and
- explicit live device, RF-path, manifest, trajectory, and calibration values.

Every matched-code target must also be present in `-P` and must remain an
allocated clean channel for the complete run. Targets are never dropped or
replaced dynamically. Live RTCM input and legacy `-A`/`-J` attack settings are
rejected in this mode so the scenario remains frozen and unambiguous.

## 1. Build and Run Hardware-Independent Tests

`x300tx` is an optional UHD target and is not built by the default `make all`:

```sh
pkg-config --modversion uhd
make x300tx
make test-x300tx-matched
./x300tx --help
```

`make test-x300tx-matched` runs dry-run and fail-closed command tests. It never
performs a live UHD send.

## 2. Freeze a Small Research Scenario

Start with one synthetic target and a short finite timeline. Keep the RINEX
file unchanged between dry run and live run.

```sh
RUN_ID="matched-code-$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$(pwd)/runs/$RUN_ID"
mkdir -p "$OUT_DIR"

RINEX="/absolute/path/to/frozen-navigation.rnx"
REFERENCE_LLH="21.0047844,105.8460541,22"
TARGET_PRNS="1"
SYNTHETIC_SATS="1:overhead"

REQUESTED_RATE_HZ="2500000"
DURATION_SEC="3"
ONSET_SEC="1"
OFFSET_SEC="2"
RAMP_SEC="0.01"
MATCHED_CODE_JS_DB="10"
PHASE_SEED="42"

MANIFEST="$OUT_DIR/run.json"
TRAJECTORY="$OUT_DIR/target-trajectory.csv"
```

`1:overhead` creates a controlled synthetic clean target for PRN 1. For a
multi-target experiment, list every target in the same order, for example
`TARGET_PRNS="1,3,8"`, and provide a matching synthetic configuration such as
`SYNTHETIC_SATS="1:overhead,3:90/45,8:180/45"`.

Do not substitute a different target after dry run merely because allocation
fails. Fix the frozen scenario, then repeat dry run and all preflight checks.

## 3. Always Run Dry Run First

Dry run executes the complete scenario, allocation, trajectory, sample-plan,
J/S, and headroom preflight without discovering or opening a UHD device:

```sh
./x300tx \
  -e "$RINEX" \
  -l "$REFERENCE_LLH" \
  -P "$TARGET_PRNS" \
  -S "$SYNTHETIC_SATS" \
  -d "$DURATION_SEC" \
  --rate "$REQUESTED_RATE_HZ" \
  --matched-code-target-prns "$TARGET_PRNS" \
  --matched-code-js-db "$MATCHED_CODE_JS_DB" \
  --matched-code-phase-seed "$PHASE_SEED" \
  --matched-code-onset "$ONSET_SEC" \
  --matched-code-offset "$OFFSET_SEC" \
  --matched-code-ramp "$RAMP_SEC" \
  --manifest "$MANIFEST" \
  --trajectory "$TRAJECTORY" \
  --dry-run
```

The process must exit zero and print that UHD was never opened. The manifest
must have `status: "dry_run"`, zero sent samples, exact planned/rendered sample
counts, and zero clipping.

Check it with:

```sh
python3 - "$MANIFEST" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
run = json.loads(path.read_text())
samples = run["sample_contract"]

assert run["schema"] == "gps-sdr-sim.x300tx-matched-code.v1"
assert run["status"] == "dry_run"
assert run["exit_status"] == 0
assert samples["sent_samples"] == 0
assert samples["rendered_clean_samples"] == samples["planned_samples"]
assert samples["rendered_jammer_samples"] == samples["planned_samples"]
assert samples["quantized_composite_samples"] == samples["planned_samples"]
assert run["measurements"]["clipped_components"] == 0
assert run["scenario"]["full_run_target_allocation_passed"] is True
print("dry-run accepted:", path)
PY
```

Dry run uses the requested rate as a provisional rate. A live run repeats all
rate-sensitive planning using the actual rate returned by UHD.

## 4. Discover and Inspect the Intended X300

Set the address from the current lab inventory, then verify that discovery
returns exactly one X300 or X310:

```sh
X300_ADDR="replace-with-current-x300-address"

uhd_find_devices --args "addr=$X300_ADDR"
uhd_usrp_probe --args "addr=$X300_ADDR"
```

Confirm the reported product and serial against the experiment record. From
the probe output, confirm that the selected channel exists, the antenna name is
valid for that channel, the requested rate is supported, and the intended
clock/time sources are available.

Discovery proves only that UHD can find the device. It does not prove a sample
was transmitted.

## 5. Load the Current Lab and Timing Values

The primary accepted start path uses a calibrated Trimble time tag. Fill every
value from the current controlled setup; do not copy values from an older run:

```sh
TX_CHANNEL="replace-with-approved-channel"
TX_ANTENNA="replace-with-approved-antenna"
APPROVED_TX_GAIN_DB="replace-with-approved-numeric-gain"
CALIBRATION_ID="replace-with-current-calibration-record-id"

TIME_TAG_HOST="replace-with-current-time-tag-host"
TIME_TAG_PORT="replace-with-current-time-tag-port"
FUTURE_START_SEC="replace-with-approved-future-start-offset"
TRIMBLE_TAG_LEAD_MS="replace-with-current-tag-lead-calibration"
TRIMBLE_TX_CAL_NS="replace-with-current-tx-delay-calibration"
GPS_TIME_PPM="replace-with-current-gps-time-rate-calibration"
```

Leaving the placeholders unchanged is intentional: numeric parsing or device
validation will fail before a timed transmission is armed.

An explicit GPS week/TOW is also supported only when the active calibration
uses an external or GPSDO time source. In that case, load the values from that
calibration record:

```sh
GPS_WEEK="replace-with-calibrated-gps-week"
GPS_TOW="replace-with-calibrated-gps-tow"
TX_ADVANCE_NS="replace-with-approved-future-start-lead"
```

Then replace both `internal` source lines and all Trimble options in the live
command with this calibrated fragment:

```sh
--gps-week "$GPS_WEEK" \
--gps-tow "$GPS_TOW" \
--clock-source external \
--time-source external \
--tx-advance-ns "$TX_ADVANCE_NS"
```

Do not use `-n` wall-clock mode for an accepted matched-code live run.

## 6. Run One Finite Live Burst

Reconfirm that the RF path is conducted or shielded before adding
`--confirm-controlled-rf`. Then run the same scenario and waveform values used
for dry run:

```sh
./x300tx \
  -e "$RINEX" \
  -l "$REFERENCE_LLH" \
  -P "$TARGET_PRNS" \
  -S "$SYNTHETIC_SATS" \
  -d "$DURATION_SEC" \
  --rate "$REQUESTED_RATE_HZ" \
  --matched-code-target-prns "$TARGET_PRNS" \
  --matched-code-js-db "$MATCHED_CODE_JS_DB" \
  --matched-code-phase-seed "$PHASE_SEED" \
  --matched-code-onset "$ONSET_SEC" \
  --matched-code-offset "$OFFSET_SEC" \
  --matched-code-ramp "$RAMP_SEC" \
  --addr "$X300_ADDR" \
  --channel "$TX_CHANNEL" \
  --antenna "$TX_ANTENNA" \
  --gain "$APPROVED_TX_GAIN_DB" \
  --clock-source internal \
  --time-source internal \
  --trimble-time-tag-host "$TIME_TAG_HOST" \
  --trimble-time-tag-port "$TIME_TAG_PORT" \
  --trimble-start-offset-sec "$FUTURE_START_SEC" \
  --trimble-tag-lead-ms "$TRIMBLE_TAG_LEAD_MS" \
  --trimble-tx-cal-ns "$TRIMBLE_TX_CAL_NS" \
  --gps-time-ppm "$GPS_TIME_PPM" \
  --manifest "$MANIFEST" \
  --trajectory "$TRAJECTORY" \
  --calibration-id "$CALIBRATION_ID" \
  --confirm-controlled-rf
```

`internal` clock/time sources in this command describe the established Trimble
time-tag workflow, where the calibrated network tag binds the scenario start
to the X300 timed event. Use different source names only when the current
calibration record explicitly covers them.

A successful terminal line has this form:

```text
status=complete sent=<planned>/<planned> underflows=0 sequence-errors=0 time-errors=0 clipping=0
```

Any nonzero exit, interruption, stale start, partial count, underflow, sequence
error, time error, or clipping means the run is not accepted. Do not rename or
edit an incomplete manifest to make it appear complete.

## 7. Validate the Final Manifest and Trajectory

This check enforces the V2 transmitter-only acceptance gates and verifies the
trajectory checksum:

```sh
python3 - "$MANIFEST" <<'PY'
import hashlib
import json
import math
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
run = json.loads(path.read_text())
samples = run["sample_contract"]
measurements = run["measurements"]
hardware = run["hardware"]
trajectory = pathlib.Path(run["trajectory"]["path"])

assert run["schema"] == "gps-sdr-sim.x300tx-matched-code.v1"
assert run["status"] == "complete"
assert run["exit_status"] == 0
assert run["failure_reason"] is None
assert run["safety"]["controlled_rf_confirmed"] is True
assert run["safety"]["calibration_id"]
assert run["scenario"]["full_run_target_allocation_passed"] is True
assert run["timing"]["start_margin_met"] is True
assert hardware["device_type"] in {"x300", "x310"}
assert hardware["actual_antenna"] == hardware["requested_antenna"]

planned = samples["planned_samples"]
assert planned > 0
assert samples["rendered_clean_samples"] == planned
assert samples["rendered_jammer_samples"] == planned
assert samples["quantized_composite_samples"] == planned
assert samples["sent_samples"] == planned
assert measurements["clipped_components"] == 0
assert measurements["underflows"] == 0
assert measurements["sequence_errors"] == 0
assert measurements["time_errors"] == 0
assert math.isfinite(measurements["achieved_js_db"])
assert abs(samples["actual_rate_hz"] - samples["requested_rate_hz"]) <= 0.5

assert trajectory.is_file()
actual_hash = hashlib.sha256(trajectory.read_bytes()).hexdigest()
assert actual_hash == run["trajectory"]["sha256"]

print("live transmitter acceptance passed")
print("samples:", planned)
print("actual rate (Hz):", samples["actual_rate_hz"])
print("achieved digital J/S (dB):", measurements["achieved_js_db"])
print("manifest:", path)
print("trajectory:", trajectory)
PY
```

The manifest uses schema `gps-sdr-sim.x300tx-matched-code.v1` and records:

- frozen RINEX and scenario checksums, reference position, GPS sample-zero, and
  ordered targets;
- phase seed, per-target independent initial phases, envelope sample indices,
  requested/reference/achieved digital J/S, fixed scale, common gain, and
  predicted headroom;
- requested and actual rate, planned/rendered/quantized/sent sample counts;
- X300 type, product, serial, address, channel, antenna, frequency, gain, and
  clock/time sources;
- calibrated timing provenance and timed-start margin;
- trajectory path and SHA-256; and
- clipping, underflow, sequence-error, time-error, interruption, status, and
  exit fields.

Keep the manifest, trajectory, exact command, frozen RINEX file, and referenced
calibration record together. The trajectory is provenance; the live renderer
does not read it back while transmitting.

## Troubleshooting

### A target is unavailable during the full-run preflight

The target must be allocated at every 100 ms boundary from sample zero through
the recovery interval. Check the frozen ephemeris, scenario start, reference
position, `-P`, and `-S`. Do not remove or replace a failed target silently;
change the declared scenario and repeat dry run.

### The command fails before opening UHD

This is the intended fail-closed behavior. Check for a missing or duplicate
target, non-finite value, missing duration/J/S/seed/onset/offset, invalid ramp,
legacy `-A`/`-J` conflict, live RTCM options, an unwritable manifest or
trajectory, or a missing explicit device/RF/calibration field.

### The requested sample rate is rejected

Use the established 2.5 Msps X300 profile unless the simulator and hardware
contract has been extended and tested. A live run is rejected if UHD coerces
the actual rate more than 0.5 Hz away from the requested rate.

### Discovery finds no device, several devices, or a non-X300 product

Check the host interface, subnet, X300 power, UHD image compatibility, and the
explicit address. Re-run `uhd_find_devices` and `uhd_usrp_probe`. The mode will
not select an ambiguous or different device automatically.

### The future start becomes stale

Increase the approved future-start offset or reduce preflight latency, then
repeat the entire run. The tool will not fall back to an immediate,
uncalibrated start.

### The manifest reports underflow, sequence error, or time error

Treat the run as failed. Stop, preserve the manifest, check host load and the
X300 transport path, and rerun only after the cause is understood. Increasing
prebuffering may help host-side preparation, but it does not excuse a nonzero
UHD fault counter.

### The manifest reports clipping

Treat the run as failed and preserve it for diagnosis. The common-gain planner
is designed to retain at least 1 dB of predicted SC16 headroom, so runtime
clipping indicates that the scenario or bound needs investigation.

## Research Interpretation

`status: "complete"` proves that the planned digital composite was generated
and accepted by the X300 streaming path as one finite timed burst. It does not
prove any of the following:

- TX-port or receiver-input power;
- RF code phase, Doppler, spectrum, spurs, or waveform fidelity;
- cable, front-end, propagation, multipath, AGC, or enclosure behavior;
- receiver acquisition, tracking, `svUsed`, pseudorange residual, or PVT; or
- mitigation performance.

Those claims require calibrated RF capture and/or receiver-side evidence in a
separate experiment. Also repeat research conclusions across explicit phase
seeds; one accepted seed proves execution, not phase-robust receiver behavior.

## Provisioned X300 Acceptance Record

The implementation was exercised on 2026-07-13 after
`uhd_find_devices --args addr=<explicit-address>` discovered exactly one X300.
The controlled finite acceptance run used the actual 2.5 Msps UHD rate and
completed with:

| Gate | Result |
| --- | ---: |
| Planned samples | 1,000,000 |
| Clean rendered samples | 1,000,000 |
| Matched-code rendered samples | 1,000,000 |
| Composite quantized samples | 1,000,000 |
| X300 sent samples | 1,000,000 |
| UHD underflows | 0 |
| UHD sequence errors | 0 |
| UHD time errors | 0 |
| Digital clipping | 0 |
| Final status | `complete` |
| Process exit status | 0 |

The device gain and calibration terms are intentionally not published as
reusable handbook values. They remain tied to the experiment's live manifest
and current laboratory calibration record.
