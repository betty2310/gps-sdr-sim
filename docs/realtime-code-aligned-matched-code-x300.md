# Selective-PRN Matched-Code Jammer-Only Transmission with `x300tx`

> **Status:** Software implemented; continuous controlled-hardware acceptance pending
>
> **Date:** 2026-07-13
>
> **Scope:** Continuous GPS L1 C/A matched-code interference in an authorized
> conducted path or verified shielded enclosure
>
> **Related:** [V2 specification](realtime-code-aligned-matched-code-x300-spec.md),
> [V1 offline matched-code dataset](code-aligned-matched-code-dataset-spec.md),
> and [general X300 usage](x300-usage.md)

## Start Here

In matched-code mode, `x300tx` transmits a jammer-only SC16 stream. The stream
contains navigation-data-free matched-code components for exactly the PRNs in
`--matched-code-target-prns`.

The clean GPS simulator still runs internally to supply code phase, carrier
Doppler, and code-rate Doppler. Its IQ is written to scratch storage and
discarded. It is never mixed into or copied to the X300 output.

```text
RINEX + position + GPS time + optional -S
                    |
                    v
          internal alignment state
                    |
                    v
       selected matched-code components
                    |
                    v
          jammer-only SC16 stream
                    |
                    v
             one X300 channel
```

The source is active at sample zero and remains active until the operator
stops it with `Ctrl-C` (`SIGINT`) or `SIGTERM`. There are intentionally no
duration, onset, offset, ramp, clean prefix, or recovery controls.

This mode is not:

- a clean-GPS generator;
- a clean-plus-jammer digital composite;
- alignment to authentic live-sky GPS;
- a coherent navigation-message replica or spoofing source; or
- proof of RF fidelity or receiver response.

## Controlled-RF Safety Boundary

GPS-band interference can disrupt safety-critical services and may be illegal
to radiate. Run the live command only after all of these conditions are true:

1. the X300 TX port is connected to an authorized conducted RF path or is
   inside a verified shielded enclosure;
2. the setup has approved attenuation, isolation, a DC block where required,
   and receiver-input power protection;
3. leakage and the emergency-stop procedure have been checked;
4. the selected X300 address, channel, antenna, and gain match the active RF
   power plan; and
5. the timing and RF setup have a current calibration record.

`--confirm-controlled-rf` is an operator acknowledgement, not a safety sensor.
The program cannot detect an antenna, unsafe power, poor shielding, or RF
leakage. Do not use these commands in an open-air setup.

The live example deliberately uses non-numeric placeholders for gain and
timing calibration. It fails before transmission until the operator replaces
them with values from the current laboratory record.

## Command Contract

`--matched-code-target-prns` is the sole transmitted-PRN selector. For example:

```text
--matched-code-target-prns 1,3
```

transmits matched-code components for PRNs 1 and 3 only.

`-S` may create synthetic satellites needed by the internal alignment model.
It does not select transmitted components. A PRN present in `-S` but absent
from `--matched-code-target-prns` is not emitted by the matched-code source.

Do not pass `-P`. It is a clean-constellation selector and is rejected in this
mode. The following controls are also rejected:

- `-d` and `-n`;
- legacy `-A`, `-J`, and `-G` attack controls;
- `--matched-code-js-db`; and
- `--matched-code-onset`, `--matched-code-offset`, and
  `--matched-code-ramp`.

There is no clean signal in the transmitted stream, so digital J/S is not a
meaningful source control. Use `--matched-code-amplitude` in `(0, 1]` instead.
It is a normalized digital amplitude, not X300 output power or receiver-input
jammer power.

For `N` selected PRNs, the conservative bound is:

```text
predicted peak full scale = amplitude * sqrt(N)
```

`x300tx` requires at least 1 dB of predicted digital headroom before arming.
X300 RF gain remains a separate explicit `--gain` setting.

## 1. Build and Run Hardware-Independent Tests

`x300tx` is an optional UHD target and is not part of the default `make all`:

```sh
pkg-config --modversion uhd
make x300tx
make test-x300tx-matched
./x300tx --help
```

`make test-x300tx-matched` runs dry-run and fail-closed CLI tests. It never
performs a live UHD send. The test also regenerates the dry-run bytes with
`matchedgen` from the emitted trajectory and compares a deterministic digest.
That check proves that the adapter rendered jammer-only bytes rather than a
clean-plus-jammer composite.

## 2. Freeze a Small Scenario

Start with a small target list. Keep the RINEX file and scenario values
unchanged between dry run and the corresponding controlled live run.

```sh
RUN_ID="matched-code-jammer-$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$(pwd)/runs/$RUN_ID"
mkdir -p "$OUT_DIR"

RINEX="/absolute/path/to/frozen-navigation.rnx"
REFERENCE_LLH="21.0047844,105.8460541,22"
TARGET_PRNS="1,3"
ALIGNMENT_SATS="1:overhead,3:90/45"

REQUESTED_RATE_HZ="2500000"
MATCHED_CODE_AMPLITUDE="0.5"
PHASE_SEED="42"

DRY_MANIFEST="$OUT_DIR/dry-run.json"
DRY_TRAJECTORY="$OUT_DIR/dry-run-trajectory.csv"
LIVE_MANIFEST="$OUT_DIR/live-run.json"
LIVE_TRAJECTORY="$OUT_DIR/live-run-trajectory.csv"
```

At amplitude `0.5`, two targets have a conservative predicted peak of about
`0.707` full scale and about 3 dB of headroom. Recalculate the bound before
changing the target count or amplitude.

The `-S` values above provide controlled alignment states for PRNs 1 and 3.
For an ephemeris-driven geometry, omit or change `-S` only as part of a newly
declared scenario, then repeat dry run and every preflight check.

## 3. Always Run Dry Run First

Dry run never discovers, opens, or configures UHD. It renders a fixed 100 ms
validation window starting at sample zero; the live mode is still unbounded.

```sh
./x300tx \
  -e "$RINEX" \
  -l "$REFERENCE_LLH" \
  -S "$ALIGNMENT_SATS" \
  --rate "$REQUESTED_RATE_HZ" \
  --matched-code-target-prns "$TARGET_PRNS" \
  --matched-code-amplitude "$MATCHED_CODE_AMPLITUDE" \
  --matched-code-phase-seed "$PHASE_SEED" \
  --manifest "$DRY_MANIFEST" \
  --trajectory "$DRY_TRAJECTORY" \
  --dry-run
```

The process must exit zero and report that UHD was never opened. Check the
manifest with:

```sh
python3 - "$DRY_MANIFEST" "$TARGET_PRNS" <<'PY'
import hashlib
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
expected_prns = [int(value) for value in sys.argv[2].split(",")]
run = json.loads(path.read_text())
samples = run["sample_contract"]
trajectory = pathlib.Path(run["trajectory"]["path"])

assert run["schema"] == "gps-sdr-sim.x300tx-matched-code.v2"
assert run["status"] == "dry_run"
assert run["exit_status"] == 0
assert run["failure_reason"] is None
assert run["rf_output"] == {
    "contains": "matched_code_interference_only",
    "clean_gps_transmitted": False,
}
assert run["scenario"]["selected_target_prns"] == expected_prns
assert run["scenario"]["startup_target_allocation_passed"] is True
assert samples["continuous"] is True
assert samples["planned_samples"] is None
assert samples["dry_run_validation_samples"] > 0
assert samples["internal_alignment_samples"] == samples["dry_run_validation_samples"]
assert samples["rendered_jammer_samples"] == samples["dry_run_validation_samples"]
assert samples["quantized_jammer_samples"] == samples["dry_run_validation_samples"]
assert samples["sent_jammer_samples"] == 0
assert run["activation"] == {
    "start_sample": 0,
    "stop_condition": "SIGINT_or_SIGTERM",
}
assert run["measurements"]["clipped_components"] == 0
assert len(run["waveform"]["jammer_iq_fnv1a64"]) == 16
assert trajectory.is_file()
assert hashlib.sha256(trajectory.read_bytes()).hexdigest() == run["trajectory"]["sha256"]

print("dry-run jammer-only validation passed:", path)
PY
```

The FNV-1a IQ digest is a deterministic regression value, not a cryptographic
provenance hash. The trajectory retains SHA-256 provenance.

Dry run uses the requested rate as its actual rate. A live run repeats all
rate-sensitive checks using the rate returned by UHD.

## 4. Discover and Inspect the Intended X300

Set the address from the current lab inventory and verify that discovery
returns exactly one X300 or X310:

```sh
X300_ADDR="replace-with-current-x300-address"

uhd_find_devices --args "addr=$X300_ADDR"
uhd_usrp_probe --args "addr=$X300_ADDR"
```

Confirm the product and serial against the experiment record. Confirm that the
selected channel exists, the antenna name is valid for it, the requested rate
is supported, and the intended clock/time sources are available. Discovery
does not prove that a sample was transmitted.

## 5. Load Current RF and Timing Values

The established Trimble path uses a calibrated network time tag with internal
X300 clock and time sources. Fill every value from the active setup:

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

Leaving the placeholders unchanged is intentional: parsing or device
validation fails before a timed transmission is armed.

An explicit GPS week/TOW path is supported only when the current calibration
uses an external or GPSDO time source. Load these values from that record:

```sh
GPS_WEEK="replace-with-calibrated-gps-week"
GPS_TOW="replace-with-calibrated-gps-tow"
TX_ADVANCE_NS="replace-with-approved-future-start-lead"
```

For that path, replace the two `internal` source lines and all Trimble options
in the live command with:

```sh
--gps-week "$GPS_WEEK" \
--gps-tow "$GPS_TOW" \
--clock-source external \
--time-source external \
--tx-advance-ns "$TX_ADVANCE_NS"
```

Do not use `-n` for a matched-code live run.

## 6. Start the Continuous Jammer-Only Stream

Reconfirm that the RF path is conducted or shielded before adding
`--confirm-controlled-rf`. Run the same target, alignment, amplitude, and phase
values that passed dry run:

```sh
./x300tx \
  -e "$RINEX" \
  -l "$REFERENCE_LLH" \
  -S "$ALIGNMENT_SATS" \
  --rate "$REQUESTED_RATE_HZ" \
  --matched-code-target-prns "$TARGET_PRNS" \
  --matched-code-amplitude "$MATCHED_CODE_AMPLITUDE" \
  --matched-code-phase-seed "$PHASE_SEED" \
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
  --manifest "$LIVE_MANIFEST" \
  --trajectory "$LIVE_TRAJECTORY" \
  --calibration-id "$CALIBRATION_ID" \
  --confirm-controlled-rf
```

The stream has no scheduled end. Observe the approved experiment interval,
then press `Ctrl-C` once. `x300tx` stops producing frames, sends end-of-burst,
finalizes the trajectory and manifest, and treats the operator stop as success
only when samples were sent and all clipping and UHD fault counters are zero.

Do not kill the process a second time while it is finalizing artifacts. A
nonzero exit, zero sent samples, target loss, clipping, underflow, sequence
error, time error, or artifact-finalization error rejects the run.

## 7. Validate the Final Manifest and Trajectory

Run this check after `x300tx` exits cleanly:

```sh
python3 - "$LIVE_MANIFEST" "$TARGET_PRNS" <<'PY'
import hashlib
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
expected_prns = [int(value) for value in sys.argv[2].split(",")]
run = json.loads(path.read_text())
samples = run["sample_contract"]
measurements = run["measurements"]
hardware = run["hardware"]
trajectory = pathlib.Path(run["trajectory"]["path"])

assert run["schema"] == "gps-sdr-sim.x300tx-matched-code.v2"
assert run["status"] == "stopped"
assert run["exit_status"] == 0
assert run["failure_reason"] is None
assert run["rf_output"] == {
    "contains": "matched_code_interference_only",
    "clean_gps_transmitted": False,
}
assert run["safety"]["controlled_rf_confirmed"] is True
assert run["safety"]["calibration_id"]
assert run["scenario"]["selected_target_prns"] == expected_prns
assert run["scenario"]["startup_target_allocation_passed"] is True
assert run["timing"]["start_margin_met"] is True
assert hardware["device_type"] in {"x300", "x310"}
assert hardware["actual_antenna"] == hardware["requested_antenna"]

assert samples["continuous"] is True
assert samples["planned_samples"] is None
assert samples["sent_jammer_samples"] > 0
assert samples["internal_alignment_samples"] == samples["rendered_jammer_samples"]
assert samples["rendered_jammer_samples"] == samples["quantized_jammer_samples"]
assert samples["sent_jammer_samples"] <= samples["quantized_jammer_samples"]
assert abs(samples["actual_rate_hz"] - samples["requested_rate_hz"]) <= 0.5

assert measurements["operator_stopped"] is True
assert measurements["clipped_components"] == 0
assert measurements["underflows"] == 0
assert measurements["sequence_errors"] == 0
assert measurements["time_errors"] == 0

assert trajectory.is_file()
assert hashlib.sha256(trajectory.read_bytes()).hexdigest() == run["trajectory"]["sha256"]

print("continuous jammer-only transmitter acceptance passed")
print("sent jammer samples:", samples["sent_jammer_samples"])
print("rendered jammer samples:", samples["rendered_jammer_samples"])
print("actual rate (Hz):", samples["actual_rate_hz"])
print("manifest:", path)
print("trajectory:", trajectory)
PY
```

Rendered and quantized counts may exceed sent counts because the live path
prebuffers frames. The manifest's IQ FNV-1a digest covers every rendered
jammer byte, including a possible unsent prebuffer tail; it is not a digest of
an independently captured RF stream.

Keep the manifest, trajectory, exact command, frozen RINEX file, and referenced
calibration record together. The trajectory is provenance; the live renderer
does not read it back while transmitting.

## Troubleshooting

### A requested PRN is unavailable

Every target must be usable in the internal alignment model. Check the frozen
ephemeris, scenario start, reference position, and `-S`. Do not add `-P` or
silently change the target list. Declare a corrected scenario and repeat dry
run. Target loss during streaming ends the run as an error.

### The command fails before opening UHD

This is intended fail-closed behavior. Check for a missing or duplicate target,
invalid amplitude or phase seed, insufficient headroom, an unwritable or
colliding artifact path, a rejected finite/timing/J/S/legacy control, live RTCM
input, or a missing explicit device, RF, timing, or calibration value.

### The requested sample rate is rejected

The current renderer requires an integer number of samples per 100 ms epoch.
Use the established 2.5 Msps profile unless the sample-rate contract has been
extended and tested. A live run is rejected if UHD coerces the requested rate
by more than 0.5 Hz.

### Discovery finds no device, several devices, or another product

Check the host interface, subnet, X300 power, UHD image compatibility, and
explicit address. Re-run `uhd_find_devices` and `uhd_usrp_probe`. The mode does
not select an ambiguous device or a non-X300/X310 product automatically.

### The future start becomes stale

Increase the approved future-start offset or reduce preflight latency, then
repeat the complete run. The tool does not fall back to an immediate,
uncalibrated start.

### The manifest reports clipping or a UHD fault

Treat the run as failed and preserve its artifacts. Check host load, transport,
amplitude/headroom, and the X300 path. Do not edit an unsuccessful manifest to
make it appear accepted.

## Research Interpretation

`status: "stopped"` with the checks above proves that `x300tx` generated the
selected continuous jammer-only digital stream and passed it through the X300
streaming path until an orderly operator stop. It does not prove:

- TX-port or receiver-input power;
- RF code phase, Doppler, spectrum, spurs, or waveform fidelity;
- cable, front-end, propagation, multipath, AGC, or enclosure behavior;
- authentic live-sky alignment;
- receiver acquisition, tracking, `svUsed`, pseudorange residual, or PVT; or
- mitigation performance.

Those claims require calibrated RF capture and/or receiver-side evidence in a
separate experiment. Repeat receiver experiments across explicit phase seeds;
one accepted seed proves execution, not phase-robust behavior.

The previous finite clean-plus-jammer X300 acceptance record is superseded. It
does not accept this continuous jammer-only contract. A new controlled run is
required before changing this guide's status to hardware accepted.
