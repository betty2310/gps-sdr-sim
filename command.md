# Current bladeRF Commands: Revive Spoofing and Jammer-Only Tests

Use these commands only in an authorized conducted RF path or a verified
shielded enclosure. Start with the approved attenuation and lowest calibrated
gain. Re-check the timing and RF calibration after changing cables, clocks,
SDR hardware, or firmware.

## 1. Build

```bash
make bladetx jammertx revive_candidates
```

## 2. Find revive candidates

Trimble port `5005` with mount `NAVIS` provides the live RTCM observations
used to reject PRNs that are currently visible. The RINEX file remains required
because revive mode uses each target PRN's own past ephemeris as its template.

```bash
./revive_candidates \
  -e hour1120.26n \
  -l 21.0047844,105.8460541,22 \
  --rtcm-host 192.168.5.245 \
  --rtcm-port 5005 \
  --rtcm-mount NAVIS \
  --rtcm-user NAVIS:navis123 \
  --rtcm-warmup-sec 6 \
  --rtcm-timeout-ms 3000 \
  --top 12
```

Choose only PRNs reported as suitable by the revive scan and absent from the
live-observed PRN set. Update `-P` and `-S` together in the transmit command.

## 3. Transmit revive spoofing with Trimble timing

This example revives PRNs `22,14,30`. Trimble port `5017` supplies the 1 PPS
UTC time tag used to schedule the bladeRF transmission.

```bash
./bladetx \
  -e hour1120.26n \
  -l 21.0047844,105.8460541,22 \
  -P 22,14,30 \
  -S 22:revive,14:revive,30:revive \
  --device libusb:serial=270c4bef62ded4be1bd9c822a60306a5 \
  --rate 2500000 \
  --txvga1 -35 \
  --txvga2 0 \
  --trimble-time-tag-host 192.168.5.245 \
  --trimble-time-tag-port 5017 \
  --trimble-start-offset-sec 2 \
  --trimble-tag-lead-ms 788 \
  --trimble-tx-cal-ns 580047 \
  --gps-time-ppm 0.002894118
```

Do not add `-n`, `--gps-week`, or `--gps-tow`; those timing modes are mutually
exclusive with Trimble time-tag mode. Press `Ctrl-C` once to stop.

## 4. Jammer-only transmission

`jammertx` supports `cw`, `narrowband`, `wideband`, `chirp`, and `pulsed`.
It intentionally does not use Trimble time, RINEX, location, PRNs, or
authentic-GPS IQ. Do not add Trimble options to these commands.

Create the manifest directory once:

```bash
mkdir -p runs/jammer
```

### CW

Dry run:

```bash
./jammertx \
  --type cw \
  --frequency 500000 \
  --amplitude 0.5 \
  --phase 0 \
  --continuous \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --manifest runs/jammer/cw-dry-run.json \
  --dry-run
```

Live controlled-RF run:

```bash
./jammertx \
  --type cw \
  --frequency 500000 \
  --amplitude 0.5 \
  --phase 0 \
  --continuous \
  --device libusb:serial=270c4bef62ded4be1bd9c822a60306a5 \
  --channel 0 \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --gain -35 \
  --start-delay 0.25 \
  --calibration-id REPLACE_WITH_CURRENT_CALIBRATION_RECORD \
  --manifest runs/jammer/cw-live.json \
  --confirm-controlled-rf
```

### Narrowband noise

Dry run:

```bash
./jammertx \
  --type narrowband \
  --frequency 500000 \
  --bandwidth 100000 \
  --amplitude 0.15 \
  --seed 20260712 \
  --continuous \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --manifest runs/jammer/narrowband-dry-run.json \
  --dry-run
```

Live controlled-RF run:

```bash
./jammertx \
  --type narrowband \
  --frequency 500000 \
  --bandwidth 100000 \
  --amplitude 0.15 \
  --seed 20260712 \
  --continuous \
  --device libusb:serial=270c4bef62ded4be1bd9c822a60306a5 \
  --channel 0 \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --gain -35 \
  --start-delay 0.25 \
  --calibration-id REPLACE_WITH_CURRENT_CALIBRATION_RECORD \
  --manifest runs/jammer/narrowband-live.json \
  --confirm-controlled-rf
```

### Wideband noise

Dry run:

```bash
./jammertx \
  --type wideband \
  --amplitude 0.15 \
  --seed 20260712 \
  --continuous \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --manifest runs/jammer/wideband-dry-run.json \
  --dry-run
```

Live controlled-RF run:

```bash
./jammertx \
  --type wideband \
  --amplitude 0.15 \
  --seed 20260712 \
  --continuous \
  --device libusb:serial=270c4bef62ded4be1bd9c822a60306a5 \
  --channel 0 \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --gain -35 \
  --start-delay 0.25 \
  --calibration-id REPLACE_WITH_CURRENT_CALIBRATION_RECORD \
  --manifest runs/jammer/wideband-live.json \
  --confirm-controlled-rf
```

### Chirp

Dry run:

```bash
./jammertx \
  --type chirp \
  --frequency -500000 \
  --end-frequency 500000 \
  --chirp-period 0.1 \
  --amplitude 0.5 \
  --phase 0 \
  --continuous \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --manifest runs/jammer/chirp-dry-run.json \
  --dry-run
```

Live controlled-RF run:

```bash
./jammertx \
  --type chirp \
  --frequency -500000 \
  --end-frequency 500000 \
  --chirp-period 0.1 \
  --amplitude 0.5 \
  --phase 0 \
  --continuous \
  --device libusb:serial=270c4bef62ded4be1bd9c822a60306a5 \
  --channel 0 \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --gain -35 \
  --start-delay 0.25 \
  --calibration-id REPLACE_WITH_CURRENT_CALIBRATION_RECORD \
  --manifest runs/jammer/chirp-live.json \
  --confirm-controlled-rf
```

### Pulsed CW

Dry run:

```bash
./jammertx \
  --type pulsed \
  --frequency 500000 \
  --pulse-period 0.01 \
  --pulse-duty 0.1 \
  --pulse-ramp 0.00005 \
  --amplitude 0.5 \
  --phase 0 \
  --continuous \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --manifest runs/jammer/pulsed-dry-run.json \
  --dry-run
```

Live controlled-RF run:

```bash
./jammertx \
  --type pulsed \
  --frequency 500000 \
  --pulse-period 0.01 \
  --pulse-duty 0.1 \
  --pulse-ramp 0.00005 \
  --amplitude 0.5 \
  --phase 0 \
  --continuous \
  --device libusb:serial=270c4bef62ded4be1bd9c822a60306a5 \
  --channel 0 \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --ramp 0.01 \
  --gain -35 \
  --start-delay 0.25 \
  --calibration-id REPLACE_WITH_CURRENT_CALIBRATION_RECORD \
  --manifest runs/jammer/pulsed-live.json \
  --confirm-controlled-rf
```

Always run the matching dry run before live transmission. Press `Ctrl-C` once
to stop a continuous live run and allow its manifest to finalize. Use one
jammer type and one unique manifest per receiver capture. Digital amplitude is
not RF power or receiver-input J/S; measure J/S at the DUT reference plane.

## 5. Matched-code jammer-only transmission

Matched-code mode uses `bladetx`, not `jammertx`. Only the PRNs named by
`--matched-code-target-prns` are transmitted. Clean simulator IQ is generated
internally to maintain code/carrier alignment and is discarded before the
bladeRF adapter.

The mode starts with jammer sample zero and runs continuously until `Ctrl-C`,
`SIGINT`, or `SIGTERM`. It has no transmitter-generated clean prefix or recovery
interval. Start the receiver recording before `bladetx`, stop `bladetx` with one
`Ctrl-C`, and keep recording afterward to capture recovery.

Use only an authorized conducted path or verified shielded enclosure. Digital
amplitude and bladeRF gain are not receiver-input power or J/S. Begin with the
approved external attenuation and the lowest calibrated gain.

### Step 1: Build and test

Build `bladetx`, `matchedgen`, and run the hardware-independent matched-code
checks:

```bash
make test-bladetx-matched
```

### Step 2: Set the run parameters

These values match the current bladeRF, Trimble timing path, location, and
timing calibration used above. `PRN 1,3` is a validated command example, not a
claim that those PRNs are absent from the current sky. Before a live run, use
the receiver's current satellite view and select target PRNs that satisfy the
experiment. Update `TARGET_PRNS` and `SYNTHETIC_TARGETS` together.

```bash
NAV=hour2320.26n
LOCATION=21.0047844,105.8460541,22
TARGET_PRNS=1,3
SYNTHETIC_TARGETS=1:overhead,3:90/45
RATE=2600000
MATCHED_AMPLITUDE=0.10
PHASE_SEED=42

DEVICE=libusb:serial=270c4bef62ded4be1bd9c822a60306a5
TXVGA1=-35
TXVGA2=0

TRIMBLE_HOST=192.168.5.245
TRIMBLE_PORT=5017
TRIMBLE_START_OFFSET_SEC=2
TRIMBLE_TAG_LEAD_MS=788
TRIMBLE_TX_CAL_NS=580047
GPS_TIME_PPM=0.002894118
PREBUFFER_EPOCHS=5

RUN="runs/matched-code/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN"
```

`--txvga1 -35` and `--txvga2 0` are the minimum bladeRF1 stage gains. They are
different from the aggregate `jammertx --gain` option. Change them only through
the controlled-RF power plan.

Confirm that the inputs are available:

```bash
test -r "$NAV"
bladeRF-cli -p
nc -vz "$TRIMBLE_HOST" "$TRIMBLE_PORT"
```

The bladeRF probe must report serial
`270c4bef62ded4be1bd9c822a60306a5`. The RINEX file is frozen scenario input;
continuous matched-code mode rejects live RTCM ephemeris.

### Step 3: Run the hardware-independent preflight

Do not add the Trimble time-tag options to this dry run. It renders 100 ms,
writes the manifest and trajectory, and never discovers or opens the bladeRF:

```bash
./bladetx \
  -e "$NAV" \
  -l "$LOCATION" \
  -S "$SYNTHETIC_TARGETS" \
  --rate "$RATE" \
  --gps-time-ppm "$GPS_TIME_PPM" \
  --matched-code-target-prns "$TARGET_PRNS" \
  --matched-code-amplitude "$MATCHED_AMPLITUDE" \
  --matched-code-phase-seed "$PHASE_SEED" \
  --manifest "$RUN/dry-run.json" \
  --trajectory "$RUN/dry-run.trajectory.csv" \
  --device "$DEVICE" \
  --txvga1 "$TXVGA1" \
  --txvga2 "$TXVGA2" \
  --prebuffer "$PREBUFFER_EPOCHS" \
  --dry-run
```

With the values above, the expected preflight reports targets `1,3`, 260,000
validation samples, approximately 16.99 dB predicted headroom, and no clipping.

### Step 4: Check the dry-run artifacts

Run this acceptance check before connecting or enabling the live transmitter:

```bash
python3 - "$RUN/dry-run.json" "$TARGET_PRNS" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text())
expected_targets = [int(prn) for prn in sys.argv[2].split(",")]
assert manifest["status"] == "dry_run"
assert manifest["exit_status"] == 0
assert manifest["rf_output"]["contains"] == "matched_code_interference_only"
assert manifest["rf_output"]["clean_gps_transmitted"] is False
assert manifest["scenario"]["selected_target_prns"] == expected_targets
assert manifest["scenario"]["startup_target_allocation_passed"] is True
assert manifest["waveform"]["predicted_headroom_db"] >= 1.0
assert manifest["measurements"]["clipped_components"] == 0
assert manifest["sample_contract"]["sent_jammer_samples"] == 0
assert Path(manifest["trajectory"]["path"]).is_file()
assert manifest["trajectory"]["sha256"]
print("matched-code dry run accepted")
PY
```

Accept the preflight only when:

- `status` is `dry_run`;
- `rf_output.contains` is `matched_code_interference_only`;
- `rf_output.clean_gps_transmitted` is `false`;
- `scenario.startup_target_allocation_passed` is `true`;
- `waveform.predicted_headroom_db` is at least 1 dB;
- `measurements.clipped_components` is `0`; and
- `sample_contract.sent_jammer_samples` is `0`.

### Step 5: Prepare receiver logging and the controlled RF path

1. Verify the selected target PRNs against the live-sky satellite list.
2. Connect the approved attenuated conducted path or close and verify the
   shielded enclosure.
3. Start raw receiver logging before starting `bladetx`. Preserve at least
   `UBX-NAV-SAT`, `UBX-NAV-PVT`, `UBX-MON-RF`, and `UBX-MON-SPAN` when the
   receiver supports them.
4. Exit any interactive `bladeRF-cli` session so `bladetx` can open the device.
5. Set the real calibration record identifier; do not use a placeholder:

```bash
export CALIBRATION_ID='your-current-controlled-rf-calibration-record'
: "${CALIBRATION_ID:?CALIBRATION_ID must be set before live TX}"
```

### Step 6: Start continuous matched-code transmission

This command opens the bladeRF, obtains a Trimble time tag, schedules the first
sample, and transmits only the selected matched-code components until operator
stop:

```bash
./bladetx \
  -e "$NAV" \
  -l "$LOCATION" \
  -S "$SYNTHETIC_TARGETS" \
  --rate "$RATE" \
  --gps-time-ppm "$GPS_TIME_PPM" \
  --matched-code-target-prns "$TARGET_PRNS" \
  --matched-code-amplitude "$MATCHED_AMPLITUDE" \
  --matched-code-phase-seed "$PHASE_SEED" \
  --manifest "$RUN/live.json" \
  --trajectory "$RUN/live.trajectory.csv" \
  --device "$DEVICE" \
  --txvga1 "$TXVGA1" \
  --txvga2 "$TXVGA2" \
  --prebuffer "$PREBUFFER_EPOCHS" \
  --trimble-time-tag-host "$TRIMBLE_HOST" \
  --trimble-time-tag-port "$TRIMBLE_PORT" \
  --trimble-start-offset-sec "$TRIMBLE_START_OFFSET_SEC" \
  --trimble-tag-lead-ms "$TRIMBLE_TAG_LEAD_MS" \
  --trimble-tx-cal-ns "$TRIMBLE_TX_CAL_NS" \
  --calibration-id "$CALIBRATION_ID" \
  --confirm-controlled-rf
```

Do not add `-d`, `-n`, `-P`, `-A`, `-J`, `-G`, `--gps-week`, `--gps-tow`, or
live RTCM options. The matched-code target list is the sole transmitted-PRN
selector, and the Trimble time-tag path supplies the live GPS epoch.

### Step 7: Stop and finalize the run

Press `Ctrl-C` once. Wait for the final `[TX] Matched-code status=...` line and
for `bladetx` to return before stopping receiver logging. A clean operator stop
has status `stopped`; `interrupted` indicates that final acceptance failed.

### Step 8: Check the live manifest

```bash
python3 - "$RUN/live.json" "$TARGET_PRNS" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text())
expected_targets = [int(prn) for prn in sys.argv[2].split(",")]
assert manifest["status"] == "stopped"
assert manifest["failure_reason"] is None
assert manifest["exit_status"] == 0
assert manifest["rf_output"]["contains"] == "matched_code_interference_only"
assert manifest["rf_output"]["clean_gps_transmitted"] is False
assert manifest["scenario"]["selected_target_prns"] == expected_targets
assert manifest["scenario"]["startup_target_allocation_passed"] is True
assert manifest["timing"]["start_mode"] == "trimble_time_tag"
assert manifest["timing"]["start_margin_met"] is True
assert manifest["sample_contract"]["sent_jammer_samples"] > 0
assert manifest["measurements"]["operator_stopped"] is True
assert manifest["measurements"]["clipped_components"] == 0
assert manifest["measurements"]["underflows"] == 0
assert manifest["measurements"]["sequence_errors"] == 0
assert manifest["measurements"]["time_errors"] == 0
assert Path(manifest["trajectory"]["path"]).is_file()
assert manifest["trajectory"]["sha256"]
print("matched-code live run accepted")
PY
```

Keep `live.json`, `live.trajectory.csv`, the exact RINEX file, receiver logs,
RF calibration record, and any monitoring IQ together. The transmitted signal
is matched-code interference, not a navigation-message-bearing spoofed GPS
constellation.

Use the full [bladeRF matched-code guide](docs/realtime-code-aligned-matched-code-bladerf.md)
for the live fail-closed template, SC16/Q15-to-Q11 boundary, artifact contract,
and troubleshooting limits.
