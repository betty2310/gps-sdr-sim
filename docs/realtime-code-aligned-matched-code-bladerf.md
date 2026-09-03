# Matched-Code Jammer-Only Transmission with `bladetx`

`bladetx` implements the same continuous matched-code waveform contract as
`x300tx`, using libbladeRF for the hardware boundary. The transmitted stream
contains only the PRNs selected by `--matched-code-target-prns`. Clean GPS IQ
is rendered internally to advance alignment state and is discarded.

GPS-band interference must only be used in an authorized conducted path or a
verified shielded enclosure. `--confirm-controlled-rf` is an operator
acknowledgement, not a leakage or connection check.

## Build and hardware-independent verification

```sh
make test-bladetx-matched
```

The test exercises dry-run rendering, deterministic regeneration through
`matchedgen`, artifact generation, and fail-closed CLI contracts. Dry-run does
not discover, open, configure, enable, or transmit through a bladeRF.

Example dry-run:

```sh
./bladetx \
  -e tests/fixtures/brdc0030.25n \
  -l 21.0047844,105.8460541,22 \
  -S 1:overhead,3:90/45 \
  --rate 2600000 \
  --matched-code-target-prns 1,3 \
  --matched-code-amplitude 0.5 \
  --matched-code-phase-seed 42 \
  --manifest runs/blade-dry.json \
  --dry-run
```

## bladeRF 1.0 sample contract

The shared source produces canonical interleaved SC16/Q15 IQ. The bladeRF 1.0
synchronous metadata API consumes SC16 Q11, so the live adapter divides each
canonical component by 16 with signed Q11 saturation immediately before
`bladerf_sync_tx`. The manifest keeps both facts explicit:

- `sample_contract.format`: `sc16_le` for the canonical reproducible bytes;
- `sample_contract.device_format`: `SC16_Q11_META` for the hardware buffer;
- `waveform.jammer_iq_fnv1a64`: digest of the canonical SC16/Q15 stream.

At the same sample rate, scenario, target list, amplitude, and phase seed, the
canonical trajectory and IQ digest must match `x300tx`.

## Live fail-closed contract

A live matched-code run additionally requires all of the following:

- an explicit `--device` selector;
- explicit `--txvga1` and `--txvga2` values from the current power plan;
- a non-empty `--calibration-id`;
- `--confirm-controlled-rf`; and
- the Trimble time-tag host and port.

The bladeRF 1.0 device timestamp is a free-running sample counter, not a GPS
time source. Therefore live matched-code mode does not accept an explicit GPS
week/TOW as a substitute for the Trimble path. The adapter schedules the first
buffer with `BLADERF_META_FLAG_TX_BURST_START`, then streams continuously until
SIGINT or SIGTERM.

Template only; replace every placeholder from the controlled-lab record:

```sh
./bladetx \
  -e /absolute/path/to/frozen-navigation.rnx \
  -l LAT,LON,ALT \
  -S TARGET_PRN:overhead \
  --rate 2600000 \
  --matched-code-target-prns TARGET_PRN \
  --matched-code-amplitude CALIBRATED_DIGITAL_AMPLITUDE \
  --matched-code-phase-seed PHASE_SEED \
  --manifest /absolute/path/to/live-run.json \
  --trajectory /absolute/path/to/live-run.trajectory.csv \
  --device '*:serial=DEVICE_SERIAL' \
  --txvga1 CALIBRATED_TXVGA1_DB \
  --txvga2 CALIBRATED_TXVGA2_DB \
  --trimble-time-tag-host TRIMBLE_HOST \
  --trimble-time-tag-port TRIMBLE_PORT \
  --trimble-start-offset-sec START_OFFSET \
  --trimble-tx-cal-ns CALIBRATION_NS \
  --calibration-id CALIBRATION_RECORD_ID \
  --confirm-controlled-rf
```

Do not add `-d`, `-n`, `-P`, `-A`, `-J`, or `-G`. The matched-code target list
is the sole transmitted-PRN selector and the mode is intentionally continuous.
