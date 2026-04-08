# UBX bladeTX Calibration Tool

## Purpose

`tools/ubx_bladetx_cal.py` estimates timing mismatch between injected GPS PRNs
and real-sky PRNs from a u-blox `.ubx` log, then recommends a better
`--trimble-tx-cal-ns` value for `player/bladetx.cpp`.

It is designed to speed up bladeTX alignment loops.

## Input

- one or more UBX logs
- injected PRN list from your TX command (`-P`)
- optional current bladeTX timing knobs (echoed in output)

Required UBX content for high-confidence calibration:

- `UBX-RXM-RAWX`
- `UBX-NAV-SAT`
- `UBX-NAV-STATUS`
- `UBX-NAV-PVT`

If `NAV-SAT` is missing, or injected PRNs are never `svUsed`, the tool reports
tracking diagnostics but cannot produce a strong calibration.

## Command

```bash
python3 tools/ubx_bladetx_cal.py <log.ubx> \
  --inject 3,4,7,8 \
  --current-trimble-tx-cal-ns 0 \
  --trimble-tag-lead-ms 790 \
  --trimble-start-offset-sec 2
```

Example with your `ver2` folder:

```bash
python3 tools/ubx_bladetx_cal.py \
  07042026/ver2/COM3___9600_260407_105448.ubx \
  07042026/ver2/COM4___9600_260407_105428.ubx \
  --inject 3,4,7,8 \
  --trimble-tag-lead-ms 790 \
  --trimble-start-offset-sec 2
```

## Output Meaning

Per capture, the tool shows:

- injected PRN tracking status from RAWX
- whether injected PRNs are actually used (`svUsed`) in NAV-SAT
- real anchor PRNs used in navigation
- common residual estimate (real minus injected)
- suggested update for `--trimble-tx-cal-ns`

Overall, it prints:

- `best delta --trimble-tx-cal-ns`
- `best next --trimble-tx-cal-ns`

## Sign Convention

The tool follows the same sign convention already documented in this repo:

- positive `--trimble-tx-cal-ns` advances the injected epoch
- recommendation uses:
  `delta_cal_ns = -(real_minus_injected_m / c) * 1e9`

## Practical Notes

- `--trimble-tx-cal-ns` is the preferred knob for constant residual correction.
- `--trimble-tag-lead-ms` should normally stay fixed during fine calibration.
- If output says injected PRNs are tracked but not `svUsed`, alignment is still
  not good enough for those PRNs, or other quality constraints are blocking
  their use.
