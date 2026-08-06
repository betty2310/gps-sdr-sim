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
