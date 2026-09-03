# Real-Time Multi-Waveform Jammer Transmission with `jammertx`

> **Status:** Implemented for finite-duration and operator-stopped continuous
> CW, narrowband, wideband, chirp, and pulsed transmission through UHD
>
> **Date:** 2026-07-12
>
> **Scope:** Jammer-only GPS L1-band receiver research in an authorized
> conducted path or verified shielded enclosure
>
> **Related:** [Offline Multi-Waveform Jamming Dataset](cw-jamming-dataset.md) and
> [Jamming Mitigation Architecture](jamming-mitigation-architecture.md)

## Purpose

`jammertx` generates an independent synthetic jammer in memory and streams it
directly to a UHD-supported transmitter such as the USRP X300. It supports
finite runs and `--continuous` runs that stop on `SIGINT`/`SIGTERM`. It
transmits only the jammer waveform.

It does **not** need or accept:

- authentic-GPS IQ;
- a RINEX navigation file;
- broadcast ephemeris;
- GPS week or time of week;
- receiver location;
- PRN selection;
- C/A-code phase, satellite Doppler, or navigation data.

Those inputs are necessary when generating a GNSS signal, including revive or
clone scenarios. They are not necessary for any implemented jammer type. When
the receiver is also receiving authentic GNSS, the authentic signal and
transmitted jammer combine physically before the receiver front end.

No implemented source requires authentic IQ. Authentic or controlled GNSS is
required only when the experiment intends to measure mitigation against a real
desired signal. A captured real interferer, propagation channel, antenna
coupling, and out-of-band front-end overload likewise require measurement or IQ
replay; these synthetic models cannot make those effects authentic.

## Safety Boundary

GNSS-band interference can disrupt safety-critical services and may be illegal
to radiate. Use `jammertx` only with all of the following controls in place:

1. an authorized conducted RF path or a verified shielded enclosure;
2. suitable attenuation, isolation, and receiver-input power protection;
3. a DC block where required by the connected hardware;
4. a measured leakage baseline and tested stop procedure;
5. compliance with the rules governing the laboratory and jurisdiction.

`jammertx` requires `--confirm-controlled-rf` before it opens the UHD device for
transmission. That flag is an acknowledgement, not proof that the physical
setup is safe. The software cannot detect an attached antenna, inadequate
shielding, excessive receiver-input power, or RF leakage.

Do not use the examples in an open-air setup.

## Implemented Signal Models

The live and offline tools share `tools/jammer_source.c`, so type names,
defaults, seeds, envelopes, and SC16 quantization are identical at the same
sample rate:

| `--type` | Model | Main controls |
| --- | --- | --- |
| `cw` | Proper complex fixed-frequency tone | `--frequency`, `--phase` |
| `narrowband` | Seeded proper-complex Gaussian noise with one-pole low-pass shaping and frequency translation | `--frequency`, `--bandwidth`, `--seed` |
| `wideband` | Seeded unfiltered proper-complex Gaussian noise across complex Nyquist | `--seed` |
| `chirp` | Repeating phase-continuous linear sweep | `--frequency`, `--end-frequency`, `--chirp-period` |
| `pulsed` | Phase-continuous CW under a periodic raised-cosine gate | `--frequency`, `--pulse-period`, `--pulse-duty`, `--pulse-ramp` |

CW is:

```text
j[n] = A envelope[n] exp(j(phi + 2 pi f_offset (n - onset) / Fs))
```

where:

- `A` is active complex RMS as a fraction of signed-16-bit full scale;
- `Fs` is the actual UHD transmit sample rate;
- `f_offset` is a signed complex-baseband frequency;
- `phi` is the oscillator phase at the onset sample;
- `envelope[n]` is zero outside the active interval and uses half-cosine
  rise/fall ramps.

The global oscillator advances only while the jammer interval is active. A
clean prefix does not change phase at onset. Pulsed mode continues carrier
phase through its off portions; it does not restart the carrier at every pulse.

For narrowband noise, `--bandwidth` is the two-sided equivalent-noise bandwidth
(ENBW), not a brick-wall occupied width. With actual UHD rate `Fs` and ENBW
`B`, the low-pass pole is `(Fs-B)/(Fs+B)`. Wideband noise has digital support
over `[-Fs/2, Fs/2)`. The default stochastic amplitude is `0.15` complex RMS to
retain SC16 peak margin; CW/chirp default to `0.5`, and pulsed `0.5` is the
on-pulse amplitude rather than interval-average RMS.

Chirp instantaneous frequency moves linearly from start to end over
`--chirp-period`, then restarts at the start frequency without resetting phase.
Pulsed mode starts its pulse schedule at global onset and reports sample-rounded
period, on-time, and pulse ramps in the manifest.

For an ideal transmitter using RF center `f_center`, the complex tone maps to:

```text
f_RF = f_center + f_offset
```

For the default values:

```text
f_center = 1575.420 MHz
f_offset = +0.500 MHz
f_RF     = 1575.920 MHz
```

Real hardware can invert the displayed sign because of an IQ convention in the
TX or monitoring receiver. Confirm the side of the carrier with a controlled
capture instead of assuming it.

## Live Authentic-Plus-Jammer Architecture

The live experiment replaces the offline digital mixer with physical RF
combination:

```text
authorized authentic GNSS feed -----------\
                                            +--> DUT receiver
X300 running jammertx --> calibrated path -/
```

The receiver input is therefore:

```text
r_DUT(t) = h_s(t) s_authentic(t) + h_j(t) j_X300(t) + receiver_noise(t)
```

Possible controlled arrangements include:

- a conducted authentic-GNSS source and X300 path joined by a suitable RF
  combiner;
- an authorized GNSS repeater/feed and X300 inside a verified shielded
  enclosure;
- a jammer-only conducted characterization path before authentic-GNSS
  combination.

`jammertx` never reads the authentic path. It is the RF setup and receiver front
end that perform the addition.

## Relationship to the Offline Dataset and Revive Mode

| Property | Offline jamming dataset | `jammertx` | Revive/clone transmitter |
| --- | --- | --- | --- |
| Desired signal | `clean-source.bin` | Authentic or controlled external feed | Authentic external feed plus generated GNSS |
| Interferer | `<type>-jammer.bin` | Selected type generated in memory | Structured GPS L1 C/A signal |
| Addition | `iqmix` | Physical RF path | Physical RF path |
| Ephemeris | Required for generated clean GPS | Not used | Required |
| GPS time alignment | Dataset scenario time | Not required | Required |
| PRN/code/Doppler | Clean GNSS only | Not used | Required |
| Power ratio | Measured digitally by `iqmix` | Must be calibrated at the DUT input | Must be calibrated at the DUT input |

The offline `jammergen` and live `jammertx` use the same source implementation in
`tools/jammer_source.c`. At the same sample rate and with the same parameters,
their SC16 samples are identical regardless of render-buffer boundaries.

Do not implement these sources as another `gpssim.c` attack method. The legacy attack
branches operate inside simulated satellite channels, while a real jammer is an
independent emitter that must also exist when the desired signal comes from the
real sky.

## Build

UHD development headers and libraries are required for `jammertx`:

```sh
pkg-config --modversion uhd
make jammertx
./jammertx --help
```

The offline tools remain available separately:

```sh
make jammergen iqmix
```

`jammertx` is not part of the default `make all` target because UHD is an
optional hardware dependency.

## Always Start with a Dry Run

Dry-run mode validates all waveform timing and Nyquist constraints, prints the
sample plan, and does not open UHD:

```sh
./jammertx \
  --type cw \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --tone-offset 500000 \
  --amplitude 0.5 \
  --phase 0 \
  --duration 90 \
  --onset 30 \
  --offset 60 \
  --ramp 0.01 \
  --manifest run-cw-dry-run.json \
  --dry-run
```

Expected plan for this profile:

```text
sample rate       2,500,000 complex samples/s
total samples     225,000,000
onset sample       75,000,000
offset sample     150,000,000
ramp samples           25,000
ideal RF tone     1,575,920,000 Hz
```

The dry-run manifest reports requested values. It is not evidence of the
actual device rate, tune result, transmitted power, or RF waveform.

Exercise every source family in dry-run before any RF work. Replace the CW
type/frequency arguments above with one of these parameter sets:

```text
--type narrowband --frequency 500000 --bandwidth 100000 --seed 20260712
--type wideband --seed 20260712
--type chirp --frequency -500000 --end-frequency 500000 --chirp-period 0.1
--type pulsed --frequency 500000 --pulse-period 0.01 --pulse-duty 0.1 \
  --pulse-ramp 0.00005
```

## Controlled Live Runs for an Authentic-Plus-Jammer Dataset

These examples keep one jammer type active from the first transmitted sample
until the operator presses `Ctrl-C`. They use `--continuous`, so do not add
`--duration`, `--onset`, or `--offset`.

Only use these commands after the RF path has passed the laboratory safety and
power checks. Supply the gain and calibration identity from the active
conducted or shielded setup rather than copying values from another experiment.
Prepare a separate directory for the authentic-only baseline and every jammer
type:

```sh
DATASET_ROOT="live-dataset/$(date -u +%Y%m%dT%H%M%SZ)"
X300_ADDR="192.168.10.2"
TX_CHANNEL="0"
TX_ANTENNA="TX/RX"
CALIBRATED_GAIN_DB="replace-with-approved-value"
CALIBRATION_ID="replace-with-calibration-record"

for TYPE in baseline cw narrowband wideband chirp pulsed; do
  mkdir -p "$DATASET_ROOT/$TYPE"
done
```

The digital amplitudes below match the source defaults selected to retain SC16
headroom: `0.5` for deterministic carrier modes and `0.15` complex RMS for
noise modes. They are not RF power or J/S settings. Use the calibrated RF path,
normally including external attenuation, to set and measure jammer power at the
DUT input.

### Continuous CW at +500 kHz baseband offset

```sh
./jammertx \
  --type cw \
  --continuous \
  --addr "$X300_ADDR" \
  --channel "$TX_CHANNEL" \
  --antenna "$TX_ANTENNA" \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --frequency 500000 \
  --amplitude 0.5 \
  --phase 0 \
  --ramp 0.01 \
  --gain "$CALIBRATED_GAIN_DB" \
  --clock-source internal \
  --time-source internal \
  --start-delay 0.25 \
  --calibration-id "$CALIBRATION_ID" \
  --manifest "$DATASET_ROOT/cw/jammertx.json" \
  --confirm-controlled-rf
```

This ideal component is at `1575.420 MHz + 0.500 MHz = 1575.920 MHz`. Verify the
measured spectral sign because an IQ convention can invert the offset.

### Continuous 100 kHz-ENBW narrowband noise

```sh
./jammertx \
  --type narrowband \
  --continuous \
  --addr "$X300_ADDR" \
  --channel "$TX_CHANNEL" \
  --antenna "$TX_ANTENNA" \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --frequency 500000 \
  --bandwidth 100000 \
  --amplitude 0.15 \
  --seed 20260712 \
  --ramp 0.01 \
  --gain "$CALIBRATED_GAIN_DB" \
  --clock-source internal \
  --time-source internal \
  --start-delay 0.25 \
  --calibration-id "$CALIBRATION_ID" \
  --manifest "$DATASET_ROOT/narrowband/jammertx.json" \
  --confirm-controlled-rf
```

The source is centered at a `+500 kHz` baseband offset. `100000` is the
two-sided ENBW of the one-pole shaped-noise model, not a brick-wall occupied
bandwidth.

### Continuous wideband noise

```sh
./jammertx \
  --type wideband \
  --continuous \
  --addr "$X300_ADDR" \
  --channel "$TX_CHANNEL" \
  --antenna "$TX_ANTENNA" \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --amplitude 0.15 \
  --seed 20260712 \
  --ramp 0.01 \
  --gain "$CALIBRATED_GAIN_DB" \
  --clock-source internal \
  --time-source internal \
  --start-delay 0.25 \
  --calibration-id "$CALIBRATION_ID" \
  --manifest "$DATASET_ROOT/wideband/jammertx.json" \
  --confirm-controlled-rf
```

At an actual rate of `2.5 Msps`, the digital noise support is the full complex
Nyquist interval `[-1.25 MHz, +1.25 MHz)`. The analog X300 and DUT filters set
the measured RF shape.

### Continuous phase-continuous linear chirp

```sh
./jammertx \
  --type chirp \
  --continuous \
  --addr "$X300_ADDR" \
  --channel "$TX_CHANNEL" \
  --antenna "$TX_ANTENNA" \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --frequency -500000 \
  --end-frequency 500000 \
  --chirp-period 0.1 \
  --amplitude 0.5 \
  --phase 0 \
  --ramp 0.01 \
  --gain "$CALIBRATED_GAIN_DB" \
  --clock-source internal \
  --time-source internal \
  --start-delay 0.25 \
  --calibration-id "$CALIBRATION_ID" \
  --manifest "$DATASET_ROOT/chirp/jammertx.json" \
  --confirm-controlled-rf
```

This repeats a linear sweep from `1574.920 MHz` to `1575.920 MHz` every
`100 ms`. Phase remains continuous when the frequency law returns to the start
of the next sweep.

### Continuous pulsed CW

```sh
./jammertx \
  --type pulsed \
  --continuous \
  --addr "$X300_ADDR" \
  --channel "$TX_CHANNEL" \
  --antenna "$TX_ANTENNA" \
  --rate 2500000 \
  --center-frequency 1575420000 \
  --frequency 500000 \
  --pulse-period 0.01 \
  --pulse-duty 0.1 \
  --pulse-ramp 0.00005 \
  --amplitude 0.5 \
  --phase 0 \
  --ramp 0.01 \
  --gain "$CALIBRATED_GAIN_DB" \
  --clock-source internal \
  --time-source internal \
  --start-delay 0.25 \
  --calibration-id "$CALIBRATION_ID" \
  --manifest "$DATASET_ROOT/pulsed/jammertx.json" \
  --confirm-controlled-rf
```

This uses a `10 ms` pulse period, `10%` on-time, and `50 us` rise/fall on each
pulse. The carrier phase continues through off-time.

### Capture order for every waveform

`jammertx` does not record receiver IQ and does not ingest the authentic path.
Use the receiver or monitoring SDR's native raw-IQ recorder for the dataset.
For each of the five commands above:

1. Record an authentic-only baseline with the complete combined RF path
   connected and the X300 not streaming.
2. Start a new raw-IQ receiver capture and receiver telemetry log for the
   selected type. Record the capture start time, rate, center frequency, format,
   RF gain/AGC state, and channel/calibration identity.
3. Start the matching `jammertx --continuous` command in the foreground. The
   jammer begins at the first transmitted sample, reaches full global amplitude
   after the `10 ms` startup ramp, and remains active while the process runs.
4. Press `Ctrl-C` once in the `jammertx` terminal when the desired jammer
   interval is complete. Keep the receiver capture running long enough to
   measure reacquisition and PVT recovery, then stop the recorder separately.
5. Accept the run only if `jammertx.json` has `status: "interrupted"`,
   `sample_contract.continuous: true`, and zero underflow, sequence-error,
   time-error, and clipping counts. Preserve the raw receiver IQ, receiver
   telemetry, monitoring capture, jammer manifest, calibration record, and
   operator timestamps together.

The resulting IQ is a capture of real authentic GNSS plus a physically combined
synthetic jammer. Label it that way; it is not an offline sample-exact mix and
does not have an automatically known J/S. Measure J/S at the DUT reference
plane and store that measurement in the dataset metadata.

Run one jammer type per capture. A single `jammertx` process does not switch
types, and reusing one manifest path across types would overwrite evidence.

The placeholders deliberately have no default laboratory meaning. Replace them
only with values from the active setup. `--confirm-controlled-rf` and explicit
`--gain` remain mandatory for live transmission. `--manifest` is optional but
recommended for dataset runs.

In continuous mode, `jammertx` streams until `SIGINT`/`SIGTERM`, sends UHD
end-of-burst metadata, and, when requested, writes the final manifest. The
global `--ramp` is the startup ramp; an operator stop has no scheduled
sample-level fall ramp. Finite mode remains available when exact onset, offset,
and recovery sample boundaries must be generated by the transmitter itself.

## Command-Line Reference

### Waveform

| Option | Default | Meaning |
| --- | ---: | --- |
| `--type TYPE` | `cw` | `cw`, `narrowband`, `wideband`, `chirp`, or `pulsed` |
| `--rate HZ` | `2500000` | Requested complex TX sample rate |
| `--center-frequency HZ` | `1575420000` | UHD RF center |
| `--tone-offset HZ`, `--frequency HZ` | type-specific | Fixed/center/chirp-start offset; the two names are aliases |
| `--end-frequency HZ` | `+500000` for chirp | Chirp end offset |
| `--bandwidth HZ` | `100000` | Narrowband two-sided ENBW |
| `--amplitude VALUE` | `0.15` noise, otherwise `0.5` | Complex RMS or pulsed on-amplitude, in `(0, 1]` |
| `--seed N` | `20260712` | Narrowband/wideband deterministic RNG seed |
| `--chirp-period SEC` | `0.1` | Repeating linear-sweep period |
| `--pulse-period SEC` | `0.01` | Pulse repetition period |
| `--pulse-duty VALUE` | `0.1` | On-time fraction in `(0, 1]` |
| `--pulse-ramp SEC` | `0.00005` | Per-pulse half-cosine rise/fall |
| `--phase RAD` | `0` | Phase at the first active sample |
| `--duration SEC` | required unless continuous | Finite duration from first transmitted sample |
| `--continuous` | absent | Transmit from sample zero until `SIGINT`/`SIGTERM`; exclusive with finite timing |
| `--onset SEC` | `0` | Start of the jammer interval in finite mode |
| `--offset SEC` | duration | End of the jammer interval in finite mode |
| `--ramp SEC` | `0.01` | Finite-mode rise/fall; continuous-mode startup ramp |

The times are rounded to the nearest sample using the actual transmit rate. The
reported sample indices and sample-derived times are authoritative.

Validation requires:

```text
finite: 0 <= onset < offset <= duration
finite: 2 * ramp < offset - onset
continuous: no duration or offset, and onset = 0
abs(tone_offset) < actual_rate / 2
0 < amplitude <= 1
chirp endpoints differ and both remain strictly inside Nyquist
narrowband nominal center +/- ENBW/2 remains strictly inside Nyquist
0 < pulse_on_samples <= pulse_period_samples
2 * pulse_ramp_samples < pulse_on_samples
```

### UHD and evidence

| Option | Default | Meaning |
| --- | ---: | --- |
| `--addr IP` | `192.168.10.2` | X300/UHD device address |
| `--channel N` | `0` | UHD TX channel |
| `--antenna NAME` | `TX/RX` | UHD TX antenna/port name |
| `--gain DB` | required live | Explicit hardware TX gain |
| `--clock-source SOURCE` | `internal` | UHD frequency-reference source |
| `--time-source SOURCE` | `internal` | UHD time source |
| `--start-delay SEC` | `0.25` | Relative hardware-timed start lead |
| `--manifest FILE` | none | Optional JSON evidence record, written before and after TX |
| `--calibration-id TEXT` | none | Identifier for the external RF calibration |
| `--confirm-controlled-rf` | absent | Required live-TX acknowledgement |
| `--dry-run` | absent | Validate without opening UHD |

The start delay is relative to the device's current hardware time. It is not a
GPS epoch. Independent jammer transmission does not need GPS-time synchronization. A GPSDO or
external reference may still be useful when the experiment requires lower
absolute frequency error.

## Sample-Rate Handling

The canonical offline dataset uses 2.6 Msps, while the X300 live path uses
2.5 Msps by default. Replaying a 2.6 Msps file at 2.5 Msps scales every digital
frequency and time law. For the default CW tone:

```text
500000 * 2500000 / 2600000 = 480769.23 Hz
```

`jammertx` avoids that error. It performs these steps:

1. requests `--rate` from UHD;
2. reads the actual rate returned by the device;
3. converts finite duration and applicable envelope times using that actual
   rate;
4. initializes oscillator, noise filter, chirp period, and pulse timing using
   the actual rate;
5. records requested and actual rates in the manifest.

Consequently, requested frequency offsets, ENBW, sweep period, and pulse timing
remain correct in the ideal RF model even if UHD chooses a slightly different
sample rate. The sample bytes do not need to match a 2.6 Msps file; the physical
waveform parameters do.

For bit-level comparison with `jammergen`, generate the offline reference at the
actual rate printed by `jammertx`.

## Power and J/S Calibration

Keep these quantities separate:

- **Digital amplitude:** SC16 nominal complex RMS/full-scale from
  `--amplitude`, or on-pulse amplitude for `pulsed`.
- **X300 gain:** hardware gain selected by `--gain`.
- **TX-port power:** RF power measured at the X300 output under the configured
  waveform and gain.
- **DUT-input jammer power:** power after cables, attenuation, combining,
  coupling, and shielding loss.
- **J/S:** DUT-input jammer power relative to a stated desired-signal reference.

`--amplitude 0.5` does not mean 0 dB J/S, and `--gain 0` does not mean 0 dBm.
Equal digital RMS also does not imply equal in-band spectral density across CW,
narrowband, and wideband sources.
`jammertx` deliberately has no `--js-db` option because it cannot observe the
authentic signal or the RF path.

For repeatable sweeps:

1. state whether `S` means aggregate authentic GNSS power, one satellite's
   carrier power, or another defined reference;
2. state the measurement bandwidth;
3. measure or derive the authentic reference using a traceable method;
4. measure jammer power at the DUT reference plane;
5. preserve a calibration identifier in every live manifest;
6. verify the receiver front end has not entered compression unless overload is
   the intended test stage.

Keeping digital amplitude and X300 gain fixed while changing a calibrated
external attenuation is often easier to interpret than moving several digital
and RF controls simultaneously. The appropriate method still depends on the
validated laboratory chain.

## Recommended Experiment Sequence

### 1. Validate without hardware

- Build `jammertx`.
- Run the complete command with `--dry-run`.
- Check the type-specific RF reference/span, sample indices, and Nyquist margin.
- Parse and archive the dry-run manifest.

### 2. Characterize the jammer-only RF path

In a protected, attenuated monitoring path:

- capture the X300 output with a spectrum analyzer or independent SDR;
- confirm the type-specific center/span or chirp law and spectral sign;
- measure power at the experiment reference plane;
- measure LO leakage, images, harmonics, and other spurs;
- confirm global ramps, active interval, and any pulse timing at the capture's
  time resolution;
- confirm the live manifest reports zero underflows, sequence errors, clipping,
  and time errors.

Do not connect the victim receiver until input power and hardware protection are
verified.

### 3. Establish the authentic baseline

With the jammer path connected but inactive:

- obtain the expected authentic PVT solution;
- record per-signal `C/N0`, `svUsed`, PVT validity, AGC, and jamming indicators;
- capture the spectrum with the X300 powered/configured but transmitting no
  active jammer samples;
- quantify any center-frequency LO leakage or added noise from the TX chain.

Digital zero samples do not guarantee zero RF output. LO leakage can remain at
the RF center throughout the nominal clean and recovery intervals.

### 4. Combine authentic GNSS and the selected jammer

- start below the expected detection/degradation threshold;
- run the clean, active, and recovery intervals;
- record the receiver and monitoring capture continuously;
- increase power only through the approved calibrated sweep;
- stop on unexpected front-end stress, leakage, or loss of control.

For `--continuous`, the clean prefix and recovery interval belong to the
receiver-recording timeline: start that recorder before `jammertx`, stop
`jammertx` with `Ctrl-C`, and stop the recorder only after the recovery window.

### 5. Verify recovery

After jammer offset:

- confirm the spectrum returns to its measured TX-connected baseline;
- measure reacquisition and PVT recovery time;
- confirm AGC and receiver RF indicators recover;
- repeat the authentic baseline after the test.

## Proving That the Receiver Saw the Intended Waveform Type

The offline `spectrum.png` and a receiver-produced image should show the same
waveform class, but they should not be expected to be pixel-identical. A live RF
path adds:

- transmitter LO leakage and IQ image;
- reference-clock and receiver-clock error;
- analog TX and RX filtering;
- receiver AGC and quantization;
- path loss, coupling, and multipath;
- different FFT windows, bin widths, averaging, and color scales.

Use a raw IQ capture at the DUT reference plane for quantitative acceptance.
Treat a receiver screenshot as supporting evidence.

Recommended live acceptance measurements are:

| Property | Evidence |
| --- | --- |
| CW identity | One stationary narrowband component at the requested offset |
| Narrowband identity | Center and measured ENBW/occupied width match the shaped-noise model |
| Wideband identity | Noise occupies the intended receiver/complex-Nyquist span without unintended clipping |
| Chirp identity | Captured instantaneous frequency follows start, end, direction, and repeat period |
| Pulse identity | Period, duty, per-pulse ramps, and global interval match the sample-rounded plan |
| Frequency | Captured reference frequencies are within clock and estimator uncertainty |
| Sign | Measured positive or negative side explicitly recorded |
| Envelope | Finite onset/offset or continuous first-sample/EOB boundary and applicable ramps match the run mode |
| Power | Measured in-band jammer power at the DUT reference plane |
| Image/spurs | Main-to-image ratio and largest spur recorded |
| Continuity | Zero UHD underflow and sequence-error counts |
| Receiver effect | `C/N0`, tracking, `svUsed`, PVT, AGC/jamming indicators, and recovery |

The offline 1 Hz frequency and 60 dB image-rejection gates apply only to CW.
Noise, chirp, and pulse modes have spectral, frequency-law, and timing gates.
Set separate tolerances for the measured RF chain based on its reference
accuracy, bandwidth, and analyzer configuration.

## Manifest

When `--manifest FILE` is supplied, `jammertx` writes a JSON record before
opening the hardware and overwrites it with final results at shutdown. If
startup fails or the process terminates abnormally, an `incomplete` preflight
record should remain.

The final manifest includes:

- requested and actual sample rates;
- SC16 sample contract, continuous-mode flag, and a finite planned sample count
  or `null` for an operator-stopped run;
- source type, frequency law, bandwidth, seed, phase, amplitude, global
  envelope, chirp, and pulse sample parameters;
- device address, channel, antenna, requested/actual center, and gain;
- ideal RF reference start/end frequencies;
- clock source, time source, and timed-start lead;
- optional calibration identifier;
- rendered and sent sample counts;
- digital RMS, peak component, and clipping count;
- UHD underflow, sequence-error, and time-error counts;
- completion or interruption state.

Possible `status` values are:

| Status | Meaning |
| --- | --- |
| `dry_run` | Configuration validated without opening UHD |
| `complete` | All samples sent with no detected stream fault |
| `interrupted` | Operator stopped a fault-free run; this is the expected successful end state for `--continuous` |
| `underflow` | UHD reported an empty TX buffer |
| `sequence_error` | UHD reported TX packet loss/order failure |
| `time_error` | The scheduled first sample missed its hardware time |
| `incomplete` | Startup, device, validation, or abnormal-exit failure |

The manifest records digital and device settings. It does not claim measured
DUT-input power or J/S.

## Troubleshooting

### `live TX requires --confirm-controlled-rf`

The safety acknowledgement is missing. Do not add it until the conducted or
shielded setup has been verified.

### `live TX requires an explicit --gain`

Specify the gain from the active calibration procedure. There is no implicit
live gain.

### Frequency or nominal narrowband span is outside Nyquist

The strict condition is:

```text
abs(tone_offset) < actual_rate / 2
```

CW/pulsed offsets and both chirp endpoints must satisfy this strict bound. For
narrowband, `abs(center) + ENBW/2` must also be below Nyquist. Increase the
sample rate, reduce the offset/span, or reduce ENBW. A reference exactly at
Nyquist is rejected.

### Tone appears on the opposite side

The monitoring chain likely has an IQ inversion or opposite spectral sign
convention. Record the measured convention. Do not silently change analysis
labels to make the plot look expected.

### Tone appears near 480.769 kHz instead of 500 kHz

This is characteristic of replaying samples generated for 2.6 Msps at 2.5
Msps. Use `jammertx`, which initializes from the actual UHD rate, or regenerate
the file for the playback rate.

### A line remains at zero offset while the jammer is inactive

That line is likely TX LO leakage. Measure it during the TX-connected baseline.
Use the laboratory's approved RF gating/calibration method if the baseline is
not acceptable; changing the digital envelope does not remove LO leakage.

### Underflow or sequence error

The RF waveform has a discontinuity or missing samples and the run is not valid
for waveform comparison. Check host load, network path, UHD configuration, and
sample rate, then repeat the run. Do not accept a receiver plot from a stream
whose manifest reports these faults.

### Timed-start error

Increase `--start-delay` and verify the device time source is operating. The
minimum accepted lead is 20 ms, but a loaded host or networked X300 can need
more margin.

### Receiver loses PVT immediately at the lowest intended level

Stop and re-check the calibrated DUT-input power, combiner loss, leakage,
front-end overload indicators, and reference definition. Do not infer that the
requested digital amplitude represented the intended J/S.

## Contributor Notes

The source boundary is:

```text
tools/jammer_source.c  -> deterministic multi-waveform samples and digital metrics
tools/jammergen.c      -> offline file backend and offline manifest
player/jammertx.cpp    -> actual-rate UHD backend and live manifest
```

Changes to oscillator, envelope, quantization, or normalization semantics must
be made in the shared source and tested in both backends. Do not duplicate the
source inside `gpssim.c`, `player/x300tx.cpp`, or `player/bladetx.cpp`.

The shared-source unit test verifies known CW quadrature samples,
render-chunk invariance, stochastic seed behavior, narrowband correlation,
chirp frequency law, pulse timing, RMS, clipping, and invalid configurations.
The offline dataset test remains the end-to-end regression for `jammergen`.

Future stepped, hopped, IQ-replay, or code-aware sources should extend this
independent jammer-source architecture and add source-specific acceptance
tests. They must not become branches that replace desired satellite channels.
