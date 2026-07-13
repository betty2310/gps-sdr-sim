# Offline Multi-Waveform Jamming Dataset: Researcher and Contributor Guide

This document is the complete operating guide for the repository's standalone
synthetic jamming dataset pipeline. It supports continuous-wave (CW),
narrowband noise, wideband noise, chirp, and pulsed-CW sources. Read it in one
of two ways:

- Researchers should start at [Research workflow](#research-workflow).
- Contributors should start at [Contributor guide](#contributor-guide).

The pipeline generates clean GPS L1 C/A baseband, generates one independent
jammer source, combines both signals with deterministic AWGN at measured power ratios,
checks the waveform numerically, and optionally runs every fixture through
GNSS-SDR. The workflow is offline-only. It does not configure an SDR or provide
RF transmission commands.

## Contents

- [Scope and safety boundary](#scope-and-safety-boundary)
- [What the pipeline produces](#what-the-pipeline-produces)
- [Architecture and data flow](#architecture-and-data-flow)
- [Research workflow](#research-workflow)
- [Manual component workflow](#manual-component-workflow)
- [Output directory reference](#output-directory-reference)
- [Contributor guide](#contributor-guide)
- [Troubleshooting](#troubleshooting)
- [Command summary](#command-summary)

## Scope and safety boundary

This pipeline is intended for reproducible receiver research using files:

```text
clean GPS IQ -> independent jammer IQ -> offline mixer -> IQ fixtures -> GNSS-SDR
```

It is intentionally different from the legacy `gps-sdr-sim -A` controls.
`jam_drop` and `jam_noise` modify simulated satellite channels inside
`gps-sdr-sim`; they are not independent RF interference sources. New external
jammer research belongs in this standalone source-and-mixer pipeline, not in the
clean GPS generator.

Do not use this guide to radiate signals. GNSS interference can be unsafe and
illegal. Keep experiments in files or in a properly authorized, shielded,
conducted laboratory setup governed by local rules.

The separate [real-time X300 jammer guide](realtime-cw-jammer-x300.md) documents the
controlled hardware backend. It shares the same deterministic renderer, but
it has a different power-calibration and evidence boundary because authentic
GNSS and jammer RF are combined physically rather than by `iqmix`.

### Authentic-signal boundary

None of the five implemented jammer sources needs an authentic GNSS recording.
CW, both noise modes, chirp, and pulsed CW are independent synthetic emitters;
`jammergen` can create them from parameters and a seed alone. The desired GPS
input can likewise come from `gps-sdr-sim` for a fully synthetic experiment.

Authentic or captured IQ is required when the research claim depends on the
real sky, a field-observed interferer, propagation/multipath, antenna coupling,
analog front-end compression outside the recorded band, or performance on an
actual receiver installation. Those effects cannot be reconstructed faithfully
from these parameterized sources. Use `--clean-input` for a lawful authentic or
hardware-simulator capture, and add an explicit IQ-replay source in future if a
recorded jammer must be reproduced; do not label a synthetic fixture as field
authentic.

## What the pipeline produces

The canonical profile uses the following fixed sample contract:

| Property | Canonical value |
| --- | ---: |
| Sample format | little-endian signed 16-bit interleaved `I,Q` |
| Complex sample rate | 2,600,000 samples/s |
| Duration | 90 s |
| Complex samples per IQ file | 234,000,000 |
| Bytes per IQ file | 936,000,000 |
| Clean prefix | 0-30 s |
| Jammer-active interval | 30-60 s |
| Recovery interval | 60-90 s |
| Default source | CW at +500 kHz baseband |
| Default phase | 0 rad at onset |
| Rise and fall | 10 ms raised-cosine ramps |
| Default CW source RMS | 0.5 of int16 full scale on the plateau |
| Coarse J/S sweep | -20, -10, 0, +10, +20, +30 dB |
| Initial clean/AWGN SNR | -10 dB full-band |
| Default noise seed | 20260712 |
| Receiver calibration target | 45 +/- 2 dB-Hz median C/N0 |

The `fast` profile preserves the sample rate, sample type, selected source, and
ramp but is only 3 s long: clean from 0-1 s, jammer active from 1-2 s, and recovery from 2-3 s.
It creates only a clean control and a 0 dB J/S fixture. Use it for development,
not for a publication result.

The `verification` profile is a bounded GNSS-SDR comparison: 90 s total, a
40 s clean prefix, jammer active from 40-60 s, recovery from 60-90 s, and one
+20 dB J/S fixture. Its longer clean prefix gives all fixed visible PRNs time
to acquire and decode navigation data before impairment. It is intended for a
single cross-waveform verification run; repeated seeds and multiple J/S levels
are still required for uncertainty or threshold claims.

## Architecture and data flow

```text
frozen RINEX navigation file
            |
            v
      gps-sdr-sim ----------------------> clean-source.bin
                                               |
jammer parameters                              |
       |                                       |
       v                                       v
 tools/jammergen.c -> <type>-jammer.bin -> tools/iqmix.c
                                          |
                       clean, J/S, AWGN, common gain
                                          |
                                          v
                               clean/fixture.bin
                               js-m20db/fixture.bin
                               ...
                               js-p30db/fixture.bin
                                          |
                     +--------------------+--------------------+
                     |                                         |
                     v                                         v
        numerical waveform analysis                        GNSS-SDR
        reports + spectrum plots                 tracking + NMEA + reports
                     |                                         |
                     +--------------------+--------------------+
                                          |
                                          v
                                  dataset-index.json
```

### 1. Clean GPS generation

`processing/cw_dataset.py:generate_clean()` either:

1. invokes `gps-sdr-sim` with a frozen RINEX file, scenario time, static
   location, 2.6 Msps, and 16-bit output; or
2. copies the first exact profile-sized window from `--clean-input`.

For generated sources, the workflow asks `gps-sdr-sim` for one extra second,
keeps exactly the required first sample window, removes the temporary oversized
file, hashes the RINEX input, and parses visible PRNs from the generator log.
The generator is run inside the output directory and receives a short output
basename; keep this behavior when refactoring because the legacy generator has
path-length-sensitive output handling.

The visible PRNs are sorted by descending elevation and later used to create a
source-specific GNSS-SDR configuration. With `--clean-input`, this geometry is
unknown, so the researcher should supply `--receiver-prns` when GNSS-SDR will
run.

### 2. Independent multi-waveform generation

`tools/jammergen.c` and `player/jammertx.cpp` both use
`tools/jammer_source.c`. The renderer is stateful, bounded-memory, deterministic
across render chunk sizes, and emits SC16 `I,Q`. The global onset/offset
envelope is zero outside the active interval and uses half-cosine ramps.

The implemented source types are:

| `--type` | Digital model | Main parameters | Authentic IQ required? |
| --- | --- | --- | --- |
| `cw` | Proper complex fixed-frequency tone | `--frequency`, `--phase` | No |
| `narrowband` | Proper-complex Gaussian noise through a one-pole low-pass, then frequency translated | `--frequency`, `--bandwidth`, `--seed` | No |
| `wideband` | Unfiltered proper-complex Gaussian noise across the complex Nyquist band | `--seed` | No |
| `chirp` | Phase-continuous repeating linear sweep | `--frequency`, `--end-frequency`, `--chirp-period` | No |
| `pulsed` | Phase-continuous CW with a periodic raised-cosine gate | `--frequency`, `--pulse-period`, `--pulse-duty`, `--pulse-ramp` | No |

CW is:

```text
j[n] = A e^(j(phi + 2 pi f (n - onset) / Fs)) envelope[n]
```

For narrowband noise, the requested `--bandwidth` is the two-sided equivalent
noise bandwidth (ENBW), not a brick-wall occupied width. With sample rate `Fs`
and requested ENBW `B`, the low-pass pole is:

```text
a = (Fs - B) / (Fs + B)
y[n] = a y[n-1] + sqrt(1-a^2) x[n]
```

where `x[n]` is unit-power proper-complex Gaussian noise. Multiplication by the
complex center oscillator moves this low-pass process to `--frequency`.
Wideband mode uses `x[n]` directly, so its digital support is the whole complex
Nyquist interval. Both modes use deterministic xorshift/Box-Muller samples; the
same nonzero seed is bit-repeatable, while seed zero maps to a fixed internal
state.

For a chirp of `Ns` samples, instantaneous frequency advances linearly from
`f0` to `f1` and then restarts at `f0`; carrier phase remains continuous at the
period boundary. Pulsed mode starts its pulse schedule at global onset. Its
carrier continues advancing through off portions, so gating does not reset
phase at every pulse.

For CW, chirp, and both noise modes, `--amplitude` is nominal active complex RMS
as a fraction of int16 full scale. For pulsed mode it is on-pulse amplitude; the
active-interval RMS is lower according to duty cycle and pulse ramps. The
default is `0.5` for CW/chirp/pulsed and `0.15` for stochastic modes to retain
SC16 peak margin. `iqmix` always measures actual average plateau power before
applying J/S, so these differing source amplitudes do not change the requested
fixture ratio. Unsupported type names fail rather than falling back to CW.

### 3. Measured-power mixing

`tools/iqmix.c` first checks that clean and jammer inputs are exactly
`samples * 4` bytes. It measures both complex powers over the steady plateau,
excluding the rise and fall ramps:

```text
Pc = mean(|clean[n]|^2)
Pj = mean(|jammer[n]|^2)
alpha = sqrt(10^(J/S / 10) * Pc / Pj)
```

For a requested full-band clean/AWGN SNR, the complex AWGN RMS is:

```text
noise_rms = sqrt(Pc / 10^(SNR / 10))
```

The proper-complex Gaussian generator uses `sigma = noise_rms / sqrt(2)` for
each of I and Q. A deterministic xorshift/Box-Muller sequence makes a given
seed bit-repeatable. A seed value of zero maps to a fixed nonzero internal RNG
state.

The mixed sample before quantization is:

```text
y[n] = gain * (clean[n] + alpha * jammer[n] + noise[n])
```

For the clean control, `alpha` is zero. `iqmix` uses two streaming passes: one
for input power measurement and one for mixing. It records requested and
measured J/S, measured SNR, noise RMS, peak component, headroom, output RMS, and
clipping in its JSON manifest.

### 4. One gain and one noise realization per sweep

The Python workflow dry-runs the highest requested J/S level at gain 1.0 and
derives:

```text
common_gain = min(1, 32767 * 10^(-1/20) / highest_level_peak)
```

This targets at least 1 dB component headroom. The same gain is then applied to
the clean control and every J/S level. Within one seed, the same deterministic
AWGN realization is also used in every fixture. These choices prevent
fixture-specific scaling or noise luck from masquerading as a jammer effect.

Do not independently normalize fixtures after mixing. Doing so breaks the
power comparison encoded in the manifests.

### 5. Numerical waveform acceptance

`analyze_fixture()` reads the IQ in bounded chunks and fails the run if any of
these conditions is not met:

| Check | Acceptance rule |
| --- | --- |
| Exact size | jammer and mixed file match the profile byte count |
| Active RMS | error is at most 0.1 dB |
| CW | frequency error at most 1 Hz and negative-frequency image rejection at least 60 dB |
| Narrowband | center, 90%-power width, and requested-band power concentration match the ENBW model |
| Wideband | 90%-power occupied width spans at least 80% of the complex sample rate |
| Chirp | instantaneous-frequency law matches requested start, end, and period |
| Pulsed | one complete pulse has the requested period, duty, ramps, and exact off samples |
| Envelope | no nonzero complex samples outside `[onset, offset)` |
| Clipping | zero source or mixed clipped I/Q components |
| Measured J/S | error is at most 0.1 dB when a jammer is enabled |
| Measured SNR | error is at most 0.1 dB when requested by SNR |

CW frequency is estimated from the mean one-sample phase step over up to two
million plateau samples. Chirp analysis compares per-sample phase steps with
the requested law. Noise analysis uses averaged PSD, and pulse analysis compares
one aligned period with the exact discrete envelope. The common checks remain
independent of type.

Unless `--no-plots` is passed, each fixture also receives:

- `spectrum.png`: jammer-only PSD and clean-versus-mixed plateau PSD;
- `spectrogram.png`: full-run sampled time/frequency view with onset and offset;
- `spectrogram-active-detail.png`: dense jammer-only view of the first 250 ms
  after the global ramp, resolving short chirp sweeps and pulse periods;
- `block-power.png`: 100 ms complex power with the active interval shaded.

### 6. GNSS-SDR validation

The checked-in `sdr-monitor/cw-dataset-receiver.conf` is a generic GPS L1 C/A,
2.6 Msps `ishort` file receiver. It has 31 automatic channels because GNSS-SDR
rejects a higher GPS L1 C/A channel count. For generated clean data, the
workflow rewrites the channel count, acquires all source-derived fixed channels
concurrently, and appends their PRN assignments to an archived
`resolved-receiver.conf`. Concurrent acquisition avoids telemetry/PVT timing
depending on a four-channel acquisition queue when clean and jammed runs are
compared separately.

The workflow collects:

- process output in `receiver.log`;
- the exact resolved configuration;
- tracking MAT dumps and per-PRN 1 Hz median C/N0;
- NMEA GGA valid-fix state and usable-satellite counts;
- receiver plots for C/N0, tracked satellites, usable satellites, and PVT;
- one PNG and one PDF per PRN containing post-correlation prompt spectrum and
  spectrogram, plus `prn-spectral-report.json`;
- `receiver-report.json` with interval summaries and recovery time.

All GPS L1 C/A PRNs overlap in the same RF band, so a raw IQ FFT cannot be
isolated or labeled by PRN. The per-PRN figures use GNSS-SDR `Prompt_I` and
`Prompt_Q` after code/carrier wipeoff. They are despread tracking-channel
spectra, not PRN-isolated RF spectra. Prompt records at 1 ms and 20 ms
integration are averaged onto a 50 Hz grid without pretending that the raw
records are uniformly sampled; low-coverage spectrogram windows are blanked.

A receiver fixture is valid for interpretation only if it obtains at least one
valid pre-onset PVT fix using four or more satellites. This is the
`clean_prefix_gate`. Acquisition or tracking alone is not enough.

The runner also converts known GNSS-SDR flowgraph messages into a logical
failure even when the GNSS-SDR process itself returns zero.

### 7. Optional C/N0 calibration

For canonical and verification runs with a receiver enabled, calibration is on
by default. A 15 s clean pilot is mixed and processed by GNSS-SDR. The workflow measures
median tracking C/N0 after the first 3 s and updates noise RMS by:

```text
next_noise_rms = noise_rms * 10^((measured_cn0 - target_cn0) / 20)
```

It stops when the result is within `--cn0-tolerance` of `--target-cn0`, or
fails after `--calibration-iterations`. The accepted noise RMS is frozen for
the whole sweep and recorded in `calibration/calibration-report.json` and the
dataset index.

## Research workflow

Run the commands in this section from the repository root unless the command
explicitly changes directory.

### Step 0: Install prerequisites

Required for waveform generation and analysis:

- a C compiler and `make`;
- Python 3.12 or newer;
- [uv](https://docs.astral.sh/uv/);
- a GPS RINEX navigation file covering the chosen scenario time.

Required only for receiver validation:

- a working GNSS-SDR executable compatible with the checked-in configuration.

Set the receiver explicitly when several builds are installed:

```sh
export GNSS_SDR_BIN=/absolute/path/to/gnss-sdr
"$GNSS_SDR_BIN" --version
```

If `GNSS_SDR_BIN` is unset, the workflow prefers an executable sibling build at
`../gnss-sdr/install/gnss-sdr`, then falls back to `gnss-sdr` on `PATH`.

### Step 1: Build and create the Python environment

```sh
make gps-sdr-sim jammergen iqmix
uv sync --project processing --locked
```

Check the user-facing interfaces:

```sh
./jammergen --help
./iqmix --help
uv run --project processing python processing/cw_dataset.py create --help
```

### Step 2: Run the fast smoke dataset

The fastest end-to-end check uses an existing exact-format clean IQ file:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile fast \
  --clean-input /absolute/path/to/clean-int16-iq.bin \
  --receiver none \
  --output-dir /tmp/cw-fast
```

If no clean IQ is available, generate one from RINEX:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile fast \
  --rinex /absolute/path/to/frozen-navigation.rnx \
  --start-time 2022/01/01,00:00:00 \
  --receiver none \
  --output-dir /tmp/cw-fast
```

Confirm both fixtures passed and the four plots exist:

```sh
jq '.fixtures[] | {label, js_db, pass, waveform_report}' \
  /tmp/cw-fast/dataset-index.json
```

Select the waveform explicitly for non-CW smoke runs. These commands show the
implemented parameter surfaces; append the same clean/RINEX, receiver, and
output arguments used above:

```text
# 100 kHz two-sided ENBW centered at +500 kHz
--jammer-type narrowband --frequency-hz 500000 --bandwidth-hz 100000 \
  --jammer-seed 20260712

# Proper-complex Gaussian noise over the complex Nyquist band
--jammer-type wideband --jammer-seed 20260712

# Phase-continuous -500 kHz to +500 kHz sweep repeated every 100 ms
--jammer-type chirp --frequency-hz=-500000 --end-frequency-hz 500000 \
  --chirp-period-s 0.1

# +500 kHz CW, 10 ms period, 10% duty, 50 us pulse edges
--jammer-type pulsed --frequency-hz 500000 --pulse-period-s 0.01 \
  --pulse-duty-cycle 0.1 --pulse-ramp-s 0.00005
```

Matched-code is an offline-only source driven by target-channel state exported
by `gps-sdr-sim` in the same clean-generation run. Targets are explicit,
ordered, and duplicate-free. The source repeats each target's GPS L1 C/A code
with a constant positive data symbol, follows the exported code phase, carrier
Doppler, and code rate, and uses an independently seeded carrier phase:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile verification \
  --rinex /absolute/path/to/frozen-navigation.rnx \
  --start-time 2026/07/13,03:20:00 \
  --jammer-type matched-code \
  --target-prns 13 \
  --jammer-seed 20260712 \
  --js-levels 0 \
  --receiver all \
  --output-dir matched-code-dataset-output/verification-prn-13-js-p0db
```

For a supplied clean fixture, also provide the trajectory exported with that
exact clean generation run:

```text
--clean-input /path/to/clean-source.bin \
--trajectory-input /path/to/target-trajectory.csv
```

Do not pair a trajectory with clean IQ from another run. The workflow hashes
and links the trajectory in the source, fixture, and dataset-index manifests.
Matched-code remains file based; it is not accepted by a hardware transmitter.

The matched-code path currently supports the `fast` development profile and
the GNSS-SDR-backed `verification` profile. It deliberately rejects
`--profile canonical`: RTCM live/replay scenario freezing, separate cold-start
receiver evaluation, and five independent matched-source carrier-phase seeds
are not implemented yet. This prevents a bounded verification dataset from
being mislabeled as a complete canonical research campaign.

`--jammer-seed` controls the narrowband/wideband source samples. `--seed`
controls mixer AWGN. Record both; changing one does not change the other.

### Step 3: Freeze the research inputs

Choose a RINEX navigation file and a scenario time actually covered by it. Keep
the original file unchanged for all runs in an experiment. Record its digest:

```sh
shasum -a 256 /absolute/path/to/frozen-navigation.rnx
```

Choose and record:

- scenario time and location;
- GNSS-SDR binary and version;
- receiver configuration;
- jammer type and all type-specific parameters;
- J/S grid;
- jammer seed, mixer base seed, and seed count;
- code commit.

The manifests capture these values, but writing them into a lab notebook before
the run protects against accidental input changes.

### Step 4: Run the canonical coarse sweep without GNSS-SDR

Use this first when checking disk, waveform generation, and plots:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical \
  --rinex /absolute/path/to/frozen-navigation.rnx \
  --start-time 2022/01/01,00:00:00 \
  --location 21.0047844,105.8460541,5 \
  --receiver none \
  --no-calibrate-noise \
  --output-dir cw-dataset-output/coarse-waveform
```

This materializes nine large IQ files: one shared clean source, one shared
jammer, a clean-plus-noise control, and six impaired fixtures. The built-in
preflight checks core IQ storage plus 10%, but receiver dumps and calibration
artifacts need additional space. Plan for more than the preflight minimum.

### Step 5: Run the canonical receiver experiment

Use explicit receiver mode for a research result:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical \
  --rinex /absolute/path/to/frozen-navigation.rnx \
  --start-time 2022/01/01,00:00:00 \
  --location 21.0047844,105.8460541,5 \
  --receiver all \
  --gnss-sdr "$GNSS_SDR_BIN" \
  --output-dir cw-dataset-output/coarse-receiver
```

Canonical `--receiver all` enables the C/N0 calibration pilot unless
`--no-calibrate-noise` is supplied. `--receiver auto` has the same effect only
when a receiver is discoverable; explicit `all` is preferable in a protocol.

To validate only the control before spending time on the sweep:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical \
  --rinex /absolute/path/to/frozen-navigation.rnx \
  --start-time 2022/01/01,00:00:00 \
  --receiver clean \
  --gnss-sdr "$GNSS_SDR_BIN" \
  --output-dir cw-dataset-output/clean-gate
```

For a reused clean file, supply the source-visible PRNs in desired acquisition
priority. Without this, the generic 31-channel configuration is used:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical \
  --clean-input /absolute/path/to/clean-source.bin \
  --receiver-prns 18,24,23,15,10,29,32,25,5,12,13,27 \
  --receiver all \
  --gnss-sdr "$GNSS_SDR_BIN" \
  --output-dir cw-dataset-output/reused-clean
```

The PRN list above is only an example. Use the geometry for the clean source,
not a copied list from another scenario.

### Step 5a: Run the bounded five-type GNSS-SDR verification

Use the `verification` profile to compare the five modes at one measured
+20 dB J/S point with a 40 s clean prefix. This is a descriptive implementation
check, not a replacement for a repeated-seed J/S sweep:

```sh
RUN_ROOT=analysis-output/runs
for TYPE in cw narrowband wideband chirp pulsed; do
  uv run --project processing python processing/cw_dataset.py create \
    --profile verification \
    --rinex /absolute/path/to/frozen-navigation.rnx \
    --start-time 2025/01/03,00:00:00 \
    --location 21.0047844,105.8460541,5 \
    --jammer-type "$TYPE" \
    --receiver all \
    --gnss-sdr "$GNSS_SDR_BIN" \
    --output-dir "$RUN_ROOT/$TYPE" || exit $?
done
```

Build the cross-type report, exact metric table, figure catalog, and PRN
gallery only after all five runs pass:

```sh
uv run --project processing python processing/jamming_campaign_report.py \
  --runs-root analysis-output/runs \
  --output-dir analysis-output
```

The report generator validates one clean-IQ hash, one timing profile, one
GNSS-SDR version, passing clean-prefix gates on both members of every pair,
passing waveform reports, and nine PRN spectral reports per receiver. It emits
`analysis-report.md`, `stats-appendix.md`, `figure-catalog.md`,
`prn-gallery.md`, `campaign-metrics.json`, and PNG/PDF summary figures.

### Step 6: Inspect waveform evidence

Start with the top-level index:

```sh
jq '{profile, source, shared, first_measured_degradation_threshold}' \
  cw-dataset-output/coarse-receiver/dataset-index.json

jq '.fixtures[] | {label, js_db, noise_seed, pass, waveform_report}' \
  cw-dataset-output/coarse-receiver/dataset-index.json
```

Inspect a fixture's numerical checks. `source_measurements` is type-specific:

```sh
jq '{source_type, pass, checks, source_measurements, plots}' \
  cw-dataset-output/coarse-receiver/js-p0db/analysis/waveform-report.json
```

Open these images for the same fixture:

```text
js-p0db/analysis/spectrum.png
js-p0db/analysis/spectrogram.png
js-p0db/analysis/spectrogram-active-detail.png
js-p0db/analysis/block-power.png
js-p0db/analysis/correlation-heatmap.png
```

For default CW, the spectrum should show a complex line at +500 kHz, not
-500 kHz. Narrowband should show a centered shaped-noise lobe, wideband should
occupy most of Nyquist, chirp should traverse the requested sweep repeatedly,
and pulsed should show periodic time-domain power. Every mode must be absent
outside 30-60 s, have smooth configured edges, and return to the pre-onset
power level after 60 s.

For matched-code, also inspect the correlation heatmap and the
`matched_code_alignment`, `matched_carrier_doppler`, `matched_code_drift`, and
`selected_to_unselected_margin` gates. A single target must exceed the
strongest unselected-PRN response by at least 18 dB. A multi-target composite
is checked against its deterministic reference correlation map.

### Step 7: Inspect receiver evidence

Check the clean control first:

```sh
jq '{exit_code, metrics}' \
  cw-dataset-output/coarse-receiver/clean/receiver/receiver-report.json
```

Do not interpret impaired fixtures unless:

```text
.exit_code == 0
.metrics.clean_prefix_gate == true
```

Then compare the jammer interval and recovery:

```sh
jq '{
  stable_median_cn0_db_hz: .metrics.stable_median_cn0_db_hz,
  jammer: .metrics.pvt_intervals.jammer,
  recovery: .metrics.pvt_intervals.recovery,
  time_to_restored_pvt_s: .metrics.time_to_restored_pvt_s
}' cw-dataset-output/coarse-receiver/js-p0db/receiver/receiver-report.json
```

Also inspect:

```text
receiver/cn0-per-prn.png
receiver/receiver-state.png
receiver/prn-spectral/prn-spectral-report.json
receiver/prn-spectral/GNN-prompt-spectrum-spectrogram.png
receiver/prn-spectral/GNN-prompt-spectrum-spectrogram.pdf
receiver/receiver.log
receiver/resolved-receiver.conf
```

`stable_median_cn0_db_hz` is the compatibility name for the clean-prefix
aggregate. Time-aligned aggregates are under
`.metrics.median_cn0_db_hz_by_interval`, and per-PRN interval medians are under
`.metrics.per_prn[PRN]`.

To regenerate PRN figures without rerunning GNSS-SDR:

```sh
uv run --project processing python processing/cw_dataset.py plot-prn \
  --profile verification \
  --tracking-dir RUN/js-p20db/receiver \
  --output-dir RUN/js-p20db/receiver
```

### Step 8: Refine around a coarse threshold

The index reports the first level where any configured degradation signal is
true:

```sh
jq '.first_measured_degradation_threshold' \
  cw-dataset-output/coarse-receiver/dataset-index.json
```

If the coarse result suggests a center of +10 dB, create a 6, 8, 10, 12, 14 dB
grid with five consecutive independent AWGN seeds:

```sh
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical \
  --rinex /absolute/path/to/frozen-navigation.rnx \
  --start-time 2022/01/01,00:00:00 \
  --refine-around 10 \
  --seed 20260712 \
  --seed-count 5 \
  --receiver all \
  --gnss-sdr "$GNSS_SDR_BIN" \
  --output-dir cw-dataset-output/refined-10db
```

Without `--seed-count`, refinement defaults to five seeds. Outside refinement,
the default is one. Labels include `-seed-N` whenever multiple seeds are used.

The dataset-index C/N0 threshold flag compares the clean control and impaired
fixture over the same jammer interval. Each receiver aggregate uses its own
clean-prefix-stable PRN set, so a strict research comparison should still
intersect the stable sets and calculate matched per-PRN deltas. The five-type
campaign report does this explicitly; do not treat opportunistically acquired
or late-locking PRNs as independent repetitions.

### Step 9: Re-run waveform analysis for an existing fixture

The `analyze` subcommand is useful after copying a dataset or changing analysis
code. It does not rerun the receiver:

```sh
TYPE=cw
uv run --project processing python processing/cw_dataset.py analyze \
  --profile canonical \
  --jammer "cw-dataset-output/coarse-receiver/$TYPE-jammer.bin" \
  --jammer-manifest "cw-dataset-output/coarse-receiver/$TYPE-jammer.tool.json" \
  --clean cw-dataset-output/coarse-receiver/clean-source.bin \
  --mixed cw-dataset-output/coarse-receiver/js-p0db/fixture.bin \
  --mix-manifest cw-dataset-output/coarse-receiver/js-p0db/iqmix.tool.json \
  --output-dir cw-dataset-output/coarse-receiver/js-p0db/analysis
```

Omit `--no-plots` to create or refresh the spectrum, full-run spectrogram,
dense active-window spectrogram, and block-power images.

### Step 10: Archive a reproducible result

Preserve at minimum:

- `dataset-index.json`;
- every fixture `manifest.json` and `iqmix.tool.json`;
- `<type>-jammer.tool.json` and `<type>-jammer.manifest.json`;
- `calibration/calibration-report.json`, when present;
- waveform and receiver reports and plots;
- `gps-sdr-sim.log`;
- each `resolved-receiver.conf` and `receiver.log`;
- the frozen RINEX file or an immutable source plus its recorded SHA-256;
- the Git commit and GNSS-SDR version.

Generated IQ is ignored by Git. Store large binary fixtures in an appropriate
dataset archive and verify their hashes after transfer. Do not publish only the
plots: they are diagnostic views, not sufficient provenance.

## Manual component workflow

The Python workflow is the supported reproducible path. This section exposes
each component for debugging and method development. The example creates a
canonical clean source, default CW jammer, dry-run gain, clean control, and
0 dB fixture. Replace the `jammergen` type and type-specific arguments using
the source table above; `iqmix` itself is unchanged.

```sh
REPO=$(pwd)
RUN="$REPO/cw-dataset-output/manual"
RINEX=/absolute/path/to/frozen-navigation.rnx
mkdir -p "$RUN/clean" "$RUN/js-p0db"
```

Generate 91 s inside the output directory, then retain exactly 90 s:

```sh
(
  cd "$RUN"
  "$REPO/gps-sdr-sim" \
    -e "$RINEX" \
    -l 21.0047844,105.8460541,5 \
    -t 2022/01/01,00:00:00 \
    -d 91 \
    -s 2600000 \
    -b 16 \
    -o clean-overgenerated.bin
)

dd if="$RUN/clean-overgenerated.bin" \
  of="$RUN/clean-source.bin" bs=4000000 count=234
```

Generate the independent jammer:

```sh
./jammergen \
  --type cw \
  --output "$RUN/cw-jammer.bin" \
  --manifest "$RUN/cw-jammer.tool.json" \
  --sample-rate 2600000 \
  --duration 90 \
  --frequency 500000 \
  --phase 0 \
  --amplitude 0.5 \
  --onset 30 \
  --offset 60 \
  --ramp 0.01
```

Measure the worst-case +30 dB fixture without writing IQ:

```sh
./iqmix \
  --clean "$RUN/clean-source.bin" \
  --jammer "$RUN/cw-jammer.bin" \
  --manifest "$RUN/common-gain-dry-run.json" \
  --samples 234000000 \
  --onset-sample 78000000 \
  --offset-sample 156000000 \
  --ramp-samples 26000 \
  --js-db 30 \
  --snr-db -10 \
  --seed 20260712 \
  --gain 1 \
  --dry-run
```

Calculate the shared gain from that manifest:

```sh
GAIN=$(uv run --project processing python -c '
import json, sys
data = json.load(open(sys.argv[1]))
peak = data["measurements"]["pre_gain_peak_component_counts"]
print(min(1.0, 32767 * 10 ** (-1 / 20) / peak))
' "$RUN/common-gain-dry-run.json")
```

Create the clean-plus-noise control:

```sh
./iqmix \
  --clean "$RUN/clean-source.bin" \
  --jammer "$RUN/cw-jammer.bin" \
  --output "$RUN/clean/fixture.bin" \
  --manifest "$RUN/clean/iqmix.tool.json" \
  --samples 234000000 \
  --onset-sample 78000000 \
  --offset-sample 156000000 \
  --ramp-samples 26000 \
  --disable-jammer \
  --snr-db -10 \
  --seed 20260712 \
  --gain "$GAIN"
```

Create the 0 dB J/S fixture with the same AWGN and gain:

```sh
./iqmix \
  --clean "$RUN/clean-source.bin" \
  --jammer "$RUN/cw-jammer.bin" \
  --output "$RUN/js-p0db/fixture.bin" \
  --manifest "$RUN/js-p0db/iqmix.tool.json" \
  --samples 234000000 \
  --onset-sample 78000000 \
  --offset-sample 156000000 \
  --ramp-samples 26000 \
  --js-db 0 \
  --snr-db -10 \
  --seed 20260712 \
  --gain "$GAIN"
```

Run the numerical analyzer using the Step 9 command with `$RUN` paths. This
manual route intentionally does not reproduce receiver C/N0 calibration or the
augmented provenance manifests; use `create` for publishable datasets.

## Output directory reference

A default-CW canonical receiver run has this shape. For another source type,
replace the three `cw-jammer` basenames with `<type>-jammer`:

```text
RUN/
├── dataset-index.json
├── clean-source.bin
├── gps-sdr-sim.log
├── cw-jammer.bin
├── cw-jammer.tool.json
├── cw-jammer.manifest.json
├── common-gain-dry-run.json
├── calibration/
│   ├── calibration-report.json
│   └── iteration-N/
│       ├── pilot.bin
│       ├── iqmix.tool.json
│       └── receiver/
├── clean/
│   ├── fixture.bin
│   ├── iqmix.tool.json
│   ├── manifest.json
│   ├── analysis/
│   │   ├── waveform-report.json
│   │   ├── waveform-summary.txt
│   │   ├── spectrum.png
│   │   ├── spectrogram.png
│   │   ├── spectrogram-active-detail.png
│   │   └── block-power.png
│   └── receiver/
│       ├── receiver-report.json
│       ├── receiver.log
│       ├── resolved-receiver.conf
│       ├── cn0-per-prn.png
│       ├── receiver-state.png
│       ├── prn-spectral/
│       │   ├── prn-spectral-report.json
│       │   └── GNN-prompt-spectrum-spectrogram.{png,pdf}
│       └── GNSS-SDR raw dumps
├── js-m20db/
├── js-m10db/
├── js-p0db/
├── js-p10db/
├── js-p20db/
└── js-p30db/
```

### Manifest responsibilities

| File | Purpose |
| --- | --- |
| `<type>-jammer.tool.json` | Direct `jammergen` parameters and measurements |
| `<type>-jammer.manifest.json` | Jammer metadata plus SHA-256 |
| `iqmix.tool.json` | Direct mixer parameters and measured power metrics |
| `manifest.json` | Fixture profile, source, tool/receiver provenance, hashes, common scaling |
| `waveform-report.json` | Machine-readable numerical acceptance result |
| `receiver-report.json` | Parsed GNSS-SDR evidence and metrics |
| `dataset-index.json` | Top-level fixture inventory, shared settings, and first threshold signal |

Treat paths in manifests as provenance from the generation host; hashes are the
portable identity. When moving a dataset, paths can become stale while hashes
remain meaningful.

## Contributor guide

### Where to start

Use this map before editing:

| Goal | Start here | Also inspect |
| --- | --- | --- |
| Change jammer sample generation | `tools/jammer_source.{c,h}` | both CLIs, waveform tests, and analyzer |
| Change J/S, AWGN, gain, or clipping logic | `tools/iqmix.c` | manifests, `derive_common_gain()` |
| Change profiles or orchestration | `processing/cw_dataset.py` | tests and this guide |
| Change numerical acceptance or plots | `analyze_fixture()` and plotting helpers | report schema and corruption tests |
| Change GNSS-SDR execution | `run_receiver()` | receiver config and receiver reports |
| Change receiver metrics | `parse_nmea()`, `parse_tracking()`, `receiver_metrics()` | threshold logic |
| Change C/N0 calibration | `calibrate_noise()` | clean-prefix gate and calibration report |
| Change GNSS-SDR blocks/channels | `sdr-monitor/cw-dataset-receiver.conf` | resolved-config rewriting |
| Add or change Python dependencies | `processing/pyproject.toml` | `processing/uv.lock` |
| Add end-to-end behavior | `tests/test_cw_dataset.py` | `Makefile` `test` target |

The main orchestration entry point is `workflow()`. Read it from top to bottom
after understanding `Profile`, `generate_clean()`, `generate_jammer()`, and
`iqmix_command()`.

### Design invariants

Preserve these unless a research protocol explicitly replaces them and the
schema/docs/tests are updated together:

1. Clean GPS and interference are independent sources.
2. IQ is little-endian interleaved int16 `I,Q` at the declared complex rate.
3. Time values must map to exact integer sample indices.
4. Input files are exact-size; short and oversized inputs fail.
5. J/S is based on measured complex plateau power, excluding ramps.
6. AWGN is proper-complex, deterministic, and seeded.
7. One noise realization and one common gain are used across a seed's sweep.
8. Highest-level dry-run gain leaves at least 1 dB component headroom.
9. Clipping is a hard failure, not a warning.
10. Generated artifacts carry source, configuration, version, and hash evidence.
11. Receiver conclusions require the clean-prefix PVT gate.
12. The workflow remains offline-only.

### Change recipe: modify a jammer source

1. Change source math and validation in `tools/jammer_source.c`; keep
   `jammergen` and `jammertx` as backends over that shared state.
2. Keep bounded streaming; do not allocate the full waveform.
3. Define phase, frequency sign, amplitude, onset/offset, and envelope semantics
   unambiguously.
4. Update the direct-tool JSON manifest. Bump its schema if existing field
   meaning changes.
5. Update `generate_jammer()` if orchestration needs a new option.
6. Add a deterministic positive test and a malformed/unsupported-input test.
7. Add analyzer logic that can distinguish the intended waveform from common
   corruptions such as conjugation, wrong timing, or wrong RMS.
8. Update this guide and command help.

To add another source family such as stepped/hopped or IQ replay, do not
silently reinterpret an existing type. Give the source explicit parameters,
extend both CLIs and the Python CLI, add source-specific acceptance checks and
plots, and preserve existing waveform behavior.

### Change recipe: modify mixing or power definitions

1. Write the power equation and measurement interval first.
2. Change `tools/iqmix.c` measurement and mixing code.
3. Update both requested and measured manifest fields.
4. Confirm dry-run and real mixing consume the identical RNG sequence and math.
5. Confirm the clean control and every J/S level still share gain and seed.
6. Test measured J/S/SNR tolerance and zero clipping at extreme configured
   levels.
7. Update the equations and interpretation in this guide.

Changing from full-band SNR to a spectral-density metric is a protocol change,
not a variable rename. Use a new field/schema and document its units.

### Change recipe: add or modify a profile

1. Add the profile to `PROFILES` in `processing/cw_dataset.py`.
2. Ensure duration, onset, offset, and ramp are exactly sample-aligned.
3. Ensure two ramps fit inside the active interval.
4. Decide the default J/S grid and expected storage.
5. Add an end-to-end test with a duration small enough for CI.
6. Document whether it is a development profile or research protocol.

Do not make the canonical profile smaller to speed tests. Add a separate test
profile or use `fast` so published canonical semantics stay stable.

### Change recipe: modify receiver validation

1. Update `sdr-monitor/cw-dataset-receiver.conf` only for generic behavior.
   Scenario-specific PRNs belong in resolved generated configs.
2. Verify GNSS-SDR accepts the configuration and actually connects the
   flowgraph.
3. Preserve raw logs and resolved configuration in the fixture directory.
4. Update MAT/NMEA parsing defensively; missing or unreadable dumps must not be
   mistaken for successful evidence.
5. Keep the clean-prefix PVT gate before interpreting impairment.
6. Define the interval and aggregation for every new metric.
7. Update `dataset-index.json` threshold logic and document the rule.
8. Test both a valid receiver report and failure/no-evidence behavior.

For a C/N0 degradation threshold, compare the same stable PRN population or
define a documented population rule. Do not compare an impaired interval to a
different set of opportunistically acquired satellites without recording that
selection effect.

### Tests and verification

Build and run the full repository test target:

```sh
make test
```

The jammer tests currently cover:

- all five implemented source types and deterministic jammer bytes;
- chunk-boundary invariance and stochastic seed divergence;
- narrowband correlation, chirp frequency law, and pulse timing;
- exact file size and exact zeros outside the active envelope;
- rejection of unsupported source types;
- the public fast-profile workflow seam;
- common gain, common seed, and zero clipping across fixtures;
- bit-identical fixture hashes across repeated runs;
- generation of all four waveform plots;
- GNSS-SDR HDF5 prompt-dump parsing and per-PRN spectrum/spectrogram output;
- rejection of a mirrored/conjugated jammer.

Run Python style and type checks without adding tools to the project runtime
dependencies:

```sh
uvx ruff check processing/cw_dataset.py \
  processing/jamming_campaign_report.py tests/test_cw_dataset.py
uvx ruff format --check processing/cw_dataset.py \
  processing/jamming_campaign_report.py tests/test_cw_dataset.py
(cd processing && uv run --with pyright pyright \
  cw_dataset.py jamming_campaign_report.py ../tests/test_cw_dataset.py)
```

On Clang-based systems, optionally run static analysis for both C tools:

```sh
cc --analyze -Wall -Wextra -Wpedantic \
  tools/jammergen.c -o /tmp/jammergen.plist
cc --analyze -Wall -Wextra -Wpedantic \
  tools/iqmix.c -o /tmp/iqmix.plist
```

Finish with repository hygiene checks:

```sh
git diff --check
git status --short
```

Do not commit generated IQ, receiver dumps, local RINEX files, or experiment
output directories. Do commit source, tests, small configuration, schemas,
documentation, and the uv lockfile when dependencies change.

### Pull request checklist

- [ ] The change states which research definition or failure mode it addresses.
- [ ] Production code and tests change together.
- [ ] Determinism is preserved or the new stochastic contract is explicit.
- [ ] Sample format, units, interval boundaries, and frequency sign are explicit.
- [ ] Manifest fields and schema versions match the code.
- [ ] No fixture-specific normalization invalidates comparisons.
- [ ] Waveform analysis fails on a representative corruption.
- [ ] Receiver claims are backed by PVT evidence, not acquisition alone.
- [ ] Fast tests pass with `make test`.
- [ ] At least one relevant canonical or receiver validation is reported when
      protocol behavior changes.
- [ ] Documentation and CLI help are updated.
- [ ] Generated binary data and unrelated worktree files are not included.

## Troubleshooting

### `insufficient free space`

The preflight estimates clean, jammer, and fixture IQ plus 10%. Free more disk,
reduce the J/S grid with `--js-levels`, use `fast`, or move `--output-dir` to a
larger volume. Remember that GNSS-SDR dumps require additional space.

### `clean input is short` or `has ... bytes; expected exactly ...`

The selected profile and input do not agree. Canonical needs at least
936,000,000 bytes and `iqmix` requires exactly that size. The Python workflow
can trim an oversized `--clean-input`; the direct C mixer cannot.

### `timing must be sample-aligned`

Every time multiplied by the sample rate must be an integer sample count. Check
duration, onset, offset, and ramp. Also ensure `onset < offset <= duration` and
that both ramps fit inside the active interval.

### Tone appears at -500 kHz

The Q component was probably conjugated or the consumer uses a different IQ
convention. This pipeline's contract is interleaved `I,Q`, and positive complex
frequency uses `I=cos`, `Q=sin`. The image-rejection acceptance check should
fail for a mirrored file.

### Measured J/S differs from the nominal source amplitude

This is expected unless the sources happen to have equal measured power.
`jammergen --amplitude` controls jammer full-scale RMS; `iqmix --js-db` scales
that waveform relative to measured clean plateau power. Use the mixer manifest's
`measured_js_db` for the actual fixture ratio.

### GNSS-SDR returns zero but no useful report exists

Inspect `receiver.log`. The workflow recognizes known flowgraph connection and
bad-configuration messages as logical failures. If a new GNSS-SDR failure text
is not recognized, add it in `run_receiver()` with a regression test.

### `clean-prefix gate failed`

The receiver did not produce a valid pre-onset GGA fix with at least four usable
satellites. Check the RINEX/time pair, visible PRNs, acquisition priority,
receiver version, resolved configuration, and clean control before changing
jammer settings. Never classify this as jammer degradation.

### Calibration has no parseable C/N0

Check that tracking MAT dumps are enabled and readable, PRNs were acquired, and
GNSS-SDR ran long enough. The parser supports ordinary SciPy MAT files and HDF5
MAT files. Review `calibration/iteration-N/receiver/receiver.log` and
`receiver-report.json`.

### Results are not bit-identical

Compare, in order:

1. clean-source SHA-256;
2. jammer SHA-256;
3. seed and seed count;
4. requested levels and common gain;
5. tool commit and compiler/platform;
6. calibrated noise RMS and receiver version.

The generated fixture bytes should repeat when all waveform inputs and tool
behavior are identical. Receiver acquisition timing can still vary, so retain
the raw receiver evidence and use multiple seeds/runs for threshold claims.

## Command summary

```sh
# Build and test
make gps-sdr-sim jammergen iqmix
uv sync --project processing --locked
make test

# Fast development fixture
uv run --project processing python processing/cw_dataset.py create \
  --profile fast --clean-input /path/to/clean.bin \
  --receiver none --output-dir /tmp/cw-fast

# Canonical waveform-only sweep
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical --rinex /path/to/nav.rnx \
  --start-time 2022/01/01,00:00:00 \
  --receiver none --no-calibrate-noise \
  --output-dir cw-dataset-output/coarse-waveform

# Canonical GNSS-SDR sweep
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical --rinex /path/to/nav.rnx \
  --start-time 2022/01/01,00:00:00 \
  --receiver all --gnss-sdr /path/to/gnss-sdr \
  --output-dir cw-dataset-output/coarse-receiver

# One-condition five-type GNSS-SDR verification
for TYPE in cw narrowband wideband chirp pulsed; do
  uv run --project processing python processing/cw_dataset.py create \
    --profile verification --rinex /path/to/nav.rnx \
    --start-time 2025/01/03,00:00:00 --jammer-type "$TYPE" \
    --receiver all --gnss-sdr /path/to/gnss-sdr \
    --output-dir "analysis-output/runs/$TYPE" || exit $?
done
uv run --project processing python processing/jamming_campaign_report.py \
  --runs-root analysis-output/runs --output-dir analysis-output

# Five-seed 2 dB refinement around +10 dB
uv run --project processing python processing/cw_dataset.py create \
  --profile canonical --rinex /path/to/nav.rnx \
  --start-time 2022/01/01,00:00:00 \
  --refine-around 10 --seed 20260712 --seed-count 5 \
  --receiver all --gnss-sdr /path/to/gnss-sdr \
  --output-dir cw-dataset-output/refined-10db
```
