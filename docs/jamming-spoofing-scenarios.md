# GNSS Interference Mitigation Scenario Catalog

> **Status:** Research catalog; legacy implementation limitations are documented below
>
> **Safety boundary:** Offline, conducted, or verified shielded testing only
>
> **Architecture:** [GNSS Jamming Mitigation Research Model and Tool Architecture](jamming-mitigation-architecture.md)

## Purpose

This catalog defines repeatable scenarios for evaluating GNSS interference detection and mitigation. It replaces the earlier assumption that the existing `jam_noise` branch is selective matched-code jamming and removes open-air transmission recipes that were based on that assumption.

The research model separates:

1. clean GNSS signal generation;
2. independent jammer generation;
3. offline signal/noise/interference mixing;
4. optional receiver-front-end modeling;
5. controlled SDR playback;
6. receiver-side measurement and analysis.

Scenario success must be demonstrated at the waveform, correlator, and receiver levels. A generated IQ file or successful SDR transmission is not sufficient evidence.

The filename is retained for existing links, but the current campaign scope is jamming and interference mitigation. Synthetic-satellite generation through `-S` remains useful as controlled GNSS signal generation; it is not a jammer source. `spoof_delay`, `spoof_nav`, and hybrid jam/spoof campaigns remain out of scope until they have independent implementations and validation plans.

## Current Implementation: What Is and Is Not Valid

The current `-A`, `-J`, `-P`, and `-G` controls remain available for reproducing legacy experiments, but they are not the target architecture.

| Interface | Current behavior | Research interpretation |
|---|---|---|
| `-P <prns>` | Renders only the selected simulated PRNs | Partial-constellation signal generation; not selective suppression of real PRNs |
| `-A PRN:normal` | Renders the simulated satellite normally | Clean or partial-signal fixture |
| `-A PRN:jam_drop` | Sets that simulated channel's gain to zero | Oracle ablation baseline; not an RF jammer waveform |
| `-A PRN:jam_noise` | Replaces the channel with fresh per-sample noise multiplied by its C/A-code sign and carrier lookup values | Legacy broadband-noise-like fixture; not validated matched-code or PRN-selective interference |
| `-J <dB>` | Applies a per-branch amplitude scale relative to simulated channel gain | Legacy digital scaling; not calibrated total receiver-input J/S |
| `-G <dB>` | Scales selected rendered channels | Signal-generation control; not a physical attenuation of real-sky satellites |
| `spoof_delay`, `spoof_nav` | Parsed but not applied | Placeholders only |

The legacy noise branch is duplicated in `gpssim.c`, `player/x300tx.cpp`, and `player/bladetx.cpp`. Selecting additional `jam_noise` PRNs adds additional independent noise-like branches and can increase total power. Results from that path must not be labeled PRN-selective unless an independent correlator test establishes selectivity.

## Implemented Independent-Source Toolchain

```text
gps-sdr-sim -> clean.bin --------\
jammergen   -> jammer.bin --------+-> iqmix -> test.bin -> software receiver
noise model -> thermal noise -----/

shared jammer source -> jammertx -> conducted or shielded receiver test
```

The implemented command-line interfaces are documented in the
[offline multi-waveform dataset guide](cw-jamming-dataset.md) and
[real-time X300 multi-waveform guide](realtime-cw-jammer-x300.md). This catalog remains at
the scenario-definition level so waveform and RF operating details have one
authoritative home.

## Experimental Rules

Apply these rules to every scenario:

- Keep total received jammer power constant when comparing waveform efficiency.
- Keep total jammer power constant when the number of sources or selected PRNs changes.
- Normalize each source after filtering and verify the final composite RMS.
- Record both requested digital power and measured in-band power.
- Use an explicit seed and preserve it in the dataset manifest.
- Use the same clean GNSS fixture when comparing jammer types offline.
- Separate tracking degradation from front-end overload; record AGC, clipping, and noise indicators.
- Sweep from below detection through degradation, denial, saturation, and recovery.
- Define success using receiver-side and algorithm-side observables, not expected behavior written in advance.
- Do not generalize a 2.6 Msps L1 C/A result to wider, adjacent-band, multiband, or spatial-array threats.

## Scenario Matrix

### A. Baselines and Legacy Characterization

| ID | Scenario | Purpose | Valid result |
|---|---|---|---|
| A0 | Clean GNSS | Establish acquisition, tracking, PVT, and measurement baseline | Receiver and waveform reference |
| A1 | Clean GNSS plus thermal noise | Validate noise calibration and effective `C/N0` | Expected receiver degradation versus noise density |
| A2 | Partial constellation using `-P` | Exercise receiver geometry with selected simulated signals | Offline or controlled signal-source scenario |
| A3 | `jam_drop` ablation | Measure algorithm behavior when measurements are removed by an oracle | Simulation-only benchmark |
| A4 | Legacy `jam_noise` PSD and RMS | Characterize what the current branch actually emits | Legacy regression fixture |
| A5 | Legacy target/non-target correlator matrix | Test the prior selectivity claim directly | Expected to show no established PRN advantage |
| A6 | Legacy selected-PRN count sweep | Measure accidental total-power scaling | Power-accounting regression |

A3 must not be described as a real jamming waveform. A4-A6 exist to document and retire incorrect assumptions, not to benchmark mitigation performance against a claimed matched-code jammer.

### B. Independent Jammer Sources

These scenarios require the planned `jammergen` tool or an equivalent externally generated, verified IQ fixture.

| ID | Source | Configurable properties | Primary mitigation question | PRN-selective? |
|---|---|---|---|---|
| B1 | CW tone | Offset, phase, power, onset | Can detection/notch processing handle a stationary interferer? | No |
| B2 | Multitone | Tone set, weights, phases | How does mitigation scale with multiple narrowband components? | No |
| B3 | Band-limited Gaussian | Center, bandwidth, filter, seed | How does performance vary with spectral overlap? | No |
| B4 | Linear chirp | Start/stop offsets, sweep time, direction | Can time-frequency mitigation follow a one-way sweep? | No |
| B5 | Triangular chirp | Bounds, up/down times | How does the receiver respond to recurring sweeps? | No |
| B6 | Stepped or hopped carrier | Frequency set, dwell, sequence, seed | Can detection follow discontinuous frequency motion? | No |
| B7 | Pulsed jammer | Source type, duty cycle, period, rise/fall | Do blanking and recovery logic outperform continuous mitigation? | No |
| B8 | Ramped jammer | Start/end power, duration, law | Where do detection and degradation thresholds occur? | No |
| B9 | Random chips | Chip rate, pulse shape, seed | How effective is GNSS-matched-spectrum interference? | No specific PRN |
| B10 | GPS C/A-code-aware | PRN, code phase, Doppler, envelope, seed | Can code-domain interference affect selected correlators at fixed total power? | Conditional |
| B11 | IQ replay | Fixture metadata, time/power scaling | Does mitigation generalize to captured interference? | Fixture-dependent |

### C. Power and Timing Sweeps

For each applicable B-series source, run the same stages:

| Stage | Description | Required evidence |
|---|---|---|
| C0 | Jammer absent | Clean baseline and calibration |
| C1 | Present below detection | No claimed effect; verify power and detector output |
| C2 | Detection onset | Detector statistic, alarm latency, false-alarm context |
| C3 | Tracking degradation | Per-signal `C/N0`, loop state, residuals, measurement quality |
| C4 | Service denial | Acquisition/tracking loss, `svUsed`, PVT validity |
| C5 | Front-end stress | AGC movement, clipping, quantizer occupancy, noise indicators |
| C6 | Recovery | Reacquisition time, PVT recovery, detector clear time |

Do not reuse absolute digital amplitudes across waveform families without post-filter RMS and in-band-power normalization.

### D. Targeted-PRN Research

Only a code-aware source belongs in this category. CW, chirp, Gaussian, and random-chip sources share the band and cannot reliably target PRN identity.

For each selected target PRN:

1. derive or specify its code phase and Doppler at the device under test;
2. generate a repeated C/A-code source with a correlated or epoch-wise envelope;
3. allocate power from a fixed total jammer budget;
4. evaluate a PRN/code-delay/Doppler correlation grid before receiver testing;
5. verify target-channel degradation and quantify non-target response;
6. repeat with deliberate code-delay and Doppler offsets;
7. monitor AGC to ensure apparent selectivity is not being masked by global front-end compression.

Required comparison cases:

| ID | Case | Research purpose |
|---|---|---|
| D0 | Target PRN, aligned code/Doppler | Maximum intended code-domain interaction |
| D1 | Target PRN, code-delay offset sweep | Correlator ambiguity and tracking sensitivity |
| D2 | Target PRN, Doppler offset sweep | Frequency-alignment sensitivity |
| D3 | Non-target PRN matrix | Cross-PRN leakage and worst case |
| D4 | One versus multiple target PRNs at fixed total power | Waveform-efficiency comparison without hidden power growth |
| D5 | Same targets with front-end overload deliberately excluded | Separate correlation effects from AGC effects |

No fixed rejection value should be claimed without specifying integration time, code/Doppler grid, receiver configuration, and whether the result is average, percentile, or worst case.

### E. Sample-Rate and Spatial Extensions

These scenarios are outside the default 2.6 Msps single-channel L1 path:

| ID | Scenario | Additional requirement |
|---|---|---|
| E1 | Wider L1 sweeps | Higher sample rate and verified anti-alias/front-end model |
| E2 | Adjacent-band overload | Wideband capture plus analog nonlinearity/compression model or hardware fixture |
| E3 | L5/E5 interference | Appropriate center frequency, bandwidth, and signal/reference model |
| E4 | Multiband attack | Synchronized RF paths and per-band calibration |
| E5 | CRPA/DOA mitigation | Coherent per-antenna streams and emitter-specific steering vectors |

## Offline Workflow

1. Generate one clean GNSS IQ fixture and manifest.
2. Generate a jammer-only fixture with an explicit seed and power target.
3. Run waveform-level acceptance tests before mixing.
4. Mix clean signal, jammer, and thermal noise at calibrated powers.
5. Optionally apply the receiver-equivalent filter, AGC/clipping, and quantization.
6. Process clean and impaired fixtures through the same receiver build and configuration.
7. Compare acquisition, tracking, measurements, PVT, detector statistics, and recovery.
8. Archive the exact manifests, logs, and tool commit with the results.

This workflow permits the same clean signal to be reused across jammer types and prevents satellite motion, navigation data, or random noise changes from confounding waveform comparisons.

## Conducted or Shielded Hardware Workflow

1. Verify the enclosure or conducted path prevents external GNSS-band radiation.
2. Protect the receiver and SDR with the required DC block, isolation, attenuation, and power limits.
3. Measure transmitter leakage with the jammer disabled and establish a clean receiver baseline.
4. Calibrate receiver-input jammer power; do not infer physical J/S from dBFS alone.
5. Begin below the detection threshold and increase in controlled steps.
6. Record raw receiver data, front-end status, transmitter settings, and calibration identifiers.
7. Include a fast, tested stop mechanism and verify recovery after each run.

Open-air jammer experiments and height/antenna recipes are intentionally outside this catalog.

## Measurement Checklist

For a u-blox receiver, useful observables include:

| Measurement | Example UBX message | Use |
|---|---|---|
| Per-satellite status and `C/N0` | `UBX-NAV-SAT` | Target/non-target tracking comparison and `svUsed` |
| Per-signal status | `UBX-NAV-SIG` | Frequency/signal-specific acquisition and tracking |
| Position solution | `UBX-NAV-PVT` | Fix validity, accuracy estimates, and recovery |
| Receiver status | `UBX-NAV-STATUS` | Integrity or spoofing indicators when applicable |
| RF front-end status | `UBX-MON-RF` | AGC, noise, and jamming indicators |
| Raw measurements | `UBX-RXM-RAWX` | Pseudorange, carrier phase, Doppler, and residual analysis |

Also preserve software-receiver acquisition maps, prompt/early/late correlator outputs, loop-discriminator values, and mitigation internal states when available.

## Acceptance Criteria

### Waveform Acceptance

- Sample count, duration, rate, and format match the manifest.
- Same seed produces bit-identical output when deterministic mode is requested.
- Measured total RMS and in-band power meet their tolerances.
- Adding sources or target PRNs does not change requested total power.
- PSD, frequency law, duty cycle, ramp, and onset/offset match configuration.
- Clipping and quantization rates are measured rather than assumed.

### Correlator Acceptance

- The tested PRN/code-delay/Doppler grid is recorded.
- Legacy per-sample code-multiplied white noise is not reported as selective without evidence.
- Code-aware fixtures exhibit the specified target ambiguity response.
- Cross-PRN leakage is reported as a distribution and worst case, not a blanket processing-gain claim.

### Receiver Acceptance

- Clean-baseline performance is reproduced before and after the run.
- Detection, degradation, denial, saturation, and recovery are distinguished.
- Targeted experiments show per-target effects and quantify non-target effects.
- Front-end AGC or clipping is reported whenever it can explain global degradation.
- Results include uncertainty or repeated trials where RNG, acquisition, or tracking is stochastic.

## Implementation Status and Next Work

| Capability | Status |
|---|---|
| Clean GPS L1 C/A generation | Implemented in `gps-sdr-sim` |
| Partial simulated constellation | Implemented via `-P` |
| Oracle satellite removal | Implemented as legacy `jam_drop` |
| Legacy per-sample noise replacement | Implemented as `jam_noise`; not PRN-selective as documented previously |
| Independent jammer-source engine | Implemented for offline CW |
| CW, band-limited Gaussian, chirp | CW implemented in `jammergen`; other families planned |
| Offline calibrated mixer and receiver front end | Measured mixing/noise/quantization implemented in `iqmix`; analog front end planned |
| Fixed-total-power multi-source normalization | Planned |
| Code-aware targeted-PRN source | Planned after correlator tests |
| Jammer-only hardware transmission | UHD/X300 CW implemented in `jammertx`; bladeRF and later source families planned |
| Wideband, multiband, and multi-antenna fixtures | Future scope |

The next work is controlled RF characterization of the UHD CW path, followed by
additional independent waveform families and a bladeRF backend. New sources
must reuse the shared jammer-source boundary rather than adding jammer branches
to the satellite render loop.
