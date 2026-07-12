# GNSS Jamming Mitigation Research Model and Tool Architecture

> **Status:** Independent CW, narrowband, wideband, chirp, and pulsed offline
> and UHD live vertical slices implemented; matched synthetic GNSS-SDR
> verification completed for all five at +20 dB J/S
>
> **Date:** 2026-07-12
>
> **Scope:** Reproducible offline and conducted/shielded GPS L1 C/A interference-mitigation research
>
> **Related:** [GNSS Interference Mitigation Scenario Catalog](jamming-spoofing-scenarios.md)

## Executive Decision

Keep `gps-sdr-sim` focused on generating clean GNSS signals. Build interference as independent, additive sources in `jammergen`, then either mix those sources with clean IQ for offline receiver testing or transmit jammer-only IQ through the USRP X300 in a controlled RF setup.

The intended tool boundary is:

```text
gps-sdr-sim  -> clean GNSS IQ ----\
jammergen    -> jammer-only IQ ----+-> iqmix -> receiver-front-end model -> test IQ
noise source -> thermal noise IQ --/

shared jammer source -> TX conditioning -> jammertx -> conducted or shielded DUT
```

This replaces the current per-satellite "attack method" model. A satellite is a desired-signal source; a jammer is a separate emitter. They should not be represented by mutually exclusive branches in the same channel.

## Why the Current `jam_noise` Model Is Not PRN-Selective

The current implementation appears in three render loops:

- [`gpssim.c`](../gpssim.c), in the `ATTACK_METHOD_JAM_NOISE` branch;
- [`player/x300tx.cpp`](../player/x300tx.cpp), in its per-sample channel renderer;
- [`player/bladetx.cpp`](../player/bladetx.cpp), in its per-sample channel renderer.

Each branch draws new independent pseudo-Gaussian I and Q values for every output sample, then multiplies them by the target channel's C/A-code sign and carrier lookup values. The normal satellite sample is replaced rather than retained and combined with an independent interferer.

For an idealized real noise sequence `n[k]` and target code `c_p[k]`:

```text
x[k] = n[k] c_p[k]
```

The target correlator gives:

```text
Z_p = sum(x[k] c_p[k]) = sum(n[k])
```

A non-target correlator gives:

```text
Z_q = sum(x[k] c_q[k]) = sum(n[k] c_p[k] c_q[k])
```

When `n[k]` is independent, symmetric white noise, multiplication by either deterministic `+1/-1` sequence preserves its distribution and autocorrelation. Both correlator outputs therefore have the same expected noise power. The C/A-code multiplication alone does not create code-period correlation, and it does not establish the documented target/non-target rejection.

The current I/Q construction also uses independent `ni*cos(phase)` and `nq*sin(phase)` terms rather than rotating a proper complex envelope. This can create time-varying component variance, but it does not restore PRN selectivity.

### Meaning of the Legacy Methods

| Legacy method | Actual behavior | Valid interpretation |
|---|---|---|
| `normal` | Renders the simulated satellite normally | Clean GNSS signal |
| `jam_drop` | Sets the selected simulated channel gain to zero | Oracle constellation-ablation baseline only |
| `jam_noise` | Replaces the selected simulated channel with per-sample noise-like IQ | Legacy broadband-noise-like branch; not demonstrated matched-code jamming |
| `spoof_delay` | Accepted but not applied | Placeholder |
| `spoof_nav` | Accepted but not applied | Placeholder |

`jam_drop` cannot describe an RF waveform that suppresses a real satellite. `-P` selects which simulated PRNs are rendered; it is not an RF isolation mechanism for real-sky PRNs. The legacy `-J` value scales each selected branch relative to its simulated channel gain and is not, by itself, a calibrated receiver-input J/S measurement.

## Research Signal Model

For offline evaluation, model the receiver input as:

```text
r[k] = sum(satellite_i[k]) + sum(jammer_m[k]) + thermal_noise[k]
```

Then apply the victim receiver model:

```text
r[k] -> receiver filter -> AGC or fixed gain -> clipping -> quantization -> IQ
```

For jammer-only hardware playback, generate only the jammer term:

```text
j[k] -> total-power normalization -> TX filter/limiter -> DAC samples
```

The real or controlled GNSS signal, propagation path, and jammer are combined physically before the device under test. Do not bake a simulated victim AGC into jammer-only transmit samples; the device under test supplies that behavior. Transmitter anti-clipping and interpolation filters are separate TX-chain concerns.

The five parameterized jammer types do not require an authentic desired signal
or captured interferer as generator input. A fully synthetic mitigation study
can use `gps-sdr-sim` plus `jammergen`. Authentic/captured input becomes required
when the claim depends on real-sky geometry and impairments, a field-observed
jammer, measured propagation/antenna coupling, analog front-end behavior that
is absent from the baseband model, or installed-receiver performance. Those
questions need a lawful capture, IQ replay, or calibrated physical combination;
the synthetic source alone cannot answer them.

## Component Responsibilities

| Component | Responsibility | Must not own |
|---|---|---|
| `gps-sdr-sim` | Clean GPS L1 C/A constellation, navigation data, pseudorange, Doppler, motion, and synthetic-satellite scenarios | Narrowband, chirp, pulsed, or broadband jammer families |
| Shared GNSS reference library | C/A-code generation plus optional code-phase and Doppler references for code-aware sources | Jammer policy or power allocation |
| `jammergen` | Independent jammer sources, envelopes, frequency laws, filtering, seeding, and total-power normalization | Victim receiver AGC or clean satellite removal |
| `iqmix` | Offline `signal + jammer + noise` composition and optional victim front-end model | RF calibration claims without measurement |
| `jammertx` / future hardware backends | Timed streaming and SDR-specific TX configuration | Desired-signal generation or victim receiver AGC |
| Analysis pipeline | PSD, power, correlator, receiver, PVT, per-PRN prompt, and repeatability measurements; cross-type descriptive reporting | Assumed success based only on generated samples |

`jammergen` and `jammertx` share `tools/jammer_source.c`, so the offline file backend and live UHD backend use the same CW/noise/chirp/pulse state, envelopes, seeds, quantization, and digital metrics. Future hardware backends must reuse the same source definitions and power normalization.

## Composable Jammer Model

Do not create one enum value for every waveform combination. A jammer source should compose independent properties:

| Property | Initial values |
|---|---|
| Base source | `tone`, `gaussian`, `random_chips`, `gnss_ca`, `iq_replay` |
| Frequency law | `fixed`, `linear`, `triangular`, `stepped`, `hopped` |
| Envelope | `continuous`, `pulsed`, `ramped` |
| Spectral shaping | `unfiltered`, `band_limited`, `gnss_matched` |
| Power allocation | Absolute digital RMS target or normalized weight |
| Reproducibility | Explicit seed and start-sample state |
| Spatial model, later | Per-emitter delay, gain, phase, and array steering vector |

The current CLI exposes five named vertical slices over that model:

| Type | Base source | Frequency law | Shaping/envelope |
| --- | --- | --- | --- |
| `cw` | tone | fixed | global ramp |
| `narrowband` | proper-complex Gaussian | fixed center | one-pole low-pass with requested two-sided ENBW |
| `wideband` | proper-complex Gaussian | full complex Nyquist band | unfiltered |
| `chirp` | tone | repeating linear start-to-end sweep | global ramp, phase continuous at repeats |
| `pulsed` | tone | fixed | periodic raised-cosine gate plus global ramp |

These names are stable command-line profiles, not a claim that every future
composition needs a new enum. Stepped/hopped laws, multitone composition,
random chips, IQ replay, and code-aware sources remain future components.

### Source Families and PRN Selectivity

| Source | Main research use | PRN-selective? |
|---|---|---|
| CW or multitone | Narrowband mitigation and notch-filter tests | No |
| Shaped narrowband or wideband Gaussian | Broadband and partial-band interference tests | No |
| Chirp, stepped, or hopped | Time-frequency mitigation tests | No |
| Pulsed variants | Blankers, transient detection, and recovery tests | No |
| Random chips at a GNSS-like chip rate | Matched-spectrum interference | No specific PRN |
| GPS C/A-code-aware source | Code-domain and target/non-target correlator tests | Conditional on code/Doppler alignment |
| IQ replay | Realistic regression fixture | Only if the recording contains aligned structure |

Tone, Gaussian, chirp, and random-chip sources occupy the common receiver band. They can affect channels differently because of Doppler, filtering, or receiver implementation, but they cannot reliably select a PRN identity.

## Code-Aware, Targeted-PRN Source

A genuinely code-aware source can be represented as:

```text
j_p[k] = sqrt(P_p) a_p[k] c_p(k - tau_p)
         exp(j (2 pi f_p k / Fs + phi_p))
```

where:

- `c_p` is the selected GPS C/A code;
- `tau_p` is the jammer code phase at the device under test;
- `f_p` is the intended Doppler or carrier offset;
- `phi_p` is carrier phase;
- `a_p[k]` is a correlated or slowly varying complex envelope;
- `P_p` is the allocated power for this source.

The envelope must not be independent at every output sample. A simple research model can hold a random symbol or complex value over a code epoch, or generate band-limited noise whose correlation time survives coherent integration. If `a_p[k]` is white at the output sample rate, the source collapses back to non-selective white noise after code multiplication.

The source must expose code delay and Doppler explicitly. Without sufficient alignment, it appears at another point in code-Doppler space and the receiver may continue tracking the authentic signal. Targeting is therefore conditional rather than absolute.

Use one independent source per selected PRN, but allocate a fixed total jammer-power budget across the selected set. Navigation data should remain absent or intentionally randomized for a jamming experiment. A coherent, valid, self-consistent navigation message and controlled pseudorange trajectory belongs to spoofing research and should be modeled separately.

## Power Definition and Calibration

For the implemented single-source pipeline, CW/chirp amplitude is exact,
narrowband/wideband noise has unit expected complex power before amplitude,
and pulsed amplitude is the on-pulse value. `jammergen` records measured plateau
RMS, and `iqmix` uses measured average plateau power—not nominal amplitude—to
realize J/S. This keeps comparisons valid across bandwidth and duty cycle.

A future multi-source composite should normalize after source-specific
filtering and envelopes are applied. For source weights `w_m` and total
requested digital jammer power `P_J`:

```text
u_m[k] = source_m[k] / rms(source_m)

j[k] = sqrt(P_J) sum_m sqrt(w_m / sum(w)) u_m[k]
```

Verify the final composite RMS as well, because coherent or correlated sources can introduce cross terms. Adding more sources or target PRNs must not silently increase total jammer power.

Keep these quantities distinct:

- **Digital source power:** RMS or dBFS in the generated IQ stream.
- **Total receiver-input J/S:** measured jammer power relative to desired-signal power at the device under test.
- **In-band jammer power:** power remaining after the receiver-equivalent filter.
- **`I0/N0` or effective `C/N0`:** density-based measures for comparing bandwidth-dependent waveforms.
- **AGC, clipping, and quantizer occupancy:** evidence that an experiment has or has not entered front-end overload.

Digital scaling alone cannot establish physical J/S. Hardware tests require a calibrated conducted or shielded path and receiver-input measurement or a traceable link budget checked by measurement.

## Sample-Rate and Spatial Boundaries

The current default stream is 2.6 Msps complex baseband. It is appropriate for narrow GPS L1 C/A studies but cannot represent every threat:

- wider sweeps and adjacent-band overload require a higher-rate fixture and an explicit front-end model;
- L5/E5 and multiband experiments require additional center frequencies, signal models, and synchronized streams;
- analog front-end compression caused by energy outside the recorded baseband cannot be inferred from a narrow IQ file;
- CRPA and direction-of-arrival algorithms require coherent per-antenna outputs with an emitter-specific steering vector, delay, gain, and phase.

Do not generalize results from the 2.6 Msps L1 path to those cases.

## Validation and Acceptance Tests

### Waveform-Level Tests

Every source type must have automated checks for:

- requested versus measured RMS and in-band power;
- constant total power as source or target count changes;
- PSD or occupied-bandwidth error after filtering;
- instantaneous-frequency law for chirp, stepped, and hopped sources;
- pulse duty cycle, ramp timing, and envelope boundaries;
- deterministic repeatability for the same seed and divergence for different seeds;
- clipping rate and output-format quantization;
- file/stream duration and sample count.

### Correlator-Level Tests

Compute the response over a PRN, code-delay, and Doppler grid rather than checking a single correlator point.

- Per-sample white Gaussian noise multiplied by a C/A code should show no statistically meaningful target/non-target advantage.
- A `gnss_ca` source should show the designed ambiguity peak and measured cross-PRN response.
- Any claimed rejection value must state the integration interval, code/Doppler offsets, statistic, and worst-case or percentile definition.

This test would immediately expose the legacy `jam_noise` selectivity error.

### Receiver-Level Tests

For each mitigation experiment, progress through:

1. clean baseline;
2. jammer present below detection or degradation;
3. first measurable tracking degradation;
4. loss of selected measurements or service denial;
5. front-end AGC movement, clipping, or saturation;
6. recovery after the jammer stops.

Record receiver-side evidence, including per-signal `C/N0`, acquisition/tracking state, `svUsed`, raw pseudorange and Doppler, PVT validity, residuals, AGC/noise/jamming indicators, and recovery time. Generated waveform checks alone are not evidence that a mitigation method worked.

Raw GPS L1 C/A spectra are composite because every PRN shares the band. A
figure labeled by PRN must come from a code-separated domain such as a
GNSS-SDR post-correlation prompt dump, and it must be labeled as despread
tracking-channel evidence rather than a PRN-isolated RF FFT. The implemented
verification workflow produces both domains and keeps their interpretations
separate.

## Dataset Manifest

Every generated fixture should carry a machine-readable manifest containing at least:

- sample rate, center-frequency reference, sample format, and sample count;
- source type and all composable properties;
- seed and initial state;
- requested and measured per-source and total RMS power;
- filter definition and measured in-band power;
- GNSS start time, receiver reference location, PRN, code phase, and Doppler when applicable;
- onset/offset samples and envelope timing;
- tool version or Git commit;
- offline front-end model or hardware calibration identifier;
- receiver firmware/configuration and captured observables for result datasets.

## Migration Plan

1. **Correct the documentation.** Mark `jam_noise` as a legacy, non-selective implementation and `jam_drop` as simulation-only.
2. **Freeze a legacy characterization test.** Demonstrate the current target/non-target correlator response before removing or renaming behavior.
3. **Extract shared primitives.** Jammer timing, sample format, oscillator,
   one-pole filter, RNG, chirp, pulse envelope, quantization, and metrics now
   live in `tools/jammer_source.c`. Shared C/A-code references remain future
   work for code-aware sources.
4. **Create `jammergen`.** CW, narrowband Gaussian, wideband Gaussian, linear
   chirp, and pulsed CW are implemented with deterministic state and
   sample-accurate envelopes. Stepped/hopped, multitone, replay, and code-aware
   sources remain future work.
5. **Create `iqmix`.** Measured J/S, deterministic thermal noise, common anti-clipping gain, and int16 quantization are implemented. Victim analog-front-end models remain future work.
6. **Make transmitters source-agnostic.** UHD transmission for all five current
   types is implemented in `jammertx` through the shared jammer source. A
   bladeRF backend and later waveform components remain future work.
7. **Verify the synthetic receiver path.** The bounded `verification` profile
   now exercises all five modes through GNSS-SDR with matched clean controls,
   numerical waveform gates, per-PRN prompt figures, PVT evidence, and a
   strict single-run descriptive report. Repeated seeds/J/S levels and
   authentic captures remain required for broader claims.
8. **Add code-aware interference.** Implement `gnss_ca` only after the correlator acceptance harness exists.
9. **Deprecate legacy attack flags.** Retain them temporarily for reproduction, then remove them after equivalent research workflows are available.

## Safety Boundary

All interference experiments must use offline processing, a conducted RF chain with suitable isolation and protection, or a verified shielded enclosure. Do not radiate GNSS-band interference into the open environment. Hardware protection, DC blocking where required, attenuation, leakage checks, and an emergency stop belong in the test plan rather than being assumed by waveform code.

## References

- Matthew J. Murrian et al., [GNSS Interference Monitoring from Low Earth Orbit](https://rnl-data.ae.utexas.edu/datastore/papers/grid_papers/leo-interference_murrian.pdf), for observed distinctions between matched-spectrum, matched-code, and spoofing-like signals.
- ESA Navipedia, [GNSS Interference Model](https://gssc.esa.int/navipedia/index.php/GNSS_Interference_Model), for receiver bandwidth, spectral separation, and effective `C/N0` framing.
