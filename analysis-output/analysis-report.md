# GNSS-SDR Multi-Waveform Jamming Verification

## Outcome

All five synthetic jammer modes passed their numerical waveform gates and completed matched clean/jammed processing with GNSS-SDR 0.0.20. At measured J/S = +20 dB, narrowband, wideband, and chirp interference eliminated valid PVT throughout the 20-second jammer interval; CW and pulsed interference retained 19/20 valid fixes but reduced the median usable-satellite count.

This is a single deterministic run per condition. It verifies this implementation and this synthetic receiver scenario; it does not establish population-level or field performance.

## Locked experiment

- Desired signal: one frozen `gps-sdr-sim` GPS L1 C/A recording, SHA-256 `a4d690ef9d7d0e171b3d8ef984e79dc7426e200d21c7224a645473113b4066ea`.
- Navigation/time/location: frozen 2025-01-03 RINEX broadcast navigation, `2025/01/03,00:00:00`, Hanoi `21.0047844,105.8460541,5`.
- IQ: signed little-endian interleaved SC16, 2.6 Msps, 90 s.
- Timing: clean prefix 0-40 s, jammer 40-60 s, recovery 60-90 s.
- Receiver: `gnss-sdr version 0.0.20`, 9 fixed visible PRNs, all assigned channels acquiring concurrently.
- Noise/J/S: deterministic seed 20260712; every condition measured +20.00 dB J/S and approximately -10.00 dB composite SNR with zero clipped components.
- Primary PRN set: G10, G12, G23, G28, G31, G32, the PRNs with clean-prefix median C/N0 at least 30 dB-Hz in every clean and jammed receiver pass.

## Waveform evidence

| Type | Gate | Key measured property |
|---|---:|---|
| CW | PASS | tone 500.000 kHz; image rejection 111.8 dB |
| Narrowband | PASS | center 499.747 kHz; 90% occupied width 390.0 kHz |
| Wideband | PASS | 90% occupied width 2.338 MHz |
| Chirp | PASS | frequency-law RMS error 10.3 Hz; max error 34.0 Hz |
| Pulsed | PASS | nonzero duty 9.996%; off-state nonzero samples 0 |

The raw waveform comparison is in [Figure 1](figures/figure-01-waveform-spectrum-spectrogram.png). It shows the expected fixed line, band-limited noise cloud, Nyquist-wide noise floor, repeating sweep, and pulsed comb/time gating. These are jammer-only RF/baseband-domain plots after applying each mix scale and are not PRN-isolated. Each run also retains a full-90-second mixed spectrogram for activation timing.

## Receiver impact

| Type | J/S (dB) | Median paired C/N0 change (dB) | Jammer PVT fixes | Median usable sats | Restored PVT after off (s) |
|---|---:|---:|---:|---:|---:|
| CW | 20.00 | -2.46 | 19/20 | 4.0 | 0.0 |
| Narrowband | 20.00 | -13.07 | 0/20 | 0.0 | 7.0 |
| Wideband | 20.00 | -10.94 | 0/20 | 0.0 | 7.0 |
| Chirp | 20.00 | -14.24 | 0/20 | 0.0 | not observed |
| Pulsed | 20.00 | -8.98 | 19/20 | 5.0 | 0.0 |

The clean control produced 19/20 valid PVT fixes and a median of 6 usable satellites in the matched interval for every type. [Figure 2](figures/figure-02-per-prn-cn0-delta.png) retains all nine PRN-level paired changes; asterisks identify the six primary stable PRNs. [Figure 3](figures/figure-03-receiver-impact.png) summarizes the primary-set C/N0 effect and matched PVT availability.

CW was selective across PRNs: some channels improved or changed little while G23/G28/G32 fell, so its primary-set median reduction was modest compared with the noise, chirp, and pulsed modes. Narrowband, wideband, and chirp prevented PVT during exposure. Narrowband and wideband restored PVT seven seconds after jammer-off; chirp did not restore PVT before NMEA output ended. Pulsed interference preserved PVT but caused a clear common-PRN C/N0 loss.

## What “spectrum of each PRN” means

All GPS L1 C/A PRNs occupy the same RF band, so a raw FFT cannot separate or label them. The 90 jammed receiver artifacts therefore include one PNG and one PDF per PRN based on GNSS-SDR post-correlation `Prompt_I`/`Prompt_Q`. Those plots are code/carrier-despread residual prompt spectra and spectrograms, not raw RF spectra assigned to a PRN. See the [PRN gallery](prn-gallery.md) and [figure catalog](figure-catalog.md).

## Authentic-signal boundary

No authentic captured signal is required to generate or numerically verify CW, narrowband, wideband, chirp, or pulsed mathematical waveforms, nor to reproduce this synthetic GNSS-SDR experiment. Authentic desired-signal and interference captures are required before claiming behavior under real antenna patterns, propagation/multipath, oscillator error, front-end filtering or AGC, ADC saturation/quantization, field interferers, or installed receiver hardware. IQ replay for captured authentic interference is not implemented in this campaign.

## Limitations

- One deterministic seed and one +20 dB J/S point per waveform: no confidence interval, significance test, or standardized effect size is identifiable.
- PRNs are not independent experimental repetitions; they share one composite IQ stream, navigation geometry, noise realization, and receiver process.
- GNSS-SDR NMEA ends near scenario second 72 although tracking dumps continue through 90 s. Recovery PVT counts therefore cover only the emitted NMEA window; full-duration recovery C/N0 remains available from tracking.
- Results are receiver/configuration-specific and do not validate a mitigation algorithm; they validate waveform generation, mixing, receiver ingestion, and observable degradation.
