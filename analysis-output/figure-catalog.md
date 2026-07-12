# Figure Catalog

## Figure 1 — waveform spectrum and spectrogram

- Files: [PNG](figures/figure-01-waveform-spectrum-spectrogram.png), [PDF](figures/figure-01-waveform-spectrum-spectrogram.pdf)
- Purpose: verify that the five generated/mixed RF-domain waveforms have distinguishable, type-correct spectral and temporal structure.
- Data: jammer-only SC16 after applying each mix manifest's effective scale; the spectrum uses the steady interval and the dense spectrogram uses the first 250 ms after the global ramp. Each run retains a separate full-90-second mixed spectrogram for activation timing.
- Reader should notice: CW is a fixed +500 kHz line; narrowband remains concentrated around +500 kHz; wideband fills the complex Nyquist band; chirp repeatedly sweeps -500 to +500 kHz; pulsed CW is intermittent and produces a comb-like spectrum.
- Implication: the receiver comparison is not five aliases of one CW generator; each requested source reaches the actual mixed-IQ path.
- Caveat: FFT values are descriptive windowed estimates, not calibrated RF power-density measurements.

## Figure 2 — per-PRN C/N0 change

- Files: [PNG](figures/figure-02-per-prn-cn0-delta.png), [PDF](figures/figure-02-per-prn-cn0-delta.pdf)
- Purpose: show which GNSS-SDR tracking channels are affected during the matched 20-second jammer interval.
- Data: per-PRN median C/N0 from each jammed receiver report minus its same-type clean control at the same time interval.
- Reader should notice: narrowband, wideband, chirp, and pulsed effects are broadly negative across stable channels; CW is more PRN-dependent.
- Implication: mitigation evaluation should retain channel-level metrics rather than relying only on one aggregate PVT status.
- Caveat: starred PRNs form the primary stable set; unstarred late/weak channels are descriptive only.

## Figure 3 — receiver impact

- Files: [PNG](figures/figure-03-receiver-impact.png), [PDF](figures/figure-03-receiver-impact.pdf)
- Purpose: connect post-correlation degradation to navigation availability.
- Data: median matched C/N0 change across the six common stable PRNs and exact valid 1 Hz PVT fixes during seconds 40-60.
- Reader should notice: narrowband, wideband, and chirp drive PVT to zero; CW and pulsed retain fixes but reduce usable-satellite margin.
- Implication: the five types exercise materially different receiver failure regimes at the same measured average J/S.
- Caveat: one deterministic run per condition; bars have no error bars because no repeated-run uncertainty exists.

## Per-PRN spectrum/spectrogram artifacts

These are GNSS-SDR post-correlation prompt-domain figures. Each PNG has a matching PDF and traceable `prn-spectral-report.json`.

| Type | PRNs | Files | Directory |
|---|---:|---:|---|
| CW | 9 | 18 | [prn-spectral](runs/cw/js-p20db/receiver/prn-spectral) |
| Narrowband | 9 | 18 | [prn-spectral](runs/narrowband/js-p20db/receiver/prn-spectral) |
| Wideband | 9 | 18 | [prn-spectral](runs/wideband/js-p20db/receiver/prn-spectral) |
| Chirp | 9 | 18 | [prn-spectral](runs/chirp/js-p20db/receiver/prn-spectral) |
| Pulsed | 9 | 18 | [prn-spectral](runs/pulsed/js-p20db/receiver/prn-spectral) |

For every PRN figure:

- Purpose: compare clean-prefix, jammer, and recovery prompt spectra and show prompt residual frequency over time.
- Interpretation: changes represent that PRN's despread tracking channel; they are not PRN-isolated raw RF FFTs.
- Missing-data rule: prompt records are averaged onto a 50 Hz grid; STFT windows below 50% tracking coverage are blank.
