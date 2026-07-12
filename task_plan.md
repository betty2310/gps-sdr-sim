# Task Plan: GNSS-SDR jammer verification campaign

## Goal
Generate clean and five jammer-condition GPS L1 C/A IQ fixtures, process them with GNSS-SDR, and produce reproducible per-PRN spectrum/spectrogram and receiver-impact evidence.

## Phases
- [x] Phase 1: Inventory the existing dataset workflow, GNSS-SDR installation/configs, and available signal artifacts
- [x] Phase 2: Define a bounded run matrix, PRN-level plot semantics, and acceptance criteria
- [x] Phase 3: Generate clean/CW/narrowband/wideband/chirp/pulsed fixtures and run GNSS-SDR
- [x] Phase 4: Generate and inspect scientific spectrum/spectrogram figures plus receiver metrics
- [x] Phase 5: Run code-quality verification and deliver the analysis bundle

## Key Questions
1. Is GNSS-SDR installed and is the repo workflow already wired to its current output format?
2. Which PRNs are actually present and tracked in the clean fixture?
3. Can “spectrum of each PRN” be measured directly from raw composite IQ, or must it be derived from GNSS-SDR channel/correlator dumps?
4. How do the five jammer modes differ spectrally and in per-PRN C/N0, tracking, and PVT outcomes at a controlled J/S?
5. Which conclusions require authentic captured signals rather than synthetic GPS and jammer IQ?

## Decisions Made
- Keep the entire campaign offline; do not invoke the UHD transmitter.
- Use one deterministic, identical clean source and timing profile for all jammer conditions.
- Treat one run per condition as descriptive verification only; do not claim statistical significance.
- Preserve raw receiver artifacts and plot metadata so every image is traceable to its IQ/config inputs.
- Use the 90 s `verification` profile: 40 s clean prefix, 20 s jammer interval, and 30 s recovery. The longer prefix avoids the pilot's borderline first-PVT timing at 30 s; the 60 s offset leaves about 12 seconds of observable post-jammer PVT before this GNSS-SDR run stops emitting NMEA near 72 s.
- Compare all modes at measured J/S = +20 dB with one deterministic noise seed. Use a clean control at the same time interval for matched receiver comparisons.
- Plot raw composite IQ at RF/baseband level and per-PRN prompt I/Q only after GNSS-SDR code/carrier wipeoff; never label a raw FFT as PRN-isolated.

## Errors Encountered
- A manual GNSS-SDR pilot invocation passed a relative receiver output directory; because GNSS-SDR runs with that directory as its CWD, the relative `--config_file` path resolved twice and was not found. The normal workflow uses an absolute output root. Re-run the manual pilot with resolved absolute paths.
- A combined source patch included duplicate-key cleanup contexts that were no longer present and did not apply. No file changed from that failed patch; split the receiver/threshold/CLI edits into exact smaller patches.
- The first synthetic per-PRN plot test failed because SciPy reports a plain HDF5 fixture as `ValueError`, while the loader only attempted HDF5 fallback for `NotImplementedError`. Expanded the same safe HDF5 fallback to all SciPy MAT read failures and reran the test.
- The first real pilot plot divided prompt I/Q by the 1 ms/20 ms logging interval and created a false roughly 20 dB step after bit synchronization. The GNSS-SDR dump's prompt magnitude is already comparable across those record intervals. Removed the division; the integration-aware step is now temporal binning only.
- The first 40/70 s verification run showed that GNSS-SDR tracking dumps continue to 90 s but NMEA output ends near scenario second 72. Moved the verification offset to 60 s so recovery PVT has a non-trivial observable window; matched C/N0 tracking remains available for the full run.
- A combined profile/notes patch assumed Ruff's pre-format line wrapping and did not apply. No file changed from that failed patch; reapplied against the current one-line profile definition.
- The first wideband receiver pass failed the clean-prefix PVT gate even though SHA-256 over the first 40 s was identical to its passing clean control. The workflow serialized 9 fixed PRNs through only 4 concurrent acquisition slots, making telemetry/PVT timing schedule-sensitive. Changed fixed-constellation runs to acquire all assigned PRNs concurrently and re-test the same IQ before restarting the campaign.
- Visual QA found that the 240-column full-90-second spectrogram aliases the 100 ms chirp and 10 ms pulse train into sparse points. Added a dense jammer-only 250 ms spectrogram for waveform-shape evidence while retaining the full-run mixed spectrogram for timing context.
- The first combined dense-spectrogram patch assumed pre-format campaign-script line wrapping and did not apply. No file changed from that failed patch; split it into exact workflow, campaign, label, and documentation edits.
- Dense spectrogram PNG rendering completed, but vectorizing roughly 12.5 million quadrilateral cells made the aggregate PDF writer exceed two minutes. Interrupted that renderer and marked only the spectrogram meshes as rasterized inside the otherwise-vector PDF.
- The first untracked-file whitespace loop used zsh's special `path` array as its loop variable, which cleared command lookup and made `git` unavailable inside the loop. No files changed; renamed the variable and reran the check.
- The first comprehensive verification passed build, tests, and Ruff but Pyright found eight static typing errors. The build-fixer pass resolved all eight without runtime changes: three SciPy-stub `detrend=False` mismatches, one optional-dict narrowing issue, one Matplotlib `Figure` import, and one complex test-array annotation.

## Status
**Complete** - Five matched GNSS-SDR conditions, 45 jammed PRN PNG/PDF pairs, raw waveform figures, receiver metrics, strict descriptive reports, documentation, and all verification gates are complete.
