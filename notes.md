# Notes: Multi-waveform jamming prototype

## GNSS-SDR Verification Campaign (2026-07-12)

- Local receiver: sibling build `../gnss-sdr/install/gnss-sdr`, version 0.0.20.
- Receiver config: `sdr-monitor/cw-dataset-receiver.conf`; SC16/ishort at 2.6 Msps, GPS L1 C/A acquisition, tracking MAT dumps, observables/PVT dumps, and 1 Hz NMEA are enabled.
- Existing workflow already produces raw composite spectrum/spectrogram, block power, per-PRN C/N0, tracked-count, and PVT plots.
- Raw IQ FFTs cannot isolate GPS L1 C/A PRNs because PRNs are code-domain signals sharing the same RF band. Correct per-PRN spectral evidence must be labeled as post-correlation/despread tracking-channel spectrum/spectrogram derived from GNSS-SDR prompt I/Q dumps.
- `gps-sdr-sim` was not built at inventory time; `jammergen` is present. The C/Python plotting stack is available.
- Available disk space is about 170 GiB, sufficient for a bounded 90 s campaign.
- Use a frozen public GPS navigation file for reproducibility: BKG IGS archive `brdc0030.25n.gz` for 2025 day 003 (2025-01-03), rather than a moving current-hour file.
- The first 90 s clean pilot tracked 9 visible PRNs: G02, G10, G12, G18, G21, G23, G28, G31, and G32. GNSS-SDR produced valid PVT before 30 s, but a repeated mixed-fixture receiver run reached its first PVT at the boundary, so the 30 s prefix gate is not robust enough for the campaign.
- The tracking MAT files expose `Prompt_I`, `Prompt_Q`, `PRN`, `PRN_start_sample_count`, `CN0_SNV_dB_Hz`, and loop metrics. Prompt logging uses 1 ms integration before bit sync and 20 ms integration afterward.
- Campaign matrix: one deterministic run per source type, measured J/S +20 dB, 90 s duration, onset 40 s, offset 60 s, with matched clean control and identical RINEX/location/time. GNSS-SDR NMEA ends near 72 s in the pilot even though tracking continues to 90 s, so recovery PVT is explicitly limited to the available NMEA window.
- Receiver determinism control: all 9 source-visible, fixed-assignment PRNs acquire concurrently. A 4-of-9 acquisition cap produced different first-PVT outcomes on byte-identical 40 s prefixes and is unsuitable for a matched campaign.
- Final campaign: all five waveform reports passed, all ten clean/jammed GNSS-SDR processes exited successfully and passed the pre-onset PVT gate, and every jammed receiver produced 9 PRN PNG/PDF figure pairs.
- Primary common stable PRNs: G10, G12, G23, G28, G31, G32. Median matched jammer-interval C/N0 changes were CW -2.46 dB, narrowband -13.07 dB, wideband -10.94 dB, chirp -14.24 dB, and pulsed -8.98 dB.
- Matched clean controls produced 19/20 PVT fixes and median 6 usable satellites during seconds 40-60 for every type. Jammed PVT was CW 19/20, narrowband 0/20, wideband 0/20, chirp 0/20, pulsed 19/20.
- Narrowband and wideband restored PVT 7 s after jammer-off. CW and pulsed retained/restored immediately. Chirp restoration was not observed before GNSS-SDR NMEA ended near scenario second 72; tracking dumps continue through 90 s.
- Raw waveform QA now includes steady spectra, full-run mixed spectrograms, and dense first-250-ms jammer-only spectrograms. The dense view resolves chirp sweeps and pulse cadence without temporal aliasing.
- Strict descriptive artifacts are under `analysis-output/`: analysis report, stats appendix, figure catalog, PRN gallery, machine-readable metrics, and PNG/PDF summary figures.
- Final verification: `make test` passed all C targets plus 3 Python tests; Ruff format/check passed; Pyright reported 0 errors/warnings; Python byte compilation passed; security search found no secrets or shell execution; `git diff --check` and explicit untracked-file whitespace checks passed.
- Artifact integrity: all five clean/jammed fixture pairs are byte-identical over their first 40 s, every waveform report has four existing plots, every receiver resolved to 9 fixed concurrent acquisitions, all 45 jammed PRN PNGs and 45 matching PDFs exist, all analysis Markdown links resolve, and all summary PNG/PDF files parse.
- No UHD command was run and no RF transmission occurred.

## Existing Implementation

- The existing CW work is uncommitted and spans `tools/jammer_source.{c,h}`, `tools/jammergen.c`, `tools/iqmix.c`, `player/jammertx.cpp`, `processing/cw_dataset.py`, tests, build files, and the requested docs.
- `jammergen` and `jammertx` already share the same stateful SC16 renderer. Extending that renderer preserves chunk-boundary invariance and avoids separate offline/live waveform implementations.
- `iqmix` measures clean and jammer power over the steady global interval, then scales any jammer IQ to requested J/S. It does not assume a tone and needs no waveform-specific change.
- The Python workflow currently hard-codes `--type cw`, CW filenames/schema names, and tone/image acceptance checks.
- The clean baseline passes: C tools build, `tests/test_jammer_source` passes, and both Python dataset tests pass.
- The dirty worktree contains unrelated tracked and untracked changes. Changes for this task must remain confined to the jammer vertical slice and the three named docs, plus planning notes.

## Design Decisions

- Shared source enum: `cw`, `narrowband`, `wideband`, `chirp`, `pulsed`.
- Type parameters: center/start frequency, chirp end and period, narrowband ENBW, RNG seed, pulse period/duty/ramp, plus the existing phase, amplitude, global interval, and global ramp.
- CW retains enum value zero and existing defaults so zero-initialized callers and existing tests remain compatible.
- Stochastic modes use a deterministic proper-complex Gaussian generator. Noise-mode default RMS will be lower than carrier modes to avoid SC16 clipping; measured power is still scaled to requested J/S by `iqmix`.
- Acceptance dispatch: tone/image for CW, spectral concentration for narrowband/wideband, instantaneous-frequency law for chirp, pulse envelope/timing for pulsed, and shared size/RMS/envelope/clipping/J/S/SNR gates for every type.
- Authentic GNSS IQ is not an input to any of these independent jammer generators. It is required only when the research question explicitly depends on real-sky desired signals, captured field interferers, real propagation/front-end artifacts, or field-valid mitigation performance.

## Verification Evidence

- Pre-change baseline: `make jammergen iqmix tests/test_jammer_source`, C test binary, and `uv run python ../tests/test_cw_dataset.py` all passed on 2026-07-12.
- Shared renderer tests now cover CW quadrature/byte stability, chunk invariance, seeded wideband repeatability and seed divergence, narrowband lag correlation, chirp frequency law, pulsed timing, and validation.
- `jammergen` generated all five modes with deterministic manifests and zero clipping at defaults for a 0.1 s smoke profile.
- `jammertx` built against local UHD 4.10.0.0_1; dry runs for all five modes produced valid v2 manifests without opening hardware.
- Live `jammertx` without `--confirm-controlled-rf` now exits with code 2 before opening UHD. The prior uncommitted baseline accidentally initialized that acknowledgement to true.
- The Python regression suite passes after the extension.
- Full `fast` dataset creation passed for narrowband, wideband, chirp, and pulsed: every clean/J/S fixture passed generation, mixing, and type-specific numerical analysis.
- `uvx ruff format/check` and Pyright completed with zero findings.
- `make test` passed, including all existing revive tests, the expanded shared-source tests, and the expanded five-type dataset tests. Existing `gpssim.c` library-test builds still emit unrelated unused-symbol warnings.
- `cc -std=c11 -Wall -Wextra -Wpedantic -fsyntax-only` passed for the shared source and offline CLI; the UHD target rebuilt successfully.
- UndefinedBehaviorSanitizer passed for the expanded shared-source test. An optional AddressSanitizer run was interrupted after exceeding 60 seconds without diagnostics on this macOS environment.
- `git diff --check`, untracked-file whitespace checks, and code-fence parity checks for all three requested docs passed.
