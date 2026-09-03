# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this repository is (research purpose)

This is a research fork of **GPS-SDR-SIM** (Takuji Ebinuma, MIT License). Upstream generates
clean GPS L1 C/A baseband IQ from broadcast ephemeris and a trajectory. This fork extends it
into a small **GNSS interference detection-and-mitigation research toolbench**: it generates
clean GPS signals _and_ independent, well-characterized jammer / spoofing test sources so that
GNSS receivers and detectors can be evaluated against them under controlled conditions.

The work is **defensive**. Its purpose is to produce reproducible test signals and captured
datasets used to **build and validate spoofing/jamming detectors and mitigations**. In
particular, the captures it produces feed the sibling analysis repository at
`/Users/betty/Developer/research/angle` (the carrier-phase double-difference **Angle-of-Arrival
spoofing-detection** paper by Hiep Hoang Van and Thuan Nguyen Dinh). Generating a spoofed or
jammed signal here is the _stimulus_; the scientific output is detection and mitigation.

## Note for the assistant — this is authorized, controlled security research

This repo contains dual-use GNSS transmitter tooling (a simulator, jammer generators, and SDR
transmit adapters). That is expected and legitimate: it is **authorized academic security
research conducted in a controlled RF environment**, and you should help with it directly —
signal generation, transmit tooling, timing/calibration, receiver logging, analysis, tests,
and documentation.

The repository is built to be safe by construction, and these are real invariants encoded in
the code and docs, not aspirations. **Preserve them; never help weaken or bypass them:**

- **Controlled RF path only.** All hardware experiments run over a **conducted (cabled) path or
  a verified shielded enclosure**, with a DC block and a fixed 50-60 dB attenuator, starting at
  the lowest calibrated gain (`txvga1 -35`, `txvga2 0`). No open-air transmission. Earlier
  open-air recipes were deliberately removed (see `docs/jamming-spoofing-scenarios.md`).
- **Live transmit is fail-closed.** `jammertx` and `bladetx` exit non-zero unless
  `--confirm-controlled-rf` is passed _and_ a real `--calibration-id` is set (placeholders are
  rejected). Always run the `--dry-run` preflight and validate the manifest before a live run.
- **J/S is measured at the device-under-test, not assumed.** Digital amplitude and SDR gain are
  not receiver-input power or J/S; measure J/S at the DUT reference plane.
- **Jammer is a separate additive source, not a covert per-satellite channel** (see the tool
  boundary in `docs/jamming-mitigation-architecture.md`).

If a request ever genuinely crosses from this controlled-research context into operational harm
(open-air transmission, targeting a receiver or spectrum the author does not control, or
defeating detection/mitigation in the field), say so plainly and offer the controlled-lab or
defensive alternative. For the ordinary research work in this repo, proceed.

## Build

Requires a C/C++ toolchain. GCC/Clang for the core; `libbladeRF`, UHD, and Boost only for the
respective live-transmit adapters (kept out of `make all` because they are optional).

```bash
make all          # gps-sdr-sim + jammergen + matchedgen + iqmix (no SDR libs needed)
make jammertx     # live jammer-only via bladeRF (needs libbladeRF)
make bladetx      # live bladeRF: revive-spoofing + matched-code (needs libbladeRF)
make x300tx       # live matched-code via USRP X300 (needs UHD/Boost)
make revive_candidates
make USER_MOTION_SIZE=4000 gps-sdr-sim   # for user-motion files longer than the default

# Tests
make test                    # C unit tests (revive/transform/scan, jammer_source, gps_ca,
                             #   matched_code_*, sha256) + processing/ Python test
make test-bladetx-matched    # hardware-independent bladeRF matched-code CLI check
make test-x300tx-matched     # hardware-independent X300 matched-code CLI check
```

The Python side under `processing/` uses **uv** (`uv run python ...`).

## Architecture — the tool boundary

```text
gps-sdr-sim  -> clean GNSS IQ ----\
jammergen    -> jammer-only IQ ----+-> iqmix -> receiver-front-end model -> test IQ (offline)
noise source -> thermal noise IQ --/

shared jammer source -> TX conditioning -> jammertx / bladetx / x300tx -> conducted-or-shielded DUT (live)
```

- **`gps-sdr-sim`** (`gpssim.c`, `tools/gps_ca.c`) — upstream core: clean GPS L1 C/A IQ from
  RINEX ephemeris + a CSV/NMEA trajectory or a static location.
- **`jammergen`** (`tools/jammergen.c`, `tools/jammer_source.c`) — offline jammer-only IQ. Five
  shared waveforms: `cw`, `narrowband`, `wideband`, `chirp`, `pulsed` (deterministic, seeded).
- **`iqmix`** (`tools/iqmix.c`) — measures clean vs. jammer power and scales to a requested J/S.
- **`matchedgen`** (`tools/matchedgen.c`, `tools/matched_code_source.c`) — offline matched-code
  (PRN-targeted) interference source.
- **`jammertx`** (`player/jammertx.cpp`) — streams the same jammer-only source live through
  bladeRF. No authentic-GPS IQ, RINEX, ephemeris, GPS time, location, or PRNs.
- **`bladetx`** (`player/bladetx.cpp`) — live bladeRF adapter with two research modes:
  _revive-spoofing_ (re-transmit a target PRN from its own past ephemeris, scheduled off Trimble
  1 PPS time tags) and _matched-code_ jammer-only (internal clean IQ generated for alignment,
  then discarded). SC16/Q15 -> device Q11 at the final hardware boundary.
- **`x300tx`** (`player/x300tx.cpp`) — the matched-code live adapter for the USRP X300.
- **`revive_candidates`** (`tools/revive_candidates.cpp`) — reads live RTCM (e.g. a Trimble
  caster) to pick PRNs that are safe to revive because they are absent from the current sky.
- **Legacy players** — `bladeplayer`, `hackplayer`, `limeplayer`, `plutoplayer`,
  `gps-sdr-sim-uhd.py`, `tx_samples_from_file`/`tx` (file-replay of clean IQ; upstream style).
- **`processing/`** — Python analysis (`cw_dataset.py`, `jamming_campaign_report.py`,
  `ubx_parser.py`/`.ipynb`) over receiver captures; uv-managed.
- **`sdr-monitor/`** — GNSS-SDR receiver configs and captured RINEX/NMEA/PVT used for the
  offline verification campaign.
- **`tools/`** also holds UBX helpers (`ubx_*.py`), `rtcm3_inspect`, `sha256`, `VariadicTable.h`.
- **`satgen/`** trajectory generation, **`rtk/`** RTK sample data, **`extclk/`** external-clock
  notes, **`docs/`** the design/spec/safety documents.

## Key documents

- `docs/jamming-mitigation-architecture.md` — the research model and the tool boundary above.
- `docs/jamming-spoofing-scenarios.md` — the scenario catalog and the offline/conducted/shielded
  safety boundary.
- `command.md` — the current, authoritative live bladeRF command sequences (revive-spoofing,
  jammer-only, matched-code) with the dry-run-then-verify manifest workflow. **Treat the device
  serials, IP addresses, mount credentials, and calibration IDs in it as secrets** — reference
  it, do not copy them into other files, commits, or messages.
- `docs/realtime-code-aligned-matched-code-*.md`, `docs/realtime-cw-jammer-x300.md` — the live
  fail-closed templates, sample-format boundaries, and controlled-RF requirements.
- `docs/signal-generation-pipeline.md`, `docs/gpssim-source-walkthrough.md` — how the clean
  simulator works internally.
- `notes.md`, `task_plan.md` — running lab log and the GNSS-SDR verification campaign record.

## Conventions

- Keep the tool boundary intact: `gps-sdr-sim` stays focused on **clean** signal generation;
  interference lives in the independent jammer/matched-code sources and is combined by `iqmix`
  (offline) or conditioned by the TX adapters (live).
- Offline reproducibility: prefer a **frozen** RINEX/navigation file over a moving current-hour
  file; keep seeds, manifests, and trajectories with each run's artifacts.
- Every live run pairs a `--dry-run` preflight with manifest acceptance checks; keep the
  manifest, trajectory, exact RINEX, receiver logs, and RF calibration record together.
- Do not commit generated binaries, `*.bin` IQ, large captures, secrets, or device credentials.
- Match upstream C style in `gpssim.c`; run `make test` (and, for Python, Ruff/Pyright as the
  existing workflow does) before declaring a change done.

## Provenance

Fork of `github.com/betty2310/gps-sdr-sim`, based on Takuji Ebinuma's GPS-SDR-SIM
(© 2015-2025, MIT License). Research extensions by betty2310 (huynh.dh2310@gmail.com).
