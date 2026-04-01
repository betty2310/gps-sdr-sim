# Task Plan: Synthetic Satellite Time-Basis Mismatch

## Goal
Produce a concrete solution plan for eliminating the synthetic-versus-real GPS time-basis mismatch during mixed live-sky plus injected-satellite tests, with separate transmitter guidance for bladeRF 1.0 and USRP X300.

## Phases
- [x] Phase 1: Read issue, setup, and pipeline documentation
- [x] Phase 2: Inspect simulator timing and transmitter paths
- [x] Phase 3: Form root-cause hypothesis and platform constraints
- [x] Phase 4: Write persistent notes and solution plan
- [x] Phase 5: Implement software-only streaming path
- [x] Phase 6: Run local non-hardware verification

## Key Questions
1. Is the dominant failure caused by simulator timing math or by delayed replay of a pre-generated I/Q file?
2. What changes are required on the transmitter side for bladeRF 1.0 and USRP X300?
3. What operational rules are mandatory for mixed real-plus-synthetic tests?

## Decisions Made
- Treat the issue primarily as a transmit-start alignment problem, not a synthetic orbit geometry problem.
- Treat raw file looping as invalid for mixed-sky testing because it rewinds GNSS time.
- Treat live streaming as the primary solution direction for both USRP X300 and bladeRF 1.0.
- Use `player/bladeplayer.c` as the bladeRF implementation reference instead of `bladeRF-cli` script replay.
- Keep USRP X300 as the preferred platform for the first robust implementation because its TX stack is easier to discipline.
- Implement the first software slice with FIFO/stdout streaming instead of adding a full generator-transmitter handshake protocol up front.

## Errors Encountered
- Minor shell search quoting mistake while scanning timing references. No impact on findings.
- `python3 -m py_compile` initially failed because the sandbox blocked default cache writes outside the workspace; verified successfully with `PYTHONPYCACHEPREFIX=/tmp`.

## Status
**Completed** - Findings are stored in `plan/notes.md`, the updated implementation plan is in `plan/synthetic-satellite-time-basis-solution-plan.md`, and the software-only stream path has been implemented in `gpssim.c`, `player/bladeplayer.c`, and `gps-sdr-sim-uhd.py`.
