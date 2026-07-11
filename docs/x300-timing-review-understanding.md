# X300 Timing Alignment — Review & Understanding Checklist

Running doc for the review of `player/x300tx.cpp` (commit `71be733`, "update x300tx
timeline model") against `docs/x300-timing-alignment-plan.md`.

Goal of the work under review: make the *generated* GPS time line up with the
*real-sky* GPS time precisely enough that a u-blox receiver marks injected PRNs
`svUsed=Y` — i.e. uses them in the PVT solution, not just tracks them.

Legend: [x] verified correct in code · [!] risk / not-as-good-as-possible · [ ] understanding to confirm

---

## 1. The problem — why timing is the whole game

- [ ] Why "tracking" is not enough; what `svUsed`/`prUsed` actually require.
- [ ] What a *common* timing offset on the injected signal looks like to the
      receiver, and what absorbs it (receiver clock-bias vs. residual threshold).
- [ ] Why ~1 µs of misalignment ≈ 300 m of pseudorange error, and why that gets
      injected PRNs rejected from the solution.
- [ ] The four time bases (real GPS, Trimble tag, host monotonic, X300 hardware)
      and where each enters.

## 2. The solution — what the code does and why

- [x] **Phase 1 — exact rate contract.** Default 2.5 MHz = 200 MHz / 80 (integer
      divisor → requested == actual). Generator re-locks to `get_tx_rate()`.
      Fixes the old 2.6 MHz (200/2.6 = 76.9 → resampled, broke the contract).
- [x] **Phase 2 — no ms quantization.** `incGpsTimePrecise()` replaces
      `incGpsTime()` (which does `round(sec*1000)/1000`) for the `--trimble-tx-cal-ns`
      term and all block times.
- [x] **Phase 3 — time from emitted sample count.** `getGpsTimeAtSampleOffset(g0,
      generated_samples, rate, ppm)`. Each block's time is computed from the
      *absolute* sample index, so there is no per-epoch accumulation drift.
- [x] **Phase 3 — generateEpoch gets the block END time.** Verified against
      `computeCodePhase()`: rate = `(rho1.range - rho0.range)/dt`, initial phase
      from `rho0` (start), then `rho0 = rho1`. So `rho1`/`grx` = end of block;
      `rho0` (start) comes from the previous epoch (or `allocateChannel` at g0).
- [x] **Phase 4 — drift knob.** `--gps-time-ppm` scales elapsed time
      `(samples/rate)*(1+ppm·1e-6)`; flows into BOTH nav-msg TOW and code phase
      via `grx`. Correct sign is `ppm ≈ −oscillator_error_ppm`.
- [x] **Pre-buffer doesn't corrupt timing.** GPS time is keyed to sample index
      and preserved through the FIFO ring; `generated_samples` (gen progress) and
      `emitted_samples` (TX progress) are separate. First emitted sample = g0.
- [x] Compiles clean (`-O3 -Wall`, no warnings).

- [ ] Why "generation contract exact" ≠ "physically aligned to true GPS time".
- [ ] What `set_time_now(0)` + timed `md.time_spec` actually do, and where the
      soft (uncalibrated, jittery) part of the timeline lives.

## 3. Risks — why it is not yet "as good as possible"

- [!] **A. Stale calibration constants.** `--trimble-tx-cal-ns 580047` /
      `--gps-time-ppm 0.002894118` (command.md) were derived from a capture made
      with the OLD ms-rounding binary — the logs prove it (`cal=580000 ns` →
      `tow …001000000` = snapped to 1 ms). The new precise binary applies the
      exact value, so those numbers no longer mean the same on-air timing.
      → Re-calibrate from a neutral baseline on the new binary.
- [!] **B. `set_time_now(0)` start jitter.** `prep_elapsed` is measured *before*
      the (variable-latency) `set_time_now` call, so the real instant of
      device-time-zero is uncertain run-to-run. This is the dominant
      non-reproducible offset; calibration only holds *within* one run.
- [!] **C. No hardware PPS discipline.** The Trimble emits a real 1PPS, but the
      code always uses `set_time_now()` + a soft ASCII tag + guessed `tag-lead-ms`.
      Wiring PPS (+10 MHz) into the X300 and using `set_time_next_pps()` would
      replace the soft timeline with a hardware-locked one. Biggest available win.
- [!] **D. tag-lead / tx-cal / set_time_now latency are indistinguishable.** Only
      their *sum* is observable at the receiver, so `--trimble-tag-lead-ms` is not
      a physical quantity here — it is just another calibration knob.
- [ ] Minor: stale note1/note2 logs (show pre-fix .001); 30 s refresh trigger
      idempotency; ppm enters Doppler-rate but not the per-sample step (~1e-9).

## 4. Broader context — why it matters

- [ ] What this enables for the research (meaconing / selective injection study).
- [ ] The honest ceiling of software-only alignment vs. a disciplined clock.
- [ ] How the calibration feedback loop (`ubx_bladetx_cal.py`) closes: measure
      `median(real prRes) − median(injected prRes)` over `svUsed` sats → constant
      → `tx-cal-ns`, slope → `gps-time-ppm`.

---

### Verdict in one line
Code **faithfully matches the plan** (all 5 phases implemented, the load-bearing
"end-of-block time" question resolved correctly, clean build). It is **not yet
optimal** because the physical binding to true GPS time is still software-soft
(risks B/C) and the carried-over calibration is stale (risk A).
