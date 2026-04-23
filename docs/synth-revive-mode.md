# Synthetic Satellite Revive Mode — Implementation Plan

## Status

Implemented — 2026-04-22

Field status after dataset `processing/dataset/22-4/ver3`: see
`docs/issues/synth-revive-ver3-current-state.md`.

## Goal

Add a new synthetic satellite mode (`SYNTH_REVIVE`) that injects a GPS PRN which is not currently in the sky by **re-animating the target PRN's own past orbital state at the current simulation time**. The satellite appears at the az/el it occupied at a recent past TOE, with natural forward orbital motion thereafter.

The core objective, stated as a receiver-side invariant:

> **Make the u-blox ZED-F9P receiver compute a satellite position at `t_now` — from the decoded navigation message — that equals the real ECEF position of the target PRN at `t_past = t_now − Δt`.**

Because the receiver uses the same `satpos()` formula as the simulator, and both evaluate the decoded ephemeris to the identical ECEF position, the pseudorange residual collapses to near zero. Every parameter in the broadcast ephemeris is drawn from a real, valid GPS broadcast — only the time-reference and node-longitude fields are shifted with exact bookkeeping.

## Relationship to Existing Modes

| Mode             | Orbit source                          | Position                | Collision with live sky                                                      |
| ---------------- | ------------------------------------- | ----------------------- | ---------------------------------------------------------------------------- |
| SYNTH_AZEL       | Synthesized circular orbit            | User-chosen az/el       | None, but ~1000 m residuals (see `issues/synth-azel-residual-divergence.md`) |
| SYNTH_CLONE      | Live donor PRN ephemeris              | Co-located with donor   | High if donor is visible; F9P flags `multi_spoof`                            |
| **SYNTH_REVIVE** | Target PRN's own past RINEX ephemeris | Target's own past az/el | None — target chosen non-visible                                             |

Revive is the minimum-divergence option for a cold-start F9P. No synthesis, no donor cross-wiring: every field of the broadcast ephemeris is either copied unchanged from a real broadcast or shifted with a closed-form transformation that preserves self-consistency with the simulator's ECEF output.

## Depends On

- `docs/synthetic-satellites.md` — core `-S` parser, `synth_config_t`, channel allocation bypass
- `docs/synth-clone-mode.md` — establishes the per-PRN synth overlay lifecycle, refresh tick cadence, and family-tracking pattern in `parseSynthConfig()`
- `docs/issues/synth-azel-residual-divergence.md` — the failure mode this feature closes

No dependency on `trimble-rtcm-ephemeris-sync.md`: revive uses the RINEX file loaded via `-e` as the template source. Live RTCM is only used (optionally) by the user to verify which PRNs are currently visible in the sky before selecting targets.

## Scope

**In scope:**

- `gpssim.h` — `SYNTH_REVIVE` enum entry, new constants, minor extensions to `synth_ephem_store_t`
- `gpssim.c` — extend `parseSynthConfig()`; add `scanEphemerisForRevive()`; add `reviveEphemerisFromTemplate()`; branch in `refreshSyntheticEphemerisSet()`, `overlaySyntheticEphemerisSet()`, `allocateChannel()`
- `player/bladetx.cpp` — CLI validation, startup scan, usage text

**Out of scope (future or explicit non-goals):**

- Per-PRN lookback override (`revive=-7200`) — all targets use the single global revive scan policy
- Template PRN different from target PRN — cross-PRN templating is what clone mode is for
- Automatic live-sky collision check — user responsibility to select non-visible targets
- Mirror to `player/x300tx.cpp` — follow after bladetx path is proven
- Adaptive refresh cadence — fixed 30-minute tick

## Design Decisions

| #   | Decision                                           | Choice                                                                                       |
| --- | -------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| D1  | CLI grammar                                        | `-S <prn>:revive` — no per-segment options                                                   |
| D2  | Default lookback                                   | 4 hours (14400 s) — avoids the too-fresh window that often only finds currently visible PRNs |
| D3  | Template source                                    | Target PRN's own ephemeris from the `-e` RINEX file                                          |
| D4  | Scan fallback if default doesn't place PRN visible | Expand outward in 15-minute steps up to 8h max; abort if still no hit                        |
| D5  | Minimum elevation for template acceptance          | 20° at `t_past` — avoids marginal rising/setting geometry                                    |
| D6  | Refresh cadence during long runs                   | 30 minutes (new constant `SYNTH_REVIVE_REFRESH_SEC`)                                         |
| D7  | IODE bump policy                                   | `iode += ceil(Δt / 7200) & 0xFF` — matches real GPS 2h broadcast cycle                       |
| D8  | Collision detection with live sky                  | Not automatic — user's responsibility                                                        |
| D9  | Required flag                                      | `-e` required; error if missing                                                              |

## User Experience

Example command (cold-start F9P, Hanoi, 4 revived targets):

```bash
bladetx \
  -e hour1120.26n \
  -l 21.0047844,105.8460541,5 \
  -P 1,2,7,8 \
  -S 1:revive,2:revive,7:revive,8:revive \
  --trimble-time-tag-host 192.168.5.245 \
  --trimble-time-tag-port 5017 \
  --trimble-tag-lead-ms 790 \
  --trimble-start-offset-sec 2 \
  --txvga1 -35
```

Startup stderr:

```
Revive PRN 01: template from t-7200s (toe=2415:266400), el=44.6 deg at t_past
Revive PRN 02: template from t-7200s (toe=2415:266400), el=13.2 deg at t_past  ← warn, will scan
Revive PRN 02: template from t-5400s (toe=2415:268200), el=28.4 deg at t_past  ← accepted
Revive PRN 07: template from t-7200s (toe=2415:266400), el=41.5 deg at t_past
Revive PRN 08: template from t-9000s (toe=2415:264600), el=22.8 deg at t_past
```

The user is responsible for selecting targets not currently in sky. Verify ahead of time by running:

```bash
gps-sdr-sim -e hour1120.26n -c <x>,<y>,<z> -t <t_now>
```

and confirming each chosen target does not appear in the visible list.

## CLI Grammar

Extend `-S` segment spec with a new `revive` form. All existing forms remain unchanged.

```
synth_spec   := segment (',' segment)*
segment      := prn ':' segment_spec
segment_spec := 'force'                       [existing]
              | 'overhead'                    [existing]
              | az '/' el                     [existing]
              | 'clone=' src_prn              [existing]
              | 'revive'                      [NEW]
```

### Parser Rules

1. Each segment belongs to one of three families: **CLASSIC** (`force`, `overhead`, `az/el`), **CLONE** (`clone=N`), or **REVIVE** (`revive`).
2. All segments in one `-S` argument must share a family. Mixing is a parse error.
3. `revive` takes no argument.
4. Valid PRN range: `1 ≤ prn ≤ MAX_SAT`.

### Error Messages

```
ERROR: Cannot mix revive and az/el synthetic modes in one -S argument.
ERROR: Cannot mix revive and clone synthetic modes in one -S argument.
ERROR: Revive mode requires -e ephemeris file.
ERROR: Revive PRN %d: no ephemeris found within %.1fh lookback.
ERROR: Revive PRN %d: not above %.1f deg at any point in lookback window.
```

## Data Structures

### gpssim.h — Enum Extension

```c
typedef enum {
  SYNTH_NONE = 0,
  SYNTH_FORCE,
  SYNTH_OVERHEAD,
  SYNTH_AZEL,
  SYNTH_CLONE,
  SYNTH_REVIVE      /* NEW — re-animate target PRN's past ephemeris */
} synth_mode_t;
```

### gpssim.h — Store Extension

Add bookkeeping so the refresh tick can detect when a different past window was picked (controls IODE continuity):

```c
typedef struct {
  ephem_t   eph[MAX_SAT];
  int       valid[MAX_SAT];
  double    revive_delta_sec[MAX_SAT];    /* NEW — Δt used for this target */
  gpstime_t template_toe[MAX_SAT];        /* NEW — past toe, for IODE continuity */
} synth_ephem_store_t;
```

No new fields in `synth_config_t` — revive is parameter-free at the CLI.

### gpssim.h — Constants

```c
#define SYNTH_REVIVE_DEFAULT_LOOKBACK_SEC  14400.0    /* 4 hours */
#define SYNTH_REVIVE_MAX_LOOKBACK_SEC      28800.0    /* 8 hours ceiling */
#define SYNTH_REVIVE_SCAN_STEP_SEC            900.0   /* 15-minute scan step */
#define SYNTH_REVIVE_MIN_ELEVATION_DEG         20.0
#define SYNTH_REVIVE_REFRESH_SEC             1800.0   /* 30-minute refresh tick */
#define SYNTH_REVIVE_IODE_BUMP_PER_SEC       7200.0   /* IODE +1 per 2h shift */
```

## Parameter Transform Model (The Core)

This section is the heart of the feature. Everything else is plumbing.

### Notation

| Symbol     | Meaning                                                 |
| ---------- | ------------------------------------------------------- |
| `template` | `ephem_t` copied from RINEX at past time `t_past`       |
| `modified` | Output ephemeris, time-shifted to `t_now`               |
| `Δt`       | `t_now − t_past` (default 14400 s)                      |
| `Ω_E`      | `OMEGA_EARTH` = 7.2921151467e-5 rad/s                   |
| `tk`       | `t_rx − toe` as seen by the receiver when satpos() runs |

Implementation note: the scanner searches around the requested lookback, then
uses the selected template's actual TOE as `t_past`. This keeps the closed-form
transform exact because `M0` is the mean anomaly at TOE.

### Receiver-Side Invariant We Must Satisfy

When the F9P decodes the broadcast ephemeris and runs:

```
satpos(modified, t_now) → P_sat_ecef
```

the result must equal:

```
satpos(template, t_past) → P_sat_ecef  (same value)
```

This is the real ECEF position of the satellite at `t_past`, which by the scan criterion is above 20° elevation from the receiver. The simulator builds the transmitted code phase from the same `P_sat_ecef`, so when the receiver compares measured to predicted pseudorange, the residual is the ephemeris-independent geometry + clock error only — no ~1000 m bias from orbit-model mismatch.

### The Transform

```c
void reviveEphemerisFromTemplate(
    ephem_t         *modified,
    const ephem_t   *template,
    double           delta_sec,
    gpstime_t        t_now_gps)
{
    /* Start from a full copy — keeps all orbit-shape and rate fields. */
    *modified = *template;

    /* --- Time references: re-stamp to t_now on a 16 s boundary ---
     *
     * GPS ephemeris toe has an LSB of 16 s. Aligning to the nearest
     * boundary avoids quantization drift between the broadcast toe and
     * what the subframe encoder emits.
     */
    long toe_aligned = ((long)(t_now_gps.sec + 8) / 16) * 16;
    modified->toe  = toe_aligned;
    modified->toc  = toe_aligned;
    modified->week = t_now_gps.week;

    double tk_at_now = t_now_gps.sec - toe_aligned;
    double toe_delta = toe_aligned - template->toe.sec;

    /* If t_now is not exactly on the 16 s TOE boundary, the receiver will
     * evaluate the ephemeris at tk != 0. Move the phase-at-TOE fields backward
     * so the phase at t_now is still the template phase at t_past.
     */
    modified->M0   = template->M0   - n * tk_at_now;
    modified->inc0 = template->inc0 - template->idot * tk_at_now;

    /* --- Right ascension of ascending node: compensate Earth rotation ---
     *
     * satpos() computes:
     *     Ω_k = Omega0 + (OmegaDot − Ω_E) * tk − Ω_E * toe
     *
     * For the receiver to place the satellite at the same ECEF node
     * rotation as at the old toe_old when it evaluates at t_now, we need:
     *     Omega0_new + (OmegaDot − Ω_E) * tk_at_now − Ω_E * toe_new
     *         == Omega0_old − Ω_E * toe_old
     *
     * This is the single most important field in the transform. Without
     * this compensation, every revived satellite is rotated ~30°/2h off
     * its intended sky position.
     */
    modified->omg0 = template->omg0
                   + OMEGA_EARTH * toe_delta
                   - (template->omgdot - OMEGA_EARTH) * tk_at_now;
    while (modified->omg0 >  M_PI) modified->omg0 -= 2.0 * M_PI;
    while (modified->omg0 < -M_PI) modified->omg0 += 2.0 * M_PI;

    /* --- Satellite clock bias: propagate forward by Δt ---
     *
     * Real satellite clocks drift between broadcasts. Without
     * propagation, dt_sv evaluated at the new toc would equal the old
     * sat clock's value at the old toc — which is the clock "frozen"
     * 2h in the past. Receiver applies the same af0 we broadcast, so
     * the simulator/receiver stay self-consistent either way; but for
     * analyst stealth (and to mimic how a real broadcast would look
     * 2h later), we extrapolate af0 using af1 and af2:
     *
     *     af0_new = af0_old + af1_old * Δt + ½ * af2_old * Δt²
     *
     * Typical magnitude: af1 ~ 1e-12 s/s, so 2h = ~7 ns ~ 2 m.
     * Negligible for F9P tracking; matters only for traces compared
     * against real IGS products.
     */
    modified->af0 = template->af0
                  + template->af1 * delta_sec
                  + 0.5 * template->af2 * delta_sec * delta_sec;
    /* af1, af2 unchanged — drift rate and acceleration are stable
     * over many hours. tgd unchanged — hardware delay. */

    /* --- IODE bump ---
     *
     * Real GPS issues a new IODE roughly every 2 hours. Advancing by
     * ceil(Δt / 7200) matches the pattern the receiver would see if
     * the real satellite had gone through Δt/7200 ephemeris cycles.
     * On a cold-start F9P the IODE value is functionally irrelevant
     * (no cache to invalidate), but this keeps the broadcast self-
     * plausible for any observer.
     */
    int n_bumps = (int)ceil(delta_sec / SYNTH_REVIVE_IODE_BUMP_PER_SEC);
    modified->iode = (template->iode + n_bumps) & 0xFF;
    modified->iodc = modified->iode;

    /* --- Flags ---
     * Force healthy regardless of template, set standard 4h fit. */
    modified->health = 0;
    modified->fit    = 1;
    modified->vflg   = 1;

    /* --- MOSTLY UNCHANGED FIELDS ---
     *
     *   M0/inc0 — shifted only to compensate the 16 s TOE rounding. At t_now,
     *             the evaluated mean anomaly and inclination match the
     *             template's values at t_past.
     *
     *   sqrtA   — semi-major axis (orbit size)
     *   ecc     — eccentricity
     *   aop     — argument of perigee
     *   deltan  — mean motion correction (dN)
     *   idot    — inclination rate
     *   omgdot  — right ascension rate (OmegaDot, secular)
     *
     *   cuc, cus, cic, cis, crc, crs — perturbation corrections.
     *   These are empirically fit for the orbital phase around the
     *   template's toe. We reuse them at the same evaluated phase, so they
     *   apply at the same phi_k where they were tuned — arguably more
     *   accurate than a real 2-hour-later broadcast, which would have re-fit
     *   them to the satellite's new phase.
     */
}
```

### Why M0 Stays Anchored To The Template Phase (Crucial)

The naive approach would be to propagate M0 forward by `n · Δt` to reflect "where the satellite is now". That is wrong for this feature.

Propagating M0 forward places the satellite at its real current orbital phase — which is the phase where the satellite is below horizon (by definition, since we picked a target not currently visible). The synthetic satellite would then appear at its real not-visible position.

When `t_now` lands exactly on the broadcast TOE boundary, keeping the broadcast
`M0` unchanged freezes the satellite at the phase it occupied at `t_past`, which
is the phase that the scan confirmed above 20° elevation. When TOE is rounded to
the nearest 16-second broadcast boundary, the receiver evaluates the ephemeris at
`tk_at_now = t_now - toe_new`; in that case the broadcast `M0` is shifted by
`-n * tk_at_now` so the evaluated phase at `t_now` is still the old template
phase.

### Why Phase Anchoring + Omega0 Shift Works (Full Derivation)

Evaluate `satpos()` with the modified ephemeris at sim time `t_now`, where
`tk_at_now = t_now - toe_new`:

```
M0_new = M0_old − n * tk_at_now
Mk  = M0_new + n * tk_at_now = M0_old
Ek  = kepler(M0_old, e)                 ← same Ek as in template at t_past
vk, φk = same                           ← orbit-plane coordinates unchanged
uk, rk, ik = same after perturbations
xp, yp = same in orbital plane

Omega0_new = Omega0_old
           + Ω_E * (toe_new − toe_old)
           − (OmegaDot − Ω_E) * tk_at_now

Ω_k = Omega0_new + (OmegaDot − Ω_E) * tk_at_now − Ω_E * toe_new
    = Omega0_old − Ω_E * toe_old
    = Ω_k_old                           ← same ECEF node rotation

X, Y, Z = same as satpos(template, t_past)
```

So the position at `t_now` using the revive ephemeris equals the real satellite's position at `t_past`.

### Forward Motion During the Simulation Run

At sim time `t = toe_new + tk` with the revive ephemeris:

```
Ω_k_revive(tk) = Omega0_new + (OmegaDot − Ω_E) * tk − Ω_E * toe_new
              = Omega0_old − Ω_E * toe_old + (OmegaDot − Ω_E) * tk
```

Compare to what the old ephemeris would give for a real satellite at absolute time `t_past + tk`:

```
Ω_k_real(tk) = Omega0_old + (OmegaDot − Ω_E) * tk − Ω_E * toe_old
             = Omega0_old − Ω_E * toe_old + (OmegaDot − Ω_E) * tk
```

Identical. The Earth-rotation compensation in Omega0 also makes the orbit-rotation rate at later `tk` values exactly match natural orbital motion from the `t_past` anchor point. No additional bookkeeping needed.

Practical consequence: the revived satellite drifts through the sky at its real orbital rate (~0.25°/min), starting from its historical (az, el). The receiver sees natural motion, not a frozen point.

### Clock Bias Self-Consistency

Sim and receiver both evaluate:

```
dt_sv(t) = af0_new + af1 * (t − toc_new) + ½ * af2 * (t − toc_new)²
```

with the same (af0_new, af1, af2, toc_new). They agree bit-for-bit. The receiver's clock residual for this satellite is zero in expectation (measurement noise only).

## Refresh Lifecycle

### `scanEphemerisForRevive()` — gpssim.c (new)

```c
int scanEphemerisForRevive(
    const ephem_t  rinex_eph[][MAX_SAT],
    int            n_sets,
    int            target_prn,
    gpstime_t      t_now,
    const double   rx_ecef[3],
    ephem_t       *out_template,
    gpstime_t     *out_template_toe,
    double        *out_delta_sec,
    double        *out_elev_deg,
    int           *out_found_ephem);
```

Scan pattern (prefer shortest delta for freshest ephemeris):

1. Try `Δ = 14400 s` first.
2. If no hit, expand: `Δ ∈ { 14400+900, 14400−900, 14400+1800, 14400−1800, ... }` up to ±3600 s around default.
3. If still no hit, extend the range to `[1800, 28800]` in 900 s steps.
4. For each candidate `Δ`:
   - `t_past = t_now − Δ`
   - Find the RINEX ephemeris set whose `toe` is closest to `t_past` for `target_prn`. Accept only if `abs(toe − t_past) < 7200 s` (satellite actually had a valid broadcast near that time).
   - Use the selected template TOE as the actual revived past state.
   - Run `satpos(template, template.toe)` → compute az/el from `rx_ecef`.
   - Accept if elevation ≥ `SYNTH_REVIVE_MIN_ELEVATION_DEG`.
5. If the full scan fails, return `FALSE`.

The first success is the best (shortest delta = freshest template).

### `refreshSyntheticEphemerisSet()` — gpssim.c

Add a SYNTH_REVIVE branch before the existing AZEL/OVERHEAD skip:

```c
for (sv = 0; sv < MAX_SAT; sv++) {
  if (cfg->mode[sv] == SYNTH_REVIVE) {
    ephem_t   template;
    gpstime_t template_toe;
    double    delta_sec;

    if (!scanEphemerisForRevive(
           rinex_eph, n_sets, sv + 1, t_now, rx_ecef,
           &template, &template_toe, &delta_sec, &elev_deg,
           &found_ephem)) {
      /* No window available — keep prior ephemeris if any.
       * If this is the initial scan, bladetx startup will abort earlier. */
      continue;
    }

    /* Only bump IODE if the chosen template differs from the last tick. */
    int template_changed =
        (store->template_toe[sv].week != template_toe.week) ||
        (store->template_toe[sv].sec  != template_toe.sec);

    reviveEphemerisFromTemplate(&store->eph[sv], &template,
                                delta_sec, t_now);

    if (!template_changed && store->valid[sv] == TRUE) {
      /* Revert IODE bump so the receiver sees a stable value between
       * refresh ticks that picked the same template. */
      store->eph[sv].iode = store->eph[sv].iode & 0xFF;  /* no-op if unchanged */
      /* TODO: cache last IODE to restore exactly — see Open Questions */
    }

    store->valid[sv]            = TRUE;
    store->revive_delta_sec[sv] = delta_sec;
    store->template_toe[sv]     = template_toe;
    changed = TRUE;
    continue;
  }

  /* ... existing SYNTH_CLONE / AZEL / OVERHEAD logic unchanged ... */
}
```

### `overlaySyntheticEphemerisSet()` — gpssim.c

Extend the mode check:

```c
for (sv = 0; sv < MAX_SAT; sv++) {
  if ((cfg->mode[sv] == SYNTH_OVERHEAD ||
       cfg->mode[sv] == SYNTH_AZEL     ||
       cfg->mode[sv] == SYNTH_CLONE    ||
       cfg->mode[sv] == SYNTH_REVIVE)  &&
      store->valid[sv] == TRUE) {
    dst[sv] = store->eph[sv];
  }
}
```

### `allocateChannel()` — gpssim.c

Add `SYNTH_REVIVE` to the existing elevation-mask bypass check alongside `SYNTH_FORCE`, `SYNTH_AZEL`, `SYNTH_CLONE`.

### Refresh Cadence

The clone-mode 30-second tick is too aggressive for revive (IODE churn). Introduce a dedicated 30-minute tick:

```c
if ((t_now.sec - last_revive_refresh_sec) >= SYNTH_REVIVE_REFRESH_SEC) {
  refreshSyntheticEphemerisSet(&synth_store, &synth_cfg,
                               rinex_eph, n_sets, t_now, rx_ecef);
  last_revive_refresh_sec = t_now.sec;
}
```

## Parser Implementation Sketch — gpssim.c

Extend `parseSynthConfig()` to accept `revive` and track a third family:

```c
int this_family;

if (strcmp(colon + 1, "revive") == 0) {
    cfg->mode[prn - 1] = SYNTH_REVIVE;
    this_family = 2;
}
else if (strncmp(colon + 1, "clone=", 6) == 0) {
    /* ... existing clone parse, this_family = 1 ... */
}
else {
    /* ... existing force / overhead / az/el parse, this_family = 0 ... */
}

if (first_family < 0) {
    first_family = this_family;
} else if (first_family != this_family) {
    const char *a = (first_family == 0) ? "az/el"
                  : (first_family == 1) ? "clone"
                  :                        "revive";
    const char *b = (this_family == 0)  ? "az/el"
                  : (this_family == 1)  ? "clone"
                  :                        "revive";
    fprintf(stderr,
            "ERROR: Cannot mix %s and %s synthetic modes in one -S argument.\n",
            a, b);
    return FALSE;
}
```

Also: zero `store->revive_delta_sec[]` and `store->template_toe[]` in `initSynthStore()`.

## Startup Sequence — player/bladetx.cpp

### Step 1 — CLI Validation

After option parsing:

```cpp
int has_revive = 0;
for (int s = 0; s < MAX_SAT; s++) {
  if (synth_cfg.mode[s] == SYNTH_REVIVE) { has_revive = 1; break; }
}
if (has_revive && navfile[0] == '\0') {
  fprintf(stderr, "ERROR: Revive mode requires -e ephemeris file.\n");
  return 1;
}
```

### Step 2 — Initial Revive Scan (fail-fast)

After RINEX load but before the simulation main loop:

Important for `bladetx`: `-n`, `-t`, explicit GPS week/TOW, and Trimble
time-tag mode can shift the active RINEX TOE/TOC values to the transmit epoch.
Revive must keep a preserved copy of the original RINEX table
(`revive_scan_eph`) for template search, while `eph` remains the active,
time-overwritten table used for normal broadcast scheduling. Startup scan and
30-second refresh both pass that preserved source into
`refreshSyntheticEphemerisSet()`.

```cpp
const ephem_t (*synth_source)[MAX_SAT] = has_revive ? revive_scan_eph : eph;

for (int s = 0; s < MAX_SAT; s++) {
  if (synth_cfg.mode[s] != SYNTH_REVIVE) continue;

  ephem_t   template;
  gpstime_t template_toe;
  double    delta_sec;

  if (!scanEphemerisForRevive(synth_source, n_sets, s + 1, g0, xyz[0],
                              &template, &template_toe, &delta_sec)) {
    fprintf(stderr,
            "ERROR: Revive PRN %d: not above %.1f deg at any point "
            "in lookback window.\n",
            s + 1, SYNTH_REVIVE_MIN_ELEVATION_DEG);
    status = 1;
    goto cleanup_module;
  }

  /* Log the chosen past window; el_past is computed inside the scanner
   * and returned via an out parameter (add if not already there). */
  fprintf(stderr,
          "Revive PRN %02d: template from t-%.0fs "
          "(toe=%d:%.0f), el=%.1f deg at t_past\n",
          s + 1, delta_sec, template_toe.week, template_toe.sec,
          elev_past_deg);
}
```

### Step 3 — Usage Text

Update `bladetx_usage()`:

```
-S <synth_spec>             Synthetic satellites. One family per -S arg:
                            classic: PRN:force | PRN:overhead | PRN:az/el
                            clone:   PRN:clone=<src_prn>
                            revive:  PRN:revive
                            Revive requires -e and injects PRN at the az/el
                            it occupied in the recent past, using its own past ephemeris.
```

## Implementation Tasks

### Task 1 — Data Structures

- [x] `gpssim.h`: add `SYNTH_REVIVE` to `synth_mode_t`
- [x] `gpssim.h`: add `revive_delta_sec[]` and `template_toe[]` to `synth_ephem_store_t`
- [x] `gpssim.h`: add the six `SYNTH_REVIVE_*` constants
- [x] `gpssim.c`: zero the new store fields in `initSynthStore()`

### Task 2 — Parser

- [x] `gpssim.c` `parseSynthConfig()`: accept `revive` segment form
- [x] Extend family tracking from two (CLASSIC/CLONE) to three (CLASSIC/CLONE/REVIVE)
- [x] Produce the three mix-mode error strings

### Task 3 — Transform Function

- [x] `gpssim.h`: declare `reviveEphemerisFromTemplate()`
- [x] `gpssim.c`: implement transform model (Omega0 compensation, af0 propagation, IODE bump, time re-stamp)
- [x] Inline comments explain the receiver-side invariant and each formula

### Task 4 — Scan Function

- [x] `gpssim.h`: declare `scanEphemerisForRevive()`
- [x] `gpssim.c`: implement, prefer default/nearby deltas, then extend the 15-minute scan
- [x] Helper `find_ephem_for_prn_at_time()` to index the multi-set RINEX table by (PRN, nearest toe)

### Task 5 — Refresh + Overlay + Allocation

- [x] `gpssim.c` `refreshSyntheticEphemerisSet()`: add SYNTH_REVIVE branch; use 30-minute cadence
- [x] `gpssim.c` `overlaySyntheticEphemerisSet()`: include SYNTH_REVIVE in mode check
- [x] `gpssim.c` `allocateChannel()`: add SYNTH_REVIVE to the elevation-mask bypass

### Task 6 — bladetx Wiring

- [x] `player/bladetx.cpp`: Step 1 CLI validation
- [x] `player/bladetx.cpp`: Step 2 initial scan + fail-fast
- [x] `player/bladetx.cpp`: Step 3 usage text update
- [x] `player/bladetx.cpp`: preserve original RINEX for revive scans when the active ephemeris is time-overwritten

### Task 7 — Documentation

- [x] Update `docs/synthetic-satellites.md` summary table to include revive
- [x] Cross-reference this plan from `docs/issues/synth-azel-residual-divergence.md` as the second closing note (clone was the first)
- [x] Add one example to `docs/synthetic-satellite-spoofing-setup.md`

## Test Plan

### Unit Tests — `tests/`

**1. `test_parse_synth_revive.c`** — implemented

- [x] `-S 1:revive` → `mode[0] == SYNTH_REVIVE`
- [x] `-S 1:revive,2:revive,7:revive` → three revive entries
- [x] `-S 1:revive,2:20/60` → parse error with "revive and az/el" message
- [x] `-S 1:revive,2:clone=5` → parse error with "revive and clone" message
- [ ] `-S 1:revive,2:clone=5,7:20/60` → first mix error reported

**2. `test_revive_transform.c`** — implemented

- [x] Deterministic template with known (M0, Omega0, af0, af1, af2, iode) values
- [x] Call `reviveEphemerisFromTemplate()` with a non-16-second-aligned `delta_sec`
- [x] Assertions:
  - `toe == toe_rounded_to_16s` and `toc == toe`
  - `Omega0_new` includes both Earth-rotation and nonzero-`tk` compensation
  - `af0_new` matches the configured clock propagation model
  - `iode_new == (iode_old + ceil(delta_sec / 7200)) & 0xFF`
  - `M0` and `inc0` compensate TOE rounding; `sqrtA, ecc, aop, deltan, idot, omgdot, cuc..cis, crc, crs` remain byte-equal to template
  - Compute `satpos(modified, t_now)` and `satpos(template, t_past)`; assert `||P_new − P_past|| < 1.0 m`

**3. `test_revive_scan.c`** — implemented basic scanner coverage

- [x] Build synthetic multi-set RINEX where PRN X is above 20° at `t_past = t_now − 7200`
- [x] Call scanner with target PRN X; assert it picks `Δ = 7200`
- [ ] Move scenario so PRN X below 20° at `t_past = t_now − 7200` but above at `t_now − 5400`
- [ ] Assert scanner picks `Δ = 5400` (or next-best scan offset)

### Integration Tests — `tests/`

**4. `test_bladetx_revive_startup.sh`**

- Launch `bladetx -e <fixture.26n> -S 1:revive,2:revive ...` with a mock time-tag source
- Assert exit 0 after SIGINT
- Assert stderr contains `Revive PRN 01: template from t-`

**5. `test_bladetx_revive_requires_e.sh`**

- Launch `bladetx -S 1:revive` with no `-e`
- Assert exit != 0
- Assert stderr contains `Revive mode requires -e ephemeris file`

**6. `test_bladetx_revive_no_window.sh`**

- Fixture RINEX where PRN X never climbs above 20° within the configured max lookback
- Assert exit != 0
- Assert stderr contains `not above 20.0 deg at any point in lookback window`

### End-to-End Live Verification

**7. Live run against real ZED-F9P (cold start)**

- Same location and time setup as Dataset 9 / ver2 (`21.0047844, 105.8460541, 5`)
- F9P cold-start before each run (factory default)
- Pick 4 target PRNs not currently visible (verify via `rtcm3_inspect` or a `gps-sdr-sim -v` dry run at `t_now`)
- Use `-S <prn>:revive` for each
- Collect UBX (NAV-SAT, NAV-PVT, NAV-STATUS, RAWX) for at least 5 minutes
- Analyze with `tools/ubx_bladetx_cal.py`

Success criteria:

| Criterion                        | Target                                                    |
| -------------------------------- | --------------------------------------------------------- |
| NAV-SAT `prRes` for revived PRNs | < 20 m (down from 1000 m in SYNTH_AZEL baseline)          |
| NAV-PVT `svUsed`                 | Contains both real-sky and revived PRNs in the same epoch |
| NAV-STATUS `spoofDetState`       | `no_spoof` for majority of epochs, not `multi_spoof`      |
| 30-minute run stability          | Residuals bounded, no drift like COM4's ±60 m             |
| One refresh-tick transition      | No tracking drop observed when ephemeris IODE bumps       |

## Failure Modes Handled Explicitly

| Scenario                                               | Handling                                                                    |
| ------------------------------------------------------ | --------------------------------------------------------------------------- |
| Revive spec + no `-e`                                  | CLI validation error at startup                                             |
| Target PRN absent from RINEX at any past time          | Abort with "no ephemeris found within %.1fh lookback"                       |
| Target PRN present but never above 20° within lookback | Abort with "not above %.1f deg at any point in lookback window"             |
| Mixing revive with az/el                               | Parse error                                                                 |
| Mixing revive with clone                               | Parse error                                                                 |
| Multiple revive targets, one fails scan                | Abort — user picks different PRN                                            |
| Target PRN rises into live sky mid-run                 | User responsibility; no automatic check                                     |
| Simulation runs longer than 4h                         | Refresh tick at 30 min picks fresh past window; IODE bumps naturally        |
| GPS week rollover between `t_past` and `t_now`         | Handled: `modified.week = t_now.week`; `toe` is seconds-of-week, auto-wraps |
| Template `af2` very large (unusual sat)                | `af0` propagation includes the `½ * af2 * Δt²` term                         |

## Verification Checklist

- [x] `-S 1:revive` parses and resolves to `mode[0] == SYNTH_REVIVE`
- [x] Multi-revive `-S 1:revive,2:revive,7:revive` parses cleanly
- [x] Mixed `-S 1:revive,2:20/60` produces the exact revive-and-az/el error
- [x] Revive without `-e` rejected at CLI validation
- [ ] Target with no past visibility produces the exact error string
- [x] `reviveEphemerisFromTemplate()` satisfies `satpos(modified, t_now) ≈ satpos(template, t_past)` to < 1 m
- [ ] On cold-start F9P: revived PRN NAV-SAT `prRes` < 20 m
- [ ] On cold-start F9P: real-sky and revived PRNs both marked `svUsed` in same epoch
- [ ] 30-minute run survives at least one refresh-tick transition without tracking drop
- [ ] NAV-STATUS not flagged as `multi_spoof`

## Order of Work

1. Task 1 — data structures and constants
2. Task 3 — transform function, with unit test `test_revive_transform.c` (the self-consistency test against `satpos()` is the key acceptance gate)
3. Task 2 — parser, with unit test `test_parse_synth_revive.c`
4. Task 4 — scan function, with unit test `test_revive_scan.c`
5. Task 5 — refresh + overlay + allocation branches
6. Task 6 — bladetx wiring + integration tests
7. Task 7 — documentation
8. End-to-end live verification against bladeRF + F9P cold start

## Open Questions (Future Passes)

- Preserve exact IODE across refresh ticks that re-pick the same template. Current sketch bumps based on `ceil(Δt / 7200)`; since Δt drifts each tick, the same template can yield different IODEs. Fix: cache last emitted IODE per PRN and reuse if template unchanged.
- Adaptive refresh cadence: trigger when the revived satellite's elevation drops below, say, 15°, rather than fixed 30 min.
- Optional `-v` diagnostic that prints `(past_time, past_az, past_el)` for each revive target at startup and each refresh.
- Automatic collision detection against live RTCM 1004/MSM observations when the Trimble NTRIP source is already configured.
- Mirror to `player/x300tx.cpp`.

## Not Implemented By Design

- **Per-PRN lookback override** (e.g., `revive=-5400`): all targets share the single global scan policy. Simpler CLI; re-run with a code change if a concrete experiment needs different defaults.
- **Cross-PRN templating** (target uses another PRN's ephemeris): that is what clone mode is for. Revive deliberately uses the target's own history.
- **Automatic live-sky collision check at startup**: would require a live RTCM listener for a pure-RINEX mode. Not added here; user verifies non-visibility manually before the run.
- **Immediate refresh on ephemeris update**: refresh is a fixed 30-minute tick. Matches the cadence at which the receiver would see natural IODE updates in a real scenario.
