# Synthetic Satellite Clone Mode — Implementation Plan

## Status

Draft — 2026-04-16

## Goal

Add a new synthetic satellite mode (`SYNTH_CLONE`) that backs each synthetic
PRN slot with the **live broadcast ephemeris of a different real GPS
satellite**, received over RTCM 1019 from a co-located Trimble receiver.

The receiver decodes a fully valid, self-consistent ephemeris for the
target PRN — identical in every field to what the donor satellite is
transmitting in the sky — which collapses the ~1000 m pseudorange residual
documented in `docs/issues/synth-azel-residual-divergence.md` and allows
both real-sky and synthetic PRNs to be marked `svUsed` in NAV-PVT
simultaneously.

## Depends On

`docs/trimble-rtcm-ephemeris-sync.md` must be implemented first. That plan
provides everything this plan consumes:

- `player/rtcm3_nav.hpp` / `.cpp` — RTCM v3 frame scanner, CRC24Q,
  1019 decoder, TCP socket lifecycle against port 5018
- `ephem_t gps_rtcm_eph[MAX_SAT]` cache populated from live 1019 messages
- Startup warmup (`--trimble-rtcm-warmup-sec`, `--trimble-rtcm-min-prns`)
- Runtime refresh during the 100 ms epoch loop
- `-e` becomes optional when `--trimble-rtcm-host` is set

Clone mode is a thin consumer on top of that cache. A reference decoder
already exists in `tools/rtcm3_inspect.c` — the production helper should
lift `crc24q`, `bits_u`, `bits_s`, `decode_1019`, and the week10→full-week
rollover logic from there verbatim. Do not abstract into a shared header.

## Scope

**In scope (this plan):**
- `gpssim.h` — new `SYNTH_CLONE` enum, new `source_prn[]` field
- `gpssim.c` — extend `parseSynthConfig()`, add `cloneEphemerisFromDonor()`,
  branch in `refreshSyntheticEphemerisSet()` and `overlaySyntheticEphemerisSet()`
- `player/bladetx.cpp` — CLI validation, donor verification after warmup,
  usage text

**Out of scope (future or explicit non-goals):**
- `player/x300tx.cpp` — mirror after bladetx path is proven
- Auto-selection of donors (`--synth-clone-count N`) — deferred
- Mixed-mode `-S` (clone + az/el in one argument) — non-goal per design Q4
- Collision detection when target PRN is visible in sky — user responsibility
- Replacing `SYNTH_AZEL` — keep intact for offline RINEX-only tests

## Design Decisions (from 2026-04-16 planning session)

| # | Decision | Choice |
|---|---|---|
| Q1 | CLI grammar | **B** — per-segment: `-S 4:clone=5,10:clone=11,16:clone=15` |
| Q2 | `-e` vs RTCM | **A** — clone mode requires `--trimble-rtcm-host`, error if missing |
| Q3 | Missing donor at warmup end | **A** — abort with explicit error |
| Q4 | Mixed modes in one `-S` | **B** — reject; all segments must share mode family |
| Q5 | Donor IODE change propagation | **B** — on next `SYNTH_EPHEM_REFRESH_SEC` tick (30 s) |
| Q6 | Donor drops from cache mid-run | **A** — keep last cached ephemeris |

## User Experience

Example command:

```bash
bladetx \
  -l 21.0047844,105.8460541,5 \
  -P 4,10,16 \
  -S 4:clone=5,10:clone=11,16:clone=15 \
  --trimble-time-tag-host 192.168.5.245 \
  --trimble-time-tag-port 5017 \
  --trimble-rtcm-host 192.168.5.245 \
  --trimble-rtcm-port 5005 \
  --trimble-rtcm-mount NAVIS \
  --trimble-rtcm-user NAVIS:navis123 \
  --trimble-tag-lead-ms 790 \
  --trimble-start-offset-sec 2 \
  --txvga1 -35
```

Meaning:
- PRN 4 is transmitted using PRN 5's live broadcast ephemeris
- PRN 10 uses PRN 11's ephemeris; PRN 16 uses PRN 15's
- The receiver decodes subframes for PRN 4 that describe PRN 5's orbit;
  synthetic PRN 4's position and pseudorange are therefore co-located with
  PRN 5's real orbital position, producing near-zero residuals

The user is responsible for picking target PRNs that are **not** currently
visible from the sky (use `rtcm3_inspect` on port 5018 to check).

If the raw Trimble TCP feed does not carry `1019` ephemeris messages, point
clone mode at an NTRIP source instead and provide
`--trimble-rtcm-mount` / `--trimble-rtcm-user`.

## CLI Grammar

Extend `-S` segment spec with a new `clone=<src>` form. Existing forms
remain unchanged.

```
synth_spec   := segment (',' segment)*
segment      := prn ':' segment_spec
segment_spec := 'force'                      [existing]
              | 'overhead'                   [existing]
              | az '/' el                    [existing]
              | 'clone=' src_prn             [NEW]
```

### Parser rules

1. Each segment belongs to either the **CLASSIC** family (`force`,
   `overhead`, `az/el`) or the **CLONE** family (`clone=N`).
2. All segments in one `-S` argument must share a family (Q4=B).
3. `src_prn` must satisfy `1 ≤ src_prn ≤ MAX_SAT`.
4. `target_prn != src_prn` — self-clone is rejected.
5. Multiple targets sharing a donor is allowed (no rule against it, but
   DOP degrades; document in the help text).

### Error messages (exact strings)

```
ERROR: Cannot mix clone and az/el synthetic modes in one -S argument.
ERROR: Clone source PRN must be 1..%d.                  (MAX_SAT)
ERROR: Clone target PRN cannot equal source PRN %d.
ERROR: Clone mode requires --trimble-rtcm-host.
ERROR: Clone donor PRN %d not found in RTCM cache after %d s warmup.
```

## Data Structures

### gpssim.h

Extend `synth_mode_t`:

```c
typedef enum {
  SYNTH_NONE = 0,
  SYNTH_FORCE,
  SYNTH_OVERHEAD,
  SYNTH_AZEL,
  SYNTH_CLONE       /* NEW — clone live ephemeris from source_prn */
} synth_mode_t;
```

Extend `synth_config_t`:

```c
typedef struct {
  synth_mode_t mode[MAX_SAT];
  double azimuth[MAX_SAT];
  double elevation[MAX_SAT];
  int    source_prn[MAX_SAT];   /* NEW — 1-indexed donor PRN, 0 if unset */
  int    enabled;
} synth_config_t;
```

`synth_ephem_store_t` needs no changes.

## Ephemeris Cloning Semantics

`ephem_t` does not carry a PRN field; PRN is implicit in the array index.
Cloning is therefore just a struct-level copy. The target PRN's identity
is established by:

- `codegen(chan[i].ca, chan[i].prn)` — C/A code for the target PRN
- `eph2sbf(active_eph[target-1], ...)` — subframes encoded from the donor's
  orbital and clock parameters

The broadcast nav message claims "PRN `target` is at donor's orbit,"
which the receiver accepts because the simulator's `computeRange()` uses
the same donor parameters to set the transmitted code phase.

### New function

```c
/* gpssim.h */
void cloneEphemerisFromDonor(ephem_t *dst, const ephem_t *donor);

/* gpssim.c */
void cloneEphemerisFromDonor(ephem_t *dst, const ephem_t *donor) {
  if (donor == NULL || donor->vflg != 1)
    return;                 /* caller handles the "no donor yet" branch */
  *dst = *donor;
  /* vflg is preserved by the struct copy */
}
```

## Refresh Lifecycle

Per Q5=B, clone refresh piggybacks on the existing
`SYNTH_EPHEM_REFRESH_SEC` (30 s) tick — no new hook into the RTCM
parser's "new 1019 received" path.

### `refreshSyntheticEphemerisSet()` — gpssim.c

The current loop skips sv whose `mode` is not `SYNTH_OVERHEAD` or
`SYNTH_AZEL`. Add a `SYNTH_CLONE` branch before that skip:

```c
for (sv = 0; sv < MAX_SAT; sv++) {
  if (cfg->mode[sv] == SYNTH_CLONE) {
    int src = cfg->source_prn[sv] - 1;

    if (src < 0 || src >= MAX_SAT)
      continue;

    if (real_set[src].vflg == 1) {
      /* Fresh donor available — copy through. */
      cloneEphemerisFromDonor(&store->eph[sv], &real_set[src]);
      if (store->valid[sv] != TRUE)
        changed = TRUE;
      store->valid[sv] = TRUE;
    }
    /* else Q6=A: leave store->eph[sv] / store->valid[sv] alone so the
       last-cached ephemeris keeps propagating until donor returns. */
    continue;
  }

  if (cfg->mode[sv] != SYNTH_OVERHEAD && cfg->mode[sv] != SYNTH_AZEL) {
    if (store->valid[sv] == TRUE)
      changed = TRUE;
    store->valid[sv] = FALSE;
    continue;
  }

  /* ... existing SYNTH_AZEL / SYNTH_OVERHEAD logic unchanged ... */
}
```

### `overlaySyntheticEphemerisSet()` — gpssim.c

Extend the mode check to include clone:

```c
for (sv = 0; sv < MAX_SAT; sv++) {
  if ((cfg->mode[sv] == SYNTH_OVERHEAD ||
       cfg->mode[sv] == SYNTH_AZEL     ||
       cfg->mode[sv] == SYNTH_CLONE)   &&
      store->valid[sv] == TRUE) {
    dst[sv] = store->eph[sv];
  }
}
```

### What does NOT need to change

- `computeRange()` — already uses `active_eph[prn-1]` generically
- `satpos()` — generic over any valid `ephem_t`
- `eph2sbf()` — generic over any valid `ephem_t`
- `allocateChannel()` — existing synth-mode bypass path (line ~2426) needs
  to accept `SYNTH_CLONE` the same way it accepts the others; see Task 4
- `generateNavMsg()` — no changes
- Nav refresh cadence — no changes
- Trimble time-tag path — no changes

## Parser Implementation Sketch — gpssim.c

Extend `parseSynthConfig()` (line 462+). Add family tracking at the top
of the function:

```c
int parseSynthConfig(synth_config_t *cfg, const char *spec) {
  char buf[1024];
  char *token;
  int first_family = -1;     /* -1 = none yet; 0 = CLASSIC; 1 = CLONE */

  /* ... existing buf copy / strtok setup ... */

  while (token != NULL) {
    /* ... existing colon split / PRN validate ... */

    int this_family;

    if (strncmp(colon + 1, "clone=", 6) == 0) {
      char *endptr;
      int src_prn = (int)strtol(colon + 1 + 6, &endptr, 10);

      if (*endptr != '\0' || src_prn < 1 || src_prn > MAX_SAT) {
        fprintf(stderr, "ERROR: Clone source PRN must be 1..%d.\n", MAX_SAT);
        return (FALSE);
      }
      if (src_prn == prn) {
        fprintf(stderr,
                "ERROR: Clone target PRN cannot equal source PRN %d.\n", prn);
        return (FALSE);
      }

      cfg->mode[prn - 1]       = SYNTH_CLONE;
      cfg->source_prn[prn - 1] = src_prn;
      this_family = 1;
    } else {
      /* ... existing force / overhead / az/el parsing ... */
      this_family = 0;
    }

    if (first_family < 0) {
      first_family = this_family;
    } else if (first_family != this_family) {
      fprintf(stderr,
              "ERROR: Cannot mix clone and az/el synthetic modes "
              "in one -S argument.\n");
      return (FALSE);
    }

    cfg->enabled = TRUE;
    token = strtok(NULL, ",");
  }
  return (TRUE);
}
```

Also: in `initSynthConfig()` (gpssim.c line 406+), zero
`cfg->source_prn[i]` alongside `cfg->mode[i]`.

## Startup Sequence — player/bladetx.cpp

Insertion points relative to current code:

### Step 1 — CLI validation (after option parsing, around line 930)

After the existing `navfile[0] == 0` check, add:

```cpp
/* Any SYNTH_CLONE entries require RTCM ephemeris mode. */
{
  int has_clone = 0;
  for (int s = 0; s < MAX_SAT; s++) {
    if (synth_cfg.mode[s] == SYNTH_CLONE) { has_clone = 1; break; }
  }
  if (has_clone && trimble_rtcm_host[0] == '\0') {
    fprintf(stderr, "ERROR: Clone mode requires --trimble-rtcm-host.\n");
    return 1;
  }
}
```

Note: `trimble_rtcm_host` and the RTCM listener setup come from the
`trimble-rtcm-ephemeris-sync.md` plan and are assumed to exist.

### Step 2 — Donor verification (after RTCM warmup)

Immediately after the warmup loop completes and `eph_live[0][]` is
populated from the RTCM cache:

```cpp
for (int s = 0; s < MAX_SAT; s++) {
  if (synth_cfg.mode[s] != SYNTH_CLONE) continue;

  int src = synth_cfg.source_prn[s] - 1;
  if (src < 0 || src >= MAX_SAT || eph_live[0][src].vflg != 1) {
    fprintf(stderr,
            "ERROR: Clone donor PRN %d not found in RTCM cache "
            "after %d s warmup.\n",
            synth_cfg.source_prn[s], trimble_rtcm_warmup_sec);
    status = 1;
    goto cleanup_module;
  }
  fprintf(stderr, "Clone PRN %02d <- donor PRN %02d (IODE=%d)\n",
          s + 1, synth_cfg.source_prn[s], eph_live[0][src].iode);
}
```

The rest of bladetx initialization (ephemeris selection, synthetic overlay
build-up at line ~3143) proceeds unchanged. The existing call to
`refreshSyntheticEphemerisSet()` at lines 3143-3144 will take the CLONE
branch automatically with `real_set = eph[ieph] = eph_live[0]`.

### Step 3 — Usage text update (`bladetx_usage()`)

Add to the `-S` help line:

```
  -S <synth_spec>             Synthetic satellites. One family per -S:
                              PRN:force | PRN:overhead | PRN:az/el
                              OR PRN:clone=<src_prn>
```

## Implementation Tasks

### Task 1 — Data structures

- [ ] `gpssim.h`: add `SYNTH_CLONE` to `synth_mode_t`
- [ ] `gpssim.h`: add `int source_prn[MAX_SAT]` to `synth_config_t`
- [ ] `gpssim.c` `initSynthConfig()`: zero `cfg->source_prn[]`

### Task 2 — Parser

- [ ] `gpssim.c` `parseSynthConfig()`: add `clone=N` segment form
- [ ] Add `first_family` tracking + mix-mode error
- [ ] Add self-clone check
- [ ] Add source-out-of-range check

### Task 3 — Clone function

- [ ] `gpssim.h`: declare `cloneEphemerisFromDonor()`
- [ ] `gpssim.c`: implement as a guarded struct copy

### Task 4 — Refresh + overlay + allocation

- [ ] `gpssim.c` `refreshSyntheticEphemerisSet()`: add `SYNTH_CLONE` branch
  before the existing `AZEL/OVERHEAD` skip
- [ ] `gpssim.c` `overlaySyntheticEphemerisSet()`: add `SYNTH_CLONE` to the
  mode-check condition
- [ ] `gpssim.c` `allocateChannel()` (line ~2426): ensure the
  `SYNTH_CLONE` mode bypasses the elevation-mask check the same way
  `SYNTH_FORCE`/`SYNTH_AZEL` do today

### Task 5 — bladetx wiring

- [ ] `player/bladetx.cpp`: Step 1 CLI validation (require RTCM for clone)
- [ ] `player/bladetx.cpp`: Step 2 donor verification after warmup
- [ ] `player/bladetx.cpp`: Step 3 usage text

### Task 6 — Usage / documentation

- [ ] Update any README or `command.md` example that touches `-S` to
  show clone syntax
- [ ] Cross-reference this plan from
  `docs/issues/synth-azel-residual-divergence.md` as the closing note
- [ ] Add a "Clone Mode" section in `trimble-rtcm-ephemeris-sync.md`
  pointing to this plan

## Test Plan

### Unit tests — tests/

1. **`test_parse_synth_clone.c`**
   - `-S 4:clone=5` → `mode[3]=SYNTH_CLONE`, `source_prn[3]=5`
   - `-S 4:clone=5,10:clone=11,16:clone=15` → three valid clone pairs
   - `-S 4:clone=5,7:20/60` → parse error with "mix" message
   - `-S 4:clone=4` → parse error with "self-clone" message
   - `-S 4:clone=99` → parse error with "source PRN" message
   - `-S 4:clone=5,7:overhead` → parse error with "mix" message

2. **`test_clone_refresh.c`**
   - Build synthetic `real_set[]` with valid `ephem_t` at index 4 (PRN 5)
   - Set `cfg.mode[3]=SYNTH_CLONE`, `cfg.source_prn[3]=5`
   - Call `refreshSyntheticEphemerisSet()`
   - Assert `store->eph[3]` byte-equals `real_set[4]`
   - Assert `store->valid[3] == TRUE`
   - Flip `real_set[4].vflg = 0`
   - Call `refreshSyntheticEphemerisSet()` again
   - Assert `store->eph[3]` unchanged (last-cached, Q6=A)
   - Assert `store->valid[3]` still `TRUE`

### Integration tests — tests/

3. **`test_bladetx_clone_startup.sh`**
   - Start a fake TCP server on localhost:5018 that emits valid RTCM 1019
     frames for PRN 5, 11, 15 (fixtures captured from real Trimble)
   - Launch `bladetx -S 4:clone=5,10:clone=11,16:clone=15
     --trimble-rtcm-host 127.0.0.1 --trimble-rtcm-port 5018 ...`
     with a mock time-tag source
   - Assert exit is 0 (or SIGINT after 2 s)
   - Assert stderr contains `Clone PRN 04 <- donor PRN 05` etc.

4. **`test_bladetx_clone_missing_donor.sh`**
   - Fake server emits only PRN 5, not PRN 11
   - Launch with `-S 4:clone=5,10:clone=11`
   - Assert exit code != 0
   - Assert stderr contains `Clone donor PRN 11 not found`

5. **`test_bladetx_clone_requires_rtcm.sh`**
   - Launch with `-S 4:clone=5` and no `--trimble-rtcm-host`
   - Assert exit code != 0
   - Assert stderr contains `Clone mode requires --trimble-rtcm-host`

### End-to-end live verification

6. **Live run against real Trimble + bladeRF + u-blox F9P**
   - Same geometry and position as Dataset 9 / ver2
   - Replace `-S` arg with clone form, picking donors that are visible
     in the Trimble sky at test time (from `rtcm3_inspect` output)
   - Collect UBX from both F9P receivers for ≥ 5 minutes
   - Analyze with `tools/ubx_bladetx_cal.py`
   - Success criteria:
     - NAV-SAT `prRes` for cloned PRNs < 20 m (down from ~1000 m)
     - NAV-PVT `svUsed` contains **both** real-sky PRNs and cloned PRNs
       in the same epoch
     - 30-minute run completes without synth-related errors or drops
     - Post-fit residuals stay bounded (no drift like COM4's +/-60 m)

## Failure Modes Handled Explicitly

| Scenario | Handling |
|---|---|
| Clone spec + no `--trimble-rtcm-host` | Abort at CLI validate step |
| Donor PRN absent after warmup | Abort with "donor PRN N not found" |
| Donor PRN drops from RTCM cache mid-run | Keep last cached ephemeris (Q6=A) |
| Donor's real IODE advances mid-run | Copied through on next 30 s tick (Q5=B); receiver sees normal ephemeris update for the synth PRN |
| Target PRN equals source PRN | Parse error |
| Mixed clone + az/el in one `-S` | Parse error |
| Two targets share same donor | Allowed; warning on stderr (degraded DOP) |
| Source PRN out of range | Parse error |
| Target PRN out of range | Existing parse error path |
| RTCM socket closes mid-run | Q6=A: keep last cached; synth keeps broadcasting until `ephem_t` exits its 4h validity window |

## Verification Checklist

- [ ] `-S 4:clone=5` parses correctly
- [ ] Multi-clone `-S 4:clone=5,10:clone=11,16:clone=15` parses correctly
- [ ] Mixed-mode `-S 4:clone=5,7:20/60` produces clear parse error
- [ ] Self-clone `-S 4:clone=4` produces clear parse error
- [ ] Clone without `--trimble-rtcm-host` rejected at startup
- [ ] Missing donor after warmup aborts with the exact error string
- [ ] On live Trimble: cloned PRN NAV-SAT `prRes` < 20 m
- [ ] On live Trimble: real-sky and cloned PRNs coexist in `svUsed`
- [ ] 30-minute run survives donor IODE transitions
- [ ] Donor dropping out mid-run keeps synth broadcasting (no crash)

## Order of Work

1. Land `trimble-rtcm-ephemeris-sync.md` Tasks 1–4 (prerequisite)
2. Task 1 — data structures
3. Task 2 — parser, with unit test `test_parse_synth_clone.c`
4. Task 3 — `cloneEphemerisFromDonor()`
5. Task 4 — refresh + overlay + allocation branches, with unit test
   `test_clone_refresh.c`
6. Task 5 — bladetx startup wiring + integration tests (fake RTCM server)
7. Task 6 — usage text + docs cross-references
8. End-to-end live verification against Trimble + bladeRF + F9P

## Open Questions (Future Passes)

- Auto-selection (`--synth-clone-count N`, visibility-based donor picking)
- Mirror to `player/x300tx.cpp`
- Optional collision warning when target PRN has a valid (fresh) ephemeris
  in the RTCM cache (likely visible in sky)
- Whether to expose a CLI knob to force an IODE bump when cloning, in case
  a future receiver under test balks at receiving a cached IODE it has
  not seen before

## Not Implemented By Design

- **Collision detection**: user is responsible for picking target PRNs not
  visible in the sky (verify via `rtcm3_inspect` before running bladetx).
  Explicit scope decision, 2026-04-16.
- **Mixed-mode `-S`**: segments within one `-S` argument must share mode
  family. Simplifies parser and is consistent with research test design.
- **Immediate refresh on 1019 arrival**: clones refresh on the existing
  30 s tick. Q5=B.
