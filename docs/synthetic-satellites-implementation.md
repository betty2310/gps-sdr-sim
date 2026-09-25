# Synthetic satellites: implementation map

Current reference, 2026-09-22. This document describes the shared ephemeris
overlay and its X300 integration. For mode behavior use
[synthetic satellites](synthetic-satellites.md), [revive](synth-revive-mode.md)
and [clone](synth-clone-mode.md). For hardware commands use
[command.md](../command.md).

## Source ownership

| Source | Responsibility |
|---|---|
| [gpssim.h](../gpssim.h) | Mode/config/store types, scan/refresh constants and public functions |
| [gpssim.c](../gpssim.c) | Parsing, orbit construction/transformation, overlay, range/code/navigation and IQ generation |
| [player/x300tx.cpp](../player/x300tx.cpp) | Mode compatibility, navigation-source selection, startup qualification and rendering |
| [player/gps_lnav.cpp](../player/gps_lnav.cpp) | Current X300 UBX GPS L1 ephemeris decoding and coherence checks |
| [player/x300_timing.hpp](../player/x300_timing.hpp) | PPS synchronization, queue/sender and continuity contract |
| [tools/revive_candidates.cpp](../tools/revive_candidates.cpp) | Historical template scan with optional observed-PRN exclusion |

Use the source definitions directly when changing data layouts. Earlier copies
of the structs in this document omitted clone source PRNs and revive lifecycle
fields and have been removed.

## Modes at the ephemeris layer

- `force` retains an input ephemeris and bypasses the ordinary visibility mask.
- `overhead` / `az/el` construct a circular GPS-like synthetic orbit for the
  requested apparent direction at the reference epoch, subject to model limits.
- `clone` copies another PRN's live ephemeris into the target slot.
- `revive` transforms the target's own past RINEX ephemeris to retain its past
  geometry at a synthetic scenario epoch.

The parser accepts comma-separated entries with valid target PRNs. Classic,
clone and revive are separate families; their entries cannot be mixed in one
specification. `-P` is the output selector for normal X300 waveforms.
Matched-code has a different selector, `--matched-code-target-prns`, and rejects
`-P`.

## Data flow

~~~mermaid
flowchart TD
    Args["-S specification"] --> Config["parseSynthConfig / synth_config_t"]
    Input["RINEX or validated UBX donor ephemeris"] --> Mode
    Config --> Mode{"mode"}
    Mode -->|force| Keep["retain input ephemeris"]
    Mode -->|overhead / az-el| Orbit["synthEphemeris"]
    Mode -->|clone| Clone["cloneEphemerisFromDonor"]
    Mode -->|revive| Revive["scanEphemerisForRevive / reviveEphemerisFromTemplate"]
    Orbit --> Store["synth_ephem_store_t"]
    Clone --> Store
    Revive --> Store
    Store --> Overlay["overlaySyntheticEphemerisSet"]
    Keep --> Overlay
    Overlay --> Active["active_eph"]
    Active --> Channels["allocateChannel"]
    Channels --> Model["satpos / computeRange / computeCodePhase"]
    Model --> IQ["navigation and C/A modulation"]
~~~

Synthetic channels reuse the ordinary renderer after overlay. Reusing this
pipeline does not prove that an independently decoding receiver sees identical
orbits or accepts the generated measurements. Navigation quantization,
propagation/clock approximations, RF delays and receiver behavior still matter.

## Current X300 sequence

1. Parse mode combinations before hardware access. Revive needs frozen `-e`;
   UBX and RINEX are exclusive. Live clone resolves target selections to donor
   ephemerides for readiness checks.
2. Load frozen navigation, or qualify the reference's time, PVT, RAWX and
   coherent donor/selected SF1–SF3. Save provenance.
3. In offline mode, bypass UHD. For hardware, configure/read the rate and
   reference sources and verify PPS. `--check-start` omits TX tuning/streamer.
4. Resolve sample-zero epoch. Live UBX fixes a future hardware deadline before
   dated model initialization. Frozen scenarios use their explicit scenario
   label and choose hardware start after prebuffering.
5. Build the scenario ephemerides and allocate the requested channels. An
   explicit `-P` selection containing only revived PRNs validates historical
   templates without requiring a current base set. Other selections retain
   the current-set check before building the synthetic overlay.
6. Render by cumulative sample count and actual configured rate. At navigation
   boundaries, refresh navigation and the overlay using the relevant source.
   Keep the original historical source available for revive.
7. Fail on invalid state, missed deadlines, clipping or transport continuity
   loss. Already accepted hardware samples may drain after failure; no silent
   retiming can repair their meaning.

Normal manifests include an input hash for frozen RINEX, GPS/sample mapping and
transport statistics. They do not contain all PRN/synthetic/gain arguments;
preserve the invocation and console log. The current implementation does not
claim verified absolute GPS alignment.

## Verification boundary

Existing parser, revive transformation and scan tests are included in
`make test`. X300 timing and UBX tests separately check readiness, replay,
fractional/week boundaries, deadlines and sender faults. The documented
single-PRN revive dry run passed on 2026-09-22 without UHD.

No X300 revive RF, live-sky mixture, first-RF-sample measurement or independent
carrier-coherence result has been established in the present qualification.
Those are tracked in the [current plan](x300-live-sky-mixtracking-research-plan.md).
