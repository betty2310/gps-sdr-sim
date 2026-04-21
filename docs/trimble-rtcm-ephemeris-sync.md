# Trimble RTCM Ephemeris Sync Plan

## Goal

Allow `bladetx` and `x300tx` to fetch live GPS broadcast ephemeris from a Trimble receiver over a TCP or NTRIP `RTCM_V3` source, so transmit commands do not require `-e <rinex_nav>`.

This plan is specifically for the live transmit path that already uses Trimble time-tag on TCP port `5017`. The first implementation target is GPS L1 C/A only.

## Scope

- In scope:
  - `player/bladetx.cpp`
  - `player/x300tx.cpp`
  - new RTCM v3 parser/helper code shared by both players
  - loading GPS ephemeris from RTCM message `1019`
  - making `-e` optional when Trimble RTCM ephemeris mode is enabled
  - startup warm-up to collect enough ephemeris before TX begins
  - optional runtime refresh when newer `1019` messages arrive during long runs

- Out of scope for the first pass:
  - standalone `gps-sdr-sim` without SDR player
  - full multi-GNSS ephemeris decode (`1020`, `1042`, `1044`, `1045`, `1046`)
  - SSR/correction decoding
  - generating a RINEX file on disk as an intermediate artifact
  - replacing Trimble time-tag on port `5017`

## Why RTCM First

RTCM is the simplest live source for this use case because GPS broadcast ephemeris is explicitly carried in message `1019`, while the current code only needs a populated `ephem_t` set near scenario start time. The simulator already knows how to propagate and transmit from `ephem_t`; the missing piece is a live source that fills those structures without `readRinexNavAll()`.

## Current Constraints In The Code

- `bladetx` and `x300tx` currently require `-e <rinex_nav>` and fail early if it is missing.
- Active ephemeris selection is built around arrays filled by `readRinexNavAll()`.
- The core simulator can operate without valid iono/UTC header data, because `eph2sbf()` already falls back to page 25 when `ionoutc.vflg == FALSE`.
- Synthetic satellite generation and donor fitting only need a valid `ephem_t` set for the current epoch; they do not require a historical RINEX file.

## Proposed User Experience

Add a live Trimble RTCM ephemeris mode alongside the existing Trimble time-tag mode.

Example target command:

```bash
bladetx \
  -l 21.0047844,105.8460541,22 \
  -P 7,8 \
  -S 7:60/55,8:105/45 \
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

### CLI Rules

- `-e` remains supported.
- `-e` becomes optional when `--trimble-rtcm-host` and `--trimble-rtcm-port` are provided.
- If both `-e` and live RTCM ephemeris are provided in the first pass, prefer live RTCM and ignore the RINEX file with a warning. This keeps behavior simple and avoids hybrid merge logic.
- `--trimble-rtcm-port` defaults to `5018` if `--trimble-rtcm-host` is set.

### New Options

- `--trimble-rtcm-host <host>`
- `--trimble-rtcm-port <port>` default `5018`
- `--trimble-rtcm-mount <name>` optional NTRIP mount point
- `--trimble-rtcm-user <user[:pass]>` optional NTRIP credentials
- `--trimble-rtcm-timeout-ms <ms>` default aligned with other Trimble TCP timeouts
- `--trimble-rtcm-warmup-sec <sec>` default `60`
- `--trimble-rtcm-min-prns <n>` default `8`

`--trimble-rtcm-warmup-sec` and `--trimble-rtcm-min-prns` make startup deterministic: do not begin TX until enough `1019` messages have been collected, or fail clearly on timeout.

## Architecture

### 1. Add A Small RTCM v3 GPS Ephemeris Decoder

Create a shared helper for the two player binaries.

Proposed files:

- `player/rtcm3_nav.hpp`
- `player/rtcm3_nav.cpp`

Responsibilities:

- read RTCM v3 frames from a TCP byte stream
- validate frame CRC24Q
- extract 12-bit message type
- decode GPS ephemeris message `1019`
- map decoded fields into `ephem_t`
- maintain a freshest-per-PRN cache keyed by GPS PRN

The helper should not try to be a general RTCM library. First-pass support for `1019` only keeps the implementation small and directly tied to the simulator’s needs.

### 2. Build A Live Ephemeris Store From RTCM

Add a runtime cache object in the player layer:

- socket handle
- receive buffer
- `ephem_t gps_rtcm_eph[MAX_SAT]`
- valid flags per PRN
- latest update time per PRN
- count of distinct valid GPS PRNs seen during warm-up

This store replaces the `readRinexNavAll()` source only for live Trimble RTCM mode.

### 3. Convert RTCM Cache Into The Existing Active Set Shape

Do not rewrite the downstream signal generation pipeline.

Instead:

- construct one in-memory ephemeris set `eph_live[1][MAX_SAT]`
- set `neph = 1`
- set `ieph = 0`
- populate `eph_live[0][sv]` from the live RTCM cache

This keeps the existing `satpos()`, `computeRange()`, `allocateChannel()`, `eph2sbf()`, synthetic overlay, and nav message generation unchanged.

### 4. Handle Missing Iono/UTC Cleanly

RTCM `1019` does not provide the same iono/UTC header content that comes from RINEX NAV headers.

In live RTCM mode:

- initialize `ionoutc` to zero
- set `ionoutc.vflg = FALSE`
- set `ionoutc.dtls` from `--trimble-leap-sec`

This is acceptable because:

- the existing code already handles `ionoutc.vflg == FALSE` when encoding nav subframes
- the current research focus is matching live broadcast ephemeris, not perfect almanac/page-18 reproduction

### 5. Refresh During Long Runs

After startup warm-up, keep the RTCM socket open.

During the normal 100 ms epoch loop:

- non-blockingly drain any available RTCM bytes
- decode additional `1019` frames
- replace per-PRN cached ephemerides when newer data arrives
- rebuild subframes for channels whose PRN ephemeris changed

This avoids thread introduction in the first pass and fits the current player control flow.

## Implementation Tasks

## Task 1: Add RTCM Decode Support

- [ ] Create `player/rtcm3_nav.hpp` with a small public API for frame ingestion and cache access.
- [ ] Create `player/rtcm3_nav.cpp` with:
  - RTCM frame scanner
  - CRC24Q implementation
  - bit-field extraction helpers
  - message-type dispatch
  - GPS `1019` decoder
- [ ] Add a conversion function from decoded `1019` fields to `ephem_t`.
- [ ] Verify all required `ephem_t` fields are filled consistently with the existing `readRinexNavAll()` post-processing expectations:
  - `vflg`
  - `toe`, `toc`
  - `af0`, `af1`, `af2`
  - `iode`, `iodc`
  - `crs`, `deltan`, `m0`
  - `cuc`, `ecc`, `cus`, `sqrta`
  - `cic`, `omg0`, `cis`, `inc0`
  - `crc`, `aop`, `omgdot`, `idot`
  - `tgd`, `svhlth`
  - derived fields `A`, `n`, `sq1e2`, `omgkdot`

## Task 2: Integrate Live RTCM Into `bladetx`

- [ ] Add CLI parsing for `--trimble-rtcm-host`, `--trimble-rtcm-port`, `--trimble-rtcm-timeout-ms`, `--trimble-rtcm-warmup-sec`, and `--trimble-rtcm-min-prns` in [player/bladetx.cpp](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/player/bladetx.cpp).
- [ ] Make `-e` optional when live RTCM ephemeris mode is enabled.
- [ ] Add startup validation so the user gets a clear error if neither `-e` nor Trimble RTCM ephemeris mode is configured.
- [ ] Connect to the Trimble RTCM socket before ephemeris selection.
- [ ] Warm up the live cache until enough distinct GPS PRNs have been received or timeout occurs.
- [ ] Build the initial `eph_live[0]` set and continue through the existing initialization path.

## Task 3: Integrate Live RTCM Into `x300tx`

- [ ] Mirror the same CLI and runtime behavior in [player/x300tx.cpp](file:///Users/betty/Developer/gnss-tools/gps-sdr-sim/player/x300tx.cpp).
- [ ] Reuse the same `player/rtcm3_nav.*` helper.
- [ ] Keep usage text and option semantics aligned across both players.

## Task 4: Runtime Refresh

- [ ] Add a non-blocking “drain RTCM socket” call to the player refresh path.
- [ ] When a PRN ephemeris changes, regenerate that PRN’s subframes through the existing `eph2sbf()` path.
- [ ] Preserve synthetic overlay behavior by applying synthetic ephemerides after live RTCM real ephemerides are refreshed.
- [ ] Ensure channel allocation never sees half-updated state.

## Task 5: Tests

### Unit Tests

- [ ] Add a small C++ test program or lightweight parser self-test executable under `tests/` for:
  - CRC24Q correctness
  - RTCM frame boundary handling with split TCP reads
  - GPS `1019` decode into expected fields
  - `ephem_t` derived field population

### Integration Tests

- [ ] Add a Python integration test that starts a fake TCP server for RTCM `1019` frames and verifies the player can initialize without `-e`.
- [ ] Add a second fake server for Trimble time-tag if needed, or split the ephemeris path test so it exercises only the new RTCM loading helper.
- [ ] Keep tests hardware-free; they should not require bladeRF/UHD access.

## Task 6: Documentation

- [ ] Update player usage text to show the new Trimble RTCM options.
- [ ] Add a short doc describing:
  - required RTCM message type `1019`
  - recommended RTCM source configuration; if raw TCP `5018` only carries MSM
    and station messages, use an NTRIP source that includes `1019`
  - startup warm-up expectations
  - why `-e` is no longer required in this mode
- [ ] Add example commands using both Trimble time-tag on `5017` and Trimble RTCM ephemeris on `5018`.

## Clone Mode

`docs/synth-clone-mode.md` consumes this live ephemeris cache. In the measured
lab setup on 2026-04-16, the raw `5018` stream carried `1077`/`1006` but not
`1019`, while the NTRIP endpoint on `5005` with mount `NAVIS` did carry
repeated GPS `1019` messages. Clone mode therefore needs whichever source in
your environment actually provides `1019`.

## Startup Policy

Default startup policy for the first pass:

- connect to RTCM stream
- collect GPS `1019` messages for up to `60 s`
- succeed once at least `8` unique GPS PRNs are loaded
- fail with a clear error if fewer than `8` GPS ephemerides are available by timeout

Reasoning:

- waiting for all 32 GPS PRNs is unnecessary and can block too long
- for a 30-minute local transmit run, fresh ephemerides for the visible sky and nearby donors are what matter
- the threshold should be configurable for debugging

## Failure Modes To Handle Explicitly

- RTCM TCP connects but no valid `1019` messages arrive
- RTCM stream contains only MSM/observables and no ephemeris
- malformed RTCM frames or CRC failures
- insufficient PRNs by warm-up timeout
- live ephemeris available but requested synthetic donor PRNs are still absent
- both `-e` and live RTCM requested at once

Each of these should produce a concrete message telling the user what to change on the Trimble output configuration.

## Verification Checklist

- [ ] `bladetx` starts without `-e` when Trimble RTCM mode is configured.
- [ ] `x300tx` starts without `-e` when Trimble RTCM mode is configured.
- [ ] GPS channels allocate and nav subframes encode from live `1019` ephemerides.
- [ ] Synthetic PRN donor selection uses live real ephemerides rather than stale hourly RINEX.
- [ ] A 30-minute run can proceed from one fresh startup snapshot without ephemeris-related start failures.
- [ ] Long runs survive live `1019` refreshes without channel corruption.

## Recommended Order Of Work

1. Implement a minimal `1019 -> ephem_t` decoder and test it offline.
2. Wire it into `bladetx` startup only, with no runtime refresh.
3. Add hardware-free integration coverage with a fake TCP RTCM server.
4. Mirror the same path into `x300tx`.
5. Add non-blocking runtime refresh.
6. Update docs and example commands.

## Notes For The First Pass

- GPS only is enough for the current simulator behavior and the current spoofing research.
- Do not build a full general-purpose RTCM stack.
- Do not mix live RTCM ephemeris with RINEX in the same first-pass code path.
- Keep the downstream signal-generation pipeline unchanged; only replace the ephemeris source.
