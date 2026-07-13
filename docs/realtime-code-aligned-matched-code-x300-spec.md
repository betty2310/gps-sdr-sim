# Selective-PRN Matched-Code Jammer-Only X300 Specification

**Status:** Software implemented; continuous controlled-hardware acceptance pending

**Date:** 2026-07-13

**Version:** V2

**Scope:** Continuous, simulator-aligned GPS L1 C/A matched-code interference
through one USRP X300/X310 TX channel in an authorized conducted path or
verified shielded enclosure

**Related:** [researcher/operator guide](realtime-code-aligned-matched-code-x300.md),
[V1 offline matched-code dataset specification](code-aligned-matched-code-dataset-spec.md),
[X300 timing alignment plan](x300-timing-alignment-plan.md), and
[jammer architecture](jamming-mitigation-architecture.md)

## Decision Summary

Matched-code mode in `x300tx` is a jammer-only transmitter.

- `--matched-code-target-prns` is the sole selector for transmitted jammer
  components.
- No simulator-generated clean GPS IQ reaches the RF output.
- Internal clean-channel state remains the alignment authority for code phase,
  carrier Doppler, and code-rate Doppler.
- The jammer is active from sample zero and continues until `SIGINT` or
  `SIGTERM`.
- The mode has no finite duration, onset, offset, or envelope ramp controls.
- `-P` is rejected because it is a clean-constellation control and would be a
  misleading jammer selector.
- `--matched-code-amplitude` replaces the composite-only digital J/S control.

The previous finite clean-plus-jammer composite meaning of V2 is superseded by
this specification.

## Problem Statement

The researcher needs to transmit navigation-data-free, matched-code
interference for a declared subset of GPS PRNs. For example:

```text
--matched-code-target-prns 1,3
```

means that the RF stream contains matched-code components for PRNs 1 and 3
only. It must not mean “render a partial clean constellation,” and it must not
also transmit clean GPS for PRNs 1 and 3 or any other PRN.

The source still needs per-PRN code phase and Doppler. This synthetic V2 obtains
those values from the simulator's internal channel model, based on the frozen
RINEX file, reference position, scenario start, and optional synthetic
satellite configuration. That clean simulator is an internal state machine,
not a desired-signal RF source.

This is not alignment to authentic live sky. Live-sky alignment would require
real-time or near-real-time tracking estimates from an authentic receiver path
and is a different architecture.

## Architecture

```text
RINEX + position + GPS time + optional -S
                    |
                    v
          internal clean-channel state
             |                  |
             |                  +--> clean IQ scratch --> discard
             v
 selected PRN code phase + Doppler
             |
             v
 navigation-data-free matched-code source
             |
             v
          SC16 quantization
             |
             v
       one X300 TX channel
```

`renderCleanEpochWide` remains in the path only because it advances simulator
navigation, code, and carrier state. It writes to a scratch buffer. The RF
adapter receives a separate SC16 buffer produced directly by
`matched_code_source_render_sc16`.

This boundary is intentional: adding, copying, or passing the clean scratch
buffer into the UHD queue is a contract violation.

## Command Contract

Matched-code jammer-only mode is enabled by:

```text
--matched-code-target-prns <ordered-list>
```

It also requires:

- `--matched-code-amplitude <value>` in `(0, 1]`;
- `--matched-code-phase-seed <unsigned-integer>`;
- `--manifest <path>`; and
- a RINEX file and alignment scenario usable for every requested PRN.

Live transmission additionally requires the existing controlled-RF gates:

- explicit `--addr`, `--channel`, `--antenna`, and `--gain`;
- `--calibration-id`;
- `--confirm-controlled-rf`; and
- either the calibrated Trimble start path or explicit GPS week/TOW with an
  approved external/GPSDO time source.

The following options are incompatible with matched-code jammer-only mode:

- `-P`, because it selects clean constellation channels rather than jammer
  components;
- `-d`, because the stream is operator-stopped and continuous;
- `-n`, because accepted live runs use the calibrated timed-start path;
- legacy `-A`, `-J`, and `-G` attack controls;
- `--matched-code-js-db`, because no clean signal is transmitted as a J/S
  reference; and
- `--matched-code-onset`, `--matched-code-offset`, and
  `--matched-code-ramp`, because the source is active for the full stream.

`-S` may define synthetic satellites needed by the internal alignment model.
It does not select jammer components. A PRN listed by `-S` but absent from
`--matched-code-target-prns` is not synthesized by the matched-code source.

## Continuous Activation Contract

The first matched-code sample is sample zero. Every subsequent sample remains
active until the operator sends `SIGINT` or `SIGTERM`.

There is no scheduled clean prefix, jammer window, recovery interval, or finite
planned sample count:

```text
sample 0 ----------------------------------------------------> operator stop
         matched-code jammer active for the entire stream
```

A clean operator stop is a successful terminal condition. The live manifest
uses `status: "stopped"`, `exit_status: 0`, and
`measurements.operator_stopped: true` when the end-of-burst is sent and all UHD
fault counters and clipping counts are zero.

Because frames are prebuffered, rendered and quantized counts may be greater
than sent counts at operator stop. This is expected. Sent samples must be
positive and may never exceed quantized samples.

## Alignment Contract

At every 100 ms epoch boundary, the internal simulator provides one state per
requested target PRN:

- first-sample C/A code phase in chips;
- carrier Doppler in hertz;
- code rate in chips per second; and
- internal clean gain for state validation and trajectory provenance.

The matched-code source uses the target PRN's C/A sequence, a constant positive
data symbol, and an independent carrier phase derived from the phase seed and
PRN. It does not copy LNAV data or clean carrier phase and must be described as
matched-code interference, not spoofing.

The target list is ordered, duplicate-free, and restricted to GPS PRNs 1
through 32. All requested targets are validated during the 100 ms startup
window. During a live run, target usability is checked again at each epoch; an
unavailable target ends the stream as an error rather than silently changing
the jammer set.

## Amplitude and Headroom Contract

`--matched-code-amplitude` is a normalized digital source setting. It uses the
same shared-source meaning as `matchedgen`: the sum uses equal component weights
and `1/sqrt(N)` target-count normalization, where `N` is the number of selected
PRNs.

It is not:

- a transmitted clean-to-jammer J/S value;
- X300 RF output power;
- receiver-input jammer power; or
- a substitute for conducted-path calibration.

Before arming, `x300tx` computes the conservative component bound:

```text
predicted peak full scale = amplitude * sqrt(target count)
```

The command is rejected unless this bound leaves at least 1 dB of digital
headroom. The X300 RF gain remains a separate explicit `--gain` setting. The
manifest records requested amplitude, predicted peak and headroom, measured
jammer RMS and peak, and clipping count.

## Dry-Run Contract

`--dry-run` never discovers, opens, configures, or transmits through UHD. It
renders a fixed 100 ms validation window beginning at sample zero.

The finite validation window exists only to prove startup allocation,
trajectory ownership, deterministic source output, amplitude/headroom, and SC16
quantization. It does not make the live stream finite.

The CLI test independently regenerates the dry-run bytes with `matchedgen` from
the emitted trajectory and compares a deterministic FNV-1a digest. This proves
that the RF-adapter bytes are the selected navigation-data-free matched-code
source, without additive clean simulator IQ. FNV-1a here is a regression digest,
not a cryptographic provenance hash.

## Manifest Contract

The schema is `gps-sdr-sim.x300tx-matched-code.v2`.

The manifest reports at least:

- `rf_output.contains: "matched_code_interference_only"`;
- `rf_output.clean_gps_transmitted: false`;
- requested and selected target PRNs;
- deterministic phase seed and initial per-target phases;
- requested amplitude, predicted peak, predicted headroom, and jammer IQ digest;
- `sample_contract.continuous: true` and `planned_samples: null`;
- dry-run validation, internal alignment, rendered jammer, quantized jammer, and
  sent jammer sample counts;
- internal alignment source and trajectory provenance;
- requested and actual sample rates;
- X300 identity, RF path, gain, clock, and timing settings;
- measured jammer RMS and peak;
- clipping, underflow, sequence-error, and time-error counters; and
- terminal status, failure reason, exit status, and operator-stop state.

The trajectory uses the existing target-state schema. For a live run it is
written continuously and finalized and checksummed after operator stop.

## Safety and Failure Behavior

The mode is for an authorized conducted path or verified shielded enclosure
only. It must fail closed for:

- missing or invalid targets, amplitude, seed, manifest, or alignment inputs;
- incompatible `-P`, duration, stream-now, envelope, J/S, legacy attack, or live
  RTCM options;
- target allocation or usability loss;
- unresolved or colliding artifact paths;
- ambiguous, unavailable, or non-X300/X310 hardware;
- invalid channel, antenna, gain, rate, or timing configuration;
- stale timed start;
- zero-sample sends;
- any clipping, underflow, sequence error, or time error; or
- failure to finalize the trajectory or manifest.

Dry-run failures occur before UHD is opened. Default automated tests never
perform a live transmission.

## Acceptance

### Software acceptance

Required checks:

1. `make test-x300tx-matched` passes.
2. The dry-run manifest uses V2 jammer-only terminology and continuous counts.
3. Dry-run SC16 bytes match `matchedgen` from the emitted trajectory.
4. `-P`, `-d`, `-n`, legacy J/S, and matched-code timing controls fail before
   UHD configuration.
5. Duplicate, invalid, and unavailable target PRNs fail closed.
6. `matchedgen` remains bit-identical for the existing offline fixture.
7. The repository test suite and `git diff --check` pass.

### Controlled-hardware acceptance

The earlier finite composite burst does not accept this revised contract. One
new continuous jammer-only run is required in the authorized setup.

The operator must start the calibrated run, observe the planned experiment
interval, and stop it with `SIGINT`. Acceptance requires:

- final `status: "stopped"` and `exit_status: 0`;
- positive sent jammer sample count;
- internal alignment count equals rendered and quantized jammer counts;
- sent count does not exceed quantized count;
- zero clipping, underflows, sequence errors, and time errors;
- finalized trajectory with a matching SHA-256; and
- no claim beyond transmitter-side execution without separate RF capture or
  receiver evidence.

## Research Boundary

Software or transmitter acceptance does not prove TX-port power, receiver-input
power, authentic-signal alignment, RF waveform fidelity, propagation behavior,
receiver acquisition/tracking, `svUsed`, pseudorange residual, PVT, or
mitigation performance. Those claims require calibrated RF and receiver-side
evidence in a separate experiment.
