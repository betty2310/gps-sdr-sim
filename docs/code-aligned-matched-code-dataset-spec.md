# Code-Aligned Matched-Code Jamming Dataset Specification

**Status:** Proposed

**Date:** 2026-07-13

**Scope:** Offline GPS L1 C/A dataset generation and receiver evaluation

## Problem Statement

The researcher can currently create reproducible offline GPS L1 C/A datasets
with CW, narrowband-noise, wideband-noise, chirp, and pulsed-CW interference,
but cannot create a defensible code-aware matched-code dataset. The legacy
selective-noise branch is not suitable: multiplying independent per-sample
white noise by a C/A code does not produce the intended PRN-selective
correlation structure, and it is coupled to the clean-signal simulator rather
than represented as an independent additive source.

The researcher wants to enter a set of GPS PRNs that are currently observed by
the local Trimble receiver and create a synthetic dataset in which:

- the clean GPS constellation represents the live sky at the Trimble location
  and RTCM epoch;
- the target PRN set is verified against the live RTCM stream and then frozen;
- each jammer component carries the target PRN's C/A code without a navigation
  message;
- the jammer component stays aligned with the corresponding clean signal's
  initial code phase, carrier Doppler, and code-rate Doppler;
- alignment is exact and deterministic because clean and jammer generation use
  the same simulator reference trajectory;
- the clean signal and jammer remain separate additive sources in the existing
  offline mixer architecture; and
- the resulting fixtures include enough provenance and receiver evidence for
  mitigation research and publication.

The current RTCM inspection and navigation helpers already connect to TCP or
NTRIP, validate RTCM frames, decode GPS ephemeris message 1019, and identify
PRNs in observation message 1004. They do not yet create a reproducible
live-sky scenario artifact or expose all state required by the offline dataset
workflow. Full RTCM pseudorange reconstruction is not required for the chosen
design because RTCM anchors the live-sky scenario while the simulator is the
alignment authority.

## Solution

Add an offline `matched-code` dataset source that extends the existing clean
GPS, independent jammer, offline mixer, and GNSS-SDR workflow.

Before waveform generation, the workflow captures or replays a bounded RTCM
snapshot. A live capture collects ten valid GPS observation epochs and healthy
GPS ephemeris for every user-requested target PRN, subject to a sixty-second
timeout. A PRN is stable when it appears in at least eight of those ten epochs.
The snapshot must contain at least eight stable, healthy GPS PRNs in total, and
all requested PRNs must be stable, healthy, and usable by the synthetic clean
scenario; otherwise creation fails without producing a partial dataset. The
target set is then frozen for the entire dataset.

The snapshot records the RTCM byte archive, decoded observation history,
station reference position, reference GPS week and TOW, freshest healthy
ephemeris per PRN, selection policy, requested PRNs, and selected PRNs. It is
immutable, checksummed, and contains no NTRIP password. A saved snapshot can be
replayed without network access to reproduce the same scenario.

The clean generator consumes the frozen scenario, creates the synthetic GPS L1
C/A signal at the Trimble antenna reference position and exact GPS epoch, and
exports a reference trajectory for every target PRN. The trajectory defines
the code phase, carrier Doppler, and code rate at sample-accurate epoch
boundaries. It is generated in the same run as the clean IQ and is the sole
authority for matched-code alignment.

The independent jammer renderer creates one navigation-data-free C/A component
per target PRN. Every component follows its clean-signal reference trajectory,
uses a constant positive data symbol, and has a deterministic independent
initial carrier phase. Equal component RMS weights are used by default and the
composite is normalized so requested total jammer RMS and J/S do not change
when the number of target PRNs changes. The current onset, offset, ramp,
quantization, mixing, AWGN, and common-gain contracts remain in force.

The workflow validates the output at the highest existing seam: one dataset
creation command must capture or replay the live-sky scenario, generate clean
and matched-code IQ, mix all requested J/S fixtures, run numerical and
code-domain acceptance, optionally run GNSS-SDR, and publish a complete dataset
index. Cold-start-under-jammer and already-tracking-before-jammer receiver runs
are reported separately because they test different receiver behavior.

## User Stories

1. As a GNSS interference researcher, I want to enter specific GPS PRNs, so that I can control which live-sky signals are represented by matched-code interference.

2. As a GNSS interference researcher, I want requested PRNs checked against the Trimble observation stream, so that I do not accidentally target satellites that are not currently observed.

3. As a GNSS interference researcher, I want the target set frozen at scenario creation, so that every fixture in a campaign uses the same experimental condition.

4. As a GNSS interference researcher, I want creation to fail when a requested PRN is unstable or unhealthy, so that missing targets are never silently removed from the experiment.

5. As a GNSS interference researcher, I want live GPS ephemeris captured with the observed PRNs, so that the synthetic clean sky represents the same epoch and location.

6. As a GNSS interference researcher, I want the Trimble antenna reference position recorded, so that the scenario geometry has an explicit spatial reference.

7. As a GNSS interference researcher, I want the exact GPS week and TOW recorded, so that the scenario is not reduced to an ambiguous wall-clock timestamp.

8. As a GNSS interference researcher, I want raw RTCM bytes archived, so that the decoded scenario can be audited against its original source.

9. As a GNSS interference researcher, I want decoded RTCM state stored in a versioned snapshot, so that I can inspect and replay the scenario without reconnecting to the caster.

10. As a GNSS interference researcher, I want NTRIP credentials excluded from all artifacts, so that datasets can be shared safely.

11. As a GNSS interference researcher, I want the clean signal generated from the frozen live-sky scenario, so that target PRNs exist in the desired signal being evaluated.

12. As a GNSS interference researcher, I want every requested target guaranteed a clean simulator channel, so that channel-allocation limits cannot invalidate the experiment silently.

13. As a GNSS interference researcher, I want the clean generator to export its target-channel trajectory, so that jammer alignment does not depend on reconstructing simulator state later.

14. As a GNSS interference researcher, I want matched-code components aligned in initial code phase, so that they occupy the intended target code cell.

15. As a GNSS interference researcher, I want matched-code components aligned in carrier Doppler, so that they remain in the target receiver's frequency cell.

16. As a GNSS interference researcher, I want matched-code components aligned in code-rate Doppler, so that code alignment does not drift during the jammer interval.

17. As a GNSS interference researcher, I want the jammer to omit navigation data, so that the waveform remains matched-code interference rather than a spoofing signal.

18. As a GNSS interference researcher, I want jammer carrier phase independently initialized and recorded, so that phase-sensitive results are reproducible without claiming carrier-phase synchronization.

19. As a GNSS interference researcher, I want several deterministic carrier-phase seeds in publication campaigns, so that mitigation conclusions are not based on one favorable coherent phase relationship.

20. As a GNSS interference researcher, I want equal default power per target component, so that the configuration is understandable and does not privilege a PRN implicitly.

21. As a GNSS interference researcher, I want total jammer RMS normalized across PRN counts, so that a J/S sweep remains comparable when the target set changes.

22. As a GNSS interference researcher, I want per-target component power and derived J/S reported, so that unequal clean satellite powers are visible in the analysis.

23. As a GNSS interference researcher, I want the existing clean, active, and recovery intervals preserved, so that matched-code results are comparable with the other jammer families.

24. As a GNSS interference researcher, I want the fast profile to support matched-code generation, so that I can iterate without materializing a full canonical campaign.

25. As a GNSS interference researcher, I want the verification profile to exercise the full receiver path, so that code-domain correctness is backed by receiver-side evidence.

26. As a GNSS interference researcher, I want the canonical profile to sweep J/S and carrier-phase seeds, so that I can estimate degradation thresholds without phase cherry-picking.

27. As a GNSS interference researcher, I want a clean control generated from the identical scenario artifact, so that differences cannot be attributed to geometry or ephemeris drift.

28. As a GNSS interference researcher, I want code-domain correlation reports for target and non-target PRNs, so that matched-code structure is proven rather than inferred from a spectrum plot.

29. As a GNSS interference researcher, I want cold-start-under-jammer results, so that I can measure acquisition denial by the matched-code source.

30. As a GNSS interference researcher, I want already-tracking-before-jammer results, so that I can measure tracking, navigation-data, and recovery behavior separately from cold start.

31. As a GNSS interference researcher, I want per-PRN C/N0, acquisition, tracking, prompt-IQ, and navigation-use evidence, so that target and non-target effects can be compared.

32. As a GNSS interference researcher, I want PVT availability, `svUsed`, and pseudorange-residual evidence, so that mitigation success is evaluated at the navigation layer as well as the correlator layer.

33. As a dataset consumer, I want versioned manifests and hashes for every source and fixture, so that I can verify provenance before analysis.

34. As a dataset consumer, I want the alignment reference artifact identified by hash, so that I know exactly which trajectory produced the jammer.

35. As a paper reviewer, I want matched-code, matched-spectrum, and spoofing behavior distinguished explicitly, so that the dataset's threat-model claims are precise.

36. As a paper reviewer, I want failed gates to stop publication of a fixture, so that a successful build cannot be mistaken for a valid research waveform.

37. As a contributor, I want the simulator and jammer to share one C/A-code implementation, so that duplicated PRN logic cannot diverge.

38. As a contributor, I want rendering to be invariant to buffer boundaries, so that offline output does not depend on machine-specific chunking.

39. As a contributor, I want malformed RTCM, stale ephemeris, week ambiguity, and incomplete snapshots rejected clearly, so that timing errors are diagnosed at the source.

40. As a contributor, I want existing jammer types and manifests to remain readable, so that adding matched-code support does not invalidate earlier datasets.

41. As a laboratory operator, I want this feature restricted to file-based generation, so that creating a dataset does not configure or activate an RF transmitter.

42. As a mitigation researcher, I want the legacy selective-noise branch retained only as a negative control, so that the new correlation acceptance can demonstrate the difference between noise-like and code-bearing interference.

## Implementation Decisions

### Architecture and ownership

- The feature remains in the independent-source architecture: live-sky scenario
  acquisition, clean GPS generation, matched-code jammer generation, offline
  mixing/front-end modeling, and receiver evaluation remain separate stages.
- RTCM network ingestion must not occur inside the sample renderer. The network
  stage produces an immutable scenario artifact; all waveform stages consume
  frozen inputs.
- The simulator is the alignment authority. RTCM determines the live-sky PRN
  set, position, epoch, and ephemeris, but RTCM pseudorange is not used to
  reconstruct jammer code phase in this version.
- The clean generator exports a target reference trajectory in the same run
  that emits clean IQ. The jammer consumes that trajectory rather than
  independently recomputing satellite range or Doppler.
- The user-facing source name is `matched-code`. The internal source taxonomy
  may use `gnss_ca`, but manifests must expose the matched-code interference
  mode explicitly.
- The matched-code source is added to offline generation only. Hardware
  backends must reject it as unsupported in this version even if they share the
  generic renderer library.

### RTCM capture and scenario freezing

- Live and replay modes are both required. Live mode connects to a TCP or NTRIP
  RTCM source; replay mode consumes an existing scenario snapshot and its raw
  RTCM archive.
- A matched-code run requires an explicit, ordered, duplicate-free target PRN
  list. Automatic target selection is not part of this version.
- Live warm-up collects ten valid RTCM 1004 observation epochs. A target is
  stable when present in at least eight epochs.
- Warm-up also requires a current, healthy RTCM 1019 ephemeris for every target
  and at least eight stable, healthy GPS PRNs in total. It also requires a
  station reference position from RTCM or an explicitly supplied,
  provenance-recorded override. Every ephemeris received during the accepted
  capture is retained, not only ephemeris for target PRNs.
- The default live-capture deadline is sixty seconds. Timeout, connection loss,
  CRC failure above the configured tolerance, missing position, ambiguous GPS
  week, missing target ephemeris, unhealthy target ephemeris, or an unstable
  target causes a hard failure.
- The default accepted CRC-error count for a research snapshot is zero. A
  nonzero tolerance is an explicit override and is recorded in provenance;
  corrupt frames are always discarded and never decoded.
- The final valid 1004 epoch in the warm-up window becomes the scenario
  reference TOW. The corresponding full GPS week is resolved from ephemeris and
  checked against the capture time. A week-resolution inconsistency is fatal.
- The selected target set exactly equals the requested set after validation.
  The workflow must never silently use an intersection or dynamically add and
  remove targets after generation begins.
- The raw RTCM frame stream covering warm-up is archived byte-for-byte. The
  decoded snapshot references its checksum and records frame counts, CRC
  failures, and dropped bytes.
- Snapshot metadata includes schema version, capture time, station identifier,
  station ECEF and geodetic position, GPS week/TOW, observation epochs, freshest
  ephemeris per PRN, requested targets, selected targets, stability statistics,
  selection thresholds, source endpoint metadata, and tool version.
- Authentication secrets are accepted only through a secret-bearing runtime
  mechanism such as an environment variable or protected credential input.
  Passwords and authorization headers must not appear in commands recorded by
  manifests, logs, raw archives, decoded snapshots, or error messages.

### Synthetic clean scenario

- The clean generator accepts the immutable live-sky scenario artifact as an
  alternative to a RINEX navigation file.
- The clean scenario begins at the exact GPS week/TOW stored in the snapshot
  and uses the snapshot station reference position. Subsecond precision must be
  retained end to end.
- The ephemeris set is populated from the snapshot's RTCM 1019 records using
  the established ephemeris representation and propagation logic.
- The normal visibility and elevation-mask rules continue to define the clean
  constellation, but every selected target is a required channel. If a target
  is not geometrically usable or cannot be allocated within the simulator's
  channel limit, generation fails. Remaining channels are filled by the normal
  deterministic allocation policy.
- Clean control and all jammer fixtures in a campaign reuse the same clean IQ,
  scenario snapshot, and target trajectory.
- The clean generator produces a versioned trajectory artifact for selected
  targets. Each record identifies the sample offset and the code phase in
  chips, carrier Doppler in hertz, code rate in chips per second, and clean
  component gain/power at the first sample governed by that record.
- Trajectory records use the clean generator's sample-generation epoch cadence,
  currently 100 milliseconds. Within each record interval, both clean and
  jammer advance their numerically controlled oscillators from the same
  boundary state and rates. Boundary ownership and first-sample semantics are
  explicit in the schema to prevent one-sample alignment errors.
- The trajectory is checksummed and linked from clean, jammer, fixture, and
  dataset manifests.

### Shared GPS L1 C/A reference

- C/A-code generation becomes a shared primitive used by both the clean
  simulator and the independent jammer renderer.
- The shared primitive supports GPS PRNs 1 through 32 and returns the same
  bipolar chip convention to both consumers.
- Existing clean-signal output must remain bit-compatible after extraction of
  the shared primitive.
- The implementation must support fractional chip phase at arbitrary supported
  sample rates; it must not require an integer number of samples per chip.

### Matched-code waveform

- A source contains one component per frozen target PRN.
- Each component consists of the target PRN's repeating GPS L1 C/A code, a
  constant positive data symbol, a complex carrier, and the common global
  onset/offset envelope.
- No LNAV words, parity, TLM, HOW, ephemeris bits, or other navigation message
  are modulated onto jammer components.
- The component's initial code phase, carrier Doppler, and code rate come from
  the target trajectory and remain locked to that trajectory for the complete
  fixture.
- Initial carrier phase is not copied from the clean signal. It is generated
  deterministically per component from the configured phase seed and recorded
  in the source manifest.
- Default component weights are equal in RMS voltage. Composite normalization
  uses the actual sum of component powers so source plateau RMS is independent
  of target count.
- The requested J/S continues to mean total jammer plateau RMS power relative
  to total clean plateau RMS power under the established offline mixer
  contract. The analysis additionally reports component power and derived
  component-to-corresponding-clean-PRN power ratio.
- Existing raised-cosine global ramps, zero output outside the active interval,
  SC16 sample contract, clipping accounting, deterministic quantization, and
  source metrics apply unchanged.
- Renderer state includes every component's code NCO, carrier NCO, trajectory
  cursor, and phase seed state so output is invariant to render buffer size.

### Dataset workflow and interfaces

- Matched-code is available in the fast, verification, and canonical profiles.
- Matched-code creation requires target PRNs plus either live RTCM connection
  parameters or a frozen RTCM scenario snapshot.
- Live connection inputs include host, port, optional mount point, username,
  protected password input, timeout, warm-up epoch count, and stability ratio.
  Defaults are sixty seconds, ten epochs, and an eighty-percent stability
  ratio.
- Replay mode performs the same validation over the archived observations and
  refuses a snapshot that does not meet the current requested-target policy.
- Fast mode creates a short clean control and a short matched-code fixture for
  development and structural acceptance.
- Verification mode uses the established long clean prefix and one bounded J/S
  condition, runs both code-domain validation and GNSS-SDR, and produces
  already-tracking receiver evidence.
- Canonical mode preserves the established coarse J/S sweep. Matched-code
  canonical campaigns use at least five deterministic carrier-phase seeds per
  J/S level unless the researcher explicitly selects a development-only
  single-seed run. Publication metadata labels single-seed output as
  insufficient for phase-robust claims.
- Cold-start evaluation begins receiver processing at the jammer-active
  interval without a prior clean acquisition window. Already-tracking
  evaluation begins at sample zero and retains the clean prefix. Both use the
  same underlying fixture where receiver input-offset support permits it.
- Dataset creation is transactional at the index level. A failed snapshot,
  waveform, correlation, mixer, or required receiver gate prevents the final
  dataset index from reporting the campaign as passed.

### Manifests and compatibility

- The jammer-source, fixture, and dataset-index schemas receive new versions
  for code-aware fields while readers continue to accept the previous schemas
  for the five existing source families.
- The source manifest records interference taxonomy, target PRNs, data-symbol
  policy, phase seed, initial carrier phases, component weights, component RMS,
  trajectory checksum, and per-component alignment parameters.
- The fixture manifest records scenario checksum, raw RTCM checksum, clean IQ
  checksum, jammer IQ checksum, trajectory checksum, mixer measurements,
  requested total J/S, measured total J/S, and derived per-target power ratios.
- The dataset index records the frozen target set, snapshot provenance, receiver
  run modes, seed dimension, J/S dimension, waveform reports, correlation
  reports, receiver reports, and first observed degradation thresholds.
- Existing CW, narrowband, wideband, chirp, and pulsed behavior is unchanged.
  Existing datasets remain analyzable.

### Failure behavior and safety

- All validation errors identify the failed PRN and failed condition where
  applicable, while redacting credentials.
- A partial or stale live-sky snapshot cannot fall back automatically to RINEX,
  wall-clock time, a smaller PRN set, or default location.
- A missing target trajectory, trajectory discontinuity, sample-count mismatch,
  or manifest-checksum mismatch is fatal before mixing.
- This workflow remains offline-only. It writes files and runs software
  receivers; it does not configure SDR hardware or provide an RF transmission
  path.

## Testing Decisions

### Primary behavioral seam

- The primary acceptance seam is the existing dataset creation command. A
  committed RTCM byte fixture representing 1004, 1006, and 1019 traffic is
  replayed through the fast profile. One command must produce the validated
  scenario snapshot, clean IQ, target trajectory, matched-code IQ, mixed clean
  and J/S fixtures, manifests, numerical reports, correlation reports, and a
  passing dataset index.
- Tests assert externally visible artifacts, schema fields, hashes, measured
  waveform properties, and receiver outcomes rather than private helper calls.
- Full verification-profile acceptance runs through GNSS-SDR and remains the
  highest confidence gate. It is allowed to be slower than the default unit
  suite but must be reproducible from committed or explicitly provisioned test
  inputs.

### RTCM contract seam

- A narrow RTCM decoder contract test is retained because malformed live input
  would otherwise be difficult to diagnose at the dataset seam.
- Recorded frames cover valid 1004 observation epochs, station position,
  healthy and unhealthy 1019 ephemeris, split socket reads, multiple frames in
  one read, CRC rejection, byte resynchronization, GPS week rollover, and
  connection truncation.
- Behavioral checks verify stable-target selection, exact requested-target
  failure, reference epoch choice, position extraction, ephemeris freshness,
  raw-archive checksum, replay equivalence, and credential redaction.

### Shared reference and renderer seam

- The existing C jammer-source test harness remains the renderer seam.
- C/A reference tests compare multiple PRNs against authoritative first-chip
  vectors and confirm the clean simulator remains bit-compatible.
- Renderer tests cover fractional samples per chip, trajectory boundary
  semantics, initial alignment, carrier-Doppler alignment, code-rate alignment,
  long-interval code drift, multiple PRNs, deterministic phase seeds, equal
  weights, composite normalization, ramps, clipping counters, and render-chunk
  invariance.
- The legacy selective-noise waveform is included as a negative control and
  must fail the matched-code correlation gate.

### Numerical and code-domain acceptance

- Existing gates remain mandatory: exact sample count, exact zero outside the
  envelope, plateau RMS within 0.1 dB, reported RMS within 0.1 dB, measured J/S
  within 0.1 dB, and zero unexpected clipping.
- Every target PRN must produce a dominant acquisition/correlation peak at its
  configured code-Doppler cell. The measured peak must be within 0.5 chip and
  25 Hz of the reference trajectory at the evaluated epoch.
- Target code alignment measured at the start and end of the active plateau
  must not drift by more than 0.5 chip relative to the clean reference.
- In a single-component acceptance fixture for each target, the selected-cell
  correlation peak must exceed the strongest unselected-PRN response by at
  least 18 dB. The actual multi-component composite is compared with its
  deterministic reference correlation map because its non-target floor depends
  on target count and cross-correlation addition.
- The report includes target and non-target correlation heatmaps, measured code
  offset, measured Doppler offset, selected-to-unselected margin, component
  RMS, and derived per-target J/S.
- Spectrum and spectrogram plots remain descriptive evidence; they are not a
  substitute for code-domain gates.

### Receiver acceptance

- The clean control must achieve the established clean-prefix PVT gate and must
  expose the target PRNs to acquisition/tracking analysis before an
  already-tracking comparison is accepted.
- Already-tracking reports include per-PRN acquisition/tracking status, C/N0,
  prompt-IQ behavior, navigation-bit behavior where available, usable-satellite
  count, `svUsed`, pseudorange residuals, PVT availability, and recovery.
- Cold-start reports include acquisition time, acquired target and non-target
  PRNs, time to first valid PVT, PVT availability, and false matched-code peak
  evidence.
- A particular degradation magnitude is a measured research result, not a
  waveform correctness gate. Dataset acceptance requires correct structure,
  passing clean controls, complete receiver execution, and honest reporting of
  whether degradation occurred at each J/S and phase seed.
- Threshold or mitigation claims require repeated carrier-phase seeds. Reports
  aggregate median, range, and per-seed outcomes rather than presenting only
  the strongest denial result.

### Prior art in the repository

- The existing jammer-source C tests provide the pattern for deterministic
  sample generation, chunk invariance, source metrics, RMS, clipping, and
  waveform-specific assertions.
- The existing Python dataset tests provide the pattern for CLI validation,
  manifest augmentation, numerical gates, plotting, receiver orchestration,
  and dataset-index assertions.
- The existing RTCM frame scanner and ephemeris tests provide the pattern for
  CRC, fragmented input, NTRIP handling, and ephemeris conversion.
- The existing GNSS-SDR dataset workflow provides the clean-prefix, per-PRN,
  PVT, `svUsed`, and residual evidence boundary.

## Out of Scope

- Alignment to an authentic live-sky IQ recording.
- Direct reconstruction of jammer code phase from RTCM 1004 pseudorange.
- Alignment to a victim receiver at a location or clock reference different
  from the Trimble scenario reference.
- RF-path, antenna-cable, transmitter-latency, or receiver-clock calibration.
- Open-air transmission, SDR configuration, or enabling matched-code in a
  hardware transmitter.
- Copying LNAV data, generating a coherent counterfeit navigation solution, or
  any other spoofing mode.
- Initial carrier-phase alignment with the clean signal.
- Dynamic target changes during a dataset run.
- Automatic selection of target PRNs without explicit researcher input.
- Galileo, GLONASS, BeiDou, QZSS, SBAS, L2, L5, or multi-frequency matched-code
  interference.
- Reworking or removing the legacy attack flags beyond retaining them as
  documented reproduction or negative-control behavior.
- Implementing the mitigation algorithm itself; this specification produces
  the dataset and evidence needed to develop and evaluate mitigation.
- Claiming field-authentic propagation, multipath, front-end compression, or
  installed-receiver behavior from the synthetic fixtures.

## Further Notes

- The term **matched-code interference** is used here for navigation-data-free
  GPS C/A components that occupy selected code-Doppler cells. It must not be
  presented as ordinary broadband noise, and it must not be presented as
  spoofing because no coherent navigation message or false PVT solution is
  generated.
- A live NAVIS sample observed during planning carried 1004 observations at
  approximately one hertz, repeated healthy 1019 ephemeris, station metadata,
  and a stable set of roughly ten GPS PRNs. This establishes feasibility but is
  not a hardcoded dataset input.
- Matched-code outcomes can depend strongly on the independent carrier phase
  relative to the clean signal. Multi-seed campaigns and per-seed reporting are
  therefore part of the research contract, not optional cosmetic metadata.
- The simulator currently has a finite channel count. Requiring target-channel
  allocation and failing clearly is preferable to silently generating jammer
  components for clean signals that are absent from the fixture.
- RTCM snapshots are research inputs and should be retained with the same care
  as RINEX, clean IQ, receiver configuration, and analysis code. Reproduction
  should begin from the snapshot, not from a later reconnect to a moving live
  stream.
- Relevant technical references include IS-GPS-200 for GPS L1 C/A code and
  timing, RTCM 10403 for messages 1004/1006/1019, and Murrian et al., *GNSS
  Interference Monitoring from Low Earth Orbit*, for the distinction between
  matched-code interference and spoofing-like signals.

The feature is complete when a researcher can provide target PRNs and either a
live or frozen RTCM source, run the offline dataset workflow, and receive a
replayable scenario plus clean, jammer, mixed, code-domain, and receiver
artifacts whose manifests prove that every requested PRN remained aligned in
code phase, carrier Doppler, and code-rate Doppler for the declared interval.
