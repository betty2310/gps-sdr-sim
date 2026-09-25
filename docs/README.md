# Documentation map

Current experiment state: 2026-09-22.

## Start here

| Document | Purpose |
|---|---|
| [command.md](../command.md) | X300/F9P setup, no-RF checks, bladeRF RX and conditional contained-TX commands |
| [X300 usage](x300-usage.md) | Implemented CLI, mode compatibility, timing and manifests |
| [Current mixtracking plan](x300-live-sky-mixtracking-research-plan.md) | Completed work, verified hardware and outstanding implementation/measurements |
| [2026-09-22 qualification](x300-f9p-rf-timing-qualification-2026-09-22.md) | Latest receiver identities, clock retry, incomplete-navigation refusals and bladeRF baseline |
| [2026-09-21 validation](x300-f9p-start-validation-2026-09-21.md) | Historical implementation evidence and the earlier source-contamination caveat |
| [Revive](synth-revive-mode.md) / [clone](synth-clone-mode.md) | Current synthetic mode semantics and input requirements |

The present antennas are exposed, and X300 GPS transmission remains off.
Local PPS checks and bladeRF sample capture have passed. GPS-labeled RF start,
controlled mixing and the final mixtracking dataset remain unverified.
Live F9P time plus frozen-RINEX revive is supported only by the offline
`--ublox-time-tcp --dry-run` integration check; its RF transmission is unsupported.

## Related implementations

- [Synthetic waveform overview](synthetic-satellites.md) and
  [implementation map](synthetic-satellites-implementation.md).
- [Matched-code X300 guide](realtime-code-aligned-matched-code-x300.md) and
  [specification](realtime-code-aligned-matched-code-x300-spec.md): separate,
  navigation-free interference; not the revive command contract.
- [Matched-code bladeRF](realtime-code-aligned-matched-code-bladerf.md) and
  [legacy RTCM consumers](trimble-rtcm-ephemeris-sync.md).
- [Offline datasets](cw-jamming-dataset.md), [multi-waveform source](realtime-cw-jammer-x300.md)
  and [scenario catalog](jamming-spoofing-scenarios.md): separate experiments.

## Historical material and consolidation

Keep dated capture reports, issue investigations and published/offline dataset
evidence as historical records. The [2026-09-19 timing review](x300-long-run-timing-rubidium-review.md)
describes the pre-refactor source and must not be used as current CLI guidance.
April receiver reports do not establish September hardware performance.
The old `analysis-output` figure catalog/gallery also reference generated run
artifacts absent from this checkout; retain those links as historical locations,
not as evidence that the underlying captures are currently available.

The following stale documents were removed after consolidating the current
requirements into the runbook, usage and plan:

- `x300-timing-alignment-plan.md`: old software-only Trimble/ppm plan.
- `x300-timing-review-understanding.md`: superseded pre-refactor checklist.
- `x300-hardware-timing-refactor-plan.md`: implementation proposal now covered
  by current completion state and remaining requirements.
- `x300-spoofing-setup.md` and `synthetic-satellite-spoofing-setup.md`: old OTA
  recipes and unqualified gain/clock assumptions.

The old 50 KB mixed-revision plan and duplicated revive/clone design snippets
were replaced with current references. Retained histories explicitly identify
their original device, date and limitations.
