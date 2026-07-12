# Statistical Appendix

## Valid analysis level

- Independent repetitions per condition: **1**.
- Noise seed: **20260712**.
- Inferential statistics: **not supported**. No p-values, confidence intervals, normality tests, or multiple-comparison corrections are reported.
- Effect magnitude: exact matched C/N0 changes in dB and exact valid-fix counts. These are physical paired differences, not standardized population effect sizes.
- Unit caution: PRNs share the same composite RF record and are not independent repetitions. The six common stable PRNs are a within-record channel breakdown only.

## Primary descriptive summary

| Type | Median C/N0 change (dB) | PRN range (dB) | Control PVT | Jammed PVT |
|---|---:|---:|---:|---:|
| CW | -2.46 | [-9.97, +3.57] | 19/20 | 19/20 |
| Narrowband | -13.07 | [-16.59, -10.32] | 19/20 | 0/20 |
| Wideband | -10.94 | [-16.19, -10.06] | 19/20 | 0/20 |
| Chirp | -14.24 | [-17.94, -12.47] | 19/20 | 0/20 |
| Pulsed | -8.98 | [-12.70, -5.60] | 19/20 | 19/20 |

Higher C/N0 and PVT availability are better. C/N0 change is `jammed - matched clean control` over seconds 40-60, so negative values indicate degradation. The range is over G10, G12, G23, G28, G31, and G32 only and is not a confidence interval.

## Exact per-PRN C/N0 changes

| PRN | CW | Narrowband | Wideband | Chirp | Pulsed |
|---|---:|---:|---:|---:|---:|
| G02 | +10.96 | -0.04 | +0.05 | -9.44 | -5.89 |
| G10* | -1.77 | -10.89 | -10.06 | -13.52 | -5.60 |
| G12* | +3.57 | -11.88 | -11.71 | -12.47 | -8.62 |
| G18 | +14.91 | +0.18 | -0.13 | -0.21 | -6.19 |
| G21 | +13.81 | -11.13 | -11.13 | -10.62 | -7.89 |
| G23* | -4.65 | -16.59 | -10.06 | -17.49 | -7.44 |
| G28* | -9.97 | -10.32 | -10.17 | -17.94 | -9.39 |
| G31* | +3.27 | -14.27 | -14.39 | -14.05 | -12.70 |
| G32* | -3.16 | -16.37 | -16.19 | -14.44 | -9.34 |

`*` marks a PRN whose clean-prefix median C/N0 was at least 30 dB-Hz in every paired receiver pass. Non-starred PRNs remain in the artifact record but are excluded from the primary median because late/weak acquisition can make their C/N0 change misleading (for example, a channel becoming stable later can look like an improvement).

## Assumptions and blockers

- The paired comparison assumes each type's clean and jammed fixtures share the same desired IQ, noise seed, receiver configuration, and pre-jammer samples. The campaign validates one shared clean SHA-256 and clean-prefix gates for both members of every pair.
- A repeated-seed campaign is required for uncertainty intervals or hypothesis testing.
- Authentic captures and receiver/front-end repetitions are required for field-general effect claims.
