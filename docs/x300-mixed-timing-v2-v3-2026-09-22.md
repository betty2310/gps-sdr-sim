# Revived GPS versus sky: v2/v3 follow-up, 22 September 2026

**The new model offset was not applied in these recordings.** The user supplied
the v2/v3 command; it omits `--model-time-offset-sec`. The matching transmitter
manifests record zero delivery, TX-path and sky-path corrections and lack the
new model-offset field. The v3 console start epoch and accepted-sample count
match its manifest exactly. The rebuilt local binary now advertises the option
in `--help`, but its default remains zero.

The three transmitted PRNs in both new captures are **10, 24 and 25**. PRN 23
was included in v1 only. Both receivers measure a common positive delay of
approximately **37.932 ms in v2** and **37.445 ms in v3** relative to sky GPS.
These are uncorrected observations, not a test of the new offset's RF effect.

## Comparison with v1

The same [offline reconstruction](../tools/analyze_mixed_gps_timing.py) and
fixed analysis location were used for all captures. Delay medians below use
only epochs with all selected PRNs' carrier phase valid and half-cycle
ambiguities resolved in both receivers. Each receiver's sky clock estimate
is formed independently; the run value gives equal weight to both receivers.
Positive delay means that the generated signal is effectively late relative
to the sky model.

| Dataset | Generated PRNs | Shared phase-valid GPS TOW | Epochs (elapsed span) | COM3 delay | COM4 delay | Residual if 36.9307 ms were subtracted offline |
|---|---|---|---:|---:|---:|---:|
| v1 | 10, 23, 24, 25 | 205077–205123 | 47 (46 s) | 36.930510 ms | 36.930855 ms | -0.000017 ms |
| v2 | 10, 24, 25 | 206569–206617 | 49 (48 s) | 37.932085 ms | 37.932086 ms | +1.001386 ms |
| v3 | 10, 24, 25 | 206726–206749 | 24 (23 s) | 37.444773 ms | 37.444772 ms | +0.514072 ms |

All times are in GPS week 2437. The last column is a counterfactual arithmetic
comparison with the exact previously proposed `0.0369307` seconds. It does
not describe a corrected transmission. A constant measured in v1 would remove
most of the bias, but these captures do not support treating it as a repeatable
sub-microsecond startup calibration.

![Measured offsets and fixed-v1-offset comparison](../analysis-output/runs/sp-partial-22-09-comparison/comparison.png)

| Relative rate during the shared clean window | v2 COM3 | v2 COM4 | v3 COM3 | v3 COM4 |
|---|---:|---:|---:|---:|
| Linear pseudorange-delay growth | 0.464 ns/s | 0.489 ns/s | 0.550 ns/s | 0.507 ns/s |
| Independent Doppler-derived rate | 0.437 ns/s | 0.427 ns/s | 0.410 ns/s | 0.415 ns/s |

The rate stays near the v1 result. Millisecond differences between starts
dominate the tens of nanoseconds accumulated during a clean window; the data
do not justify a ppm-scale sample-rate adjustment.

## Distance and the GPS label of the local PPS

The user reports about 1 m separation in v1 and up to 3 m in v2/v3. Using
`delay = distance / 299792458`, direct propagation is about 3.34 ns at 1 m and
10.01 ns at 3 m. The additional 2 m contributes only **6.67 ns**, versus
observed inter-run changes of approximately 1,001,403 ns and 514,090 ns.
This distance change cannot explain the millisecond offset. It also does not
measure RF hardware group delay or multipath.

A useful cross-run consistency check is to add each effective delay back to
the fractional model start. The table uses the short clean-window linear
delay fit extrapolated back to sample zero, rather than a later-window median:

| Dataset | Logged model-start fraction | Inferred arrival fraction after adding delay |
|---|---:|---:|
| v1 | 188.012058 ms | 224.942743 ms |
| v2 | 187.011407 ms | 224.943472 ms |
| v3 | 187.498782 ms | 224.943530 ms |

The inferred arrival phase changes by less than one microsecond over 1,631 s,
while the software model-start fraction changes by about one millisecond.
A three-point linear fit gives approximately **0.485 ns/s**, consistent with
the short-window rate scale. Three points and model-based extrapolation do
not establish nanosecond absolute accuracy; sky residuals remain around
21–22 m and atmospheric/RF path effects are not separately estimated.

This pattern is consistent with a stable local PPS boundary whose GPS label
is estimated with varying software arrival delay. It strengthens the case
for improving the **hardware-edge-to-GPS association at startup**. It does
not prove a particular USB/TCP latency or independently measure a GPS-labeled
hardware edge. The approximately 224.943 ms value is a retrospective,
setup-specific inferred arrival phase, not a new universal software constant.

The manifests illustrate why their small jitter figures cannot bound the
absolute bias:

| Dataset | Arrival correction | Recent arrival variation | Hardware-query half width |
|---|---:|---:|---:|
| v1 | 0.000 us | 845.644 us | 114.938 us |
| v2 | 42.734 us | 153.629 us | 127.187 us |
| v3 | 619.971 us | 635.835 us | 141.729 us |

All three report `constant_delivery_bias_known: false`. In particular, the
v3 log's `arrival correction 0.000620 s` is the existing arrival-window
estimator term; it is not the new 36.9307 ms empirical model correction.

## Tracking, navigation and transport

In v2, COM3 has complete resolved phase from 206564 through 206617; COM4 has
it from its first epoch, 206569, through 206617. All three PRNs lose usable
phase thereafter and none is used in NAV-SAT navigation. COM3 obtains valid
PVT at 206568 after six initially invalid epochs, giving 90/96 valid fixes;
COM4 has 88/88. Their navigation continues on sky signals.

In v3, COM3 has complete resolved phase from its first epoch, 206726, through
206749; COM4 has it from 206717 through 206749. They have 81/81 and 89/89
valid fixes. Later NAV-SAT reports PRN 10 used for 16 epochs on COM3 and PRN 25
used for 47 epochs on COM3 and 17 on COM4. **None of those later uses overlaps
resolved carrier phase on those PRNs.** Some navigation epochs have no matching
RAWX row. Later near-zero pseudorange residuals therefore do not demonstrate
that the transmitted signal became aligned or that the three-PRN phase set
recovered. The final valid-phase observations still show 44–51 dB-Hz in v2
and 45–50 dB-Hz in v3.

Both captures have zero failed UBX checksums, no reported RAWX clock resets,
monotonic NAV-STATUS uptime, and `spoofDetState=1` throughout. Those flags do
not establish the cause of phase loss.

| Dataset | Matching transmitter manifest | Accepted sample endpoint in model TOW | First epoch without the complete phase set |
|---|---|---:|---:|
| v2 | [1790068922-82153](../x300tx-ublox-1790068922-82153.json) | 206622.000000 | 206618 |
| v3 | [1790069063-82563](../x300tx-ublox-1790069063-82563.json) | 206797.900000 | 206750 |

Both runs end with one `EVENT_CODE_SEQ_ERROR`, zero reported underflows and
zero time errors. Their first error has no hardware timestamp. An accepted
sample endpoint is an accounting value, not a measured RF stop time. The
recordings lose phase before those endpoints, especially v3; the logs do not
prove the final reported sequence error caused the earlier loss. Full-run
sample continuity remains unverified.

Full timelines: [v2](../analysis-output/runs/sp-partial-22-09-v2-timing/timing.png),
[v3](../analysis-output/runs/sp-partial-22-09-v3-timing/timing.png).

## Software implication and validation status

`--model-time-offset-sec` is already implemented as an explicit signed model
epoch advance, applied once before rendering while preserving the hardware
deadline and actual sample rate. These commands did not exercise it. The
default remains zero; no new transmitter code or default offset was changed
for this follow-up. The current local binary's `--help` confirms the option.

For clarity, the previously proposed candidate is:

```text
--model-time-offset-sec 0.0369307
```

It is a v1 empirical correction. A recorded run actually using it must show
`model_time_offset_s: 0.0369307` in its manifest. The resulting mixed recording
would measure the remaining error. This comparison indicates that startup
association also needs calibration or a directly GPS-labeled edge before
repeatable close alignment can be claimed. It does not support silently
replacing the candidate with either v2's or v3's measured delay.

Refitting each new capture separately gives the following *within-capture*
holdout results:

| Dataset | First-half fitted correction | Held-out epochs | Held-out p95 absolute residual |
|---|---:|---|---:|
| v2 | 0.0379320799 s | 206593–206617 | 0.0234 us |
| v3 | 0.0374447685 s | 206738–206749 | 0.0143 us |

These small residuals indicate internal stability over short intervals. They
are not an independent validation of absolute timing, repeatability across
starts, or the v1 offset. The per-capture `clean-window.png` plots subtract
their own fitted offset and must be interpreted accordingly.

## Reproduction and artifacts

The recorded SF1–SF3 payloads provide the historical ephemerides. For generated
PRNs 10/24/25, decoded IODEs are 76/84/17; their TOE/TOC is 206544 in v2 and
206688 in v3. These values agree with the selected PRNs and 16-second-aligned
revive epochs of the matched starts. Sky PRNs 3, 4, 8, 16, 26, 27, 28 and 31
provide eight phase-valid reference signals in every analyzed clean epoch.
PRN 32 is on the independently identified sky list but has no complete decoded
ephemeris in these mixed captures. PRN 9 in v3 is not added to the baseline.
The orbit/clock reconstruction uses the same method and limitations documented
in the [v1 report](x300-mixed-timing-2026-09-22.md).

```sh
rtk proxy uv run --project processing python tools/analyze_mixed_gps_timing.py \
  /Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v2/COM3___9600_260922_092218.ubx \
  /Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v2/COM4___9600_260922_092226.ubx \
  --revived 10 24 25 --sky-prns 3 4 8 16 26 27 28 31 32 \
  --llh 21.0047844 105.8460541 22 \
  --output analysis-output/runs/sp-partial-22-09-v2-timing

rtk proxy uv run --project processing python tools/analyze_mixed_gps_timing.py \
  /Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v3/COM3___9600_260922_092502.ubx \
  /Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v3/COM4___9600_260922_092454.ubx \
  --revived 10 24 25 --sky-prns 3 4 8 16 26 27 28 31 32 \
  --llh 21.0047844 105.8460541 22 \
  --output analysis-output/runs/sp-partial-22-09-v3-timing

rtk proxy uv run --project processing python \
  analysis-output/runs/sp-partial-22-09-comparison/compare.py
```

Independent `angle/probe_dataset.py ... --spoof 10 24 25` checks agree exactly
on the 49-epoch v2 and 24-epoch v3 shared phase windows and navigation-use
counts. SHA-256 verification confirms all six v1/v2/v3 source recordings are
unchanged. No hardware was accessed or transmission started.

- [v2 summary and input hashes](../analysis-output/runs/sp-partial-22-09-v2-timing/summary.json)
- [v3 summary and input hashes](../analysis-output/runs/sp-partial-22-09-v3-timing/summary.json)
- [Comparison JSON](../analysis-output/runs/sp-partial-22-09-comparison/comparison.json)
- [Per-receiver comparison CSV](../analysis-output/runs/sp-partial-22-09-comparison/receiver_comparison.csv)
- [Reproducible comparison script](../analysis-output/runs/sp-partial-22-09-comparison/compare.py)
- [Comparison figure as SVG](../analysis-output/runs/sp-partial-22-09-comparison/comparison.svg)
