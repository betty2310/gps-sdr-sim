# Revived GPS versus live sky: 22 September v1

Follow-up: the [v2/v3 comparison](x300-mixed-timing-v2-v3-2026-09-22.md)
finds uncorrected delays of 37.932 ms and 37.445 ms. The supplied commands did
not apply the new model-offset option. Those captures confirm that the v1
value below is capture-specific; its repeatability across starts is unverified.

The stable part of both receiver recordings shows a common **+36.93 ms
effective pseudorange delay** on transmitted PRNs 10, 23, 24 and 25 relative
to sky GPS. The rate mismatch is much smaller: approximately **0.4–0.5 ns/s**
over the shared valid-phase interval. The supported software correction is a
constant advance of the model epoch, approximately **+0.0369307 s**, while
retaining the hardware sample rate. This is an empirical candidate for this
setup, not a measured USB latency or a verified absolute GPS/RF alignment.

No hardware was accessed or RF transmitted during this investigation. The
software was rebuilt and tested offline, with fake-radio and loopback TCP tests.

## Evidence and scope

Inputs are the two UBX files in
`/Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v1`:

| Receiver | RAWX epochs | Receiver-local TOW span | Valid PVT fixes |
|---|---:|---|---:|
| COM3 | 164 | 205076.998–205239.998 | 164/164 |
| COM4 | 170 | 205069.994–205238.994 | 170/170 |

Both are in GPS week 2437. Neither file contains a failed UBX checksum or a
reported RAWX clock-reset epoch. `NAV-STATUS.spoofDetState` stays 1 throughout
both recordings. These flags do not identify the reason for tracking loss.
Recorded MON-VER identifies COM3 as HPG 1.51 / protocol 27.50, and COM4 as
HPG 1.32 / protocol 27.31; both are ZED-F9P. The transmitter's separate time
reference identifies as HPG 1.32.
Their hashes are retained in the [analysis summary](../analysis-output/runs/sp-partial-22-09-v1-timing/summary.json).

The exact manifest matching the supplied start epoch is
[`x300tx-ublox-1790067432-76364.json`](../x300tx-ublox-1790067432-76364.json).
It records hardware start 4.0 s, model start **2437:205061.1880120575**, actual
sample rate 2,500,000 Hz, and all three delay assumptions as zero. It confirms
the four selected PRNs and channel 1.

That manifest's RINEX SHA-256 is
`605908504ddf0f76b89e895f584bb0347ddd6e32c6d76dd5044ea52d8faba2a9`.
The current `hour2650.26n` has SHA-256
`424e21a1f085c5fc88975ec388e455dec1d1475a09351a880bd1838cbbb75037`.
It has changed since the run. Consequently, the historical timing calculation
uses ephemerides decoded from the recorded navigation, not today's RINEX file.
An exact replay of the old generator would require its original RINEX and
software snapshot.

The player's printed calendar start is GPS time. It must not be compared
directly with NAV-PVT's UTC calendar fields. Calculations here use GPS week/TOW.

## Measurement method

The [analysis tool](../tools/analyze_mixed_gps_timing.py) checks UBX checksums,
decodes GPS L1 SF1–SF3, and requires matching IODE/IODC. It retains distinct
issues and reconstructs the revived orbits actually broadcast. At each RAWX
measurement it calculates satellite transmit time, satellite clock including
TGD/relativity, geometric range and Earth rotation, using the specified fixed
location `21.0047844,105.8460541,22`.

For each receiver and epoch, define `B_i = measured_pseudorange_i -
predicted_pseudorange_i`. The median `B` from known sky PRNs estimates the
common receiver clock term. Then:

```text
effective_relative_delay_i = (B_i - median(B_sky)) / c
relative_rate_i = (-lambda * measured_Doppler_i - predicted_range_rate_i
                   - median(sky_carrier_clock_rates)) / c
```

`rcvTow - pseudorange/c` cancels the receiver clock before satellite-clock
correction. `NAV-CLOCK` is not subtracted a second time. The sky list is
3, 4, 8, 16, 26, 27, 28, 31, 32, independently supported by the transmitter's
reference receiver recording/manifest. PRN 32 lacks a complete ephemeris in
the two mixed captures and is omitted from geometry calculations. The baseline
requires at least three resolved-phase sky signals above 10 degrees and
25 dB-Hz; the clean interval actually retains at least six for COM3 and five
for COM4.

Other tracked identifiers are not automatically treated as authentic. For
example, recorded PRNs 7 and 22 contain navigation payloads identical to those
of transmitted PRNs 23/24; additional identifiers show partial duplicates.
They cannot independently establish sky timing and are excluded. This
observation alone does not identify the acquisition/correlation mechanism.

The decoder agrees with the existing independently decoded RTKLIB ephemeris
fixture. Reconstructed azimuth/elevation median absolute differences from
valid NAV-SAT geometry are about 0.19–0.23 degrees. Sky residual RMS during the
clean interval is 21.3 m (COM3) and 15.5 m (COM4). Atmospheric delay, multipath,
antenna position differences and RF path delay are not separately estimated;
sub-microsecond details must be interpreted with these limits.

## Timing and tracking results

All four PRNs have valid, resolved carrier phase in **both** receivers at
TOW **205077–205123**: 47 epochs spanning 46 seconds. COM3 retains its full
four-PRN set through 205129. COM4 loses the full set at 205124, while PRN 10
continues through 205129. Neither regains a full resolved-phase set afterward.

| Quantity, shared clean interval | COM3 | COM4 |
|---|---:|---:|
| Median revived-minus-sky delay | 36.930510 ms | 36.930855 ms |
| Linear delay growth | 0.495 ns/s | 0.368 ns/s |
| Doppler-derived relative rate | 0.433 ns/s | 0.426 ns/s |
| PRN 10 median delay | 36.930575 ms | 36.930554 ms |
| PRN 23 median delay | 36.930445 ms | 36.930979 ms |
| PRN 24 median delay | 36.930458 ms | 36.930975 ms |
| PRN 25 median delay | 36.930786 ms | 36.930791 ms |

The rate estimates are short-window relative measurements, not an absolute
rubidium frequency calibration. They do not justify a new ppm adjustment.
The common delay corresponds to roughly **11,071 km of pseudorange**, much
larger than the tens-of-metres position discrepancy between the specified
model location and the receivers' PVT locations.

At the end of stable tracking the generated PRNs still report approximately
50–52 dB-Hz. Later raw observations often jump by integer milliseconds and
lose carrier validity. Their apparent near-zero residuals are not evidence
that the transmitted waveform became synchronized. No epoch uses all four
generated PRNs simultaneously in navigation; PRN 25 is never used. Both
receivers continue producing valid position fixes, chiefly supported by sky.

![Full timing and phase-validity timeline](../analysis-output/runs/sp-partial-22-09-v1-timing/timing.png)

The matching transmitter manifest ends with `EVENT_CODE_SEQ_ERROR`, zero
underflows and zero time errors. It had accepted 419,279,970 samples, or
167.711988 s; the accepted sample endpoint is model TOW **205228.9**. The event
has no hardware timestamp. The final reference snapshot is also near TOW
205228. This logged failure is substantially later than the first phase loss
at 205124/205130 and cannot by itself explain that earlier loss. Earlier
unobserved RF/sample disturbances or receiver tracking behaviour remain
possible. Do not label the full run continuous or claim a proven anti-spoof
shutdown.

## Implemented timing-model change

`x300tx` now accepts **`--model-time-offset-sec`**, a signed value from -1 to
+1 second, default zero. It represents an empirical generated-versus-sky
correction separately from USB/TCP delivery and RF-path delay estimates:

```text
G_model(n) = G_receiver_estimated_at_H_start
             + delivery_delay + tx_path_delay - sky_path_delay
             + model_time_offset + n / actual_sample_rate
```

Here `G_receiver_estimated_at_H_start` excludes the three explicit delay
parameters. Positive offset advances the model epoch at the same hardware
deadline and reduces a positive relative pseudorange delay. It is applied once
before channel/ephemeris initialization and rendering, across every PRN. It
does not retime buffered IQ, change sample rate or continuously follow host
packet jitter. Both signs normalize GPS week boundaries.

The value and operator-supplied source are recorded in the run manifest;
hardware plans also retain the model epoch before correction. Receiver time
estimates and health anchors remain independent. Unsupported combinations
(replay, explicit scenarios, receiver-only checks) reject the option before
receiver/radio access. `--check-start` and finite time-only `--dry-run` use the
same correction as transmission. Alignment/calibration flags remain false.

For the same setup, append this candidate to the original command:

```sh
--model-time-offset-sec 0.0369307
```

Leave `--delivery-delay-sec` and the RF-path terms at zero when using this
entire measured residual. Do not apply the same offset twice. This moves the
example model start from 205061.1880120575 to **205061.2249427575**, keeping its
hardware start at 4.0 s. A new invocation chooses a new hardware deadline and
fresh epoch; those historical times are illustrative, not settings to reuse.

The existing `--delivery-delay-sec 0.0369307` could produce the same numeric
advance when the other terms are zero, but would describe the entire measured
effect as receiver/transport delivery. The new option preserves the distinction
the dataset can actually support and also permits negative empirical residuals.

## Validation and remaining measurement

Fitting only TOW 205077–205099 gives +0.0369306826 s. Subtracting this estimate
from the held-out TOW 205100–205123 observations leaves a median residual of
0.0024 microseconds and a 95th-percentile absolute residual of **0.421
microseconds**. This checks within-capture stability and the correction sign;
it is not a re-transmission test or a guarantee for the next startup. Neither
the unknown fixed delivery floor nor the UHD query midpoint error is proved
repeatable by one recording. The original query bracket was 229.9 microseconds
wide, and recent arrival variation was 845.6 microseconds.

![Clean-window residuals and held-out epochs](../analysis-output/runs/sp-partial-22-09-v1-timing/clean-window.png)

Passed offline checks:

- `make test-ubx test-x300-timing test-x300tx-matched`: parser, signed mapping,
  GPS week rollover, sample-rate preservation, deadlines, transport faults,
  manifests and 24 Python CLI cases.
- Rebuilt and ran `tests/test_x300_live_revive` after extending it to generate
  finite/continuous revived signals with positive and negative corrections
  through the production renderer and fake radio.
- `tests/test_nav_timing`: fractional navigation frames, HOW and week rollover.
- `make test-mixed-gps-timing`: RTKLIB fixture comparison and injected positive/
  negative signal-delay recovery with two different receiver clock biases.

A subsequent recording is needed to establish the residual after the corrected
waveform is transmitted. Evaluate the resolved-phase interval against the same
independent sky baseline and retain its transmitter manifest. GPS-labeled
hardware-edge association and RF-path measurements are still needed to claim
absolute GPS alignment; a one-way USB/TCP stream alone cannot identify its
constant latency.

## Reproduction and sources

The analysis has no hardware operations and writes only to its output directory.
From the repository root, with the existing processing environment:

```sh
rtk proxy uv run --project processing python tools/analyze_mixed_gps_timing.py \
  /Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v1/COM3___9600_260922_085733.ubx \
  /Users/betty/Developer/research/angle/dataset/sp-partial-22-09/v1/COM4___9600_260922_085726.ubx \
  --revived 10 23 24 25 --sky-prns 3 4 8 16 26 27 28 31 32 \
  --llh 21.0047844 105.8460541 22 \
  --output analysis-output/runs/sp-partial-22-09-v1-timing
```

The output contains checksummed input identities, parsed RAWX/navigation/clock
tables, decoded subframes/ephemerides, per-observation timing residuals, JSON
summary and plots. Source UBX files are unchanged.

Message semantics were checked against the
[u-blox F9 HPG 1.32 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)
and [HPG 1.51 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.51_InterfaceDescription_UBXDOC-963802114-13124.pdf)
(NAV-CLOCK, NAV-TIMEGPS and RAWX), and hardware-edge/epoch distinctions against
[Ettus device synchronization](https://files.ettus.com/manual/page_sync.html).
