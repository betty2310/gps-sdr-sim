# X300 live-sky mixtracking: current state and remaining plan

Updated 2026-09-22. This document supersedes the earlier software-only timing
and pre-refactor proposals. [command.md](../command.md) is the operator runbook;
[X300 usage](x300-usage.md) defines the implemented CLI. Dated reports retain
the actual measurements and their original input conditions.

**Goal:** continuously generate revived PRNs on the current GPS timeline from
the independent sky F9P, then qualify their arrival timing alongside authentic
signals at the DUT. Revived orbit/navigation remains intentional synthetic
content. The requested result is a common time base, not identical authentic
orbit data, navigation bits or carrier phase.

**Implemented:** u-blox input, local PPS timing, fixed-deadline estimated GPS
startup for live navigation, failure handling, and F9P-time + RINEX revive
with finite TX, continuous `--stream`, `--check-start` and offline rendering.
GPS-now and both waveform paths share minimum-delay time estimation; hardware
planning selects the narrowest of eight clock-query brackets.
**Not established:** actual X300 RF transmission,
absolute GPS-to-RF alignment, controlled live-sky mixing, or a completed
mixtracking dataset. This workflow has not transmitted X300 RF during the
qualification series.

The active constraint is **F9P USB/TCP only for GPS time**, with existing PRS10
10 MHz/PPS wiring. A direct F9P-to-X300 PPS cable is unavailable and is not a
prerequisite for the software milestones below. Software can estimate the
association; it cannot promise perfect absolute alignment from these messages
alone. No hardware state was refreshed during this documentation/code review.

## 1. Current hardware and measured evidence

| Component | Last verified state |
|---|---|
| X300 | `192.168.10.2`, serial `32244B4`; two UBX-40 boards; `A:0 B:0` gives channel 1 = RF B |
| PRS10 | Powered; 10 MHz to X300 REF IN and local PPS to PPS IN; no verified GNSS discipline/epoch association |
| Sky F9P | HPG 1.32 / protocol 27.31; chip `314852bd18`; last USB `/dev/cu.usbmodem11301` |
| Second F9P | Same firmware/protocol; chip `d44162bdda`; last USB `/dev/cu.usbmodem1301` |
| bladeRF | x40, serial `270c4bef62ded4be1bd9c822a60306a5`; FPGA 0.16.0 loaded, USB SuperSpeed |
| RF path | User replaced bare X300-to-F9P coax with X300 TX and bladeRF RX antennas, both exposed on the bench; TX remains off |

USB paths can change. Match receiver chip IDs and observed roles after each
reconnection. Neither a module name nor MON-RF unknown antenna/power states
identify a carrier board's bias circuit or accessible pulse pins.

- **2026-09-21:** three no-TX-streamer startup checks passed, one refused.
  Another transmitter was subsequently found active; those passes establish
  software/hardware preparation only, not clean live-sky startup. Later checks
  after shutdown refused on bad reference state or incomplete navigation.
  [Full report](x300-f9p-start-validation-2026-09-21.md).
- **2026-09-22:** the external clock/PPS check initially failed PLL lock, then
  passed on retry. No cause was proved. The focused reference capture had valid
  3D PVT and nonempty RAWX in all 90 epochs. Both startup attempts refused before
  UHD/IQ: automatic selection lacked PRN 15 navigation; explicit selection
  lacked PRN 24 SF3. Latest DUT check had no fix/observations.
- **bladeRF baseline:** five seconds at 1575.42 MHz, 2.5 MS/s, 2.5 MHz bandwidth,
  manual 5 dB gain; 12.5 million complex samples / 50 MB, no clipping or CLI RX
  error. Raw IQ has no timestamp metadata; continuity, antenna coupling and
  GPS epoch remain unverified.
  [RF/timing qualification](x300-f9p-rf-timing-qualification-2026-09-22.md).

## 2. Implemented software

| Component | Implemented behavior |
|---|---|
| `player/ubx_receiver.*` | Bounded/checksummed UBX parsing; background TCP reader; immutable snapshots; raw bytes and monotonic arrival records; explicit disconnect failure; offline replay |
| Receiver diagnostics | MON-VER, SEC-UNIQID, TIMEGPS, PVT, CLOCK, RAWX, SFRBX, MON-RF, TIM-TP and TIM-TM2; validity/age retained |
| `--check-time` | Receive-only estimated GPS now, independent of navigation availability; rejects replay/stale time, handles week rollover, records unknown absolute uncertainty; no IQ/UHD/TX |
| `--ublox-time-tcp` | F9P TIMEGPS and frozen RINEX revive with explicit location/PRNs; finite/continuous TX, `--check-start`, and offline `--dry-run`; target live navigation is not required |
| `player/gps_lnav.*` | GPS L1 C/A SF1–SF3; matching issues, health and conservative freshness; original TOE/TOC, week, URA and fit metadata |
| `player/x300_live_time.hpp` | Shared minimum-delay time estimate, narrowest of eight host/UHD brackets, fixed future deadline before dated rendering, declared delays and unknown absolute uncertainty |
| `player/x300_timing.hpp` | External reference/PPS checks and latch; bounded producer/sender queue; partial/zero-send handling; fatal continuity/health faults; EOB acknowledgement |
| `player/x300tx.cpp` | Shared normal/matched sender, scenario and live UBX paths, `--check-sync`, `--check-ublox`, `--check-start` and manifests |
| `tools/ubx_tcp_bridge.py` | One serial owner; loopback read-only fanout; identity/RF/pulse polling; slow-client disconnect; no receiver configuration/reset |
| Generator timing | Actual configured rate, cumulative sample counts, fractional starts, frame-boundary handling and GPS HOW/week rollover checks |

X300 Trimble/NTRIP input, host-clock `-n`, ppm scaling and Trimble residual
calibration controls were removed. Shared RTCM and legacy bladeRF tools remain
separate; their commands/calibration do not apply to X300.

Verified regression evidence includes `make test`, `test-x300-timing`,
`test-ubx`, `test-x300tx-matched` and the relevant bladeRF checks as recorded in
the dated reports. Independent RTKLIB comparison matched 270 ephemeris fields
across ten PRNs; it did not authenticate the RF source.

The documented revive fixture was additionally exercised offline on 2026-09-22:
2,812,500 samples, zero clipping, no UHD access. Combining revive and live UBX
was checked to fail before hardware access. This is command qualification, not
RF validation.

## 3. Supported modes and the revive gap

| X300 mode | Navigation input | Epoch source | Revive support |
|---|---|---|---|
| RINEX scenario | Frozen `-e` file | Explicit scenario calendar or week/TOW; otherwise first RINEX epoch | Yes, target's own past ephemeris |
| Live UBX | Reference SFRBX | Estimated future GPS epoch from TIMEGPS and receipt/hardware queries | No |
| Live F9P time + RINEX | Frozen `-e` file; live `--ublox-time-tcp` supplies time only | Estimated GPS epoch at a fixed future X300 deadline | Yes, finite TX / `--stream` / `--check-start` |
| Offline F9P time + RINEX | Same independent time/navigation sources | Receiver time projected to a future host-monotonic instant; no RF deadline | Yes, `--dry-run` |
| Offline UBX replay | Recorded UBX plus arrival CSV | Explicit week/TOW for rendering | No |
| Matched-code | Frozen scenario | Scenario epoch | Synthetic alignment may use frozen revive; emitted signal has no navigation data |

Live clone consumes donor navigation from UBX. Revive uses its target PRN's
own past RINEX ephemeris, including in the time-only modes.

`--ublox-tcp` still requires live navigation and rejects frozen `-e` and
revive. The `--ublox-time-tcp` path separates receiver time from navigation
and renders/transmits revive without `-t`. It needs only F9P identity and
fresh consecutive TIMEGPS epochs, plus explicit model location and target
PRNs. Original RINEX coverage/geometry checks remain enforced. It requires no
F9P-to-X300 pulse cable. `--check-start` opens UHD for rate/PPS/deadline checks
without a TX streamer; `--dry-run` does not open UHD or save IQ.

The current user constraint is software timing from F9P TCP with the existing
PRS10 wiring; a direct F9P PPS connection is unavailable. The software estimate
is possible under that constraint, but unknown transport bias remains.
The software now supports **RF transmission of revived PRNs at the estimated
current GPS epoch**, including `--stream`. Physical RF acceptance remains
unmeasured. The receive-only `--check-time` is a separate diagnostic. No mode proves
absolute RF alignment or identity with authentic navigation/code/carrier.

Offline validation uses a loopback F9P-time source without any PVT/RAWX/SFRBX:
the fixture rendered 312,500 samples with zero clipping and zero accepted
samples. A reference epoch outside the frozen RINEX coverage is rejected;
the file is not rebased. Tests check source conflicts, week rollover,
stale/disconnected/reset references, and ordinary live-navigation readiness.
The new fake-radio integration suite prepares real revive IQ with no reference
PVT/RAWX/SFRBX, sends finite and continuous streams with partial sends, crosses
a navigation-frame boundary, and rejects missed deadlines/disconnected time.
The extension passed 20 CLI tests plus UBX, PPS/transport and revive TX C++
suites. No hardware or RF was exercised by those tests.

### 3.1 Current operator workflow

[command.md](../command.md) now contains the USB enumeration, TIMEGPS enable,
loopback bridge, GPS-now check and automatic-epoch offline revive commands.
It includes no-TX `--check-start` and estimated current-GPS revive `--stream`
commands with the existing RINEX/PRN selection checks retained.
Do not turn a printed GPS-now value into a manual `-t` workaround: time passes
while the next process starts, loads navigation and prepares the radio.

The bridge requires Python `pyserial`; it polls identity and diagnostics but
does not enable TIMEGPS. The documented `ubxsetrate` 1.0.6 command enables
TIMEGPS once per navigation solution on all receiver interfaces, using
`CFG-MSG`. It does not set the solution period, change pulse wiring or save
configuration to flash. USB UBX output must already be enabled. Its successful
send is not an acknowledgement/readback, so `--check-time` remains the runtime
test for identity and consecutive valid time. See the
[utility's implementation](https://raw.githubusercontent.com/semuconsulting/pyubxutils/main/src/pyubxutils/ubxsetrate.py).
Full observation/navigation capture additionally needs PVT, RAWX and SFRBX;
the time-only example does not configure those messages.

The checked CLI accepts these options. Receiver configuration and the Hanoi
example have not been executed against the physical receiver in this review;
the example RINEX filename/PRNs are not a freshly qualified constellation.
The latest code regression run passed 19 Python CLI tests plus the UBX and
X300 C++ suites, using loopback/replay/fake-radio inputs rather than RF.

### 3.2 Source review: weaknesses and consequences

Priorities below concern the requested current-GPS revive stream. Existing
checks for invalid epochs, checksums, missing navigation, missed deadlines,
partial sends, underflows and disconnects should be retained.

| Priority | Current source finding | Consequence / required change |
|---|---|---|
| Resolved in software | [x300tx.cpp](../player/x300tx.cpp): time-only TX and `--check-start` now construct a fixed hardware deadline before dated rendering. | Fake-radio finite/continuous tests pass; hardware/RF qualification remains separate. |
| Resolved in software | [x300tx.cpp](../player/x300tx.cpp): `LiveRun::checkHealth` applies the navigation-source policy to rendering, start checking and sending. | Revived targets need no authentic SFRBX; ordinary live-navigation checks remain enforced. |
| P1 | [ubx_receiver.cpp](../player/ubx_receiver.cpp): all modes now use the least-delayed observation in a recent window; `--delivery-delay-sec` defaults to zero. | Extra queue delay is reduced, but unknown constant receiver/USB/bridge/TCP latency remains. A paced old stream is not detected merely by marking an endpoint live. |
| P1 | [x300_live_time.hpp](../player/x300_live_time.hpp): all paths share an estimator; the narrowest of eight hardware query brackets is selected and its half-width reported. | A bracket as wide as 50 ms is still accepted as a fault guard. A measured absolute accuracy budget remains unavailable. |
| P1 | [ubx_receiver.cpp](../player/ubx_receiver.cpp): readiness checks the `MOD=ZED-F9P` string, not the expected chip ID; receiver `tAcc` is recorded without an acceptance threshold. Time-only intentionally does not require PVT/RAWX. | The wrong F9P or a degraded time solution can qualify. Add explicit reference identity and timing-quality policy. RAWX clock resets are detected only when RAWX is present; reference diagnostics must remain separate from target navigation requirements. |
| P1 | [ubx_tcp_bridge.py](../tools/ubx_tcp_bridge.py) limits queued bytes, not queued time; [ubx_receiver.cpp](../player/ubx_receiver.cpp) timestamps an entire received chunk once. | Buffered data can be old despite regular delivery. Multiple TIMEGPS messages in one chunk share an arrival timestamp and fail the positive-arrival-step test. Preserve source-side receipt metadata for diagnostics, bound buffering by age, and test batching/stalls without treating receipt time as the GPS epoch. |
| P1 | [x300_live_time.hpp](../player/x300_live_time.hpp) freezes the startup association; health allows up to 250 ms time/arrival departure. [x300_timing.hpp](../player/x300_timing.hpp) monitors local PPS/locks, not GPS-to-RF phase. | No long-run precision/drift claim follows from a successful start. Log residuals against new F9P epochs and repeated hardware observations, with explicit limits and segment invalidation. Do not restore block-only ppm corrections or step dated IQ. |
| P1 | [x300tx.cpp](../player/x300tx.cpp): RINEX coverage and target allocation are checked at startup, while `refreshNavState()` has no dedicated time-only per-target expiry/continuity acceptance policy. Manifests retain file hash/PRNs, not the full revive selection history. | Define run duration/expiry, template transitions and target-loss handling for streaming. Record template TOE/TOC, issue, lookback, transform and refresh events. Add live observation-based candidate/collision decisions; explicit `-P` is not proof of sky absence. |
| P2 | [x300tx.cpp](../player/x300tx.cpp): time-only reports unverified alignment and a null hardware start correctly, but shares a hardware-time schema with zero/default hardware fields. | Represent absent hardware/calibration measurements consistently as absent/null; separate declared settings, estimates and measured values in the next schema. |

The legacy [bladetx.cpp](../player/bladetx.cpp) confirms that software epoch
association plus hardware-counter scheduling is feasible without a direct
receiver PPS cable. It uses Trimble tags/host receipt time, not F9P UBX. Its
late-start paths can clamp the remaining lead without rebuilding dated IQ;
that behavior must not be copied into X300's fixed-deadline path.

### 3.3 Implementation sequence

Steps 3 and 4 are implemented and fake-radio tested; step 2 now has a shared
estimator and uncertainty reporting, with transport/absolute-time validation
still pending. Remaining items below retain their acceptance criteria. Physical RF
qualification remains a separate prerequisite for transmitting experiments.

| Order | Deliverable | Acceptance evidence |
|---|---|---|
| 1 | Reproducible receiver setup and reference identity: verify active USB UBX output/rates with acknowledgements/readback; pin the sky chip ID; record receiver firmware, configuration and optional PVT/RAWX/CLOCK diagnostics. | Missing TIMEGPS, wrong receiver, invalid time, poor declared accuracy, reset and disconnect have explicit outcomes. The setup does not reset the receiver or silently change its other roles. |
| 2 | One time-estimation component shared by GPS-now, offline render and hardware planning. Preserve GPS week/fractional TOW and epoch joins. Separate receiver error, arrival variation, unknown constant delay and hardware-query uncertainty. | Deterministic tests for fractional/week rollover, drift, changing delay, batching, duplicate/out-of-order epochs and paced stale input. Unknown absolute bias stays unknown; diagnostics never manufacture calibration from low jitter. |
| 3 | Extend `--ublox-time-tcp` to `--check-start` with frozen revive RINEX. Read actual UHD rate, latch the existing local PPS and freeze a future `(H_start, G_zero)` before dated rendering. Reuse `planLiveStart`/sender timing contracts rather than introducing a host-sleep scheduler. | Fake-radio tests bind sample zero, navigation boundaries and cumulative samples to the same plan. Real no-RF preparation opens no TX streamer. A missed deadline discards the entire dated render; any retry selects a new plan and regenerates. |
| 4 | Integrate continuous time-only revive into the existing bounded sender, preserving `--stream` as the operator choice and keeping F9P time independent of RINEX target navigation. Update every source-health callback and expose the complete plan in the manifest. | Transport tests cover partial/zero sends, late starts, underflow, asynchronous errors, lost reference and shutdown. Invalid continuity ends the segment; restart requires fresh warmup. Only after this stage may `command.md` contain a supported current-GPS revive TX command. |
| 5 | Define long-run navigation and timing policy. Record each revive template/transform, validate expiry and PRN presence/collisions, and monitor GPS/host/hardware residuals and producer margins. Freeze input provenance within each segment. | Run across navigation refresh, file coverage exhaustion, template changes and simulated clock drift. Stop or deliberately start a new documented segment when validity is lost; no silent replay of stale ephemeris or relabeling of queued samples. |
| 6 | Qualify timing at the DUT in a contained experiment, with independently established capture timing and characterized RF branch delays. Direct F9P PPS is one optional future route to absolute measurement, not a prerequisite for stages 1–5. | Publish start bias, jitter, drift, measurement uncertainty and continuity evidence. Distinguish relative waveform correlation from absolute GPS time; a local bladeRF sample index alone is insufficient. |
| 7 | Set numerical mixtracking acceptance criteria and collect authentic-only/generated-only/combined controls. Keep timing, code/Doppler/carrier, navigation content, power and receiver PVT outcomes separate. | The report names the tolerance, reference plane, run length and confidence/uncertainty. No `gps_alignment_verified=true` until independent evidence meets those criteria. |

Completion is **bounded, measured timing agreement**, not a claim of zero
error. Absolute-time/code-phase tolerance and required coherence duration are
still unspecified. They must be chosen before experiment acceptance; the
current 2.5 s freshness, 250 ms watchdog, 50 ms query bracket and 20 ms send
margin are not substitutes for them.

## 4. Timing contract and limits

PRS10 frequency and PPS provide local hardware timing. The distinction between
selecting frequency/PPS sources and latching device
time follows [UHD's synchronization contract](https://files.ettus.com/manual/page_sync.html).
For a scenario:

~~~text
G_model(N) = G_scenario_zero + N / Fs
H_sample(N) = H_start + N / Fs
~~~

For implemented live UBX startup:

~~~text
G_est(H_start) = G_receiver + (H_start - H_at_receipt) + fastest_arrival_residual
               + delivery_delay_estimate
G_model(N) = G_est(H_start) + N / Fs + tx_path_delay - sky_path_delay
~~~

For the implemented time-only offline mode:

~~~text
M_zero = M_query + start_lead
G_zero = G_receiver + (M_zero - M_received) + fastest_arrival_residual
         + delivery_delay_estimate
G_model(N) = G_zero + N / Fs_requested
~~~

`M` denotes host monotonic time. The offline epoch is a label; rendering does
not wait for `M_zero` and there is no hardware deadline. In the live path,
`H_at_receipt` is estimated using a bracketed host/UHD clock observation.
Selecting a whole local hardware second does not select a GPS second boundary.

`Fs` is the configured UHD rate. Rate readback is not a measurement of the
physical oscillator. The live estimate uses monotonic arrival and bracketed
hardware queries; it does not obtain an edge from TCP. Unknown constant
receiver/USB/TCP delay survives low arrival jitter or a small receiver `tAcc`.
Current manifests must retain `gps_alignment_verified=false` and, for live
UBX, `absolute_gps_uncertainty_s=null`.

`NAV-TIMEGPS` supplies the GPS epoch of the latest navigation solution, using
week, integer milliseconds and fractional nanoseconds. It is not the instant
the message reaches the Mac. `NAV-CLOCK` is diagnostic; its receiver clock bias
must not simply be added to an already reported GPS solution. Likewise, a
navigation word's satellite transmission time differs from signal arrival at
the modeled receiver location. The generator already accounts for satellite
propagation. These distinctions follow the
[F9 HPG 1.32 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf).

Software can estimate relative offset/rate and detect some disturbances, but
one-way receiver-to-host messages cannot separate constant delivery delay from
absolute clock offset. Repeated queries and larger start lead do not solve
that identifiability problem. Lead is preparation margin, not latency
calibration. Consequently TCP alone provides no guarantee of perfect RF sync.

For independent absolute qualification, establish `(H_ref, G_ref)`: X300
hardware time and GPS week/TOW for the **same physical event**, or an
independently timed/calibrated RF measurement. Optional future wiring routes:

- Keep PRS10 10 MHz and feed a properly conditioned F9P TIMEPULSE to X300 PPS,
  replacing the local PRS10 PPS; identify the exact pulse label and verify
  subsequent edges.
- Retain PRS10 PPS and label a common instrumented event through F9P EXTINT /
  TIM-TM2, explicitly associating it with the X300 event.
- For longer runs, investigate GPS discipline of PRS10, with measured settling,
  distribution delay and holdover behavior.

Board pinout, electrical levels, loading and delay need verification first.
TIM-TP labels the next pulse; the observed report used a UTC time base, so it
cannot be copied directly to GPS TOW. TIM-TP/TIM-TM2 logging alone establishes
no hardware association. See [wiring options and sources](x300-f9p-rf-timing-qualification-2026-09-22.md#missing-measurement-for-absolute-gps-alignment).

After physical association, the desired arrival model at the DUT plane is:

~~~text
T_DUT(N) = G_ref + (H_start - H_ref) + N / Fs + tau_TX
G_model(N) = T_DUT(N) - tau_sky
~~~

`tau_TX` includes the characterized timestamp-to-RF/filter/cable path.
`tau_sky` is the authentic branch delay relative to the antenna model point.
Satellite propagation is already part of the generator; do not add it twice.
Carrier phase response needs separate characterization.

## 5. RF capture and controlled mixtracking

A safe RF path and a timing measurement solve different requirements. The
present exposed antennas solve neither containment nor GPS association.
Do not transmit generated GPS on this bench arrangement. Use a measured
attenuated conducted path or verified shielding; software gain is not input dBm.

The intended experimental arrangement, not the presently verified wiring, is:

~~~text
Live antenna -> independent reference F9P / authentic observations
             -> characterized live branch --------------------+
                                                             |
X300 RF B -> measured attenuation and DC isolation -> combiner +-> DUT
                                                             +-> IQ monitor
~~~

Account for splitter/combiner loss, active-antenna power routing, leakage into
the reference, and differential delay/phase. A separate reference antenna adds
a geometry/propagation difference that must be modeled. The latest bladeRF RX
baseline was separate from any qualified live-sky combination.

Before claiming RF start time, capture IQ with timestamps, actual sample
counts, overrun flags and explicit sample-zero provenance. Relate bladeRF's
counter to the X300/GPS reference using a qualified physical event/clock path.
Correlation can locate a waveform within a recording; its local sample index
alone does not give an absolute GPS epoch. CLI success and exact raw file length
are insufficient to prove continuity.
The metadata requirements follow [Nuand's timestamped RX interface](https://www.nuand.com/libbladeRF-doc/v2.5.0/sync_rx_meta.html).

## 6. Remaining waveform and dataset qualification

| Work | Required evidence |
|---|---|
| Stable input and live startup | Repeat complete clean-reference startup with the current PRN set; missing navigation must continue to refuse |
| RF start and path delay | Measured GPS bias, drift, jitter and uncertainty at a declared receiver plane |
| Long continuous runs | No loss of sample continuity; bounded render/queue margin, current navigation and logged reference health |
| Navigation fidelity | SF4/SF5 schedule, ionosphere/UTC integration, emitted-word history and independent parity/bit comparisons |
| Code/carrier fidelity | Per-PRN code delay, Doppler and carrier trajectory; NCO/interpolation/model-error characterization |
| Receiver behavior | Separate acquisition, RAWX/code/carrier validity, half-cycle/lock, satellite use and PVT outcomes |
| Dataset | Authentic-only, generated-only and combined controls with calibrated power and simultaneous reference observations |

SFRBX arrives after a complete subframe; it cannot reveal unknown live bits in
advance. Reconstructing ephemerides is not full live bit identity. Real-time
coherent matching needs prediction, later comparison and invalidation around
unpredictable updates. A guaranteed delayed replay is a different experiment.

PPS does not establish satellite carrier phase. RAWX is receiver-clock-relative
and includes ambiguities/hardware biases; do not copy pseudorange or carrier
cycles straight into the generator. Specify tolerances, coherence duration and
measurement uncertainty before accepting per-PRN matches. At 2.5 MS/s the sample
interval is 400 ns; finer delay estimation requires qualified interpolation or
correlation rather than integer shifting alone.

Keep model epochs, hardware time, transport acceptance and measured RF arrival
distinct in every dataset. No one-hour/overnight RF soak or completed live-sky
mixtracking result is claimed.

## 7. Documentation and evidence map

- [Runbook](../command.md): receiver message setup, USB/TCP bridge, automatic
  F9P-time offline revive and conditional fixed-scenario TX commands.
- [CLI and architecture](x300-usage.md): timing, compatibility and manifest rules.
- [2026-09-21 evidence](x300-f9p-start-validation-2026-09-21.md): implementation,
  independent ephemeris comparison and early-source caveats.
- [2026-09-22 evidence](x300-f9p-rf-timing-qualification-2026-09-22.md): receiver
  identities, latest failures, PPS retry, bladeRF baseline and missing wiring.
- [Revive semantics](synth-revive-mode.md) and [clone semantics](synth-clone-mode.md).
- [Historical timing review](x300-long-run-timing-rubidium-review.md): retained
  pre-refactor investigation; its line numbers/options are not current commands.

The superseded software-only alignment plan, review checklist, hardware
refactor proposal, and old X300/bladeRF OTA setup recipes were removed. Their
remaining requirements are consolidated here. The historical offline datasets
and other waveform tools retain their own documentation.
