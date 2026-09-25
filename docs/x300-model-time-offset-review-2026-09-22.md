# X300 model-time review: software limit and F9P PPS integration

Recommendation: use the time-reference F9P's physical PPS at the X300 and
implement explicit association of that hardware edge with its GPS epoch.
Software can still improve diagnostics and calibration, but the present
one-way UBX arrival stream cannot determine its own unknown fixed delay.
Simply connecting a different PPS source does not change the current startup
algorithm: it will still estimate the GPS label from TCP arrival time.

This is a source review and offline test result. No production code, receiver
configuration, wiring, or RF operation was changed for this review.

## Current timing path

| Stage | Source | Assessment |
|---|---|---|
| Parse receiver epoch | `player/ubx_receiver.cpp`, `Decoder::message()` | Uses NAV-TIMEGPS `iTOW + fTOW` with validity, progression and reset checks. NAV-CLOCK bias is not subtracted again. |
| Estimate GPS at a host instant | `player/ubx_receiver.cpp`, `estimateGpsNow()` | Uses the lowest observed delivery delay in the last five seconds. Suppresses variable queuing but retains the unknown minimum delay. |
| Establish local hardware seconds | `player/x300_timing.hpp`, `synchronize()`; `player/x300tx.cpp`, `UhdRadio::latchNextPps()` | Arms `set_time_next_pps(0)` and verifies zero followed by a one-second increment. No GPS week/TOW is assigned to that edge. |
| Relate host and hardware clocks | `player/x300_live_time.hpp`, `observeClock()` | Chooses the narrowest of eight before/after brackets. Assumes the hardware read lies at the bracket midpoint. This reduces jitter but does not measure network asymmetry. |
| Assign the model epoch | `player/x300_live_time.hpp`, `planLiveStart()`; `player/x300tx.cpp`, `prepareLiveStart()` | Extrapolates the arrival-based GPS estimate to a future hardware second; adds signed model correction once, before rendering. |
| Advance the signal | `player/x300tx.cpp`, normal `render` lambda; `player/x300_timing.hpp`, `sampleTime()` | Uses cumulative sample count divided by actual UHD rate. The model offset changes the initial epoch, not the sample rate or hardware deadline. |
| Start and maintain transmission | `player/x300_timing.hpp`, `transmit()` | Uses a timed first sample and preserves unsent IQ suffixes. Rejects stale starts and reported continuity faults. |

I found no sign error or double application of the model offset in this path.
Increasing `--start-lead-sec` gives preparation time; increasing `--prebuffer`
gives queued IQ. Neither identifies the missing GPS epoch offset.

In simplified notation the current mapping is:

```text
G_model(n) = estimated_GPS_from_UBX_arrivals_at_H_start
             + delivery_correction + TX_path_delay - sky_path_delay
             + model_time_offset + n / actual_sample_rate
```

The data from v1/v2/v3 show why the initial estimate is the priority. Their
effective delays are 36.930683, 37.932086 and 37.444772 ms, while the relative
rate is only approximately 0.4–0.55 ns/s. Logged hardware-query half widths
are approximately 115–142 microseconds. A smaller query bracket would help,
but would not determine the separate receiver/USB/TCP delay.

The [cross-capture analysis](x300-mixed-timing-v2-v3-2026-09-22.md) also finds
that adding the measured delay to the model-start fraction gives an inferred
arrival phase near 224.943 ms into the GPS second in every run. This supports
a stable local PPS with a varying estimated GPS label. It is not an independent
measurement of the RF start or a universal phase constant.

## What software alone can still do

For receiver epoch `G_i`, host arrival `A_i`, host-to-GPS clock offset `B`, and
delivery delay `L_i`, the observation has the form:

```text
A_i = G_i + B + L_i
min(A_i - G_i) = B + min(L_i)
```

Arrival filtering observes their sum. It cannot separate clock offset from
the fixed part of delivery latency without another timing observation or a
calibration assumption. Receiver `tAcc` describes its time solution, not
when the host received the message. The existing
`testMinimumDelayEstimate()` explicitly demonstrates that a 40 ms fixed floor
survives filtering until a known correction is supplied.

Useful software-only improvements remain possible:

- Record a longer bounded startup history and compare lower-delay estimates
  over several windows; reject unstable starts instead of only printing jitter.
  A longer window needs clock-drift handling and still cannot recover the
  fixed latency floor.
- Record timing-message arrival diagnostics at the USB bridge as well as the
  client. This can localize buffering and serial/TCP effects, but does not
  timestamp the physical PPS edge. The bridge already uses `TCP_NODELAY` and
  disconnects slow clients, so those are not missing optimizations.
- Calibrate the GPS phase of an unchanged local PPS source and reuse that
  phase, with expiry and source/reset detection, rather than reuse a fixed
  arrival-delay correction. The inferred 224.943 ms phase suggests a possible
  interim approach. It needs external calibration evidence and loses validity
  when the source phase or setup changes; three retrospective fits do not
  validate it for future startups.

These would improve repeatability or observability. The largest improvement
comes from supplying a physical GPS-referenced edge and using it directly in
the startup mapping.

## Hardware path and remaining software work

The practical next configuration is:

```text
PRS10 10 MHz --------------------------------> X300 REF IN
Reference F9P TIMEPULSE -> suitable interface -> X300 PPS/TRIG IN
Same F9P UBX time/pulse metadata -------------> host
```

Keep `clock_source=external` and `time_source=external`. Use the reference F9P
tracking authentic sky for the pulse and its label. The 10 MHz reference sets
the continuous rate; PPS establishes the boundary. PPS does not continuously
correct an independent rubidium frequency or guarantee carrier-phase identity.
Ettus recommends coherent reference signals; with separate F9P PPS and PRS10
frequency, verify their relative phase/interval behavior. A common disciplined
10 MHz/PPS source is the longer-term way to make both references coherent.
See [Ettus synchronization](https://files.ettus.com/manual/page_sync.html).

The missing software feature is a distinct PPS-associated startup mode:

1. Validate the configured physical pulse source, one-second rate, selected
   edge, and time base. Prefer GPS time grid for this GPS generator.
2. Retain a history of valid `TIM-TP` labels and hardware PPS observations.
   Associate the intended pulse with the USRP edge using checked label/edge
   sequences and an explicit delivery bound/margin. Reject stale, skipped or
   ambiguous associations; never guess between adjacent seconds.
3. Latch a local hardware epoch and record the pair `(H_ref, G_ref)`. Absolute
   GPS seconds inside UHD are optional; local hardware time plus the explicit
   GPS week/TOW pair is sufficient.
4. Verify subsequent labeled edges and derive sample zero directly from that
   pair. USB arrival time then helps identify the label and detect stale data;
   it no longer sets the fractional GPS phase.
5. Preserve the existing fixed-deadline and cumulative-sample renderer. Monitor
   PPS-to-clock drift; do not step the hardware or model time during a burst.

The resulting model is:

```text
G_model(n) = G_ref + (H_start - H_ref) + n / actual_sample_rate
             + calibrated_RF_path_correction
```

`G_ref` must include the defined edge/cable/receiver correction convention.
This formulation assumes the existing one-second-per-hardware-second rate
model; long-run frequency error must still be monitored or disciplined.
Do not carry the old approximately 37 ms transport compensation into this
mode. Measure any remaining cable, digital/RF-chain and comparison-plane
delay separately after the edge association is established. A verified PPS
association and a verified RF arrival require separate manifest evidence.

The code already decodes raw `TIM-TP` and `TIM-TM2` fields, but neither controls
`planLiveStart()`. There is no implemented PPS/GPS label-association state
machine. The bridge's five-second diagnostic poll of `TIM-TP` also does not
guarantee the continuous pulse-label stream required by the proposed mode.

One concrete integration issue is visible in all three recorded initial
snapshots: `TIM-TP.flags = 27 (0x1b)` and `refInfo = 63 (0x3f)`. Under the
[HPG 1.32 interface description, section 3.19.2](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf),
these specify UTC time base, UTC available, and invalid quantization-error
data; the zero `qErr` must not be treated as a measured zero error. `TIM-TP`
labels the next pulse, not the most recently observed PPS. Configure the GPS
grid or perform a validated UTC-to-GPS conversion using the receiver's valid
leap-second information. Neither treating the UTC label as GPS nor naively
pairing it with the last PPS is correct.

## Electrical and accuracy limits

The [X300 manual](https://files.ettus.com/manual/page_usrp_x3x0.html) specifies
a 5 Vpp square wave for its PPS input. The bare ZED-F9P-04B TIMEPULSE output is
VCC-referenced and its datasheet specifies 4 mA drive/sink capability. Inspect
the carrier board's PPS circuitry and verify the loaded waveform; use suitable
buffering/level translation if the pad does not meet the X300 input requirement.
A common ground and the correct rising-edge configuration are part of this
interface. A raw module pad is not automatically a qualified coax driver.

The [F9P-04B datasheet](https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf)
lists time-pulse accuracy of 30 ns RMS / 60 ns at 99%. That is a receiver
specification, not a promise of end-to-end generated-signal accuracy. The
[integration manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
recommends full-setup timing calibration. FPGA edge sampling, pulse/cable delay,
radio group delay, position/model errors and signal continuity remain relevant.
The user's 1-to-3 m free-space distance change contributes only about 6.7 ns.

The `EVENT_CODE_SEQ_ERROR` in the captures is a separate transport-continuity
problem. Changing PPS will not establish that it is fixed or explain the
earlier carrier-phase loss. Validate continuity and the remaining mixed-signal
timing residual independently after startup association is implemented.

## Verification performed

`rtk proxy make test-ubx test-x300-timing` passed on the current checkout:

- C++ UBX/LNAV parser, time validity, reset, minimum-delay estimator, signed
  model offsets and week rollover.
- Production time-only revive rendering with a fake radio, finite/continuous
  operation, deadlines, source health and manifests.
- C++ sample timeline, PPS, queue and transport-fault tests.
- 13 UBX CLI and 8 hardware-timing CLI tests.

These tests verify the present software contract. They do not validate a
physical F9P-to-X300 pulse connection or an unimplemented PPS-associated mode.
