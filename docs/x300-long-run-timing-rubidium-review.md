**X300 long-run timing review — PRS10 and ZED-F9P**

**Historical investigation, not current operating instructions.** The source
findings, line numbers, calibration examples and diagnostic results below refer
to the 2026-09-19 checkout. PPS startup, shared continuity handling, navigation
boundaries and the F9P input have since been implemented; Trimble/ppm X300
options were removed. GPS-labeled edge association and physical RF validation
remain outstanding. Use the [current plan](x300-live-sky-mixtracking-research-plan.md),
[X300 usage](x300-usage.md) and [runbook](../command.md).

Reviewed 2026-09-19 at commit `7c2f6c0980b2827ef5381890fc5966a21699e279`.
This is an investigation of the current code, with offline diagnostics. No
transmitter source was changed and no USRP was opened or transmitted through.
The user confirmed that the PRS10C currently runs without an external PPS input
and proposed adding a ZED-F9P. SRS publishes the relevant documentation under
PRS10 and lists an option C; the hardware discussion assumes this PRS10 family.
The exact breakout board, output conditioning, calibration, and firmware remain
unverified.

The PRS10 should improve X300 frequency stability when warmed up, operating
correctly, and selected as the external reference. Adding a GPS-derived PPS input
to the PRS10 can also provide long-term GPS discipline. Reliable transmission
still requires software that binds the generated GPS epoch to a hardware PPS
edge and preserves uninterrupted samples. The present application does not
implement that PPS binding.

The untracked `note1.txt` mentions failure to solve PVT after a cold start around
three minutes into an experiment. It does not identify the command, clock state,
UHD events, or receiver measurements. None of the findings below establishes the
cause of that particular observation.

**What the existing time model gets right**

The default sample rate is 2.5 MS/s. The application reads back UHD's configured
rate and uses it for generation, sample counts, and sample duration
(`player/x300tx.cpp:2968–3024`). This handles rate coercion. UHD's reported rate
is a configured rate, not an independent measurement of oscillator accuracy.

The block-time model is:

```text
G_model(N) = G0 + N / F_UHD * (1 + p * 10^-6)
```

`N` is the cumulative generated sample count and `p` is `--gps-time-ppm`.
`getGpsTimeAtSampleOffset()` handles GPS week rollover without millisecond
rounding (`player/x300tx.cpp:417–453`). Block-end time is correctly passed to
`computeCodePhase()`: that function derives initial code phase from the previous
range epoch and then stores the new endpoint (`gpssim.c:2469–2502`).

For an uninterrupted stream, let the physical rate be
`F_actual = F_UHD * (1 + epsilon)`. The model epoch then advances relative to
real elapsed time at the factor `(1 + epsilon) * (1 + p * 10^-6)`. To first order,
the accumulating timing error is `(epsilon + p * 10^-6) * elapsed`, plus the
initial offset and fixed signal-path delay. This is a model-time calculation;
the waveform-level limitation of nonzero `p` is described below.

For scale, a residual frequency error of 1 ppb accumulates 180 ns in three
minutes and 3.6 us in one hour. Multiplying by the speed of light gives about
54 m and 1.08 km of common pseudorange-equivalent error. These are calculated
equivalents, not predicted position errors or universal receiver rejection
thresholds. A receiver observing an entirely simulated constellation can absorb
common bias into its clock solution; mixing independently timed signal groups
places different demands on consistency.

**Code findings, in priority order**

1. **P1 — External PPS is selected but never used to establish the epoch.**
   `set_clock_source()` and `set_time_source()` are called at lines 2882–2883.
   The normal transmitter then calls `set_time_now(0)` at line 3615; the separate
   matched-code path does the same at line 1539. Neither path waits for PPS,
   labels a captured PPS edge, or calls `set_time_next_pps()`. The Trimble path
   schedules from TCP tag receipt plus an estimated lead and host preparation
   time (lines 3047–3052 and 3557–3592). Variable message and control latency
   therefore remains in the initial offset. Connecting PPS and passing
   `--time-source external` cannot by itself fix this.

   UHD provides a separate operation to latch device time at the next PPS. Its
   time label must belong to that next edge, and the latch must be verified
   before scheduling dependent operations. [UHD time API](https://files.ettus.com/manual/classuhd_1_1usrp_1_1multi__usrp.html)

2. **P1 — The normal GPS path continues after losing sample continuity.**
   Lines 3699–3715 consume only one asynchronous event per block, count only
   ordinary underflows, and continue after them. Packet sequence errors and
   in-packet underflows are ignored. Only the first chunk carries a timestamp;
   later samples have no absolute schedule. After a gap, sample index alone
   cannot establish when a sample actually left the radio. `emitted_samples`
   records samples accepted by `send()`, not verified uninterrupted RF output.
   A reference clock does not repair the gap.

   These are distinct UHD events for buffer starvation, packet loss, and late
   timestamps. The matched-code path already handles more of them as fatal
   (`accountMatchedAsyncEvent()`, lines 1300 onward). The normal path should also
   drain and classify every event, invalidate the run on continuity loss, and
   return failure. It currently returns zero even after its fatal time-error
   branch. [UHD asynchronous event definitions](https://files.ettus.com/manual/structuhd_1_1async__metadata__t.html)

3. **P1 for affected nonzero corrections — The 30-second refresh can be skipped.**
   The normal path checks rounded TOW for exact divisibility by 300
   (`player/x300tx.cpp:3718–3725`, also during prebuffering). A scaled time step
   can skip the integer that satisfies this test. An offline sweep reproduced
   a 59.9-second interval between refreshes with allowed positive ppm values.
   During that gap, the existing `computeCodePhase()` index calculation reaches
   `dwrd[60]`, outside the valid 0–59 array (`gpssim.h:41`, `gpssim.c:2489–2499`).
   The diagnostic guarded the access rather than executing undefined behavior.

   Use an explicit next-boundary or frame-crossing scheduler, with navigation
   words available before their samples are rendered, and bound-check word
   access. This was reproduced at +100 and +1000 ppm; it was not reproduced
   in the tested 24-hour zero-ppm or +0.002894118 ppm cases. It is not evidence
   for a three-minute failure at the default settings.

4. **P2 — Nonzero ppm does not produce a continuous time-scaled waveform.**
   Block endpoints use the scaled time, while `delt` remains `1 / samp_freq`
   (`player/x300tx.cpp:2992`). Code and carrier advance with that unscaled step
   (`player/x300tx.cpp:524–552`). `computeCodePhase()` resets the code phase
   from the scaled epoch at the next block. In a constant-range test, +1 ppm
   caused a roughly 0.1023-chip / 100 ns code-phase jump every 100 ms.
   +100 ppm caused about 10.23 chips / 10 us. Carrier progression also needs
   a coherent treatment; changing timestamps alone is incomplete compensation.

   This qualified the earlier, now-retired review checklist's conclusion that
   the drift knob was complete.
   For a calibrated external reference, begin a new calibration with
   `--gps-time-ppm 0`. Do not carry an internal-clock or bladeRF correction over
   unchanged. For example, +0.002894118 ppm alone advances model time by about
   10.419 us/hour relative to a perfect physical clock.

5. **P2 — The host ring does not provide an independent transmit reserve.**
   The normal loop generates another full block before sending the oldest
   buffered block (`player/x300tx.cpp:3661–3696`). Samples waiting in host RAM
   cannot feed the device while that same thread is generating or refreshing
   navigation state. The actual reserve is what has already reached UHD/device
   queues. Increasing `--prebuffer` alone does not establish a guaranteed
   underrun margin. Separate generation from the sender or maintain a measured
   hardware-time lead; record worst-case generation and refresh latency.

6. **P2 — Zero-length sends are not handled in the normal path.**
   The first send clears timed/start metadata even if `send()` returns zero
   (`player/x300tx.cpp:3635–3642`). Both send loops can retry indefinitely without
   checking stop state or a no-progress deadline. UHD permits short sends on
   timeout. Preserve first-sample metadata until progress and bound retries.
   [UHD send contract](https://files.ettus.com/manual/classuhd_1_1tx__streamer.html)

7. **P2 — Startup and long-run validity checks are incomplete.**
   The application logs requested clock/time source strings without reading
   them back and has no application-level reference-lock or PPS monitoring.
   UHD may perform initialization checks, but that is not continuous monitoring.
   In `-n` mode, if preparation overruns, the start is moved to a 20 ms future
   margin without regenerating IQ for the new epoch (lines 3594–3610). It also
   measures preparation using wall time rather than monotonic elapsed time.
   The Trimble path correctly rejects a stale start.

   For long sessions, file ephemerides eventually run out of coverage; no
   ordinary-path age watchdog stops continued extrapolation. RTCM disconnects
   explicitly retain cached ephemerides indefinitely (lines 735–745). Several
   start modes also shift file ephemeris timestamps (lines 3105–3128), which
   does not make old orbital data current. Frequency and timing improvements
   cannot validate geometry or navigation contents.

**Using the PRS10 with the X300**

The PRS10's specified 10 MHz output is a 0.5 Vrms sine wave into 50 ohms
(approximately +7 dBm). Its shipment frequency accuracy is specified as
±5e-11; this is not a measurement of the user's present unit. SRS also lists
option C as an improved aging option. [SRS PRS10 specifications](https://www.thinksrs.com/products/PRS10.htm)

The X300 accepts a sinusoidal 10 MHz external reference at `REF IN`; its stated
maximum reference input is +15 dBm. The nominal PRS10 output is therefore a
suitable reference level. Verify the actual breakout output and X300
`ref_locked` state. [X300 hardware manual](https://files.ettus.com/manual/page_usrp_x3x0.html)

For an initial 10 MHz-only experiment, the existing clock-related options are:

```text
--clock-source external --time-source internal --gps-time-ppm 0
```

These are additions to a separately validated experiment command. They improve
the frequency basis; startup still follows the existing software timing path.
Do not infer absolute GPS alignment from a rubidium-lock indication.

**Adding the ZED-F9P**

Use a dedicated real-sky timing receiver so cold-starting the receiver under
test does not reset the reference. The proposed connection is:

```text
Real-sky antenna -> ZED-F9P
                    | TIMEPULSE, configured as GPS-aligned 1 PPS
                    v
              level/edge conditioning -> PRS10 1 PPS IN
                                           | 10 MHz OUT -> X300 REF IN
                                           | 1 PPS OUT -> pulse buffer if needed
                                           |                -> X300 PPS/TRIG IN

ZED-F9P UBX time messages -------------------------------> host epoch mapping
```

The F9P supports configurable TIMEPULSE and `UBX-TIM-TP`, which identifies the
next pulse. Configure 1 Hz, rising edge, GPS time grid, alignment to TOW, and
valid-time-dependent output. Use a fixed, accurate antenna position and account
for antenna/cable delay. Serial message receipt is not the PPS edge.
[F9P integration manual, section 3.13](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)

For example, the F9P-02B specifies time-pulse accuracy of 30 ns RMS and 60 ns at
99%; those figures are not an end-to-end guarantee for this setup.
[F9P-02B datasheet](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-02B_DataSheet_UBX-21023276.pdf)

The PRS10 can discipline its 10 MHz and PPS outputs from an external PPS. Its
default algorithm initially aligns after 256 consecutive acceptable pulses;
the default PT8 natural loop time constant is about 2.25 hours. Allow and
measure settling. Read `ID?`, `ST?`, `PL?`, `PT?`, and a sequence of `TT?`
results to establish configuration and behavior; enabling the PLL is not
proof that discipline has settled. [PRS10 manual, pages 17 and 31–35](https://www.thinksrs.com/downloads/pdfs/manuals/PRS10m.pdf)

Electrical verification is still needed. A bare F9P provides supply-referenced
logic, commonly around 3.3 V on a breakout; its digital output is not a general
coax driver. Check the actual board and use a suitable level translator/buffer
for the PRS10 input where required. [F9P-01B electrical specifications](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-01B_DataSheet_UBX-17051259.pdf)

The PRS10 and PRBB PPS outputs have 1 kohm output impedance, so cable loading
can slow the edge. A pulse buffer may be needed; inspect the loaded waveform
at the destination and include its delay in calibration.
[SRS PPS output note](https://www.thinksrs.com/downloads/pdfs/other%20stuff/PRS10_FS725_1pps_output.pdf)
Ettus documents a 5 Vpp PPS input waveform for X300.
[X300 PPS input](https://kb.ettus.com/X300/X310)

**Required hardware-aligned software contract**

Use a verified pair `(H_ref, G_ref)`: hardware time latched at a PPS edge and the
GPS week/TOW belonging to that edge. For a future hardware start `H_start`,
represent sample `N` at:

```text
G_model(N) = G_ref + (H_start - H_ref) + N / F_UHD + delta_cal
```

`delta_cal` represents measured fixed timing offsets at the chosen DUT reference
plane, including PPS distribution and RF path delay. The generator's `grx` is
the modeled receiver epoch, so distinguish the SDR timestamp from arrival time
at that plane. At zero residual frequency error, sample time then follows the
external clock continuously.

The implementation should configure and read back sources, verify reference
lock and PPS edges, latch and verify device time at a correctly labeled future
PPS, then precompute IQ and schedule a sufficiently future start. If preparation
misses the target, select a new target and regenerate the corresponding samples.
Monitor PPS/reference status without stepping the hardware clock during the
continuous burst. PPS provides edge alignment; the 10 MHz reference governs
rate. [UHD synchronization model](https://files.ettus.com/manual/page_sync.html)

A PPS derived from the same free-running rubidium as the 10 MHz cannot
independently establish its accuracy against GPS. The F9P provides the external
time reference. Its time messages supply the second label, which a pulse alone
does not contain.

**Verification completed**

The current source builds with UHD 4.10.0.0. The shell's default macOS 27 SDK
failed to link with the installed Xcode 17 toolchain; selecting its matching
macOS 26 SDK for this invocation passed without source changes:

```bash
rtk proxy env SDKROOT=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.0.sdk make x300tx
```

An offline C++ probe included the actual `x300tx.cpp` helpers and linked the
current generator objects. It never constructed a UHD device. Diagnostic source
and executable were retained in `/tmp/x300-timing-review.PLsPba/` for this session.

| Diagnostic | Result |
|---|---|
| 24-hour sample-count increment across GPS week rollover | Exactly 86400 seconds in the reported result |
| 24-hour refresh sweep, 0 ppm | 2880 refreshes; all gaps 30 seconds |
| 24-hour refresh sweep, +0.002894118 ppm | 2880 refreshes; all gaps 30 seconds |
| Constant-range block boundary, 0 ppm | About 0.006 ns numerical difference |
| Constant-range block boundary, +1 ppm | About +100.007 ns jump |
| Constant-range block boundary, +100 ppm | About +9999.995 ns jump |
| +1000 ppm, initial week 2400 / TOW 120000.000580 | Invalid next word index 60 at nominal elapsed 10549.7 s |
| +100 ppm, same initial epoch | Invalid next word index 60 at nominal elapsed 58494.4 s |

The long sweeps advance timing/state rapidly; they are not 24-hour IQ renders
or hardware soak tests. The nav-index diagnostic used a constant 70 ms range
delay and checked the beginning-of-block access. Hardware drift, PPS waveform
quality, streaming headroom, and receiver PVT performance remain unmeasured.

**Validation sequence for the physical setup**

1. Establish PRS10 health and warm-up, measure the loaded output levels, and
   confirm X300 external-reference lock without transmitting.
2. Configure the independent F9P timing output, discipline the PRS10, and log
   settling and loss-of-reference behavior. Measure offset and drift against
   the GPS reference rather than relying on a single lock indicator.
3. Implement PPS epoch binding and make any continuity loss invalidate the
   normal GPS run. Correct refresh scheduling and either repair waveform time
   scaling or constrain the calibrated external-reference path to zero ppm.
4. In the existing conducted/shielded test setup, compare internal-reference,
   free-running PRS10, and GPS-disciplined PRS10 runs. Keep scenario, PRNs,
   position, receiver configuration, and RF path controlled. Start with the
   reported three-minute window, then extend to an hour and overnight.
5. Record the exact command, UHD version and all TX events, clock/PPS status,
   generation latency, ephemeris age, receiver raw measurements, satellite use
   flags, and PVT. Cold-start the DUT while the independent reference continues.
   Separate initial bias, smooth drift, sudden gaps, and PRN-specific residuals.

Acceptance requires a valid clock/epoch relationship, uninterrupted samples,
consistent navigation data, and successful receiver behavior for the intended
scenario. Tracking alone, zero reported ordinary underflows, or a stable 10 MHz
signal does not establish all of these.
