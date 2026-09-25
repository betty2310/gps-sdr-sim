# ZED-F9P / X300 estimated-start implementation and hardware checks

**Historical session: 2026-09-21.** Receiver ports, RF wiring and source
conditions below belong to that session. For the present setup and subsequent
results, see the [2026-09-22 qualification](x300-f9p-rf-timing-qualification-2026-09-22.md)
and [current runbook](../command.md). The earlier passes must not be promoted to
clean-sky RF or GPS-alignment verification.

The X300 player now replaces Trimble input with a ZED-F9P UBX/TCP source.
Receiver acquisition, GPS LNAV decoding and fixed-deadline startup are
implemented and tested. **Physical RF start and absolute GPS alignment have
not been measured.** The reported RF B → DUT connection is a direct cable
without attenuation, so this session did not create a TX streamer or send RF.

**Final follow-up in this session:** after the other transmitter was switched off, the
reference briefly recovered a valid 3D fix, then lost it again. The follow-up
startup attempts all refused to arm during receiver qualification. The earlier
hardware passes below used the earlier input conditions and do not validate
startup from the recovered live-sky feed.

## Connected hardware and input

| Role | Connection | Observed identity |
|---|---|---|
| Reference, described by the user as clean sky | `/dev/cu.usbmodem11301`, USB location `1-1.3`; temporary bridge `127.0.0.1:5019` | ZED-F9P, HPG 1.32, protocol 27.31 |
| DUT connected to RF B | `/dev/cu.usbmodem111301`, USB location `1-1.1.3`; temporary bridge `127.0.0.1:5020` | ZED-F9P, HPG 1.32, protocol 27.31 |
| Transmitter | X300 `192.168.10.2`, serial `32244B4`, TX channel 1 = `B:0` (`A:0 B:0` readback) | UHD 4.10.0.0, 200 MHz master clock; 2.5 MS/s actual rate |
| Timing | External reference and external PPS, following the existing PRS10 wiring | Reference lock and PPS time-zero/next-second latch passed |

No receiver configuration was changed. The bridge only polled `MON-VER`; the
receiver's existing stream supplied the other messages. The reference capture
contained valid `NAV-TIMEGPS`, 3D `NAV-PVT`, `NAV-CLOCK`, `RXM-RAWX` and GPS L1
`RXM-SFRBX`. The initial 40-second capture decoded complete ephemerides for
PRNs 3, 4, 8, 9, 16, 26, 27, 28, 31 and 32, with no UBX checksum errors.
Other NMEA/UBX traffic was preserved in the raw recording.

Both receivers reported roughly 10 m/s and nearby moving positions before this
session sent any RF; the reference later reported 11.36 m/s. This is an observed
input condition, not independent proof of a clean static sky reference. The
reference's leap-second validity flag was false, so UTC conversion remains
unavailable. A separate receive-only comparison found approximately +77.33 s
relative to the host clock **if** the receiver's unvalidated 18-second offset
is used. Neither the host clock nor that conversion was qualified as ground
truth, and no such offset was applied to the generator. Source authenticity,
static geometry and absolute epoch still require qualification.

## Follow-up after the other transmitter was switched off

The user subsequently confirmed switching off another transmitter. The first
40-second follow-up capture found **no valid fix on either receiver**, no RAWX
observations and no new SFRBX. Both continued reporting valid GPS week/TOW;
those flags alone did not establish a recovered live-sky reference. The next
90-second startup attempt correctly failed with
`UBX warmup: valid fresh 3D NAV-PVT position required`, before opening UHD.
All 90 PVT epochs were invalid. PRN 26 appeared only in the last five RAWX
epochs, at 21–25 dB-Hz, with no SFRBX received.

At the user's request, the first F9P was scanned again for 40 seconds. All 40
PVT epochs now reported a valid 3D fix with eight satellites used. The final
position was 21.0045369° N, 105.8464613° E, ellipsoid height 24.347 m; reported
horizontal accuracy was 13.142 m and speed 0.133 m/s. GPS week/TOW was
2437:124283.000381982, with receiver-reported `tAcc` 21 ns. Leap-second validity
was still false; this is a recorded GPS epoch, not a qualified UTC conversion
or a measurement of USB/TCP timing accuracy.

The rescan decoded complete GPS ephemerides for PRNs 3, 16 and 26. Observations
were intermittent: 35 RAWX epochs contained six measurements, two contained
seven, and the remaining epochs contained two, zero and one respectively.
The second F9P remained without a fix or RAWX observations during a subsequent
20-second check. These results distinguish reference recovery from the earlier
shared moving solution; they do not prove physical RF timing or long-run input
stability.

The subsequent 90-second startup check with automatic PRN selection also
refused to arm: PRNs 3 and 26 had complete ephemerides, but observed PRN 27
had no SF1–SF3 in that capture. The recorded error was
`UBX warmup: PRN 27 missing SF1,SF2,SF3`; no samples were generated or sent and
UHD was not opened. The final reference fix remained valid, with speed
0.032 m/s and reported horizontal accuracy 22.089 m. A valid position alone
therefore did not bypass the navigation requirements.

A final focused attempt explicitly selected PRNs 3 and 26, which had been
complete in the previous recording. During this new 90-second acquisition,
the reference lost its valid fix and ended with zero RAWX observations,
reported horizontal accuracy 440.958 m and incomplete ephemerides. Startup
again failed with `UBX warmup: valid fresh 3D NAV-PVT position required`.
Its manifest is `after-other-tx-off/start-prn3-26.json`. No follow-up attempt
opened UHD, generated IQ or sent samples; the earlier hardware timing checks
must not be relabeled as successful checks of this later reference feed.
The antenna/feed must be made stable before a fresh startup measurement.

The captured `MON-RF` antenna status and power fields both decode as unknown,
so they cannot establish whether the antenna feed is powered. See the
[HPG 1.32 interface description, page 132](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf#page=132).
Follow-up recordings, manifests and decoded summaries are under
`analysis-output/runs/f9p-x300-20260921/after-other-tx-off/`; initial failures and
the later receiver recovery are retained as separate captures. Receiver
configuration was not changed.

## Hardware results with no TX streamer

`--check-sync` verified external reference lock, latched local hardware time
zero at PPS, and observed the next PPS at exactly `1.000000000 s` in the device
time representation. That is a local epoch check, not a GPS-labeled edge.

`--check-start` then used live receiver data and actual device time to choose
the deadline **before** dated waveform initialization and prebuffering:

| Attempt | Result | Prepared samples | Prebuffer wall time | Remaining submission margin | Mapping |
|---|---|---:|---:|---:|---|
| 01, lead 3 s / queue 5 | Passed | 1,250,000 | 25.103 ms | 3.966742 s | Hardware 5 s → GPS 2437:121796.968144747 |
| 02, lead 2 s / queue 50 | Passed | 12,482,749 | 250.540 ms | 2.743105 s | Hardware 4 s → GPS 2437:122036.206900565 |
| 03, lead 2 s / queue 50 | Refused during 60 s receiver warmup | 0 | — | — | No valid plan; UHD was not opened |
| 04, lead 2 s / queue 50, 90 s warmup limit | Passed after reference recovery | 12,479,297 | 306.925 ms | 2.679505 s | Hardware 4 s → GPS 2437:122846.408281595 |

Attempt 02 crosses a navigation-frame boundary, which shortens one queued
block. The queue count therefore does not imply exactly five seconds of IQ.
Attempt 03 received only SF2 and SF3 for PRN 3 and could not complete SF1–SF3;
other PRNs were also incomplete. It recorded the failure and made no send
attempt. Attempt 04 acquired 11 usable GPS PRNs and passed. These three passes
and one refusal do **not** establish reliable startup under all present input
conditions.

The first two UHD query brackets were 195.958 µs and 209.291 µs wide. Recent
receiver-arrival variation was 0.480 ms and 0.886 ms respectively. These values
are diagnostics, not bounds on unknown constant USB/TCP delivery delay. Both
plans used the declared zero delivery/path-delay convention. Attempt 04 had a
228.208 µs query bracket and 1.657 ms recent arrival variation. All accepted
sample counts were zero, `send_attempted` was false, and
`gps_alignment_verified` remained false. No burst acknowledgement or physical
RF event was observed or claimed.

## Implementation and regression evidence

- `player/ubx_receiver.*`: bounded UBX framing, documented message/version and
  numeric validation, immutable snapshot copies, background TCP input, original
  byte/monotonic-arrival recording, explicit disconnect failure and offline replay.
- `player/gps_lnav.*`: GPS L1 C/A filtering, issue-consistent SF1–SF3, original
  ephemeris epochs, health/URA/fit metadata and conservative freshness checks.
- `player/x300_live_time.hpp`: receiver epoch → host monotonic → hardware
  estimate, normalized week/fraction arithmetic and explicit unknown absolute
  uncertainty. No host wall-clock time or fixed leap offset controls the waveform.
- `player/x300_timing.hpp`: fixed live deadline preserved through prebuffering,
  abort-before-send on missed deadlines, receiver-health callbacks and generated
  IQ digest. RINEX scenarios retain their independent prebuffer-first behavior.
- `player/x300tx.cpp`: Trimble migration errors, receiver/start-only diagnostics,
  live navigation, frozen reference position, recorded-input rendering and manifests.
- `tools/ubx_tcp_bridge.py`: one serial owner, loopback read-only TCP fanout,
  unchanged receiver settings and disconnection of slow clients.

Independent RTKLIB `convbin` at commit
`71db0ffa0d9735697c6adfd06fdf766d0e5ce807` decoded the same 40-second recording.
All **270 compared ephemeris fields across 10 PRNs** agreed within the oracle
RINEX output's 12-significant-digit rounding (relative tolerance `5e-12`,
absolute `1e-17`). A durable PRN 3 fixture retains words and independent
expected values in `tests/fixtures/f9p-lnav-prn3.json`; source authenticity is
explicitly unverified.

Offline replay of attempt 04, using its recorded receiver stream and the exact
manifest epoch, position, PRNs and sample count, reproduced all **12,479,297
SC16 samples by FNV-1a digest** (`0e4202e465d5516a`). The replay used the final
build and never opened UHD. This validates the dated generator/replay path;
it remains separate from an RF measurement.

Passed regression commands:

```sh
make test-ubx
make test-x300-timing
make test-x300tx-matched
make test
make test-bladetx-matched
```

The new checks cover fragmented/coalesced/corrupt/oversize UBX, invalid and
backlogged time, negative fTOW, week/1024-week rollover, RAWX resets/NaNs,
wrong-signal/version filtering, partial/mixed/stale/unhealthy ephemerides,
deliberately biased delivery assumptions, fixed/missed deadlines, receiver
disconnect, replay rejection for live time, artifact aliases and deterministic
offline IQ. Existing transport/PPS and shared-generator tests remain passing.

## Evidence and remaining measurement

Session artifacts are local and ignored by Git under
`analysis-output/runs/f9p-x300-20260921/`:

- `usb-inventory.json`, `reference.json`, `reference.ubx` and its `.rx.csv`;
- `dut-before.json`, `dut-before.ubx` and its `.rx.csv`;
- `start-check-01.json` through `start-check-04.json`, with each raw UBX/arrival log;
- `rtklib-oracle.nav`, `rtklib-oracle.obs`, `navigation-oracle-check.json`.
- `start-check-04-offline-replay.json`, `hardware-plan-iq-replay-check.json`,
  and `final-build-sha256.json` for the final executable/source checksums.

Both temporary TCP bridges were stopped after validation, releasing the USB
ports. They reported no slow-client disconnections. Restart the reference
bridge using the command in [the usage guide](x300-usage.md) for another check.

An actual RF test still needs a suitable attenuated conducted path, input level
and DC-path qualification for the particular receiver board, and a physical
timing/receive measurement. A successful UHD timed send alone would establish
neither the absolute receiver epoch nor code/carrier agreement at the DUT.
Full live navigation pages, emitted-word history, RAWX-derived correction,
calibrated RF delay and long-duration RF validation remain later plan stages.

Protocol references: [u-blox HPG 1.32 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf),
[UHD synchronization](https://files.ettus.com/manual/page_sync.html), and
[RTKLIB navigation decoder](https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rcvraw.c).
