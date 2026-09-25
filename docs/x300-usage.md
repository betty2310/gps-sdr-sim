# USRP X300 hardware-timed player

Current as of 2026-09-22. Start with [command.md](../command.md) for the current
F9P/X300/bladeRF experiment sequence and
[the qualification report](x300-f9p-rf-timing-qualification-2026-09-22.md) for
measured results. X300 RF B now has an exposed antenna and bladeRF has a receive
antenna. Only receive/no-RF checks are qualified on that arrangement; no X300
RF transmission or absolute GPS alignment has been measured.

`x300tx` generates GPS L1 C/A samples in-process and sends SC16 IQ through UHD.
Normal and matched-code modes share the same PPS synchronization and buffered
sender. The matched-code mode remains navigation-data-free interference;
see its [researcher guide](realtime-code-aligned-matched-code-x300.md).

## Clock wiring and a check without RF

For the PRS10C setup:

- rubidium 10 MHz OUT → X300 REF IN;
- rubidium PPS OUT → X300 PPS/TRIG IN.

Both sources default to `external`. `gpsdo` is also accepted if installed;
`internal` is rejected. A frequency reference alone is insufficient: a working
PPS input is required.

```sh
make x300tx
./x300tx --check-sync --addr 192.168.10.2
```

`--check-sync` checks reference lock, observes a PPS, arms time zero on the next
PPS, verifies the latch, and verifies the following second. It changes the
X300 time counter, so run it when no other application is using the device.
It never tunes TX, creates a TX streamer, or sends samples.

On 2026-09-21 this check passed on the connected X300 at `192.168.10.2`, UHD
4.10.0.0, 200 MHz clock: `ref_locked=true`, verified zero latch, subsequent PPS
`1.000000000 s`. No GPSDO was present. No RF was transmitted during validation.
On 2026-09-22 an initial external-reference PLL failure was followed by a
successful retry; the cause of the initial failure was not established.
Observed subdevices were `A:0 B:0`, with two UBX-40 boards: channel 1 selects
RF B. Recheck that mapping after a hardware/configuration change.

## What is synchronized

The PPS defines a **local hardware epoch**. In a RINEX scenario, after prebuffering the player
chooses a future whole hardware second and attaches that timestamp to the first
accepted packet. There is one start-of-burst timestamp; subsequent samples are
continuous. The scenario epoch supplied by `-t` or `--gps-week/--gps-tow` is the
label assigned to sample zero:

```text
scenario_time(n) = scenario_epoch + n / actual_UHD_sample_rate
hardware_time(n) = timed_start     + n / actual_UHD_sample_rate
```

GPS week rollover is normalized. Render blocks end at navigation-frame
boundaries as needed, including fractional-second starts. Navigation HOW wraps
at the week boundary. Ordinary input ephemeris TOE/TOC are preserved. Revive
intentionally constructs a separately dated synthetic ephemeris from a frozen
historical template; that transformation does not make the input current sky.

**This does not establish live-sky GPS alignment.** Rubidium PPS OUT without a
verified GPS week/TOW label cannot identify the current GPS epoch. The manifest
therefore always records `gps_alignment_verified: false`. Software estimation
does not require direct F9P PPS wiring; absolute qualification still requires
independent epoch/RF measurement and characterized path delay. Neither a timed
TX acknowledgement nor receiver tracking
alone measures code/carrier agreement at the receiver input.

## ZED-F9P over TCP: automatic estimated GPS startup

The X300 live input now uses binary UBX from a ZED-F9P. Trimble/NTRIP options
have been removed from **X300**; the shared RTCM decoder remains available to
bladeRF and the candidate tools. The receiver must observe the independent
reference branch. TCP supplies a receiver epoch estimate; it does not provide
a physical GPS-labeled PPS edge.

For a locally connected, already configured receiver, start the receive-only
bridge (Python `pyserial` is required). The concise
[setup commands](../command.md) also show how to enable TIMEGPS before starting
the bridge; it does not configure that message itself.

```sh
python3 tools/ubx_tcp_bridge.py --serial /dev/cu.usbmodem11301 --port 5019
```

The bridge defaults to loopback, owns the serial port, and polls `MON-VER`,
`SEC-UNIQID`, `MON-RF` and `TIM-TP` every five seconds. These are read-only
queries; it does not change receiver configuration. TCP clients cannot
send configuration/reset commands. A slow client is disconnected once its
pending output exceeds 64 KiB. This setup needs periodic `NAV-TIMEGPS`, `NAV-PVT`,
`RXM-RAWX`, and GPS L1 C/A `RXM-SFRBX`. `NAV-CLOCK` is logged when present.
Original bytes include other messages, including `NAV-SAT`/`NAV-SIG` if enabled.

Receiver reports include the persistent chip `unique_id`, valid-3D and RAWX
epoch counts, per-signal RAWX measurements/flags, and `MON-RF` diagnostics.
Identify the two receivers by chip ID after reconnecting; USB port names and
hub positions alone do not establish their roles. `MON-RF.antenna_status=1`
and `antenna_power=2` both mean unknown, not a powered or correctly connected
antenna. AGC and interference indicators are not calibrated RF power meters.

`time_pulse` retains the `TIM-TP` next-pulse label, its time-base flags,
reference information and quantization-error flags. `time_mark` retains
`TIM-TM2` external-event reports when the receiver emits them. Absent reports
are `null`; older reports retain their ages. The fractional units differ:
`TIM-TP` uses 2^-32 milliseconds, while `TIM-TM2` uses nanoseconds. These
diagnostics never replace `NAV-TIMEGPS` or establish an X300 edge association.
Every receiver report keeps `hardware_edge_association_verified: false`.

Four distinct checks are available:

```sh
# Estimated GPS now at the host query instant; no IQ generation or UHD access.
./x300tx --check-time --ublox-tcp 127.0.0.1:5019 \
  --ublox-warmup-sec 15 --manifest gps-now.json

# Receiver only, no UHD access. Save a replayable stream and decoded report.
./x300tx --check-ublox --ublox-tcp 127.0.0.1:5019 -d 40 \
  --ublox-record reference.ubx --manifest receiver.json

# X300 PPS only, no receiver input and no TX streamer.
./x300tx --check-sync --addr 192.168.10.2

# Receiver/navigation warmup, actual X300 rate and PPS, dated rendering,
# and submission margin. No TX tuning or TX streamer, no RF transmission.
./x300tx --ublox-tcp 127.0.0.1:5019 --check-start \
  --addr 192.168.10.2 --channel 1 --prebuffer 50 --manifest start-check.json
```

`--check-time` waits for ZED-F9P identity and three consecutive valid
`NAV-TIMEGPS` solutions, each with the existing freshness checks (latest age
at most 2.5 seconds). It reports `gps_now` at `host_monotonic_s` using the
least-delayed observation in the last five seconds, elapsed host time, and the
declared `--delivery-delay-sec` estimate. The `time_estimate` object records the
window, observation count, correction and arrival variation. It needs no RINEX,
PVT, RAWX or SFRBX.
GPS week rollover and fractional TOW are preserved. The
[u-blox timing documentation](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
describes solution epochs and time-validity flags; the message timestamp is
not a measurement of USB/TCP delivery delay.

The report uses schema `gps-sdr-sim.ublox-gps-now.v1`, status `estimated` on
success and `unavailable` with `gps_now=null` on failure. Receiver accuracy,
identity and source epochs remain in `receiver`. Absolute uncertainty is
`null`, GPS/hardware edge alignment stays unverified, and UHD/IQ/send fields
remain false/zero. Replay, explicit epochs, waveform options and other check
modes are rejected. This diagnostic cannot schedule transmission or enable
live-time revive; copying its result to `-t` does not establish RF alignment.

The latest reference chip ID is `314852bd18`, last seen at
`/dev/cu.usbmodem11301`; the second receiver is `d44162bdda`, last seen at
`/dev/cu.usbmodem1301`. Both identify as HPG 1.32 / protocol 27.31. These are
dated observations; USB paths can change. Earlier port assignments and the
unattenuated direct-F9P cable are historical. The latest RF arrangement uses
exposed X300/bladeRF antennas, with X300 TX off. The bladeRF receive baseline
passed, but neither antenna coupling nor absolute RF start was measured.
See the [2026-09-22 qualification](x300-f9p-rf-timing-qualification-2026-09-22.md).

In `--ublox-tcp` mode the receiver/navigation reader runs independently from rendering
and sending. Startup requires three consecutive valid GPS time epochs, a fresh
3D PVT solution and RAWX, and complete healthy SF1–SF3 ephemerides for every
selected PRN. `-P` selects the required PRNs; otherwise current GPS L1 C/A RAWX
observations select them. Warmup is bounded by `--ublox-warmup-sec` (60 seconds
by default). The first implementation conservatively limits TOE/TOC age to
two hours and subframe receive age to 90 seconds. Original TOE/TOC, IODE/IODC,
health, URA, fit flag and received-word coverage are retained.

The PVT coordinates are frozen during warmup unless `-l` or `-c` is supplied.
Receiver-reported speed and position accuracy are recorded; a frozen position
does not model motion. Navigation is reconstructed through the existing
generator: authentic SF4/SF5 pages, ionosphere/UTC injection, transmitted-word
history and full bit identity are not implemented by this milestone.

The hardware start is selected **before** dated channel initialization and IQ
rendering. Live mode defaults to a two-second lead, rounded up to a whole local
hardware second. With bracketed host-monotonic/UHD queries and the recent
receiver arrivals, the mapping is:

```text
G_est(H_start) = G_receiver + (H_start - H_at_receipt) + fastest_arrival_residual
               + delivery_delay_estimate
G_model(n)    = G_est(H_start) + n/Fs + tx_path_delay - sky_path_delay
               + model_time_offset
```

GPS-now, offline time-only rendering and hardware planning share a nominal
unit-slope estimator over the last five seconds (at least three observations).
It selects the largest GPS-minus-arrival offset, reducing added queuing delay.
Eight bracketed UHD queries are made; the narrowest bracket defines the
host/hardware association. Its half-width is reported separately from arrival
variation and receiver accuracy. No host-derived ppm slope is applied to IQ.
`NAV-TIMEGPS.fTOW` is retained, negative fractional TOW and week rollover are
normalized, and no host wall clock or hardcoded leap offset drives generation.
UTC conversion is unavailable when the receiver does not validate its leap
offset. `NAV-CLOCK` bias is diagnostic and is not applied a second time.

`--delivery-delay-sec` (0–10), `--tx-path-delay-sec` and
`--sky-path-delay-sec` (0–1) declare delay estimates; all default to zero.
`--delivery-delay-sec` now refers to the remaining delay of the fastest recent
TIMEGPS observation, rather than the median or latest arrival. A previously
measured correction for those older estimators should be requalified.
`--sky-path-delay-sec` is the extra authentic RF-branch delay at the comparison
plane; geometric satellite propagation is already modeled in the generator.
These declared values do not establish calibration. Unknown constant receiver/USB/TCP delay
remains unknown even if arrival variation, query width and receiver `tAcc` are
small. Manifests use `epoch_association: ublox_receiver_time_estimate`,
`gps_alignment_verified: false`, and `absolute_gps_uncertainty_s: null`.

`--model-time-offset-sec` (-1 to +1, default 0) is a separate, signed empirical
correction measured from generated-versus-sky observations. Positive values
advance the model GPS epoch at the same hardware deadline, reducing a positive
generated-minus-sky pseudorange delay. Negative values retard the model epoch.
The correction is applied once before ephemeris/channel/IQ initialization;
sample rate, subsequent `n/Fs`, PPS timing and the hardware start are unchanged.
It works with live UBX waveform generation, time-only offline rendering and
`--check-start`. It is rejected for replay, explicit scenarios and receiver-only
checks so a requested correction cannot be silently ignored. Receiver GPS-now
estimates and their health anchors remain independent of this model correction.

The [22 September mixed-capture investigation](x300-mixed-timing-2026-09-22.md)
measured an effective delay of approximately +36.9307 ms and a much smaller
relative rate error in `sp-partial-22-09/v1`. Its candidate correction is
`--model-time-offset-sec 0.0369307`. This is a capture-specific estimate;
changing receiver output, transport or RF paths requires remeasurement. Do not
also put that same residual into `--delivery-delay-sec` or RF-path delay flags:
the terms add. A correction does not prove which part was USB processing,
transport, RF delay or receiver ambiguity, and does not set calibration/alignment
flags to true. Manifests record its value and operator-supplied source; hardware
plans also retain the GPS epoch before this correction.

The [v2/v3 follow-up](x300-mixed-timing-v2-v3-2026-09-22.md) measured +37.932 ms
and +37.445 ms with the option omitted. Subtracting the fixed v1 candidate
offline leaves about +1.001 ms and +0.514 ms. Its repeatability must therefore
be checked across starts even when using the same hardware. The correction
remains opt-in and defaults to zero.

Preparation consumes the selected deadline's margin. If the margin falls below
20 ms before the first send, the run aborts and discards its dated IQ. There is
no automatic retiming/retry. A new invocation acquires fresh state and selects a
new deadline. During the run, time/reset faults, TCP disconnect, stale mandatory
messages, unusable selected ephemerides, PPS/reference loss and UHD continuity
faults fail the segment. Reconnect requires a new segment and warmup. Samples
already accepted by the radio can still drain after a failure.

Every live run writes raw bytes and `offset,length,host_monotonic_s` arrivals to
`MANIFEST.ubx` and `MANIFEST.ubx.rx.csv`, unless `--ublox-record` selects another
path. Those files must be new; they are never silently overwritten. Replay
requires both files and can only inspect or render offline with an explicit
scenario epoch:

```sh
./x300tx --check-ublox --ublox-replay reference.ubx --manifest replay-probe.json
./x300tx --ublox-replay reference.ubx --dry-run \
  --gps-week 2437 --gps-tow 121729.98 -P 3 -d 0.125 --manifest replay.json
make test-ubx
```

Replace the example replay epoch with one covered by the recording. Replay
never arms hardware or supplies fresh live time. It freezes the final decoded
navigation snapshot from the recording; it is not a simulation of every
navigation update over the recording's entire duration. The generated SC16
FNV-1a digest permits deterministic replay comparisons.

## Offline verification

```sh
./x300tx -e tests/fixtures/brdc0030.25n \
  -l 21.0047844,105.8460541,22 -P 1 -S 1:overhead \
  --gps-week 2347 --gps-tow 432029.98 \
  --dry-run -d 1.125 --manifest /tmp/x300-offline.json

make test-x300-timing
make test-ubx
make test-x300tx-matched
make test
```

Dry run never opens UHD. Normal mode renders the requested finite duration
(default 0.1 seconds for dry run). The matched-code validation window remains
0.1 seconds. The hardware timing tests inject PPS/lock loss, time jumps, partial
and zero sends, underflows, sequence/time errors, stale starts and missing
end-of-burst acknowledgements. They also check finite queue draining and IQ
sample ordering. LNAV tests independently decode HOW and week fields.

On macOS, the Makefile selects the SDK from the active Xcode/Command Line Tools
using `xcrun --sdk macosx --show-sdk-path`. This avoids mixing the selected
compiler/linker with an incompatible SDK inherited from the shell. Plain
`make` and `make x300tx` need no `SDKROOT` prefix. For an intentional custom SDK,
use `make SDKROOT=/path/to/sdk x300tx`.

## Running a scenario

### Revive and live navigation are separate modes

`-S PRN:revive` requires frozen `-e` ephemeris and is rejected with
`--ublox-tcp` / `--ublox-replay` navigation input. `-e` and any u-blox
navigation source are mutually exclusive. The separate time-only source below
does not supply navigation.
Use `-P` for the revived output PRNs and `-S` for their construction. For an
explicit all-revive selection, the file must contain usable past templates
for every selected target; a current ephemeris set is not required.
Selections that include non-revived output retain the current-set check.
See [revive semantics](synth-revive-mode.md) and the command templates in
[command.md](../command.md).

RF scheduling with F9P time plus historical RINEX revive uses `--ublox-time-tcp`.
`-t` and `--gps-week/--gps-tow` label a scenario; they do not request a
wall-clock/GPS start. For live clone, donor ephemerides come from UBX instead;
see [clone mode](synth-clone-mode.md).

### Revive with current F9P time

`--ublox-time-tcp` reads fresh `NAV-TIMEGPS` over TCP while `-e` supplies all
navigation and historical revive templates. It requires explicit location and
PRNs. It supports finite TX (`-d`), continuous TX (`--stream`), preparation
without a TX streamer (`--check-start`), and finite offline rendering
(`--dry-run`). It rejects `-t`, explicit week/TOW, replay and live-clone navigation.

For example, with a RINEX containing usable historical templates for the
selected targets (the filename and PRNs below are examples):

```sh
./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  -e brdc2650.26n -l 21.0047844,105.8460541,22 \
  -P 14,22,30 -S 14:revive,22:revive,30:revive \
  --start-lead-sec 2 --dry-run -d 1 --manifest revive-time-dry-run.json

# Same source separation, with a fixed X300 deadline and no TX streamer:
./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  -e brdc2650.26n -l 21.0047844,105.8460541,22 \
  -P 14,22,30 -S 14:revive,22:revive,30:revive \
  --addr 192.168.10.2 --channel 1 --check-start --manifest revive-start.json

# Continuous estimated-GPS revive, using a prepared laboratory RF path:
./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  -e brdc2650.26n -l 21.0047844,105.8460541,22 \
  -P 14,22,30 -S 14:revive,22:revive,30:revive \
  --addr 192.168.10.2 --channel 1 --antenna TX/RX --gain SETUP_GAIN_DB \
  --stream --start-lead-sec 2 --prebuffer 50 --manifest revive-stream.json
```

In TX and `--check-start`, the model epoch is mapped to a fixed future hardware
deadline before channel initialization and prebuffering. It uses the same
minimum-delay estimate and path corrections as live-navigation startup.
In `--dry-run`, it uses a future host-monotonic instant and the same estimator;
no UHD device is opened and no RF deadline is constructed.
No host wall clock or fixed UTC leap offset selects it. Three consecutive
valid receiver epochs are required; selected PRNs need no live SFRBX, PVT or
RAWX. RINEX coverage, revive geometry, and target allocation are checked.
All renderer, preflight and sender health checks use time-only readiness in
this mode. Missing authentic navigation for a revived PRN is allowed.
Receiver disconnects, stale time and reset/discontinuity fail the segment.
Original RINEX bytes/epochs remain unchanged and its SHA-256 is recorded.

Manifests use `navigation_source=frozen_rinex` and `time_reference_only=true`.
TX and `--check-start` record `epoch_association=ublox_receiver_time_estimate`,
`hardware_start_s`, and a valid `live_start_plan`. Offline rendering instead
records `epoch_association=ublox_host_epoch_estimate` and
`host_epoch_monotonic_s`, with null hardware plan/start and `uhd_opened=false`.
Both record `time_estimate`, `gps_alignment_verified=false`, and unknown
absolute uncertainty. For an explicit `-P` selection containing only revived
PRNs, startup validates historical templates without requiring a current RINEX
set. The current-set requirement still applies when non-revived PRNs are generated.
Finite/continuous sending is tested with a fake radio and real revive IQ.
No RF test or independent GPS-to-RF alignment measurement was performed.

### Controlled scenario TX

For a prepared conducted laboratory path, use a navigation file covering the
scenario epoch, the intended reference position, and the setup's channel/gain:

```sh
./x300tx -e navigation.rnx -l 21.0047844,105.8460541,22 \
  --gps-week SCENARIO_WEEK --gps-tow SCENARIO_TOW \
  --addr 192.168.10.2 --channel 1 --antenna TX/RX --gain SETUP_GAIN_DB \
  --clock-source external --time-source external \
  --start-lead-sec 0.25 --prebuffer 5 --stream --manifest run.json
```

Replace the uppercase placeholders with the scenario and setup values. No live
transmit test was run as part of this refactor. `--stream` runs until Ctrl-C;
use `-d <seconds>` for a finite run. Ordinary live mode defaults to 300 seconds
with the default build. If no epoch is supplied, the first RINEX epoch is used.
The normal/revive mode rejects matched-code-only `--confirm-controlled-rf` and
`--calibration-id`; it has no physical containment interlock. Establish the RF
path and save its calibration separately before using a transmit command.

The bounded producer queue renders independently of the sender. The first TX
is scheduled after prebuffering, with at least `--start-lead-sec` margin and
up to one additional second for rounding to a hardware second. A stale start
fails; there is no immediate-start fallback. Partial sends retain the unsent
suffix. Finite runs drain all frames, including a partial final frame.

The device/transport FIFO can fill before the scheduled start even when the
host has many prebuffered frames. Once the first samples are accepted, a send
that returns zero before that fixed start is retried with the same unsent IQ.
Each retry checks reference/PPS and receiver health and drains UHD error events;
it neither changes the start timestamp nor skips samples. The manifest counts
these waits as `transport.prestart_backpressure_waits` and all zero-sample data
sends as `transport.zero_sample_sends`. EOB also allows time for a full FIFO
to begin draining at the scheduled start.

Reference/LO lock loss, missing PPS, hardware-time discontinuity, zero send
progress on the initial send or after the scheduled start without an operator
stop, UHD continuity errors and clipping fail the run. Async events are drained
after every data send, including successful and partial sends. An operator stop
that interrupts a blocked send is checked before declaring no-progress failure;
an actual UHD continuity error still fails the run. Errors stop new
submissions; already queued hardware samples may still drain. Shutdown sends
EOB and waits for a burst acknowledgement. There is no automatic restart or
retiming after a continuity failure.

## Timing options and migration

| Option | Meaning |
|---|---|
| `--check-sync` | PPS/reference verification without a TX stream |
| `--clock-source`, `--time-source` | `external` (default) or `gpsdo` |
| `--start-lead-sec` | Future lead, 0.02–60 s; default 0.25 s for RINEX, 2 s for UBX |
| `-r`, `--tx-advance-ns` | Aliases for that margin, in seconds/nanoseconds |
| `--prebuffer` | Queue capacity, 1–50 blocks; default 5, each at most 100 ms |
| `--rate` | Requested rate, 1–200 Msps; default 2.5 Msps; hardware limits apply |
| `--gps-week`, `--gps-tow` | Scenario GPS epoch at sample zero; supply together |
| `-t` | Scenario **GPS calendar** epoch, including fractional seconds |
| `--stream` | Continuous normal waveform mode; an explicit `-d` makes it finite |
| `--dry-run` | Offline rendering without UHD |
| `--manifest` | Run JSON path; normal mode defaults to `x300tx-run-<time>-<pid>.json` |

Removed options fail explicitly: host-clock `-n`, `--gps-time-ppm`,
`--tx-time-scale-ppm`, Trimble time-tag scheduling and `--trimble-tx-cal-ns`.
All `--trimble-rtcm-*` options also fail with a migration message to
`--ublox-tcp host:port`. Use `--stream` for continuous operation; RINEX scenarios
need a scenario epoch, while the live UBX path chooses its own future estimate.
The old automatic TOE/TOC shift has been removed. See `./x300tx --help` for
waveform, position and UBX options.

Normal manifests use `gps-sdr-sim.x300tx-hardware-time.v1`; matched-code
manifests use `gps-sdr-sim.x300tx-matched-code.v3`. Both record scenario/hardware
time mapping, PPS verification, generated versus UHD-accepted sample counts,
continuity counters, first reported error time, rendering time and queued lead.
An accepted sample count means UHD accepted the samples; it is not an RF
measurement. The count at first error is a submission frontier, not a proven
RF corruption index. A final success status requires a clean EOB acknowledgement.

Transport diagnostics also include `data_send_calls`, `partial_sample_sends`
(positive short returns), `max_samples_per_send`, and `maximum_send_s` (including
pre-start blocking). EOB is excluded from these data-send counters.
`first_event_observed_host_monotonic_s` records when the host first consumes an
error. It does not replace `first_event_hardware_s`, which stays null when UHD
provides no event timestamp. See the
[sequence-error investigation](x300-sequence-error-debug-2026-09-22.md) for the
reported long-run failures and a transport-only comparison.

Normal manifest statuses include `dry_run`, `start_checked`, `completed`,
`stopped` and `failed`. The console's `[TX] status=success` is not the JSON
status for finite transmission: that JSON status is `completed`. Normal
manifests do not yet record every target/synthetic/gain argument. Save the exact
invocation and console output with the manifest. `--check-start` success must
still have zero accepted samples and `transport.send_attempted=false`.

An existing manifest path is replaced atomically. Navigation input, manifest,
trajectory and their temporary paths must be distinct, including symlink aliases.
