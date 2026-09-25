# X300 revive — live F9P PPS + PRS10C 10 MHz / Hanoi

Wiring: live-sky F9P TIMEPULSE0 -> X300 PPS/TRIG IN; PRS10C 10 MHz -> X300
REF IN. USB time/pulse metadata must come from that same F9P. The successful
24 September 2026 PPS check used receiver ID `d44162bdda` at
`/dev/cu.usbmodem111301`; enumerate the port again before use.

The hardware commands below use `--gps-pps`: sample-zero GPS time comes from
the verified hardware PPS/GPS label pair. Absolute RF arrival alignment remains
unmeasured. The host's independently synchronized, unsmeared UTC clock must stay
within the declared `--pps-host-utc-bound-sec 0.2` bound. Host UTC only identifies
the correct second; the physical PPS sets the fractional timing reference.

Keep the reference F9P tracking authentic sky. PRNs, location and gain below
retain the existing Hanoi example. Set `X300_NAV_FILE` to an existing RINEX
archive with usable templates covering the current revive lookback (up to eight
hours). The previous `brdc2650.26n` example is dated 22 September and is not a
current navigation input for a 24 September run.

~~~bash
make x300tx
/usr/bin/python3 -m serial.tools.list_ports -v

# Read-only UTC clock check. Require a successful reply whose absolute offset
# plus reported uncertainty is below 0.2 s; this command does not set the clock.
sntp -t 3 time.apple.com

# If TIMEGPS output is missing, stop the bridge/other USB readers before this
# configuration command. Enable TIMEGPS once per navigation solution.
uvx --from pyubxutils==1.0.6 ubxsetrate --port /dev/cu.usbmodem111301 \
  --baudrate 115200 --msgClass 0x01 --msgID 0x20 --rate 1

# Terminal 1: keep the USB -> raw UBX TCP bridge running, including PPS queries
/usr/bin/python3 tools/ubx_tcp_bridge.py --serial /dev/cu.usbmodem111301 \
  --baud 115200 --host 127.0.0.1 --port 5019 --pps

# Terminal 2: verify estimated GPS now and the receiver ID; no UHD/RF
./x300tx --check-time --ublox-tcp 127.0.0.1:5019 --ublox-warmup-sec 15

# Verify external frequency lock, PPS latch and three labeled GPS edges.
# No navigation file, TX streamer or RF. This resets the local hardware counter:
# run while no other experiment owns the X300. --check-pps implies --gps-pps.
./x300tx --check-pps --ublox-tcp 127.0.0.1:5019 \
  --addr 192.168.10.2 --clock-source external --time-source external \
  --pps-host-utc-bound-sec 0.2 --ublox-warmup-sec 15

# Terminal 2: replace this path before running the waveform commands below.
X300_NAV_FILE=/absolute/path/to/revive-archive.rnx

# Optional offline waveform check: arrival-estimated GPS time, no UHD/PPS/RF.
# --dry-run does not accept --gps-pps and cannot verify hardware alignment.
./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  -e "$X300_NAV_FILE" -l 21.0047844,105.8460541,22 \
  -P 14,22,30 -S 14:revive,22:revive,30:revive \
  --rate 2500000 --start-lead-sec 2 --dry-run -d 1

# Check PPS-associated revive rendering and the fixed deadline; no RF
./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  --gps-pps --pps-host-utc-bound-sec 0.2 \
  -e "$X300_NAV_FILE" -l 21.0047844,105.8460541,22 \
  -P 14,22,30 -S 14:revive,22:revive,30:revive \
  --addr 192.168.10.2 --channel 1 --check-start \
  --clock-source external --time-source external \
  --rate 2500000 --start-lead-sec 2 --prebuffer 50

# PPS-associated current-GPS revive TX: qualified conducted/shielded setup, RF B
# Ctrl-C stops streaming; replace --stream with -d 60 for a finite burst.
./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  --gps-pps --pps-host-utc-bound-sec 0.2 \
  -e "$X300_NAV_FILE" -l 21.0047844,105.8460541,22 \
  -P 14,22,30 -S 14:revive,22:revive,30:revive \
  --addr 192.168.10.2 --channel 1 --antenna TX/RX \
  --clock-source external --time-source external \
  --rate 2500000 --gain 0 --stream --start-lead-sec 2 --prebuffer 50
~~~

Requires `uv` and `pyserial` in the Python interpreter used for the bridge,
with USB UBX output enabled. The
[rate command](https://github.com/semuconsulting/pyubxutils#ubxsetrate-cli)
enables TIMEGPS on all receiver interfaces; it does not change the navigation
solution rate or save configuration to flash. The bridge only forwards/polls;
`--pps` adds TIM-TP, CFG-TP5 and NAV-TIMELS queries, without changing pulse settings.
PPS mode requires a 1 Hz rising, GNSS/TOW-aligned UTC or GPS pulse, zero user pulse
delay and fresh valid time/leap metadata. The current UTC grid is supported;
GPS-minus-UTC is read from the receiver rather than hardcoded.
`--check-time` verifies receiver time readiness; `--check-pps` verifies the
hardware epoch association. A sent configuration command proves neither.

Use RINEX containing usable historical templates for all selected revive PRNs.
An all-revive `-P` selection does not require a current ephemeris set; selections
that also generate non-revived PRNs retain that check. Manifests and UBX logs
get unique default names. `--ublox-tcp` still supplies live navigation
and rejects revive; use `--ublox-time-tcp` for the combination above. Do not set
`-t` or `--gps-week/--gps-tow`: sample zero is planned automatically before rendering.

The old approximately 37 ms empirical correction must be removed. PPS mode
rejects nonzero `--delivery-delay-sec` and `--model-time-offset-sec`.
`--tx-path-delay-sec` adds a measured TX-path correction and
`--sky-path-delay-sec` subtracts an extra authentic RF-branch delay; both default
to zero and remain marked uncalibrated in the manifest. A missed startup
deadline requires more `--start-lead-sec` and a new run; it does not justify
changing the model's GPS time. PPS/label/reference loss, drift and UHD continuity
faults stop the run.

Inspect these fields after the checks and TX:

| Field | Expected meaning |
|---|---|
| `epoch_association` | `ublox_tim_tp_hardware_pps` |
| `pps_latch_verified` | `true`: local PPS latch checked |
| `pps_epoch_association_verified` | `true`: hardware edges associated with GPS labels |
| `live_start_plan.pps_epoch_association` | Frozen `(H_ref, GPS week/TOW)` and declared bounds |
| `pps_monitor` | Subsequent label/edge confirmations and drift checks |
| `transport.failure_reason` | Empty on success; inspect underflow/sequence/time errors |
| `gps_alignment_verified`, `rf_alignment_verified` | Still `false`: calibrated RF timing unmeasured |

Our achieved milestone is **GPS epoch association and sample-time scheduling**.
The successful no-TX check paired three edges, including `H_ref=2 s` with GPS
`2437:385359`, and sent zero samples. This does not yet establish the timing
error of transmitted signals relative to real sky at the receiver input.

The generator models a receiver epoch at `-l`/`-c`; each PRN's code and data time
also includes its modeled satellite propagation delay. Revive uses transformed
historical ephemerides, so its orbit, Doppler, code phase and navigation content
need not match that PRN's authentic current broadcast. PPS supplies the common
GPS time reference; it does not make all satellite code phases identical.

The next measurement is a common-reference comparison of generated and sky
signals at the intended receiver input: measure the remaining time bias after
accounting for modeled per-PRN propagation, qualify TX/sky-path delays, and check
bias drift and sample continuity over time. Carrier-phase coherence requires
separate evidence. The PRS10C frequency reference is independent of the F9P PPS,
so a short startup pass does not establish long-run rate agreement. This matches
the distinction between sample-clock, PPS and phase synchronization in the
[Ettus synchronization guidance](https://files.ettus.com/manual/page_sync.html).

See [PPS mode, implementation and measured validation](docs/x300-gps-pps-alignment.md)
for the timing contract and the existing
[RF qualification plan](docs/x300-f9p-rf-timing-qualification-2026-09-22.md)
for the remaining measurement scope. Exposed bench antennas remain unqualified
for TX.
