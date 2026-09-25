# X300 sequence-error investigation, 2026-09-22

The reported `S` / `EVENT_CODE_SEQ_ERROR` is a device-reported TX packet
sequence discontinuity. UHD defines this as packet loss between the host and
device. A successful `send()` return records submission, not verified RF
delivery. The application correctly invalidates the run on this event.
[UHD event definitions](https://files.ettus.com/manual/structuhd_1_1async__metadata__t.html)

The user confirmed a direct Ethernet connection. The current evidence locates
the failure in the host-to-X300 transport path; it does not distinguish the
host network stack, Ethernet adapter/driver, cable, UHD transport, or device.
No hardware transport fix has been established by this investigation.

## Evidence from the failed runs

Both runs used 2,500,000 complex SC16 samples/s. That is 10 MB/s (80 Mb/s)
of IQ payload, approximately 6,868 full 364-sample packets/s before shorter
frame-ending packets and protocol overhead.

| Manifest | Accepted sample duration at error observation | Underflows | Sequence errors | Maximum render call |
|---|---:|---:|---:|---:|
| `x300tx-ublox-1790068922-82153.json` | 70.8129888 s | 0 | 1 | 3.715 ms |
| `x300tx-ublox-1790069063-82563.json` | 105.7125016 s | 0 | 1 | 2.682 ms |

In the reported run, generated minus accepted samples was exactly 12,500,000
(five seconds of rendered IQ). The minimum recorded accepted-sample lead was
22.725 ms. These measurements show ample rendering headroom at the observation
point. `--prebuffer 50` buffers samples in host RAM; it cannot repair a missing
Ethernet packet. The eleven zero-sample sends all occurred while waiting for
the scheduled start and were recorded as pre-start backpressure retries.

The error has no hardware timestamp. UHD 4.10's CHDR transport converts
`STRS_SEQERR` into `EVENT_CODE_SEQ_ERROR` with `has_time_spec=false`, consistent
with the manifest. Accepted sample duration is therefore an observation
frontier, not a measurement of the exact corrupted RF sample or failure time.
[UHD 4.10 transport source](https://github.com/EttusResearch/uhd/blob/v4.10.0.0/host/lib/include/uhdlib/rfnoc/chdr_tx_data_xport.hpp)

The exact reported epoch, RINEX, location and three revived PRNs also rendered
180 seconds offline: 450,000,000 samples, zero clipped components, status
`dry_run`, IQ FNV-1a `6cd82cde23e4dcb9`. The temporary manifest is
`/tmp/x300-seq-debug-render-180s.json`. This exercises the renderer across the
reported failure window; it does not test Ethernet or RF continuity.

## Software defects corrected

The shared sender in `player/x300_timing.hpp`, used by both x300tx waveform
paths, previously drained UHD events only between render frames or on zero
sends. During a normal 100 ms frame, it could submit all remaining packets
after a sequence error became available. It now drains after every data send
and stops submitting the frame suffix on failure. A fault-injection regression
failed before this change and passes afterward, including an untimestamped
error on a 364-sample packet inside a 250,000-sample frame.

A blocked send returning zero during an operator stop previously became a
false no-progress failure. Two nearby manifests show that combination
(`1790068823-81820` and `1790068867-81965`). The sender now checks stop state
after draining events and before declaring a no-progress failure. The tests
cover both a clean stop and a simultaneous sequence error, which remains fatal.

Manifests now record data-send call count, positive short-send count, maximum
requested send size, maximum completed send-call duration, and the first
error's host-monotonic observation time. A missing hardware event timestamp
remains null. Detection improvements preserve the fail-on-discontinuity
contract; they do not recover dropped packets or establish loss-free operation.

## Verification and remaining hardware comparison

`make test-x300-timing test-ubx test-x300tx-matched` passed after rebuilding
`x300tx`: three C++ test executables and 23 Python integration tests. No RF was
transmitted during this investigation. The dedicated interface previously
called `en7` was absent; the route to `192.168.10.2` currently used the default
`en0` gateway, so hardware testing could not proceed.

After reconnecting, check the actual route and inspect the interface it names:

```sh
rtk proxy route -n get 192.168.10.2
rtk proxy ifconfig INTERFACE
rtk proxy netstat -I INTERFACE -b -d
```

The route should use the dedicated Ethernet interface. Check negotiated link
speed, MTU, and before/after error/drop counters. Substitute its real name for
`INTERFACE`. The 1472-byte UHD frame size already fits a normal 1500-byte MTU;
there is no evidence here that forcing jumbo frames or larger socket buffers
will fix the failure. UHD documents that MTU must be supported across the
path and that large send buffers can reduce transmit performance.
[UHD transport guidance](https://files.ettus.com/manual/page_transport.html)

With the existing laboratory RF path prepared and x300tx stopped, compare a
ten-minute UHD-only TX run at the same rate, channel and data formats. This
command opens/configures the radio and performs TX; it is not a read-only
probe. Its options were checked against the installed UHD 4.10 example's help.

```sh
rtk proxy /opt/homebrew/lib/uhd/examples/benchmark_rate \
  --args "type=x300,addr=192.168.10.2" \
  --tx_rate 2500000 --tx_channels 1 --tx_cpu sc16 --tx_otw sc16 \
  --ref external --pps external --duration 600 \
  --seq-threshold 0 --underrun-threshold 0
```

Sequence errors in this benchmark reproduce transport loss independently of
GPS generation, revive state and the F9P reader. A clean benchmark followed
by repeatable x300tx failures instead calls for comparing application scheduling
and control-query load. One clean run cannot exclude intermittent loss.
Compare cable/adapter/port substitutions one at a time if UHD-only loss persists.
Keep each application's exact command, console output, manifest where available,
and interface counters. Do not suppress the continuity error to keep a run alive.
