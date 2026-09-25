# F9P / X300 / bladeRF RF and timing qualification, 2026-09-22

The two F9P receivers and X300 were checked without transmitting RF. Later,
the user replaced the direct X300-to-F9P RF cable with two exposed antennas:
X300 RF B TX to a transmit antenna, and a receive antenna to bladeRF.
The bladeRF receive-only baseline passed. **No X300 transmission, RF-link
verification or absolute GPS-to-RF alignment measurement was performed.**

## Later topology: exposed antennas and bladeRF RX

The user confirmed both antennas are exposed on the bench, outside a shielded
enclosure. X300 GPS transmission was not started. Close antenna spacing does
not provide RF containment. A subsequent generated-GPS link test requires an
RF-shielded enclosure or a conducted path with known attenuation and appropriate
DC isolation. The direct-F9P results below describe the earlier topology.

`bladeRF-cli --probe`, `info`, `version` and an RX-only CLI script established:

| Check | Measured result |
|---|---|
| Board / serial | Nuand bladeRF x40 (`bladerf1`), `270c4bef62ded4be1bd9c822a60306a5` |
| USB / FPGA | SuperSpeed; 40 KLE, loaded, version 0.16.0 |
| Firmware / library | 2.6.0-git-09c82087 / 2.6.0-git-fcf94233-dirty |
| RX1 center / actual sample rate | 1575.420 MHz / 2,500,000 samples/s |
| RX bandwidth / gain | 2.5 MHz / fixed 5 dB; AGC off |
| Saved IQ | 12,500,000 complex SC16 Q11 samples, 50,000,000 bytes; nominally 5 s |
| CLI completion | Exit 0, RX idle, last RX error `None`; TX idle and no file configured |
| Sample quality | Zero rail samples and zero out-of-Q11-range samples |
| Clock | Internal sampling; SMB clock and VCTCXO tamer disabled |

The sample component ranges were I = 5..15 and Q = 20..30 counts, with standard
deviations 1.087 and 0.935 counts. Mean-removed power was -63.094 dBFS under the
definition in `quality.json`; this is not calibrated input dBm. The low-gain
baseline contains small fluctuations and DC offset. It establishes sample
acquisition, not GPS acquisition, antenna continuity or reception of X300 RF.

The CLI binary format stores interleaved IQ without timestamp records. Exact
file length and no CLI error do not prove uninterrupted hardware sampling;
no overrun count or timestamp continuity result is claimed. The commands and
sample interpretation follow [Nuand's CLI documentation](https://github.com/Nuand/bladeRF/wiki/bladeRF-CLI-Tips-and-Tricks).
For RF timing, use [RX metadata](https://www.nuand.com/libbladeRF-doc/v2.5.0/sync_rx_meta.html)
to retain hardware timestamps, actual sample counts and overrun flags, then
establish their relationship to the X300/GPS time base. Detecting a burst's
index in a free-running bladeRF recording alone cannot establish absolute GPS
transmit time. No common bladeRF/X300 timing connection has been verified.

Local, Git-ignored evidence is under
`analysis-output/runs/bladerf-l1-rx-20260922-0947/`: `capture.cli`, `capture.log`,
`baseline.sc16q11` and `quality.json`. The JSON includes sample statistics and
SHA-256 hashes of the script, log and IQ file. The CLI exited and released USB
after capture. The remaining F9P/X300 qualification below was not repeated.

## Receiver identification and baseline

USB names changed after reconnection. The current role assignment follows the
observed receiver data and the user's stated sky/DUT arrangement:

| Role | USB device | Persistent chip ID | TCP endpoint during capture |
|---|---|---|---|
| Reference with a live GPS fix | `/dev/cu.usbmodem11301` | `314852bd18` | `127.0.0.1:5019` |
| DUT on the reported RF B cable | `/dev/cu.usbmodem1301` | `d44162bdda` | `127.0.0.1:5020` |

Both identify as ZED-F9P, HPG 1.32 / protocol 27.31. The module identification
does not identify the carrier board, antenna bias circuit or available headers.
An initial capture used endpoints in the reverse order; those recordings are
named by USB port (`port1301` and `port11301`) to preserve their identity.

The later 60-second reference capture contained 59 valid 3D epochs and 60
nonempty RAWX epochs. Its last solution used six satellites and reported speed
0.279 m/s and horizontal accuracy 13.848 m. GPS week/TOW was
2437:180648.999582007, with receiver `tAcc` 20 ns and **valid** leap offset 18 s.
This is improved availability, not a long-duration stability result. That
accuracy field does not measure USB/TCP delay or RF arrival error.

The DUT diagnostic capture contained 20 epochs, no valid position, no valid GPS
time, and no RAWX observations. This is consistent with the transmitter being
off; it establishes neither cable continuity nor successful RF reception.
Both streams had zero checksum errors and zero malformed messages.

The reference returned `TIM-TP` with flags 27 and reference information 63:
UTC time base, UTC available, and quantization-error-invalid set. Its next-pulse
label was week 2437 / 180632000 ms. Relative to the last GPS solution, the
18-second GPS–UTC offset explains why the next UTC pulse label is numerically
17 seconds behind the current integer GPS second. No time-mark (`TIM-TM2`)
report was received. A USB pulse report does not demonstrate a physical
connection to X300 or qualify pulse accuracy.

Both RF monitors reported antenna status 1 and power status 2, meaning unknown.
Jamming state 0 means unknown/disabled, not a clean-RF verdict. AGC/noise/CW
suppression values are retained as receiver diagnostics, not input dBm.
Definitions follow the installed firmware's
[HPG 1.32 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf).

## X300 timing checks

The first `--check-sync` attempt failed with
`Reference Clock PLL failed to lock to external source`. The user confirmed
that PRS10 power and both clock/PPS cables were connected. A subsequent retry
**passed** external reference lock, PPS time-zero latch and next-PPS readback
of 1.000000000 s. No cause is assigned to the first failure from this evidence.

The automatic live startup check then failed during its 90-second receiver
warmup on `PRN 15 missing SF1,SF2,SF3`. That attempt did not open UHD or generate
samples. Some weak observed satellites still do not provide complete navigation.

A second check explicitly selected PRNs 18, 20, 24 and 32, each complete in the
preceding diagnostic capture. The new 90-second acquisition had a valid 3D fix
and nonempty RAWX in **all 90 epochs**, but received no SF3 for PRN 24. It stopped
with `UBX warmup: PRN 24 missing SF3`, before opening UHD or generating IQ.
The final fix used six satellites, speed 0.010 m/s and reported horizontal
accuracy 10.151 m. Thus local clock/PPS passed today, but neither live startup
attempt reached waveform preparation. The earlier 2026-09-21 passes remain
separate evidence and must not be reported as today's live-source result.

## What the RF cable can establish

A conducted GPS L1 C/A waveform can be supplied to the F9P antenna input, and
the DUT can report acquisition, tracking, raw measurements, navigation and
position. Its UBX output is not an IQ recording of the input waveform. First
track, first RAWX and first valid PVT are receiver acquisition events, not the
instant at which the first transmitted sample reached the RF connector.

The reported bare coax connection has no established RF attenuation or DC
isolation. The [Ettus UBX specification](https://kb.ettus.com/UBX) lists maximum
TX power of +20 dBm below 3 GHz. The
[ZED-F9P-01B datasheet](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-01B_DataSheet_UBX-17051259.pdf)
lists +10 dBm CW as the module's absolute RF input limit, not an operating test
level. That module limit does not qualify an unidentified carrier board's
front end. Therefore a bare cable is not an established test configuration.

Use a 50-ohm conducted path with known attenuation at L1, measured transmitter
output and receiver input levels, and appropriate DC isolation from any antenna
bias. An attenuator lowers the RF signal; a DC block stops antenna supply
voltage travelling along the coax. The installed loss must follow the measured
power budget, not a guessed fixed number of dB or an uncalibrated software gain.
No TX streamer was created and no samples were sent in this session.

## Missing measurement for absolute GPS alignment

The existing software maps receiver epochs through host receipt time to a
future hardware deadline. Two USB receivers still leave their transport delays
and independent clock relationships unknown. A DUT tracking only the generated
signal derives its time from that signal; agreement with the requested scenario
time does not independently validate the absolute transmission epoch.

The available hardware provides possible timing paths, but the current wiring
does not supply the required common physical event. One concrete next wiring
option is:

```text
PRS10 10 MHz ------------------------------> X300 REF IN
Sky F9P TIMEPULSE -- verified level buffer -> X300 PPS IN
Sky F9P USB/TCP -- pulse labels -----------> host epoch association
X300 RF B -- attenuation / DC isolation ---> DUT antenna input
DUT USB/TCP ------------------------------> receiver observations
```

This option replaces the PRS10 PPS input while retaining its frequency
reference. Configure/read back the F9P pulse rate, time grid, polarity and
validity policy, associate a specific labeled edge with the X300 latch, and
verify subsequent edges before arming. The current UTC pulse report must not
be interpreted directly as GPS TOW. X300 documentation specifies a
[5 Vpp PPS square wave](https://files.ettus.com/manual/page_usrp_x3x0.html);
the unidentified F9P carrier output must be checked and appropriately buffered.
These are proposed wiring changes, not connections made during this session.

Alternatively, retain the PRS10 PPS wiring and instrument a common hardware
event through the reference F9P's EXTINT input, using suitable input levels.
`TIM-TM2` can label detected edges; associating the particular event with an
X300 hardware timestamp must be explicit and unambiguous. This alternative
requires a physical event connection too. The
[u-blox timing description](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf#page=67)
describes the event-reporting mechanism.

GPS epoch association then needs a separate RF delay/arrival measurement at
the DUT plane. Pulse wiring alone does not calibrate DAC/filter/cable delays,
prove carrier phase, or turn receiver acquisition time into RF start time.

## Software and evidence

The UBX adapter now records chip ID, PVT/RAWX availability counts, per-signal
observations, `MON-RF`, `TIM-TP` and `TIM-TM2`. It validates message lengths,
versions and timing ranges, preserves time-base flags and the different pulse
fractional units, and leaves `hardware_edge_association_verified: false`.
These optional diagnostics never substitute for live time or arm transmission.
The bridge adds only read-only identity/RF/pulse queries; receiver configuration
and reset state were not changed.

Passed `make test-ubx test-x300-timing test-x300tx-matched`, including malformed
diagnostics, distinct TP/TM2 fractional units, week-boundary event reports and
the requirement that timing labels alone cannot supply valid live GPS time.

Artifacts are local and Git-ignored under
`analysis-output/runs/f9p-x300-20260922/`: USB inventory, port-named baseline
captures, `reference-diagnostics.*`, `dut-diagnostics.*`, raw UBX/arrival records,
`start-check.*`, the selected-PRN check, and both clock-check logs.
`validation-summary.json` records both refused startups and SHA-256 hashes of
the executable, changed receiver/bridge code and 21 capture/log artifacts.
Both temporary bridges were stopped after validation and the USB ports were
released; neither bridge reported a slow-client disconnection.
