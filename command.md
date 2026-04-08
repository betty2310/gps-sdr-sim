Distribute 5 sats around the sky

```bash
$ gps-sdr-sim -e hour0910.26n -l 21.0047844,105.8460541,5 \
             -P 1,2,3,5,7 \
             -S 1:0.0/60.0,2:90.0/45.0,3:180.0/50.0,5:270.0/40.0,7:45.0/55.0 \
             -d 300 -v
```


BladeRF hardware test (old pipe path):

```bash
gps-sdr-sim -e hour0930.26n -l 21.0047844,105.8460541,5 \
               -P 3,5,4,8 -S 3:30/60,5:45.0/55.0,4:90/45,8:120/30 \
               -n -o - | ./player/bladeplayer -f - -b 16 -g -35

```

BladeRF integrated path (bladetx, Trimble time-tag):

```bash
  ./bladetx -e hour0910.26n -l 21.0047844,105.8460541,5 \
    -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \
    --trimble-time-tag-host 192.168.5.245 \
    --trimble-time-tag-port 5017 \
    --trimble-start-offset-sec 2 --txvga1 -35
```

X300 hardware test later, using a FIFO:

```bash
  mkfifo /tmp/gpssim.iq
  python3 gps-sdr-sim-uhd.py \
    -t /tmp/gpssim.iq \
    -s 2600000 \
    -x 0 \
    -a "type=x300,clock_source=external,time_source=external" &
  ./gps-sdr-sim -e hour0910.26n -l 21.0047844,105.8460541,5 \
    -P 1,2 -S 1:overhead,2:90.0/45.0 \
    -n -r 0.0 -o /tmp/gpssim.iq
```

# Calibrate time offset

```bash
uv run tools/ubx_bladetx_cal.py processing/dataset/COM4___9600_260408_034137.ubx --inject 3,4,7,8 --trimble-tag-lead-ms 790 --trimble-start-offset-sec 2
```

# Tue 7 2026 ngon

```bash
bladetx -e hour0970.26n -l 21.0047844,105.8460541,5 -P 3,4,7,8 -S 3:20/60,4:90/45,7:180/30,8:45/55 --trimble-time-tag-host 192.168.5.245   --trimble-time-tag-port 5017 --trimble-tag-lead-ms 790 --trimble-start-offset-sec 2 --txvga1 -35
```


# Web 8 Apr 2026 

```bash
bladetx -e hour0980.26n -l 21.0047844,105.8460541,5 -P 4,5,8,9 -S 4:20/60,5:90/45,8:60/30,9:45/55 --trimble-time-tag-host 192.168.5.245   --trimble-time-tag-port 5017 --trimble-tag-lead-ms 790   --trimble-start-offset-sec 2 --txvga1 -35
xyz =  -1626547.4,   5730511.6,   2271891.3
llh =   21.004784,  105.846054,         5.0

[BLADE] Opening bladeRF device ...
[INFO @ host/libraries/libbladeRF/src/board/bladerf1/bladerf1.c:883] Waiting for device to become ready...
[WARNING @ host/libraries/libbladeRF/src/board/bladerf1/bladerf1.c:1742] RX DC calibration table not found. Manual gain control will be used instead.
[INFO @ host/libraries/libbladeRF/src/board/bladerf1/bladerf1.c:1743] To enable AGC, see "Generating a DC offset table" at https://github.com/Nuand/bladeRF/wiki/DC-offset-and-IQ-Imbalance-Correction
[BLADE] Device: libusb  serial: 270c4bef62ded4be1bd9c822a60306a5
[BLADE] TX frequency: 1575420000 Hz
[BLADE] TX sample rate: 2600000 sps
[BLADE] TX bandwidth: 2500000 Hz
[BLADE] TX VGA1: -35 dB
[BLADE] TX VGA2: 0 dB
[TIMING] Generator sample rate: 2600000.000000 Hz
[TIMING] Epoch sample count: 260000 samples every 100.0 ms

[TRIMBLE] Connecting to 192.168.5.245:5017 ...
[TRIMBLE] Connected.
[TRIMBLE] Tag received: UTC 2026-04-08 03:37:12
[TRIMBLE] Config: start-offset=2 s  leap=18 s  tag-lead=790 ms  cal=0 ns
[TRIMBLE] Target GPS epoch: week 2413  tow 272252.000000000
[TRIMBLE] Target datetime:  2026/04/08,03:37:32.000
Start time = 2026/04/08,03:37:32.000 (2413:272252.000)
Duration = streaming until interrupted
Synthetic PRN 04: az=20.0 el=60.0 deg
Synthetic PRN 05: az=90.0 el=45.0 deg
Synthetic PRN 08: az=60.0 el=30.0 deg
Synthetic PRN 09: az=45.0 el=55.0 deg
04   20.0  59.9  20855303.5   6.2
05   90.1  45.0  21668476.5   7.6
08   60.1  30.0  22799406.9  10.1
09   45.2  55.0  21088056.3   6.6
Partial constellation mode: rendering only PRNs 4 5 8 9

[TX] Pre-buffering 5 epochs (0.5 s) ...
[TX] Pre-buffer complete.
[TRIMBLE] Tag lead estimate:     0.790 s
[TRIMBLE] Planned delay (lead + offset): 2.790 s
[TRIMBLE] Prep elapsed since tag: 32.989 ms
[TRIMBLE] Remaining until TX epoch: 2757.011 ms
[TX] Waiting 2.5 s for TX epoch (then 300 ms lead) ...

[TX] bladeRF device timestamp: 8215740 samples
[TX] TX lead after sleep: 294.9 ms = 766805 samples
[TX] Scheduled TX start: 8982545 samples (device_now + lead)
[TX] Streaming ...

Time into run = 1668.6^C

[TX] Shutting down ...
[BLADE] Closing device...
```
