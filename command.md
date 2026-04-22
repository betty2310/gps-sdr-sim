Distribute 5 sats around the sky


```bash
$ gps-sdr-sim -e hour0910.26n -c -1626569.949,5730535.146,2271863.661 \
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
```

# Find best PRN to revive

```bash
revive_candidates \
           -e hour1120.26n \
           -l 21.0047844,105.8460541,5 \
           --rtcm-host 192.168.5.245 \
           --rtcm-port 5005 \
           --rtcm-mount NAVIS \
           --rtcm-user NAVIS:navis123 \
           --rtcm-warmup-sec 6
```

# BladeTx with revive

```bash
 bladetx -l 21.0047844,105.8460541,22 -e hour1120.26n -P 22,14,30 \
           -S 22:revive,14:revive,30:revive \
           --trimble-time-tag-host 192.168.5.245 \
           --trimble-time-tag-port 5017 \
           --trimble-start-offset-sec 2 --txvga1 -35 --trimble-tag-lead-ms 788 --trimble-tx-cal-ns 580000
```
