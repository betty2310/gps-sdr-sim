# UBX Best Tuple Tool

## Purpose

`tools/ubx_best_tuple.py` reads a ZED-F9P `.ubx` log and emits the best GPS tuple the log supports:

- `tx_week`
- `tx_tow`
- `rx_week`
- `rx_tow`

## Required UBX Messages

- `UBX-RXM-RAWX`
- `UBX-NAV-TIMEGPS`
- `UBX-NAV-PVT`
- `UBX-RXM-SFRBX`

## Invocation

```bash
python3 tools/ubx_best_tuple.py sample.ubx > tuples.csv
```

## Output Columns

- `sv_id`
- `sig_id`
- `tx_week`
- `tx_tow`
- `rx_week`
- `rx_tow`

## Acceptance Rules

- GPS measurements only (`gnssId == 0`)
- pseudorange valid (`trkStat & 0x01`)
- `NAV-TIMEGPS` week and TOW valid
- nearest `NAV-TIMEGPS` epoch must match the `RAWX` epoch within `10 ms`
- `NAV-PVT` valid time plus `flags2.confirmedTime`
- `RAWX.recStat` must not report clock reset
- GPS broadcast clock terms must exist for the satellite

## Limitations

- `tx_*` is still a derived value, not a direct receiver field
- current plan uses GPS broadcast clock correction, not precise external orbit/clock products
- `TIM-TM2` is not used to move the observation epoch; it is only useful later for receiver timing validation
