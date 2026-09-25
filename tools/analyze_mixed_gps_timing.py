#!/usr/bin/env python3
"""Offline mixed GPS timing reconstruction from checksummed UBX recordings.

Requires numpy, pandas and matplotlib. No radio, receiver or input-file writes.
Decode broadcast GPS L1 ephemerides from the recordings, remove satellite
geometry/clock, then difference revived and sky observations at each receiver
measurement epoch. Absolute receiver clock terms cancel in this difference.
Atmosphere and multipath are not modelled; treat small residuals accordingly.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np
import pandas as pd

C = 299792458.0
MU = 3.986005e14
OMEGA = 7.2921151467e-5


def field(data, start, size, signed=False):
    n = int(data[start:start+size], 2)
    return n - (1 << size) if signed and n & (1 << (size-1)) else n

def decode(data, week_hint=0):
    sf = field(data, 43, 3)
    result = {'sf': sf, 'how_tow': field(data, 24, 17)*6}
    cursor = 48

    def take(n, scale=1, signed=False):
        nonlocal cursor
        value = field(data, cursor, n, signed)*scale
        cursor += n
        return value

    if sf == 1:
        w10 = take(10)
        result['week'] = w10 + round((week_hint-w10)/1024)*1024
        result['code'] = take(2)
        result['ura'] = take(4)
        result['health'] = take(6)
        iodc_hi = take(2)
        result['flag'] = take(1)
        cursor += 87
        tgd = take(8, signed=True)
        result['tgd'] = 0 if tgd == -128 else tgd*2**-31
        result['iodc'] = (iodc_hi << 8) + take(8)
        result['toc'] = take(16, 16)
        result['f2'] = take(8, 2**-55, True)
        result['f1'] = take(16, 2**-43, True)
        result['f0'] = take(22, 2**-31, True)
    elif sf == 2:
        result['iode'] = take(8)
        result['crs'] = take(16, 2**-5, True)
        result['dn'] = take(16, math.pi*2**-43, True)
        result['m0'] = take(32, math.pi*2**-31, True)
        result['cuc'] = take(16, 2**-29, True)
        result['ecc'] = take(32, 2**-33)
        result['cus'] = take(16, 2**-29, True)
        result['sqrt_a'] = take(32, 2**-19)
        result['toe'] = take(16, 16)
    elif sf == 3:
        result['cic'] = take(16, 2**-29, True)
        result['omega0'] = take(32, math.pi*2**-31, True)
        result['cis'] = take(16, 2**-29, True)
        result['i0'] = take(32, math.pi*2**-31, True)
        result['crc'] = take(16, 2**-5, True)
        result['arg_perigee'] = take(32, math.pi*2**-31, True)
        result['omega_dot'] = take(24, math.pi*2**-43, True)
        result['iode'] = take(8)
        result['i_dot'] = take(14, math.pi*2**-43, True)
    return result

def weekdiff(a, b):
    return (a - b + 302400) % 604800 - 302400

def satpos(ep, tow):
    tk = weekdiff(tow, ep['toe'])
    a, ecc = ep['sqrt_a']**2, ep['ecc']
    m = ep['m0'] + (math.sqrt(MU/a**3) + ep['dn'])*tk
    anomaly = m
    for _ in range(30):
        step = (anomaly - ecc*math.sin(anomaly) - m)/(1 - ecc*math.cos(anomaly))
        anomaly -= step
        if abs(step) < 1e-14:
            break
    se, ce = math.sin(anomaly), math.cos(anomaly)
    u = math.atan2(math.sqrt(1-ecc**2)*se, ce-ecc) + ep['arg_perigee']
    r = a*(1-ecc*ce)
    inc = ep['i0'] + ep['i_dot']*tk
    s2, c2 = math.sin(2*u), math.cos(2*u)
    u += ep['cus']*s2 + ep['cuc']*c2
    r += ep['crs']*s2 + ep['crc']*c2
    inc += ep['cis']*s2 + ep['cic']*c2
    x, y = r*math.cos(u), r*math.sin(u)
    omega = ep['omega0'] + (ep['omega_dot']-OMEGA)*tk - OMEGA*ep['toe']
    co, so, ci, si = math.cos(omega), math.sin(omega), math.cos(inc), math.sin(inc)
    xyz = np.array([x*co-y*ci*so, x*so+y*ci*co, y*si])
    tc = weekdiff(tow, ep['toc'])
    clock = ep['f0'] + ep['f1']*tc + ep['f2']*tc**2 - 2*math.sqrt(MU*a)*ecc*se/C**2
    return xyz, clock

def receiver_ecef(lat, lon, height):
    p, l = np.deg2rad([lat, lon])
    e2 = 6.6943799901413165e-3
    n = 6378137/math.sqrt(1-e2*math.sin(p)**2)
    return np.array([(n+height)*math.cos(p)*math.cos(l),
                     (n+height)*math.cos(p)*math.sin(l),
                     (n*(1-e2)+height)*math.sin(p)])

def azel(satellite, receiver, lat, lon):
    p, l = np.deg2rad([lat, lon])
    dx, dy, dz = satellite-receiver
    east = -math.sin(l)*dx + math.cos(l)*dy
    north = -math.sin(p)*math.cos(l)*dx - math.sin(p)*math.sin(l)*dy + math.cos(p)*dz
    up = math.cos(p)*math.cos(l)*dx + math.cos(p)*math.sin(l)*dy + math.sin(p)*dz
    return math.degrees(math.atan2(east, north)) % 360, math.degrees(math.atan2(up, math.hypot(east, north)))

def range_and_clock(ep, tx, receiver):
    xyz, clk = satpos(ep, tx)
    rho = np.linalg.norm(xyz - receiver) + OMEGA/C*(xyz[0]*receiver[1]-xyz[1]*receiver[0])
    return rho, clk - ep['tgd'], xyz

def effective_range(ep, tx, receiver):
    rho, clk, _ = range_and_clock(ep, tx, receiver)
    return rho - C*clk


def reconstruct_observation(ep, tow, pseudorange, doppler, receiver):
    # Receiver clock cancels in this transmission timestamp. Satellite clock
    # (including L1 group delay) is applied exactly once, not NAV-CLOCK again.
    tx_local = tow-pseudorange/C
    tx = tx_local
    for _ in range(3):
        _, clk = satpos(ep, tx)
        tx = tx_local-(clk-ep['tgd'])
    rho, clk, satellite = range_and_clock(ep, tx, receiver)
    rate = (effective_range(ep, tx+.1, receiver)-effective_range(ep, tx-.1, receiver))/.2
    return dict(tx_tow=tx, apparent_clock_m=pseudorange-rho+C*clk,
                carrier_clock_m_s=-doppler*C/1575.42e6-rate), satellite


def read_capture(path):
    data = path.read_bytes()
    tables = {k: [] for k in ('raw', 'nav', 'clock', 'pvt', 'status', 'sf')}
    counts, bad, cursor, epoch = Counter(), 0, 0, None
    week = None
    identity = []
    while cursor < len(data):
        start = data.find(b'\xb5\x62', cursor)
        if start < 0 or start + 8 > len(data):
            break
        size = int.from_bytes(data[start+4:start+6], 'little')
        end = start + size + 8
        if end > len(data):
            break
        a = b = 0
        for value in data[start+2:end-2]:
            a = (a + value) & 255
            b = (b + a) & 255
        if data[end-2:end] != bytes((a, b)):
            bad += 1
            cursor = start + 1
            continue
        cls, mid = data[start+2:start+4]
        counts[f'{cls:02x}-{mid:02x}'] += 1
        p = data[start+6:end-2]
        cursor = end
        if (cls, mid) == (10, 4) and size >= 40 and (size-40) % 30 == 0:
            identity = [p[:30].split(b'\0')[0].decode('ascii', errors='replace'),
                        p[30:40].split(b'\0')[0].decode('ascii', errors='replace')]
            identity += [p[j:j+30].split(b'\0')[0].decode('ascii', errors='replace')
                         for j in range(40, size, 30)]
        elif (cls, mid) == (2, 0x15) and size >= 16 and p[13] == 1 and size == 16 + 32*p[11]:
            tow, week = struct.unpack_from('<dH', p)
            epoch = round(tow)
            for j in range(16, size, 32):
                pr, cp, dop = struct.unpack_from('<ddf', p, j)
                flags = p[j+30]
                tables['raw'].append(dict(epoch=epoch, tow=tow, week=week,
                    reset=bool(p[12] & 2), prn=p[j+21], gnss=p[j+20], signal=p[j+22],
                    pr=pr, cp=cp, doppler=dop, lock_ms=int.from_bytes(p[j+24:j+26], 'little'),
                    cno=p[j+26], pr_valid=bool(flags & 1), cp_valid=bool(flags & 2),
                    half_resolved=bool(flags & 4)))
        elif (cls, mid) == (1, 0x35) and size >= 8 and size == 8 + 12*p[5]:
            tow = int.from_bytes(p[:4], 'little') / 1000
            for j in range(8, size, 12):
                gnss, prn, cno, el, az, residual, flags = struct.unpack_from('<BBBbhhI', p, j)
                tables['nav'].append(dict(epoch=round(tow), gnss=gnss, prn=prn,
                    el=el, az=az, pr_residual_m=residual*.1, used=bool(flags & 8),
                    quality=flags & 7, eph_available=bool(flags & 2048)))
        elif (cls, mid) == (1, 0x22) and size == 20:
            tow, bias, drift, tacc, facc = struct.unpack('<IiiII', p)
            tables['clock'].append(dict(epoch=round(tow/1000), bias_ns=bias,
                drift_ns_s=drift, tacc_ns=tacc))
        elif (cls, mid) == (1, 7) and size == 92:
            tow = int.from_bytes(p[:4], 'little')/1000
            lon, lat, height = struct.unpack_from('<iii', p, 24)
            tables['pvt'].append(dict(epoch=round(tow), fix=p[20], fix_ok=bool(p[21] & 1),
                num_sv=p[23], lat=lat*1e-7, lon=lon*1e-7, height=height*.001))
        elif (cls, mid) == (1, 3) and size == 16:
            tow = int.from_bytes(p[:4], 'little')/1000
            tables['status'].append(dict(epoch=round(tow), fix=p[4], fix_ok=bool(p[5] & 1),
                spoof_state=(p[7] >> 3) & 3, uptime_ms=int.from_bytes(p[12:16], 'little')))
        elif (cls, mid) == (2, 0x13) and size == 48 and p[0] == 0 and p[2] == 0:
            words = struct.unpack_from('<10I', p, 8)
            bits = ''.join(f'{(w >> 6) & 0xffffff:024b}' for w in words)
            if field(bits, 0, 8) != 0x8b:
                continue
            decoded = decode(bits, week or 0)
            if decoded['sf'] in (1, 2, 3):
                decoded.update(prn=p[1], epoch=epoch, payload_hash=hashlib.sha256(bits[48:].encode()).hexdigest())
                tables['sf'].append(decoded)
    weeks = {r['week'] for r in tables['raw']}
    if len(weeks) != 1:
        raise ValueError(f'{path}: analysis requires nonempty RAWX from a single GPS week')
    week_hint = next(iter(weeks))
    # A subframe may precede the first RAWX in the file.
    for sf in tables['sf']:
        if sf['sf'] == 1:
            w10 = sf['week'] % 1024
            sf['week'] = w10 + round((week_hint-w10)/1024)*1024
    meta = dict(path=str(path.resolve()), sha256=hashlib.sha256(data).hexdigest(),
                bytes=len(data), messages=dict(counts), checksum_errors=bad, identity=identity)
    return {k: pd.DataFrame(v) for k, v in tables.items()}, meta


def ephemerides(subframes):
    candidates = []
    for prn, group in subframes.groupby('prn'):
        # Preserve every distinct orbit issue; never substitute sky ephemerides
        # for a revived satellite or silently merge different IODE values.
        for _, two in group.loc[group.sf.eq(2)].drop_duplicates('payload_hash').iterrows():
            ones = group.loc[group.sf.eq(1) & group.iodc.mod(256).eq(two.iode)]
            threes = group.loc[group.sf.eq(3) & group.iode.eq(two.iode)]
            for _, one in ones.drop_duplicates('payload_hash').iterrows():
                for _, three in threes.drop_duplicates('payload_hash').iterrows():
                    ep = {}
                    for part in (one, two, three):
                        ep.update({k: v for k, v in part.dropna().items()
                                   if k not in ('sf', 'how_tow', 'epoch', 'payload_hash', 'receiver')})
                    ep['payloads'] = ':'.join(p.payload_hash for p in (one, two, three))
                    ep['first_complete_epoch'] = max(p.epoch for p in (one, two, three))
                    candidates.append(ep)
    return pd.DataFrame(candidates).drop_duplicates(['prn', 'payloads'])


def segments(epochs):
    groups = []
    for epoch in sorted(set(int(e) for e in epochs)):
        if not groups or epoch != groups[-1][-1]+1:
            groups.append([epoch])
        else:
            groups[-1].append(epoch)
    return [dict(start=g[0], end=g[-1], epochs=len(g)) for g in groups]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('captures', type=Path, nargs='+')
    ap.add_argument('--revived', type=int, nargs='+', required=True)
    ap.add_argument('--sky-prns', type=int, nargs='+', required=True,
                    help='Independently identified authentic GPS PRNs; not every other track is sky')
    ap.add_argument('--llh', type=float, nargs=3, required=True,
                    help='Fixed analysis latitude, longitude, ellipsoid height')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if len(args.captures) != 2:
        ap.error('supply exactly two receiver recordings')
    if set(args.revived) & set(args.sky_prns):
        ap.error('revived and sky PRNs must be disjoint')
    args.output.mkdir(parents=True, exist_ok=True)
    captures, inputs = {}, []
    for path in args.captures:
        captures[path.stem], meta = read_capture(path)
        inputs.append(meta)
    sf = pd.concat([t['sf'].assign(receiver=rx) for rx, t in captures.items()], ignore_index=True)
    if sf.empty:
        raise RuntimeError('no GPS L1 subframes decoded')
    eph = ephemerides(sf)
    sf.to_csv(args.output/'subframes.csv', index=False)
    eph.to_csv(args.output/'ephemerides.csv', index=False)
    receiver = receiver_ecef(*args.llh)
    rows, summary = [], dict(inputs=inputs, analysis_llh=args.llh, revived_prns=args.revived,
                            sky_prns=args.sky_prns, receivers={})
    common_phase = None
    for rx, tables in captures.items():
        for name, table in tables.items():
            table.assign(receiver=rx).to_csv(args.output/f'{rx}-{name}.csv', index=False)
        raw = tables['raw']
        l1 = raw.loc[raw.gnss.eq(0) & raw.signal.eq(0)]
        phase = l1.loc[l1.cp_valid & l1.half_resolved & l1.prn.isin(args.revived)]
        complete = set(phase.groupby('epoch').prn.nunique().loc[lambda n: n.eq(len(args.revived))].index)
        common_phase = complete if common_phase is None else common_phase & complete
        pvt = tables['pvt']
        rs = dict(raw_epochs=int(raw.epoch.nunique()), first_tow=float(raw.tow.min()),
                  last_tow=float(raw.tow.max()), valid_fixes=int(pvt.fix_ok.sum()), pvt_epochs=len(pvt),
                  clock_reset_epochs=sorted(raw.loc[raw.reset, 'epoch'].unique().tolist()),
                  complete_revived_phase=segments(complete), per_prn={})
        for prn in args.revived:
            q = l1.loc[l1.prn.eq(prn)]
            nav = tables['nav'].loc[lambda n: n.gnss.eq(0) & n.prn.eq(prn)]
            rs['per_prn'][str(prn)] = dict(observations=len(q),
                phase_segments=segments(phase.loc[phase.prn.eq(prn), 'epoch']),
                used_epochs=segments(nav.loc[nav.used, 'epoch']))
        summary['receivers'][rx] = rs
        for o in l1.loc[l1.pr_valid].itertuples():
            choices = eph.loc[eph.prn.eq(o.prn) & eph.health.eq(0)]
            if choices.empty:
                continue
            ep = choices.iloc[np.argmin(np.abs(choices.toe-o.tow))].to_dict()
            # rcvTow - pseudorange/c eliminates the receiver clock, yielding
            # the satellite-clock transmission epoch. Apply SV clock once.
            reconstructed, satellite = reconstruct_observation(ep, o.tow, o.pr, o.doppler, receiver)
            az, el = azel(satellite, receiver, args.llh[0], args.llh[1])
            rows.append(dict(receiver=rx, epoch=o.epoch, tow=o.tow, prn=o.prn,
                revived=o.prn in args.revived, phase_valid=o.cp_valid and o.half_resolved,
                cno=o.cno, lock_ms=o.lock_ms, pseudorange_m=o.pr,
                **reconstructed, az=az, el=el, iode=ep['iode'], toe=ep['toe']))
    obs = pd.DataFrame(rows)
    # Use sky signals with resolved carrier and reasonable elevation for the
    # common receiver clock. Require at least three independent sky PRNs.
    sky = obs.loc[obs.prn.isin(args.sky_prns) & obs.phase_valid & obs.el.ge(10) & obs.cno.ge(25)]
    clocks = sky.groupby(['receiver', 'epoch']).agg(
        sky_clock_m=('apparent_clock_m', 'median'), sky_prns=('prn', 'nunique'),
        sky_drift_m_s=('carrier_clock_m_s', 'median'))
    clocks = clocks.loc[clocks.sky_prns.ge(3)]
    obs = obs.merge(clocks, on=['receiver', 'epoch'], how='left')
    obs['relative_delay_ms'] = (obs.apparent_clock_m-obs.sky_clock_m)/C*1e3
    obs['relative_drift_ppm'] = (obs.carrier_clock_m_s-obs.sky_drift_m_s)/C*1e6
    obs.to_csv(args.output/'timing_observations.csv', index=False)
    summary['decoded_prns'] = sorted(eph.prn.astype(int).unique().tolist())
    summary['common_complete_phase'] = segments(common_phase)
    clean = obs.loc[obs.revived & obs.phase_valid & obs.epoch.isin(common_phase)].dropna(subset=['relative_delay_ms'])
    if clean.empty:
        raise RuntimeError('no paired complete phase window with at least three sky PRNs')
    # The calibration uses the first half of the shared valid-phase epochs.
    # The second half is only a retrospective holdout, not an RF re-test.
    ordered = sorted(common_phase)
    split = ordered[len(ordered)//2]
    calibration = clean.loc[clean.epoch.lt(split)]
    holdout = clean.loc[clean.epoch.ge(split)]
    correction_ms = calibration.groupby('receiver').relative_delay_ms.median().mean()
    residual_us = (holdout.relative_delay_ms-correction_ms)*1000
    summary['empirical_epoch_correction'] = dict(
        model_time_offset_s=correction_ms/1000, positive_means='advance_model_epoch_at_same_hardware_start',
        calibration_first_tow=int(calibration.epoch.min()), calibration_last_tow=int(calibration.epoch.max()),
        holdout_first_tow=int(holdout.epoch.min()), holdout_last_tow=int(holdout.epoch.max()),
        holdout_median_residual_us=float(residual_us.median()),
        holdout_p95_abs_residual_us=float(residual_us.abs().quantile(.95)),
        rf_retest_performed=False, absolute_gps_alignment_verified=False)
    for rx in captures:
        q = obs.loc[obs.receiver.eq(rx) & obs.revived & obs.phase_valid & obs.epoch.isin(common_phase)].dropna(subset=['relative_delay_ms'])
        group = q.groupby('epoch').relative_delay_ms.median()
        fit = np.polyfit(group.index-group.index.min(), group*1000, 1) if len(group)>1 else [np.nan, np.nan]
        summary['receivers'][rx]['paired_phase_timing'] = dict(
            epochs=len(group), origin_tow=int(group.index.min()) if len(group) else None,
            offset_at_origin_us=fit[1], slope_us_per_s=fit[0],
            doppler_relative_drift_ppm=float(q.relative_drift_ppm.median()),
            median_delay_ms=float(q.relative_delay_ms.median()),
            per_prn_delay_ms=q.groupby('prn').relative_delay_ms.median().to_dict(),
            min_sky_prns=int(q.sky_prns.min()) if len(q) else 0)
        nav = captures[rx]['nav'].loc[lambda n: n.gnss.eq(0) & n.el.ge(-90)]
        geometry = obs.loc[obs.receiver.eq(rx)].merge(nav, on=['epoch', 'prn'], suffixes=('_model', '_nav'))
        da = (geometry.az_model-geometry.az_nav+180)%360-180
        summary['receivers'][rx]['geometry_check'] = dict(
            rows=len(geometry), median_abs_az_error_deg=float(da.abs().median()),
            median_abs_el_error_deg=float((geometry.el_model-geometry.el_nav).abs().median()))
        sky_rows = obs.loc[obs.receiver.eq(rx) & obs.prn.isin(args.sky_prns) & obs.epoch.isin(common_phase)]
        summary['receivers'][rx]['sky_residual_rms_m'] = float(np.sqrt(np.mean((sky_rows.relative_delay_ms*C/1000)**2)))
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex='col', constrained_layout=True)
    origin = min(t['raw'].epoch.min() for t in captures.values())
    for col, (rx, tables) in enumerate(captures.items()):
        for prn in args.revived:
            q = obs.loc[obs.receiver.eq(rx) & obs.prn.eq(prn)]
            valid = q.loc[q.phase_valid]
            unresolved = q.loc[~q.phase_valid]
            axes[0,col].plot(unresolved.epoch-origin, unresolved.relative_delay_ms, '.', color='.75', ms=3)
            axes[1,col].plot(unresolved.epoch-origin, unresolved.relative_drift_ppm, '.', color='.75', ms=3)
            axes[0,col].plot(valid.epoch-origin, valid.relative_delay_ms, '.', ms=3, label=f'PRN {prn}')
            axes[1,col].plot(valid.epoch-origin, valid.relative_drift_ppm, '.', ms=3)
            axes[2,col].plot(valid.epoch-origin, np.full(len(valid), prn), '.', ms=4)
        axes[0,col].set_title(rx.split('_')[0]); axes[0,col].legend(ncol=2)
        axes[2,col].set_xlabel(f'Seconds from GPS TOW {origin}')
        for ax in axes[:,col]: ax.grid(alpha=.25)
    axes[0,0].set_ylabel('Revived minus sky delay (ms)')
    axes[1,0].set_ylabel('Doppler-derived relative drift (ppm)')
    axes[2,0].set_ylabel('PRN with valid resolved phase')
    fig.suptitle('Colored: resolved carrier phase; gray: unresolved observations (not calibration data)')
    fig.savefig(args.output/'timing.png', dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, constrained_layout=True)
    for col, rx in enumerate(captures):
        for prn in args.revived:
            q = clean.loc[clean.receiver.eq(rx) & clean.prn.eq(prn)]
            axes[0,col].plot(q.epoch-ordered[0], (q.relative_delay_ms-correction_ms)*1000,
                             '.-', ms=3, lw=.7, label=f'PRN {prn}')
            axes[1,col].plot(q.epoch-ordered[0], q.relative_drift_ppm*1000, '.', ms=3)
        axes[0,col].set_title(rx.split('_')[0]); axes[0,col].legend(ncol=2)
        axes[1,col].set_xlabel(f'Seconds from GPS TOW {ordered[0]}')
        for ax in axes[:,col]:
            ax.grid(alpha=.25)
            ax.axvline(split-ordered[0]-.5, color='.4', ls='--', lw=1)
    axes[0,0].set_ylabel('Delay after subtracting fitted offset (µs)')
    axes[1,0].set_ylabel('Doppler-derived relative drift (ns/s)')
    fig.suptitle(f'Offline subtraction of {correction_ms:.7f} ms; dashed line starts held-out epochs')
    fig.savefig(args.output/'clean-window.png', dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
