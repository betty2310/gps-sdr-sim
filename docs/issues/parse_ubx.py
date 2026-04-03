#!/usr/bin/env python3
"""
Parse UBX binary log files and analyze pseudorange time-basis mismatch
between synthetic (gps-sdr-sim) and real sky GPS signals.

Usage:
    python3 parse_ubx.py <file.ubx> [--synth PRN,PRN,...] [--out parsed.txt]

Outputs:
    1. A parsed data file (TSV) with all RXM-RAWX, NAV-SAT, NAV-STATUS, NAV-PVT
    2. Console analysis of time-basis mismatch
"""

import argparse
import struct
import sys
from pathlib import Path
from collections import Counter

C_LIGHT = 299792458.0  # speed of light m/s
PR_MIN = 18e6  # minimum plausible GPS pseudorange (m)
PR_MAX = 28e6  # maximum plausible GPS pseudorange (m)


# ── UBX message parsers ──────────────────────────────────────────────────────

def iter_ubx(data):
    """Yield (class, id, payload) for each UBX frame in raw data."""
    i = 0
    end = len(data) - 5
    while i < end:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls = data[i + 2]
            mid = data[i + 3]
            length = struct.unpack_from('<H', data, i + 4)[0]
            frame_end = i + 6 + length + 2
            if frame_end <= len(data):
                yield cls, mid, data[i + 6:i + 6 + length]
                i = frame_end
                continue
        i += 1


def parse_rawx(payload):
    """Parse UBX-RXM-RAWX (0x02 0x15)."""
    if len(payload) < 16:
        return None
    rcvTow, week, leapS, numMeas, recStat, version = struct.unpack_from(
        '<dHbBBB', payload, 0)
    meas = []
    for j in range(numMeas):
        off = 16 + j * 32
        if off + 32 > len(payload):
            break
        prMes, cpMes, doMes = struct.unpack_from('<ddf', payload, off)
        gnssId = payload[off + 20]
        svId = payload[off + 21]
        sigId = payload[off + 22]
        freqId = payload[off + 23]
        locktime, cno = struct.unpack_from('<HB', payload, off + 24)
        trkStat = payload[off + 30]
        meas.append({
            'gnssId': gnssId, 'svId': svId, 'sigId': sigId, 'freqId': freqId,
            'prMes': prMes, 'cpMes': cpMes, 'doMes': doMes,
            'cno': cno, 'locktime': locktime, 'trkStat': trkStat,
        })
    return {
        'rcvTow': rcvTow, 'week': week, 'leapS': leapS,
        'numMeas': numMeas, 'recStat': recStat, 'meas': meas,
    }


def parse_nav_sat(payload):
    """Parse UBX-NAV-SAT (0x01 0x35)."""
    if len(payload) < 8:
        return None
    iTOW = struct.unpack_from('<I', payload, 0)[0]
    version = payload[4]
    numSvs = payload[5]
    sats = []
    for j in range(numSvs):
        off = 8 + j * 12
        if off + 12 > len(payload):
            break
        gnssId, svId, cno, elev = struct.unpack_from('<BBBb', payload, off)
        azim, prRes = struct.unpack_from('<hh', payload, off + 4)
        flags = struct.unpack_from('<I', payload, off + 8)[0]
        sats.append({
            'gnssId': gnssId, 'svId': svId, 'cno': cno, 'elev': elev,
            'azim': azim, 'prRes': prRes / 10.0,
            'qualityInd': flags & 0x07,
            'svUsed': bool(flags & 0x08),
            'health': (flags >> 4) & 0x03,
        })
    return {'iTOW': iTOW, 'numSvs': numSvs, 'sats': sats}


def parse_nav_status(payload):
    """Parse UBX-NAV-STATUS (0x01 0x03)."""
    if len(payload) < 16:
        return None
    iTOW, gpsFix, flags, fixStat, flags2 = struct.unpack_from(
        '<IBBBB', payload, 0)
    ttff, msss = struct.unpack_from('<II', payload, 8)
    return {
        'iTOW': iTOW, 'gpsFix': gpsFix,
        'spoofDetState': (flags2 >> 3) & 0x03,
        'ttff': ttff, 'msss': msss,
    }


def parse_nav_pvt(payload):
    """Parse UBX-NAV-PVT (0x01 0x07)."""
    if len(payload) < 92:
        return None
    fields = struct.unpack_from('<IHBBBBBx', payload, 0)
    iTOW, year, month, day, hour, minute, second = fields
    fixType = payload[20]
    numSV = payload[23]
    lon, lat, height, hMSL = struct.unpack_from('<iiII', payload, 24)
    return {
        'iTOW': iTOW, 'year': year, 'month': month, 'day': day,
        'hour': hour, 'minute': minute, 'second': second,
        'fixType': fixType, 'numSV': numSV,
        'lat': lat / 1e7, 'lon': lon / 1e7, 'height': height / 1000.0,
    }


# ── Parse all messages from file ─────────────────────────────────────────────

def parse_file(path):
    data = Path(path).read_bytes()
    rawx, nav_sat, nav_status, nav_pvt = [], [], [], []
    for cls, mid, payload in iter_ubx(data):
        if cls == 0x02 and mid == 0x15:
            r = parse_rawx(payload)
            if r:
                rawx.append(r)
        elif cls == 0x01 and mid == 0x35:
            s = parse_nav_sat(payload)
            if s:
                nav_sat.append(s)
        elif cls == 0x01 and mid == 0x03:
            st = parse_nav_status(payload)
            if st:
                nav_status.append(st)
        elif cls == 0x01 and mid == 0x07:
            p = parse_nav_pvt(payload)
            if p:
                nav_pvt.append(p)
    return data, rawx, nav_sat, nav_status, nav_pvt


# ── Write parsed data to file ────────────────────────────────────────────────

FIX_LABELS = {0: 'no_fix', 1: 'DR', 2: '2D', 3: '3D', 4: '3D+DR', 5: 'time'}
SPOOF_LABELS = {0: 'unknown', 1: 'no_spoof', 2: 'spoof_indicated', 3: 'multi_spoof'}


def write_parsed(out_path, rawx, nav_sat, nav_status, nav_pvt, synth_prns):
    with open(out_path, 'w') as f:
        # ── RXM-RAWX ──
        f.write("# UBX-RXM-RAWX — Raw pseudorange measurements\n")
        f.write("# rcvTow\tweek\tgnssId\tsvId\tsigId\tprMes\tcpMes\t"
                "doMes\tcno\tlocktime\ttrkStat\tgroup\n")
        for epoch in rawx:
            for m in epoch['meas']:
                group = 'synth' if m['svId'] in synth_prns and m['gnssId'] == 0 else 'sky'
                f.write(f"{epoch['rcvTow']:.3f}\t{epoch['week']}\t"
                        f"{m['gnssId']}\t{m['svId']}\t{m['sigId']}\t"
                        f"{m['prMes']:.3f}\t{m['cpMes']:.3f}\t"
                        f"{m['doMes']:.1f}\t{m['cno']}\t"
                        f"{m['locktime']}\t{m['trkStat']:#04x}\t{group}\n")

        # ── NAV-SAT ──
        f.write("\n# UBX-NAV-SAT — Satellite status\n")
        f.write("# iTOW_ms\tgnssId\tsvId\tcno\telev\tazim\tprRes\t"
                "qualityInd\tsvUsed\thealth\tgroup\n")
        for epoch in nav_sat:
            for s in epoch['sats']:
                group = 'synth' if s['svId'] in synth_prns and s['gnssId'] == 0 else 'sky'
                f.write(f"{epoch['iTOW']}\t{s['gnssId']}\t{s['svId']}\t"
                        f"{s['cno']}\t{s['elev']}\t{s['azim']}\t"
                        f"{s['prRes']:.1f}\t{s['qualityInd']}\t"
                        f"{'Y' if s['svUsed'] else 'N'}\t"
                        f"{s['health']}\t{group}\n")

        # ── NAV-STATUS ──
        f.write("\n# UBX-NAV-STATUS — Fix and spoofing detection\n")
        f.write("# iTOW_ms\tgpsFix\tspoofDetState\tttff_ms\tmsss_ms\n")
        for st in nav_status:
            f.write(f"{st['iTOW']}\t{st['gpsFix']}\t"
                    f"{st['spoofDetState']}\t{st['ttff']}\t{st['msss']}\n")

        # ── NAV-PVT ──
        f.write("\n# UBX-NAV-PVT — Position/velocity/time\n")
        f.write("# iTOW_ms\tyear\tmonth\tday\thour\tmin\tsec\t"
                "fixType\tnumSV\tlat\tlon\theight_m\n")
        for p in nav_pvt:
            f.write(f"{p['iTOW']}\t{p['year']}\t{p['month']}\t{p['day']}\t"
                    f"{p['hour']}\t{p['minute']}\t{p['second']}\t"
                    f"{p['fixType']}\t{p['numSV']}\t"
                    f"{p['lat']:.7f}\t{p['lon']:.7f}\t{p['height']:.1f}\n")


# ── Console analysis ─────────────────────────────────────────────────────────

def pr_normal(pr):
    return PR_MIN < pr < PR_MAX


def analyze(rawx, nav_sat, nav_status, nav_pvt, synth_prns, focus_prns):
    out = []
    def p(s=''):
        out.append(s)

    sep = '=' * 90

    # ── PRN overview ──
    p(f"Parsed: {len(rawx)} RXM-RAWX, {len(nav_sat)} NAV-SAT, "
      f"{len(nav_status)} NAV-STATUS, {len(nav_pvt)} NAV-PVT epochs")
    p()
    p(sep)
    p("PRN OVERVIEW (GPS L1 C/A, from RXM-RAWX)")
    p(sep)

    prn_data = {}
    for epoch in rawx:
        for m in epoch['meas']:
            if m['gnssId'] == 0 and m['sigId'] == 0:
                prn_data.setdefault(m['svId'], []).append(
                    (epoch['rcvTow'], m['prMes'], m['cno'], m['doMes']))

    for prn in sorted(prn_data):
        entries = prn_data[prn]
        label = 'SYNTH' if prn in synth_prns else 'SKY'
        cno_vals = [e[2] for e in entries]
        pr_vals = [e[1] for e in entries]
        p(f"  PRN {prn:2d} [{label:5s}]: {len(entries):3d} epochs, "
          f"C/N0 {min(cno_vals):2d}-{max(cno_vals):2d} dB-Hz, "
          f"PR range [{min(pr_vals):.1f}, {max(pr_vals):.1f}] m")

    # ── Pseudorange time series ──
    p()
    p(sep)
    p(f"PSEUDORANGE TIME SERIES — PRNs {focus_prns}")
    p(sep)

    epoch_data = []
    for epoch in rawx:
        tow = epoch['rcvTow']
        prn_pr = {}
        for m in epoch['meas']:
            if m['gnssId'] == 0 and m['sigId'] == 0 and m['svId'] in focus_prns:
                prn_pr[m['svId']] = (m['prMes'], m['cno'], m['doMes'], m['trkStat'])
        if prn_pr:
            epoch_data.append((tow, prn_pr))

    hdr = f"{'rcvTow':>14s}"
    for prn in focus_prns:
        tag = 'S' if prn in synth_prns else 'R'
        hdr += f" | PRN{prn:02d}({tag}) PR(m)  C/N0"
    p(hdr)
    p('-' * len(hdr))

    for tow, prn_pr in epoch_data:
        line = f"{tow:14.3f}"
        for prn in focus_prns:
            if prn in prn_pr:
                pr, cno, _, _ = prn_pr[prn]
                flag = ' ' if pr_normal(pr) else '*'
                line += f" | {pr:17.1f}{flag} {cno:3d}"
            else:
                line += f" | {'---':>17s}  {'--':>3s}"
        p(line)

    # ── Pseudorange bias analysis ──
    p()
    p(sep)
    p("PSEUDORANGE BIAS ANALYSIS: synthetic vs real")
    p(sep)
    p("For each epoch with both groups, flag PRNs outside [18M, 28M] m range.")
    p()

    for tow, prn_pr in epoch_data:
        all_prs = {pp: d[0] for pp, d in prn_pr.items()}
        anomalous = {pp: pr for pp, pr in all_prs.items() if not pr_normal(pr)}
        normal = {pp: pr for pp, pr in all_prs.items() if pr_normal(pr)}

        has_synth = any(pp in synth_prns for pp in prn_pr)
        has_real = any(pp not in synth_prns for pp in prn_pr)
        if not (has_synth and has_real):
            continue

        if anomalous:
            for pp, pr in anomalous.items():
                tag = 'SYNTH' if pp in synth_prns else 'REAL'
                offset_m = pr - 22e6
                offset_s = offset_m / C_LIGHT
                p(f"  TOW={tow:14.3f}  PRN{pp:02d}({tag:5s}): PR={pr:20.1f} m  "
                  f"offset~{offset_s:+.4f}s ({offset_m / 1e6:+.1f} Mm)")
        else:
            real_prs = {pp: prn_pr[pp][0] for pp in prn_pr if pp not in synth_prns}
            synth_prs_d = {pp: prn_pr[pp][0] for pp in prn_pr if pp in synth_prns}
            if real_prs and synth_prs_d:
                ref_prn = sorted(real_prs)[0]
                ref_pr = real_prs[ref_prn]
                for pp, pr in sorted(synth_prs_d.items()):
                    p(f"  TOW={tow:14.3f}  PRN{pp:02d}(S)-PRN{ref_prn:02d}(R): "
                      f"dPR={pr - ref_pr:+15.1f} m (both normal range)")

    # ── Epoch classification ──
    p()
    p(sep)
    p("EPOCH CLASSIFICATION: which satellite group holds the time reference?")
    p(sep)
    p("'NORMAL' = PR in [18M, 28M] m.  'OFFSET' = outside that range.")
    p()

    classifications = []
    for tow, prn_pr in epoch_data:
        has_synth = any(pp in synth_prns for pp in prn_pr)
        has_real = any(pp not in synth_prns for pp in prn_pr)
        synth_ok = all(pr_normal(prn_pr[pp][0]) for pp in prn_pr if pp in synth_prns)
        real_ok = all(pr_normal(prn_pr[pp][0]) for pp in prn_pr if pp not in synth_prns)

        if has_synth and has_real:
            if synth_ok and real_ok:
                cls = 'BOTH_NORMAL'
            elif synth_ok and not real_ok:
                cls = 'SYNTH_REF'
            elif not synth_ok and real_ok:
                cls = 'REAL_REF'
            else:
                cls = 'BOTH_OFFSET'
        elif has_synth:
            cls = 'SYNTH_ONLY'
        elif has_real:
            cls = 'REAL_ONLY'
        else:
            cls = 'EMPTY'
        classifications.append((tow, cls))
        s_count = sum(1 for pp in prn_pr if pp in synth_prns)
        r_count = sum(1 for pp in prn_pr if pp not in synth_prns)
        p(f"  TOW={tow:14.3f}  {cls:15s}  synth={s_count} real={r_count}")

    p()
    p("Summary:")
    for cls, cnt in Counter(c for _, c in classifications).most_common():
        p(f"  {cls:15s}: {cnt:3d} epochs")

    # ── Time bias estimation ──
    p()
    p(sep)
    p("TIME BIAS ESTIMATION (from offset epochs)")
    p(sep)

    bias_estimates = []
    for tow, prn_pr in epoch_data:
        synth_in = {pp: prn_pr[pp] for pp in prn_pr if pp in synth_prns}
        real_in = {pp: prn_pr[pp] for pp in prn_pr if pp not in synth_prns}
        if not synth_in or not real_in:
            continue
        synth_prs_v = [d[0] for d in synth_in.values()]
        real_prs_v = [d[0] for d in real_in.values()]
        synth_ok = all(pr_normal(pr) for pr in synth_prs_v)
        real_ok = all(pr_normal(pr) for pr in real_prs_v)
        if (synth_ok and not real_ok) or (not synth_ok and real_ok):
            offset_group = real_prs_v if not real_ok else synth_prs_v
            bias_m = sum(offset_group) / len(offset_group) - 22e6
            bias_estimates.append((tow, bias_m / C_LIGHT, bias_m))

    if bias_estimates:
        biases_s = [b[1] for b in bias_estimates]
        p(f"Estimated from {len(bias_estimates)} offset epochs:")
        p(f"  Mean:   {sum(biases_s) / len(biases_s):+.6f} s")
        p(f"  Min:    {min(biases_s):+.6f} s")
        p(f"  Max:    {max(biases_s):+.6f} s")
        p(f"  Range:  {max(biases_s) - min(biases_s):.6f} s")
        p(f"  Equiv:  {sum(biases_s) / len(biases_s) * C_LIGHT / 1e6:+.1f} Mm")
    else:
        p("No offset epochs found with both groups present.")

    # Precise bias from adjacent epoch pair
    p()
    p("Precise bias from adjacent REAL_REF / SYNTH_REF epoch pair:")
    synth_ref_epoch = real_ref_epoch = None
    for tow, prn_pr in epoch_data:
        has_synth = any(pp in synth_prns for pp in prn_pr)
        has_real = any(pp not in synth_prns for pp in prn_pr)
        if not (has_synth and has_real):
            continue
        synth_ok = all(pr_normal(prn_pr[pp][0]) for pp in prn_pr if pp in synth_prns)
        real_ok = all(pr_normal(prn_pr[pp][0]) for pp in prn_pr if pp not in synth_prns)
        if synth_ok and not real_ok:
            synth_ref_epoch = (tow, prn_pr)
        elif real_ok and not synth_ok:
            real_ref_epoch = (tow, prn_pr)
        if synth_ref_epoch and real_ref_epoch:
            break

    if synth_ref_epoch and real_ref_epoch:
        # Find common PRNs between the two epochs
        common = set(synth_ref_epoch[1]) & set(real_ref_epoch[1])
        deltas = []
        for pp in sorted(common):
            pr_s = synth_ref_epoch[1][pp][0]
            pr_r = real_ref_epoch[1][pp][0]
            delta = abs(pr_r - pr_s)
            deltas.append(delta)
            p(f"  PRN{pp:02d}: |PR_real_ref - PR_synth_ref| = {delta:,.1f} m")
        if deltas:
            avg = sum(deltas) / len(deltas)
            p(f"  Average delta: {avg:,.1f} m")
            p(f"  Time bias: {avg / C_LIGHT:.6f} s = {avg / C_LIGHT * 1000:.3f} ms")

    # ── rcvTow fractional analysis ──
    p()
    p(sep)
    p("rcvTow FRACTIONAL PART BY STATE")
    p(sep)
    synth_fracs = []
    real_fracs = []
    for (tow, cls) in classifications:
        frac = tow - int(tow)
        if cls == 'SYNTH_REF':
            synth_fracs.append(frac)
        elif cls == 'REAL_REF':
            real_fracs.append(frac)
    if synth_fracs:
        p(f"  SYNTH_REF: frac = {synth_fracs[0]:.3f} "
          f"(all {len(synth_fracs)} epochs identical: "
          f"{len(set(f'{f:.3f}' for f in synth_fracs))} unique)")
    if real_fracs:
        unique = sorted(set(f'{f:.3f}' for f in real_fracs))
        p(f"  REAL_REF:  frac = {', '.join(unique)} "
          f"({len(real_fracs)} epochs)")
    if synth_fracs and real_fracs:
        delta_frac = abs(synth_fracs[0] - real_fracs[0])
        if delta_frac > 0.5:
            delta_frac = 1.0 - delta_frac
        p(f"  Fractional offset between states: {delta_frac:.3f} s")

    # ── Transition analysis ──
    p()
    p(sep)
    p("STATE TRANSITIONS")
    p(sep)
    prev = None
    transition_count = 0
    for tow, cls in classifications:
        if cls != prev:
            transition_count += 1
            frac = tow - int(tow)
            p(f"  TOW={tow:14.3f} frac={frac:.3f} -> {cls}")
        prev = cls
    p(f"\nTotal transitions: {transition_count}")

    # ── NAV-SAT quality & used ──
    p()
    p(sep)
    p("NAV-SAT: QUALITY & USED STATUS (last 20 epochs with focus PRNs)")
    p(sep)
    shown = 0
    for ns in nav_sat[-40:]:
        gps_sats = [s for s in ns['sats']
                     if s['gnssId'] == 0 and s['svId'] in focus_prns]
        if not gps_sats:
            continue
        p(f"  iTOW={ns['iTOW'] / 1000:.3f}s:")
        for s in sorted(gps_sats, key=lambda x: x['svId']):
            tag = 'SYNTH' if s['svId'] in synth_prns else 'REAL'
            used = 'USED' if s['svUsed'] else '----'
            p(f"    PRN{s['svId']:02d}({tag:5s}): C/N0={s['cno']:2d} "
              f"el={s['elev']:+3d} az={s['azim']:3d} "
              f"qInd={s['qualityInd']} prRes={s['prRes']:+7.1f}m {used}")
        shown += 1
        if shown >= 20:
            break

    # ── NAV-STATUS spoofing ──
    p()
    p(sep)
    p("NAV-STATUS: SPOOFING DETECTION")
    p(sep)
    for st in nav_status:
        fix = FIX_LABELS.get(st['gpsFix'], f"?{st['gpsFix']}")
        spoof = SPOOF_LABELS.get(st['spoofDetState'], f"?{st['spoofDetState']}")
        p(f"  iTOW={st['iTOW'] / 1000:10.3f}s  fix={fix:8s}  "
          f"spoofDet={st['spoofDetState']}({spoof})")

    # ── NAV-PVT position ──
    p()
    p(sep)
    p("NAV-PVT: POSITION SOLUTION")
    p(sep)
    for pv in nav_pvt:
        fix = FIX_LABELS.get(pv['fixType'], f"?{pv['fixType']}")
        p(f"  {pv['year']:04d}-{pv['month']:02d}-{pv['day']:02d} "
          f"{pv['hour']:02d}:{pv['minute']:02d}:{pv['second']:02d}  "
          f"fix={fix:5s}  numSV={pv['numSV']:2d}  "
          f"lat={pv['lat']:11.7f}  lon={pv['lon']:12.7f}  "
          f"hgt={pv['height']:8.1f}m")

    return '\n'.join(out)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Parse UBX log and analyze time-basis mismatch.')
    parser.add_argument('ubx_file', help='Path to .ubx log file')
    parser.add_argument('--synth', default='3,4,7,8',
                        help='Comma-separated synthetic PRN list (default: 3,4,7,8)')
    parser.add_argument('--focus', default=None,
                        help='Comma-separated PRNs for detailed view '
                             '(default: synth PRNs + first sky PRN)')
    parser.add_argument('--out', default=None,
                        help='Output path for parsed TSV data '
                             '(default: <input>_parsed.txt)')
    args = parser.parse_args()

    synth_prns = set(int(x) for x in args.synth.split(','))

    ubx_path = Path(args.ubx_file)
    if not ubx_path.exists():
        print(f"Error: {ubx_path} not found", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {ubx_path} ({ubx_path.stat().st_size} bytes)...")
    data, rawx, nav_sat, nav_status, nav_pvt = parse_file(ubx_path)

    # Auto-detect focus PRNs if not specified
    if args.focus:
        focus_prns = [int(x) for x in args.focus.split(',')]
    else:
        # Find sky PRNs from RAWX data
        sky_prns = set()
        for epoch in rawx:
            for m in epoch['meas']:
                if m['gnssId'] == 0 and m['sigId'] == 0 and m['svId'] not in synth_prns:
                    sky_prns.add(m['svId'])
        # Pick the most frequent sky PRN
        sky_counts = Counter()
        for epoch in rawx:
            for m in epoch['meas']:
                if m['gnssId'] == 0 and m['sigId'] == 0 and m['svId'] in sky_prns:
                    sky_counts[m['svId']] += 1
        top_sky = [prn for prn, _ in sky_counts.most_common(2)]
        focus_prns = sorted(synth_prns) + top_sky

    # Write parsed data
    out_path = args.out or str(ubx_path.with_suffix('')) + '_parsed.txt'
    write_parsed(out_path, rawx, nav_sat, nav_status, nav_pvt, synth_prns)
    print(f"Parsed data written to: {out_path}")

    # Run analysis
    print()
    result = analyze(rawx, nav_sat, nav_status, nav_pvt, synth_prns, focus_prns)
    print(result)


if __name__ == '__main__':
    main()
