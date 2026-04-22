#!/usr/bin/env python3
"""Analyze ver3 revive-mode recordings. PRNs 14, 22, 30 injected."""

from pathlib import Path
import pandas as pd
from pyubx2 import UBXReader

GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 5: "QZSS", 6: "GLO"}
INJECTED = {14, 22, 30}

SPOOF_LBL = {0: "unknown", 1: "no_spoof", 2: "spoof_indicated", 3: "multi_spoof"}
JAM_LBL = {0: "unknown", 1: "ok", 2: "warning", 3: "critical"}


def _attr(msg, parsed, name):
    if name in msg:
        return msg[name]
    return getattr(parsed, name, None)


def parse_all(path: Path):
    rawx, nsat, nsig, nstat, npvt, secsig = [], [], [], [], [], []
    with open(path, "rb") as f:
        for raw, parsed in UBXReader(f, protfilter=2):
            if parsed is None:
                continue
            msg = parsed.__dict__ if not hasattr(parsed, "val") else parsed.val
            ident = parsed.identity

            if ident == "RXM-RAWX":
                rcv = _attr(msg, parsed, "rcvTow")
                n = _attr(msg, parsed, "numMeas")
                if rcv is None or n is None:
                    continue
                for i in range(1, n + 1):
                    s = f"_{i:02d}"
                    pr = _attr(msg, parsed, f"prMes{s}")
                    sv = _attr(msg, parsed, f"svId{s}")
                    g = _attr(msg, parsed, f"gnssId{s}")
                    cno = _attr(msg, parsed, f"cno{s}")
                    if sv is None or pr is None or pr <= 0:
                        continue
                    rawx.append({"rcvTow": rcv, "gnssId": g, "svId": int(sv),
                                 "prMes": pr, "cno": cno})
            elif ident == "NAV-SAT":
                itow = _attr(msg, parsed, "iTOW")
                n = _attr(msg, parsed, "numSvs")
                if itow is None or n is None:
                    continue
                for i in range(1, n + 1):
                    s = f"_{i:02d}"
                    sv = _attr(msg, parsed, f"svId{s}")
                    g = _attr(msg, parsed, f"gnssId{s}")
                    if sv is None:
                        continue
                    nsat.append({
                        "iTOW": itow, "gnssId": g, "svId": int(sv),
                        "cno": _attr(msg, parsed, f"cno{s}"),
                        "elev": _attr(msg, parsed, f"elev{s}"),
                        "azim": _attr(msg, parsed, f"azim{s}"),
                        "prRes": _attr(msg, parsed, f"prRes{s}"),
                        "svUsed": _attr(msg, parsed, f"svUsed{s}"),
                        "health": _attr(msg, parsed, f"health{s}"),
                        "ephAvail": _attr(msg, parsed, f"ephAvail{s}"),
                        "orbitSource": _attr(msg, parsed, f"orbitSource{s}"),
                        "qualityInd": _attr(msg, parsed, f"qualityInd{s}"),
                    })
            elif ident == "NAV-SIG":
                itow = _attr(msg, parsed, "iTOW")
                n = _attr(msg, parsed, "numSigs")
                if itow is None or n is None:
                    continue
                for i in range(1, n + 1):
                    s = f"_{i:02d}"
                    sv = _attr(msg, parsed, f"svId{s}")
                    g = _attr(msg, parsed, f"gnssId{s}")
                    if sv is None:
                        continue
                    nsig.append({
                        "iTOW": itow, "gnssId": g, "svId": int(sv),
                        "sigId": _attr(msg, parsed, f"sigId{s}"),
                        "cno": _attr(msg, parsed, f"cno{s}"),
                        "prRes": _attr(msg, parsed, f"prRes{s}"),
                        "prUsed": _attr(msg, parsed, f"prUsed{s}"),
                        "qualityInd": _attr(msg, parsed, f"qualityInd{s}"),
                    })
            elif ident == "NAV-STATUS":
                itow = _attr(msg, parsed, "iTOW")
                if itow is None:
                    continue
                nstat.append({
                    "iTOW": itow,
                    "gpsFix": _attr(msg, parsed, "gpsFix"),
                    "spoofDetState": _attr(msg, parsed, "spoofDetState"),
                    "jammingState": _attr(msg, parsed, "jammingState"),
                })
            elif ident == "NAV-PVT":
                itow = _attr(msg, parsed, "iTOW")
                if itow is None:
                    continue
                npvt.append({
                    "iTOW": itow,
                    "fixType": _attr(msg, parsed, "fixType"),
                    "numSV": _attr(msg, parsed, "numSV"),
                    "lat": _attr(msg, parsed, "lat"),
                    "lon": _attr(msg, parsed, "lon"),
                    "height": _attr(msg, parsed, "height"),
                    "hAcc": _attr(msg, parsed, "hAcc"),
                    "vAcc": _attr(msg, parsed, "vAcc"),
                })
            elif ident == "SEC-SIG":
                spoof = _attr(msg, parsed, "spoofingState")
                jam = _attr(msg, parsed, "jammingState")
                if spoof is None and jam is None:
                    continue
                secsig.append({
                    "jammingState": jam,
                    "spoofingState": spoof,
                })

    return (pd.DataFrame(rawx), pd.DataFrame(nsat), pd.DataFrame(nsig),
            pd.DataFrame(nstat), pd.DataFrame(npvt), pd.DataFrame(secsig))


def analyze(path: Path, label: str):
    print(f"\n{'=' * 72}")
    print(f"  {label}  —  {path.name}")
    print('=' * 72)

    rawx, nsat, nsig, nstat, npvt, secsig = parse_all(path)

    if rawx.empty:
        print("No RAWX data.")
        return

    rawx_gps = rawx[rawx["gnssId"] == 0]
    n_epochs = rawx_gps["rcvTow"].nunique()
    span = rawx_gps["rcvTow"].max() - rawx_gps["rcvTow"].min()
    print(f"\nRAWX GPS: {n_epochs} epochs over {span:.1f} s, "
          f"{rawx_gps['svId'].nunique()} GPS PRNs tracked")

    # CNo per PRN, mark injected
    cno_per = (rawx_gps.groupby("svId")
               .agg(epochs=("rcvTow", "nunique"),
                    mean_cno=("cno", "mean"),
                    min_cno=("cno", "min"),
                    max_cno=("cno", "max"))
               .round(1))
    cno_per["injected"] = cno_per.index.isin(INJECTED)
    print("\n-- RAWX per-PRN (GPS only) --")
    print(cno_per.sort_values("epochs", ascending=False).to_string())

    # NAV-SAT svUsed
    if not nsat.empty:
        nsat_gps = nsat[nsat["gnssId"] == 0].copy()
        nsat_epochs = nsat_gps["iTOW"].nunique()
        svused = (nsat_gps.groupby("svId")
                  .agg(epochs=("iTOW", "nunique"),
                       svUsed_epochs=("svUsed", "sum"),
                       mean_cno=("cno", "mean"),
                       mean_elev=("elev", "mean"),
                       mean_azim=("azim", "mean"),
                       mean_prRes=("prRes", "mean"),
                       abs_prRes=("prRes", lambda s: s.abs().mean()),
                       max_abs_prRes=("prRes", lambda s: s.abs().max()),
                       ephAvail=("ephAvail", "sum"),
                       orbitSrc=("orbitSource", "max"))
                  .round(1))
        svused["svUsed_rate"] = (svused["svUsed_epochs"] / svused["epochs"]).round(3)
        svused["injected"] = svused.index.isin(INJECTED)
        print(f"\n-- NAV-SAT per-PRN (GPS, {nsat_epochs} epochs) --")
        print(svused.sort_values(
            ["svUsed_epochs", "mean_cno"], ascending=[False, False]).to_string())

        # Injected-only focus
        inj = svused[svused["injected"]]
        real = svused[~svused["injected"]]
        print(f"\n-- Injected PRNs {sorted(INJECTED)} --")
        if inj.empty:
            print("NOT seen in NAV-SAT")
        else:
            print(inj.to_string())
            total_used = int(inj["svUsed_epochs"].sum())
            total_epochs = int(inj["epochs"].sum())
            print(f"Injected svUsed rate (aggregate): "
                  f"{total_used}/{total_epochs} = "
                  f"{total_used / max(total_epochs, 1):.3f}")

        # Real-sky PRN usage for contrast
        if not real.empty:
            real_used = real[real["svUsed_epochs"] > 0]
            print(f"\nReal-sky PRNs with svUsed>0: {len(real_used)}/{len(real)}")
            print(real_used[["epochs", "svUsed_epochs", "svUsed_rate",
                             "mean_cno", "mean_elev", "abs_prRes"]]
                  .sort_values("svUsed_epochs", ascending=False).head(12).to_string())

    # NAV-PVT fix quality
    if not npvt.empty:
        fix_counts = npvt["fixType"].value_counts().to_dict()
        fix_lbl = {0: "nofix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "time"}
        fix_str = ", ".join(f"{fix_lbl.get(k, k)}={v}" for k, v in fix_counts.items())
        print(f"\n-- NAV-PVT fix: {fix_str}")
        use = npvt[npvt["fixType"] >= 2]
        if not use.empty:
            print(f"   numSV median={use['numSV'].median():.0f}, "
                  f"hAcc median={use['hAcc'].median() / 1000:.1f} m, "
                  f"vAcc median={use['vAcc'].median() / 1000:.1f} m")
            print(f"   lat/lon first: {use['lat'].iloc[0] / 1e7:.7f}, "
                  f"{use['lon'].iloc[0] / 1e7:.7f}, "
                  f"h={use['height'].iloc[0] / 1000:.1f} m")

    # NAV-STATUS spoof/jam
    if not nstat.empty:
        sp = nstat["spoofDetState"].value_counts().to_dict()
        jm = nstat["jammingState"].value_counts().to_dict()
        sp_str = ", ".join(f"{SPOOF_LBL.get(k, k)}={v}" for k, v in sp.items())
        jm_str = ", ".join(f"{JAM_LBL.get(k, k)}={v}" for k, v in jm.items())
        print(f"\n-- NAV-STATUS spoof: {sp_str}")
        print(f"   NAV-STATUS jam:   {jm_str}")

    if not secsig.empty:
        sp = secsig["spoofingState"].value_counts().to_dict()
        jm = secsig["jammingState"].value_counts().to_dict()
        sp_str = ", ".join(f"state{k}={v}" for k, v in sp.items())
        jm_str = ", ".join(f"state{k}={v}" for k, v in jm.items())
        print(f"\n-- SEC-SIG spoof: {sp_str}")
        print(f"   SEC-SIG jam:   {jm_str}")

    # Joint svUsed check: epoch where both injected + real-sky used
    if not nsat.empty:
        nsat_gps = nsat[nsat["gnssId"] == 0]
        by_epoch = nsat_gps.groupby("iTOW")
        joint = 0
        inj_only = 0
        real_only = 0
        neither = 0
        for itow, g in by_epoch:
            used = g[g["svUsed"] == 1]
            if used.empty:
                neither += 1
                continue
            has_inj = used["svId"].isin(INJECTED).any()
            has_real = (~used["svId"].isin(INJECTED)).any()
            if has_inj and has_real:
                joint += 1
            elif has_inj:
                inj_only += 1
            elif has_real:
                real_only += 1
        total = joint + inj_only + real_only + neither
        print(f"\n-- Epoch svUsed composition ({total} NAV-SAT epochs) --")
        print(f"   both injected+real used: {joint} ({joint/max(total,1):.1%})")
        print(f"   real-only used:          {real_only}")
        print(f"   injected-only used:      {inj_only}")
        print(f"   none used:               {neither}")

        # When injected used, which PRNs
        inj_used = nsat_gps[(nsat_gps["svUsed"] == 1) &
                            (nsat_gps["svId"].isin(INJECTED))]
        if not inj_used.empty:
            print(f"\n-- When injected svUsed==1 --")
            print(inj_used.groupby("svId").agg(
                used_epochs=("iTOW", "nunique"),
                mean_cno=("cno", "mean"),
                mean_elev=("elev", "mean"),
                mean_prRes=("prRes", "mean"),
                abs_prRes=("prRes", lambda s: s.abs().mean()),
                max_abs_prRes=("prRes", lambda s: s.abs().max()),
            ).round(1).to_string())

            # First/last epoch of injected use
            for sv in sorted(INJECTED):
                rows = inj_used[inj_used["svId"] == sv]
                if rows.empty:
                    continue
                first = rows["iTOW"].min()
                last = rows["iTOW"].max()
                print(f"   PRN {sv:02d}: first used at iTOW={first}, "
                      f"last at iTOW={last}, "
                      f"span={((last - first) / 1000.0):.1f}s, "
                      f"n_used_epochs={rows['iTOW'].nunique()}")


if __name__ == "__main__":
    root = Path("dataset/22-4/ver3")
    for f in sorted(root.glob("COM*.ubx")):
        label = "COM3" if "COM3" in f.name else "COM4"
        analyze(f, label)
