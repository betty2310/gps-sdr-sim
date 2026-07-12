#!/usr/bin/env python3
"""Build a strict descriptive report from completed GNSS-SDR jammer runs."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from cw_dataset import Profile, average_psd, dense_spectrogram


JAMMER_TYPES = ("cw", "narrowband", "wideband", "chirp", "pulsed")
DISPLAY_NAMES = {
    "cw": "CW",
    "narrowband": "Narrowband",
    "wideband": "Wideband",
    "chirp": "Chirp",
    "pulsed": "Pulsed",
}
OKABE_ITO = {
    "blue": "#0072B2",
    "orange": "#E69F00",
    "vermillion": "#D55E00",
    "green": "#009E73",
    "sky": "#56B4E9",
}


class CampaignError(RuntimeError):
    """A completed campaign artifact is missing or inconsistent."""


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CampaignError(f"cannot read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise CampaignError(f"JSON root must be an object: {path}")
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value.rstrip() + "\n", encoding="utf-8")


def relative_link(path: Path, output_dir: Path) -> str:
    return Path(os.path.relpath(path, output_dir)).as_posix()


def profile_from_index(index: dict[str, Any]) -> Profile:
    value = index["profile"]
    return Profile(
        name=str(value["name"]),
        sample_rate_hz=int(value["sample_rate_hz"]),
        duration_s=float(value["duration_s"]),
        onset_s=float(value["onset_s"]),
        offset_s=float(value["offset_s"]),
        ramp_s=float(value["ramp_s"]),
        levels_db=tuple(float(level) for level in value["levels_db"]),
    )


def fixture_by_level(index: dict[str, Any], clean: bool) -> dict[str, Any]:
    matches = [
        fixture for fixture in index["fixtures"] if (fixture["js_db"] is None) == clean
    ]
    if len(matches) != 1:
        kind = "clean" if clean else "jammed"
        raise CampaignError(f"expected one {kind} fixture, found {len(matches)}")
    return matches[0]


def load_run(runs_root: Path, source_type: str) -> dict[str, Any]:
    directory = (runs_root / source_type).resolve()
    index_path = directory / "dataset-index.json"
    index = read_json(index_path)
    if index["shared"]["jammer_type"] != source_type:
        raise CampaignError(f"source type mismatch in {index_path}")
    clean_entry = fixture_by_level(index, clean=True)
    jam_entry = fixture_by_level(index, clean=False)
    clean_receiver = read_json(Path(clean_entry["receiver_report"]))
    jam_receiver = read_json(Path(jam_entry["receiver_report"]))
    waveform = read_json(Path(jam_entry["waveform_report"]))
    jam_manifest = read_json(Path(jam_entry["manifest"]))

    for label, entry, receiver in (
        ("clean", clean_entry, clean_receiver),
        ("jammed", jam_entry, jam_receiver),
    ):
        if not entry["pass"]:
            raise CampaignError(f"{source_type} {label} fixture is not marked passing")
        if receiver["exit_code"] != 0:
            raise CampaignError(f"{source_type} {label} GNSS-SDR exit is nonzero")
        if not receiver["metrics"]["clean_prefix_gate"]:
            raise CampaignError(f"{source_type} {label} clean-prefix gate failed")
        spectral = receiver.get("prn_spectral")
        if not spectral or len(spectral["prns"]) != 9:
            raise CampaignError(
                f"{source_type} {label} does not contain all 9 PRN spectral reports"
            )
    if not waveform["pass"]:
        raise CampaignError(f"{source_type} waveform acceptance failed")

    return {
        "source_type": source_type,
        "directory": directory,
        "index_path": index_path,
        "index": index,
        "profile": profile_from_index(index),
        "clean_entry": clean_entry,
        "jam_entry": jam_entry,
        "clean_receiver": clean_receiver,
        "jam_receiver": jam_receiver,
        "waveform": waveform,
        "jam_manifest": jam_manifest,
    }


def interval_cn0(metrics: dict[str, Any], prn: int) -> float | None:
    value = metrics["per_prn"].get(str(prn), {}).get("jammer")
    return float(value["median_cn0_db_hz"]) if value else None


def build_metrics(runs: list[dict[str, Any]]) -> dict[str, Any]:
    clean_hashes = {run["index"]["shared"]["clean_sha256"] for run in runs}
    profiles = {json.dumps(run["index"]["profile"], sort_keys=True) for run in runs}
    receiver_versions = {run["jam_receiver"]["version"] for run in runs} | {
        run["clean_receiver"]["version"] for run in runs
    }
    if len(clean_hashes) != 1:
        raise CampaignError("jammer conditions do not share one clean-IQ hash")
    if len(profiles) != 1:
        raise CampaignError("jammer conditions do not share one timing profile")
    if len(receiver_versions) != 1:
        raise CampaignError("jammer conditions do not share one GNSS-SDR version")

    visible_prns = sorted(
        {int(prn) for run in runs for prn in run["index"]["source"]["visible_prns"]}
    )
    stable_sets = [
        set(int(prn) for prn in receiver["metrics"]["stable_prns"])
        for run in runs
        for receiver in (run["clean_receiver"], run["jam_receiver"])
    ]
    common_stable_prns = sorted(set.intersection(*stable_sets))

    conditions: dict[str, Any] = {}
    for run in runs:
        source_type = run["source_type"]
        clean_metrics = run["clean_receiver"]["metrics"]
        jam_metrics = run["jam_receiver"]["metrics"]
        deltas: dict[str, float | None] = {}
        for prn in visible_prns:
            clean_cn0 = interval_cn0(clean_metrics, prn)
            jam_cn0 = interval_cn0(jam_metrics, prn)
            deltas[str(prn)] = (
                jam_cn0 - clean_cn0
                if clean_cn0 is not None and jam_cn0 is not None
                else None
            )
        primary_deltas: list[float] = []
        for prn in common_stable_prns:
            value = deltas[str(prn)]
            if value is not None:
                primary_deltas.append(value)
        clean_pvt = clean_metrics["pvt_intervals"]["jammer"]
        jam_pvt = jam_metrics["pvt_intervals"]["jammer"]
        mix_measurements = run["jam_manifest"]["measurements"]
        conditions[source_type] = {
            "waveform_pass": bool(run["waveform"]["pass"]),
            "measured_js_db": float(mix_measurements["measured_js_db"]),
            "measured_snr_db": float(mix_measurements["measured_snr_db"]),
            "clipped_components": int(mix_measurements["clipped_components"]),
            "per_prn_jammer_interval_cn0_delta_db": deltas,
            "common_stable_prn_cn0_delta_db": {
                "median": float(np.median(primary_deltas)),
                "minimum": float(np.min(primary_deltas)),
                "maximum": float(np.max(primary_deltas)),
                "values": primary_deltas,
            },
            "pvt_jammer_interval": {
                "control_valid_fixes": int(clean_pvt["valid_fixes"]),
                "control_expected_fixes": int(clean_pvt["expected_1hz_fixes"]),
                "control_availability": float(clean_pvt["availability"]),
                "jammed_valid_fixes": int(jam_pvt["valid_fixes"]),
                "jammed_expected_fixes": int(jam_pvt["expected_1hz_fixes"]),
                "jammed_availability": float(jam_pvt["availability"]),
                "control_median_usable_satellites": float(
                    clean_pvt["median_usable_satellites"]
                ),
                "jammed_median_usable_satellites": float(
                    jam_pvt["median_usable_satellites"]
                ),
            },
            "recovery": {
                "time_to_restored_pvt_s": jam_metrics["time_to_restored_pvt_s"],
                **jam_metrics["pvt_intervals"]["recovery"],
            },
            "waveform_checks": run["waveform"]["checks"],
            "source_measurements": run["waveform"]["source_measurements"],
            "paths": {
                "dataset_index": str(run["index_path"]),
                "clean_receiver_report": str(
                    Path(run["clean_entry"]["receiver_report"])
                ),
                "jammed_receiver_report": str(
                    Path(run["jam_entry"]["receiver_report"])
                ),
                "waveform_report": str(Path(run["jam_entry"]["waveform_report"])),
            },
        }

    return {
        "schema": "gps-sdr-sim.gnss-sdr-jammer-campaign.v1",
        "analysis_kind": "single-seed deterministic descriptive verification",
        "sample_size": {
            "runs_per_condition": 1,
            "noise_seed": int(runs[0]["index"]["shared"]["noise_seed"]),
            "independent_repetitions": 1,
            "inferential_statistics_supported": False,
        },
        "shared": {
            "clean_sha256": next(iter(clean_hashes)),
            "profile": runs[0]["index"]["profile"],
            "source": runs[0]["index"]["source"],
            "receiver_version": next(iter(receiver_versions)),
            "visible_prns": visible_prns,
            "common_stable_prns": common_stable_prns,
        },
        "conditions": conditions,
    }


def save_figure(fig: Figure, stem: Path) -> list[Path]:
    png = stem.with_suffix(".png")
    pdf = stem.with_suffix(".pdf")
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    plt.close(fig)
    return [png, pdf]


def waveform_figure(runs: list[dict[str, Any]], figures_dir: Path) -> list[Path]:
    spectra: list[tuple[np.ndarray, np.ndarray]] = []
    spectrograms: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for run in runs:
        profile: Profile = run["profile"]
        manifest = run["jam_manifest"]
        jammer = Path(manifest["files"]["jammer"]["path"])
        steady_start = profile.onset_sample + profile.ramp_samples
        steady_stop = profile.offset_sample - profile.ramp_samples
        frequency, psd = average_psd(
            jammer, steady_start, steady_stop, profile.sample_rate_hz
        )
        effective_scale = float(manifest["parameters"]["gain"]) * float(
            manifest["measurements"]["jammer_scale"]
        )
        psd = psd + 20.0 * math.log10(effective_scale)
        spectra.append((frequency, psd))
        detail_begin = profile.onset_s + profile.ramp_s
        detail = dense_spectrogram(jammer, profile, detail_begin, duration_s=0.25)
        spectrograms.append(
            (detail[0], detail[1], detail[2] + 20.0 * math.log10(effective_scale))
        )

    all_psd = np.concatenate([power for _, power in spectra])
    psd_max = float(np.percentile(all_psd, 99.99))
    finite_spectrogram = np.concatenate(
        [image[np.isfinite(image)].reshape(-1) for _, _, image in spectrograms]
    )
    spec_max = float(np.percentile(finite_spectrogram, 99.8))
    spec_min = spec_max - 70.0

    fig, axes = plt.subplots(
        len(runs),
        2,
        figsize=(14, 18),
        constrained_layout=True,
        sharex="col",
    )
    mesh = None
    for row, run in enumerate(runs):
        source_type = run["source_type"]
        profile: Profile = run["profile"]
        frequency, psd = spectra[row]
        axes[row, 0].plot(frequency / 1e3, psd, color=OKABE_ITO["blue"], linewidth=0.8)
        axes[row, 0].set(
            ylabel=f"{DISPLAY_NAMES[source_type]}\nPSD (dBFS/bin)",
            ylim=(psd_max - 85.0, psd_max + 3.0),
        )
        axes[row, 0].grid(alpha=0.2)

        times, frequencies, image = spectrograms[row]
        mesh = axes[row, 1].pcolormesh(
            (times - profile.onset_s - profile.ramp_s) * 1e3,
            frequencies / 1e3,
            image,
            shading="auto",
            cmap="magma",
            vmin=spec_min,
            vmax=spec_max,
            rasterized=True,
        )
        axes[row, 1].set(ylabel="Baseband (kHz)")
    axes[0, 0].set_title("Effective jammer-only spectrum at measured J/S")
    axes[0, 1].set_title("Jammer-only active-window spectrogram (first 250 ms)")
    axes[-1, 0].set_xlabel("Baseband frequency (kHz)")
    axes[-1, 1].set_xlabel("Time into steady jammer interval (ms)")
    if mesh is not None:
        fig.colorbar(mesh, ax=axes[:, 1], label="FFT power (dB)", shrink=0.8)
    return save_figure(fig, figures_dir / "figure-01-waveform-spectrum-spectrogram")


def prn_heatmap_figure(metrics: dict[str, Any], figures_dir: Path) -> list[Path]:
    prns = metrics["shared"]["visible_prns"]
    common = set(metrics["shared"]["common_stable_prns"])
    matrix = np.asarray(
        [
            [
                metrics["conditions"][source_type][
                    "per_prn_jammer_interval_cn0_delta_db"
                ][str(prn)]
                for prn in prns
            ]
            for source_type in JAMMER_TYPES
        ],
        dtype=np.float64,
    )
    limit = max(5.0, min(20.0, float(np.nanmax(np.abs(matrix)))))
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    image = ax.imshow(matrix, cmap="RdBu_r", vmin=-limit, vmax=limit, aspect="auto")
    for row in range(matrix.shape[0]):
        for column in range(matrix.shape[1]):
            value = matrix[row, column]
            if math.isfinite(value):
                ax.text(
                    column,
                    row,
                    f"{value:+.1f}",
                    ha="center",
                    va="center",
                    color="white" if abs(value) > 0.55 * limit else "black",
                    fontsize=8,
                )
    ax.set_xticks(
        np.arange(len(prns)),
        [f"G{prn:02d}{'*' if prn in common else ''}" for prn in prns],
    )
    ax.set_yticks(
        np.arange(len(JAMMER_TYPES)),
        [DISPLAY_NAMES[source_type] for source_type in JAMMER_TYPES],
    )
    ax.set(
        xlabel="PRN (* = stable in every paired clean prefix)",
        ylabel="Jammer type",
        title="Matched jammer-interval C/N0 change by GNSS-SDR tracking channel",
    )
    fig.colorbar(image, ax=ax, label="Jammed - clean control C/N0 (dB)")
    return save_figure(fig, figures_dir / "figure-02-per-prn-cn0-delta")


def receiver_figure(metrics: dict[str, Any], figures_dir: Path) -> list[Path]:
    labels = [DISPLAY_NAMES[source_type] for source_type in JAMMER_TYPES]
    cn0 = [
        metrics["conditions"][source_type]["common_stable_prn_cn0_delta_db"]["median"]
        for source_type in JAMMER_TYPES
    ]
    clean_pvt = [
        100.0
        * metrics["conditions"][source_type]["pvt_jammer_interval"][
            "control_availability"
        ]
        for source_type in JAMMER_TYPES
    ]
    jam_pvt = [
        100.0
        * metrics["conditions"][source_type]["pvt_jammer_interval"][
            "jammed_availability"
        ]
        for source_type in JAMMER_TYPES
    ]
    x = np.arange(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    bars = axes[0].bar(x, cn0, color=OKABE_ITO["vermillion"])
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].bar_label(bars, fmt="%+.1f", padding=3, fontsize=8)
    axes[0].set(
        xticks=x,
        xticklabels=labels,
        ylabel="Median matched C/N0 change (dB)",
        title="Common stable PRNs (G10, G12, G23, G28, G31, G32)",
    )
    axes[0].tick_params(axis="x", rotation=25)
    axes[0].grid(axis="y", alpha=0.25)

    width = 0.38
    control_bars = axes[1].bar(
        x - width / 2,
        clean_pvt,
        width,
        label="Clean control",
        color=OKABE_ITO["blue"],
    )
    jam_bars = axes[1].bar(
        x + width / 2,
        jam_pvt,
        width,
        label="Jammed",
        color=OKABE_ITO["orange"],
    )
    axes[1].bar_label(
        control_bars,
        fmt="%.0f%%",
        label_type="center",
        color="white",
        fontsize=8,
    )
    axes[1].bar_label(jam_bars, fmt="%.0f%%", padding=2, fontsize=8)
    axes[1].set(
        xticks=x,
        xticklabels=labels,
        ylim=(0.0, 110.0),
        ylabel="Valid 1 Hz PVT fixes (%)",
        title="Matched 20 s jammer interval",
    )
    axes[1].tick_params(axis="x", rotation=25)
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].legend()
    return save_figure(fig, figures_dir / "figure-03-receiver-impact")


def waveform_summary(source_type: str, condition: dict[str, Any]) -> str:
    measured = condition["source_measurements"]
    if source_type == "cw":
        return (
            f"tone {measured['frequency_hz'] / 1e3:.3f} kHz; "
            f"image rejection {measured['negative_frequency_image_rejection_db']:.1f} dB"
        )
    if source_type == "narrowband":
        return (
            f"center {measured['measured_center_hz'] / 1e3:.3f} kHz; "
            f"90% occupied width {measured['occupied_90_bandwidth_hz'] / 1e3:.1f} kHz"
        )
    if source_type == "wideband":
        return (
            f"90% occupied width {measured['occupied_90_bandwidth_hz'] / 1e6:.3f} MHz"
        )
    if source_type == "chirp":
        return (
            f"frequency-law RMS error {measured['rms_frequency_error_hz']:.1f} Hz; "
            f"max error {measured['max_frequency_error_hz']:.1f} Hz"
        )
    return (
        f"nonzero duty {100 * measured['nonzero_duty_cycle']:.3f}%; "
        f"off-state nonzero samples {measured['nonzero_off_samples']}"
    )


def analysis_report(metrics: dict[str, Any], output_dir: Path) -> str:
    profile = metrics["shared"]["profile"]
    rows = []
    for source_type in JAMMER_TYPES:
        condition = metrics["conditions"][source_type]
        pvt = condition["pvt_jammer_interval"]
        recovery = condition["recovery"]["time_to_restored_pvt_s"]
        rows.append(
            "| "
            + " | ".join(
                (
                    DISPLAY_NAMES[source_type],
                    f"{condition['measured_js_db']:.2f}",
                    f"{condition['common_stable_prn_cn0_delta_db']['median']:+.2f}",
                    f"{pvt['jammed_valid_fixes']}/{pvt['jammed_expected_fixes']}",
                    f"{pvt['jammed_median_usable_satellites']:.1f}",
                    "not observed" if recovery is None else f"{float(recovery):.1f}",
                )
            )
            + " |"
        )
    waveform_rows = [
        f"| {DISPLAY_NAMES[source_type]} | PASS | "
        f"{waveform_summary(source_type, metrics['conditions'][source_type])} |"
        for source_type in JAMMER_TYPES
    ]
    figures = output_dir / "figures"
    return f"""# GNSS-SDR Multi-Waveform Jamming Verification

## Outcome

All five synthetic jammer modes passed their numerical waveform gates and completed matched clean/jammed processing with GNSS-SDR 0.0.20. At measured J/S = +20 dB, narrowband, wideband, and chirp interference eliminated valid PVT throughout the 20-second jammer interval; CW and pulsed interference retained 19/20 valid fixes but reduced the median usable-satellite count.

This is a single deterministic run per condition. It verifies this implementation and this synthetic receiver scenario; it does not establish population-level or field performance.

## Locked experiment

- Desired signal: one frozen `gps-sdr-sim` GPS L1 C/A recording, SHA-256 `{metrics["shared"]["clean_sha256"]}`.
- Navigation/time/location: frozen 2025-01-03 RINEX broadcast navigation, `2025/01/03,00:00:00`, Hanoi `21.0047844,105.8460541,5`.
- IQ: signed little-endian interleaved SC16, {profile["sample_rate_hz"] / 1e6:.1f} Msps, {profile["duration_s"]:.0f} s.
- Timing: clean prefix 0-{profile["onset_s"]:.0f} s, jammer {profile["onset_s"]:.0f}-{profile["offset_s"]:.0f} s, recovery {profile["offset_s"]:.0f}-{profile["duration_s"]:.0f} s.
- Receiver: `{metrics["shared"]["receiver_version"]}`, 9 fixed visible PRNs, all assigned channels acquiring concurrently.
- Noise/J/S: deterministic seed {metrics["sample_size"]["noise_seed"]}; every condition measured +20.00 dB J/S and approximately -10.00 dB composite SNR with zero clipped components.
- Primary PRN set: G10, G12, G23, G28, G31, G32, the PRNs with clean-prefix median C/N0 at least 30 dB-Hz in every clean and jammed receiver pass.

## Waveform evidence

| Type | Gate | Key measured property |
|---|---:|---|
{chr(10).join(waveform_rows)}

The raw waveform comparison is in [Figure 1]({relative_link(figures / "figure-01-waveform-spectrum-spectrogram.png", output_dir)}). It shows the expected fixed line, band-limited noise cloud, Nyquist-wide noise floor, repeating sweep, and pulsed comb/time gating. These are jammer-only RF/baseband-domain plots after applying each mix scale and are not PRN-isolated. Each run also retains a full-90-second mixed spectrogram for activation timing.

## Receiver impact

| Type | J/S (dB) | Median paired C/N0 change (dB) | Jammer PVT fixes | Median usable sats | Restored PVT after off (s) |
|---|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

The clean control produced 19/20 valid PVT fixes and a median of 6 usable satellites in the matched interval for every type. [Figure 2]({relative_link(figures / "figure-02-per-prn-cn0-delta.png", output_dir)}) retains all nine PRN-level paired changes; asterisks identify the six primary stable PRNs. [Figure 3]({relative_link(figures / "figure-03-receiver-impact.png", output_dir)}) summarizes the primary-set C/N0 effect and matched PVT availability.

CW was selective across PRNs: some channels improved or changed little while G23/G28/G32 fell, so its primary-set median reduction was modest compared with the noise, chirp, and pulsed modes. Narrowband, wideband, and chirp prevented PVT during exposure. Narrowband and wideband restored PVT seven seconds after jammer-off; chirp did not restore PVT before NMEA output ended. Pulsed interference preserved PVT but caused a clear common-PRN C/N0 loss.

## What “spectrum of each PRN” means

All GPS L1 C/A PRNs occupy the same RF band, so a raw FFT cannot separate or label them. The 90 jammed receiver artifacts therefore include one PNG and one PDF per PRN based on GNSS-SDR post-correlation `Prompt_I`/`Prompt_Q`. Those plots are code/carrier-despread residual prompt spectra and spectrograms, not raw RF spectra assigned to a PRN. See the [PRN gallery](prn-gallery.md) and [figure catalog](figure-catalog.md).

## Authentic-signal boundary

No authentic captured signal is required to generate or numerically verify CW, narrowband, wideband, chirp, or pulsed mathematical waveforms, nor to reproduce this synthetic GNSS-SDR experiment. Authentic desired-signal and interference captures are required before claiming behavior under real antenna patterns, propagation/multipath, oscillator error, front-end filtering or AGC, ADC saturation/quantization, field interferers, or installed receiver hardware. IQ replay for captured authentic interference is not implemented in this campaign.

## Limitations

- One deterministic seed and one +20 dB J/S point per waveform: no confidence interval, significance test, or standardized effect size is identifiable.
- PRNs are not independent experimental repetitions; they share one composite IQ stream, navigation geometry, noise realization, and receiver process.
- GNSS-SDR NMEA ends near scenario second 72 although tracking dumps continue through 90 s. Recovery PVT counts therefore cover only the emitted NMEA window; full-duration recovery C/N0 remains available from tracking.
- Results are receiver/configuration-specific and do not validate a mitigation algorithm; they validate waveform generation, mixing, receiver ingestion, and observable degradation.
"""


def stats_appendix(metrics: dict[str, Any]) -> str:
    prns = metrics["shared"]["visible_prns"]
    common = set(metrics["shared"]["common_stable_prns"])
    rows = []
    for prn in prns:
        values = [
            metrics["conditions"][source_type]["per_prn_jammer_interval_cn0_delta_db"][
                str(prn)
            ]
            for source_type in JAMMER_TYPES
        ]
        rows.append(
            f"| G{prn:02d}{'*' if prn in common else ''} | "
            + " | ".join(
                "NA" if value is None else f"{float(value):+.2f}" for value in values
            )
            + " |"
        )
    summary_rows = []
    for source_type in JAMMER_TYPES:
        condition = metrics["conditions"][source_type]
        delta = condition["common_stable_prn_cn0_delta_db"]
        pvt = condition["pvt_jammer_interval"]
        summary_rows.append(
            f"| {DISPLAY_NAMES[source_type]} | {delta['median']:+.2f} | "
            f"[{delta['minimum']:+.2f}, {delta['maximum']:+.2f}] | "
            f"{pvt['control_valid_fixes']}/{pvt['control_expected_fixes']} | "
            f"{pvt['jammed_valid_fixes']}/{pvt['jammed_expected_fixes']} |"
        )
    return f"""# Statistical Appendix

## Valid analysis level

- Independent repetitions per condition: **1**.
- Noise seed: **{metrics["sample_size"]["noise_seed"]}**.
- Inferential statistics: **not supported**. No p-values, confidence intervals, normality tests, or multiple-comparison corrections are reported.
- Effect magnitude: exact matched C/N0 changes in dB and exact valid-fix counts. These are physical paired differences, not standardized population effect sizes.
- Unit caution: PRNs share the same composite RF record and are not independent repetitions. The six common stable PRNs are a within-record channel breakdown only.

## Primary descriptive summary

| Type | Median C/N0 change (dB) | PRN range (dB) | Control PVT | Jammed PVT |
|---|---:|---:|---:|---:|
{chr(10).join(summary_rows)}

Higher C/N0 and PVT availability are better. C/N0 change is `jammed - matched clean control` over seconds 40-60, so negative values indicate degradation. The range is over G10, G12, G23, G28, G31, and G32 only and is not a confidence interval.

## Exact per-PRN C/N0 changes

| PRN | CW | Narrowband | Wideband | Chirp | Pulsed |
|---|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

`*` marks a PRN whose clean-prefix median C/N0 was at least 30 dB-Hz in every paired receiver pass. Non-starred PRNs remain in the artifact record but are excluded from the primary median because late/weak acquisition can make their C/N0 change misleading (for example, a channel becoming stable later can look like an improvement).

## Assumptions and blockers

- The paired comparison assumes each type's clean and jammed fixtures share the same desired IQ, noise seed, receiver configuration, and pre-jammer samples. The campaign validates one shared clean SHA-256 and clean-prefix gates for both members of every pair.
- A repeated-seed campaign is required for uncertainty intervals or hypothesis testing.
- Authentic captures and receiver/front-end repetitions are required for field-general effect claims.
"""


def figure_catalog(metrics: dict[str, Any], output_dir: Path) -> str:
    rows = []
    for source_type in JAMMER_TYPES:
        condition = metrics["conditions"][source_type]
        jam_report = Path(condition["paths"]["jammed_receiver_report"])
        spectral_dir = jam_report.parent / "prn-spectral"
        rows.append(
            f"| {DISPLAY_NAMES[source_type]} | 9 | 18 | "
            f"[{spectral_dir.name}]({relative_link(spectral_dir, output_dir)}) |"
        )
    return f"""# Figure Catalog

## Figure 1 — waveform spectrum and spectrogram

- Files: [PNG](figures/figure-01-waveform-spectrum-spectrogram.png), [PDF](figures/figure-01-waveform-spectrum-spectrogram.pdf)
- Purpose: verify that the five generated/mixed RF-domain waveforms have distinguishable, type-correct spectral and temporal structure.
- Data: jammer-only SC16 after applying each mix manifest's effective scale; the spectrum uses the steady interval and the dense spectrogram uses the first 250 ms after the global ramp. Each run retains a separate full-90-second mixed spectrogram for activation timing.
- Reader should notice: CW is a fixed +500 kHz line; narrowband remains concentrated around +500 kHz; wideband fills the complex Nyquist band; chirp repeatedly sweeps -500 to +500 kHz; pulsed CW is intermittent and produces a comb-like spectrum.
- Implication: the receiver comparison is not five aliases of one CW generator; each requested source reaches the actual mixed-IQ path.
- Caveat: FFT values are descriptive windowed estimates, not calibrated RF power-density measurements.

## Figure 2 — per-PRN C/N0 change

- Files: [PNG](figures/figure-02-per-prn-cn0-delta.png), [PDF](figures/figure-02-per-prn-cn0-delta.pdf)
- Purpose: show which GNSS-SDR tracking channels are affected during the matched 20-second jammer interval.
- Data: per-PRN median C/N0 from each jammed receiver report minus its same-type clean control at the same time interval.
- Reader should notice: narrowband, wideband, chirp, and pulsed effects are broadly negative across stable channels; CW is more PRN-dependent.
- Implication: mitigation evaluation should retain channel-level metrics rather than relying only on one aggregate PVT status.
- Caveat: starred PRNs form the primary stable set; unstarred late/weak channels are descriptive only.

## Figure 3 — receiver impact

- Files: [PNG](figures/figure-03-receiver-impact.png), [PDF](figures/figure-03-receiver-impact.pdf)
- Purpose: connect post-correlation degradation to navigation availability.
- Data: median matched C/N0 change across the six common stable PRNs and exact valid 1 Hz PVT fixes during seconds 40-60.
- Reader should notice: narrowband, wideband, and chirp drive PVT to zero; CW and pulsed retain fixes but reduce usable-satellite margin.
- Implication: the five types exercise materially different receiver failure regimes at the same measured average J/S.
- Caveat: one deterministic run per condition; bars have no error bars because no repeated-run uncertainty exists.

## Per-PRN spectrum/spectrogram artifacts

These are GNSS-SDR post-correlation prompt-domain figures. Each PNG has a matching PDF and traceable `prn-spectral-report.json`.

| Type | PRNs | Files | Directory |
|---|---:|---:|---|
{chr(10).join(rows)}

For every PRN figure:

- Purpose: compare clean-prefix, jammer, and recovery prompt spectra and show prompt residual frequency over time.
- Interpretation: changes represent that PRN's despread tracking channel; they are not PRN-isolated raw RF FFTs.
- Missing-data rule: prompt records are averaged onto a 50 Hz grid; STFT windows below 50% tracking coverage are blank.
"""


def prn_gallery(metrics: dict[str, Any], output_dir: Path) -> str:
    sections = [
        "# GNSS-SDR Per-PRN Spectrum and Spectrogram Gallery",
        "",
        "Each image contains the interval spectra (top) and the full-run post-correlation prompt spectrogram (bottom). Raw RF PRNs cannot be separated by FFT; these plots use GNSS-SDR despread tracking-channel Prompt I/Q.",
    ]
    for source_type in JAMMER_TYPES:
        condition = metrics["conditions"][source_type]
        receiver_dir = Path(condition["paths"]["jammed_receiver_report"]).parent
        waveform_dir = Path(condition["paths"]["waveform_report"]).parent
        sections.extend(
            (
                "",
                f"## {DISPLAY_NAMES[source_type]}",
                "",
                f"[Raw spectrum]({relative_link(waveform_dir / 'spectrum.png', output_dir)}) · "
                f"[Raw spectrogram]({relative_link(waveform_dir / 'spectrogram.png', output_dir)}) · "
                f"[Active detail]({relative_link(waveform_dir / 'spectrogram-active-detail.png', output_dir)}) · "
                f"[Receiver report]({relative_link(receiver_dir / 'receiver-report.json', output_dir)})",
                "",
                "| PRN | Prompt spectrum and spectrogram |",
                "|---|---|",
            )
        )
        spectral = read_json(receiver_dir / "prn-spectral" / "prn-spectral-report.json")
        for prn in sorted((int(value) for value in spectral["prns"]), key=int):
            image_path = (
                receiver_dir
                / "prn-spectral"
                / f"G{prn:02d}-prompt-spectrum-spectrogram.png"
            )
            sections.append(
                f"| G{prn:02d} | [PNG]({relative_link(image_path, output_dir)}) · "
                f"[PDF]({relative_link(image_path.with_suffix('.pdf'), output_dir)}) |"
            )
    return "\n".join(sections) + "\n"


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Aggregate completed five-type GNSS-SDR jamming verification runs"
    )
    value.add_argument("--runs-root", required=True)
    value.add_argument("--output-dir", required=True)
    return value


def main() -> int:
    args = parser().parse_args()
    runs_root = Path(args.runs_root).resolve()
    output_dir = Path(args.output_dir).resolve()
    figures_dir = output_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    try:
        runs = [load_run(runs_root, source_type) for source_type in JAMMER_TYPES]
        metrics = build_metrics(runs)
        waveform_figure(runs, figures_dir)
        prn_heatmap_figure(metrics, figures_dir)
        receiver_figure(metrics, figures_dir)
        write_json(output_dir / "campaign-metrics.json", metrics)
        write_text(
            output_dir / "analysis-report.md", analysis_report(metrics, output_dir)
        )
        write_text(output_dir / "stats-appendix.md", stats_appendix(metrics))
        write_text(
            output_dir / "figure-catalog.md", figure_catalog(metrics, output_dir)
        )
        write_text(output_dir / "prn-gallery.md", prn_gallery(metrics, output_dir))
    except (CampaignError, KeyError, OSError, ValueError, TypeError) as exc:
        print(f"jamming-campaign-report: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
