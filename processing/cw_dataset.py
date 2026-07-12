#!/usr/bin/env python3
"""Reproducible offline synthetic-jamming dataset workflow for GPS L1 C/A."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, cast

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import h5py
import numpy as np
from scipy import signal
from scipy.io import loadmat


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECEIVER_CONFIG = ROOT / "sdr-monitor" / "cw-dataset-receiver.conf"
INT16_FULL_SCALE = 32767.0
CHUNK_COMPLEX = 1_048_576
# SciPy accepts False to preserve DC, while its current type stub narrows this to str.
NO_DETREND = cast(str, False)
JAMMER_TYPES = ("cw", "narrowband", "wideband", "chirp", "pulsed")


@dataclass(frozen=True)
class Profile:
    name: str
    sample_rate_hz: int
    duration_s: float
    onset_s: float
    offset_s: float
    ramp_s: float
    levels_db: tuple[float, ...]

    @property
    def samples(self) -> int:
        return exact_samples(self.duration_s, self.sample_rate_hz, "duration")

    @property
    def onset_sample(self) -> int:
        return exact_samples(self.onset_s, self.sample_rate_hz, "onset")

    @property
    def offset_sample(self) -> int:
        return exact_samples(self.offset_s, self.sample_rate_hz, "offset")

    @property
    def ramp_samples(self) -> int:
        return exact_samples(self.ramp_s, self.sample_rate_hz, "ramp")


PROFILES = {
    "fast": Profile("fast", 2_600_000, 3.0, 1.0, 2.0, 0.01, (0.0,)),
    "verification": Profile("verification", 2_600_000, 90.0, 40.0, 60.0, 0.01, (20.0,)),
    "canonical": Profile(
        "canonical",
        2_600_000,
        90.0,
        30.0,
        60.0,
        0.01,
        (-20.0, -10.0, 0.0, 10.0, 20.0, 30.0),
    ),
}


@dataclass(frozen=True)
class JammerParameters:
    source_type: str
    frequency_hz: float
    end_frequency_hz: float
    bandwidth_hz: float
    phase_rad: float
    amplitude: float
    seed: int
    chirp_period_s: float
    pulse_period_s: float
    pulse_duty_cycle: float
    pulse_ramp_s: float


def resolve_jammer_parameters(args: argparse.Namespace) -> JammerParameters:
    source_type = str(args.jammer_type)
    if args.frequency_hz is None:
        if source_type == "chirp":
            frequency_hz = -500_000.0
        elif source_type == "wideband":
            frequency_hz = 0.0
        else:
            frequency_hz = 500_000.0
    else:
        frequency_hz = float(args.frequency_hz)
    end_frequency_hz = (
        float(args.end_frequency_hz)
        if args.end_frequency_hz is not None
        else (500_000.0 if source_type == "chirp" else frequency_hz)
    )
    amplitude = (
        float(args.jammer_amplitude)
        if args.jammer_amplitude is not None
        else (0.15 if source_type in {"narrowband", "wideband"} else 0.5)
    )
    return JammerParameters(
        source_type=source_type,
        frequency_hz=frequency_hz,
        end_frequency_hz=end_frequency_hz,
        bandwidth_hz=float(args.bandwidth_hz),
        phase_rad=float(args.phase_rad),
        amplitude=amplitude,
        seed=int(args.jammer_seed),
        chirp_period_s=float(args.chirp_period_s),
        pulse_period_s=float(args.pulse_period_s),
        pulse_duty_cycle=float(args.pulse_duty_cycle),
        pulse_ramp_s=float(args.pulse_ramp_s),
    )


def default_receiver_binary() -> str:
    configured = os.environ.get("GNSS_SDR_BIN")
    if configured:
        return configured
    sibling_build = ROOT.parent / "gnss-sdr" / "install" / "gnss-sdr"
    if sibling_build.is_file() and os.access(sibling_build, os.X_OK):
        return str(sibling_build)
    return "gnss-sdr"


class WorkflowError(RuntimeError):
    """Expected user-facing workflow failure."""


def exact_samples(seconds: float, sample_rate: int, name: str) -> int:
    exact = seconds * sample_rate
    rounded = round(exact)
    if seconds < 0 or abs(exact - rounded) > 1e-6:
        raise WorkflowError(
            f"{name}={seconds} is not sample-aligned at {sample_rate} Hz"
        )
    return rounded


def run(
    command: list[str], *, cwd: Path = ROOT, capture: bool = False
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            cwd=cwd,
            check=True,
            text=True,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.STDOUT if capture else None,
        )
    except FileNotFoundError as exc:
        raise WorkflowError(f"required executable not found: {command[0]}") from exc
    except subprocess.CalledProcessError as exc:
        detail = f"\n{exc.stdout}" if exc.stdout else ""
        raise WorkflowError(
            f"command failed ({exc.returncode}): {' '.join(command)}{detail}"
        ) from exc


def read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise WorkflowError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise WorkflowError(f"JSON root must be an object: {path}")
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def git_commit() -> str:
    result = run(["git", "rev-parse", "HEAD"], capture=True)
    return (result.stdout or "").strip()


def executable_version(command: str) -> str:
    try:
        result = subprocess.run(
            [command, "--version"], check=False, text=True, capture_output=True
        )
    except FileNotFoundError:
        return "not-found"
    return (result.stdout or result.stderr).strip().splitlines()[0]


def copy_exact_samples(source: Path, destination: Path, samples: int) -> None:
    required = samples * 4
    if source.stat().st_size < required:
        raise WorkflowError(
            f"clean input is short: {source.stat().st_size} bytes, need {required}"
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    remaining = required
    with source.open("rb") as src, destination.open("wb") as dst:
        while remaining:
            block = src.read(min(8 * 1024 * 1024, remaining))
            if not block:
                raise WorkflowError(f"clean input ended early: {source}")
            dst.write(block)
            remaining -= len(block)


def generate_clean(
    args: argparse.Namespace, profile: Profile, output: Path
) -> dict[str, Any]:
    if args.clean_input:
        source = Path(args.clean_input).resolve()
        copy_exact_samples(source, output, profile.samples)
        return {
            "mode": "provided_fixture",
            "source_path": str(source),
            "source_sha256": sha256_file(source),
            "retained_first_sample_offset": 0,
        }

    if not args.rinex or not args.start_time:
        raise WorkflowError(
            "--rinex and --start-time are required without --clean-input"
        )
    rinex = Path(args.rinex).resolve()
    if not rinex.is_file():
        raise WorkflowError(f"RINEX navigation file does not exist: {rinex}")
    raw = output.with_name("clean-overgenerated.bin")
    generated_duration = profile.duration_s + 1.0
    command = [
        str(ROOT / "gps-sdr-sim"),
        "-e",
        str(rinex),
        "-l",
        args.location,
        "-t",
        args.start_time,
        "-d",
        f"{generated_duration:.3f}",
        "-s",
        str(profile.sample_rate_hz),
        "-b",
        "16",
        "-o",
        raw.name,
    ]
    generation = run(command, cwd=raw.parent, capture=True)
    (output.parent / "gps-sdr-sim.log").write_text(
        generation.stdout or "", encoding="utf-8"
    )
    copy_exact_samples(raw, output, profile.samples)
    raw_size = raw.stat().st_size
    raw.unlink()
    visible_geometry = [
        (int(match.group(1)), float(match.group(2)))
        for match in re.finditer(
            r"^\s*(\d{2})\s+[-+0-9.]+\s+([-+0-9.]+)",
            generation.stdout or "",
            re.MULTILINE,
        )
        if 1 <= int(match.group(1)) <= 32
    ]
    visible_prns = [
        prn for prn, _ in sorted(visible_geometry, key=lambda item: -item[1])
    ]
    return {
        "mode": "gps-sdr-sim",
        "rinex_path": str(rinex),
        "rinex_sha256": sha256_file(rinex),
        "scenario_start_time": args.start_time,
        "location_llh": args.location,
        "generator_requested_duration_s": generated_duration,
        "generator_output_bytes": raw_size,
        "retained_first_sample_offset": 0,
        "retained_samples": profile.samples,
        "visible_prns": visible_prns,
        "command": command,
    }


def generate_jammer(
    args: argparse.Namespace, profile: Profile, directory: Path
) -> tuple[Path, Path]:
    parameters = resolve_jammer_parameters(args)
    iq = directory / f"{parameters.source_type}-jammer.bin"
    manifest = directory / f"{parameters.source_type}-jammer.tool.json"
    run(
        [
            str(ROOT / "jammergen"),
            "--type",
            parameters.source_type,
            "--output",
            str(iq),
            "--manifest",
            str(manifest),
            "--sample-rate",
            str(profile.sample_rate_hz),
            "--duration",
            str(profile.duration_s),
            "--frequency",
            str(parameters.frequency_hz),
            "--end-frequency",
            str(parameters.end_frequency_hz),
            "--bandwidth",
            str(parameters.bandwidth_hz),
            "--phase",
            str(parameters.phase_rad),
            "--amplitude",
            str(parameters.amplitude),
            "--seed",
            str(parameters.seed),
            "--chirp-period",
            str(parameters.chirp_period_s),
            "--pulse-period",
            str(parameters.pulse_period_s),
            "--pulse-duty",
            str(parameters.pulse_duty_cycle),
            "--pulse-ramp",
            str(parameters.pulse_ramp_s),
            "--onset",
            str(profile.onset_s),
            "--offset",
            str(profile.offset_s),
            "--ramp",
            str(profile.ramp_s),
        ]
    )
    return iq, manifest


def iqmix_command(
    *,
    clean: Path,
    jammer: Path,
    output: Path | None,
    manifest: Path,
    profile: Profile,
    seed: int,
    gain: float,
    js_db: float | None,
    snr_db: float | None = None,
    noise_rms: float | None = None,
    dry_run: bool = False,
) -> list[str]:
    command = [
        str(ROOT / "iqmix"),
        "--clean",
        str(clean),
        "--jammer",
        str(jammer),
        "--manifest",
        str(manifest),
        "--samples",
        str(profile.samples),
        "--onset-sample",
        str(profile.onset_sample),
        "--offset-sample",
        str(profile.offset_sample),
        "--ramp-samples",
        str(profile.ramp_samples),
        "--seed",
        str(seed),
        "--gain",
        f"{gain:.17g}",
    ]
    if output is not None:
        command.extend(["--output", str(output)])
    if js_db is None:
        command.append("--disable-jammer")
    else:
        command.extend(["--js-db", f"{js_db:.17g}"])
    if noise_rms is not None:
        command.extend(["--noise-rms", f"{noise_rms:.17g}"])
    elif snr_db is not None:
        command.extend(["--snr-db", f"{snr_db:.17g}"])
    if dry_run:
        command.append("--dry-run")
    return command


def derive_common_gain(
    clean: Path,
    jammer: Path,
    profile: Profile,
    seed: int,
    highest_js_db: float,
    snr_db: float | None,
    noise_rms: float | None,
    directory: Path,
) -> tuple[float, dict[str, Any]]:
    manifest = directory / "common-gain-dry-run.json"
    run(
        iqmix_command(
            clean=clean,
            jammer=jammer,
            output=None,
            manifest=manifest,
            profile=profile,
            seed=seed,
            gain=1.0,
            js_db=highest_js_db,
            snr_db=snr_db,
            noise_rms=noise_rms,
            dry_run=True,
        )
    )
    metrics = read_json(manifest)
    peak = float(metrics["measurements"]["pre_gain_peak_component_counts"])
    if not math.isfinite(peak) or peak <= 0:
        raise WorkflowError("iqmix returned an invalid dataset peak")
    gain = INT16_FULL_SCALE * 10 ** (-1.0 / 20.0) / peak
    if gain <= 0 or not math.isfinite(gain):
        raise WorkflowError("cannot derive the common anti-clipping gain")
    return min(gain, 1.0), metrics


def iter_complex(
    path: Path, start: int, stop: int, chunk: int = CHUNK_COMPLEX
) -> Iterable[tuple[int, np.ndarray]]:
    with path.open("rb") as stream:
        stream.seek(start * 4)
        position = start
        while position < stop:
            count = min(chunk, stop - position)
            raw = np.fromfile(stream, dtype="<i2", count=count * 2)
            if raw.size != count * 2:
                raise WorkflowError(
                    f"truncated IQ while reading {path} at sample {position}"
                )
            pair = raw.reshape(-1, 2).astype(np.float64)
            yield position, pair[:, 0] + 1j * pair[:, 1]
            position += count


def estimate_tone(path: Path, profile: Profile) -> dict[str, float]:
    start = profile.onset_sample + profile.ramp_samples
    available = profile.offset_sample - profile.ramp_samples - start
    count = min(2_000_000, available)
    if count < 2:
        raise WorkflowError("CW plateau is too short for frequency estimation")
    with path.open("rb") as stream:
        stream.seek(start * 4)
        raw = np.fromfile(stream, dtype="<i2", count=count * 2)
    if raw.size != count * 2:
        raise WorkflowError(f"truncated jammer IQ: {path}")
    pair = raw.reshape(-1, 2).astype(np.float64)
    z = pair[:, 0] + 1j * pair[:, 1]
    lag = np.vdot(z[:-1], z[1:])
    frequency = float(np.angle(lag) * profile.sample_rate_hz / (2 * np.pi))
    rms = float(np.sqrt(np.mean(np.abs(z) ** 2)) / INT16_FULL_SCALE)
    n = np.arange(z.size, dtype=np.float64)
    omega = 2 * np.pi * frequency / profile.sample_rate_hz
    positive = abs(np.sum(z * np.exp(-1j * omega * n)))
    negative = abs(np.sum(z * np.exp(1j * omega * n)))
    image_rejection = float(
        20 * np.log10(max(positive, 1e-300) / max(negative, 1e-300))
    )
    return {
        "frequency_hz": frequency,
        "active_rms_full_scale": rms,
        "negative_frequency_image_rejection_db": image_rejection,
    }


def measure_plateau_rms(path: Path, profile: Profile) -> float:
    start = profile.onset_sample + profile.ramp_samples
    stop = profile.offset_sample - profile.ramp_samples
    power = 0.0
    samples = 0
    for _, values in iter_complex(path, start, stop):
        normalized = values / INT16_FULL_SCALE
        power += float(np.sum(np.abs(normalized) ** 2))
        samples += values.size
    if samples == 0:
        raise WorkflowError("jammer plateau is empty")
    return math.sqrt(power / samples)


def estimate_noise_spectrum(
    path: Path, profile: Profile, center_hz: float, bandwidth_hz: float
) -> dict[str, float]:
    start = profile.onset_sample + profile.ramp_samples
    stop = profile.offset_sample - profile.ramp_samples
    frequency, psd_db = average_psd(
        path, start, stop, profile.sample_rate_hz, nfft=8192
    )
    weights = 10 ** ((psd_db - float(np.max(psd_db))) / 10.0)
    total = float(np.sum(weights))
    if not math.isfinite(total) or total <= 0.0:
        raise WorkflowError("jammer PSD has no finite power")
    delta = (
        frequency - center_hz + profile.sample_rate_hz / 2
    ) % profile.sample_rate_hz - profile.sample_rate_hz / 2
    order = np.argsort(np.abs(delta))
    cumulative = np.cumsum(weights[order])
    index = min(int(np.searchsorted(cumulative, 0.9 * total)), order.size - 1)
    occupied_90 = 2.0 * abs(float(delta[order[index]]))
    in_requested = float(np.sum(weights[np.abs(delta) <= bandwidth_hz / 2.0]) / total)
    circular_moment = np.sum(
        weights * np.exp(1j * 2.0 * np.pi * frequency / profile.sample_rate_hz)
    )
    measured_center = float(
        np.angle(circular_moment) * profile.sample_rate_hz / (2.0 * np.pi)
    )
    center_error = float(
        (measured_center - center_hz + profile.sample_rate_hz / 2)
        % profile.sample_rate_hz
        - profile.sample_rate_hz / 2
    )
    return {
        "measured_center_hz": measured_center,
        "center_error_hz": center_error,
        "occupied_90_bandwidth_hz": occupied_90,
        "power_fraction_in_requested_band": in_requested,
    }


def estimate_chirp(
    path: Path, profile: Profile, parameters: dict[str, Any]
) -> dict[str, float]:
    sweep = int(parameters["sweep_samples"])
    start_frequency = float(parameters["frequency_hz"])
    end_frequency = float(parameters["end_frequency_hz"])
    steady_start = profile.onset_sample + profile.ramp_samples
    steady_stop = profile.offset_sample - profile.ramp_samples
    relative = (steady_start - profile.onset_sample) % sweep
    aligned = steady_start + ((sweep - relative) % sweep)
    if aligned + sweep + 1 <= steady_stop:
        start = aligned
        relative = 0
        count = sweep
    else:
        start = steady_start
        relative = (start - profile.onset_sample) % sweep
        count = min(steady_stop - start - 1, sweep - relative)
    if count < 32:
        raise WorkflowError("chirp plateau is too short for frequency-law analysis")
    with path.open("rb") as stream:
        stream.seek(start * 4)
        raw = np.fromfile(stream, dtype="<i2", count=(count + 1) * 2)
    if raw.size != (count + 1) * 2:
        raise WorkflowError(f"truncated chirp IQ: {path}")
    pair = raw.reshape(-1, 2).astype(np.float64)
    values = pair[:, 0] + 1j * pair[:, 1]
    instantaneous = (
        np.angle(np.conj(values[:-1]) * values[1:])
        * profile.sample_rate_hz
        / (2.0 * np.pi)
    )
    positions = relative + np.arange(count, dtype=np.float64)
    expected = start_frequency + (end_frequency - start_frequency) * positions / (
        sweep - 1
    )
    error = instantaneous - expected
    edge = max(8, min(count // 20, 4096))
    return {
        "measured_start_hz": float(np.mean(instantaneous[:edge])),
        "expected_start_hz": float(np.mean(expected[:edge])),
        "measured_end_hz": float(np.mean(instantaneous[-edge:])),
        "expected_end_hz": float(np.mean(expected[-edge:])),
        "rms_frequency_error_hz": float(np.sqrt(np.mean(error**2))),
        "max_frequency_error_hz": float(np.max(np.abs(error))),
        "analyzed_samples": float(count),
    }


def pulse_envelope_values(parameters: dict[str, Any]) -> np.ndarray:
    period = int(parameters["pulse_period_samples"])
    on_samples = int(parameters["pulse_on_samples"])
    ramp = int(parameters["pulse_ramp_samples"])
    position = np.arange(period, dtype=np.int64)
    envelope = np.zeros(period, dtype=np.float64)
    envelope[position < on_samples] = 1.0
    if ramp > 0:
        rising = position < ramp
        envelope[rising] = 0.5 - 0.5 * np.cos(np.pi * position[rising] / ramp)
        falling = (position >= on_samples - ramp) & (position < on_samples)
        envelope[falling] = 0.5 - 0.5 * np.cos(
            np.pi * (on_samples - position[falling]) / ramp
        )
    return envelope


def periodic_power_sum(envelope: np.ndarray, start: int, length: int) -> float:
    period = envelope.size
    start %= period
    first = min(length, period - start)
    total = float(np.sum(envelope[start : start + first] ** 2))
    remaining = length - first
    cycles, tail = divmod(remaining, period)
    total += cycles * float(np.sum(envelope**2))
    total += float(np.sum(envelope[:tail] ** 2))
    return total


def expected_pulsed_rms(parameters: dict[str, Any], profile: Profile) -> float:
    envelope = pulse_envelope_values(parameters)
    plateau_start = profile.onset_sample + profile.ramp_samples
    plateau_length = profile.offset_sample - profile.ramp_samples - plateau_start
    relative_start = plateau_start - profile.onset_sample
    mean_power = periodic_power_sum(envelope, relative_start, plateau_length) / (
        plateau_length
    )
    return float(parameters["amplitude_full_scale"]) * math.sqrt(mean_power)


def analyze_pulse(
    path: Path, profile: Profile, parameters: dict[str, Any]
) -> dict[str, float | int]:
    period = int(parameters["pulse_period_samples"])
    steady_start = profile.onset_sample + profile.ramp_samples
    steady_stop = profile.offset_sample - profile.ramp_samples
    relative = (steady_start - profile.onset_sample) % period
    start = steady_start + ((period - relative) % period)
    if start + period > steady_stop:
        raise WorkflowError("pulsed plateau is too short for one complete period")
    with path.open("rb") as stream:
        stream.seek(start * 4)
        raw = np.fromfile(stream, dtype="<i2", count=period * 2)
    if raw.size != period * 2:
        raise WorkflowError(f"truncated pulsed IQ: {path}")
    pair = raw.reshape(-1, 2).astype(np.float64)
    observed = np.hypot(pair[:, 0], pair[:, 1]) / INT16_FULL_SCALE
    envelope = pulse_envelope_values(parameters)
    expected = float(parameters["amplitude_full_scale"]) * envelope
    off = envelope == 0.0
    return {
        "nominal_duty_cycle": int(parameters["pulse_on_samples"]) / period,
        "nonzero_duty_cycle": float(np.count_nonzero(observed)) / period,
        "envelope_max_error_full_scale": float(np.max(np.abs(observed - expected))),
        "nonzero_off_samples": int(np.count_nonzero(observed[off])),
    }


def jammer_outside_nonzero(path: Path, profile: Profile) -> int:
    nonzero = 0
    for begin, end in (
        (0, profile.onset_sample),
        (profile.offset_sample, profile.samples),
    ):
        for _, values in iter_complex(path, begin, end):
            nonzero += int(np.count_nonzero(values))
    return nonzero


def average_psd(
    path: Path, start: int, stop: int, sample_rate: int, nfft: int = 8192
) -> tuple[np.ndarray, np.ndarray]:
    windows = 64
    if stop - start < nfft:
        raise WorkflowError(f"interval in {path} is too short for PSD")
    offsets = np.linspace(start, stop - nfft, windows, dtype=np.int64)
    power = np.zeros(nfft, dtype=np.float64)
    window = np.hanning(nfft)
    with path.open("rb") as stream:
        for offset in offsets:
            stream.seek(int(offset) * 4)
            raw = np.fromfile(stream, dtype="<i2", count=nfft * 2)
            pair = raw.reshape(-1, 2).astype(np.float64)
            z = (pair[:, 0] + 1j * pair[:, 1]) / INT16_FULL_SCALE
            power += np.abs(np.fft.fftshift(np.fft.fft(z * window))) ** 2
    power /= windows * np.sum(window**2)
    frequency = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / sample_rate))
    return frequency, 10 * np.log10(np.maximum(power, 1e-20))


def sampled_spectrogram(
    path: Path, profile: Profile, nfft: int = 4096, columns: int = 360
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    offsets = np.linspace(0, profile.samples - nfft, columns, dtype=np.int64)
    image = np.empty((nfft, columns), dtype=np.float32)
    window = np.hanning(nfft)
    with path.open("rb") as stream:
        for column, offset in enumerate(offsets):
            stream.seek(int(offset) * 4)
            raw = np.fromfile(stream, dtype="<i2", count=nfft * 2)
            pair = raw.reshape(-1, 2).astype(np.float64)
            z = (pair[:, 0] + 1j * pair[:, 1]) / INT16_FULL_SCALE
            spectrum = np.abs(np.fft.fftshift(np.fft.fft(z * window))) ** 2
            image[:, column] = 10 * np.log10(np.maximum(spectrum, 1e-20))
    frequency = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / profile.sample_rate_hz))
    times = offsets / profile.sample_rate_hz
    return times, frequency, image


def dense_spectrogram(
    path: Path,
    profile: Profile,
    begin_s: float,
    duration_s: float = 0.25,
    nfft: int = 4096,
    hop: int = 1024,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    start = exact_samples(begin_s, profile.sample_rate_hz, "spectrogram begin")
    count = min(
        exact_samples(duration_s, profile.sample_rate_hz, "spectrogram duration"),
        profile.samples - start,
    )
    if count < nfft:
        raise WorkflowError(f"interval in {path} is too short for dense spectrogram")
    with path.open("rb") as stream:
        stream.seek(start * 4)
        raw = np.fromfile(stream, dtype="<i2", count=count * 2)
    if raw.size != count * 2:
        raise WorkflowError(f"truncated IQ while reading dense spectrogram from {path}")
    pair = raw.reshape(-1, 2).astype(np.float64)
    values = (pair[:, 0] + 1j * pair[:, 1]) / INT16_FULL_SCALE
    frequencies, times, power = signal.spectrogram(
        values,
        fs=profile.sample_rate_hz,
        window="hann",
        nperseg=nfft,
        noverlap=nfft - hop,
        detrend=NO_DETREND,
        return_onesided=False,
        scaling="density",
        mode="psd",
    )
    return (
        times + begin_s,
        np.fft.fftshift(frequencies),
        10.0 * np.log10(np.maximum(np.fft.fftshift(power, axes=0), 1e-20)),
    )


def block_power(
    path: Path, profile: Profile, block_s: float = 0.1
) -> tuple[np.ndarray, np.ndarray]:
    block = max(1, exact_samples(block_s, profile.sample_rate_hz, "power block"))
    times: list[float] = []
    powers: list[float] = []
    for start in range(0, profile.samples, block):
        stop = min(start + block, profile.samples)
        total = 0.0
        count = 0
        for _, values in iter_complex(path, start, stop, chunk=block):
            normalized = values / INT16_FULL_SCALE
            total += float(np.sum(np.abs(normalized) ** 2))
            count += values.size
        times.append((start + stop) / (2 * profile.sample_rate_hz))
        powers.append(10 * math.log10(max(total / count, 1e-20)))
    return np.asarray(times), np.asarray(powers)


def plot_waveforms(
    jammer: Path, clean: Path, mixed: Path, profile: Profile, directory: Path
) -> list[str]:
    directory.mkdir(parents=True, exist_ok=True)
    steady_start = profile.onset_sample + profile.ramp_samples
    steady_stop = profile.offset_sample - profile.ramp_samples
    freq, jammer_psd = average_psd(
        jammer, steady_start, steady_stop, profile.sample_rate_hz
    )
    _, clean_psd = average_psd(clean, steady_start, steady_stop, profile.sample_rate_hz)
    _, mixed_psd = average_psd(mixed, steady_start, steady_stop, profile.sample_rate_hz)
    psd_path = directory / "spectrum.png"
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
    axes[0].plot(freq / 1e3, jammer_psd, linewidth=0.8)
    axes[0].set(title="Jammer-only power spectrum", ylabel="Power (dB)")
    axes[1].plot(freq / 1e3, clean_psd, label="Clean GPS", linewidth=0.8)
    axes[1].plot(freq / 1e3, mixed_psd, label="Mixed", linewidth=0.8)
    axes[1].set(
        title="Steady-state clean/mixed spectrum",
        xlabel="Baseband frequency (kHz)",
        ylabel="Power (dB)",
    )
    axes[1].legend()
    fig.savefig(psd_path, dpi=150)
    plt.close(fig)

    times, frequencies, spectrogram = sampled_spectrogram(mixed, profile)
    spectrogram_path = directory / "spectrogram.png"
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    mesh = ax.pcolormesh(
        times, frequencies / 1e3, spectrogram, shading="auto", cmap="magma"
    )
    ax.axvline(profile.onset_s, color="cyan", linestyle="--", linewidth=0.8)
    ax.axvline(profile.offset_s, color="cyan", linestyle="--", linewidth=0.8)
    ax.set(
        title="Full-run sampled spectrogram",
        xlabel="Time (s)",
        ylabel="Baseband frequency (kHz)",
    )
    fig.colorbar(mesh, ax=ax, label="Power (dB)")
    fig.savefig(spectrogram_path, dpi=150)
    plt.close(fig)

    detail_begin = profile.onset_s + profile.ramp_s
    detail_times, detail_frequencies, detail = dense_spectrogram(
        jammer, profile, detail_begin
    )
    detail_path = directory / "spectrogram-active-detail.png"
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    mesh = ax.pcolormesh(
        (detail_times - detail_begin) * 1e3,
        detail_frequencies / 1e3,
        detail,
        shading="auto",
        cmap="magma",
    )
    ax.set(
        title="Jammer-only active-window spectrogram (first 250 ms)",
        xlabel="Time into steady jammer interval (ms)",
        ylabel="Baseband frequency (kHz)",
    )
    fig.colorbar(mesh, ax=ax, label="PSD (dB/Hz)")
    fig.savefig(detail_path, dpi=150)
    plt.close(fig)

    time_axis, power = block_power(mixed, profile)
    power_path = directory / "block-power.png"
    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax.plot(time_axis, power, linewidth=1.0)
    ax.axvspan(
        profile.onset_s,
        profile.offset_s,
        alpha=0.15,
        color="tab:red",
        label="Jammer active",
    )
    ax.set(title="Complex IQ block power", xlabel="Time (s)", ylabel="Power (dBFS)")
    ax.legend()
    fig.savefig(power_path, dpi=150)
    plt.close(fig)
    return [
        str(psd_path),
        str(spectrogram_path),
        str(detail_path),
        str(power_path),
    ]


def analyze_fixture(
    *,
    jammer: Path,
    jammer_manifest: Path,
    clean: Path,
    mixed: Path,
    mix_manifest: Path,
    profile: Profile,
    expected_frequency: float | None,
    output_dir: Path,
    make_plots: bool,
) -> dict[str, Any]:
    expected_bytes = profile.samples * 4
    jammer_tool = read_json(jammer_manifest)
    mix_tool = read_json(mix_manifest)
    source_type = str(jammer_tool["source_type"])
    if source_type not in JAMMER_TYPES:
        raise WorkflowError(f"unsupported jammer source in manifest: {source_type}")
    parameters = jammer_tool["parameters"]
    measured_rms = measure_plateau_rms(jammer, profile)
    nonzero_outside = jammer_outside_nonzero(jammer, profile)
    requested_rms = (
        expected_pulsed_rms(parameters, profile)
        if source_type == "pulsed"
        else float(parameters["amplitude_full_scale"])
    )
    rms_error_db = 20 * math.log10(measured_rms / requested_rms)
    reported_rms = float(jammer_tool["measurements"]["active_plateau_rms_full_scale"])
    reported_rms_error_db = 20 * math.log10(measured_rms / reported_rms)
    checks: dict[str, dict[str, Any]] = {
        "sample_count": {
            "pass": jammer.stat().st_size == expected_bytes
            and mixed.stat().st_size == expected_bytes,
            "expected_bytes": expected_bytes,
            "jammer_bytes": jammer.stat().st_size,
            "mixed_bytes": mixed.stat().st_size,
        },
        "active_rms": {"pass": abs(rms_error_db) <= 0.1, "error_db": rms_error_db},
        "reported_active_rms": {
            "pass": abs(reported_rms_error_db) <= 0.1,
            "error_db": reported_rms_error_db,
        },
        "zero_outside_envelope": {
            "pass": nonzero_outside == 0,
            "nonzero_complex_samples": nonzero_outside,
        },
        "no_clipping": {
            "pass": int(jammer_tool["measurements"]["clipped_components"]) == 0
            and int(mix_tool["measurements"]["clipped_components"]) == 0,
            "source_clipped_components": int(
                jammer_tool["measurements"]["clipped_components"]
            ),
            "mixed_clipped_components": int(
                mix_tool["measurements"]["clipped_components"]
            ),
        },
    }
    source_measurements: dict[str, Any] = {
        "active_plateau_rms_full_scale": measured_rms,
        "requested_active_plateau_rms_full_scale": requested_rms,
    }
    if source_type == "cw":
        tone = estimate_tone(jammer, profile)
        target_frequency = (
            float(expected_frequency)
            if expected_frequency is not None
            else float(parameters["frequency_hz"])
        )
        checks["tone_frequency"] = {
            "pass": abs(tone["frequency_hz"] - target_frequency) <= 1.0,
            "error_hz": tone["frequency_hz"] - target_frequency,
        }
        checks["negative_frequency_image"] = {
            "pass": tone["negative_frequency_image_rejection_db"] >= 60.0,
            "rejection_db": tone["negative_frequency_image_rejection_db"],
        }
        source_measurements.update(tone)
    elif source_type in {"narrowband", "wideband"}:
        spectrum = estimate_noise_spectrum(
            jammer,
            profile,
            float(parameters["frequency_hz"]),
            float(parameters["bandwidth_hz"]),
        )
        source_measurements.update(spectrum)
        if source_type == "narrowband":
            bandwidth = float(parameters["bandwidth_hz"])
            center_tolerance = max(bandwidth * 0.1, 3 * profile.sample_rate_hz / 8192)
            checks["narrowband_spectrum"] = {
                "pass": abs(spectrum["center_error_hz"]) <= center_tolerance
                and spectrum["occupied_90_bandwidth_hz"] <= 6.0 * bandwidth
                and spectrum["power_fraction_in_requested_band"] >= 0.4,
                "center_error_hz": spectrum["center_error_hz"],
                "center_tolerance_hz": center_tolerance,
                "occupied_90_bandwidth_hz": spectrum["occupied_90_bandwidth_hz"],
                "power_fraction_in_requested_band": spectrum[
                    "power_fraction_in_requested_band"
                ],
            }
        else:
            checks["wideband_spectrum"] = {
                "pass": spectrum["occupied_90_bandwidth_hz"]
                >= 0.8 * profile.sample_rate_hz,
                "occupied_90_bandwidth_hz": spectrum["occupied_90_bandwidth_hz"],
            }
    elif source_type == "chirp":
        chirp = estimate_chirp(jammer, profile, parameters)
        source_measurements.update(chirp)
        tolerance = max(
            50.0,
            abs(
                float(parameters["end_frequency_hz"])
                - float(parameters["frequency_hz"])
            )
            * 2e-4,
        )
        checks["chirp_frequency_law"] = {
            "pass": chirp["rms_frequency_error_hz"] <= tolerance
            and chirp["max_frequency_error_hz"] <= 4.0 * tolerance,
            "rms_frequency_error_hz": chirp["rms_frequency_error_hz"],
            "max_frequency_error_hz": chirp["max_frequency_error_hz"],
            "tolerance_hz": tolerance,
        }
    else:
        pulse = analyze_pulse(jammer, profile, parameters)
        source_measurements.update(pulse)
        duty_tolerance = max(2.0 / int(parameters["pulse_period_samples"]), 1e-6)
        checks["pulse_envelope"] = {
            "pass": pulse["envelope_max_error_full_scale"] <= 0.001
            and pulse["nonzero_off_samples"] == 0
            and abs(
                float(pulse["nonzero_duty_cycle"]) - float(pulse["nominal_duty_cycle"])
            )
            <= duty_tolerance,
            **pulse,
            "duty_tolerance": duty_tolerance,
        }
    requested_js = mix_tool["parameters"]["requested_js_db"]
    measured_js = mix_tool["measurements"]["measured_js_db"]
    if requested_js is not None:
        error = float(measured_js) - float(requested_js)
        checks["measured_js"] = {"pass": abs(error) <= 0.1, "error_db": error}
    requested_snr = mix_tool["parameters"]["requested_snr_db"]
    if requested_snr is not None:
        error = float(mix_tool["measurements"]["measured_snr_db"]) - float(
            requested_snr
        )
        checks["measured_snr"] = {"pass": abs(error) <= 0.1, "error_db": error}
    plots = (
        plot_waveforms(jammer, clean, mixed, profile, output_dir) if make_plots else []
    )
    report = {
        "schema": "gps-sdr-sim.jammer-analysis.v2",
        "source_type": source_type,
        "pass": all(check["pass"] for check in checks.values()),
        "checks": checks,
        "source_measurements": source_measurements,
        "sha256": {
            "jammer": sha256_file(jammer),
            "clean": sha256_file(clean),
            "mixed": sha256_file(mixed),
        },
        "plots": plots,
    }
    write_json(output_dir / "waveform-report.json", report)
    with (output_dir / "waveform-summary.txt").open("w", encoding="utf-8") as stream:
        stream.write(
            f"{source_type} waveform acceptance: "
            f"{'PASS' if report['pass'] else 'FAIL'}\n"
        )
        for name, check in checks.items():
            stream.write(f"{'PASS' if check['pass'] else 'FAIL'} {name}\n")
    if not report["pass"]:
        failed = ", ".join(name for name, check in checks.items() if not check["pass"])
        raise WorkflowError(f"waveform acceptance failed for {mixed.name}: {failed}")
    return report


def parse_nmea_time(value: str) -> float | None:
    if len(value) < 6:
        return None
    try:
        return int(value[0:2]) * 3600 + int(value[2:4]) * 60 + float(value[4:])
    except ValueError:
        return None


def parse_nmea(
    directory: Path, profile: Profile, scenario_start_time: str | None
) -> dict[str, Any]:
    candidates = sorted(directory.glob("*.nmea"))
    fixes: list[dict[str, Any]] = []
    if not candidates:
        return {"files": [], "fixes": [], "valid_fix_count": 0}
    first_time: float | None = None
    if scenario_start_time:
        try:
            clock = scenario_start_time.split(",", 1)[1]
            hours, minutes, seconds = clock.split(":")
            first_time = int(hours) * 3600 + int(minutes) * 60 + float(seconds)
        except (IndexError, ValueError):
            first_time = None
    for path in candidates:
        with path.open(errors="replace") as stream:
            for line in stream:
                fields = line.strip().split(",")
                if not fields or not fields[0].endswith("GGA") or len(fields) < 8:
                    continue
                timestamp = parse_nmea_time(fields[1])
                if timestamp is None:
                    continue
                if first_time is None:
                    first_time = timestamp
                elapsed = timestamp - first_time
                if elapsed < -43200:
                    elapsed += 86400
                try:
                    quality = int(fields[6] or 0)
                    satellites = int(fields[7] or 0)
                except ValueError:
                    continue
                fixes.append(
                    {"time_s": elapsed, "valid": quality > 0, "satellites": satellites}
                )
    return {
        "files": [str(path) for path in candidates],
        "fixes": fixes,
        "valid_fix_count": sum(fix["valid"] for fix in fixes),
    }


def _mat_vector(data: dict[str, Any], names: tuple[str, ...]) -> np.ndarray | None:
    lowered = {key.lower(): key for key in data if not key.startswith("__")}
    for name in names:
        key = lowered.get(name.lower())
        if key is not None:
            return np.asarray(data[key]).reshape(-1)
    for key, original in lowered.items():
        if any(name.lower() in key for name in names):
            return np.asarray(data[original]).reshape(-1)
    return None


def load_tracking_dump(path: Path) -> dict[str, Any] | None:
    try:
        return loadmat(path, variable_names=None)
    except (NotImplementedError, OSError, ValueError):
        try:
            with h5py.File(path, "r") as hdf:
                return {key: np.asarray(hdf[key]) for key in hdf.keys()}
        except (OSError, KeyError):
            return None


def parse_tracking(directory: Path, profile: Profile) -> dict[str, Any]:
    series: list[dict[str, Any]] = []
    files = sorted(directory.glob("tracking_ch_*.mat"))
    for path in files:
        data = load_tracking_dump(path)
        if data is None:
            continue
        cn0 = _mat_vector(data, ("CN0_SNV_dB_Hz", "cn0_db_hz", "cn0"))
        prn = _mat_vector(data, ("PRN", "prn"))
        sample = _mat_vector(
            data,
            ("PRN_start_sample_count", "tracking_sample_counter", "sample_counter"),
        )
        if cn0 is None or cn0.size == 0:
            continue
        length = cn0.size
        if prn is None or prn.size == 0:
            prn = np.zeros(length)
        if sample is None or sample.size == 0:
            time_s = np.arange(length, dtype=np.float64) * 0.001
        else:
            time_s = sample[:length].astype(np.float64) / profile.sample_rate_hz
        for satellite in np.unique(prn[:length].astype(np.int64)):
            if satellite <= 0:
                continue
            mask = prn[:length].astype(np.int64) == satellite
            values = cn0[:length][mask].astype(np.float64)
            times = time_s[mask]
            seconds = np.floor(times).astype(np.int64)
            sampled_times: list[float] = []
            sampled_cn0: list[float] = []
            for second in np.unique(seconds):
                second_values = values[seconds == second]
                valid = second_values[np.isfinite(second_values) & (second_values > 0)]
                if valid.size:
                    sampled_times.append(float(second) + 0.5)
                    sampled_cn0.append(float(np.median(valid)))
            series.append(
                {
                    "prn": int(satellite),
                    "time_s": sampled_times,
                    "cn0_db_hz": sampled_cn0,
                }
            )
    return {"files": [str(path) for path in files], "series": series}


@dataclass(frozen=True)
class PromptChannel:
    prn: int
    sample_count: np.ndarray
    time_s: np.ndarray
    prompt: np.ndarray
    files: tuple[str, ...]


def load_prompt_channels(directory: Path, profile: Profile) -> dict[int, PromptChannel]:
    sample_parts: dict[int, list[np.ndarray]] = {}
    prompt_parts: dict[int, list[np.ndarray]] = {}
    source_files: dict[int, list[str]] = {}
    for path in sorted(directory.glob("tracking_ch_*.mat")):
        data = load_tracking_dump(path)
        if data is None:
            continue
        prompt_i = _mat_vector(data, ("Prompt_I", "prompt_i"))
        prompt_q = _mat_vector(data, ("Prompt_Q", "prompt_q"))
        prn = _mat_vector(data, ("PRN", "prn"))
        sample = _mat_vector(
            data,
            ("PRN_start_sample_count", "tracking_sample_counter", "sample_counter"),
        )
        if prompt_i is None or prompt_q is None or prn is None or sample is None:
            continue
        length = min(prompt_i.size, prompt_q.size, prn.size, sample.size)
        prompt = prompt_i[:length].astype(np.float64) + 1j * prompt_q[:length].astype(
            np.float64
        )
        sample_count = sample[:length].astype(np.int64)
        satellites = prn[:length].astype(np.int64)
        finite = (
            np.isfinite(prompt.real)
            & np.isfinite(prompt.imag)
            & (sample_count >= 0)
            & (sample_count < profile.samples)
        )
        for satellite in np.unique(satellites[finite]):
            if satellite <= 0:
                continue
            selected = finite & (satellites == satellite)
            sample_parts.setdefault(int(satellite), []).append(sample_count[selected])
            prompt_parts.setdefault(int(satellite), []).append(prompt[selected])
            source_files.setdefault(int(satellite), []).append(str(path))

    channels: dict[int, PromptChannel] = {}
    for satellite, parts in sample_parts.items():
        sample_count = np.concatenate(parts)
        prompt = np.concatenate(prompt_parts[satellite])
        order = np.argsort(sample_count, kind="stable")
        sample_count = sample_count[order]
        prompt = prompt[order]
        unique = np.concatenate(([True], np.diff(sample_count) > 0))
        sample_count = sample_count[unique]
        prompt = prompt[unique]
        channels[satellite] = PromptChannel(
            prn=satellite,
            sample_count=sample_count,
            time_s=sample_count.astype(np.float64) / profile.sample_rate_hz,
            prompt=prompt,
            files=tuple(dict.fromkeys(source_files[satellite])),
        )
    return channels


def uniform_prompt_series(
    channel: PromptChannel, profile: Profile, bin_s: float = 0.02
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, int]]:
    bins = math.ceil(profile.duration_s / bin_s)
    times = (np.arange(bins, dtype=np.float64) + 0.5) * bin_s
    values = np.full(bins, np.nan + 1j * np.nan, dtype=np.complex128)
    counts = np.zeros(bins, dtype=np.int64)
    if channel.time_s.size == 0:
        return times, values, counts.astype(bool), {}

    deltas = np.diff(channel.time_s)
    valid_deltas = deltas[(deltas >= 0.0005) & (deltas <= 0.025)]
    indices = np.floor(channel.time_s / bin_s).astype(np.int64)
    retained = (indices >= 0) & (indices < bins)
    sums = np.zeros(bins, dtype=np.complex128)
    np.add.at(sums, indices[retained], channel.prompt[retained])
    np.add.at(counts, indices[retained], 1)
    observed = counts > 0
    values[observed] = sums[observed] / counts[observed]

    rounded_ms = np.rint(valid_deltas * 1000.0).astype(np.int64)
    integration_histogram = {
        str(int(value)): int(np.count_nonzero(rounded_ms == value))
        for value in np.unique(rounded_ms)
    }
    return times, values, observed, integration_histogram


def prompt_welch_psd(
    values: np.ndarray, selected: np.ndarray, sample_rate_hz: float
) -> tuple[np.ndarray, np.ndarray]:
    interval = values[selected]
    if interval.size < 16 or np.count_nonzero(np.isfinite(interval)) < 16:
        return np.empty(0), np.empty(0)
    filled = np.where(np.isfinite(interval), interval, 0.0)
    nperseg = min(256, filled.size)
    frequencies, power = signal.welch(
        filled,
        fs=sample_rate_hz,
        window="hann",
        nperseg=nperseg,
        noverlap=nperseg // 2,
        detrend=NO_DETREND,
        return_onesided=False,
        scaling="density",
    )
    return np.fft.fftshift(frequencies), np.fft.fftshift(power)


def prompt_spectrogram(
    values: np.ndarray, observed: np.ndarray, sample_rate_hz: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if values.size < 32:
        return np.empty(0), np.empty(0), np.empty((0, 0)), np.empty(0)
    nperseg = min(128, values.size)
    noverlap = min(96, nperseg - 1)
    filled = np.where(np.isfinite(values), values, 0.0)
    frequencies, times, power = signal.spectrogram(
        filled,
        fs=sample_rate_hz,
        window="hann",
        nperseg=nperseg,
        noverlap=noverlap,
        detrend=NO_DETREND,
        return_onesided=False,
        scaling="density",
        mode="psd",
    )
    coverage = np.convolve(
        observed.astype(np.float64), np.ones(nperseg) / nperseg, mode="same"
    )
    coverage_indices = np.clip(
        np.rint(times * sample_rate_hz).astype(np.int64), 0, coverage.size - 1
    )
    window_coverage = coverage[coverage_indices]
    power[:, window_coverage < 0.5] = np.nan
    return (
        times,
        np.fft.fftshift(frequencies),
        np.fft.fftshift(power, axes=0),
        window_coverage,
    )


def plot_prn_spectral_report(
    tracking_directory: Path, profile: Profile, output_dir: Path
) -> dict[str, Any]:
    spectral_dir = output_dir / "prn-spectral"
    spectral_dir.mkdir(parents=True, exist_ok=True)
    channels = load_prompt_channels(tracking_directory, profile)
    interval_bounds = {
        "clean_prefix": (0.0, profile.onset_s),
        "jammer": (
            profile.onset_s + profile.ramp_s,
            profile.offset_s - profile.ramp_s,
        ),
        "recovery": (profile.offset_s, profile.duration_s),
    }
    interval_labels = {
        "clean_prefix": "Clean prefix",
        "jammer": "Jammer active",
        "recovery": "Recovery",
    }
    colors = {
        "clean_prefix": "#0072B2",
        "jammer": "#D55E00",
        "recovery": "#009E73",
    }
    line_styles = {"clean_prefix": "-", "jammer": "--", "recovery": ":"}
    plots: list[str] = []
    prn_reports: dict[str, Any] = {}
    uniform_rate_hz = 50.0

    for satellite, channel in sorted(channels.items()):
        times, values, observed, integration_histogram = uniform_prompt_series(
            channel, profile, 1.0 / uniform_rate_hz
        )
        baseline = (
            (times >= 0.0) & (times < profile.onset_s) & observed & np.isfinite(values)
        )
        if not np.any(baseline):
            continue
        baseline_power = float(np.mean(np.abs(values[baseline]) ** 2))
        if not math.isfinite(baseline_power) or baseline_power <= 0.0:
            continue
        normalized = values / math.sqrt(baseline_power)
        interval_reports: dict[str, Any] = {}

        fig, axes = plt.subplots(
            2, 1, figsize=(11, 7), constrained_layout=True, height_ratios=(1, 1.25)
        )
        for name, (begin, end) in interval_bounds.items():
            selected = (times >= begin) & (times < end)
            available = selected & observed & np.isfinite(normalized)
            frequency, power = prompt_welch_psd(normalized, selected, uniform_rate_hz)
            if frequency.size:
                axes[0].plot(
                    frequency,
                    10.0 * np.log10(np.maximum(power, 1e-20)),
                    color=colors[name],
                    linestyle=line_styles[name],
                    linewidth=1.2,
                    label=interval_labels[name],
                )
            relative_power_db = (
                10.0
                * math.log10(
                    max(float(np.mean(np.abs(normalized[available]) ** 2)), 1e-20)
                )
                if np.any(available)
                else None
            )
            interval_reports[name] = {
                "begin_s": begin,
                "end_s": end,
                "available_bins": int(np.count_nonzero(available)),
                "expected_bins": int(np.count_nonzero(selected)),
                "coverage_fraction": float(
                    np.count_nonzero(available) / max(1, np.count_nonzero(selected))
                ),
                "prompt_power_relative_to_clean_prefix_db": relative_power_db,
                "welch_frequency_bins": int(frequency.size),
            }
        axes[0].set(
            title=f"GPS G{satellite:02d} post-correlation prompt spectrum",
            xlabel="Residual prompt frequency (Hz)",
            ylabel="PSD relative to prefix power (dB/Hz)",
            xlim=(-uniform_rate_hz / 2.0, uniform_rate_hz / 2.0),
        )
        axes[0].grid(alpha=0.25)
        axes[0].legend(fontsize=8)

        spectrogram_times, frequencies, power, coverage = prompt_spectrogram(
            normalized, observed, uniform_rate_hz
        )
        if power.size:
            power_db = 10.0 * np.log10(np.maximum(power, 1e-20))
            finite_power = power_db[np.isfinite(power_db)]
            vmax = (
                float(np.percentile(finite_power, 99.0)) if finite_power.size else 0.0
            )
            vmin = vmax - 50.0
            mesh = axes[1].pcolormesh(
                spectrogram_times,
                frequencies,
                power_db,
                shading="auto",
                cmap="magma",
                vmin=vmin,
                vmax=vmax,
            )
            fig.colorbar(mesh, ax=axes[1], label="PSD (dB/Hz)")
        axes[1].axvline(profile.onset_s, color="#56B4E9", linestyle="--", linewidth=1.0)
        axes[1].axvline(
            profile.offset_s, color="#56B4E9", linestyle="--", linewidth=1.0
        )
        axes[1].set(
            title="Despread prompt spectrogram (low-coverage windows are blank)",
            xlabel="Time (s)",
            ylabel="Residual prompt frequency (Hz)",
            xlim=(0.0, profile.duration_s),
            ylim=(-uniform_rate_hz / 2.0, uniform_rate_hz / 2.0),
        )

        stem = spectral_dir / f"G{satellite:02d}-prompt-spectrum-spectrogram"
        png_path = stem.with_suffix(".png")
        pdf_path = stem.with_suffix(".pdf")
        fig.savefig(png_path, dpi=300)
        fig.savefig(pdf_path)
        plt.close(fig)
        plots.extend((str(png_path), str(pdf_path)))
        prn_reports[str(satellite)] = {
            "source_files": list(channel.files),
            "raw_prompt_records": int(channel.prompt.size),
            "integration_interval_histogram_ms": integration_histogram,
            "uniform_prompt_rate_hz": uniform_rate_hz,
            "baseline_prompt_power_correlator_units2": baseline_power,
            "spectrogram_windows": int(spectrogram_times.size),
            "spectrogram_median_coverage": (
                float(np.median(coverage)) if coverage.size else None
            ),
            "intervals": interval_reports,
            "plots": [str(png_path), str(pdf_path)],
        }

    report = {
        "schema": "gps-sdr-sim.gnss-sdr-prn-spectral-report.v1",
        "measurement_domain": "GNSS-SDR post-correlation prompt I/Q",
        "interpretation_boundary": (
            "These are despread tracking-channel spectra, not raw RF spectra "
            "isolated by PRN. Raw GPS L1 C/A PRNs overlap in the same RF band."
        ),
        "resampling": (
            "The 1 ms or 20 ms prompt correlator records are averaged onto a "
            "50 Hz time grid without treating them as uniformly sampled, then "
            "normalized to each PRN clean-prefix prompt power."
        ),
        "spectrogram_missing_data_policy": (
            "Missing 20 ms bins are zero-filled for the transform; STFT windows "
            "with less than 50 percent tracking coverage are blanked."
        ),
        "prns": prn_reports,
        "plots": plots,
    }
    write_json(spectral_dir / "prn-spectral-report.json", report)
    return report


def interval_mask(times: np.ndarray, begin: float, end: float) -> np.ndarray:
    return (times >= begin) & (times < end)


def receiver_metrics(
    nmea: dict[str, Any], tracking: dict[str, Any], profile: Profile
) -> dict[str, Any]:
    intervals = {
        "clean_prefix": (0.0, profile.onset_s),
        "jammer": (profile.onset_s + profile.ramp_s, profile.offset_s - profile.ramp_s),
        "recovery": (profile.offset_s, profile.duration_s),
    }
    grouped: dict[int, tuple[list[float], list[float]]] = {}
    for row in tracking["series"]:
        times, cn0 = grouped.setdefault(int(row["prn"]), ([], []))
        times.extend(float(value) for value in row["time_s"])
        cn0.extend(float(value) for value in row["cn0_db_hz"])
    per_prn: dict[str, dict[str, Any]] = {}
    for satellite, (time_values, cn0_values) in grouped.items():
        times = np.asarray(time_values)
        cn0 = np.asarray(cn0_values)
        entry = per_prn.setdefault(str(satellite), {})
        for name, (begin, end) in intervals.items():
            mask = interval_mask(times, begin, end) & np.isfinite(cn0) & (cn0 > 0)
            if np.any(mask):
                entry[name] = {
                    "median_cn0_db_hz": float(np.median(cn0[mask])),
                    "samples": int(np.count_nonzero(mask)),
                }
    fixes = nmea["fixes"]
    prefix = [fix for fix in fixes if fix["time_s"] < profile.onset_s and fix["valid"]]
    gate = any(fix["satellites"] >= 4 for fix in prefix)
    recovery = next(
        (
            fix["time_s"] - profile.offset_s
            for fix in fixes
            if fix["time_s"] >= profile.offset_s
            and fix["valid"]
            and fix["satellites"] >= 4
        ),
        None,
    )
    pvt_intervals: dict[str, dict[str, Any]] = {}
    for name, (begin, end) in intervals.items():
        interval_fixes = [fix for fix in fixes if begin <= fix["time_s"] < end]
        valid = sum(fix["valid"] for fix in interval_fixes)
        expected = max(1, round(end - begin))
        pvt_intervals[name] = {
            "valid_fixes": valid,
            "expected_1hz_fixes": expected,
            "availability": valid / expected,
            "median_usable_satellites": (
                float(
                    np.median(
                        [fix["satellites"] for fix in interval_fixes if fix["valid"]]
                    )
                )
                if any(fix["valid"] for fix in interval_fixes)
                else 0.0
            ),
        }
    stable_prns = {
        satellite
        for satellite, values in per_prn.items()
        if "clean_prefix" in values
        and values["clean_prefix"]["median_cn0_db_hz"] >= 30.0
    }
    median_cn0_by_interval: dict[str, float | None] = {}
    for interval_name in intervals:
        interval_cn0 = [
            float(per_prn[satellite][interval_name]["median_cn0_db_hz"])
            for satellite in stable_prns
            if interval_name in per_prn[satellite]
        ]
        median_cn0_by_interval[interval_name] = (
            float(np.median(interval_cn0)) if interval_cn0 else None
        )
    tracked_counts: dict[int, set[int]] = {}
    for row in tracking["series"]:
        for time_s, cn0 in zip(row["time_s"], row["cn0_db_hz"], strict=False):
            if cn0 >= 30.0:
                tracked_counts.setdefault(int(float(time_s)), set()).add(
                    int(row["prn"])
                )
    return {
        "clean_prefix_gate": gate,
        "clean_prefix_valid_fixes": len(prefix),
        "clean_prefix_max_usable_satellites": max(
            (fix["satellites"] for fix in prefix), default=0
        ),
        "stable_prns": sorted(int(satellite) for satellite in stable_prns),
        "stable_median_cn0_db_hz": median_cn0_by_interval["clean_prefix"],
        "median_cn0_db_hz_by_interval": median_cn0_by_interval,
        "per_prn": per_prn,
        "pvt_intervals": pvt_intervals,
        "tracked_count_1hz": [
            {"time_s": second, "count": len(prns)}
            for second, prns in sorted(tracked_counts.items())
        ],
        "time_to_restored_pvt_s": recovery,
    }


def plot_receiver_report(
    report: dict[str, Any], profile: Profile, output_dir: Path
) -> list[str]:
    metrics = report["metrics"]
    paths: list[str] = []
    cn0_path = output_dir / "cn0-per-prn.png"
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    for row in report["tracking"]["series"]:
        ax.plot(
            row["time_s"], row["cn0_db_hz"], linewidth=0.9, label=f"G{row['prn']:02d}"
        )
    ax.axvspan(profile.onset_s, profile.offset_s, color="tab:red", alpha=0.12)
    ax.set(title="GNSS-SDR per-PRN C/N0", xlabel="Time (s)", ylabel="C/N0 (dB-Hz)")
    if report["tracking"]["series"]:
        ax.legend(ncol=4, fontsize=8)
    fig.savefig(cn0_path, dpi=150)
    plt.close(fig)
    paths.append(str(cn0_path))

    state_path = output_dir / "receiver-state.png"
    fig, axes = plt.subplots(
        2, 1, figsize=(11, 7), sharex=True, constrained_layout=True
    )
    counts = metrics["tracked_count_1hz"]
    axes[0].step(
        [item["time_s"] for item in counts],
        [item["count"] for item in counts],
        where="mid",
        label="Tracked (C/N0 >= 30)",
    )
    fixes = report["nmea"]["fixes"]
    axes[0].scatter(
        [fix["time_s"] for fix in fixes],
        [fix["satellites"] for fix in fixes],
        s=12,
        label="PVT usable satellites",
    )
    axes[0].set(ylabel="Satellite count", title="Tracked and PVT-usable satellites")
    axes[0].legend()
    axes[1].step(
        [fix["time_s"] for fix in fixes],
        [1 if fix["valid"] else 0 for fix in fixes],
        where="post",
    )
    axes[1].set(xlabel="Time (s)", ylabel="Valid PVT", yticks=(0, 1))
    for ax in axes:
        ax.axvspan(profile.onset_s, profile.offset_s, color="tab:red", alpha=0.12)
    fig.savefig(state_path, dpi=150)
    plt.close(fig)
    paths.append(str(state_path))
    return paths


def run_receiver(
    fixture: Path,
    profile: Profile,
    receiver_binary: str,
    config: Path,
    output_dir: Path,
    scenario_start_time: str | None,
    receiver_prns: tuple[int, ...] = (),
    make_prn_spectra: bool = True,
) -> dict[str, Any]:
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    resolved = output_dir / "resolved-receiver.conf"
    config_text = config.read_text(encoding="utf-8")
    if receiver_prns:
        config_text = re.sub(
            r"^Channels_1C\.count=\d+$",
            f"Channels_1C.count={len(receiver_prns)}",
            config_text,
            flags=re.MULTILINE,
        )
        config_text = re.sub(
            r"^Channels\.in_acquisition=\d+$",
            f"Channels.in_acquisition={len(receiver_prns)}",
            config_text,
            flags=re.MULTILINE,
        )
        assignments = ["", "; Source-derived visible constellation"]
        for channel, prn in enumerate(receiver_prns):
            assignments.extend(
                [f"Channel{channel}.signal=1C", f"Channel{channel}.satellite={prn}"]
            )
        config_text += "\n".join(assignments) + "\n"
    resolved.write_text(config_text, encoding="utf-8")
    command = [
        receiver_binary,
        f"--config_file={resolved}",
        f"--signal_source={fixture.resolve()}",
        "--keyboard=false",
        f"--log_dir={output_dir}",
    ]
    result = subprocess.run(command, cwd=output_dir, text=True, capture_output=True)
    receiver_log = (result.stdout or "") + (result.stderr or "")
    (output_dir / "receiver.log").write_text(receiver_log, encoding="utf-8")
    logical_exit = result.returncode
    if (
        "Unable to connect flowgraph" in receiver_log
        or "configuration file is not well defined" in receiver_log
    ):
        logical_exit = logical_exit or 2
    nmea = parse_nmea(output_dir, profile, scenario_start_time)
    tracking = parse_tracking(output_dir, profile)
    metrics = receiver_metrics(nmea, tracking, profile)
    report = {
        "schema": "gps-sdr-sim.gnss-sdr-report.v1",
        "command": command,
        "version": executable_version(receiver_binary),
        "exit_code": logical_exit,
        "process_exit_code": result.returncode,
        "configuration": str(resolved),
        "nmea": nmea,
        "tracking": tracking,
        "metrics": metrics,
    }
    report["plots"] = plot_receiver_report(report, profile, output_dir)
    if make_prn_spectra:
        report["prn_spectral"] = plot_prn_spectral_report(
            output_dir, profile, output_dir
        )
        report["plots"].extend(report["prn_spectral"]["plots"])
    else:
        report["prn_spectral"] = None
    write_json(output_dir / "receiver-report.json", report)
    return report


def calibration_cn0(report: dict[str, Any]) -> float | None:
    values = [
        float(cn0)
        for row in report["tracking"]["series"]
        for time_s, cn0 in zip(row["time_s"], row["cn0_db_hz"], strict=False)
        if 3.0 <= float(time_s) and math.isfinite(float(cn0)) and float(cn0) >= 30.0
    ]
    return float(np.median(values)) if values else None


def calibrate_noise(
    *,
    args: argparse.Namespace,
    profile: Profile,
    clean: Path,
    jammer: Path,
    levels: tuple[float, ...],
    initial_noise_rms: float,
    output_dir: Path,
    receiver_config: Path,
    receiver_prns: tuple[int, ...],
) -> tuple[float, list[dict[str, Any]]]:
    duration = min(15.0, profile.duration_s)
    pilot = Profile(
        "calibration",
        profile.sample_rate_hz,
        duration,
        duration / 3,
        2 * duration / 3,
        profile.ramp_s,
        (),
    )
    calibration_dir = output_dir / "calibration"
    calibration_dir.mkdir(parents=True, exist_ok=True)
    pilot_clean = calibration_dir / "clean-source.bin"
    copy_exact_samples(clean, pilot_clean, pilot.samples)
    pilot_jammer, _ = generate_jammer(args, pilot, calibration_dir)
    noise_rms = initial_noise_rms
    records: list[dict[str, Any]] = []
    for iteration in range(args.calibration_iterations):
        gain, _ = derive_common_gain(
            clean,
            jammer,
            profile,
            args.seed,
            max(levels),
            None,
            noise_rms,
            calibration_dir,
        )
        iteration_dir = calibration_dir / f"iteration-{iteration + 1}"
        iteration_dir.mkdir(parents=True, exist_ok=True)
        fixture = iteration_dir / "pilot.bin"
        tool_manifest = iteration_dir / "iqmix.tool.json"
        run(
            iqmix_command(
                clean=pilot_clean,
                jammer=pilot_jammer,
                output=fixture,
                manifest=tool_manifest,
                profile=pilot,
                seed=args.seed,
                gain=gain,
                js_db=None,
                noise_rms=noise_rms,
            )
        )
        receiver = run_receiver(
            fixture,
            pilot,
            args.gnss_sdr,
            receiver_config,
            iteration_dir / "receiver",
            args.start_time,
            receiver_prns,
            make_prn_spectra=False,
        )
        if receiver["exit_code"] != 0:
            raise WorkflowError("GNSS-SDR noise-calibration pilot failed")
        cn0 = calibration_cn0(receiver)
        record = {
            "iteration": iteration + 1,
            "noise_rms_counts": noise_rms,
            "common_gain": gain,
            "median_cn0_db_hz": cn0,
            "target_cn0_db_hz": args.target_cn0,
        }
        records.append(record)
        if cn0 is None:
            raise WorkflowError(
                "GNSS-SDR calibration produced no parseable tracking C/N0"
            )
        if abs(cn0 - args.target_cn0) <= args.cn0_tolerance:
            write_json(
                calibration_dir / "calibration-report.json",
                {"pass": True, "iterations": records},
            )
            return noise_rms, records
        noise_rms *= 10 ** ((cn0 - args.target_cn0) / 20.0)
    write_json(
        calibration_dir / "calibration-report.json",
        {"pass": False, "iterations": records},
    )
    raise WorkflowError(
        f"GNSS-SDR noise calibration did not reach {args.target_cn0:g} +/- "
        f"{args.cn0_tolerance:g} dB-Hz"
    )


def augment_manifest(
    *,
    tool_manifest: Path,
    output_manifest: Path,
    fixture: Path,
    clean: Path,
    jammer: Path,
    profile: Profile,
    source: dict[str, Any],
    common: dict[str, Any],
    receiver_config: Path,
    receiver_binary: str,
    seed: int,
) -> dict[str, Any]:
    manifest = read_json(tool_manifest)
    manifest.update(
        {
            "schema": "gps-sdr-sim.jamming-fixture.v2",
            "profile": asdict(profile),
            "source": source,
            "provenance": {
                "tool_commit": git_commit(),
                "gnss_sdr_executable": receiver_binary,
                "gnss_sdr_version": executable_version(receiver_binary),
                "receiver_configuration": str(receiver_config),
                "noise_seed": seed,
            },
            "common_scaling": common,
            "files": {
                "clean": {"path": str(clean), "sha256": sha256_file(clean)},
                "jammer": {"path": str(jammer), "sha256": sha256_file(jammer)},
                "fixture": {"path": str(fixture), "sha256": sha256_file(fixture)},
            },
        }
    )
    write_json(output_manifest, manifest)
    return manifest


def check_disk(directory: Path, profile: Profile, fixture_count: int) -> None:
    required = profile.samples * 4 * (fixture_count + 2)
    free = shutil.disk_usage(directory).free
    if free < int(required * 1.1):
        raise WorkflowError(
            f"insufficient free space: need about {required / 1e9:.2f} GB plus 10%, "
            f"have {free / 1e9:.2f} GB"
        )


def workflow(args: argparse.Namespace) -> int:
    profile = PROFILES[args.profile]
    if args.refine_around is not None:
        levels = tuple(args.refine_around + offset for offset in (-4, -2, 0, 2, 4))
    elif args.js_levels:
        levels = tuple(float(value) for value in args.js_levels.split(","))
    else:
        levels = profile.levels_db
    if not levels:
        raise WorkflowError("at least one J/S level is required")
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    check_disk(output_dir, profile, len(levels) + 1)
    if not args.skip_build:
        run(["make", "gps-sdr-sim", "jammergen", "iqmix"])

    clean = output_dir / "clean-source.bin"
    source = generate_clean(args, profile, clean)
    jammer, jammer_tool_manifest = generate_jammer(args, profile, output_dir)
    jammer_hash = sha256_file(jammer)
    jammer_metadata = read_json(jammer_tool_manifest)
    jammer_metadata["sha256"] = jammer_hash
    write_json(
        output_dir / f"{jammer_metadata['source_type']}-jammer.manifest.json",
        jammer_metadata,
    )

    gain, dry_metrics = derive_common_gain(
        clean,
        jammer,
        profile,
        args.seed,
        max(levels),
        args.snr_db,
        None,
        output_dir,
    )
    common = {
        "gain": gain,
        "headroom_target_db": 1.0,
        "highest_js_db": max(levels),
        "dry_run_measurements": dry_metrics["measurements"],
    }

    def level_label(level: float) -> str:
        sign = "p" if level >= 0 else "m"
        return f"js-{sign}{abs(level):g}db"

    seed_count = (
        args.seed_count
        if args.seed_count is not None
        else (5 if args.refine_around is not None else 1)
    )
    if seed_count < 1:
        raise WorkflowError("--seed-count must be at least one")
    seeds = tuple(args.seed + offset for offset in range(seed_count))
    specs: list[tuple[str, float | None, int]] = []
    for seed in seeds:
        suffix = f"-seed-{seed}" if seed_count > 1 else ""
        specs.append((f"clean{suffix}", None, seed))
        specs.extend((f"{level_label(level)}{suffix}", level, seed) for level in levels)
    index_entries: list[dict[str, Any]] = []
    receiver_mode = args.receiver
    if receiver_mode == "auto":
        receiver_mode = (
            "all"
            if profile.name in {"canonical", "verification"}
            and shutil.which(args.gnss_sdr)
            else "none"
        )
    receiver_config = Path(args.receiver_config).resolve()
    if args.receiver_prns:
        receiver_prns = tuple(
            dict.fromkeys(int(value) for value in args.receiver_prns.split(","))
        )
    else:
        receiver_prns = tuple(int(value) for value in source.get("visible_prns", []))
    if any(prn < 1 or prn > 32 for prn in receiver_prns):
        raise WorkflowError("--receiver-prns values must be in 1..32")
    calibrate = args.calibrate_noise
    if calibrate is None:
        calibrate = (
            profile.name in {"canonical", "verification"} and receiver_mode != "none"
        )
    noise_rms: float | None = None
    calibration_records: list[dict[str, Any]] = []
    if calibrate:
        noise_rms = float(dry_metrics["measurements"]["noise_rms_counts"])
        noise_rms, calibration_records = calibrate_noise(
            args=args,
            profile=profile,
            clean=clean,
            jammer=jammer,
            levels=levels,
            initial_noise_rms=noise_rms,
            output_dir=output_dir,
            receiver_config=receiver_config,
            receiver_prns=receiver_prns,
        )
        gain, dry_metrics = derive_common_gain(
            clean,
            jammer,
            profile,
            args.seed,
            max(levels),
            None,
            noise_rms,
            output_dir,
        )
        common = {
            "gain": gain,
            "headroom_target_db": 1.0,
            "highest_js_db": max(levels),
            "dry_run_measurements": dry_metrics["measurements"],
            "calibrated_noise_rms_counts": noise_rms,
            "calibration_iterations": calibration_records,
        }

    for label, level, seed in specs:
        fixture_dir = output_dir / label
        fixture_dir.mkdir(parents=True, exist_ok=True)
        fixture = fixture_dir / "fixture.bin"
        tool_manifest = fixture_dir / "iqmix.tool.json"
        run(
            iqmix_command(
                clean=clean,
                jammer=jammer,
                output=fixture,
                manifest=tool_manifest,
                profile=profile,
                seed=seed,
                gain=gain,
                js_db=level,
                snr_db=args.snr_db if noise_rms is None else None,
                noise_rms=noise_rms,
            )
        )
        manifest_path = fixture_dir / "manifest.json"
        manifest = augment_manifest(
            tool_manifest=tool_manifest,
            output_manifest=manifest_path,
            fixture=fixture,
            clean=clean,
            jammer=jammer,
            profile=profile,
            source=source,
            common=common,
            receiver_config=receiver_config,
            receiver_binary=args.gnss_sdr,
            seed=seed,
        )
        report = analyze_fixture(
            jammer=jammer,
            jammer_manifest=jammer_tool_manifest,
            clean=clean,
            mixed=fixture,
            mix_manifest=tool_manifest,
            profile=profile,
            expected_frequency=args.frequency_hz,
            output_dir=fixture_dir / "analysis",
            make_plots=not args.no_plots,
        )
        receiver_report = None
        should_run = receiver_mode == "all" or (
            receiver_mode == "clean" and level is None
        )
        if should_run:
            receiver_report = run_receiver(
                fixture,
                profile,
                args.gnss_sdr,
                receiver_config,
                fixture_dir / "receiver",
                args.start_time,
                receiver_prns,
                make_prn_spectra=not args.no_prn_plots,
            )
            if receiver_report["exit_code"] != 0:
                raise WorkflowError(f"GNSS-SDR failed for {label}")
            if not receiver_report["metrics"]["clean_prefix_gate"]:
                raise WorkflowError(
                    f"GNSS-SDR clean-prefix gate failed for {label}: "
                    "no valid PVT with four satellites"
                )
        index_entries.append(
            {
                "label": label,
                "js_db": level,
                "noise_seed": seed,
                "fixture": str(fixture),
                "manifest": str(manifest_path),
                "sha256": manifest["files"]["fixture"]["sha256"],
                "waveform_report": str(
                    fixture_dir / "analysis" / "waveform-report.json"
                ),
                "receiver_report": str(
                    fixture_dir / "receiver" / "receiver-report.json"
                )
                if receiver_report
                else None,
                "pass": report["pass"]
                and (receiver_report is None or receiver_report["exit_code"] == 0),
            }
        )

    threshold: dict[str, Any] | None = None
    receiver_entries = [entry for entry in index_entries if entry["receiver_report"]]
    baseline_entry = next(
        (entry for entry in receiver_entries if entry["js_db"] is None), None
    )
    if baseline_entry:
        baseline_metrics = read_json(Path(baseline_entry["receiver_report"]))["metrics"]
        baseline_cn0 = baseline_metrics["median_cn0_db_hz_by_interval"]["jammer"]
        baseline_usable = baseline_metrics["pvt_intervals"]["jammer"][
            "median_usable_satellites"
        ]
        for entry in sorted(
            (item for item in receiver_entries if item["js_db"] is not None),
            key=lambda item: float(item["js_db"]),
        ):
            metrics = read_json(Path(entry["receiver_report"]))["metrics"]
            jammer_pvt = metrics["pvt_intervals"]["jammer"]
            cn0 = metrics["median_cn0_db_hz_by_interval"]["jammer"]
            signals = {
                "cn0_drop_at_least_3_db": bool(
                    baseline_cn0 is not None
                    and cn0 is not None
                    and baseline_cn0 - cn0 >= 3.0
                ),
                "usable_satellite_loss": bool(
                    jammer_pvt["median_usable_satellites"] <= baseline_usable - 1
                ),
                "pvt_availability_below_90_percent": jammer_pvt["availability"] < 0.9,
            }
            if any(signals.values()):
                threshold = {"js_db": entry["js_db"], "signals": signals}
                break

    index = {
        "schema": "gps-sdr-sim.jamming-dataset-index.v2",
        "offline_only": True,
        "safety": "For offline GNSS-SDR processing only; this workflow does not emit open-air transmission commands.",
        "profile": asdict(profile),
        "source": source,
        "shared": {
            "clean_sha256": sha256_file(clean),
            "jammer_sha256": jammer_hash,
            "noise_seed": args.seed,
            "snr_db": args.snr_db,
            "calibrated_noise_rms_counts": noise_rms,
            "jammer_type": jammer_metadata["source_type"],
            "jammer_parameters": jammer_metadata["parameters"],
            "common_gain": gain,
        },
        "fixtures": index_entries,
        "first_measured_degradation_threshold": threshold,
    }
    write_json(output_dir / "dataset-index.json", index)
    return 0


def analyze_command(args: argparse.Namespace) -> int:
    profile = PROFILES[args.profile]
    analyze_fixture(
        jammer=Path(args.jammer),
        jammer_manifest=Path(args.jammer_manifest),
        clean=Path(args.clean),
        mixed=Path(args.mixed),
        mix_manifest=Path(args.mix_manifest),
        profile=profile,
        expected_frequency=args.frequency_hz,
        output_dir=Path(args.output_dir),
        make_plots=not args.no_plots,
    )
    return 0


def plot_prn_command(args: argparse.Namespace) -> int:
    report = plot_prn_spectral_report(
        Path(args.tracking_dir),
        PROFILES[args.profile],
        Path(args.output_dir),
    )
    if not report["prns"]:
        raise WorkflowError("no parseable per-PRN prompt I/Q tracking dumps found")
    return 0


def parser() -> argparse.ArgumentParser:
    main = argparse.ArgumentParser(
        description=(
            "Generate and validate reproducible offline synthetic-jammer/GPS IQ "
            "datasets."
        )
    )
    commands = main.add_subparsers(dest="command", required=True)
    create = commands.add_parser(
        "create", help="run generation through receiver validation"
    )
    create.add_argument("--profile", choices=PROFILES, default="canonical")
    create.add_argument("--output-dir", required=True)
    source = create.add_mutually_exclusive_group()
    source.add_argument(
        "--clean-input", help="existing exact-format clean IQ (fast/reuse path)"
    )
    source.add_argument("--rinex", help="frozen RINEX navigation file")
    create.add_argument("--start-time", help="gps-sdr-sim time YYYY/MM/DD,hh:mm:ss")
    create.add_argument("--location", default="21.0047844,105.8460541,5")
    create.add_argument("--jammer-type", choices=JAMMER_TYPES, default="cw")
    create.add_argument(
        "--frequency-hz",
        type=float,
        help="fixed/center/chirp-start offset (type-specific default)",
    )
    create.add_argument("--end-frequency-hz", type=float)
    create.add_argument("--bandwidth-hz", type=float, default=100_000.0)
    create.add_argument("--phase-rad", type=float, default=0.0)
    create.add_argument(
        "--jammer-amplitude",
        type=float,
        help="defaults to 0.15 for noise modes and 0.5 otherwise",
    )
    create.add_argument("--jammer-seed", type=int, default=20260712)
    create.add_argument("--chirp-period-s", type=float, default=0.1)
    create.add_argument("--pulse-period-s", type=float, default=0.01)
    create.add_argument("--pulse-duty-cycle", type=float, default=0.1)
    create.add_argument("--pulse-ramp-s", type=float, default=0.00005)
    create.add_argument("--snr-db", type=float, default=-10.0)
    calibration = create.add_mutually_exclusive_group()
    calibration.add_argument(
        "--calibrate-noise", dest="calibrate_noise", action="store_true"
    )
    calibration.add_argument(
        "--no-calibrate-noise", dest="calibrate_noise", action="store_false"
    )
    create.set_defaults(calibrate_noise=None)
    create.add_argument("--target-cn0", type=float, default=45.0)
    create.add_argument("--cn0-tolerance", type=float, default=2.0)
    create.add_argument("--calibration-iterations", type=int, default=4)
    create.add_argument("--seed", type=int, default=20260712)
    levels = create.add_mutually_exclusive_group()
    levels.add_argument("--js-levels", help="comma-separated override")
    levels.add_argument(
        "--refine-around",
        type=float,
        help="materialize center +/- 4 dB at 2 dB spacing",
    )
    create.add_argument(
        "--seed-count",
        type=int,
        help="independent consecutive seeds (defaults to five for refinement)",
    )
    create.add_argument(
        "--receiver", choices=("auto", "none", "clean", "all"), default="auto"
    )
    create.add_argument("--gnss-sdr", default=default_receiver_binary())
    create.add_argument("--receiver-config", default=str(DEFAULT_RECEIVER_CONFIG))
    create.add_argument(
        "--receiver-prns",
        help="comma-separated source-visible PRNs (normally derived from gps-sdr-sim)",
    )
    create.add_argument("--skip-build", action="store_true")
    create.add_argument("--no-plots", action="store_true")
    create.add_argument(
        "--no-prn-plots",
        action="store_true",
        help="skip post-correlation per-PRN spectrum/spectrogram figures",
    )
    create.set_defaults(func=workflow)

    analyze = commands.add_parser("analyze", help="run numerical waveform acceptance")
    analyze.add_argument("--profile", choices=PROFILES, required=True)
    analyze.add_argument("--jammer", required=True)
    analyze.add_argument("--jammer-manifest", required=True)
    analyze.add_argument("--clean", required=True)
    analyze.add_argument("--mixed", required=True)
    analyze.add_argument("--mix-manifest", required=True)
    analyze.add_argument(
        "--frequency-hz",
        type=float,
        help="optional CW expected-frequency override; manifest is authoritative",
    )
    analyze.add_argument("--output-dir", required=True)
    analyze.add_argument("--no-plots", action="store_true")
    analyze.set_defaults(func=analyze_command)

    plot_prn = commands.add_parser(
        "plot-prn", help="plot per-PRN spectra from GNSS-SDR tracking MAT dumps"
    )
    plot_prn.add_argument("--profile", choices=PROFILES, required=True)
    plot_prn.add_argument("--tracking-dir", required=True)
    plot_prn.add_argument("--output-dir", required=True)
    plot_prn.set_defaults(func=plot_prn_command)
    return main


def main() -> int:
    args = parser().parse_args()
    try:
        return int(args.func(args))
    except (WorkflowError, OSError, ValueError, KeyError) as exc:
        print(f"cw-dataset: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
