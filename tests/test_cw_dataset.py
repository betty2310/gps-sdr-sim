#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np
from numpy.typing import NDArray


ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / "processing" / "cw_dataset.py"


def command(*args: object, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(value) for value in args],
        cwd=ROOT,
        check=check,
        text=True,
        capture_output=True,
    )


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class CwDatasetTest(unittest.TestCase):
    def test_per_prn_spectral_plots_use_gnss_sdr_prompt_dumps(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            tracking = directory / "tracking"
            tracking.mkdir()
            times = np.arange(0.0, 3.0, 0.02, dtype=np.float64)
            prompt: NDArray[np.complex128] = np.asarray(
                np.exp(2j * np.pi * 2.0 * times), dtype=np.complex128
            )
            active = (times >= 1.0) & (times < 2.0)
            prompt[active] += 0.5 * np.exp(2j * np.pi * 10.0 * times[active])
            sample_count = np.rint(times * 2_600_000).astype(np.uint64)
            with h5py.File(tracking / "tracking_ch_0.mat", "w") as hdf:
                hdf.create_dataset("PRN", data=np.full((times.size, 1), 10))
                hdf.create_dataset(
                    "PRN_start_sample_count", data=sample_count.reshape(-1, 1)
                )
                hdf.create_dataset(
                    "Prompt_I", data=prompt.real.astype(np.float32).reshape(-1, 1)
                )
                hdf.create_dataset(
                    "Prompt_Q", data=prompt.imag.astype(np.float32).reshape(-1, 1)
                )

            output = directory / "output"
            command(
                "python",
                WORKFLOW,
                "plot-prn",
                "--profile",
                "fast",
                "--tracking-dir",
                tracking,
                "--output-dir",
                output,
            )
            report = json.loads(
                (output / "prn-spectral" / "prn-spectral-report.json").read_text()
            )
            self.assertEqual(
                report["measurement_domain"],
                "GNSS-SDR post-correlation prompt I/Q",
            )
            self.assertIn("10", report["prns"])
            self.assertEqual(
                report["prns"]["10"]["integration_interval_histogram_ms"],
                {"20": 149},
            )
            self.assertIsNotNone(
                report["prns"]["10"]["intervals"]["jammer"][
                    "prompt_power_relative_to_clean_prefix_db"
                ]
            )
            self.assertTrue(all(Path(path).is_file() for path in report["plots"]))

    def test_jammergen_supports_all_deterministic_source_types(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for source_type in (
                "cw",
                "narrowband",
                "wideband",
                "chirp",
                "pulsed",
            ):
                outputs = [
                    directory / f"{source_type}-a.bin",
                    directory / f"{source_type}-b.bin",
                ]
                manifests = [
                    directory / f"{source_type}-a.json",
                    directory / f"{source_type}-b.json",
                ]
                for output, manifest in zip(outputs, manifests, strict=True):
                    command(
                        ROOT / "jammergen",
                        "--type",
                        source_type,
                        "--output",
                        output,
                        "--manifest",
                        manifest,
                        "--sample-rate",
                        2_600_000,
                        "--duration",
                        0.3,
                        "--onset",
                        0.1,
                        "--offset",
                        0.2,
                        "--ramp",
                        0.01,
                        "--chirp-period",
                        0.02,
                    )
                self.assertEqual(digest(outputs[0]), digest(outputs[1]))
                self.assertEqual(outputs[0].stat().st_size, 3_120_000)
                data = np.fromfile(outputs[0], dtype="<i2").reshape(-1, 2)
                self.assertFalse(np.any(data[:260_000]))
                self.assertFalse(np.any(data[520_000:]))
                metadata = json.loads(manifests[0].read_text())
                self.assertEqual(metadata["source_type"], source_type)
                self.assertEqual(metadata["measurements"]["clipped_components"], 0)

            for source_type in ("narrowband", "wideband"):
                changed = directory / f"{source_type}-changed-seed.bin"
                command(
                    ROOT / "jammergen",
                    "--type",
                    source_type,
                    "--output",
                    changed,
                    "--duration",
                    0.3,
                    "--onset",
                    0.1,
                    "--offset",
                    0.2,
                    "--ramp",
                    0.01,
                    "--seed",
                    99,
                )
                self.assertNotEqual(
                    digest(directory / f"{source_type}-a.bin"), digest(changed)
                )

            rejected = command(
                ROOT / "jammergen",
                "--type",
                "iq-replay",
                "--output",
                directory / "bad.bin",
                "--duration",
                1,
                check=False,
            )
            self.assertNotEqual(rejected.returncode, 0)
            self.assertIn("unsupported source type", rejected.stderr)

    def test_public_workflow_seam_and_analysis_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            clean = directory / "clean.bin"
            command(
                ROOT / "jammergen",
                "--output",
                clean,
                "--sample-rate",
                2_600_000,
                "--duration",
                3,
                "--frequency=-300000",
                "--onset",
                0,
                "--offset",
                3,
                "--ramp",
                0,
            )
            dataset = directory / "dataset"
            command(
                "python",
                WORKFLOW,
                "create",
                "--profile",
                "fast",
                "--clean-input",
                clean,
                "--output-dir",
                dataset,
                "--receiver",
                "none",
                "--no-plots",
                "--skip-build",
            )
            index = json.loads((dataset / "dataset-index.json").read_text())
            self.assertTrue(all(item["pass"] for item in index["fixtures"]))
            self.assertTrue(index["offline_only"])
            gains = {
                json.loads(Path(item["manifest"]).read_text())["parameters"]["gain"]
                for item in index["fixtures"]
            }
            seeds = {
                json.loads(Path(item["manifest"]).read_text())["parameters"]["seed"]
                for item in index["fixtures"]
            }
            self.assertEqual(len(gains), 1)
            self.assertEqual(len(seeds), 1)
            self.assertTrue(
                all(
                    json.loads(Path(item["manifest"]).read_text())["measurements"][
                        "clipped_components"
                    ]
                    == 0
                    for item in index["fixtures"]
                )
            )

            for source_type in ("narrowband", "wideband", "chirp", "pulsed"):
                typed_dataset = directory / f"dataset-{source_type}"
                command(
                    "python",
                    WORKFLOW,
                    "create",
                    "--profile",
                    "fast",
                    "--jammer-type",
                    source_type,
                    "--clean-input",
                    clean,
                    "--output-dir",
                    typed_dataset,
                    "--receiver",
                    "none",
                    "--no-plots",
                    "--skip-build",
                )
                typed_index = json.loads(
                    (typed_dataset / "dataset-index.json").read_text()
                )
                self.assertEqual(typed_index["shared"]["jammer_type"], source_type)
                self.assertTrue(all(item["pass"] for item in typed_index["fixtures"]))
                shutil.rmtree(typed_dataset)

            repeated = directory / "dataset-repeated"
            command(
                "python",
                WORKFLOW,
                "create",
                "--profile",
                "fast",
                "--clean-input",
                clean,
                "--output-dir",
                repeated,
                "--receiver",
                "none",
                "--skip-build",
            )
            repeated_index = json.loads((repeated / "dataset-index.json").read_text())
            self.assertEqual(
                [item["sha256"] for item in index["fixtures"]],
                [item["sha256"] for item in repeated_index["fixtures"]],
            )
            for item in repeated_index["fixtures"]:
                report = json.loads(Path(item["waveform_report"]).read_text())
                self.assertEqual(len(report["plots"]), 4)
                self.assertTrue(all(Path(path).is_file() for path in report["plots"]))

            mirrored = directory / "mirrored.bin"
            jammer = dataset / "cw-jammer.bin"
            samples = np.fromfile(jammer, dtype="<i2").reshape(-1, 2)
            samples[:, 1] *= -1
            samples.tofile(mirrored)
            failed = command(
                "python",
                WORKFLOW,
                "analyze",
                "--profile",
                "fast",
                "--jammer",
                mirrored,
                "--jammer-manifest",
                dataset / "cw-jammer.tool.json",
                "--clean",
                dataset / "clean-source.bin",
                "--mixed",
                Path(index["fixtures"][0]["fixture"]),
                "--mix-manifest",
                dataset / "clean" / "iqmix.tool.json",
                "--output-dir",
                directory / "corrupt-analysis",
                "--no-plots",
                check=False,
            )
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn("waveform acceptance failed", failed.stderr)


if __name__ == "__main__":
    unittest.main()
