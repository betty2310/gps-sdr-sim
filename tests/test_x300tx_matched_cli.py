#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
X300TX = ROOT / "x300tx"
MATCHEDGEN = ROOT / "matchedgen"
NAVIGATION = ROOT / "tests" / "fixtures" / "brdc0030.25n"
DRY_RUN_SECONDS = 0.1
SAMPLE_RATE_HZ = 2_500_000


def invoke(*arguments: object) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(X300TX), *(str(argument) for argument in arguments)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def base_arguments(manifest: Path) -> list[object]:
    return [
        "-e",
        NAVIGATION,
        "-l",
        "21.0047844,105.8460541,22",
        "-S",
        "1:overhead,3:90/45,8:180/45",
        "--matched-code-target-prns",
        "1,3",
        "--matched-code-amplitude",
        0.5,
        "--matched-code-phase-seed",
        42,
        "--manifest",
        manifest,
        "--dry-run",
    ]


def fnv1a64(data: bytes) -> str:
    value = 0xCBF29CE484222325
    for byte in data:
        value ^= byte
        value = (value * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


class X300MatchedCliTest(unittest.TestCase):
    def test_dry_run_renders_continuous_jammer_only_validation_window(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            manifest = directory / "run.json"
            completed = invoke(*base_arguments(manifest))
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("UHD was never opened", completed.stderr)
            metadata = json.loads(manifest.read_text())
            samples = metadata["sample_contract"]

            self.assertEqual(metadata["schema"], "gps-sdr-sim.x300tx-matched-code.v2")
            self.assertEqual(metadata["status"], "dry_run")
            self.assertEqual(metadata["timing"]["start_mode"], "frozen_rinex_epoch")
            self.assertEqual(
                metadata["rf_output"],
                {
                    "contains": "matched_code_interference_only",
                    "clean_gps_transmitted": False,
                },
            )
            self.assertEqual(metadata["scenario"]["requested_target_prns"], [1, 3])
            self.assertEqual(metadata["scenario"]["selected_target_prns"], [1, 3])
            self.assertTrue(metadata["scenario"]["startup_target_allocation_passed"])
            self.assertTrue(samples["continuous"])
            self.assertIsNone(samples["planned_samples"])
            self.assertEqual(
                samples["dry_run_validation_samples"],
                int(DRY_RUN_SECONDS * SAMPLE_RATE_HZ),
            )
            self.assertEqual(
                samples["internal_alignment_samples"],
                samples["dry_run_validation_samples"],
            )
            self.assertEqual(
                samples["rendered_jammer_samples"],
                samples["dry_run_validation_samples"],
            )
            self.assertEqual(
                samples["quantized_jammer_samples"],
                samples["dry_run_validation_samples"],
            )
            self.assertEqual(samples["sent_jammer_samples"], 0)
            self.assertNotIn("rendered_clean_samples", samples)
            self.assertNotIn("quantized_composite_samples", samples)
            self.assertEqual(metadata["waveform"]["output_amplitude_full_scale"], 0.5)
            self.assertAlmostEqual(
                metadata["waveform"]["equal_component_weight"],
                1 / math.sqrt(2),
            )
            self.assertAlmostEqual(
                metadata["waveform"]["predicted_peak_full_scale"],
                0.5 * math.sqrt(2),
            )
            self.assertGreater(metadata["waveform"]["predicted_headroom_db"], 1)
            self.assertNotIn("requested_js_db", metadata["waveform"])
            self.assertNotIn("achieved_js_db", metadata["measurements"])
            self.assertEqual(metadata["measurements"]["clipped_components"], 0)
            self.assertGreater(metadata["measurements"]["jammer_rms_full_scale"], 0)
            self.assertLessEqual(
                metadata["measurements"]["jammer_peak_full_scale"],
                metadata["waveform"]["predicted_peak_full_scale"] + 1e-9,
            )

            trajectory = directory / "run.trajectory.csv"
            trajectory_prns = {
                int(line.split(",")[1])
                for line in trajectory.read_text().splitlines()
                if line and not line.startswith("#") and not line.startswith("sample_")
            }
            self.assertEqual(trajectory_prns, {1, 3})

            expected_iq = directory / "expected.bin"
            expected = subprocess.run(
                [
                    str(argument)
                    for argument in [
                        MATCHEDGEN,
                        "--output",
                        expected_iq,
                        "--trajectory",
                        trajectory,
                        "--target-prns",
                        "1,3",
                        "--sample-rate",
                        SAMPLE_RATE_HZ,
                        "--duration",
                        DRY_RUN_SECONDS,
                        "--onset",
                        0,
                        "--offset",
                        DRY_RUN_SECONDS,
                        "--ramp",
                        0,
                        "--amplitude",
                        0.5,
                        "--phase-seed",
                        42,
                    ]
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(expected.returncode, 0, expected.stderr)
            self.assertEqual(
                metadata["waveform"]["jammer_iq_fnv1a64"],
                fnv1a64(expected_iq.read_bytes()),
            )

    def test_invalid_command_contracts_fail_before_uhd(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cases: list[tuple[list[object], str]] = []

            missing_amplitude = base_arguments(directory / "missing-amplitude.json")
            amplitude_index = missing_amplitude.index("--matched-code-amplitude")
            del missing_amplitude[amplitude_index : amplitude_index + 2]
            cases.append((missing_amplitude, "amplitude"))

            duplicate = base_arguments(directory / "duplicate.json")
            duplicate[duplicate.index("--matched-code-target-prns") + 1] = "1,1"
            cases.append((duplicate, "duplicate"))

            finite = base_arguments(directory / "finite.json")
            finite.extend(["-d", 1])
            cases.append((finite, "does not accept -d"))

            stream_now = base_arguments(directory / "stream-now.json")
            stream_now.extend(["-n"])
            cases.append((stream_now, "does not accept -n"))

            partial = base_arguments(directory / "partial.json")
            partial.extend(["-P", 1])
            cases.append((partial, "rejects -P"))

            legacy_timing = base_arguments(directory / "timing.json")
            legacy_timing.extend(["--matched-code-onset", 0])
            cases.append((legacy_timing, "Unknown option --matched-code-onset"))

            legacy_js = base_arguments(directory / "js.json")
            legacy_js.extend(["--matched-code-js-db", 10])
            cases.append((legacy_js, "Unknown option --matched-code-js-db"))

            legacy = base_arguments(directory / "legacy.json")
            legacy.extend(["-J", 5])
            cases.append((legacy, "mutually exclusive"))

            unsupported_rate = base_arguments(directory / "rate.json")
            unsupported_rate.extend(["--rate", 2_500_001])
            cases.append((unsupported_rate, "integer 100 ms"))

            invalid_amplitude = base_arguments(directory / "amplitude.json")
            invalid_amplitude[
                invalid_amplitude.index("--matched-code-amplitude") + 1
            ] = 0
            cases.append((invalid_amplitude, "in (0, 1]"))

            invalid_gain = base_arguments(directory / "gain.json")
            invalid_gain.extend(["--gain", "not-a-number"])
            cases.append((invalid_gain, "--gain must be a finite"))

            collision_path = directory / "collision.json"
            artifact_collision = base_arguments(collision_path)
            artifact_collision.extend(["--trajectory", collision_path])
            cases.append((artifact_collision, "must use different paths"))

            live_without_safety = base_arguments(directory / "live.json")
            live_without_safety.remove("--dry-run")
            cases.append((live_without_safety, "live matched-code TX requires"))

            for arguments, expected in cases:
                with self.subTest(expected=expected):
                    completed = invoke(*arguments)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(expected, completed.stderr)
                    self.assertNotIn("Configuring USRP", completed.stderr)

    def test_unavailable_target_and_unwritable_manifest_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            unavailable = base_arguments(directory / "unavailable.json")
            unavailable[unavailable.index("--matched-code-target-prns") + 1] = "2"
            completed = invoke(*unavailable)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("target PRN 2 is not", completed.stderr)
            self.assertIn("UHD discovery/open/configuration is intentionally skipped", completed.stderr)
            failure = json.loads((directory / "unavailable.json").read_text())
            self.assertEqual(failure["status"], "preflight_error")
            self.assertIn("target PRN 2", failure["failure_reason"])

            unwritable = base_arguments(directory / "unused.json")
            unwritable[unwritable.index("--manifest") + 1] = "/dev/null/run.json"
            completed = invoke(*unwritable)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("manifest path is not writable", completed.stderr)
            self.assertNotIn("Configuring USRP", completed.stderr)


if __name__ == "__main__":
    unittest.main()
