#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BLADETX = ROOT / "bladetx"
MATCHEDGEN = ROOT / "matchedgen"
NAVIGATION = ROOT / "tests" / "fixtures" / "brdc0030.25n"
SAMPLE_RATE_HZ = 2_600_000
DRY_RUN_SECONDS = 0.1


def invoke(*arguments: object) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(BLADETX), *(str(argument) for argument in arguments)],
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
        "--rate",
        SAMPLE_RATE_HZ,
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


class BladeMatchedCliTest(unittest.TestCase):
    def test_dry_run_is_jammer_only_and_reproducible(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            manifest = directory / "run.json"
            completed = invoke(*base_arguments(manifest))
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("bladeRF was never opened", completed.stderr)

            metadata = json.loads(manifest.read_text())
            samples = metadata["sample_contract"]
            self.assertEqual(
                metadata["schema"], "gps-sdr-sim.bladetx-matched-code.v2"
            )
            self.assertEqual(metadata["status"], "dry_run")
            self.assertEqual(
                metadata["rf_output"],
                {
                    "contains": "matched_code_interference_only",
                    "clean_gps_transmitted": False,
                },
            )
            self.assertEqual(metadata["scenario"]["selected_target_prns"], [1, 3])
            self.assertTrue(
                metadata["scenario"]["startup_target_allocation_passed"]
            )
            self.assertEqual(samples["device_format"], "SC16_Q11_META")
            self.assertEqual(
                samples["dry_run_validation_samples"],
                int(SAMPLE_RATE_HZ * DRY_RUN_SECONDS),
            )
            self.assertEqual(
                samples["internal_alignment_samples"],
                samples["dry_run_validation_samples"],
            )
            self.assertEqual(
                samples["rendered_jammer_samples"],
                samples["dry_run_validation_samples"],
            )
            self.assertEqual(samples["sent_jammer_samples"], 0)
            self.assertEqual(metadata["measurements"]["clipped_components"], 0)

            trajectory = directory / "run.trajectory.csv"
            expected_iq = directory / "expected.bin"
            expected = subprocess.run(
                [
                    str(MATCHEDGEN),
                    "--output",
                    str(expected_iq),
                    "--trajectory",
                    str(trajectory),
                    "--target-prns",
                    "1,3",
                    "--sample-rate",
                    str(SAMPLE_RATE_HZ),
                    "--duration",
                    str(DRY_RUN_SECONDS),
                    "--onset",
                    "0",
                    "--offset",
                    str(DRY_RUN_SECONDS),
                    "--ramp",
                    "0",
                    "--amplitude",
                    "0.5",
                    "--phase-seed",
                    "42",
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

    def test_invalid_and_live_unsafe_contracts_fail_before_device_open(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cases: list[tuple[list[object], str]] = []

            finite = base_arguments(directory / "finite.json")
            finite.extend(["-d", 1])
            cases.append((finite, "does not accept -d"))

            partial = base_arguments(directory / "partial.json")
            partial.extend(["-P", 1])
            cases.append((partial, "rejects -P"))

            duplicate = base_arguments(directory / "duplicate.json")
            duplicate[duplicate.index("--matched-code-target-prns") + 1] = "1,1"
            cases.append((duplicate, "duplicate"))

            unsupported_rate = base_arguments(directory / "rate.json")
            unsupported_rate[unsupported_rate.index("--rate") + 1] = 2_600_001
            cases.append((unsupported_rate, "integer 100 ms"))

            live = base_arguments(directory / "live.json")
            live.remove("--dry-run")
            cases.append((live, "live matched-code TX requires"))

            no_gps_time_source = base_arguments(directory / "no-time-source.json")
            no_gps_time_source.remove("--dry-run")
            no_gps_time_source.extend(
                [
                    "--device",
                    "*:serial=test-only",
                    "--txvga1",
                    -35,
                    "--txvga2",
                    0,
                    "--calibration-id",
                    "test-only",
                    "--confirm-controlled-rf",
                ]
            )
            cases.append((no_gps_time_source, "requires the Trimble time-tag path"))

            for arguments, expected in cases:
                with self.subTest(expected=expected):
                    completed = invoke(*arguments)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(expected, completed.stderr)
                    self.assertNotIn("Opening bladeRF device", completed.stderr)

    def test_unavailable_target_and_unwritable_manifest_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            unavailable = base_arguments(directory / "unavailable.json")
            unavailable[unavailable.index("--matched-code-target-prns") + 1] = "2"
            completed = invoke(*unavailable)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("target PRN 2 is not", completed.stderr)
            failure = json.loads((directory / "unavailable.json").read_text())
            self.assertEqual(failure["status"], "preflight_error")

            unwritable = base_arguments(directory / "unused.json")
            unwritable[unwritable.index("--manifest") + 1] = "/dev/null/run.json"
            completed = invoke(*unwritable)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("manifest path is not writable", completed.stderr)
            self.assertNotIn("Opening bladeRF device", completed.stderr)


if __name__ == "__main__":
    unittest.main()
