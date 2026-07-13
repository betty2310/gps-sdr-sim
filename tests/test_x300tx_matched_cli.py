#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
X300TX = ROOT / "x300tx"
NAVIGATION = ROOT / "tests" / "fixtures" / "brdc0030.25n"


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
        "-P",
        "1",
        "-S",
        "1:overhead",
        "-d",
        0.04,
        "--matched-code-target-prns",
        "1",
        "--matched-code-js-db",
        10,
        "--matched-code-phase-seed",
        42,
        "--matched-code-onset",
        0.01,
        "--matched-code-offset",
        0.03,
        "--matched-code-ramp",
        0.001,
        "--manifest",
        manifest,
        "--dry-run",
    ]


class X300MatchedCliTest(unittest.TestCase):
    def test_dry_run_renders_exact_plan_without_uhd(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            manifest = Path(temporary) / "run.json"
            completed = invoke(*base_arguments(manifest))
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("UHD was never opened", completed.stderr)
            metadata = json.loads(manifest.read_text())
            self.assertEqual(metadata["status"], "dry_run")
            self.assertEqual(metadata["timing"]["start_mode"], "frozen_rinex_epoch")
            self.assertEqual(metadata["sample_contract"]["planned_samples"], 100_000)
            self.assertEqual(
                metadata["sample_contract"]["rendered_clean_samples"], 100_000
            )
            self.assertEqual(
                metadata["sample_contract"]["rendered_jammer_samples"], 100_000
            )
            self.assertEqual(
                metadata["sample_contract"]["quantized_composite_samples"],
                100_000,
            )
            self.assertEqual(metadata["sample_contract"]["sent_samples"], 0)
            self.assertEqual(metadata["measurements"]["clipped_components"], 0)
            self.assertAlmostEqual(metadata["measurements"]["achieved_js_db"], 10.0)

    def test_invalid_command_contracts_fail_before_uhd(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cases: list[tuple[list[object], str]] = []

            missing_js = base_arguments(directory / "missing-js.json")
            js_index = missing_js.index("--matched-code-js-db")
            del missing_js[js_index : js_index + 2]
            cases.append((missing_js, "requires -d"))

            duplicate = base_arguments(directory / "duplicate.json")
            duplicate[duplicate.index("--matched-code-target-prns") + 1] = "1,1"
            cases.append((duplicate, "duplicate"))

            continuous = base_arguments(directory / "continuous.json")
            continuous.extend(["-n"])
            cases.append((continuous, "finite"))

            invalid_timing = base_arguments(directory / "timing.json")
            invalid_timing[invalid_timing.index("--matched-code-offset") + 1] = 0.011
            cases.append((invalid_timing, "plateau"))

            legacy = base_arguments(directory / "legacy.json")
            legacy.extend(["-J", 5])
            cases.append((legacy, "mutually exclusive"))

            partial = base_arguments(directory / "partial.json")
            partial[partial.index("--matched-code-target-prns") + 1] = "2"
            cases.append((partial, "partial constellation"))

            unsupported_rate = base_arguments(directory / "rate.json")
            unsupported_rate.extend(["--rate", 2_500_001])
            cases.append((unsupported_rate, "integer 100 ms"))

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
            unavailable[unavailable.index("-P") + 1] = "2"
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
