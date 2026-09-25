#!/usr/bin/env python3
"""Offline integration tests: these commands never open UHD or transmit."""
import hashlib
import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "tests/fixtures/brdc0030.25n"


def invoke(*args):
    return subprocess.run([str(ROOT / "x300tx"), *map(str, args)], cwd=ROOT,
                          text=True, capture_output=True, timeout=20)


def base(manifest, nav=NAV):
    return ["-e", nav, "-l", "21.0047844,105.8460541,22", "-S", "1:overhead",
            "-P", "1", "--dry-run", "--manifest", manifest]


class HardwareTimingCliTest(unittest.TestCase):
    def test_pps_contract_fails_before_receiver_or_hardware_access(self):
        endpoint = ["--ublox-tcp", "127.0.0.1:1"]
        pps = [*endpoint, "--gps-pps", "--pps-host-utc-bound-sec", ".2"]
        cases = [
            (["--gps-pps"], "--gps-pps requires"),
            ([*endpoint, "--gps-pps"], "--gps-pps requires"),
            ([*pps, "--delivery-delay-sec", ".037"], "--gps-pps requires"),
            ([*pps, "--model-time-offset-sec", ".037"], "--gps-pps requires"),
            ([*pps, "--dry-run"], "--gps-pps requires"),
            ([*pps, "--check-time"], "--gps-pps requires"),
            ([*pps, "--time-source", "gpsdo"], "--gps-pps requires"),
            ([*pps, "--clock-source", "gpsdo"], "--gps-pps requires"),
            (["--pps-host-utc-bound-sec", ".2"], "PPS bounds require"),
            (["--pps-max-delivery-sec", ".8"], "PPS bounds require"),
            ([*pps, "--check-pps", "-e", NAV], "timing-only hardware check"),
        ]
        for value in ("0", "-1", ".251", "nan", "inf", "0.2junk"):
            cases.append((["--pps-host-utc-bound-sec", value], "invalid --pps-host"))
        for args, expected in cases:
            with self.subTest(args=args):
                result = invoke(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(expected, result.stderr)
                self.assertNotIn("Configuring USRP", result.stderr)
                self.assertNotIn("cannot connect", result.stderr)

    def test_revive_only_uses_historical_templates_without_a_current_set(self):
        # Every fixture ephemeris is at 432000: the requested epoch is over
        # two hours later, and the render crosses a navigation-frame boundary.
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            nav = directory / "historical.nav"
            fixture = NAV.read_text()
            record_start = fixture.index(" 1 25  1  3  0  0  0.0")
            second_prn = fixture[record_start:].replace(
                " 1 25  1  3  0  0  0.0", " 2 25  1  3  0  0  0.0", 1)
            nav.write_text(fixture + second_prn)
            digest = hashlib.sha256(nav.read_bytes()).hexdigest()
            for prns, synth in [("1", "1:revive"), ("1,2", "1:revive,2:revive")]:
                with self.subTest(prns=prns):
                    manifest = directory / "revive.json"
                    result = invoke("-e", nav, "-l", "0,0,0", "-P", prns,
                                    "-S", synth, "--gps-week", 2347,
                                    "--gps-tow", 439229.98, "--dry-run", "-d", .125,
                                    "--manifest", manifest)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    meta = json.loads(manifest.read_text())
                    self.assertEqual(meta["status"], "dry_run")
                    self.assertEqual(meta["sample_zero_gps_tow"], 439229.98)
                    self.assertEqual(meta["transport"]["generated_samples"], 312500)
                    self.assertEqual(meta["transport"]["accepted_samples"], 0)
                    self.assertEqual(meta["ephemeris_sha256"], digest)
                    self.assertNotEqual(meta["transport"]["generated_iq_fnv1a64"],
                                        "cbf29ce484222325")
                    self.assertIn("Revive PRN 01:", result.stderr)
                    if prns == "1,2":
                        self.assertIn("Revive PRN 02:", result.stderr)
                    self.assertIn("UHD was never opened", result.stderr)
                    self.assertNotIn("Configuring USRP", result.stderr)
            self.assertEqual(hashlib.sha256(nav.read_bytes()).hexdigest(), digest)

    def test_non_revived_outputs_still_require_a_current_set(self):
        with tempfile.TemporaryDirectory() as tmp:
            for selection in ([], ["-P", "1,2"], ["-P", "2"]):
                with self.subTest(selection=selection):
                    result = invoke("-e", NAV, "-l", "0,0,0", "-S", "1:revive",
                                    *selection, "--gps-week", 2347,
                                    "--gps-tow", 439229.98, "--dry-run",
                                    "--manifest", Path(tmp) / "mixed.json")
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("No current ephemeris set", result.stderr)
                    self.assertNotIn("Configuring USRP", result.stderr)

    def test_revive_only_still_rejects_missing_target_template(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = invoke("-e", NAV, "-l", "0,0,0", "-P", "1,2",
                            "-S", "1:revive,2:revive", "--gps-week", 2347,
                            "--gps-tow", 439229.98, "--dry-run",
                            "--manifest", Path(tmp) / "missing.json")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Revive PRN 2: no ephemeris", result.stderr)
            self.assertNotIn("No current ephemeris set", result.stderr)
            self.assertNotIn("Configuring USRP", result.stderr)

    def test_finite_samples_and_scenario_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = Path(tmp) / "run.json"
            completed = invoke(*base(manifest), "-d", "0.234567", "--rate", 2500001)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("UHD was never opened", completed.stderr)
            meta = json.loads(manifest.read_text())
            self.assertEqual(meta["status"], "dry_run")
            self.assertEqual(meta["sample_rate_hz"], 2500001)
            self.assertEqual(meta["transport"]["generated_samples"],
                             math.floor(0.234567 * 2500001 + 0.5))
            self.assertEqual(meta["transport"]["accepted_samples"], 0)
            self.assertEqual(meta["epoch_association"], "scenario_at_sample_zero")
            self.assertFalse(meta["gps_alignment_verified"])
            self.assertFalse(meta["pps_latch_verified"])
            self.assertEqual(meta["clock_source"], "external")
            self.assertEqual(meta["time_source"], "external")
            self.assertEqual(meta["ephemeris_sha256"], hashlib.sha256(NAV.read_bytes()).hexdigest())
            self.assertEqual(meta["clipped_components"], 0)

    def test_fractional_start_across_frame_and_week(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            # Keep the orbital fields unchanged; move this test ephemeris' epoch
            # to the end of its GPS week so freshness checks still apply.
            weekend = directory / "weekend.nav"
            weekend.write_text(NAV.read_text().replace(
                " 1 25  1  3  0  0  0.0", " 1 25  1  4 23 59 44.0").replace(
                "0.432000000000D+06", "0.604784000000D+06"))
            for nav, tow in [(NAV, 432029.98), (weekend, 604799.98)]:
                with self.subTest(tow=tow):
                    manifest = directory / f"{tow}.json"
                    completed = invoke(*base(manifest, nav), "--gps-week", 2347,
                                       "--gps-tow", tow, "-d", 0.125)
                    self.assertEqual(completed.returncode, 0, completed.stderr)
                    meta = json.loads(manifest.read_text())
                    self.assertEqual(meta["sample_zero_gps_tow"], tow)
                    self.assertEqual(meta["transport"]["generated_samples"], 312500)
                    self.assertEqual(meta["clipped_components"], 0)
            calendar = invoke(*base(directory / "calendar.json"), "-t", "2025/01/03,00:00:29.98")
            self.assertEqual(calendar.returncode, 0, calendar.stderr)
            self.assertEqual(json.loads((directory / "calendar.json").read_text())[
                "sample_zero_gps_tow"], 432029.98)

    def test_clipping_fails_and_marks_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = Path(tmp) / "clipped.json"
            completed = invoke(*base(manifest), "-G", 50)
            self.assertNotEqual(completed.returncode, 0)
            meta = json.loads(manifest.read_text())
            self.assertEqual(meta["status"], "failed")
            self.assertGreater(meta["clipped_components"], 0)
            self.assertIn("clipped", meta["transport"]["failure_reason"])
            self.assertEqual(meta["transport"]["accepted_samples"], 0)

    def test_retired_and_invalid_timing_fail_before_hardware(self):
        cases = [(["-n"], "does not accept -n"),
                 (["--gps-time-ppm", 1], "was removed"),
                 (["--tx-time-scale-ppm", 0], "was removed"),
                 (["--trimble-time-tag-host", "localhost"], "was removed"),
                 (["--trimble-tx-cal-ns", 0], "was removed"),
                 (["--clock-source", "internal"], "hardware sync requires"),
                 (["--time-source", "internal"], "hardware sync requires"),
                 (["--start-lead-sec", 0.001], "0.02..60"),
                 (["--start-lead-sec", "nan"], "0.02..60"),
                 (["--prebuffer", "5bad"], "prebuffer"),
                 (["--rate", 10], "1000000..200000000"),
                 (["--gps-week", 2347], "must both"),
                 (["-t", "2025/01/03,00:00:nan"], "Invalid date"),
                 (["-d", "0.1bad"], "Invalid duration"),
                 (["--gps-week", 2500, "--gps-tow", 432000], "No current ephemeris")]
        with tempfile.TemporaryDirectory() as tmp:
            for args, message in cases:
                with self.subTest(args=args):
                    completed = invoke(*base(Path(tmp) / "failure.json"), *args)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(message, completed.stderr)
                    self.assertNotIn("Configuring USRP", completed.stderr)

    def test_artifacts_cannot_overwrite_navigation(self):
        with tempfile.TemporaryDirectory() as tmp:
            nav = Path(tmp) / "input.nav"
            nav.write_bytes(NAV.read_bytes())
            digest = hashlib.sha256(nav.read_bytes()).hexdigest()
            alias = Path(tmp) / "alias.json"
            alias.symlink_to(nav)
            for output in (nav, alias):
                completed = invoke(*base(output, nav))
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("must use different paths", completed.stderr)
                self.assertEqual(hashlib.sha256(nav.read_bytes()).hexdigest(), digest)


if __name__ == "__main__":
    unittest.main()
