#!/usr/bin/env python3
"""Offline replay/negative CLI and loopback TCP tests; never open UHD."""
import hashlib
import json
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((ROOT / "tests/fixtures/f9p-lnav-prn3.json").read_text())


def ubx(cls, ident, payload):
    data = bytes((cls, ident)) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for value in data:
        a = (a + value) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + data + bytes((a, b))


def messages(index, subframes=(1, 2, 3)):
    tow = 121722 + index
    payload = struct.pack("<IihbBI", tow * 1000, -123456, 2437, 18, 3, 10)
    data = ubx(1, 0x20, payload)
    if not index:
        data = ubx(10, 4, bytes(40) + b"MOD=ZED-F9P".ljust(30, b"\x00")) + data
    pvt = bytearray(92)
    struct.pack_into("<I", pvt, 0, tow * 1000)
    pvt[20:24] = bytes((3, 1, 0, 9))
    struct.pack_into("<iiiii", pvt, 24, 1050000000, 210000000, 22000, 0, 2000)
    raw = bytearray(48)
    struct.pack_into("<dH", raw, 0, tow, 2437)
    raw[11], raw[13] = 1, 1
    struct.pack_into("<ddf", raw, 16, 21000000, 12345, -500)
    raw[37], raw[46] = 3, 15
    data += ubx(1, 7, pvt) + ubx(2, 0x15, raw)
    for sf in subframes:
        payload = bytes((0, 3, 0, 0, 10, 0, 2, 0))
        payload += struct.pack("<10I", *FIXTURE["words"][str(sf)])
        data += ubx(2, 0x13, payload)
    return data


def recording(path, subframes=(1, 2, 3)):
    offset = 0
    rows = ["offset,length,host_monotonic_s"]
    with path.open("wb") as output:
        for index in range(3):
            chunk = messages(index, subframes)
            output.write(chunk)
            rows.append(f"{offset},{len(chunk)},{100 + index}")
            offset += len(chunk)
    Path(str(path) + ".rx.csv").write_text("\n".join(rows) + "\n")


def invoke(*args):
    return subprocess.run([str(ROOT / "x300tx"), *map(str, args)], cwd=ROOT,
                          capture_output=True, text=True, timeout=15)


class UbxCliTest(unittest.TestCase):
    def test_time_reference_with_frozen_revive_navigation(self):
        nav = ROOT / "tests/fixtures/brdc0030.25n"
        digest = hashlib.sha256(nav.read_bytes()).hexdigest()
        # Revive-only needs a usable historical template, including when all
        # source epochs are outside the ordinary one-hour freshness window.
        for tow, usable in ((435538, True), (439198, True), (475198, False)):
            with self.subTest(usable=usable), tempfile.TemporaryDirectory() as directory, socket.socket() as server:
                server.bind(("127.0.0.1", 0))
                server.listen(1)
                endpoint = f"127.0.0.1:{server.getsockname()[1]}"
                finished = threading.Event()

                def sender():
                    with server.accept()[0] as client:
                        client.sendall(ubx(10, 4, bytes(40) + b"MOD=ZED-F9P".ljust(30, b"\x00")))
                        # Fixture-matched time only: deliberately no PVT/RAWX/SFRBX.
                        for index in range(3):
                            payload = struct.pack("<IihbBI", (tow + index) * 1000,
                                                  -123456, 2347, 18, 3, 10)
                            client.sendall(ubx(1, 0x20, payload))
                            if finished.wait(1):
                                return
                        finished.wait(5)

                thread = threading.Thread(target=sender, daemon=True)
                thread.start()
                manifest = Path(directory) / "revive.json"
                try:
                    result = invoke("--ublox-time-tcp", endpoint, "--dry-run",
                                    "--ublox-warmup-sec", 5, "--start-lead-sec", 2,
                                    "--delivery-delay-sec", .125,
                                    "--model-time-offset-sec", .0369307,
                                    "-e", nav, "-l", "0,0,0", "-P", "1",
                                    "-S", "1:revive", "-d", .125,
                                    "--manifest", manifest)
                finally:
                    finished.set()
                    thread.join(5)
                meta = json.loads(manifest.read_text())
                self.assertNotIn("Configuring USRP", result.stderr)
                self.assertFalse(meta["uhd_opened"])
                self.assertFalse(meta["transport"]["send_attempted"])
                self.assertFalse(meta["gps_alignment_verified"])
                self.assertIsNone(meta["hardware_start_s"])
                self.assertIsNone(meta["live_start_plan"])
                self.assertEqual(meta["navigation_source"], "frozen_rinex")
                self.assertEqual(meta["ephemeris_sha256"], digest)
                self.assertEqual(meta["receiver_initial"]["navigation"], [])
                if not usable:
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("revive target scenario is not usable",
                                  meta["transport"]["failure_reason"])
                    continue
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(meta["status"], "dry_run")
                self.assertEqual(meta["epoch_association"], "ublox_host_epoch_estimate")
                self.assertEqual(meta["sample_zero_gps_week"], 2347)
                receiver = meta["receiver_initial"]
                expected = (receiver["gps_tow"] + meta["host_epoch_monotonic_s"]
                            - receiver["time_received_monotonic_s"] + .125 + .0369307
                            + meta["time_estimate"]["arrival_correction_s"])
                self.assertAlmostEqual(meta["sample_zero_gps_tow"], expected, delta=1e-8)
                self.assertEqual(meta["model_time_offset_s"], .0369307)
                self.assertEqual(meta["model_time_offset_source"], "operator_supplied")
                self.assertEqual(meta["selected_prns"], [1])
                self.assertEqual(meta["transport"]["generated_samples"], 312500)
                self.assertEqual(meta["transport"]["accepted_samples"], 0)
                self.assertEqual(meta["clipped_components"], 0)
                self.assertNotEqual(meta["transport"]["generated_iq_fnv1a64"], "cbf29ce484222325")
                self.assertIn("UHD was never opened", result.stderr)
        self.assertEqual(hashlib.sha256(nav.read_bytes()).hexdigest(), digest)

    def test_time_only_source_rejects_incompatible_options(self):
        base = ["--ublox-time-tcp", "127.0.0.1:1", "-e", "missing.rnx",
                "-l", "21.0047844,105.8460541,22", "-P", "14", "-S", "14:revive"]
        cases = [([*base, "--dry-run", "--check-time"], "incompatible"),
                 ([*base, "--dry-run", "--ublox-replay", "missing.ubx"], "incompatible"),
                 ([*base, "--dry-run", "--stream"], "finite rendering"),
                 ([*base, "--dry-run", "--check-start"], "incompatible"),
                 ([*base, "--dry-run", "--tx-path-delay-sec", .1], "RF-path"),
                 ([*base, "--dry-run", "-t", "2026/09/22,12:00:00"], "chooses its future"),
                 ([*base, "--dry-run", "--ublox-tcp", "127.0.0.1:2"], "mutually exclusive"),
                 (["--ublox-tcp", "127.0.0.1:2", *base, "--dry-run"], "mutually exclusive"),
                 (["--ublox-time-tcp", "127.0.0.1:1", "--dry-run"], "requires frozen -e")]
        for args, message in cases:
            with self.subTest(args=args):
                result = invoke(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertNotIn("Configuring USRP", result.stderr)

    def test_model_offset_cannot_be_invalid_or_silently_ignored(self):
        for value in ('nan', 'inf', '-inf', '1.01', '-1.01', '12ms', ''):
            result = invoke('--model-time-offset-sec', value)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('invalid --model-time-offset-sec', result.stderr)
            self.assertNotIn('Configuring USRP', result.stderr)
        for mode in ([], ['--check-sync'], ['--ublox-replay', 'missing.ubx'],
                     ['--ublox-tcp', '127.0.0.1:1', '--check-time'],
                     ['--ublox-tcp', '127.0.0.1:1', '--check-ublox']):
            result = invoke(*mode, '--model-time-offset-sec', 0)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('requires a live UBX waveform', result.stderr)
            self.assertNotIn('Configuring USRP', result.stderr)

    def test_time_only_tx_stream_and_check_start_require_live_receiver(self):
        # These modes now pass the CLI gate. A missing source must still fail
        # before UHD access. The fake-radio suite exercises successful sending.
        with tempfile.TemporaryDirectory() as directory:
            for index, mode in enumerate(([], ["--stream"], ["--check-start"])):
                with self.subTest(mode=mode):
                    manifest = Path(directory) / f"failed{index}.json"
                    result = invoke("--ublox-time-tcp", "127.0.0.1:1",
                                    "-e", "tests/fixtures/brdc0030.25n",
                                    "-l", "0,0,0", "-P", "1", "-S", "1:revive",
                                    "--tx-path-delay-sec", .001,
                                    "--manifest", manifest, *mode)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("cannot connect", result.stderr)
                    self.assertNotIn("Configuring USRP", result.stderr)
                    meta = json.loads(manifest.read_text())
                    self.assertEqual(meta["status"], "receiver_error")
                    self.assertFalse(meta["uhd_opened"])
                    self.assertFalse(meta["transport"]["send_attempted"])

    def test_gps_now_without_navigation(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            endpoint = f"127.0.0.1:{server.getsockname()[1]}"
            finished = threading.Event()

            def sender():
                with server.accept()[0] as client:
                    client.sendall(ubx(10, 4, bytes(40) + b"MOD=ZED-F9P".ljust(30, b"\x00")))
                    for index in range(3):
                        payload = struct.pack("<IihbBI", (121722 + index) * 1000,
                                              -123456, 2437, 18, 3, 10)
                        client.sendall(ubx(1, 0x20, payload))
                        if finished.wait(1):
                            return
                    finished.wait(5)

            thread = threading.Thread(target=sender, daemon=True)
            thread.start()
            manifest = Path(directory) / "time.json"
            try:
                result = invoke("--check-time", "--ublox-tcp", endpoint,
                                "--ublox-warmup-sec", 5, "--delivery-delay-sec", .125,
                                "--manifest", manifest)
            finally:
                finished.set()
                thread.join(5)
            self.assertEqual(result.returncode, 0, result.stderr)
            meta = json.loads(result.stdout)
            self.assertEqual(meta, json.loads(manifest.read_text()))
            self.assertEqual(meta["status"], "estimated")
            self.assertFalse(meta["receiver"]["position_valid"])
            self.assertEqual(meta["receiver"]["navigation"], [])
            self.assertEqual(meta["gps_now"]["week"], 2437)
            expected = (meta["receiver"]["gps_tow"] + meta["host_monotonic_s"]
                        - meta["receiver"]["time_received_monotonic_s"] + .125
                        + meta["time_estimate"]["arrival_correction_s"])
            self.assertEqual(meta["time_estimate"]["method"], "minimum_delay_window")
            self.assertEqual(meta["time_estimate"]["observations"], 3)
            self.assertAlmostEqual(meta["gps_now"]["tow"], expected, delta=1e-8)
            self.assertIsNone(meta["absolute_gps_uncertainty_s"])
            self.assertFalse(meta["gps_alignment_verified"])
            self.assertFalse(meta["hardware_edge_association_verified"])
            self.assertFalse(meta["uhd_opened"])
            self.assertFalse(meta["send_attempted"])
            self.assertEqual(meta["generated_samples"], 0)
            self.assertIn("UHD was never opened", result.stderr)
            self.assertNotIn("Configuring USRP", result.stderr)

    def test_gps_now_failure_reports_no_epoch(self):
        for disconnect in (False, True):
            with self.subTest(disconnect=disconnect), tempfile.TemporaryDirectory() as directory, socket.socket() as server:
                server.bind(("127.0.0.1", 0))
                server.listen(1)
                endpoint = f"127.0.0.1:{server.getsockname()[1]}"
                finished = threading.Event()

                def sender():
                    with server.accept()[0] as client:
                        if not disconnect:
                            client.sendall(ubx(10, 4, bytes(40) + b"MOD=ZED-F9P".ljust(30, b"\x00")))
                            client.sendall(ubx(1, 0x20, struct.pack(
                                "<IihbBI", 121722000, 0, 2437, 18, 0, 10)))
                            finished.wait(5)

                thread = threading.Thread(target=sender, daemon=True)
                thread.start()
                manifest = Path(directory) / "time.json"
                try:
                    result = invoke("--check-time", "--ublox-tcp", endpoint,
                                    "--ublox-warmup-sec", 1, "--manifest", manifest)
                finally:
                    finished.set()
                    thread.join(5)
                self.assertNotEqual(result.returncode, 0)
                meta = json.loads(manifest.read_text())
                self.assertEqual(meta["status"], "unavailable")
                self.assertIsNone(meta["gps_now"])
                self.assertIn("disconnected" if disconnect else "consecutive valid",
                              meta["failure_reason"])
                self.assertFalse(meta["uhd_opened"])
                self.assertFalse(meta["send_attempted"])
                self.assertNotIn("Configuring USRP", result.stderr)

    def test_gps_now_rejects_replay_and_waveform_options(self):
        cases = [(["--check-time"], "requires live"),
                 (["--check-time", "--ublox-replay", "missing.ubx"], "requires live")]
        for options in (["--stream"], ["-S", "14:revive"], ["-e", "missing.rnx"],
                        ["--dry-run"], ["--check-start"], ["--check-ublox"],
                        ["--check-sync"], ["-t", "2026/09/22,12:00:00"],
                        ["--gps-week", "2437"], ["-d", "10"]):
            cases.append((["--check-time", "--ublox-tcp", "127.0.0.1:1", *options],
                          "receiver-only"))
        for args, message in cases:
            with self.subTest(args=args):
                result = invoke(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertNotIn("Configuring USRP", result.stderr)

    def test_replay_probe_and_render(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.ubx"
            recording(path)
            output = invoke("--check-ublox", "--ublox-replay", path,
                            "--manifest", Path(directory) / "probe.json")
            self.assertEqual(output.returncode, 0, output.stderr)
            state = json.loads(output.stdout)
            self.assertTrue(state["replay"])
            self.assertEqual(state["navigation"][0]["iode"], 42)
            self.assertIsNone(state["leap_seconds"])
            for index in range(2):
                manifest = Path(directory) / f"render{index}.json"
                output = invoke("--ublox-replay", path, "--dry-run", "-P", 3,
                                "--gps-week", 2437, "--gps-tow", 121729.98,
                                "-d", 0.125, "--manifest", manifest)
                self.assertEqual(output.returncode, 0, output.stderr)
                meta = json.loads(manifest.read_text())
                self.assertEqual(meta["transport"]["generated_samples"], 312500)
                self.assertEqual(meta["transport"]["accepted_samples"], 0)
                self.assertNotEqual(meta["transport"]["generated_iq_fnv1a64"], "cbf29ce484222325")
                self.assertFalse(meta["gps_alignment_verified"])
                self.assertEqual(meta["epoch_association"], "scenario_at_sample_zero")
                self.assertEqual(meta["navigation_source"], "ublox_gps_l1_sfrbx")
                self.assertIn("UHD was never opened", output.stderr)
                if index:
                    self.assertEqual(meta["transport"], previous["transport"])
                previous = meta

    def test_no_live_replay_and_migration(self):
        cases = [(["--ublox-replay", "missing.ubx"], "never fresh live time"),
                 (["--check-start"], "require --ublox"),
                 (["--check-ublox"], "require --ublox"),
                 (["--ublox-tcp", "localhost:1", "--dry-run"], "chooses its future"),
                 (["--ublox-tcp", "localhost:1", "--gps-week", 2437], "chooses its future"),
                 (["--ublox-tcp", "localhost:1", "--check-sync"], "incompatible"),
                 (["--ublox-tcp", "localhost:1", "--ublox-replay", "x"], "incompatible"),
                 (["--trimble-rtcm-host", "localhost"], "use --ublox-tcp"),
                 (["--delivery-delay-sec", "nan"], "invalid"),
                 (["--ublox-warmup-sec", "301"], "invalid")]
        for args, message in cases:
            with self.subTest(args=args):
                result = invoke(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertNotIn("Configuring USRP", result.stderr)

    def test_missing_subframe_cannot_arm(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "incomplete.ubx"
            recording(path, subframes=(2, 3))
            manifest = Path(directory) / "failure.json"
            result = invoke("--ublox-replay", path, "--dry-run", "-P", 3,
                            "--gps-week", 2437, "--gps-tow", 121724,
                            "--manifest", manifest)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("missing SF1", result.stderr)
            self.assertNotIn("Configuring USRP", result.stderr)
            meta = json.loads(manifest.read_text())
            self.assertEqual(meta["status"], "receiver_error")
            self.assertFalse(meta["transport"]["send_attempted"])

    def test_input_artifacts_cannot_alias(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.ubx"
            recording(path)
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            alias = Path(directory) / "alias.json"
            alias.symlink_to(path)
            result = invoke("--check-ublox", "--ublox-replay", path,
                            "--manifest", alias)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("different paths", result.stderr)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_truncated_arrival_log_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.ubx"
            recording(path)
            path.write_bytes(path.read_bytes()[:-5])
            result = invoke("--check-ublox", "--ublox-replay", path,
                            "--manifest", Path(directory) / "probe.json")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("truncated UBX", result.stderr)

    def test_tcp_fragmentation_disconnect_and_recording(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            endpoint = f"127.0.0.1:{server.getsockname()[1]}"

            def sender():
                with server.accept()[0] as client:
                    for index in range(3):
                        data = messages(index)
                        client.sendall(data[:3])
                        time.sleep(.005)
                        client.sendall(data[3:])
                        time.sleep(.995)

            thread = threading.Thread(target=sender, daemon=True)
            thread.start()
            path = Path(directory) / "tcp.ubx"
            result = invoke("--check-ublox", "--ublox-tcp", endpoint, "-d", 4,
                            "--ublox-record", path,
                            "--manifest", Path(directory) / "probe.json")
            thread.join(5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("disconnected", result.stderr)
            self.assertNotIn("Configuring USRP", result.stderr)
            replay = invoke("--check-ublox", "--ublox-replay", path,
                            "--manifest", Path(directory) / "replay.json")
            self.assertEqual(replay.returncode, 0, replay.stderr)
            self.assertEqual(json.loads(replay.stdout)["consecutive_times"], 3)


if __name__ == "__main__":
    unittest.main()
