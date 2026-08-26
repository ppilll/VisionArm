#!/usr/bin/env python3
"""Pure-Python tests for the ffplay/telemetry lightweight viewer."""

from __future__ import annotations

import json
import pathlib
from types import SimpleNamespace
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from view_v8_5_overlay import (  # noqa: E402
    TelemetryAccumulator,
    build_ffplay_command,
    build_video_url,
)


def message(
        sequence: int, captured_frames: int, *, camera_fps: float = 30.0,
        inference_fps: float = 29.0) -> dict:
    return {
        "schema": "visionarm.telemetry.v1",
        "sequence": sequence,
        "sent_monotonic_ns": sequence * 100_000_000,
        "pipeline": {"running": True, "fatal_error": False},
        "camera": {
            "fps": camera_fps,
            "captured_frames": captured_frames,
        },
        "inference": {
            "fps": inference_fps,
            "results": captured_frames - 1,
        },
        "video": {"frames_encoded": captured_frames},
        "network": {"state": "running"},
        "recording": {"state": "running"},
        "target": {
            "available": True,
            "valid": True,
            "state": "DETECTED",
            "confidence": 0.9,
            "bbox": [10.0, 20.0, 110.0, 120.0],
            "source_width": 1920,
            "source_height": 1080,
        },
        "control": {
            "valid": True,
            "frame_id": captured_frames - 1,
            "capture_monotonic_ns": sequence * 100_000_000 - 80_000_000,
            "generated_monotonic_ns": sequence * 100_000_000,
            "capture_to_result_ms": 80.0,
            "capture_media_pts_ms": sequence * 100.0,
            "result_staleness_ms": 20.0,
        },
    }


def payload(value: dict) -> bytes:
    return json.dumps(value, separators=(",", ":")).encode("utf-8")


class TelemetryAccumulatorTest(unittest.TestCase):
    def test_healthy_stream_produces_aggregate_summary(self) -> None:
        accumulator = TelemetryAccumulator()
        accumulator.add_payload(
            payload(message(1, 10, camera_fps=28.0)), 1.0)
        accumulator.add_payload(
            payload(message(2, 20, camera_fps=30.0)), 1.1)
        summary = accumulator.snapshot()
        self.assertEqual(summary["wire_validation"], "PASS")
        self.assertEqual(summary["valid_datagrams"], 2)
        self.assertEqual(summary["first_sequence"], 1)
        self.assertEqual(summary["last_sequence"], 2)
        self.assertAlmostEqual(summary["camera_fps"]["mean"], 29.0)
        self.assertEqual(summary["target_states"], {"DETECTED": 2})
        self.assertEqual(summary["latest"]["sequence"], 2)

    def test_wire_faults_are_visible_and_fail_validation(self) -> None:
        accumulator = TelemetryAccumulator()
        accumulator.add_payload(payload(message(1, 10)), 1.0)
        accumulator.add_payload(payload(message(3, 30)), 1.1)
        accumulator.add_payload(payload(message(2, 20)), 1.2)
        accumulator.add_payload(b"not-json", 1.3)
        accumulator.add_payload(b"{}", 1.4)
        summary = accumulator.snapshot()
        self.assertEqual(summary["wire_validation"], "FAIL")
        self.assertEqual(summary["sequence_gaps"], 1)
        self.assertGreater(summary["out_of_order"], 0)
        self.assertEqual(summary["counter_regressions"], 1)
        self.assertEqual(summary["parse_errors"], 1)
        self.assertEqual(summary["schema_errors"], 1)


class FfplayCommandTest(unittest.TestCase):
    def test_command_matches_verified_v8_4_low_latency_path(self) -> None:
        args = SimpleNamespace(
            bind_address="0.0.0.0",
            video_port=5000,
            ffplay_fifo_size=65536,
            video_receive_buffer_bytes=4194304,
            probesize=5000000,
            analyzeduration=5000000,
        )
        url = build_video_url(args)
        self.assertEqual(
            url,
            "udp://0.0.0.0:5000?fifo_size=65536&overrun_nonfatal=0"
            "&buffer_size=4194304")
        command = build_ffplay_command(args, "/usr/bin/ffplay")
        self.assertEqual(command[0], "/usr/bin/ffplay")
        self.assertIn("nobuffer", command)
        self.assertIn("low_delay", command)
        self.assertIn("-framedrop", command)
        self.assertEqual(command[-1], url)


if __name__ == "__main__":
    unittest.main()
