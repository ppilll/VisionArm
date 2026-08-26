#!/usr/bin/env python3
"""Pure-Python tests for the V8.5 media-PTS overlay matcher."""

from __future__ import annotations

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from v8_5_overlay_core import TelemetryTimeline  # noqa: E402


def message(
        sequence: int, capture_pts_ms: float, frame_id: int,
        *, state: str = "DETECTED", result_staleness_ms: float = 10.0):
    valid = state == "DETECTED"
    return {
        "schema": "visionarm.telemetry.v1",
        "sequence": sequence,
        "camera": {"fps": 30.0},
        "inference": {"fps": 10.0},
        "network": {"state": "running"},
        "recording": {"state": "running"},
        "target": {
            "available": True,
            "state": state,
            "valid": valid,
            "confidence": 0.9,
            "bbox": [100.0, 50.0, 300.0, 250.0],
            "source_width": 1920,
            "source_height": 1080,
        },
        "control": {
            "valid": valid,
            "capture_media_pts_ms": capture_pts_ms,
            "result_staleness_ms": result_staleness_ms,
            "capture_to_result_ms": 25.0,
            "frame_id": frame_id,
            "dx_px": 1.0,
            "dy_px": -2.0,
        },
    }


class TelemetryTimelineTest(unittest.TestCase):
    def test_exact_and_previous_pts_match(self):
        timeline = TelemetryTimeline()
        self.assertTrue(timeline.add(message(1, 100.0, 10), 1.0))
        self.assertTrue(timeline.add(message(2, 200.0, 20), 1.1))
        exact = timeline.match(200.0, 1.2)
        self.assertEqual(exact.result_message["control"]["frame_id"], 20)
        self.assertTrue(exact.bbox_visible)
        previous = timeline.match(199.999, 1.2)
        self.assertEqual(previous.result_message["control"]["frame_id"], 10)

    def test_future_result_is_never_applied(self):
        timeline = TelemetryTimeline()
        timeline.add(message(1, 500.0, 50), 1.0)
        match = timeline.match(499.999, 1.1)
        self.assertIsNone(match.result_message)
        self.assertEqual(match.reason, "FUTURE_RESULT")
        self.assertFalse(match.bbox_visible)

    def test_result_staleness_suppresses_bbox(self):
        timeline = TelemetryTimeline(result_stale_ms=300.0)
        timeline.add(message(1, 100.0, 10), 1.0)
        match = timeline.match(401.0, 1.1)
        self.assertTrue(match.result_stale)
        self.assertEqual(match.reason, "RESULT_STALE")
        self.assertFalse(match.bbox_visible)

    def test_sender_reported_staleness_suppresses_bbox(self):
        timeline = TelemetryTimeline(result_stale_ms=300.0)
        timeline.add(message(
            1, 100.0, 10, result_staleness_ms=301.0), 1.0)
        self.assertFalse(timeline.match(100.0, 1.1).bbox_visible)

    def test_telemetry_staleness_suppresses_bbox(self):
        timeline = TelemetryTimeline(telemetry_stale_ms=1000.0)
        timeline.add(message(1, 100.0, 10), 1.0)
        match = timeline.match(100.0, 2.001)
        self.assertTrue(match.telemetry_stale)
        self.assertEqual(match.reason, "TELEMETRY_STALE")
        self.assertFalse(match.bbox_visible)

    def test_capacity_and_repeated_result_are_bounded(self):
        timeline = TelemetryTimeline(capacity=2)
        timeline.add(message(1, 100.0, 10), 1.0)
        timeline.add(message(2, 100.0, 10), 1.1)
        timeline.add(message(3, 200.0, 20), 1.2)
        timeline.add(message(4, 300.0, 30), 1.3)
        stats = timeline.stats()
        self.assertEqual(stats.accepted_datagrams, 4)
        self.assertEqual(stats.retained_results, 2)
        self.assertEqual(stats.evicted_results, 1)

    def test_out_of_order_rejected_and_gap_counted(self):
        timeline = TelemetryTimeline()
        self.assertTrue(timeline.add(message(10, 100.0, 10), 1.0))
        self.assertFalse(timeline.add(message(10, 100.0, 10), 1.1))
        self.assertTrue(timeline.add(message(13, 200.0, 20), 1.2))
        stats = timeline.stats()
        self.assertEqual(stats.rejected_out_of_order, 1)
        self.assertEqual(stats.sequence_gaps, 2)


if __name__ == "__main__":
    unittest.main()
