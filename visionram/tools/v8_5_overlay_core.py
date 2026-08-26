#!/usr/bin/env python3
"""Dependency-free telemetry timeline used by the V8.5 PC overlay."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Any


@dataclass(frozen=True)
class OverlayMatch:
    """Immutable decision for one decoded video frame."""

    video_pts_ms: float | None
    runtime_message: dict[str, Any] | None
    result_message: dict[str, Any] | None
    telemetry_age_ms: float | None
    result_age_at_video_ms: float | None
    telemetry_stale: bool
    result_stale: bool
    bbox_visible: bool
    reason: str


@dataclass(frozen=True)
class TimelineStats:
    accepted_datagrams: int
    rejected_out_of_order: int
    sequence_gaps: int
    retained_results: int
    evicted_results: int
    last_sequence: int | None


@dataclass(frozen=True)
class _ResultSample:
    sequence: int
    received_monotonic: float
    capture_media_pts_ms: float
    frame_id: int
    message: dict[str, Any]


def _finite_nonnegative(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    converted = float(value)
    if not math.isfinite(converted) or converted < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return converted


class TelemetryTimeline:
    """A bounded, thread-safe timeline of unique inference results.

    Runtime state always advances with the newest accepted UDP datagram. A
    repeated fixed-rate datagram does not duplicate its inference result. The
    video thread only selects a result whose capture media PTS is not newer
    than the decoded frame PTS.
    """

    def __init__(
            self, capacity: int = 512, result_stale_ms: float = 300.0,
            telemetry_stale_ms: float = 1000.0) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        for value, name in (
                (result_stale_ms, "result_stale_ms"),
                (telemetry_stale_ms, "telemetry_stale_ms")):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")
        self._capacity = capacity
        self._result_stale_ms = result_stale_ms
        self._telemetry_stale_ms = telemetry_stale_ms
        self._lock = threading.Lock()
        self._results: deque[_ResultSample] = deque(maxlen=capacity)
        self._latest_message: dict[str, Any] | None = None
        self._latest_received: float | None = None
        self._last_sequence: int | None = None
        self._last_result_key: tuple[int, float] | None = None
        self._accepted = 0
        self._out_of_order = 0
        self._sequence_gaps = 0
        self._evicted = 0

    def add(
            self, message: dict[str, Any],
            received_monotonic: float | None = None) -> bool:
        """Accept a validated wire message; reject duplicate/old sequence."""
        if not isinstance(message, dict) or \
                message.get("schema") != "visionarm.telemetry.v1":
            raise ValueError("unexpected telemetry schema")
        sequence = message.get("sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or \
                sequence <= 0:
            raise ValueError("sequence must be a positive integer")
        target = message.get("target")
        control = message.get("control")
        if not isinstance(target, dict) or not isinstance(control, dict):
            raise ValueError("target and control must be objects")
        if not isinstance(target.get("available"), bool):
            raise ValueError("target.available must be boolean")
        capture_pts = _finite_nonnegative(
            control.get("capture_media_pts_ms"),
            "control.capture_media_pts_ms")
        _finite_nonnegative(
            control.get("result_staleness_ms"),
            "control.result_staleness_ms")
        frame_id = control.get("frame_id")
        if isinstance(frame_id, bool) or not isinstance(frame_id, int) or \
                frame_id < 0:
            raise ValueError("control.frame_id must be nonnegative integer")
        received = time.monotonic() if received_monotonic is None else \
            float(received_monotonic)
        if not math.isfinite(received) or received < 0.0:
            raise ValueError("received_monotonic is invalid")

        with self._lock:
            if self._last_sequence is not None and \
                    sequence <= self._last_sequence:
                self._out_of_order += 1
                return False
            if self._last_sequence is not None and \
                    sequence > self._last_sequence + 1:
                self._sequence_gaps += sequence - self._last_sequence - 1
            self._last_sequence = sequence
            self._accepted += 1
            self._latest_message = message
            self._latest_received = received

            result_key = (frame_id, capture_pts)
            if target["available"] and result_key != self._last_result_key:
                if len(self._results) == self._capacity:
                    self._evicted += 1
                self._results.append(_ResultSample(
                    sequence=sequence,
                    received_monotonic=received,
                    capture_media_pts_ms=capture_pts,
                    frame_id=frame_id,
                    message=message))
                self._last_result_key = result_key
            return True

    def match(
            self, video_pts_ms: float | None,
            now_monotonic: float | None = None) -> OverlayMatch:
        """Match one video frame without ever selecting a future result."""
        now = time.monotonic() if now_monotonic is None else \
            float(now_monotonic)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("now_monotonic is invalid")
        valid_video_pts = video_pts_ms is not None and \
            math.isfinite(float(video_pts_ms)) and float(video_pts_ms) >= 0.0

        with self._lock:
            runtime = self._latest_message
            latest_received = self._latest_received
            results = tuple(self._results)

        telemetry_age_ms = None if latest_received is None else \
            max(0.0, (now - latest_received) * 1000.0)
        telemetry_stale = telemetry_age_ms is None or \
            telemetry_age_ms > self._telemetry_stale_ms
        if runtime is None:
            return OverlayMatch(
                None if not valid_video_pts else float(video_pts_ms),
                None, None, None, None, True, True, False,
                "NO_TELEMETRY")
        if not valid_video_pts:
            return OverlayMatch(
                None, runtime, None, telemetry_age_ms, None,
                telemetry_stale, True, False, "NO_VIDEO_PTS")

        pts = float(video_pts_ms)
        selected: _ResultSample | None = None
        for sample in reversed(results):
            if sample.capture_media_pts_ms <= pts:
                selected = sample
                break
        if selected is None:
            reason = "FUTURE_RESULT" if results else "NO_RESULT"
            return OverlayMatch(
                pts, runtime, None, telemetry_age_ms, None,
                telemetry_stale, True, False, reason)

        media_age_ms = max(0.0, pts - selected.capture_media_pts_ms)
        sent_age_ms = _finite_nonnegative(
            selected.message["control"]["result_staleness_ms"],
            "control.result_staleness_ms")
        result_stale = max(media_age_ms, sent_age_ms) > self._result_stale_ms
        target = selected.message["target"]
        control = selected.message["control"]
        bbox = target.get("bbox")
        bbox_valid = isinstance(bbox, list) and len(bbox) == 4 and all(
            isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(float(value)) for value in bbox)
        bbox_visible = bool(
            not telemetry_stale and not result_stale and bbox_valid and
            target.get("state") == "DETECTED" and target.get("valid") is True
            and control.get("valid") is True and
            target.get("source_width", 0) > 0 and
            target.get("source_height", 0) > 0)
        reason = "OK" if bbox_visible else (
            "TELEMETRY_STALE" if telemetry_stale else
            "RESULT_STALE" if result_stale else "NO_VALID_TARGET")
        return OverlayMatch(
            pts, runtime, selected.message, telemetry_age_ms, media_age_ms,
            telemetry_stale, result_stale, bbox_visible, reason)

    def stats(self) -> TimelineStats:
        with self._lock:
            return TimelineStats(
                accepted_datagrams=self._accepted,
                rejected_out_of_order=self._out_of_order,
                sequence_gaps=self._sequence_gaps,
                retained_results=len(self._results),
                evicted_results=self._evicted,
                last_sequence=self._last_sequence)
