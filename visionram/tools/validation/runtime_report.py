#!/usr/bin/env python3
"""Validate a VisionArm runtime report acceptance record."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import pathlib
import re
import sys
from typing import Mapping


SCHEMA = "visionarm.runtime_report.v1"
REPORT_LEVELS = ("summary", "performance", "diagnostic")
_KEY_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


@dataclass(frozen=True)
class Expectations:
    minimum_duration_seconds: float = 600.0
    audio: str = "enabled"
    network: str = "enabled"
    recording: str = "enabled"
    telemetry: str = "enabled"
    control: str = "uart"


def _unescape_value(value: str, line_number: int) -> str:
    decoded: list[str] = []
    index = 0
    while index < len(value):
        character = value[index]
        if character != "\\":
            decoded.append(character)
            index += 1
            continue
        index += 1
        if index >= len(value):
            raise ValueError(
                f"line {line_number}: trailing backslash in value")
        escape = value[index]
        if escape in ("\\", "="):
            decoded.append(escape)
        elif escape == "n":
            decoded.append("\n")
        elif escape == "r":
            decoded.append("\r")
        elif escape == "t":
            decoded.append("\t")
        elif escape == "x":
            digits = value[index + 1:index + 3]
            if len(digits) != 2 or any(
                    digit not in "0123456789abcdefABCDEF" for digit in digits):
                raise ValueError(
                    f"line {line_number}: invalid hexadecimal escape")
            decoded.append(chr(int(digits, 16)))
            index += 2
        else:
            raise ValueError(
                f"line {line_number}: unknown escape \\{escape}")
        index += 1
    return "".join(decoded)


def parse_report_text(text: str) -> dict[str, str]:
    raw_values: dict[str, tuple[str, int]] = {}
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        if not raw_line or raw_line.startswith("#"):
            continue
        if "=" not in raw_line:
            raise ValueError(f"line {line_number}: expected key=value")
        key, value = raw_line.split("=", 1)
        key = key.strip()
        if not key or not _KEY_PATTERN.fullmatch(key):
            raise ValueError(f"line {line_number}: invalid key {key!r}")
        if key in raw_values:
            raise ValueError(f"line {line_number}: duplicate key {key}")
        raw_values[key] = (value, line_number)
    if not raw_values:
        raise ValueError("report is empty")

    encoding = raw_values.get("value_encoding", ("", 0))[0]
    if encoding == "backslash-v1":
        return {
            key: _unescape_value(value, line_number)
            for key, (value, line_number) in raw_values.items()
        }
    return {key: value for key, (value, _) in raw_values.items()}


def _integer(
        report: Mapping[str, str], key: str, errors: list[str]) -> int | None:
    value = report.get(key)
    if value is None:
        errors.append(f"missing {key}")
        return None
    try:
        return int(value, 10)
    except ValueError:
        errors.append(f"{key} is not an integer: {value!r}")
        return None


def _number(
        report: Mapping[str, str], key: str,
        errors: list[str]) -> float | None:
    value = report.get(key)
    if value is None:
        errors.append(f"missing {key}")
        return None
    try:
        converted = float(value)
    except ValueError:
        errors.append(f"{key} is not numeric: {value!r}")
        return None
    if not math.isfinite(converted):
        errors.append(f"{key} is not finite: {value!r}")
        return None
    return converted


def _expect_text(
        report: Mapping[str, str], key: str, expected: str,
        errors: list[str]) -> None:
    value = report.get(key)
    if value is None:
        errors.append(f"missing {key}")
    elif value != expected:
        errors.append(f"{key}={value!r}, expected {expected!r}")


def _expect_integer(
        report: Mapping[str, str], key: str, expected: int,
        errors: list[str]) -> None:
    value = _integer(report, key, errors)
    if value is not None and value != expected:
        errors.append(f"{key}={value}, expected {expected}")


def _expect_positive(
        report: Mapping[str, str], key: str, errors: list[str]) -> None:
    value = _integer(report, key, errors)
    if value is not None and value <= 0:
        errors.append(f"{key}={value}, expected > 0")


def _expect_equal(
        report: Mapping[str, str], left_key: str, right_key: str,
        errors: list[str]) -> None:
    left = _integer(report, left_key, errors)
    right = _integer(report, right_key, errors)
    if left is not None and right is not None and left != right:
        errors.append(
            f"{left_key}={left}, expected equal to {right_key}={right}")


def _expect_sum(
        report: Mapping[str, str], total_key: str,
        part_keys: tuple[str, ...], errors: list[str]) -> None:
    total = _integer(report, total_key, errors)
    parts = [_integer(report, key, errors) for key in part_keys]
    if total is not None and all(value is not None for value in parts):
        part_total = sum(value for value in parts if value is not None)
        if total != part_total:
            errors.append(
                f"{total_key}={total}, expected sum of {part_keys}="
                f"{part_total}")


def _validate_queue(
        report: Mapping[str, str], prefix: str,
        errors: list[str]) -> None:
    capacity = _integer(report, f"{prefix}.capacity", errors)
    high = _integer(report, f"{prefix}.high_watermark", errors)
    current = _integer(report, f"{prefix}.current_size", errors)
    pushed = _integer(report, f"{prefix}.pushed", errors)
    popped = _integer(report, f"{prefix}.popped", errors)
    replaced = _integer(report, f"{prefix}.replaced_oldest", errors)
    stopped = _integer(report, f"{prefix}.stopped", errors)
    if capacity is not None and capacity <= 0:
        errors.append(f"{prefix}.capacity={capacity}, expected > 0")
    for suffix, value in (("high_watermark", high), ("current_size", current),
                          ("pushed", pushed), ("popped", popped),
                          ("replaced_oldest", replaced)):
        if value is not None and value < 0:
            errors.append(f"{prefix}.{suffix}={value}, expected >= 0")
    if high is not None and capacity is not None and high > capacity:
        errors.append(
            f"{prefix}.high_watermark={high} exceeds capacity={capacity}")
    if current is not None and capacity is not None and current > capacity:
        errors.append(
            f"{prefix}.current_size={current} exceeds capacity={capacity}")
    if current is not None and current != 0:
        errors.append(f"{prefix}.current_size={current}, expected drained 0")
    if stopped is not None and stopped != 1:
        errors.append(f"{prefix}.stopped={stopped}, expected 1")
    if replaced is not None and replaced != 0:
        errors.append(
            f"{prefix}.replaced_oldest={replaced}, expected no replacement")
    if pushed is not None and popped is not None and replaced is not None and \
            pushed != popped + replaced:
        errors.append(
            f"{prefix}.pushed={pushed}, expected popped+replaced_oldest="
            f"{popped + replaced}")


def _validate_queue_summary(
        report: Mapping[str, str], prefix: str, errors: list[str]) -> None:
    current = _integer(report, f"{prefix}.current_size", errors)
    replaced = _integer(report, f"{prefix}.replaced_oldest", errors)
    stopped = _integer(report, f"{prefix}.stopped", errors)
    if current is not None and current != 0:
        errors.append(f"{prefix}.current_size={current}, expected drained 0")
    if replaced is not None and replaced != 0:
        errors.append(
            f"{prefix}.replaced_oldest={replaced}, expected no replacement")
    if stopped is not None and stopped != 1:
        errors.append(f"{prefix}.stopped={stopped}, expected 1")


def _validate_latency(
        report: Mapping[str, str], prefix: str,
        errors: list[str], expected_samples_key: str) -> None:
    total = _integer(report, f"{prefix}.total_samples", errors)
    retained = _integer(report, f"{prefix}.retained_samples", errors)
    truncated = _integer(report, f"{prefix}.truncated", errors)
    expected = _integer(report, expected_samples_key, errors)
    if total is not None and total <= 0:
        errors.append(f"{prefix}.total_samples={total}, expected > 0")
    if retained is not None and retained <= 0:
        errors.append(f"{prefix}.retained_samples={retained}, expected > 0")
    if total is not None and retained is not None and retained > total:
        errors.append(
            f"{prefix}.retained_samples={retained} exceeds total={total}")
    if total is not None and retained is not None and truncated == 0 and \
            retained != total:
        errors.append(
            f"{prefix}.retained_samples={retained}, expected total_samples="
            f"{total} when truncated=0")
    if total is not None and expected is not None and total != expected:
        errors.append(
            f"{prefix}.total_samples={total}, expected "
            f"{expected_samples_key}={expected}")
    if truncated is not None and truncated != 0:
        errors.append(f"{prefix}.truncated={truncated}, expected 0")

    values = {
        suffix: _number(report, f"{prefix}.{suffix}_ms", errors)
        for suffix in ("mean", "p50", "p95", "p99", "maximum")
    }
    for suffix, value in values.items():
        if value is not None and value < 0.0:
            errors.append(f"{prefix}.{suffix}_ms={value}, expected >= 0")
    ordered = [values[name] for name in ("p50", "p95", "p99", "maximum")]
    if all(value is not None for value in ordered):
        concrete = [value for value in ordered if value is not None]
        if concrete != sorted(concrete):
            errors.append(
                f"{prefix} percentiles are not ordered p50<=p95<=p99<=maximum")
    mean = values["mean"]
    maximum = values["maximum"]
    if mean is not None and maximum is not None and mean > maximum:
        errors.append(
            f"{prefix}.mean_ms={mean} exceeds maximum_ms={maximum}")


def _validate_rate(
        report: Mapping[str, str], rate_key: str, count_key: str,
        observed_seconds: float | None, errors: list[str]) -> None:
    rate = _number(report, rate_key, errors)
    count = _integer(report, count_key, errors)
    if rate is not None and rate <= 0.0:
        errors.append(f"{rate_key}={rate}, expected > 0")
    if rate is None or count is None or observed_seconds is None or \
            observed_seconds <= 0.0:
        return
    expected = count / observed_seconds
    tolerance = max(0.05, expected * 0.005)
    if abs(rate - expected) > tolerance:
        errors.append(
            f"{rate_key}={rate:.3f}, expected {count_key}/"
            f"observed_duration_seconds={expected:.3f} +/- {tolerance:.3f}")


def _module_enabled(
        report: Mapping[str, str], module: str,
        expected: bool, errors: list[str]) -> None:
    _expect_integer(report, f"module.{module}.enabled", int(expected), errors)


def _validate_common(
        report: Mapping[str, str], expectations: Expectations,
        errors: list[str]) -> str | None:
    _expect_text(report, "schema", SCHEMA, errors)
    _expect_text(report, "schema_version", "1", errors)
    _expect_text(report, "value_encoding", "backslash-v1", errors)
    level = report.get("report_level")
    if level is None:
        errors.append("missing report_level")
    elif level not in REPORT_LEVELS:
        errors.append(
            f"report_level={level!r}, expected one of {REPORT_LEVELS!r}")

    _expect_text(report, "result", "PASS", errors)
    _expect_integer(report, "completed_requested_duration", 1, errors)
    _expect_integer(report, "terminated_by_signal", 0, errors)
    requested = _number(report, "requested_duration_seconds", errors)
    observed = _number(report, "observed_duration_seconds", errors)
    minimum = expectations.minimum_duration_seconds
    if requested is not None and requested < minimum:
        errors.append(
            f"requested_duration_seconds={requested:.3f}, expected >= "
            f"{minimum:.3f}")
    if observed is not None and observed < minimum:
        errors.append(
            f"observed_duration_seconds={observed:.3f}, expected >= "
            f"{minimum:.3f}")

    for key in (
            "auxiliary_media_runtime_fault", "fatal_error",
            "camera_outstanding_before_stop",
            "broker_outstanding_frames_before_camera_stop",
            "broker_outstanding_leases_before_camera_stop",
            "broker_outstanding_frames_after_stop",
            "broker_outstanding_leases_after_stop",
            "log.dropped_critical", "log.sink_failures"):
        _expect_integer(report, key, 0, errors)
    for key in (
            "graceful_shutdown_completed",
            "split_final_completed_frame_drained", "control_ok",
            "log.flush_ok"):
        _expect_integer(report, key, 1, errors)
    for key in (
            "captured_frames", "inference_successes",
            "postprocess_successes", "video_frames_encoded",
            "h265_bytes_written"):
        _expect_positive(report, key, errors)

    log_capacity = _integer(report, "log.queue_capacity", errors)
    log_critical_capacity = _integer(
        report, "log.critical_queue_capacity", errors)
    log_current = _integer(report, "log.current_size", errors)
    log_high = _integer(report, "log.high_watermark", errors)
    log_accepted = _integer(report, "log.accepted", errors)
    log_emitted = _integer(report, "log.emitted", errors)
    log_dropped = _integer(report, "log.dropped", errors)
    log_contention = _integer(report, "log.dropped_contention", errors)
    log_overflow = _integer(report, "log.dropped_overflow", errors)
    log_critical = _integer(report, "log.dropped_critical", errors)
    if log_capacity is not None and log_capacity <= 0:
        errors.append("log.queue_capacity must be > 0")
    if log_critical_capacity is not None and log_critical_capacity <= 0:
        errors.append("log.critical_queue_capacity must be > 0")
    for key, value in (
            ("log.current_size", log_current),
            ("log.high_watermark", log_high),
            ("log.accepted", log_accepted),
            ("log.emitted", log_emitted),
            ("log.dropped", log_dropped),
            ("log.dropped_contention", log_contention),
            ("log.dropped_overflow", log_overflow),
            ("log.dropped_critical", log_critical)):
        if value is not None and value < 0:
            errors.append(f"{key}={value}, expected >= 0")
    if log_current is not None and log_current != 0:
        errors.append(f"log.current_size={log_current}, expected drained 0")
    if log_high is not None and log_capacity is not None and \
            log_critical_capacity is not None and \
            log_high > log_capacity + log_critical_capacity:
        errors.append(
            f"log.high_watermark={log_high} exceeds combined capacity="
            f"{log_capacity + log_critical_capacity}")
    if log_accepted is not None and log_emitted is not None and \
            log_accepted != log_emitted:
        errors.append(
            f"log.emitted={log_emitted}, expected accepted={log_accepted}")
    if log_dropped is not None and log_contention is not None and \
            log_overflow is not None and \
            log_dropped != log_contention + log_overflow:
        errors.append(
            f"log.dropped={log_dropped}, expected contention+overflow="
            f"{log_contention + log_overflow}")
    if log_critical is not None and log_dropped is not None and \
            log_critical > log_dropped:
        errors.append(
            f"log.dropped_critical={log_critical} exceeds dropped="
            f"{log_dropped}")

    _module_enabled(report, "camera", True, errors)
    _module_enabled(report, "inference", True, errors)
    _module_enabled(report, "video", True, errors)
    audio_enabled = expectations.audio == "enabled"
    encoded_audio_enabled = audio_enabled and (
        expectations.recording == "enabled" or
        expectations.network == "enabled")
    _module_enabled(report, "audio", audio_enabled, errors)
    _module_enabled(report, "audio_encoder", encoded_audio_enabled, errors)
    _module_enabled(report, "recorder",
                    expectations.recording == "enabled", errors)
    _module_enabled(report, "network",
                    expectations.network == "enabled", errors)
    _module_enabled(report, "telemetry",
                    expectations.telemetry == "enabled", errors)
    _module_enabled(report, "uart", expectations.control == "uart", errors)
    _expect_text(report, "control_backend", expectations.control, errors)

    if audio_enabled:
        _expect_integer(report, "audio_path_ok", 1, errors)
    if encoded_audio_enabled:
        _expect_integer(report, "audio_encode_path_ok", 1, errors)
    if expectations.recording == "enabled":
        _expect_integer(report, "local_av_mux_ok", 1, errors)
    if expectations.network == "enabled":
        _expect_integer(report, "network_mux_ok", 1, errors)
    if expectations.telemetry == "enabled":
        _expect_integer(report, "telemetry_ok", 1, errors)
    if level == "summary":
        required_queues = [
            "queue.captured", "queue.prepared", "queue.video",
            "queue.encoded",
        ]
        topology = report.get("topology")
        if topology == "split_npu_postprocess":
            required_queues.append("queue.completed")
        elif topology == "fused_npu_postprocess":
            unexpected = sorted(
                key for key in report if key.startswith("queue.completed."))
            if unexpected:
                errors.append(
                    "fused_npu_postprocess must omit queue.completed fields: "
                    + ", ".join(unexpected))
        else:
            errors.append(
                f"topology={topology!r}, expected fused_npu_postprocess or "
                "split_npu_postprocess")
        if audio_enabled:
            required_queues.append("queue.audio_pcm")
        if encoded_audio_enabled:
            required_queues.append("queue.audio_encoded")
        if expectations.network == "enabled":
            required_queues.append("queue.network")
        for prefix in required_queues:
            _validate_queue_summary(report, prefix, errors)
    return level


def _validate_performance(
        report: Mapping[str, str], expectations: Expectations,
        errors: list[str]) -> None:
    zero_keys = [
        "camera_timeouts", "driver_dropped_frames",
        "replaced_waiting_frames", "skipped_no_input_slot",
        "preprocess_failures", "inference_failures",
        "postprocess_failures", "result_publish_failures",
        "requeue_failures", "dmabuf_sync_failures",
        "video_frames_dropped", "video_branch_failed",
        "video_encode_failures", "video_packets_dropped",
        "video_sink_failures", "mpp_encode_failures",
        "mpp_source_reimports", "h265_write_failures",
        "state_control_sink_failures", "state_perception_sink_failures",
        "state_invalid_timestamp_packets",
    ]
    positive_keys = [
        "camera_buffer_count_at_start", "rga_process_calls",
        "rga_process_successes", "mpp_encoded_frames",
        "state_processed_packets", "h265_bytes_written",
    ]
    required_queues = [
        "queue.captured", "queue.prepared", "queue.video", "queue.encoded",
    ]
    topology = report.get("topology")
    if topology == "split_npu_postprocess":
        required_queues.append("queue.completed")
    elif topology == "fused_npu_postprocess":
        unexpected = sorted(
            key for key in report if key.startswith("queue.completed."))
        if unexpected:
            errors.append(
                "fused_npu_postprocess must omit queue.completed fields: " +
                ", ".join(unexpected))
    else:
        errors.append(
            f"topology={topology!r}, expected fused_npu_postprocess or "
            "split_npu_postprocess")

    observed = _number(report, "observed_duration_seconds", errors)
    for rate_key, count_key in (
            ("camera_fps", "captured_frames"),
            ("inference_fps", "inference_successes"),
            ("postprocess_fps", "postprocess_successes"),
            ("video_fps", "video_frames_encoded")):
        _validate_rate(report, rate_key, count_key, observed, errors)

    for left_key, right_key in (
            ("captured_frames", "inference_successes"),
            ("inference_successes", "postprocess_successes"),
            ("captured_frames", "video_frames_encoded"),
            ("rga_process_calls", "rga_process_successes"),
            ("rga_process_successes", "inference_successes"),
            ("mpp_encoded_frames", "video_frames_encoded"),
            ("state_processed_packets", "postprocess_successes")):
        _expect_equal(report, left_key, right_key, errors)

    for latency_name, expected_samples_key in (
            ("input_slot_wait", "inference_successes"),
            ("latest_frame_queue_wait", "inference_successes"),
            ("capture_to_preprocess_start", "inference_successes"),
            ("preprocess", "inference_successes"),
            ("rknn_input_submit", "inference_successes"),
            ("rknn_output_bind", "inference_successes"),
            ("rknn_bind_total", "inference_successes"),
            ("rknn_run", "inference_successes"),
            ("rknn_output_get", "inference_successes"),
            ("rknn_output_release", "inference_successes"),
            ("rknn_total", "inference_successes"),
            ("postprocess", "postprocess_successes"),
            ("capture_to_result", "postprocess_successes"),
            ("result_age", "postprocess_successes")):
        _validate_latency(
            report, f"latency.{latency_name}", errors,
            expected_samples_key)

    audio_enabled = expectations.audio == "enabled"
    encoded_audio_enabled = audio_enabled and (
        expectations.recording == "enabled" or
        expectations.network == "enabled")
    if audio_enabled:
        zero_keys.extend((
            "audio_worker_fatal_error", "audio_queue_push_failures",
            "audio_xruns", "audio_suspends", "audio_recoveries",
            "audio_short_reads", "audio_status_errors",
            "audio_timeline_discontinuities",
            "audio_timeline_pts_regressions",
            "audio_timeline_continuous_pts_mismatches",
            "audio_timeline_max_forward_gap_ns",
            "media_audio_discontinuities",
            "media_audio_timestamp_failures",
        ))
        _expect_integer(report, "audio_worker_started", 1, errors)
        positive_keys.extend((
            "audio_rate_hz", "audio_channels", "audio_timed_chunks",
            "audio_timed_frames", "audio_timed_bytes",
            "audio_timeline_chunks", "audio_timeline_frames",
            "audio_timeline_bytes", "audio_timeline_reanchors",
            "media_audio_chunks_stamped", "media_audio_reanchors",
        ))
        required_queues.append("queue.audio_pcm")
        _validate_rate(
            report, "audio_observed_rate_hz", "audio_timed_frames",
            observed, errors)
        observed_rate = _number(
            report, "audio_observed_rate_hz", errors) \
            if "audio_observed_rate_hz" in report else None
        configured_rate = _number(report, "audio_rate_hz", errors)
        if observed_rate is not None and configured_rate is not None:
            rate_tolerance = max(1.0, configured_rate * 0.02)
            if abs(observed_rate - configured_rate) > rate_tolerance:
                errors.append(
                    f"audio_observed_rate_hz={observed_rate:.3f}, expected "
                    f"within 2% of audio_rate_hz={configured_rate:.3f}")
        for left_key, right_key in (
                ("audio_timeline_chunks", "audio_timed_chunks"),
                ("audio_timeline_frames", "audio_timed_frames"),
                ("audio_timeline_bytes", "audio_timed_bytes"),
                ("media_audio_reanchors", "audio_timeline_reanchors")):
            _expect_equal(report, left_key, right_key, errors)
        _expect_integer(report, "media_audio_anchor_valid", 1, errors)
        stamped_chunks = _integer(
            report, "media_audio_chunks_stamped", errors)
        timed_chunks = _integer(report, "audio_timed_chunks", errors)
        if stamped_chunks is not None and timed_chunks is not None and \
                stamped_chunks not in (timed_chunks, timed_chunks + 1):
            errors.append(
                f"media_audio_chunks_stamped={stamped_chunks}, expected "
                f"audio_timed_chunks={timed_chunks} or one shutdown "
                "in-flight chunk")
        audio_frames = _integer(report, "audio_timed_frames", errors)
        audio_channels = _integer(report, "audio_channels", errors)
        audio_bytes = _integer(report, "audio_timed_bytes", errors)
        if audio_frames is not None and audio_channels is not None and \
                audio_bytes is not None and \
                audio_bytes != audio_frames * audio_channels * 2:
            errors.append(
                f"audio_timed_bytes={audio_bytes}, expected S16 frames*"
                f"channels*2={audio_frames * audio_channels * 2}")
        for first_key, last_key in (
                ("audio_timeline_first_pts_ns",
                 "audio_timeline_last_pts_ns"),
                ("audio_timeline_last_pts_ns",
                 "audio_timeline_last_end_pts_ns"),
                ("media_audio_last_pts_ns",
                 "media_audio_last_end_pts_ns")):
            first = _number(report, first_key, errors)
            last = _number(report, last_key, errors)
            if first is not None and last is not None and first >= last:
                errors.append(
                    f"{first_key}={first}, expected < {last_key}={last}")
    if encoded_audio_enabled:
        zero_keys.extend((
            "audio_encode_worker_fatal_error",
            "audio_encode_queue_push_failures", "audio_encoder_failures",
            "audio_encoder_buffered_input_frames",
            "audio_encoded_sink_fatal_error",
            "audio_encoded_sink_failures",
        ))
        for key in (
                "audio_encode_worker_started", "audio_encoder_drained",
                "audio_encoded_sink_started"):
            _expect_integer(report, key, 1, errors)
        positive_keys.extend((
            "audio_encode_chunks_consumed", "audio_encode_packets_pushed",
            "audio_encode_bytes_pushed", "audio_encoder_input_frames",
            "audio_encoder_submitted_codec_frames",
            "audio_encoder_emitted_packets", "audio_encoder_emitted_bytes",
            "audio_encoded_sink_packets_written",
            "audio_encoded_sink_bytes_written",
        ))
        required_queues.append("queue.audio_encoded")
        for left_key, right_key in (
                ("audio_encode_chunks_consumed", "audio_timed_chunks"),
                ("audio_encoder_input_frames", "audio_timed_frames"),
                ("audio_encoder_emitted_packets",
                 "audio_encode_packets_pushed"),
                ("audio_encoder_emitted_bytes", "audio_encode_bytes_pushed"),
                ("audio_encoded_sink_packets_written",
                 "audio_encode_packets_pushed"),
                ("audio_encoded_sink_bytes_written",
                 "audio_encode_bytes_pushed")):
            _expect_equal(report, left_key, right_key, errors)
        padding = _integer(report, "audio_encoder_padding_packets", errors)
        if padding is not None and padding < 0:
            errors.append(
                f"audio_encoder_padding_packets={padding}, expected >= 0")

    for key in zero_keys:
        _expect_integer(report, key, 0, errors)
    for key in positive_keys:
        _expect_positive(report, key, errors)
    for prefix in required_queues:
        _validate_queue(report, prefix, errors)

    rss_samples = _integer(report, "rss.samples", errors)
    rss_first = _integer(report, "rss.first_kb", errors)
    rss_last = _integer(report, "rss.last_kb", errors)
    rss_minimum = _integer(report, "rss.minimum_kb", errors)
    rss_maximum = _integer(report, "rss.maximum_kb", errors)
    rss_growth = _integer(report, "rss.growth_kb", errors)
    rss_limit = _integer(report, "rss.enforced_growth_limit_kb", errors)
    if rss_samples is not None and rss_samples <= 0:
        errors.append(f"rss.samples={rss_samples}, expected > 0")
    rss_values = [rss_first, rss_last, rss_minimum, rss_maximum]
    if any(value is not None and value <= 0 for value in rss_values):
        errors.append("rss first/last/minimum/maximum must all be > 0")
    if rss_first is not None and rss_last is not None and \
            rss_growth is not None and rss_growth != rss_last - rss_first:
        errors.append(
            f"rss.growth_kb={rss_growth}, expected last-first="
            f"{rss_last - rss_first}")
    if rss_minimum is not None and rss_first is not None and \
            rss_last is not None and \
            rss_minimum > min(rss_first, rss_last):
        errors.append("rss.minimum_kb exceeds first/last RSS")
    if rss_maximum is not None and rss_first is not None and \
            rss_last is not None and \
            rss_maximum < max(rss_first, rss_last):
        errors.append("rss.maximum_kb is below first/last RSS")
    if rss_limit is not None and rss_limit <= 0:
        errors.append("rss.enforced_growth_limit_kb must be > 0 for acceptance")
    if rss_growth is not None and rss_limit is not None and \
            rss_limit > 0 and rss_growth > rss_limit:
        errors.append(
            f"rss.growth_kb={rss_growth} exceeds enforced limit={rss_limit}")

    if expectations.network == "enabled":
        _validate_queue(report, "queue.network", errors)
        for key in (
                "network_stop_ok", "network_started", "network_finalized"):
            _expect_integer(report, key, 1, errors)
        for key in (
                "network_fatal_error", "network_queue_overload_failures",
                "network_write_failures", "queue.network.replaced_oldest"):
            _expect_integer(report, key, 0, errors)
        for key in (
                "network_video_fragments_received",
                "network_video_access_units_enqueued",
                "network_video_access_units_written",
                "network_video_bytes_written",
                "network_audio_packets_enqueued",
                "network_audio_packets_written",
                "network_audio_bytes_written"):
            _expect_positive(report, key, errors)
        for left_key, right_key in (
                ("network_video_fragments_received", "video_frames_encoded"),
                ("network_video_access_units_enqueued",
                 "video_frames_encoded"),
                ("network_video_access_units_written",
                 "network_video_access_units_enqueued"),
                ("network_audio_packets_enqueued",
                 "audio_encoded_sink_packets_written"),
                ("network_audio_packets_written",
                 "network_audio_packets_enqueued"),
                ("network_audio_bytes_written",
                 "audio_encoded_sink_bytes_written")):
            _expect_equal(report, left_key, right_key, errors)
        _expect_sum(
            report, "queue.network.pushed",
            ("network_video_access_units_enqueued",
             "network_audio_packets_enqueued"), errors)
        for first_key, last_key in (
                ("network_first_video_pts_us",
                 "network_last_video_pts_us"),
                ("network_first_audio_pts_ns",
                 "network_last_audio_pts_ns")):
            first = _number(report, first_key, errors)
            last = _number(report, last_key, errors)
            if first is not None and last is not None and first >= last:
                errors.append(
                    f"{first_key}={first}, expected < {last_key}={last}")

    if expectations.recording == "enabled":
        for key in (
                "local_av_finalize_ok", "local_av_header_written",
                "local_av_finalized"):
            _expect_integer(report, key, 1, errors)
        for key in ("local_av_fatal_error", "local_av_write_failures"):
            _expect_integer(report, key, 0, errors)
        for key in (
                "local_av_video_fragments_received",
                "local_av_video_samples_written",
                "local_av_video_bytes_written",
                "local_av_audio_packets_written",
                "local_av_audio_bytes_written"):
            _expect_positive(report, key, errors)
        for left_key, right_key in (
                ("local_av_video_fragments_received", "video_frames_encoded"),
                ("local_av_video_samples_written", "video_frames_encoded"),
                ("local_av_audio_packets_written",
                 "audio_encoded_sink_packets_written"),
                ("local_av_audio_bytes_written",
                 "audio_encoded_sink_bytes_written")):
            _expect_equal(report, left_key, right_key, errors)
        for first_key, last_key in (
                ("local_av_first_video_pts_us",
                 "local_av_last_video_pts_us"),
                ("local_av_first_audio_pts_ns",
                 "local_av_last_audio_pts_ns")):
            first = _number(report, first_key, errors)
            last = _number(report, last_key, errors)
            if first is not None and last is not None and first >= last:
                errors.append(
                    f"{first_key}={first}, expected < {last_key}={last}")

    if expectations.telemetry == "enabled":
        for key in (
                "telemetry_start_ok", "telemetry_final_update_ok",
                "telemetry_stop_ok", "telemetry_started",
                "telemetry_stopped_cleanly"):
            _expect_integer(report, key, 1, errors)
        for key in (
                "telemetry_runtime_fault", "telemetry_fatal_error",
                "telemetry_send_failures", "telemetry_serialization_failures",
                "telemetry_oversized_datagrams"):
            _expect_integer(report, key, 0, errors)
        for key in (
                "telemetry_control_updates", "telemetry_runtime_updates",
                "telemetry_datagrams_attempted", "telemetry_datagrams_sent",
                "telemetry_bytes_sent", "telemetry_final_sequence"):
            _expect_positive(report, key, errors)
        _expect_equal(
            report, "telemetry_datagrams_attempted",
            "telemetry_datagrams_sent", errors)
        _expect_equal(
            report, "telemetry_final_sequence",
            "telemetry_datagrams_attempted", errors)
        control_updates = _integer(report, "telemetry_control_updates", errors)
        processed_packets = _integer(report, "state_processed_packets", errors)
        if control_updates is not None and processed_packets is not None and \
                control_updates > processed_packets:
            errors.append(
                f"telemetry_control_updates={control_updates} exceeds "
                f"state_processed_packets={processed_packets}")

    if expectations.control == "uart":
        for key in (
                "uart.adapter.rejected",
                "uart.adapter.nonfinite_invalidations",
                "uart.adapter.invalid_timestamp_invalidations",
                "uart.adapter.invalid_state_invalidations",
                "uart.adapter.identity_truncations", "uart.poll_errors",
                "uart.read_errors", "uart.write_errors",
                "uart.message_decode_errors", "uart.unexpected_responses",
                "uart.parser_crc_errors", "uart.parser_length_errors",
                "uart.parser_version_errors", "uart.parser_escape_errors",
                "uart.parser_oversize_errors"):
            _expect_integer(report, key, 0, errors)
        _expect_integer(report, "uart.peer_boot_id_valid", 1, errors)
        for key in (
                "uart.tx_bytes", "uart.rx_bytes", "uart.hello_ack_received",
                "uart.status_received", "uart.control_accepted",
                "uart.control_sent"):
            _expect_positive(report, key, errors)
        for left_key, right_key in (
                ("uart.adapter.submissions", "state_processed_packets"),
                ("uart.adapter.accepted", "uart.adapter.submissions"),
                ("uart.control_accepted", "uart.adapter.accepted")):
            _expect_equal(report, left_key, right_key, errors)
        _expect_equal(
            report, "uart.adapter.valid_inputs", "state_valid_controls",
            errors)
        _expect_sum(
            report, "uart.adapter.accepted",
            ("uart.adapter.valid_inputs", "uart.adapter.invalid_inputs"),
            errors)
        _expect_sum(
            report, "uart.control_accepted",
            ("uart.control_overwritten", "uart.control_sent"), errors)
        _expect_sum(
            report, "uart.control_sent",
            ("uart.valid_control_sent", "uart.invalid_control_sent"), errors)
        link_state = report.get("uart.link_state_before_stop")
        if link_state is None:
            errors.append("missing uart.link_state_before_stop")
        elif link_state not in ("READY", "DEGRADED"):
            errors.append(
                f"uart.link_state_before_stop={link_state!r}, expected "
                "READY or DEGRADED")


def validate_report(
        report: Mapping[str, str], expectations: Expectations) -> list[str]:
    if not math.isfinite(expectations.minimum_duration_seconds) or \
            expectations.minimum_duration_seconds <= 0.0:
        return ["minimum duration must be finite and positive"]
    errors: list[str] = []
    level = _validate_common(report, expectations, errors)
    if level in ("performance", "diagnostic"):
        _validate_performance(report, expectations, errors)
    return errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate VisionArm runtime_report.v1 acceptance report")
    parser.add_argument("report", type=pathlib.Path)
    parser.add_argument(
        "--minimum-duration-seconds", type=float, default=600.0)
    parser.add_argument(
        "--expect-audio", choices=("enabled", "disabled"),
        default="enabled")
    parser.add_argument(
        "--expect-network", choices=("enabled", "disabled"),
        default="enabled")
    parser.add_argument(
        "--expect-recording", choices=("enabled", "disabled"),
        default="enabled")
    parser.add_argument(
        "--expect-telemetry", choices=("enabled", "disabled"),
        default="enabled")
    parser.add_argument(
        "--expect-control", choices=("uart", "mock"), default="uart")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = parse_report_text(args.report.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        print(f"report_error={error}", file=sys.stderr)
        print("runtime_report_validation=FAIL")
        return 2
    expectations = Expectations(
        minimum_duration_seconds=args.minimum_duration_seconds,
        audio=args.expect_audio,
        network=args.expect_network,
        recording=args.expect_recording,
        telemetry=args.expect_telemetry,
        control=args.expect_control)
    errors = validate_report(report, expectations)
    for error in errors:
        print(f"validation_error={error}")
    print(f"validation_error_count={len(errors)}")
    print("runtime_report_validation=" + ("PASS" if not errors else "FAIL"))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
