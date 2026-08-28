#!/usr/bin/env python3
"""Validate a VisionArm runtime_report.v1 acceptance report."""

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


def _validate_queue(
        report: Mapping[str, str], prefix: str,
        errors: list[str]) -> None:
    capacity = _integer(report, f"{prefix}.capacity", errors)
    high = _integer(report, f"{prefix}.high_watermark", errors)
    current = _integer(report, f"{prefix}.current_size", errors)
    if capacity is not None and capacity <= 0:
        errors.append(f"{prefix}.capacity={capacity}, expected > 0")
    if high is not None and capacity is not None and high > capacity:
        errors.append(
            f"{prefix}.high_watermark={high} exceeds capacity={capacity}")
    if current is not None and capacity is not None and current > capacity:
        errors.append(
            f"{prefix}.current_size={current} exceeds capacity={capacity}")
    if current is not None and current != 0:
        errors.append(f"{prefix}.current_size={current}, expected drained 0")


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
    return level


def _validate_performance(
        report: Mapping[str, str], expectations: Expectations,
        errors: list[str]) -> None:
    zero_keys = [
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
        "camera_buffer_count_at_start", "rga_process_successes",
        "mpp_encoded_frames",
    ]
    required_queues = [
        "queue.captured", "queue.prepared", "queue.video", "queue.encoded",
    ]
    if report.get("topology") == "split":
        required_queues.append("queue.completed")

    audio_enabled = expectations.audio == "enabled"
    encoded_audio_enabled = audio_enabled and (
        expectations.recording == "enabled" or
        expectations.network == "enabled")
    if audio_enabled:
        zero_keys.extend((
            "audio_worker_fatal_error", "audio_queue_push_failures",
            "audio_xruns", "audio_suspends", "audio_status_errors",
            "media_audio_timestamp_failures",
        ))
        _expect_integer(report, "audio_worker_started", 1, errors)
        positive_keys.extend((
            "audio_timed_chunks", "audio_timed_frames",
        ))
        required_queues.append("queue.audio_pcm")
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
            "audio_encode_packets_pushed",
            "audio_encoded_sink_packets_written",
        ))
        required_queues.append("queue.audio_encoded")

    for key in zero_keys:
        _expect_integer(report, key, 0, errors)
    for key in positive_keys:
        _expect_positive(report, key, errors)
    for prefix in required_queues:
        _validate_queue(report, prefix, errors)
    if audio_enabled:
        _expect_integer(report, "queue.audio_pcm.replaced_oldest", 0, errors)
    if encoded_audio_enabled:
        _expect_integer(
            report, "queue.audio_encoded.replaced_oldest", 0, errors)

    rss_growth = _integer(report, "rss.growth_kb", errors)
    rss_limit = _integer(report, "rss.enforced_growth_limit_kb", errors)
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
                "network_video_access_units_written",
                "network_audio_packets_written"):
            _expect_positive(report, key, errors)

    if expectations.recording == "enabled":
        for key in (
                "local_av_finalize_ok", "local_av_header_written",
                "local_av_finalized"):
            _expect_integer(report, key, 1, errors)
        for key in ("local_av_fatal_error", "local_av_write_failures"):
            _expect_integer(report, key, 0, errors)
        for key in (
                "local_av_video_samples_written",
                "local_av_audio_packets_written"):
            _expect_positive(report, key, errors)

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
                "telemetry_datagrams_sent"):
            _expect_positive(report, key, errors)

    if expectations.control == "uart":
        for key in (
                "uart.adapter.rejected", "uart.poll_errors",
                "uart.read_errors", "uart.write_errors",
                "uart.message_decode_errors", "uart.unexpected_responses",
                "uart.parser_crc_errors", "uart.parser_length_errors",
                "uart.parser_version_errors", "uart.parser_escape_errors",
                "uart.parser_oversize_errors"):
            _expect_integer(report, key, 0, errors)
        _expect_integer(report, "uart.peer_boot_id_valid", 1, errors)
        for key in (
                "uart.hello_ack_received", "uart.status_received",
                "uart.control_sent"):
            _expect_positive(report, key, errors)
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
