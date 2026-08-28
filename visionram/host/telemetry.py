#!/usr/bin/env python3
"""Receive and validate VisionArm JSON UDP telemetry."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import socket
import sys
import time
from typing import Any


VALID_OUTPUT_STATES = {"disabled", "starting", "running", "finalized", "fatal"}
VALID_TARGET_STATES = {
    "NO_TARGET", "CANDIDATE", "DETECTED", "LOST", "INVALID", "STALE"
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and validate VisionArm side telemetry")
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("duration_seconds", type=float)
    parser.add_argument("port", type=int, nargs="?", default=5001)
    parser.add_argument("--bind", default="0.0.0.0", dest="bind_address")
    parser.add_argument("--start-timeout-seconds", type=float, default=30.0)
    parser.add_argument("--receive-buffer-bytes", type=int, default=1024 * 1024)
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--stale-result-ms", type=float, default=300.0)
    parser.add_argument(
        "--expect-network", choices=("any", "enabled", "disabled"),
        default="any")
    parser.add_argument(
        "--expect-recording", choices=("any", "enabled", "disabled"),
        default="any")
    parser.add_argument("--report", type=pathlib.Path)
    args = parser.parse_args()
    if args.duration_seconds <= 0 or args.start_timeout_seconds <= 0:
        parser.error("durations must be positive")
    if not math.isfinite(args.stale_result_ms) or \
            args.stale_result_ms < 0.0:
        parser.error("stale result threshold must be finite and nonnegative")
    if not 1 <= args.port <= 65535:
        parser.error("port must be in [1, 65535]")
    if args.receive_buffer_bytes <= 0 or args.print_every <= 0:
        parser.error("buffer size and print interval must be positive")
    return args


def nested(message: dict[str, Any], key: str) -> dict[str, Any]:
    value = message.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be an object")
    return value


def finite_number(value: Any, name: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    converted = float(value)
    if not math.isfinite(converted) or (nonnegative and converted < 0.0):
        raise ValueError(f"{name} is invalid")
    return converted


def validate_message(message: Any) -> dict[str, Any]:
    if not isinstance(message, dict):
        raise ValueError("root must be an object")
    if message.get("schema") != "visionarm.telemetry.v1":
        raise ValueError("unexpected schema")
    sequence = message.get("sequence")
    sent_ns = message.get("sent_monotonic_ns")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= 0:
        raise ValueError("sequence must be a positive integer")
    if isinstance(sent_ns, bool) or not isinstance(sent_ns, int) or sent_ns <= 0:
        raise ValueError("sent_monotonic_ns must be positive")

    pipeline = nested(message, "pipeline")
    camera = nested(message, "camera")
    inference = nested(message, "inference")
    video = nested(message, "video")
    network = nested(message, "network")
    recording = nested(message, "recording")
    target = nested(message, "target")
    control = nested(message, "control")
    if network.get("state") not in VALID_OUTPUT_STATES:
        raise ValueError("invalid network state")
    if recording.get("state") not in VALID_OUTPUT_STATES:
        raise ValueError("invalid recording state")
    if target.get("state") not in VALID_TARGET_STATES:
        raise ValueError("invalid target state")
    if not isinstance(pipeline.get("running"), bool):
        raise ValueError("pipeline.running must be boolean")
    if not isinstance(pipeline.get("fatal_error"), bool):
        raise ValueError("pipeline.fatal_error must be boolean")
    if not isinstance(target.get("available"), bool) or \
            not isinstance(target.get("valid"), bool) or \
            not isinstance(control.get("valid"), bool):
        raise ValueError("target/control validity must be boolean")
    bbox = target.get("bbox")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ValueError("target.bbox must have four elements")

    for value, name in (
        (camera.get("fps"), "camera.fps"),
        (inference.get("fps"), "inference.fps"),
        (target.get("confidence"), "target.confidence"),
        (control.get("capture_to_result_ms"), "control.capture_to_result_ms"),
        (control.get("capture_media_pts_ms"), "control.capture_media_pts_ms"),
        (control.get("result_staleness_ms"), "control.result_staleness_ms"),
        *[(item, f"target.bbox[{index}]") for index, item in enumerate(bbox)],
    ):
        finite_number(value, name, nonnegative=name in {
            "camera.fps", "inference.fps", "control.capture_to_result_ms",
            "control.capture_media_pts_ms", "control.result_staleness_ms"
        })
    for container, key, name in (
        (camera, "captured_frames", "camera.captured_frames"),
        (inference, "results", "inference.results"),
        (video, "frames_encoded", "video.frames_encoded"),
        (control, "frame_id", "control.frame_id"),
        (target, "source_width", "target.source_width"),
        (target, "source_height", "target.source_height"),
        (control, "capture_monotonic_ns", "control.capture_monotonic_ns"),
        (control, "generated_monotonic_ns", "control.generated_monotonic_ns"),
    ):
        value = container.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a nonnegative integer")
    return message


def expected_state_ok(expectation: str, observed: set[str]) -> bool:
    if expectation == "any":
        return True
    if expectation == "disabled":
        return observed == {"disabled"}
    return bool(observed & {"running", "finalized"}) and "fatal" not in observed


def main() -> int:
    args = parse_args()
    report_path = args.report or args.output.with_suffix(
        args.output.suffix + ".summary.txt")
    packets = 0
    bytes_received = 0
    parse_errors = 0
    schema_errors = 0
    sequence_gaps = 0
    out_of_order = 0
    counter_regressions = 0
    positive_camera_fps_samples = 0
    target_available_samples = 0
    stale_result_samples = 0
    maximum_result_staleness_ms = 0.0
    target_states: set[str] = set()
    network_states: set[str] = set()
    recording_states: set[str] = set()
    previous_sequence: int | None = None
    previous_sent_ns: int | None = None
    previous_captured: int | None = None
    first_arrival: float | None = None
    last_arrival: float | None = None
    latest: dict[str, Any] | None = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                        args.receive_buffer_bytes)
        sock.bind((args.bind_address, args.port))
        sock.settimeout(0.5)
        actual_buffer = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        print(f"listening=udp://{args.bind_address}:{args.port}", flush=True)
        print(f"capture_output={args.output}", flush=True)
        wait_deadline = time.monotonic() + args.start_timeout_seconds
        with args.output.open("w", encoding="utf-8", newline="\n") as output:
            while True:
                now = time.monotonic()
                if first_arrival is None:
                    if now >= wait_deadline:
                        print("no telemetry received before start timeout",
                              file=sys.stderr)
                        return 1
                elif now - first_arrival >= args.duration_seconds:
                    break
                try:
                    payload, _peer = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                arrival = time.monotonic()
                first_arrival = arrival if first_arrival is None else first_arrival
                last_arrival = arrival
                packets += 1
                bytes_received += len(payload)
                try:
                    text = payload.decode("utf-8")
                    message = validate_message(json.loads(text))
                except json.JSONDecodeError:
                    parse_errors += 1
                    continue
                except (UnicodeDecodeError, ValueError):
                    schema_errors += 1
                    continue
                output.write(text.rstrip("\r\n") + "\n")
                latest = message
                sequence = message["sequence"]
                sent_ns = message["sent_monotonic_ns"]
                captured = message["camera"]["captured_frames"]
                if previous_sequence is not None:
                    if sequence <= previous_sequence:
                        out_of_order += 1
                    elif sequence != previous_sequence + 1:
                        sequence_gaps += sequence - previous_sequence - 1
                if previous_sent_ns is not None and sent_ns <= previous_sent_ns:
                    out_of_order += 1
                if previous_captured is not None and captured < previous_captured:
                    counter_regressions += 1
                previous_sequence = sequence
                previous_sent_ns = sent_ns
                previous_captured = captured
                if message["camera"]["fps"] > 0.0:
                    positive_camera_fps_samples += 1
                if message["target"]["available"]:
                    target_available_samples += 1
                result_staleness_ms = \
                    message["control"]["result_staleness_ms"]
                maximum_result_staleness_ms = max(
                    maximum_result_staleness_ms, result_staleness_ms)
                if result_staleness_ms > args.stale_result_ms:
                    stale_result_samples += 1
                target_states.add(message["target"]["state"])
                network_states.add(message["network"]["state"])
                recording_states.add(message["recording"]["state"])
                if packets % args.print_every == 0:
                    print(
                        "telemetry"
                        f" seq={sequence}"
                        f" camera_fps={message['camera']['fps']:.2f}"
                        f" inference_fps={message['inference']['fps']:.2f}"
                        f" target={message['target']['state']}"
                        f" latency_ms={message['control']['capture_to_result_ms']:.2f}"
                        f" network={message['network']['state']}"
                        f" recording={message['recording']['state']}",
                        flush=True)
    except KeyboardInterrupt:
        print("telemetry capture interrupted", file=sys.stderr)
        return 130
    finally:
        sock.close()

    elapsed = 0.0 if first_arrival is None or last_arrival is None else \
        max(last_arrival - first_arrival, 0.0)
    network_ok = expected_state_ok(args.expect_network, network_states)
    recording_ok = expected_state_ok(args.expect_recording, recording_states)
    passed = (
        packets > 0 and latest is not None and parse_errors == 0 and
        schema_errors == 0 and sequence_gaps == 0 and out_of_order == 0 and
        counter_regressions == 0 and positive_camera_fps_samples > 0 and
        target_available_samples > 0 and network_ok and recording_ok
    )
    lines = [
        f"input_port={args.port}",
        f"output={args.output}",
        f"observed_duration_seconds={elapsed:.6f}",
        f"requested_receive_buffer_bytes={args.receive_buffer_bytes}",
        f"actual_receive_buffer_bytes={actual_buffer}",
        f"datagrams_received={packets}",
        f"bytes_received={bytes_received}",
        f"parse_errors={parse_errors}",
        f"schema_errors={schema_errors}",
        f"sequence_gaps={sequence_gaps}",
        f"out_of_order={out_of_order}",
        f"counter_regressions={counter_regressions}",
        f"positive_camera_fps_samples={positive_camera_fps_samples}",
        f"target_available_samples={target_available_samples}",
        f"stale_result_threshold_ms={args.stale_result_ms:.3f}",
        f"stale_result_samples={stale_result_samples}",
        f"maximum_result_staleness_ms={maximum_result_staleness_ms:.3f}",
        "target_states=" + ",".join(sorted(target_states)),
        "network_states=" + ",".join(sorted(network_states)),
        "recording_states=" + ",".join(sorted(recording_states)),
        f"network_expectation_ok={int(network_ok)}",
        f"recording_expectation_ok={int(recording_ok)}",
        "telemetry_validation=PASS" if passed else
        "telemetry_validation=FAIL",
    ]
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"report={report_path}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
