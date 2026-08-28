#!/usr/bin/env python3
"""Low-latency ffplay viewer plus independent telemetry JSON summary."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import pathlib
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any

from telemetry import validate_message


class RunningStats:
    def __init__(self) -> None:
        self.count = 0
        self.total = 0.0
        self.minimum: float | None = None
        self.maximum: float | None = None
        self.last: float | None = None

    def add(self, value: float) -> None:
        converted = float(value)
        self.count += 1
        self.total += converted
        self.minimum = converted if self.minimum is None else min(
            self.minimum, converted)
        self.maximum = converted if self.maximum is None else max(
            self.maximum, converted)
        self.last = converted

    def snapshot(self) -> dict[str, float | int | None]:
        return {
            "samples": self.count,
            "minimum": self.minimum,
            "maximum": self.maximum,
            "mean": None if self.count == 0 else self.total / self.count,
            "last": self.last,
        }


class TelemetryAccumulator:
    """Thread-safe wire-health and runtime-state aggregation."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._datagrams_received = 0
        self._bytes_received = 0
        self._valid_datagrams = 0
        self._parse_errors = 0
        self._schema_errors = 0
        self._sequence_gaps = 0
        self._out_of_order = 0
        self._counter_regressions = 0
        self._receiver_errors = 0
        self._first_sequence: int | None = None
        self._last_sequence: int | None = None
        self._previous_sent_ns: int | None = None
        self._previous_captured: int | None = None
        self._first_arrival: float | None = None
        self._last_arrival: float | None = None
        self._camera_fps = RunningStats()
        self._inference_fps = RunningStats()
        self._capture_to_result_ms = RunningStats()
        self._result_staleness_ms = RunningStats()
        self._target_states: Counter[str] = Counter()
        self._network_states: Counter[str] = Counter()
        self._recording_states: Counter[str] = Counter()
        self._latest: dict[str, Any] | None = None
        self._last_error = ""

    def add_payload(
            self, payload: bytes,
            arrival_monotonic: float | None = None) -> tuple[
                dict[str, Any] | None, str | None]:
        arrival = time.monotonic() if arrival_monotonic is None else \
            float(arrival_monotonic)
        with self._lock:
            self._datagrams_received += 1
            self._bytes_received += len(payload)
        try:
            text = payload.decode("utf-8")
            decoded = json.loads(text)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            with self._lock:
                self._parse_errors += 1
                self._last_error = str(error)
            return None, None
        try:
            message = validate_message(decoded)
        except (ValueError, KeyError, TypeError) as error:
            with self._lock:
                self._schema_errors += 1
                self._last_error = str(error)
            return None, None

        sequence = message["sequence"]
        sent_ns = message["sent_monotonic_ns"]
        captured = message["camera"]["captured_frames"]
        with self._lock:
            self._valid_datagrams += 1
            if self._first_sequence is None:
                self._first_sequence = sequence
            if self._last_sequence is not None:
                if sequence <= self._last_sequence:
                    self._out_of_order += 1
                elif sequence > self._last_sequence + 1:
                    self._sequence_gaps += sequence - self._last_sequence - 1
            if self._previous_sent_ns is not None and \
                    sent_ns <= self._previous_sent_ns:
                self._out_of_order += 1
            if self._previous_captured is not None and \
                    captured < self._previous_captured:
                self._counter_regressions += 1

            if self._last_sequence is None or sequence > self._last_sequence:
                self._last_sequence = sequence
                self._previous_sent_ns = sent_ns
                self._previous_captured = captured
                self._latest = message
            self._first_arrival = arrival if self._first_arrival is None else \
                self._first_arrival
            self._last_arrival = arrival
            self._camera_fps.add(message["camera"]["fps"])
            self._inference_fps.add(message["inference"]["fps"])
            self._capture_to_result_ms.add(
                message["control"]["capture_to_result_ms"])
            self._result_staleness_ms.add(
                message["control"]["result_staleness_ms"])
            self._target_states[message["target"]["state"]] += 1
            self._network_states[message["network"]["state"]] += 1
            self._recording_states[message["recording"]["state"]] += 1
        return message, text

    def set_receiver_error(self, error: BaseException) -> None:
        with self._lock:
            self._receiver_errors += 1
            self._last_error = str(error)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            duration = 0.0
            if self._first_arrival is not None and \
                    self._last_arrival is not None:
                duration = max(
                    0.0, self._last_arrival - self._first_arrival)
            latest = None
            if self._latest is not None:
                message = self._latest
                latest = {
                    "sequence": message["sequence"],
                    "sent_monotonic_ns": message["sent_monotonic_ns"],
                    "pipeline": message["pipeline"],
                    "camera": message["camera"],
                    "inference": message["inference"],
                    "video": message["video"],
                    "network": message["network"],
                    "recording": message["recording"],
                    "target": message["target"],
                    "control": message["control"],
                }
            wire_ok = (
                self._valid_datagrams > 0 and self._parse_errors == 0 and
                self._schema_errors == 0 and self._sequence_gaps == 0 and
                self._out_of_order == 0 and
                self._counter_regressions == 0 and
                self._receiver_errors == 0)
            return {
                "datagrams_received": self._datagrams_received,
                "bytes_received": self._bytes_received,
                "valid_datagrams": self._valid_datagrams,
                "parse_errors": self._parse_errors,
                "schema_errors": self._schema_errors,
                "first_sequence": self._first_sequence,
                "last_sequence": self._last_sequence,
                "sequence_gaps": self._sequence_gaps,
                "out_of_order": self._out_of_order,
                "counter_regressions": self._counter_regressions,
                "receiver_errors": self._receiver_errors,
                "observed_duration_seconds": duration,
                "camera_fps": self._camera_fps.snapshot(),
                "inference_fps": self._inference_fps.snapshot(),
                "capture_to_result_ms":
                    self._capture_to_result_ms.snapshot(),
                "result_staleness_ms":
                    self._result_staleness_ms.snapshot(),
                "target_states": dict(sorted(self._target_states.items())),
                "network_states": dict(
                    sorted(self._network_states.items())),
                "recording_states": dict(
                    sorted(self._recording_states.items())),
                "wire_validation": "PASS" if wire_ok else "FAIL",
                "last_error": self._last_error,
                "latest": latest,
            }


class TelemetryReceiver:
    def __init__(
            self, accumulator: TelemetryAccumulator, bind_address: str,
            port: int, receive_buffer_bytes: int,
            log_path: pathlib.Path | None, print_every: int) -> None:
        self._accumulator = accumulator
        self._stop = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer_bytes)
        self._socket.bind((bind_address, port))
        self._socket.settimeout(0.5)
        self.actual_receive_buffer_bytes = self._socket.getsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF)
        self._log_path = log_path
        self._print_every = print_every
        self._thread = threading.Thread(
            target=self._run, name="visionarm-telemetry-receiver",
            daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._socket.close()

    def _run(self) -> None:
        output = None
        valid_count = 0
        try:
            if self._log_path is not None:
                self._log_path.parent.mkdir(parents=True, exist_ok=True)
                output = self._log_path.open(
                    "w", encoding="utf-8", newline="\n")
            while not self._stop.is_set():
                try:
                    payload, _peer = self._socket.recvfrom(65535)
                except socket.timeout:
                    continue
                message, text = self._accumulator.add_payload(payload)
                if message is None or text is None:
                    continue
                valid_count += 1
                if output is not None:
                    output.write(text.rstrip("\r\n") + "\n")
                    output.flush()
                if valid_count % self._print_every == 0:
                    print(
                        "telemetry"
                        f" seq={message['sequence']}"
                        f" camera_fps={message['camera']['fps']:.2f}"
                        f" inference_fps={message['inference']['fps']:.2f}"
                        f" target={message['target']['state']}"
                        f" network={message['network']['state']}"
                        f" recording={message['recording']['state']}",
                        flush=True)
        except OSError as error:
            if not self._stop.is_set():
                self._accumulator.set_receiver_error(error)
        finally:
            if output is not None:
                output.close()


def build_video_url(args: argparse.Namespace) -> str:
    return (
        f"udp://{args.bind_address}:{args.video_port}"
        f"?fifo_size={args.ffplay_fifo_size}"
        "&overrun_nonfatal=0"
        f"&buffer_size={args.video_receive_buffer_bytes}")


def build_ffplay_command(
        args: argparse.Namespace, ffplay_executable: str) -> list[str]:
    return [
        ffplay_executable,
        "-hide_banner",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-framedrop",
        "-probesize", str(args.probesize),
        "-analyzeduration", str(args.analyzeduration),
        "-window_title", "VisionArm live A/V",
        build_video_url(args),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=("Low-latency ffplay UDP/5000 viewer with independent "
                     "UDP/5001 telemetry summary"))
    parser.add_argument("--video-port", type=int, default=5000)
    parser.add_argument("--telemetry-port", type=int, default=5001)
    parser.add_argument("--bind", default="0.0.0.0", dest="bind_address")
    parser.add_argument("--video-receive-buffer-bytes", type=int,
                        default=4 * 1024 * 1024)
    parser.add_argument("--telemetry-receive-buffer-bytes", type=int,
                        default=1024 * 1024)
    parser.add_argument("--ffplay-fifo-size", type=int, default=65536)
    parser.add_argument("--probesize", type=int, default=5_000_000)
    parser.add_argument("--analyzeduration", type=int, default=5_000_000)
    parser.add_argument("--ffplay", default="ffplay",
                        help="ffplay executable or absolute path")
    parser.add_argument("--duration-seconds", type=float, default=0.0,
                        help="0 runs until the ffplay window closes")
    parser.add_argument("--telemetry-print-every", type=int, default=10)
    parser.add_argument("--telemetry-log", type=pathlib.Path)
    parser.add_argument("--summary-json", type=pathlib.Path,
                        default=pathlib.Path("visionarm_view_summary.json"))
    args = parser.parse_args()

    for port in (args.video_port, args.telemetry_port):
        if not 1 <= port <= 65535:
            parser.error("ports must be in [1, 65535]")
    if args.video_port == args.telemetry_port:
        parser.error("video and telemetry ports must differ")
    for value, name in (
            (args.video_receive_buffer_bytes, "video receive buffer"),
            (args.telemetry_receive_buffer_bytes,
             "telemetry receive buffer"),
            (args.ffplay_fifo_size, "ffplay FIFO size"),
            (args.probesize, "probesize"),
            (args.analyzeduration, "analyzeduration"),
            (args.telemetry_print_every, "telemetry print interval")):
        if value <= 0:
            parser.error(f"{name} must be positive")
    if not math.isfinite(args.duration_seconds) or \
            args.duration_seconds < 0.0:
        parser.error("duration must be finite and nonnegative")
    return args


def _resolve_ffplay(value: str) -> str | None:
    path = pathlib.Path(value)
    if path.parent != pathlib.Path("."):
        return str(path) if path.is_file() else None
    return shutil.which(value)


def _terminate_process(process: subprocess.Popen[Any]) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)


def main() -> int:
    args = parse_args()
    ffplay = _resolve_ffplay(args.ffplay)
    if ffplay is None:
        print(
            f"ffplay not found: {args.ffplay}; install FFmpeg or pass "
            "--ffplay /absolute/path/to/ffplay", file=sys.stderr)
        return 2
    accumulator = TelemetryAccumulator()
    try:
        receiver = TelemetryReceiver(
            accumulator, args.bind_address, args.telemetry_port,
            args.telemetry_receive_buffer_bytes, args.telemetry_log,
            args.telemetry_print_every)
    except OSError as error:
        print(f"telemetry startup failed: {error}", file=sys.stderr)
        return 2

    video_url = build_video_url(args)
    command = build_ffplay_command(args, ffplay)
    stop_requested = threading.Event()
    stop_reason = "ffplay_exit"

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    receiver.start()
    started_monotonic = time.monotonic()
    started_utc = datetime.now(timezone.utc).isoformat()
    print(f"video={video_url}", flush=True)
    print(
        f"telemetry=udp://{args.bind_address}:{args.telemetry_port}",
        flush=True)
    print(f"summary_json={args.summary_json}", flush=True)
    try:
        # Keep terminal Ctrl-C under the Python supervisor's control; it then
        # terminates ffplay and always has a chance to finalize summary JSON.
        process = subprocess.Popen(command, start_new_session=True)
    except OSError as error:
        receiver.stop()
        print(f"failed to start ffplay: {error}", file=sys.stderr)
        return 2

    try:
        while process.poll() is None:
            if stop_requested.wait(timeout=0.2):
                stop_reason = "signal"
                break
            if args.duration_seconds > 0.0 and \
                    time.monotonic() - started_monotonic >= \
                    args.duration_seconds:
                stop_reason = "duration"
                break
    finally:
        _terminate_process(process)
        receiver.stop()
    if stop_requested.is_set():
        stop_reason = "signal"

    ended_monotonic = time.monotonic()
    telemetry = accumulator.snapshot()
    telemetry["bind_address"] = args.bind_address
    telemetry["port"] = args.telemetry_port
    telemetry["requested_receive_buffer_bytes"] = \
        args.telemetry_receive_buffer_bytes
    telemetry["actual_receive_buffer_bytes"] = \
        receiver.actual_receive_buffer_bytes
    telemetry["raw_log"] = None if args.telemetry_log is None else \
        str(args.telemetry_log)
    summary = {
        "schema": "visionarm.viewer_summary.v1",
        "started_utc": started_utc,
        "ended_utc": datetime.now(timezone.utc).isoformat(),
        "observed_duration_seconds": max(
            0.0, ended_monotonic - started_monotonic),
        "stop_reason": stop_reason,
        "video": {
            "backend": "ffplay",
            "url": video_url,
            "low_latency": True,
            "framedrop": True,
            "probesize": args.probesize,
            "analyzeduration": args.analyzeduration,
            "exit_code": process.returncode,
        },
        "telemetry": telemetry,
    }
    try:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
    except OSError as error:
        print(f"failed to write summary JSON: {error}", file=sys.stderr)
        return 1

    print(
        f"telemetry_valid={telemetry['valid_datagrams']} "
        f"parse_errors={telemetry['parse_errors']} "
        f"schema_errors={telemetry['schema_errors']} "
        f"sequence_gaps={telemetry['sequence_gaps']} "
        f"receiver_errors={telemetry['receiver_errors']} "
        f"wire_validation={telemetry['wire_validation']}", flush=True)
    print(f"summary={args.summary_json}", flush=True)
    if telemetry["valid_datagrams"] == 0:
        return 1
    if stop_reason == "ffplay_exit" and process.returncode not in (0, None):
        return int(process.returncode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
