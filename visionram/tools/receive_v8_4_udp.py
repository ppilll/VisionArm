#!/usr/bin/env python3
"""Capture raw UDP payloads without remuxing their MPEG-TS packets."""

from __future__ import annotations

import argparse
import collections
import pathlib
import socket
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture V8.4 raw MPEG-TS UDP datagrams")
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("duration_seconds", type=float)
    parser.add_argument("port", type=int, nargs="?", default=5000)
    parser.add_argument("--bind", default="0.0.0.0", dest="bind_address")
    parser.add_argument("--receive-buffer-bytes", type=int, default=4 * 1024 * 1024)
    parser.add_argument("--start-timeout-seconds", type=float, default=30.0)
    parser.add_argument("--report", type=pathlib.Path)
    args = parser.parse_args()
    if args.duration_seconds <= 0:
        parser.error("duration_seconds must be positive")
    if not 1 <= args.port <= 65535:
        parser.error("port must be in [1, 65535]")
    if args.receive_buffer_bytes <= 0 or args.start_timeout_seconds <= 0:
        parser.error("buffer size and start timeout must be positive")
    return args


def main() -> int:
    args = parse_args()
    report_path = args.report or args.output.with_suffix(
        args.output.suffix + ".udp_stats.txt")
    size_counts: collections.Counter[int] = collections.Counter()
    datagrams = 0
    byte_count = 0
    first_arrival: float | None = None
    last_arrival: float | None = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                        args.receive_buffer_bytes)
        sock.bind((args.bind_address, args.port))
        sock.settimeout(0.5)
        actual_buffer = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        print(f"listening=udp://{args.bind_address}:{args.port}", flush=True)
        print(f"capture_output={args.output}", flush=True)
        print("waiting_for_first_datagram=1", flush=True)
        wait_deadline = time.monotonic() + args.start_timeout_seconds

        with args.output.open("wb") as output:
            while True:
                now = time.monotonic()
                if first_arrival is None:
                    if now >= wait_deadline:
                        print("no UDP datagram received before start timeout",
                              file=sys.stderr)
                        return 1
                elif now - first_arrival >= args.duration_seconds:
                    break
                try:
                    payload, _peer = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                arrival = time.monotonic()
                if first_arrival is None:
                    first_arrival = arrival
                    print("first_datagram_received=1", flush=True)
                last_arrival = arrival
                output.write(payload)
                datagrams += 1
                byte_count += len(payload)
                size_counts[len(payload)] += 1
    except KeyboardInterrupt:
        print("capture interrupted", file=sys.stderr)
        return 130
    finally:
        sock.close()

    elapsed = 0.0
    if first_arrival is not None and last_arrival is not None:
        elapsed = max(last_arrival - first_arrival, 0.0)
    lines = [
        f"output={args.output}",
        f"bind={args.bind_address}",
        f"port={args.port}",
        f"requested_duration_seconds={args.duration_seconds:.3f}",
        f"observed_duration_seconds={elapsed:.6f}",
        f"requested_receive_buffer_bytes={args.receive_buffer_bytes}",
        f"actual_receive_buffer_bytes={actual_buffer}",
        f"datagrams={datagrams}",
        f"bytes={byte_count}",
    ]
    for size, count in sorted(size_counts.items()):
        lines.append(f"datagram_size.{size}={count}")
    lines.append("raw_udp_capture=PASS" if datagrams else "raw_udp_capture=FAIL")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"report={report_path}")
    return 0 if datagrams else 1


if __name__ == "__main__":
    raise SystemExit(main())
