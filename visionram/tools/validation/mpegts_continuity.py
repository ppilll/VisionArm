#!/usr/bin/env python3
"""Check continuity counters in a raw MPEG-TS UDP capture."""

from __future__ import annotations

import argparse
import dataclasses
import pathlib
from collections.abc import Callable


TS_PACKET_SIZE = 188


@dataclasses.dataclass
class PidStats:
    packets: int = 0
    payload_packets: int = 0
    continuity_gap_events: int = 0
    estimated_missing_packets: int = 0
    duplicate_payload_packets: int = 0
    discontinuities: int = 0
    last_payload_cc: int | None = None
    last_payload_packet: bytes | None = None


class SectionAssembler:
    def __init__(self, callback: Callable[[bytes], None]) -> None:
        self._buffer = bytearray()
        self._callback = callback

    def _drain(self) -> None:
        while self._buffer:
            if self._buffer[0] == 0xFF:
                self._buffer.clear()
                return
            if len(self._buffer) < 3:
                return
            section_length = ((self._buffer[1] & 0x0F) << 8) | self._buffer[2]
            total_length = 3 + section_length
            if section_length > 4093:
                self._buffer.clear()
                return
            if len(self._buffer) < total_length:
                return
            section = bytes(self._buffer[:total_length])
            del self._buffer[:total_length]
            self._callback(section)

    def push(self, payload: bytes, payload_unit_start: bool) -> None:
        if not payload:
            return
        if payload_unit_start:
            pointer = payload[0]
            if 1 + pointer > len(payload):
                self._buffer.clear()
                return
            if self._buffer and pointer:
                self._buffer.extend(payload[1:1 + pointer])
                self._drain()
            self._buffer.clear()
            self._buffer.extend(payload[1 + pointer:])
        elif self._buffer:
            self._buffer.extend(payload)
        self._drain()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate raw MPEG-TS sync and PID continuity counters")
    parser.add_argument("input", type=pathlib.Path)
    parser.add_argument("--report", type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_path = args.report or args.input.with_suffix(
        args.input.suffix + ".continuity.txt")
    pid_stats: dict[int, PidStats] = {}
    pmt_pids: set[int] = set()
    stream_types: dict[int, int] = {}
    pmt_assemblers: dict[int, SectionAssembler] = {}
    packet_count = 0
    sync_errors = 0
    transport_errors = 0
    invalid_packets = 0
    trailing_bytes = 0
    pat_sections = 0
    pmt_sections = 0

    def consume_pmt(section: bytes) -> None:
        nonlocal pmt_sections
        if len(section) < 16 or section[0] != 0x02:
            return
        pmt_sections += 1
        program_info_length = ((section[10] & 0x0F) << 8) | section[11]
        offset = 12 + program_info_length
        end = len(section) - 4
        while offset + 5 <= end:
            stream_type = section[offset]
            elementary_pid = ((section[offset + 1] & 0x1F) << 8) | section[offset + 2]
            descriptor_length = ((section[offset + 3] & 0x0F) << 8) | section[offset + 4]
            stream_types[elementary_pid] = stream_type
            offset += 5 + descriptor_length

    def consume_pat(section: bytes) -> None:
        nonlocal pat_sections
        if len(section) < 12 or section[0] != 0x00:
            return
        pat_sections += 1
        offset = 8
        end = len(section) - 4
        while offset + 4 <= end:
            program_number = (section[offset] << 8) | section[offset + 1]
            program_map_pid = ((section[offset + 2] & 0x1F) << 8) | section[offset + 3]
            if program_number != 0:
                pmt_pids.add(program_map_pid)
                pmt_assemblers.setdefault(
                    program_map_pid, SectionAssembler(consume_pmt))
            offset += 4

    pat_assembler = SectionAssembler(consume_pat)

    with args.input.open("rb") as source:
        while True:
            packet = source.read(TS_PACKET_SIZE)
            if not packet:
                break
            if len(packet) != TS_PACKET_SIZE:
                trailing_bytes = len(packet)
                break
            packet_count += 1
            if packet[0] != 0x47:
                sync_errors += 1
                continue
            if packet[1] & 0x80:
                transport_errors += 1
            payload_unit_start = bool(packet[1] & 0x40)
            pid = ((packet[1] & 0x1F) << 8) | packet[2]
            adaptation_control = (packet[3] >> 4) & 0x03
            continuity_counter = packet[3] & 0x0F
            stats = pid_stats.setdefault(pid, PidStats())
            stats.packets += 1
            if adaptation_control == 0:
                invalid_packets += 1
                continue

            payload_offset = 4
            discontinuity = False
            if adaptation_control in (2, 3):
                adaptation_length = packet[4]
                payload_offset = 5 + adaptation_length
                if payload_offset > TS_PACKET_SIZE:
                    invalid_packets += 1
                    continue
                if adaptation_length > 0:
                    discontinuity = bool(packet[5] & 0x80)
            if discontinuity:
                stats.discontinuities += 1

            has_payload = adaptation_control in (1, 3) and payload_offset < TS_PACKET_SIZE
            if not has_payload:
                continue
            stats.payload_packets += 1
            if stats.last_payload_cc is not None and not discontinuity:
                expected = (stats.last_payload_cc + 1) & 0x0F
                if continuity_counter == stats.last_payload_cc and packet == stats.last_payload_packet:
                    stats.duplicate_payload_packets += 1
                elif continuity_counter != expected:
                    stats.continuity_gap_events += 1
                    stats.estimated_missing_packets += (
                        continuity_counter - expected) & 0x0F
            stats.last_payload_cc = continuity_counter
            stats.last_payload_packet = packet

            payload = packet[payload_offset:]
            if pid == 0:
                pat_assembler.push(payload, payload_unit_start)
            assembler = pmt_assemblers.get(pid)
            if assembler is not None:
                assembler.push(payload, payload_unit_start)

    hevc_pids = sorted(pid for pid, stream_type in stream_types.items()
                       if stream_type == 0x24)
    aac_pids = sorted(pid for pid, stream_type in stream_types.items()
                      if stream_type in (0x0F, 0x11))
    total_gap_events = sum(item.continuity_gap_events
                           for item in pid_stats.values())
    total_missing = sum(item.estimated_missing_packets
                        for item in pid_stats.values())
    total_duplicates = sum(item.duplicate_payload_packets
                           for item in pid_stats.values())

    roles: dict[int, str] = {0: "PAT"}
    roles.update((pid, "PMT") for pid in pmt_pids)
    roles.update((pid, "HEVC") for pid in hevc_pids)
    roles.update((pid, "AAC") for pid in aac_pids)
    interesting_pids = sorted(
        set(roles) |
        {pid for pid, item in pid_stats.items()
         if item.continuity_gap_events or item.discontinuities})

    passed = (
        packet_count > 0 and sync_errors == 0 and trailing_bytes == 0 and
        transport_errors == 0 and invalid_packets == 0 and
        total_gap_events == 0 and pat_sections > 0 and pmt_sections > 0 and
        bool(hevc_pids) and bool(aac_pids)
    )
    lines = [
        f"input={args.input}",
        f"ts_packets={packet_count}",
        f"sync_errors={sync_errors}",
        f"trailing_bytes={trailing_bytes}",
        f"transport_error_packets={transport_errors}",
        f"invalid_ts_packets={invalid_packets}",
        f"pat_sections={pat_sections}",
        f"pmt_sections={pmt_sections}",
        "pmt_pids=" + ",".join(f"0x{pid:04x}" for pid in sorted(pmt_pids)),
        "hevc_pids=" + ",".join(f"0x{pid:04x}" for pid in hevc_pids),
        "aac_pids=" + ",".join(f"0x{pid:04x}" for pid in aac_pids),
        f"continuity_gap_events={total_gap_events}",
        f"estimated_missing_packets={total_missing}",
        f"duplicate_payload_packets={total_duplicates}",
    ]
    for pid in interesting_pids:
        item = pid_stats.get(pid, PidStats())
        prefix = f"pid.0x{pid:04x}"
        lines.extend([
            f"{prefix}.role={roles.get(pid, 'other')}",
            f"{prefix}.packets={item.packets}",
            f"{prefix}.continuity_gap_events={item.continuity_gap_events}",
            f"{prefix}.estimated_missing_packets={item.estimated_missing_packets}",
            f"{prefix}.discontinuities={item.discontinuities}",
        ])
    lines.append("mpegts_continuity=PASS" if passed else "mpegts_continuity=FAIL")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"report={report_path}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
