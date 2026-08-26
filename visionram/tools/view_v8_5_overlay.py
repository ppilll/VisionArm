#!/usr/bin/env python3
"""Play V8.4 MPEG-TS A/V and draw V8.5 side telemetry on the PC."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import signal
import socket
import sys
import threading
from typing import Any

from receive_v8_5_telemetry import validate_message
from v8_5_overlay_core import OverlayMatch, TelemetryTimeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="V8.5 HEVC+AAC MPEG-TS player with display-only overlay")
    parser.add_argument("--video-port", type=int, default=5000)
    parser.add_argument("--telemetry-port", type=int, default=5001)
    parser.add_argument("--bind", default="0.0.0.0", dest="bind_address")
    parser.add_argument("--video-receive-buffer-bytes", type=int,
                        default=4 * 1024 * 1024)
    parser.add_argument("--telemetry-receive-buffer-bytes", type=int,
                        default=1024 * 1024)
    parser.add_argument("--timeline-capacity", type=int, default=512)
    parser.add_argument("--result-stale-ms", type=float, default=300.0)
    parser.add_argument("--telemetry-stale-ms", type=float, default=1000.0)
    parser.add_argument(
        "--video-pts-offset-ms", type=float, default=0.0,
        help="constant added to decoded video PTS; leave 0 unless measured")
    parser.add_argument("--telemetry-log", type=pathlib.Path)
    args = parser.parse_args()
    for port in (args.video_port, args.telemetry_port):
        if not 1 <= port <= 65535:
            parser.error("ports must be in [1, 65535]")
    if args.video_port == args.telemetry_port:
        parser.error("video and telemetry ports must differ")
    if args.video_receive_buffer_bytes <= 0 or \
            args.telemetry_receive_buffer_bytes <= 0 or \
            args.timeline_capacity <= 0:
        parser.error("buffer sizes and timeline capacity must be positive")
    for value, name in (
            (args.result_stale_ms, "result stale threshold"),
            (args.telemetry_stale_ms, "telemetry stale threshold")):
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"{name} must be finite and nonnegative")
    if not math.isfinite(args.video_pts_offset_ms):
        parser.error("video PTS offset must be finite")
    return args


class TelemetryReceiver:
    def __init__(
            self, timeline: TelemetryTimeline, bind_address: str, port: int,
            receive_buffer_bytes: int,
            log_path: pathlib.Path | None) -> None:
        self._timeline = timeline
        self._stop = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer_bytes)
        self._socket.bind((bind_address, port))
        self._socket.settimeout(0.5)
        self._log_path = log_path
        self._thread = threading.Thread(
            target=self._run, name="v8.5-telemetry-receiver", daemon=True)
        self._lock = threading.Lock()
        self._received = 0
        self._invalid = 0
        self._last_error = ""

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._socket.close()

    def summary(self) -> str:
        with self._lock:
            received = self._received
            invalid = self._invalid
            last_error = self._last_error
        stats = self._timeline.stats()
        return (
            f"telemetry_received={received} telemetry_invalid={invalid} "
            f"accepted={stats.accepted_datagrams} "
            f"out_of_order={stats.rejected_out_of_order} "
            f"sequence_gaps={stats.sequence_gaps} "
            f"retained_results={stats.retained_results} "
            f"last_error={last_error}")

    def _run(self) -> None:
        output = None
        try:
            if self._log_path is not None:
                output = self._log_path.open(
                    "w", encoding="utf-8", newline="\n")
            while not self._stop.is_set():
                try:
                    payload, _peer = self._socket.recvfrom(65535)
                except socket.timeout:
                    continue
                try:
                    text = payload.decode("utf-8")
                    message = validate_message(json.loads(text))
                    accepted = self._timeline.add(message)
                    if accepted and output is not None:
                        output.write(text.rstrip("\r\n") + "\n")
                        output.flush()
                    with self._lock:
                        self._received += 1
                except (UnicodeDecodeError, json.JSONDecodeError,
                        ValueError, KeyError, TypeError) as error:
                    with self._lock:
                        self._invalid += 1
                        self._last_error = str(error)
        except OSError as error:
            if not self._stop.is_set():
                with self._lock:
                    self._last_error = str(error)
        finally:
            if output is not None:
                output.close()


class OverlayRenderer:
    def __init__(
            self, timeline: TelemetryTimeline, gst: Any,
            video_pts_offset_ms: float) -> None:
        self._timeline = timeline
        self._gst = gst
        self._video_pts_offset_ms = video_pts_offset_ms
        self._width = 1920
        self._height = 1080

    def on_caps_changed(self, _overlay: Any, caps: Any) -> None:
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        if isinstance(width, int) and width > 0:
            self._width = width
        if isinstance(height, int) and height > 0:
            self._height = height

    def on_draw(
            self, _overlay: Any, context: Any, timestamp: int,
            _duration: int) -> None:
        pts_ms = None
        if timestamp != self._gst.CLOCK_TIME_NONE:
            pts_ms = timestamp / 1_000_000.0 + self._video_pts_offset_ms
        match = self._timeline.match(pts_ms)
        self._draw_status(context, match)
        if match.bbox_visible:
            self._draw_bbox(context, match)

    @staticmethod
    def _state_lines(match: OverlayMatch) -> list[str]:
        runtime = match.runtime_message
        if runtime is None:
            return ["VISIONARM V8.5", "TELEMETRY: WAITING"]
        camera = runtime["camera"]
        inference = runtime["inference"]
        network = runtime["network"]
        recording = runtime["recording"]
        lines = [
            f"CAM {camera['fps']:.2f} FPS   NPU {inference['fps']:.2f} FPS",
            f"NET {network['state']}   REC {recording['state']}",
        ]
        if match.result_message is None:
            lines.append(f"TARGET --   ({match.reason})")
        else:
            target = match.result_message["target"]
            control = match.result_message["control"]
            lines.extend([
                f"TARGET {target['state']}   conf={target['confidence']:.3f}",
                f"CTRL dx={control['dx_px']:.1f} dy={control['dy_px']:.1f} "
                f"C->R={control['capture_to_result_ms']:.1f} ms",
                f"RESULT age={match.result_age_at_video_ms:.1f} ms   "
                f"{match.reason}",
            ])
        telemetry_age = "--" if match.telemetry_age_ms is None else \
            f"{match.telemetry_age_ms:.0f} ms"
        health = "STALE" if match.telemetry_stale else "LIVE"
        lines.append(
            f"TELEMETRY {health} age={telemetry_age} "
            f"seq={runtime['sequence']}")
        return lines

    def _draw_status(self, context: Any, match: OverlayMatch) -> None:
        lines = self._state_lines(match)
        line_height = 25.0
        box_width = min(660.0, max(420.0, self._width * 0.42))
        box_height = 18.0 + line_height * len(lines)
        context.save()
        context.set_source_rgba(0.0, 0.0, 0.0, 0.67)
        context.rectangle(12.0, 12.0, box_width, box_height)
        context.fill()
        context.select_font_face("Sans", 0, 0)
        context.set_font_size(18.0)
        if match.telemetry_stale:
            context.set_source_rgba(1.0, 0.25, 0.2, 1.0)
        else:
            context.set_source_rgba(0.2, 1.0, 0.35, 1.0)
        for index, line in enumerate(lines):
            if index > 0:
                context.set_source_rgba(1.0, 1.0, 1.0, 1.0)
            context.move_to(24.0, 39.0 + line_height * index)
            context.show_text(line)
        context.restore()

    def _draw_bbox(self, context: Any, match: OverlayMatch) -> None:
        assert match.result_message is not None
        target = match.result_message["target"]
        source_width = float(target["source_width"])
        source_height = float(target["source_height"])
        x1, y1, x2, y2 = [float(value) for value in target["bbox"]]
        scale_x = self._width / source_width
        scale_y = self._height / source_height
        x = max(0.0, min(self._width, x1 * scale_x))
        y = max(0.0, min(self._height, y1 * scale_y))
        right = max(0.0, min(self._width, x2 * scale_x))
        bottom = max(0.0, min(self._height, y2 * scale_y))
        if right <= x or bottom <= y:
            return
        context.save()
        context.set_source_rgba(0.1, 1.0, 0.2, 0.95)
        context.set_line_width(max(2.0, self._width / 640.0))
        context.rectangle(x, y, right - x, bottom - y)
        context.stroke()
        context.set_font_size(18.0)
        context.move_to(x + 3.0, max(20.0, y - 5.0))
        context.show_text(
            f"football {target['confidence']:.3f}")
        context.restore()


def make_element(gst: Any, factory: str, name: str) -> Any:
    element = gst.ElementFactory.make(factory, name)
    if element is None:
        raise RuntimeError(
            f"missing GStreamer element '{factory}' (required by {name})")
    return element


def link_chain(elements: list[Any]) -> None:
    for upstream, downstream in zip(elements, elements[1:]):
        if not upstream.link(downstream):
            raise RuntimeError(
                f"failed to link {upstream.get_name()} -> "
                f"{downstream.get_name()}")


def build_pipeline(args: argparse.Namespace, timeline: TelemetryTimeline,
                   gst: Any) -> tuple[Any, OverlayRenderer]:
    pipeline = gst.Pipeline.new("visionarm-v8-5-overlay")
    if pipeline is None:
        raise RuntimeError("failed to create GStreamer pipeline")
    source = make_element(gst, "udpsrc", "mpegts-udp-source")
    demux = make_element(gst, "tsdemux", "mpegts-demux")
    video_queue = make_element(gst, "queue", "video-queue")
    video_decoder = make_element(gst, "decodebin", "video-decoder")
    video_convert = make_element(
        gst, "videoconvert", "overlay-input-convert")
    overlay_caps = make_element(
        gst, "capsfilter", "overlay-input-caps")
    overlay = make_element(gst, "cairooverlay", "telemetry-overlay")
    video_output_convert = make_element(
        gst, "videoconvert", "video-output-convert")
    video_sink = make_element(gst, "autovideosink", "video-output")
    audio_queue = make_element(gst, "queue", "audio-queue")
    audio_decoder = make_element(gst, "decodebin", "audio-decoder")
    audio_convert = make_element(gst, "audioconvert", "audio-convert")
    audio_resample = make_element(gst, "audioresample", "audio-resample")
    audio_sink = make_element(gst, "autoaudiosink", "audio-output")

    source.set_property("address", args.bind_address)
    source.set_property("port", args.video_port)
    source.set_property("buffer-size", args.video_receive_buffer_bytes)
    source.set_property("caps", gst.Caps.from_string(
        "video/mpegts,systemstream=(boolean)true,packetsize=(int)188"))
    # cairooverlay accepts only a small set of RGB raw formats. Make both
    # conversion boundaries explicit: decoded I420/NV12 -> BGRA for Cairo,
    # then BGRA -> whatever the selected platform video sink accepts.
    overlay_caps.set_property("caps", gst.Caps.from_string(
        "video/x-raw,format=(string)BGRA"))
    video_sink.set_property("sync", True)
    audio_sink.set_property("sync", True)
    video_queue.set_property("max-size-time", 2 * gst.SECOND)
    audio_queue.set_property("max-size-time", 2 * gst.SECOND)

    elements = [
        source, demux, video_queue, video_decoder, video_convert, overlay_caps,
        overlay, video_output_convert, video_sink, audio_queue, audio_decoder,
        audio_convert, audio_resample, audio_sink,
    ]
    for element in elements:
        pipeline.add(element)
    if not source.link(demux):
        raise RuntimeError("failed to link UDP source to MPEG-TS demuxer")
    link_chain([video_queue, video_decoder])
    link_chain([
        video_convert, overlay_caps, overlay, video_output_convert,
        video_sink])
    link_chain([audio_queue, audio_decoder])
    link_chain([audio_convert, audio_resample, audio_sink])

    def on_decoded_pad(
            decoder: Any, pad: Any, expected_caps_name: str,
            target: Any) -> None:
        caps = pad.get_current_caps() or pad.query_caps(None)
        caps_name = caps.get_structure(0).get_name() if caps and \
            caps.get_size() > 0 else ""
        if caps_name != expected_caps_name:
            print(
                f"ignoring unexpected {decoder.get_name()} output: "
                f"{caps.to_string() if caps else 'unknown caps'}",
                file=sys.stderr, flush=True)
            return
        if target.is_linked():
            print(
                f"ignoring additional {decoder.get_name()} output: "
                f"{caps.to_string()}", file=sys.stderr, flush=True)
            return
        if expected_caps_name == "video/x-raw":
            # A normal raw-video link recursively compares I420 with
            # cairooverlay through videoconvert and can reject a convertible
            # format as NOFORMAT. Validate hierarchy here; videoconvert + the
            # explicit BGRA filter negotiate when the CAPS event arrives.
            result = pad.link_full(target, gst.PadLinkCheck.HIERARCHY)
        else:
            # The audio path already negotiates successfully with the normal
            # caps check; keep that stricter behavior unchanged.
            result = pad.link(target)
        if result != gst.PadLinkReturn.OK:
            print(
                f"failed decoded link for {caps.to_string()}: {result}",
                file=sys.stderr, flush=True)
        else:
            print(
                f"linked decoded {caps_name}: {caps.to_string()}",
                flush=True)

    video_decoder.connect(
        "pad-added", on_decoded_pad, "video/x-raw",
        video_convert.get_static_pad("sink"))
    audio_decoder.connect(
        "pad-added", on_decoded_pad, "audio/x-raw",
        audio_convert.get_static_pad("sink"))

    def on_pad_added(_demux: Any, pad: Any) -> None:
        caps = pad.get_current_caps() or pad.query_caps(None)
        caps_name = caps.get_structure(0).get_name() if caps and \
            caps.get_size() > 0 else ""
        target = None
        if caps_name == "video/x-h265":
            target = video_queue.get_static_pad("sink")
        elif caps_name == "audio/mpeg":
            target = audio_queue.get_static_pad("sink")
        if target is not None and not target.is_linked():
            # Each queue now terminates at a decodebin ANY sink pad, so normal
            # caps-checked linking is valid for tsdemux's initial encoded caps.
            result = pad.link(target)
            if result != gst.PadLinkReturn.OK:
                print(
                    f"failed dynamic link for {caps.to_string()}: {result}",
                    file=sys.stderr, flush=True)
            else:
                print(
                    f"linked {caps_name} demux pad: {caps.to_string()}",
                    flush=True)

    demux.connect("pad-added", on_pad_added)
    renderer = OverlayRenderer(timeline, gst, args.video_pts_offset_ms)
    overlay.connect("caps-changed", renderer.on_caps_changed)
    overlay.connect("draw", renderer.on_draw)
    return pipeline, renderer


def main() -> int:
    args = parse_args()
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import GLib, Gst
        import cairo  # noqa: F401  # required by cairooverlay bindings
    except (ImportError, ValueError) as error:
        print(
            "PyGObject/GStreamer/Cairo is unavailable; install the packages "
            "listed in README.md\n" + str(error), file=sys.stderr)
        return 2

    Gst.init(None)
    timeline = TelemetryTimeline(
        capacity=args.timeline_capacity,
        result_stale_ms=args.result_stale_ms,
        telemetry_stale_ms=args.telemetry_stale_ms)
    try:
        receiver = TelemetryReceiver(
            timeline, args.bind_address, args.telemetry_port,
            args.telemetry_receive_buffer_bytes, args.telemetry_log)
        pipeline, _renderer = build_pipeline(args, timeline, Gst)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"startup failed: {error}", file=sys.stderr)
        return 2

    loop = GLib.MainLoop()
    exit_code = 0

    def on_bus_message(_bus: Any, message: Any) -> None:
        nonlocal exit_code
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            print(f"GStreamer error: {error}", file=sys.stderr)
            if debug:
                print(debug, file=sys.stderr)
            exit_code = 1
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            loop.quit()

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message)

    def request_stop(*_unused: Any) -> bool:
        loop.quit()
        return False

    signal.signal(signal.SIGINT, lambda *_unused: GLib.idle_add(request_stop))
    signal.signal(signal.SIGTERM, lambda *_unused: GLib.idle_add(request_stop))
    receiver.start()
    print(
        f"video=udp://{args.bind_address}:{args.video_port} "
        f"telemetry=udp://{args.bind_address}:{args.telemetry_port}",
        flush=True)
    state_result = pipeline.set_state(Gst.State.PLAYING)
    if state_result == Gst.StateChangeReturn.FAILURE:
        print("failed to set GStreamer pipeline to PLAYING", file=sys.stderr)
        exit_code = 1
    else:
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
    pipeline.set_state(Gst.State.NULL)
    bus.remove_signal_watch()
    receiver.stop()
    print(receiver.summary(), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
