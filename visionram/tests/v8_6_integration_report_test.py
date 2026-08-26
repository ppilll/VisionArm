#!/usr/bin/env python3
"""Unit tests for the dependency-free V8.6 report validator."""

from __future__ import annotations

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from validate_v8_6_integration_report import (  # noqa: E402
    Expectations,
    validate_report,
)


def add_queue(report: dict[str, str], prefix: str) -> None:
    report.update({
        f"{prefix}.capacity": "16",
        f"{prefix}.high_watermark": "8",
        f"{prefix}.current_size": "0",
        f"{prefix}.pushed": "100",
        f"{prefix}.popped": "100",
        f"{prefix}.replaced_oldest": "0",
        f"{prefix}.stopped": "1",
    })


def passing_report() -> dict[str, str]:
    report = {
        "vision_pipeline_r7_r8_probe": "PASS",
        "requested_duration_seconds": "600",
        "observed_duration_seconds": "600.125",
        "completed_requested_duration": "1",
        "terminated_by_signal": "0",
        "auxiliary_media_runtime_fault": "0",
        "preprocess_failures": "0",
        "inference_failures": "0",
        "postprocess_failures": "0",
        "result_publish_failures": "0",
        "requeue_failures": "0",
        "dmabuf_sync_failures": "0",
        "video_frames_dropped": "0",
        "video_branch_failed": "0",
        "video_encode_failures": "0",
        "video_packets_dropped": "0",
        "video_sink_failures": "0",
        "fatal_error": "0",
        "mpp_encode_failures": "0",
        "mpp_source_reimports": "0",
        "h265_write_failures": "0",
        "state_control_sink_failures": "0",
        "state_perception_sink_failures": "0",
        "state_invalid_timestamp_packets": "0",
        "camera_outstanding_before_stop": "0",
        "broker_outstanding_frames_before_camera_stop": "0",
        "broker_outstanding_leases_before_camera_stop": "0",
        "broker_outstanding_frames_after_stop": "0",
        "broker_outstanding_leases_after_stop": "0",
        "audio_worker_fatal_error": "0",
        "audio_queue_push_failures": "0",
        "audio_xruns": "0",
        "audio_suspends": "0",
        "audio_status_errors": "0",
        "audio_encode_worker_fatal_error": "0",
        "audio_encode_queue_push_failures": "0",
        "audio_encoder_failures": "0",
        "audio_encoder_buffered_input_frames": "0",
        "audio_encoded_sink_fatal_error": "0",
        "audio_encoded_sink_failures": "0",
        "media_audio_timestamp_failures": "0",
        "graceful_shutdown_completed": "1",
        "split_final_completed_frame_drained": "1",
        "audio_enabled": "1",
        "encoded_audio_enabled": "1",
        "audio_worker_started": "1",
        "audio_encode_worker_started": "1",
        "audio_encoder_drained": "1",
        "audio_encoded_sink_started": "1",
        "audio_path_ok": "1",
        "audio_encode_path_ok": "1",
        "captured_frames": "18000",
        "inference_successes": "17900",
        "postprocess_successes": "17900",
        "video_frames_encoded": "18000",
        "camera_buffer_count_at_start": "6",
        "rga_process_successes": "17900",
        "mpp_encoded_frames": "18000",
        "h265_bytes_written": "500000000",
        "audio_timed_chunks": "28000",
        "audio_timed_frames": "28800000",
        "audio_encode_packets_pushed": "28000",
        "audio_encoded_sink_packets_written": "28000",
        "rss.growth_kb": "4096",
        "rss.enforced_growth_limit_kb": "65536",
        "network_mux_enabled": "1",
        "network_stop_ok": "1",
        "network_started": "1",
        "network_finalized": "1",
        "network_mux_ok": "1",
        "network_fatal_error": "0",
        "network_queue_overload_failures": "0",
        "network_write_failures": "0",
        "network_video_access_units_written": "18000",
        "network_audio_packets_written": "28000",
        "local_av_mux_enabled": "1",
        "local_av_finalize_ok": "1",
        "local_av_header_written": "1",
        "local_av_finalized": "1",
        "local_av_mux_ok": "1",
        "local_av_fatal_error": "0",
        "local_av_write_failures": "0",
        "local_av_video_samples_written": "18000",
        "local_av_audio_packets_written": "28000",
        "telemetry_enabled": "1",
        "telemetry_start_ok": "1",
        "telemetry_final_update_ok": "1",
        "telemetry_stop_ok": "1",
        "telemetry_started": "1",
        "telemetry_stopped_cleanly": "1",
        "telemetry_ok": "1",
        "telemetry_runtime_fault": "0",
        "telemetry_fatal_error": "0",
        "telemetry_send_failures": "0",
        "telemetry_serialization_failures": "0",
        "telemetry_oversized_datagrams": "0",
        "telemetry_control_updates": "5900",
        "telemetry_runtime_updates": "6000",
        "telemetry_datagrams_sent": "6001",
        "control_backend": "uart",
        "uart.adapter.rejected": "0",
        "uart.link_state_before_stop": "READY",
        "uart.peer_boot_id_valid": "1",
        "uart.hello_ack_received": "1",
        "uart.status_received": "500",
        "uart.control_sent": "17900",
        "uart.poll_errors": "0",
        "uart.read_errors": "0",
        "uart.write_errors": "0",
        "uart.message_decode_errors": "0",
        "uart.unexpected_responses": "0",
        "uart.parser_crc_errors": "0",
        "uart.parser_length_errors": "0",
        "uart.parser_version_errors": "0",
        "uart.parser_escape_errors": "0",
        "uart.parser_oversize_errors": "0",
    }
    for prefix in (
            "queue.captured", "queue.prepared", "queue.completed",
            "queue.video", "queue.encoded", "queue.audio_pcm",
            "queue.audio_encoded", "queue.network"):
        add_queue(report, prefix)
    return report


class V86IntegrationReportTest(unittest.TestCase):
    def setUp(self) -> None:
        self.expectations = Expectations()

    def assert_fails_with(self, report: dict[str, str], fragment: str) -> None:
        errors = validate_report(report, self.expectations)
        self.assertTrue(
            any(fragment in error for error in errors),
            msg=f"missing {fragment!r} in {errors!r}")

    def test_complete_report_passes(self) -> None:
        self.assertEqual(
            validate_report(passing_report(), self.expectations), [])

    def test_short_or_interrupted_run_fails(self) -> None:
        report = passing_report()
        report["observed_duration_seconds"] = "45.0"
        report["completed_requested_duration"] = "0"
        report["terminated_by_signal"] = "1"
        self.assert_fails_with(report, "observed_duration_seconds")

    def test_nonzero_lease_fails(self) -> None:
        report = passing_report()
        report["broker_outstanding_leases_after_stop"] = "1"
        self.assert_fails_with(report, "broker_outstanding_leases_after_stop")

    def test_queue_overflow_and_silent_drop_fail(self) -> None:
        report = passing_report()
        report["queue.video.high_watermark"] = "17"
        report["queue.network.replaced_oldest"] = "1"
        self.assert_fails_with(report, "exceeds capacity")
        self.assert_fails_with(report, "queue.network.replaced_oldest")

    def test_media_and_telemetry_faults_fail(self) -> None:
        for key in (
                "network_write_failures", "local_av_write_failures",
                "telemetry_send_failures"):
            with self.subTest(key=key):
                report = passing_report()
                report[key] = "1"
                self.assert_fails_with(report, key)

    def test_uart_protocol_error_fails(self) -> None:
        report = passing_report()
        report["uart.parser_crc_errors"] = "1"
        self.assert_fails_with(report, "uart.parser_crc_errors")


if __name__ == "__main__":
    unittest.main()
