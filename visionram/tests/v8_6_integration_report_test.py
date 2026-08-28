#!/usr/bin/env python3
"""Unit tests for the dependency-free V8.6 report validator."""

from __future__ import annotations

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from validate_v8_6_integration_report import (  # noqa: E402
    Expectations,
    parse_report_text,
    validate_report,
)


LATENCY_NAMES = (
    "input_slot_wait", "latest_frame_queue_wait",
    "capture_to_preprocess_start", "preprocess", "rknn_input_submit",
    "rknn_output_bind", "rknn_bind_total", "rknn_run",
    "rknn_output_get", "rknn_output_release", "rknn_total",
    "postprocess", "capture_to_result", "result_age",
)


def add_queue(
        report: dict[str, str], prefix: str, count: int = 18000) -> None:
    report.update({
        f"{prefix}.capacity": "64",
        f"{prefix}.high_watermark": "8",
        f"{prefix}.current_size": "0",
        f"{prefix}.pushed": str(count),
        f"{prefix}.popped": str(count),
        f"{prefix}.replaced_oldest": "0",
        f"{prefix}.stopped": "1",
    })


def add_latency(
        report: dict[str, str], prefix: str, samples: int = 18000) -> None:
    report.update({
        f"{prefix}.total_samples": str(samples),
        f"{prefix}.retained_samples": str(samples),
        f"{prefix}.truncated": "0",
        f"{prefix}.mean_ms": "1.000",
        f"{prefix}.p50_ms": "0.800",
        f"{prefix}.p95_ms": "1.200",
        f"{prefix}.p99_ms": "1.500",
        f"{prefix}.maximum_ms": "2.000",
    })


def passing_report() -> dict[str, str]:
    report = {
        "schema": "visionarm.runtime_report.v1",
        "schema_version": "1",
        "report_level": "diagnostic",
        "value_encoding": "backslash-v1",
        "result": "PASS",
        "vision_pipeline_r7_r8_probe": "PASS",
        "topology": "fused_npu_postprocess",
        "module.camera.enabled": "1",
        "module.inference.enabled": "1",
        "module.video.enabled": "1",
        "module.audio.enabled": "1",
        "module.audio_encoder.enabled": "1",
        "module.recorder.enabled": "1",
        "module.network.enabled": "1",
        "module.telemetry.enabled": "1",
        "module.uart.enabled": "1",
        "requested_duration_seconds": "600.000",
        "observed_duration_seconds": "600.000",
        "completed_requested_duration": "1",
        "terminated_by_signal": "0",
        "auxiliary_media_runtime_fault": "0",
        "fatal_error": "0",
        "graceful_shutdown_completed": "1",
        "split_final_completed_frame_drained": "1",
        "camera_outstanding_before_stop": "0",
        "broker_outstanding_frames_before_camera_stop": "0",
        "broker_outstanding_leases_before_camera_stop": "0",
        "broker_outstanding_frames_after_stop": "0",
        "broker_outstanding_leases_after_stop": "0",
        "control_ok": "1",
        "camera_fps": "30.000",
        "inference_fps": "30.000",
        "postprocess_fps": "30.000",
        "video_fps": "30.000",
        "captured_frames": "18000",
        "inference_successes": "18000",
        "postprocess_successes": "18000",
        "video_frames_encoded": "18000",
        "camera_buffer_count_at_start": "6",
        "camera_timeouts": "0",
        "driver_dropped_frames": "0",
        "replaced_waiting_frames": "0",
        "skipped_no_input_slot": "0",
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
        "mpp_encode_failures": "0",
        "mpp_source_reimports": "0",
        "h265_write_failures": "0",
        "rga_process_calls": "18000",
        "rga_process_successes": "18000",
        "mpp_encoded_frames": "18000",
        "h265_bytes_written": "500000000",
        "state_processed_packets": "18000",
        "state_valid_controls": "12000",
        "state_control_sink_failures": "0",
        "state_perception_sink_failures": "0",
        "state_invalid_timestamp_packets": "0",
        "audio_enabled": "1",
        "encoded_audio_enabled": "1",
        "audio_worker_started": "1",
        "audio_worker_fatal_error": "0",
        "audio_queue_push_failures": "0",
        "audio_xruns": "0",
        "audio_suspends": "0",
        "audio_recoveries": "0",
        "audio_short_reads": "0",
        "audio_status_errors": "0",
        "audio_rate_hz": "48000",
        "audio_channels": "2",
        "audio_timed_chunks": "28125",
        "audio_timed_frames": "28800000",
        "audio_timed_bytes": "115200000",
        "audio_observed_rate_hz": "48000.000",
        "audio_timeline_chunks": "28125",
        "audio_timeline_frames": "28800000",
        "audio_timeline_bytes": "115200000",
        "audio_timeline_reanchors": "1",
        "audio_timeline_discontinuities": "0",
        "audio_timeline_pts_regressions": "0",
        "audio_timeline_continuous_pts_mismatches": "0",
        "audio_timeline_first_pts_ns": "1000000",
        "audio_timeline_last_pts_ns": "599000000000",
        "audio_timeline_last_end_pts_ns": "600000000000",
        "audio_timeline_max_forward_gap_ns": "0",
        "media_audio_anchor_valid": "1",
        "media_audio_chunks_stamped": "28126",
        "media_audio_reanchors": "1",
        "media_audio_discontinuities": "0",
        "media_audio_timestamp_failures": "0",
        "media_audio_last_pts_ns": "599000000000",
        "media_audio_last_end_pts_ns": "600000000000",
        "audio_path_ok": "1",
        "audio_encode_worker_started": "1",
        "audio_encode_worker_fatal_error": "0",
        "audio_encode_chunks_consumed": "28125",
        "audio_encode_packets_pushed": "28126",
        "audio_encode_bytes_pushed": "9600000",
        "audio_encode_queue_push_failures": "0",
        "audio_encoder_input_frames": "28800000",
        "audio_encoder_submitted_codec_frames": "28125",
        "audio_encoder_emitted_packets": "28126",
        "audio_encoder_emitted_bytes": "9600000",
        "audio_encoder_padding_packets": "1",
        "audio_encoder_failures": "0",
        "audio_encoder_buffered_input_frames": "0",
        "audio_encoder_drained": "1",
        "audio_encoded_sink_started": "1",
        "audio_encoded_sink_fatal_error": "0",
        "audio_encoded_sink_packets_written": "28126",
        "audio_encoded_sink_bytes_written": "9600000",
        "audio_encoded_sink_failures": "0",
        "audio_encode_path_ok": "1",
        "rss.samples": "602",
        "rss.first_kb": "48000",
        "rss.last_kb": "44096",
        "rss.minimum_kb": "40000",
        "rss.maximum_kb": "64000",
        "rss.growth_kb": "-3904",
        "rss.enforced_growth_limit_kb": "65536",
        "network_mux_enabled": "1",
        "network_stop_ok": "1",
        "network_started": "1",
        "network_finalized": "1",
        "network_mux_ok": "1",
        "network_fatal_error": "0",
        "network_queue_overload_failures": "0",
        "network_write_failures": "0",
        "network_video_fragments_received": "18000",
        "network_video_access_units_enqueued": "18000",
        "network_video_access_units_written": "18000",
        "network_video_bytes_written": "500000000",
        "network_audio_packets_enqueued": "28126",
        "network_audio_packets_written": "28126",
        "network_audio_bytes_written": "9600000",
        "network_first_video_pts_us": "1000",
        "network_last_video_pts_us": "599000000",
        "network_first_audio_pts_ns": "1000000",
        "network_last_audio_pts_ns": "599000000000",
        "local_av_mux_enabled": "1",
        "local_av_finalize_ok": "1",
        "local_av_header_written": "1",
        "local_av_finalized": "1",
        "local_av_mux_ok": "1",
        "local_av_fatal_error": "0",
        "local_av_write_failures": "0",
        "local_av_video_fragments_received": "18000",
        "local_av_video_samples_written": "18000",
        "local_av_video_bytes_written": "500000000",
        "local_av_audio_packets_written": "28126",
        "local_av_audio_bytes_written": "9600000",
        "local_av_first_video_pts_us": "1000",
        "local_av_last_video_pts_us": "599000000",
        "local_av_first_audio_pts_ns": "1000000",
        "local_av_last_audio_pts_ns": "599000000000",
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
        "telemetry_control_updates": "12000",
        "telemetry_runtime_updates": "6000",
        "telemetry_datagrams_attempted": "6001",
        "telemetry_datagrams_sent": "6001",
        "telemetry_bytes_sent": "1200000",
        "telemetry_final_sequence": "6001",
        "control_backend": "uart",
        "uart.adapter.submissions": "18000",
        "uart.adapter.accepted": "18000",
        "uart.adapter.rejected": "0",
        "uart.adapter.valid_inputs": "12000",
        "uart.adapter.invalid_inputs": "6000",
        "uart.adapter.nonfinite_invalidations": "0",
        "uart.adapter.invalid_timestamp_invalidations": "0",
        "uart.adapter.invalid_state_invalidations": "0",
        "uart.adapter.identity_truncations": "0",
        "uart.link_state_before_stop": "READY",
        "uart.peer_boot_id_valid": "1",
        "uart.tx_bytes": "3200000",
        "uart.rx_bytes": "640000",
        "uart.hello_ack_received": "1",
        "uart.status_received": "500",
        "uart.control_accepted": "18000",
        "uart.control_overwritten": "100",
        "uart.control_sent": "17900",
        "uart.valid_control_sent": "11900",
        "uart.invalid_control_sent": "6000",
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
        "log.queue_capacity": "1024",
        "log.critical_queue_capacity": "32",
        "log.current_size": "0",
        "log.high_watermark": "4",
        "log.accepted": "20",
        "log.emitted": "20",
        "log.dropped": "1",
        "log.dropped_contention": "1",
        "log.dropped_overflow": "0",
        "log.dropped_critical": "0",
        "log.sink_failures": "0",
        "log.flush_ok": "1",
    }
    for prefix in (
            "queue.captured", "queue.prepared", "queue.video",
            "queue.encoded"):
        add_queue(report, prefix)
    add_queue(report, "queue.audio_pcm", 28125)
    add_queue(report, "queue.audio_encoded", 28126)
    add_queue(report, "queue.network", 46126)
    for name in LATENCY_NAMES:
        add_latency(report, f"latency.{name}")
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

    def test_split_topology_requires_and_accepts_completed_queue(self) -> None:
        report = passing_report()
        report["topology"] = "split_npu_postprocess"
        self.assert_fails_with(report, "queue.completed.capacity")
        add_queue(report, "queue.completed")
        self.assertEqual(validate_report(report, self.expectations), [])

    def test_fused_topology_rejects_completed_queue(self) -> None:
        report = passing_report()
        add_queue(report, "queue.completed")
        self.assert_fails_with(report, "must omit queue.completed")

    def test_summary_uses_common_contract_only(self) -> None:
        report = passing_report()
        report["report_level"] = "summary"
        for key in list(report):
            if key.startswith("latency."):
                del report[key]
            elif key.startswith("queue.") and not key.endswith((
                    ".current_size", ".replaced_oldest", ".stopped")):
                del report[key]
        self.assertEqual(
            validate_report(report, self.expectations), [])

    def test_summary_rejects_undrained_queue(self) -> None:
        report = passing_report()
        report["report_level"] = "summary"
        report["queue.audio_pcm.current_size"] = "1"
        self.assert_fails_with(report, "queue.audio_pcm.current_size")

    def test_disabled_modules_do_not_require_detail(self) -> None:
        report = passing_report()
        report["report_level"] = "performance"
        for module in ("audio", "audio_encoder", "recorder", "network",
                       "telemetry", "uart"):
            report[f"module.{module}.enabled"] = "0"
        report["control_backend"] = "mock"
        expectations = Expectations(
            audio="disabled", network="disabled", recording="disabled",
            telemetry="disabled", control="mock")
        self.assertEqual(validate_report(report, expectations), [])

    def test_audio_capture_without_encoded_outputs(self) -> None:
        report = passing_report()
        report["report_level"] = "performance"
        report["module.audio_encoder.enabled"] = "0"
        report["module.recorder.enabled"] = "0"
        report["module.network.enabled"] = "0"
        for key in list(report):
            if key.startswith((
                    "audio_encode_", "audio_encoder_",
                    "audio_encoded_sink_", "queue.audio_encoded")):
                del report[key]
        expectations = Expectations(network="disabled", recording="disabled")
        self.assertEqual(validate_report(report, expectations), [])

    def test_schema_parser_unescapes_and_rejects_duplicates(self) -> None:
        parsed = parse_report_text(
            "schema=visionarm.runtime_report.v1\n"
            "value_encoding=backslash-v1\n"
            "fatal_message=disk\\=full\\nretry\n")
        self.assertEqual(parsed["fatal_message"], "disk=full\nretry")
        with self.assertRaisesRegex(ValueError, "duplicate"):
            parse_report_text("result=PASS\nresult=FAIL\n")

    def test_fault_report_fails_acceptance(self) -> None:
        report = passing_report()
        report["report_level"] = "summary"
        report["result"] = "FAIL"
        report["fatal_error"] = "1"
        report["completed_requested_duration"] = "0"
        self.assert_fails_with(report, "result")

    def test_short_or_interrupted_run_fails(self) -> None:
        report = passing_report()
        report["observed_duration_seconds"] = "45.0"
        report["completed_requested_duration"] = "0"
        report["terminated_by_signal"] = "1"
        self.assert_fails_with(report, "observed_duration_seconds")

    def test_wrong_fps_fails(self) -> None:
        report = passing_report()
        report["inference_fps"] = "29.0"
        self.assert_fails_with(report, "inference_fps")

    def test_latency_missing_truncated_or_unordered_fails(self) -> None:
        cases = (
            ("latency.rknn_run.p99_ms", None, "missing"),
            ("latency.rknn_run.truncated", "1", "truncated"),
            ("latency.rknn_run.p99_ms", "3.0", "not ordered"),
        )
        for key, value, fragment in cases:
            with self.subTest(key=key, value=value):
                report = passing_report()
                if value is None:
                    del report[key]
                else:
                    report[key] = value
                self.assert_fails_with(report, fragment)

    def test_frame_counter_mismatch_fails(self) -> None:
        report = passing_report()
        report["postprocess_successes"] = "17999"
        self.assert_fails_with(report, "postprocess_successes")

    def test_nonzero_lease_fails(self) -> None:
        report = passing_report()
        report["broker_outstanding_leases_after_stop"] = "1"
        self.assert_fails_with(report, "broker_outstanding_leases_after_stop")

    def test_queue_not_stopped_unbalanced_or_overflow_fails(self) -> None:
        cases = (
            ("queue.video.stopped", "0", "expected 1"),
            ("queue.video.popped", "17999", "popped+replaced_oldest"),
            ("queue.video.high_watermark", "65", "exceeds capacity"),
            ("queue.network.replaced_oldest", "1", "no replacement"),
        )
        for key, value, fragment in cases:
            with self.subTest(key=key):
                report = passing_report()
                report[key] = value
                self.assert_fails_with(report, fragment)

    def test_audio_timeline_discontinuity_fails(self) -> None:
        report = passing_report()
        report["audio_timeline_discontinuities"] = "1"
        self.assert_fails_with(report, "audio_timeline_discontinuities")

    def test_aac_counter_mismatch_fails(self) -> None:
        report = passing_report()
        report["audio_encoder_emitted_packets"] = "28125"
        self.assert_fails_with(report, "audio_encoder_emitted_packets")

    def test_mux_counter_mismatch_fails(self) -> None:
        for key in (
                "local_av_video_samples_written",
                "local_av_audio_bytes_written",
                "network_video_access_units_written",
                "network_audio_bytes_written"):
            with self.subTest(key=key):
                report = passing_report()
                report[key] = "17999"
                self.assert_fails_with(report, key)

    def test_logger_drain_and_accounting_fail(self) -> None:
        for key, value, fragment in (
                ("log.current_size", "1", "drained"),
                ("log.emitted", "19", "expected accepted")):
            with self.subTest(key=key):
                report = passing_report()
                report[key] = value
                self.assert_fails_with(report, fragment)

    def test_rss_growth_mismatch_fails(self) -> None:
        report = passing_report()
        report["rss.growth_kb"] = "1"
        self.assert_fails_with(report, "expected last-first")

    def test_uart_sum_mismatch_fails(self) -> None:
        report = passing_report()
        report["uart.invalid_control_sent"] = "5999"
        self.assert_fails_with(report, "uart.control_sent")

    def test_media_and_telemetry_faults_fail(self) -> None:
        for key in ("network_write_failures", "local_av_write_failures",
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
