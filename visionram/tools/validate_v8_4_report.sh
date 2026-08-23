#!/bin/sh
set -eu

MODE="${1:-steady}"
REPORT="${2:-}"

if [ -z "${REPORT}" ] || [ ! -f "${REPORT}" ]; then
    echo "usage: $0 steady|reconnect|simultaneous REPORT" >&2
    exit 2
fi

value() {
    key="$1"
    sed -n "s/^${key}=//p" "${REPORT}" | tail -n 1
}

require_eq() {
    key="$1"
    expected="$2"
    actual="$(value "${key}")"
    if [ "${actual}" != "${expected}" ]; then
        echo "FAIL: ${key}: expected ${expected}, got '${actual}'" >&2
        exit 1
    fi
}

require_gt_zero() {
    key="$1"
    actual="$(value "${key}")"
    case "${actual}" in
        ''|*[!0-9]*) echo "FAIL: ${key}: invalid '${actual}'" >&2; exit 1 ;;
    esac
    if [ "${actual}" -le 0 ]; then
        echo "FAIL: ${key}: expected > 0, got ${actual}" >&2
        exit 1
    fi
}

require_zero() {
    key="$1"
    require_eq "${key}" 0
}

require_eq network_av_mux_enabled 1
require_eq network_sink_fatal_error 0
require_eq network_sink_clean_stop 1
require_eq network_audio_queue_overload_faults 0
require_eq network_mux_finalize_failures 0
require_gt_zero network_connections_opened
require_gt_zero network_video_access_units_sent
require_gt_zero network_audio_packets_sent
require_eq network_av_mux_ok 1
require_eq vision_pipeline_r7_r8_probe PASS

case "${MODE}" in
    steady)
        require_zero network_disconnect_events
        require_zero network_reconnect_successes
        require_zero network_video_dropped_queue_full
        require_zero network_video_dropped_disconnected
        require_zero network_video_dropped_waiting_keyframe
        require_zero network_audio_dropped_disconnected
        ;;
    reconnect)
        require_gt_zero network_disconnect_events
        require_gt_zero network_reconnect_successes
        require_zero network_video_dropped_queue_full
        ;;
    simultaneous)
        require_eq local_av_mux_enabled 1
        require_eq local_av_mux_ok 1
        require_zero network_disconnect_events
        require_zero network_video_dropped_queue_full
        require_zero network_video_dropped_disconnected
        require_zero network_audio_dropped_disconnected
        ;;
    *)
        echo "usage: $0 steady|reconnect|simultaneous REPORT" >&2
        exit 2
        ;;
esac

echo "V8_4_REPORT_${MODE}=PASS"
