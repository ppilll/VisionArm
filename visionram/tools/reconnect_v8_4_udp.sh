#!/bin/sh
set -eu

PORT="${1:-5600}"
FIRST_DURATION="${2:-10}"
GAP_DURATION="${3:-5}"
SECOND_WINDOW="${4:-15}"
PREFIX="${5:-v8_4_reconnect}"
URL="udp://0.0.0.0:${PORT}?fifo_size=1000000&overrun_nonfatal=1"
FIRST_OUTPUT="${PREFIX}_before.ts"
SECOND_OUTPUT="${PREFIX}_after.ts"

if ! command -v ffmpeg >/dev/null 2>&1 || ! command -v ffprobe >/dev/null 2>&1; then
    echo "error: ffmpeg and ffprobe are required" >&2
    exit 2
fi
if ! command -v timeout >/dev/null 2>&1; then
    echo "error: GNU timeout not found" >&2
    exit 2
fi

capture_first_segment() {
    rm -f "${FIRST_OUTPUT}"
    ffmpeg \
        -hide_banner \
        -loglevel warning \
        -probesize 4194304 \
        -analyzeduration 3000000 \
        -i "${URL}" \
        -map 0:v:0 \
        -map 0:a:0 \
        -c copy \
        -t "${FIRST_DURATION}" \
        -y "${FIRST_OUTPUT}"
}

capture_recovery_window() {
    rm -f "${SECOND_OUTPUT}"
    set +e
    timeout --signal=INT --kill-after=3 "${SECOND_WINDOW}" \
        ffmpeg \
            -hide_banner \
            -loglevel warning \
            -probesize 4194304 \
            -analyzeduration 3000000 \
            -i "${URL}" \
            -map 0:v:0 \
            -map 0:a:0 \
            -c copy \
            -y "${SECOND_OUTPUT}"
    status="$?"
    set -e
    case "${status}" in
        0|124|130) ;;
        *) echo "FAIL: recovery receiver exited with status ${status}" >&2; exit 1 ;;
    esac
}

validate_segment() {
    output="$1"
    video_codec="$(ffprobe -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "${output}")"
    audio_codec="$(ffprobe -v error -select_streams a:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "${output}")"
    audio_rate="$(ffprobe -v error -select_streams a:0 -show_entries stream=sample_rate -of default=nw=1:nk=1 "${output}")"
    audio_channels="$(ffprobe -v error -select_streams a:0 -show_entries stream=channels -of default=nw=1:nk=1 "${output}")"
    [ "${video_codec}" = "hevc" ] || { echo "FAIL: ${output}: video=${video_codec}" >&2; exit 1; }
    [ "${audio_codec}" = "aac" ] || { echo "FAIL: ${output}: audio=${audio_codec}" >&2; exit 1; }
    [ "${audio_rate}" = "48000" ] || { echo "FAIL: ${output}: rate=${audio_rate}" >&2; exit 1; }
    [ "${audio_channels}" = "2" ] || { echo "FAIL: ${output}: channels=${audio_channels}" >&2; exit 1; }
}

echo "[V8.4 reconnect] capture before disconnect: ${FIRST_DURATION}s"
echo "[V8.4 reconnect] start the RK3588 sender now"
capture_first_segment

echo "[V8.4 reconnect] receiver intentionally OFF for ${GAP_DURATION}s"
sleep "${GAP_DURATION}"

echo "[V8.4 reconnect] receiver back ON for a ${SECOND_WINDOW}s wall-clock window"
capture_recovery_window

validate_segment "${FIRST_OUTPUT}"
validate_segment "${SECOND_OUTPUT}"

echo "V8_4_RECONNECT_RECEIVER=PASS"
