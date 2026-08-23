#!/bin/sh
set -eu

PORT="${1:-5600}"
DURATION="${2:-15}"
OUTPUT="${3:-v8_4_udp_capture.ts}"

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "error: ffmpeg not found" >&2
    exit 2
fi
if ! command -v ffprobe >/dev/null 2>&1; then
    echo "error: ffprobe not found" >&2
    exit 2
fi

URL="udp://0.0.0.0:${PORT}?fifo_size=1000000&overrun_nonfatal=1"

echo "[V8.4] listening: ${URL}"
echo "[V8.4] capture duration: ${DURATION}s"
echo "[V8.4] output: ${OUTPUT}"
echo "[V8.4] start the RK3588 sender now"

rm -f "${OUTPUT}"
ffmpeg \
    -hide_banner \
    -loglevel info \
    -probesize 4194304 \
    -analyzeduration 3000000 \
    -i "${URL}" \
    -map 0:v:0 \
    -map 0:a:0 \
    -c copy \
    -t "${DURATION}" \
    -y "${OUTPUT}"

echo "[V8.4] captured; probing streams"
ffprobe \
    -v error \
    -show_entries \
stream=index,codec_name,codec_type,width,height,avg_frame_rate,sample_rate,channels \
    -of default=noprint_wrappers=1 \
    "${OUTPUT}"

VIDEO_CODEC="$(ffprobe -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "${OUTPUT}")"
AUDIO_CODEC="$(ffprobe -v error -select_streams a:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "${OUTPUT}")"
AUDIO_RATE="$(ffprobe -v error -select_streams a:0 -show_entries stream=sample_rate -of default=nw=1:nk=1 "${OUTPUT}")"
AUDIO_CHANNELS="$(ffprobe -v error -select_streams a:0 -show_entries stream=channels -of default=nw=1:nk=1 "${OUTPUT}")"

if [ "${VIDEO_CODEC}" != "hevc" ]; then
    echo "FAIL: expected HEVC video, got '${VIDEO_CODEC}'" >&2
    exit 1
fi
if [ "${AUDIO_CODEC}" != "aac" ]; then
    echo "FAIL: expected AAC audio, got '${AUDIO_CODEC}'" >&2
    exit 1
fi
if [ "${AUDIO_RATE}" != "48000" ]; then
    echo "FAIL: expected AAC sample_rate=48000, got '${AUDIO_RATE}'" >&2
    exit 1
fi
if [ "${AUDIO_CHANNELS}" != "2" ]; then
    echo "FAIL: expected AAC channels=2, got '${AUDIO_CHANNELS}'" >&2
    exit 1
fi

echo "V8_4_UDP_CAPTURE=PASS"
