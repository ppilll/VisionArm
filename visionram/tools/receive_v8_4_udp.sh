#!/bin/sh

set -eu

PORT="${1:-5600}"
WINDOW_SECONDS="${2:-30}"
OUTPUT="${3:-v8_4_udp_capture.ts}"

URL="udp://0.0.0.0:${PORT}?fifo_size=1000000&overrun_nonfatal=1"

#
# Dependencies
#
if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "error: ffmpeg not found" >&2
    exit 2
fi

if ! command -v ffprobe >/dev/null 2>&1; then
    echo "error: ffprobe not found" >&2
    exit 2
fi

if ! command -v timeout >/dev/null 2>&1; then
    echo "error: GNU timeout not found" >&2
    exit 2
fi

#
# Basic argument validation
#
case "${PORT}" in
    ''|*[!0-9]*)
        echo "error: PORT must be an integer" >&2
        exit 2
        ;;
esac

case "${WINDOW_SECONDS}" in
    ''|*[!0-9]*)
        echo "error: WINDOW_SECONDS must be an integer" >&2
        exit 2
        ;;
esac

#
# Print receiver configuration
#
echo "[V8.4] listening: ${URL}"
echo "[V8.4] receiver wall-clock window: ${WINDOW_SECONDS}s"
echo "[V8.4] keep this window longer than the RK3588 sender run"
echo "[V8.4] output: ${OUTPUT}"
echo "[V8.4] start the RK3588 sender now"

rm -f "${OUTPUT}"

#
# Important:
#
#   --foreground
#       Keep ffmpeg in the foreground process group so terminal/job-control
#       does not suspend it unexpectedly.
#
#   -nostdin
#       Disable ffmpeg console input handling. This is important when ffmpeg
#       is launched through timeout or another non-interactive test harness.
#
#   timeout sends SIGINT so ffmpeg gets an opportunity to close/finalize
#   the MPEG-TS output cleanly.
#
set +e

timeout \
    --foreground \
    --signal=INT \
    --kill-after=3 \
    "${WINDOW_SECONDS}" \
    ffmpeg \
        -nostdin \
        -hide_banner \
        -loglevel info \
        -probesize 4194304 \
        -analyzeduration 3000000 \
        -i "${URL}" \
        -map 0:v:0 \
        -map 0:a:0 \
        -c copy \
        -f mpegts \
        -y "${OUTPUT}"

CAPTURE_STATUS="$?"

set -e

#
# GNU timeout normally returns:
#
#   0   -> ffmpeg exited normally before timeout
#   124 -> timeout expired and signal was sent
#
# 124 itself is NOT proof that anything was captured.
#
case "${CAPTURE_STATUS}" in
    0)
        echo "[V8.4] ffmpeg exited normally"
        ;;
    124)
        echo "[V8.4] receiver wall-clock window expired normally"
        ;;
    137)
        echo "FAIL: ffmpeg required SIGKILL after timeout" >&2
        exit 1
        ;;
    *)
        echo "FAIL: ffmpeg receiver exited with status ${CAPTURE_STATUS}" >&2
        exit 1
        ;;
esac

#
# Do not call a timeout a successful capture unless a real file exists.
#
if [ ! -f "${OUTPUT}" ]; then
    echo "FAIL: UDP capture produced no output file: ${OUTPUT}" >&2
    echo "FAIL: verify that UDP/${PORT} was bound and packets reached ffmpeg" >&2
    exit 1
fi

if [ ! -s "${OUTPUT}" ]; then
    echo "FAIL: UDP capture output is empty: ${OUTPUT}" >&2
    exit 1
fi

OUTPUT_SIZE="$(wc -c < "${OUTPUT}")"

echo "[V8.4] capture complete"
echo "[V8.4] output size: ${OUTPUT_SIZE} bytes"
echo "[V8.4] probing streams"

#
# Show the streams first for diagnostic purposes.
#
if ! ffprobe \
    -v error \
    -show_entries \
stream=index,codec_name,codec_type,width,height,avg_frame_rate,sample_rate,channels \
    -of default=noprint_wrappers=1 \
    "${OUTPUT}"
then
    echo "FAIL: ffprobe could not parse ${OUTPUT}" >&2
    exit 1
fi

#
# Extract expected stream properties.
#
VIDEO_CODEC="$(
    ffprobe \
        -v error \
        -select_streams v:0 \
        -show_entries stream=codec_name \
        -of default=nw=1:nk=1 \
        "${OUTPUT}"
)"

AUDIO_CODEC="$(
    ffprobe \
        -v error \
        -select_streams a:0 \
        -show_entries stream=codec_name \
        -of default=nw=1:nk=1 \
        "${OUTPUT}"
)"

AUDIO_RATE="$(
    ffprobe \
        -v error \
        -select_streams a:0 \
        -show_entries stream=sample_rate \
        -of default=nw=1:nk=1 \
        "${OUTPUT}"
)"

AUDIO_CHANNELS="$(
    ffprobe \
        -v error \
        -select_streams a:0 \
        -show_entries stream=channels \
        -of default=nw=1:nk=1 \
        "${OUTPUT}"
)"

#
# Validate codecs and audio configuration.
#
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

#
# Verify packet DTS monotonicity.
#
check_monotonic_dts() {
    selector="$1"
    label="$2"

    if ! ffprobe \
        -v error \
        -select_streams "${selector}" \
        -show_entries packet=dts_time \
        -of csv=p=0 \
        "${OUTPUT}" |
        awk '
            $1 != "N/A" && $1 != "" {
                value = $1 + 0.0;

                if (have && value + 0.000001 < last) {
                    exit 1;
                }

                last = value;
                have = 1;
            }

            END {
                if (!have) {
                    exit 2;
                }
            }
        '
    then
        echo "FAIL: ${label} packet DTS is missing or regressed" >&2
        exit 1
    fi
}

check_monotonic_dts v:0 video
check_monotonic_dts a:0 audio

echo "V8_4_UDP_DTS=PASS"
echo "V8_4_UDP_CAPTURE=PASS"