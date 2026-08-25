#!/bin/sh
set -eu

usage() {
    cat >&2 <<'EOF'
Usage:
  receive_v8_4_mpegts.sh play [PORT]
  receive_v8_4_mpegts.sh play-low-latency [PORT]
  receive_v8_4_mpegts.sh capture OUTPUT.ts DURATION_SECONDS [PORT]

Use play for qualification. Only try play-low-latency after play and the raw
TS continuity test pass. Start the receiver before the sender. Default: 5000.
EOF
    exit 2
}

[ "$#" -ge 1 ] || usage
MODE=$1

case "$MODE" in
    play)
        [ "$#" -le 2 ] || usage
        PORT=${2:-5000}
        command -v ffplay >/dev/null 2>&1 || {
            echo "ffplay not found" >&2
            exit 1
        }
        URL="udp://0.0.0.0:${PORT}?fifo_size=65536&overrun_nonfatal=0&buffer_size=4194304"
        echo "listening=$URL"
        exec ffplay -hide_banner -probesize 20000000 \
            -analyzeduration 10000000 "$URL"
        ;;
    play-low-latency)
        [ "$#" -le 2 ] || usage
        PORT=${2:-5000}
        command -v ffplay >/dev/null 2>&1 || {
            echo "ffplay not found" >&2
            exit 1
        }
        URL="udp://0.0.0.0:${PORT}?fifo_size=65536&overrun_nonfatal=0&buffer_size=4194304"
        echo "listening=$URL"
        exec ffplay -hide_banner -fflags nobuffer -flags low_delay \
            -framedrop -probesize 5000000 -analyzeduration 5000000 "$URL"
        ;;
    capture)
        [ "$#" -ge 3 ] && [ "$#" -le 4 ] || usage
        OUTPUT=$2
        DURATION=$3
        PORT=${4:-5000}
        command -v ffmpeg >/dev/null 2>&1 || {
            echo "ffmpeg not found" >&2
            exit 1
        }
        URL="udp://0.0.0.0:${PORT}?fifo_size=65536&overrun_nonfatal=0&buffer_size=4194304"
        echo "listening=$URL"
        echo "capture_output=$OUTPUT"
        exec ffmpeg -hide_banner -fflags +genpts -i "$URL" \
            -t "$DURATION" -map 0:v:0 -map 0:a:0 -c copy -y "$OUTPUT"
        ;;
    *)
        usage
        ;;
esac
