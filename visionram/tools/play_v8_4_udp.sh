#!/bin/sh
set -eu

PORT="${1:-5600}"
URL="udp://0.0.0.0:${PORT}?fifo_size=1000000&overrun_nonfatal=1"

if ! command -v ffplay >/dev/null 2>&1; then
    echo "error: ffplay not found" >&2
    exit 2
fi

echo "[V8.4] live playback: ${URL}"
exec ffplay \
    -hide_banner \
    -loglevel info \
    -probesize 4194304 \
    -analyzeduration 3000000 \
    -fflags nobuffer \
    -flags low_delay \
    -framedrop \
    "${URL}"
