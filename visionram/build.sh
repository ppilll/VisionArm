#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

readonly TARGET="visionarm_runtime"
readonly BUILD_DIR="${PROJECT_DIR}/build/rk3588"

BUILD_TYPE="Release"
JOBS="$(nproc)"
COPY_ENABLED=1
COPY_DIR="${VISIONARM_COPY_DIR:-/home/liu2004/nfs_dir}"
CLEAN=0

usage() {
    cat <<'USAGE'
Usage: ./build.sh [--clean] [--debug] [--jobs N] [--no-copy] [--copy-dir PATH]

Environment overrides:
  VISIONARM_TOOLCHAIN_ENV  Toolchain environment setup script
  VISIONARM_SDK            RK3588 Linux SDK root
  VISIONARM_SYSROOT        Cross-compilation sysroot
  VISIONARM_COPY_DIR       Default deployment directory
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            CLEAN=1
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --jobs)
            if [[ $# -lt 2 || ! "$2" =~ ^[1-9][0-9]*$ ]]; then
                echo "--jobs requires a positive integer" >&2
                exit 2
            fi
            JOBS="$2"
            shift 2
            ;;
        --no-copy)
            COPY_ENABLED=0
            shift
            ;;
        --copy-dir)
            if [[ $# -lt 2 || -z "$2" ]]; then
                echo "--copy-dir requires a path" >&2
                exit 2
            fi
            COPY_DIR="$2"
            COPY_ENABLED=1
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

TOOLCHAIN_ENV="${VISIONARM_TOOLCHAIN_ENV:-/opt/atk-dlrk3588-toolchain/environment-setup}"
SDK="${VISIONARM_SDK:-${HOME}/work/Linux_SDK/atk_dlrk3588_linux5.10}"
SYSROOT="${VISIONARM_SYSROOT:-/opt/atk-dlrk3588-toolchain/aarch64-buildroot-linux-gnu/sysroot}"

if [[ ! -f "$TOOLCHAIN_ENV" ]]; then
    echo "Toolchain environment script not found: $TOOLCHAIN_ENV" >&2
    exit 1
fi

if [[ "$CLEAN" -eq 1 ]]; then
    rm -rf -- "$BUILD_DIR"
fi

# shellcheck disable=SC1090
source "$TOOLCHAIN_ENV"

RKNN_INCLUDE_DIR="$SDK/external/rknpu2/runtime/Linux/librknn_api/include"
RKNN_LIBRARY="$SDK/external/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
RGA_INCLUDE_DIR="$SDK/external/rknpu2/examples/3rdparty/rga/include"
RGA_LIBRARY="$SDK/external/rknpu2/examples/3rdparty/rga/libs/Linux/gcc-aarch64/librga.so"
MPP_INCLUDE_DIR="$SDK/external/rknpu2/examples/3rdparty/mpp/include/rockchip"
MPP_LIBRARY="$SDK/external/rknpu2/examples/3rdparty/mpp/Linux/aarch64/librockchip_mpp.so.1"
FFMPEG_INCLUDE_DIR="$SYSROOT/usr/include"
FFMPEG_AVCODEC_LIBRARY="$SYSROOT/usr/lib/libavcodec.so"
FFMPEG_AVUTIL_LIBRARY="$SYSROOT/usr/lib/libavutil.so"
FFMPEG_AVFORMAT_LIBRARY="$SYSROOT/usr/lib/libavformat.so"

cmake \
    -S "$SCRIPT_DIR" \
    -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DVISIONARM_DEFAULT_LOG_LEVEL=INFO \
    -DVISIONARM_ENABLE_OPENCV_PREPROCESS=OFF \
    -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
    -DVISIONARM_ENABLE_MPP_VIDEO=ON \
    -DVISIONARM_ENABLE_ALSA_AUDIO=ON \
    -DVISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON \
    -DVISIONARM_ENABLE_FFMPEG_MP4_MUX=ON \
    -DVISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK=ON \
    -DVISIONARM_ENABLE_UART_CONTROL=ON \
    -DRKNN_INCLUDE_DIR:PATH="$RKNN_INCLUDE_DIR" \
    -DRKNN_LIBRARY:FILEPATH="$RKNN_LIBRARY" \
    -DRGA_INCLUDE_DIR:PATH="$RGA_INCLUDE_DIR" \
    -DRGA_LIBRARY:FILEPATH="$RGA_LIBRARY" \
    -DMPP_INCLUDE_DIR:PATH="$MPP_INCLUDE_DIR" \
    -DMPP_LIBRARY:FILEPATH="$MPP_LIBRARY" \
    -DFFMPEG_INCLUDE_DIR:PATH="$FFMPEG_INCLUDE_DIR" \
    -DFFMPEG_AVCODEC_LIBRARY:FILEPATH="$FFMPEG_AVCODEC_LIBRARY" \
    -DFFMPEG_AVUTIL_LIBRARY:FILEPATH="$FFMPEG_AVUTIL_LIBRARY" \
    -DFFMPEG_AVFORMAT_LIBRARY:FILEPATH="$FFMPEG_AVFORMAT_LIBRARY"

cmake --build "$BUILD_DIR" --target "$TARGET" --parallel "$JOBS"

BINARY="$BUILD_DIR/bin/$TARGET"
if [[ ! -x "$BINARY" ]]; then
    echo "Build completed without the expected executable: $BINARY" >&2
    exit 1
fi

echo "Build success: $BINARY"

if [[ "$COPY_ENABLED" -eq 1 ]]; then
    mkdir -p -- "$COPY_DIR"
    cp -- "$BINARY" "$COPY_DIR/$TARGET"
    echo "Copied to: $COPY_DIR/$TARGET"
fi
