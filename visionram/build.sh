#!/usr/bin/env bash

set -e

TARGET="vision_pipeline_r7_r8_probe"
BUILD_DIR="build/v8"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --target)
            TARGET="$2"
            shift 2
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

source /opt/atk-dlrk3588-toolchain/environment-setup

SDK="$HOME/work/Linux_SDK/atk_dlrk3588_linux5.10"

SYSROOT="/opt/atk-dlrk3588-toolchain/aarch64-buildroot-linux-gnu/sysroot"

RKNN_INCLUDE_DIR="$SDK/external/rknpu2/runtime/Linux/librknn_api/include"
RKNN_LIBRARY="$SDK/external/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"

RGA_INCLUDE_DIR="$SDK/external/rknpu2/examples/3rdparty/rga/include"
RGA_LIBRARY="$SDK/external/rknpu2/examples/3rdparty/rga/libs/Linux/gcc-aarch64/librga.so"

OpenCV_DIR="$SDK/external/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV"

MPP_INCLUDE_DIR="$SDK/external/rknpu2/examples/3rdparty/mpp/include/rockchip"
MPP_LIBRARY="$SDK/external/rknpu2/examples/3rdparty/mpp/Linux/aarch64/librockchip_mpp.so.1"

FFMPEG_INCLUDE_DIR="$SYSROOT/usr/include"
FFMPEG_AVCODEC_LIBRARY="$SYSROOT/usr/lib/libavcodec.so"
FFMPEG_AVUTIL_LIBRARY="$SYSROOT/usr/lib/libavutil.so"
FFMPEG_AVFORMAT_LIBRARY="$SYSROOT/usr/lib/libavformat.so"

echo "TARGET    = $TARGET"
echo "BUILD_DIR = $BUILD_DIR"

rm -rf "$BUILD_DIR"

/usr/bin/cmake \
  -S visionram \
  -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISIONARM_BUILD_RUNTIME=ON \
  -DVISIONARM_BUILD_TESTS=ON \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=ON \
  -DVISIONARM_BUILD_ACCELERATION_TOOLS=ON \
  -DVISIONARM_ENABLE_OPENCV_PREPROCESS=ON \
  -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
  -DVISIONARM_ENABLE_MPP_VIDEO=ON \
  -DVISIONARM_ENABLE_UART_CONTROL=ON \
  -DVISIONARM_ENABLE_ALSA_AUDIO=ON \
  -DVISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON \
  -DVISIONARM_ENABLE_FFMPEG_MP4_MUX=ON \
  -DVISIONARM_ENABLE_FFMPEG_MPEGTS_UDP=ON \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/cmake/rk3588-toolchain.cmake" \
  -DRKNN_INCLUDE_DIR:PATH="$RKNN_INCLUDE_DIR" \
  -DRKNN_LIBRARY:FILEPATH="$RKNN_LIBRARY" \
  -DRGA_INCLUDE_DIR:PATH="$RGA_INCLUDE_DIR" \
  -DRGA_LIBRARY:FILEPATH="$RGA_LIBRARY" \
  -DOpenCV_DIR:PATH="$OpenCV_DIR" \
  -DMPP_INCLUDE_DIR:PATH="$MPP_INCLUDE_DIR" \
  -DMPP_LIBRARY:FILEPATH="$MPP_LIBRARY" \
  -DFFMPEG_INCLUDE_DIR:PATH="$FFMPEG_INCLUDE_DIR" \
  -DFFMPEG_AVCODEC_LIBRARY:FILEPATH="$FFMPEG_AVCODEC_LIBRARY" \
  -DFFMPEG_AVUTIL_LIBRARY:FILEPATH="$FFMPEG_AVUTIL_LIBRARY" \
  -DFFMPEG_AVFORMAT_LIBRARY:FILEPATH="$FFMPEG_AVFORMAT_LIBRARY"

/usr/bin/cmake \
  --build "$BUILD_DIR" \
  --target "$TARGET" \
  --parallel "$(nproc)"

cp "$BUILD_DIR/$TARGET" /home/liu2004/nfs_dir/

echo
echo "Build success:"
echo "  $BUILD_DIR/$TARGET"
echo
echo "Copied to:"
echo "  /home/liu2004/nfs_dir/$TARGET"