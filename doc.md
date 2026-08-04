# 如何编译以及使用
交叉编译：

source /opt/atk-dlrk3588-toolchain/environment-setup
SDK="$HOME/work/Linux_SDK/atk_dlrk3588_linux5.10"

RKNN_INCLUDE_DIR="$SDK/external/rknpu2/runtime/Linux/librknn_api/include"
RKNN_LIBRARY="$SDK/external/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
DRGA_INCLUDE_DIR="$SDK/external/rknpu2/examples/3rdparty/rga/include"
DRGA_LIBRARY="$SDK/external/rknpu2/examples/3rdparty/rga/libs/Linux/gcc-aarch64/librga.so"
OpenCV_DIR="$SDK/external/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV"
DMPP_INCLUDE_DIR="$SDK/external/rknpu2/examples/3rdparty/mpp/include/rockchip"
DMPP_LIBRARY="$SDK/external/rknpu2/examples/3rdparty/mpp/Linux/aarch64/librockchip_mpp.so.1"

rm -rf build/v4-r5

/usr/bin/cmake \
  -S visionram  \
  -B build/v4-r5 \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISIONARM_BUILD_RUNTIME=ON \
  -DVISIONARM_BUILD_TESTS=ON \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=ON \
  -DVISIONARM_BUILD_ACCELERATION_TOOLS=ON \
  -DVISIONARM_ENABLE_OPENCV_PREPROCESS=ON \
  -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
  -DVISIONARM_ENABLE_MPP_VIDEO=ON \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/cmake/rk3588-toolchain.cmake" \
  -DRKNN_INCLUDE_DIR:PATH="$RKNN_INCLUDE_DIR" \
  -DRKNN_LIBRARY:FILEPATH="$RKNN_LIBRARY" \
  -DRGA_INCLUDE_DIR:PATH="$DRGA_INCLUDE_DIR" \
  -DRGA_LIBRARY:FILEPATH="$DRGA_LIBRARY" \
  -DOpenCV_DIR:PATH="$OpenCV_DIR" \
  -DMPP_INCLUDE_DIR:PATH="$DMPP_INCLUDE_DIR" \
  -DMPP_LIBRARY:FILEPATH="$DMPP_LIBRARY"

/usr/bin/cmake \
  --build build/v4-r5 \
  --parallel "$(nproc)"

cp -a build/v4-r5/vision_pipeline_r5_r6_probe /home/liu2004/nfs_dir/


./vision_pipeline_r5_r6_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r5/fused_1x1_60s.h265 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --buffers 6 \
  --video-queue 2 \
  --bitrate 4000000 \
  --gop 60 \
  --duration-sec 60 \
  --topology fused \
  --input-slots 1 \
  --output-slots 1 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32 \
  --report reports/v4_r5/fused_1x1_60s.txt

ffprobe -v error \
  -f hevc \
  -show_entries stream=codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 \
  reports/v4_r5/fused_1x1_60s.h265

LD_LIBRARY_PATH=/mnt/nfs/visionarm-mpp-test/lib \
./vision_pipeline_r5_r6_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r5/fused_1x1_60s.h265 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --buffers 6 \
  --video-queue 2 \
  --bitrate 4000000 \
  --gop 60 \
  --duration-sec 60 \
  --topology fused \
  --input-slots 1 \
  --output-slots 1 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32 \
  --report reports/v4_r5/fused_1x1_60s.txt

LD_LIBRARY_PATH=/mnt/nfs/visionarm-mpp-test/lib \
./vision_pipeline_r5_r6_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r6/D_split_2x2.txt \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --buffers 6 \
  --video-queue 2 \
  --bitrate 4000000 \
  --gop 60 \
  --duration-sec 120 \
  --topology split \
  --input-slots 2 \
  --output-slots 2 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32 \
  --report reports/v4_r6/D_split_2x2.txt