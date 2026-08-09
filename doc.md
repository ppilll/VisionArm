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


ffprobe -v error \
  -f hevc \
  -show_entries stream=codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 \
  reports/v4_r5/fused_1x1_60s.h265

LD_LIBRARY_PATH=/mnt/nfs/visionarm-mpp-test/lib \
./vision_pipeline_r5_r6_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r5/fused_1x1_3s.h265 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --buffers 6 \
  --video-queue 2 \
  --bitrate 4000000 \
  --gop 60 \
  --duration-sec 3 \
  --topology fused \
  --input-slots 1 \
  --output-slots 1 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32 \
  --report reports/v4_r5/fused_1x1_3s.txt


/usr/bin/cmake  \
  -S visionram  \
  -B build/v4-r7 \
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
  --build build/v4-r7 \
  --parallel "$(nproc)"

cmake --build build/v4-r7 -j"$(nproc)"
ctest --test-dir build-r7-r8-board --output-on-failure

LD_LIBRARY_PATH=/mnt/nfs/visionarm-mpp-test/lib \
./vision_pipeline_r7_r8_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r8/smoke_60s.h265 \
  --report reports/v4_r8/smoke_60s.txt \
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
  --acquire-hits 2 \
  --lost-misses 3 \
  --max-result-age-ms 100 \
  --latency-samples 65536 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32

LD_LIBRARY_PATH=/mnt/nfs/visionarm-mpp-test/lib \
  ./vision_pipeline_r7_r8_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r8/fused_1x1_60fps_10min.h265 \
  --report reports/v4_r8/fused_1x1_60fps_10min.txt \
  --width 1280 --height 720 --fps 60 \
  --buffers 6 --video-queue 2 \
  --bitrate 8000000 --gop 120 \
  --duration-sec 600 \
  --topology fused \
  --input-slots 1 --output-slots 1 \
  --acquire-hits 2 --lost-misses 3 \
  --max-result-age-ms 100 \
  --latency-samples 65536 \
  --max-rss-growth-kb 32768 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32

LD_LIBRARY_PATH=/mnt/nfs/visionarm-mpp-test/lib \
  ./vision_pipeline_r7_r8_probe \
  --device /dev/video22 \
  --model model/best_i8.rknn \
  --output reports/v4_r8/fused_1x1_30fps_10min.h265 \
  --report reports/v4_r8/fused_1x1_30fps_10min.txt \
  --width 1280 --height 720 --fps 30 \
  --buffers 6 --video-queue 2 \
  --bitrate 4000000 --gop 60 \
  --duration-sec 600 \
  --topology fused \
  --input-slots 1 --output-slots 1 \
  --acquire-hits 2 --lost-misses 3 \
  --max-result-age-ms 100 \
  --latency-samples 65536 \
  --max-rss-growth-kb 32768 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32
  

  cmake -S . -B build-r6-l2-board \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DVISIONARM_BUILD_RUNTIME=ON \
  -DVISIONARM_BUILD_TESTS=OFF \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=OFF \
  -DVISIONARM_BUILD_ACCELERATION_TOOLS=ON \
  -DVISIONARM_ENABLE_OPENCV_PREPROCESS=OFF \
  -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
  -DVISIONARM_ENABLE_MPP_VIDEO=ON \
  -DVISIONARM_ENABLE_UART_CONTROL=ON \
  -DRKNN_INCLUDE_DIR="$RKNN_INCLUDE_DIR" \
  -DRKNN_LIBRARY="$RKNN_LIBRARY" \
  -DRGA_INCLUDE_DIR="$RGA_INCLUDE_DIR" \
  -DRGA_LIBRARY="$RGA_LIBRARY" \
  -DMPP_INCLUDE_DIR="$MPP_INCLUDE_DIR" \
  -DMPP_LIBRARY="$MPP_LIBRARY"

# run the unchanged mock backend
sudo ./build-r6-l2-board/vision_pipeline_r7_r8_probe \
  --device /dev/videoX \
  --model /actual/path/model.rknn \
  --output reports/v6/r6_l2/mock_smoke.h265 \
  --width YOUR_WIDTH \
  --height YOUR_HEIGHT \
  --fps YOUR_FPS \
  --bitrate YOUR_BITRATE \
  --gop YOUR_GOP \
  --duration-sec 60 \
  --control-backend mock \
  --report reports/v6/r6_l2/mock_smoke.txt

# Run the real UART backend
  sudo ./build-r6-l2-board/vision_pipeline_r7_r8_probe \
  --device /dev/videoX \
  --model /actual/path/model.rknn \
  --output reports/v6/r6_l2/uart_smoke.h265 \
  --width YOUR_WIDTH \
  --height YOUR_HEIGHT \
  --fps YOUR_FPS \
  --bitrate YOUR_BITRATE \
  --gop YOUR_GOP \
  --duration-sec 60 \
  --control-backend uart \
  --uart-device /dev/ttyS3 \
  --uart-baud 115200 \
  --uart-ready-timeout-ms 5000 \
  --report reports/v6/r6_l2/uart_smoke.txt
