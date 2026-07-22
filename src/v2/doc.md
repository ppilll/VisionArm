交叉编译：
cmake -S src/linux_app/camera \
      -B build/v2_camera-cross \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE=<TOOLCHAIN_FILE>

cmake --build build/v2_camera-cross -j"$(nproc)"

./build/v2_camera/v4l2_capture \
  --device <VIDEO_DEVICE> \
  --width <WIDTH> \
  --height <HEIGHT> \
  --format <FOURCC> \
  --fps <FPS> \
  --buffers 4 \
  --frames 300 \
  --timeout-ms 2000 \
  --output-dir reports/images/v2 \
  --save-first 1 \
  --csv logs/v2_camera/frames.csv \
  --nonblock \
  | tee logs/v2_camera/smoke_test.log