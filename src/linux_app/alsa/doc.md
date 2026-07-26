# 如何编译以及使用
交叉编译：
source /opt/atk-dlrk3588-toolchain/environment-setup
/usr/bin/cmake \
  -S src/linux_app/audio \
  -B build/v2a_audio

/usr/bin/cmake \
  --build build/v2a_audio \
  -j"$(nproc)"

cp -a build/v2_camera-rk3588/v4l2_capture /home/liu2004/nfs_dir/

./build/v2a_audio/v2a_audio_capture \
  --device <HW_PCM> \
  --rate <ACTUAL_RATE> \
  --channels <ACTUAL_CHANNELS> \
  --format <ACTUAL_FORMAT> \
  --period-frames <ACTUAL_PERIOD> \
  --buffer-frames <ACTUAL_BUFFER> \
  --duration 60 \
  --output reports/audio/cpp_capture.wav \
  --csv logs/v2a_audio/audio_frames.csv \
  --timestamp-type monotonic