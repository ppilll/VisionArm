# 如何编译以及使用
交叉编译：
source /opt/atk-dlrk3588-toolchain/environment-setup
/usr/bin/cmake \
  -S src/linux_app/audio \
  -B build/v2a_audio \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/cmake/rk3588-toolchain.cmake"

/usr/bin/cmake \
  --build build/v2a_audio \
  --verbose

rm -rf build/v2a_audio
cp -a build/v2a_audio/v2a_audio_capture /home/liu2004/nfs_dir/

./v2a_audio_capture \
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

./v2a_audio_capture \
  --device hw:3,0 \
  --rate 48000 \
  --channels 2 \
  --format S16_LE \
  --period-frames 1024 \
  --buffer-frames 4096 \
  --duration 60 \
  --output reports/audio/cpp_capture.wav \
  --csv logs/v2a_audio/audio_frames.csv \
  --timestamp-type monotonic