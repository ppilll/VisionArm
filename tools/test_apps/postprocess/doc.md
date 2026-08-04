# 如何编译以及使用
交叉编译：
source /opt/atk-dlrk3588-toolchain/environment-setup

SDK="$HOME/work/Linux_SDK/atk_dlrk3588_linux5.10"
RKNN_INCLUDE_DIR="$SDK/external/rknpu2/runtime/Linux/librknn_api/include"
RKNN_LIBRARY="$SDK/external/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
OpenCV_DIR="$SDK/external/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV"

/usr/bin/cmake \
  -S src/linux__test_app/postprocess \
  -B build/v4_postprocess \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/cmake/rk3588-toolchain.cmake" \
  
/usr/bin/cmake \
  --build build/v4_postprocess \
  --parallel "$(nproc)"

rm -rf build/v4_postprocess
cp -a build/v4_postprocess/rknn_inference /home/liu2004/nfs_dir/

cmake -S . -B build-rknn \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISIONARM_WITH_RKNN=ON \
  -DRKNN_INCLUDE_DIR=/path/to/librknn_api/include \
  -DRKNN_LIBRARY=/path/to/librknnrt.so
  
# 
A C++17 reference implementation for the frozen VisionArm model contract:

input size: 960 x 960

output type: INT8 affine, NCHW

3 branches, strides 8 / 16 / 32

each branch: box [1,64,H,W], class [1,2,H,W], sum [1,1,H,W]

class 1: football

Pipeline:

9 raw RKNN tensors
-> affine dequantization
-> DFL decode
-> model-space xyxy candidates
-> class-aware NMS
-> reverse letterbox
-> original-frame xyxy
-> football target selection
-> center and normalized error