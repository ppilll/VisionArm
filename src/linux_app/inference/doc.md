# 如何编译以及使用
交叉编译：
source /opt/atk-dlrk3588-toolchain/environment-setup

SDK="$HOME/work/Linux_SDK/atk_dlrk3588_linux5.10"
RKNN_INCLUDE_DIR="$SDK/external/rknpu2/runtime/Linux/librknn_api/include"
RKNN_LIBRARY="$SDK/external/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
OpenCV_DIR="$SDK/external/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV"

/usr/bin/cmake \
  -S src/linux_app/inference \
  -B build/v3_inference \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/cmake/rk3588-toolchain.cmake" \
  -DRKNN_INCLUDE_DIR:PATH="$RKNN_INCLUDE_DIR" \
  -DRKNN_LIBRARY:FILEPATH="$RKNN_LIBRARY" \
  -DOpenCV_DIR:PATH="$OpenCV_DIR"


/usr/bin/cmake \
  --build build/v3_inference \
  --parallel "$(nproc)"

rm -rf build/v3_inference
cp -a build/v3_inference/rknn_inference /home/liu2004/nfs_dir/

只检查模型
./rknn_inference model/best_i8.rknn \
  2>&1 | tee logs/v3/v3_tensor_contract.log

运行静态图片推理
./rknn_inference \
  model/best_i8.rknn \
  image/20220317_144226_002_jpg.rf.l2kwEfUmbCkCz3jUJ8qE.jpg \
  rknn_dump

# 生命周期
```
constructor
    ↓
LoadModelFile
    ↓
rknn_init
    ↓
RKNN_QUERY_SDK_VERSION
    ↓
RKNN_QUERY_IN_OUT_NUM
    ↓
RKNN_QUERY_INPUT_ATTR
    ↓
RKNN_QUERY_OUTPUT_ATTR
    ↓
Infer...
    ↓
rknn_outputs_release EVERY TIME
    ↓
rknn_destroy
```
只检查模型
./rknn_engine_demo model/best_i8.rknn \
  2>&1 | tee v3_tensor_contract.log

运行静态图片推理
./rknn_engine_demo \
  model/best_i8.rknn \
  images/test.jpg \
  rknn_dump

  # 设计取舍

连续视频单帧输入会怎样
单线程调用方式下，下一帧不会在上一帧推理过程中进入这个 RknnEngine
调用者会被阻塞在当前 Infer() 中。

我们能否多帧合并？

正常的流水线
```
采集线程
    │
    ▼
有界帧队列
    │
    ▼
预处理/推理线程
    │
    ▼
有界结果队列
    │
    ▼
后处理/显示/保存线程
```
线程 A：采集
线程 B：预处理
线程 C：NPU 推理
线程 D：后处理和显示

```
时间 ───────────────────────────────────────→

采集线程： [取F0][取F1][取F2][取F3]

预处理：       [预F0][预F1][预F2]

NPU：                [推F0][推F1][推F2]

后处理：                   [后F0][后F1]
```

