# VisionArm V4-R5/R6 — MPP H.265 Main Path and Inference Topology Benchmark

## 1. Scope

This tree extends the completed V4-R0 through V4-R4 implementation with:

- **V4-R5:** a minimal Rockchip MPP H.265 video branch sharing each dequeued V4L2 DMA-BUF through an independent `VideoLease`;
- **V4-R6:** two selectable inference thread topologies, `FUSED_NPU_POSTPROCESS` and `SPLIT_NPU_POSTPROCESS`, plus a board probe that measures full latency distributions under simultaneous H.265 encoding.

The R3/R4 board conclusions remain frozen:

```text
RGA 300-frame parity passed
RGA 18000-frame stability passed
Camera and RKNN RGA handles are cached, not imported per frame
RKNN DMA32 input slots are shared by RGA and RKNN
RGB888 padding fill uses low-24-bit 0x00727272
Traditional and Bound Host I/O produce identical logical outputs
1x1 and 2x2 slot rotation is stable and free of stale output contamination
Bound Host I/O is faster than the traditional path on this board
```

## 2. Product data flow

```text
Camera / ISP
    |
V4L2 MMAP + EXPBUF buffers
    |
Capture owner thread / DQBUF
    |
CaptureBufferBroker
    |---------------- VideoLease --------------------------+
    |                                                       |
    |                                               MPP H.265 encoder
    |                                                       |
    |                                               owned EncodedPacket
    |                                                       |
    |                                               bounded packet queue
    |                                                       |
    |                                               H265FileSink
    |
    +---------------- InferenceLease -----------------------+
                            |
                    latest-frame queue
                            |
                    RGA NV12 -> RGB aspect-preserving centered letterbox (960x544)
                            |
                    DMA32 RKNN input slot
                            |
                    Bound Host I/O / NPU
                            |
                    fused or split Top-1 postprocess
                            |
                    PerceptionPacket
```

Only the Camera owner thread performs V4L2 QBUF. The raw Camera buffer is requeued after both VideoLease and InferenceLease are released.

## 3. R5 contracts

`MppH265Encoder` accepts only the frozen R2/R3 layout:

```text
single-plane linear V4L2_PIX_FMT_NV12
one exported DMA-BUF fd
data_offset == 0
known horizontal and vertical stride
```

Camera DMA-BUF imports are cached by `buffer_index`. The encoder performs:

```text
mpp_buffer_import(external DMA-BUF)
MppFrame setup
encode_put_frame
encode_get_packet
copy compressed bytes to EncodedPacket
return from Encode
release VideoLease
```

The packet/file thread owns only compressed bytes and can never extend a raw Camera lease.

The video raw-frame queue preserves order and uses bounded backpressure. It is intentionally different from the inference latest-frame queue.

## 4. R6 topologies

### FUSED_NPU_POSTPROCESS

```text
PreprocessThread
NpuPostprocessThread: RKNN -> output sync -> Top-1 -> result
```

### SPLIT_NPU_POSTPROCESS

```text
PreprocessThread
NpuThread: RKNN -> CompletedFrame queue
PostprocessThread: output sync -> Top-1 -> result
```

The probe reports mean, p50, p95, p99 and maximum for:

```text
preprocess
inference
postprocess
capture_to_result
```

The final topology must be selected from real board results, not from thread-count assumptions.

## 5. Main files

```text
include/video/
├── encoded_packet.h
├── encoded_packet_sink.h
├── h265_file_sink.h
├── mpp_h265_encoder.h
├── nv12_mpp_layout.h
└── video_encoder.h

src/video/
├── h265_file_sink.cpp
├── mpp_h265_encoder.cpp
└── nv12_mpp_layout.cpp

include/pipeline/inference_pipeline.h
src/pipeline/inference_pipeline.cpp

tests/nv12_mpp_layout_test.cpp
tools/vision_pipeline_r5_r6_probe.cpp

docs/
├── v4_r5_mpp_h265_main_path.md
├── v4_r6_inference_topology.md
└── v4_r5_r6_operation_guide.md
```

## 6. Host tests

```bash
cmake -S . -B build-r5-r6-host \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISIONARM_BUILD_RUNTIME=OFF \
  -DVISIONARM_BUILD_TESTS=ON \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=ON

cmake --build build-r5-r6-host -j"$(nproc)"
ctest --test-dir build-r5-r6-host --output-on-failure
```

Expected tests:

```text
capture_buffer_contract_test
capture_buffer_broker_test
v4l2_dmabuf_contract_test
letterbox_geometry_test
nv12_mpp_layout_test
yolov8_top1_postprocessor_test
```

## 7. RK3588 build

Use the exact RKNN, librga and MPP versions already validated with the BSP:

```bash
cmake -S . -B build-r5-r6-board \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISIONARM_BUILD_RUNTIME=ON \
  -DVISIONARM_BUILD_TESTS=ON \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=ON \
  -DVISIONARM_BUILD_ACCELERATION_TOOLS=ON \
  -DVISIONARM_ENABLE_OPENCV_PREPROCESS=ON \
  -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
  -DVISIONARM_ENABLE_MPP_VIDEO=ON \
  -DRKNN_INCLUDE_DIR=/actual/rknn/include \
  -DRKNN_LIBRARY=/actual/librknnrt.so \
  -DRGA_INCLUDE_DIR=/actual/librga/include \
  -DRGA_LIBRARY=/actual/librga.so \
  -DMPP_INCLUDE_DIR=/actual/mpp/include \
  -DMPP_LIBRARY=/actual/librockchip_mpp.so

cmake --build build-r5-r6-board -j"$(nproc)"
ctest --test-dir build-r5-r6-board --output-on-failure
```

## 8. Minimal R5/R6 probe

```bash
./build-r5-r6-board/vision_pipeline_r5_r6_probe \
  --device /dev/videoX \
  --model /actual/model.rknn \
  --output reports/v4_r5/fused_1x1.h265 \
  --width 1280 --height 720 --fps 30 \
  --buffers 6 --video-queue 2 \
  --bitrate 4000000 --gop 60 \
  --duration-sec 60 \
  --topology fused \
  --input-slots 1 --output-slots 1 \
  --input-dma-heap /dev/dma_heap/system-uncached-dma32 \
  --report reports/v4_r5/fused_1x1.txt
```

Validate the output:

```bash
ffprobe -v error -f hevc \
  -show_entries stream=codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 \
  reports/v4_r5/fused_1x1.h265
```

See `MIGRATION_960x544.md` for the 960x544 single-class model migration, build, probe and validation procedure.

## 9. Current boundary

R5/R6 do not implement:

```text
Audio or A/V synchronization
container muxing
RTSP/RTP
product TCP framing/reconnect
local recording index
UART protocol
actuator control
physical closed loop
```

`H265FileSink` is the minimal R5 validation sink. A later TCP sink can implement the same `IEncodedPacketSink` interface without changing Camera, MPP or inference ownership.
