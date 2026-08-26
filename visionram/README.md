# VisionArm BallTrack V8.5 最终轮：独立 Telemetry 与 PC Overlay

V8.4 已通过真实板端、本地回传和网络对端验收并冻结：本地 MP4、HEVC+AAC MPEG-TS/UDP、TS continuity、完整解码均正常，无花屏或撕裂。本轮不修改 HEVC/AAC 编码参数、MPEG-TS 时间戳修复、MediaClock 或 MP4 封装语义。

V8.5 本轮完成以下内容：

- Telemetry 从同步 control sink 链彻底移出，由 supervisor 采样 `LatestResultStore`。
- Camera→Inference 与 Camera→Video 分发不再因 video FIFO 满而互相等待。
- Video 编码/文件/Mux/UDP 分支失败时只关闭媒体分支，Inference 继续运行；故障仍令最终报告 FAIL。
- Telemetry 启动或运行失败不会提前停止 Camera/Inference/媒体主循环。
- PC 端同时播放 HEVC+AAC MPEG-TS，并按 media PTS 绘制 Camera FPS、检测框、控制/延迟、Network、Recording 状态。
- Overlay timeline 有界、拒绝乱序结果、绝不把未来推理结果画到更早的视频帧，并隐藏过期框。

代码完成不等于实机 PASS。只有本文最后的板端、PC、故障隔离和稳定性验收均通过后，才能冻结 V8.5。

## 1. 完整架构与数据流

```text
                                  RK3588 board

                         Camera owner / V4L2 DQBUF
                                    │
                       CaptureBufferBroker::Publish
                         ┌──────────┴──────────┐
                 inference FrameLease     video FrameLease
                         │                       │
             latest-frame queue, cap=1    video FIFO, bounded
                 replace old, no history  PushLatest, never waits
                         │                       │
                  RGA → RKNN → postprocess      MPP HEVC
                         │                       │ release lease
                TargetStateMachine              ▼
                  ┌──────┴──────┐       owning encoded packets
                  │             │               │
          mock/UART control  LatestResultStore  ├─ raw H.265 file
                                                ├─ MP4 mux
                                                └─ MPEG-TS sink queue
                                                        │
                                               UDP worker / port 5000

                  supervisor thread, 100 ms sampling only
                  ┌───────────────────────────────────────┐
                  │ LatestResultStore copy                │
                  │ cheap atomic pipeline counters        │
                  │ MP4/network snapshot counters         │
                  └──────────────────┬────────────────────┘
                                     ▼
                         latest telemetry mailbox
                                     │
                         JSON serializer + UDP worker
                              UDP / port 5001

                                      PC
                  ┌────────────────────┴────────────────────┐
             GStreamer MPEG-TS path                  telemetry thread
        tsdemux → HEVC decode → cairooverlay      bounded result timeline
        tsdemux → AAC decode  → audio output             │
                  └────────────────────┬───────────────────┘
                                  display only
```

### 1.1 Camera ownership 与两条分支

Camera buffer 仍只由 `CaptureBufferBroker + FrameLease` 管理。一次 DQBUF 为 Inference 和 Video 创建两个独立 lease；最后一个 lease release 后，只有 Camera owner 才执行 QBUF。任何 Mux、UDP、Telemetry 或 PC Overlay 都看不到 DMA-BUF，也不能延长 Camera buffer 生命周期。

Inference 分支使用 capacity=1 的 latest-frame queue。输入 slot 忙时只替换等待中的旧推理帧，不处理历史积压。

Video 分支使用有界 FIFO，但 Capture 现在使用非阻塞 `PushLatest()`：

- FIFO 有空间：移交 video lease 给 MPP。
- FIFO 已满：保留最新 video lease，立即 release 被替换的旧 lease，并增加 `video_frames_dropped`。
- 媒体分支已关闭：立即 release 新 video lease，增加 `video_frames_dropped`，Camera/Inference 继续。
- 正常验收要求 `video_frames_dropped=0`；非零不是静默成功，而是明确 FAIL。

这保证媒体过载不会让 Camera owner 等待 video FIFO，从软件队列层面切断 Video→Inference 的反压路径。Camera、RGA、MPP、RKNN 仍共享 SoC 内存带宽等物理资源，性能验收仍必须看实机 FPS/latency；“独立”不表示硬件资源凭空隔离。

### 1.2 推理、控制与 Telemetry

正式推理路径只有：

```text
postprocess → TargetStateMachine → primary control sink (mock 或 UART)
                                 → LatestResultStore
```

`UdpTelemetrySink` 不再继承 `IControlSink`，也不在 `TargetStateMachine` 的同步 fanout 中。Supervisor 每 100 ms 读取一份已完成的 latest result 和少量 atomic counter，再调用 `UpdateControl()/UpdateRuntime()` 覆盖 latest mailbox。更新函数不做 DNS、JSON 序列化或 socket I/O。

Telemetry worker 才负责固定频率 JSON/UDP 发送。Telemetry 启动、序列化或 `sendto()` 失败会锁存 `telemetry_runtime_fault=1`，最终 `telemetry_ok=0`，但不会改变 control result，也不会提前停止 Inference。

因此 `telemetry_control_updates` 是 supervisor 采样到的不同结果数，不再等于 state machine 处理数，正确关系为：

```text
0 < telemetry_control_updates <= state_processed_packets
```

### 1.3 Video/Audio、Mux 与 UDP

MPP 消费输入后立即 release Camera lease，后续对象只拥有自己的 HEVC bytes。Audio 仍使用既有 ALSA→MediaClock→AAC 路径。MPEG-TS/UDP sink 的 `Write()/WriteAudio()` 只把 owning HEVC/AAC packet 放进它自己的 bounded queue；libavformat 和 UDP socket 只由网络 worker 操作。

UDP/5000 从不读取、轮询或等待 `LatestResultStore`，也不把 detection JSON 塞进 MPEG-TS。编码 access unit 的 PTS 只来自 V4L2 capture monotonic timestamp 与共同 media epoch，不来自 inference 完成时间。

若编码、文件或网络 sink 失败，`SignalVideoFailure()` 只停止 video/encoded queues；Capture 随后立即释放新的 video lease，Inference 仍可继续产生结果。Audio/Mux/Network 故障同样只锁存 `auxiliary_media_runtime_fault=1`，主循环运行到设定 duration 或用户信号。最终报告仍会 FAIL，防止把残缺媒体误判为成功。

### 1.4 PC Overlay 与 PTS 匹配

Overlay 完全位于 PC：GStreamer 解码 MPEG-TS 中的 HEVC/AAC，`cairooverlay` 只修改显示帧，不回传命令、不修改板端 H.265、不二次编码。

Telemetry 中的：

```text
capture_media_pts_ms = capture_monotonic_ns - media_epoch_monotonic_ns
```

与视频 packet 使用同一 media epoch。每个 decoded video buffer 到来时，PC 从有界 timeline 选择 `capture_media_pts_ms <= video_pts_ms` 的最新结果：

- 默认不容许未来结果，绝不会把较新检测画到较早画面。
- 同一个 inference frame 的周期性重复 datagram 只保存一次。
- timeline 默认最多 512 个不同结果，满后淘汰最旧结果。
- 结果相对视频超过 300 ms，或发送端报告 `result_staleness_ms>300`：隐藏 bbox，显示 `RESULT_STALE`。
- 1 s 未收到 telemetry：隐藏 bbox，显示红色 `TELEMETRY STALE`，视频和音频继续。
- 只有 `DETECTED + target.valid + control.valid` 才绘制 bbox。

默认 `--video-pts-offset-ms 0`。不要凭观察随意修改；只有用抓取的 TS/telemetry 证明接收栈存在固定 PTS 偏移后，才传入测得的常数。

## 2. 本轮文件

板端与协议：

- `include/common/pipeline_types.h`
- `include/pipeline/inference_pipeline.h`
- `src/pipeline/inference_pipeline.cpp`
- `include/telemetry/udp_telemetry_sink.h`
- `src/telemetry/udp_telemetry_sink.cpp`
- `tools/vision_pipeline_r7_r8_probe.cpp`
- `tests/udp_telemetry_sink_test.cpp`
- `tests/bounded_queue_metrics_test.cpp`
- `CMakeLists.txt`

PC 工具：

- `tools/receive_v8_5_telemetry.py`
- `tools/v8_5_overlay_core.py`
- `tools/view_v8_5_overlay.py`
- `tests/v8_5_overlay_core_test.py`

## 3. 应用第二轮 patch

实际仓库必须已经处于 V8.5 第一轮代码状态，然后在仓库根目录执行：

```sh
git apply --check V8.5-round2.patch
git apply V8.5-round2.patch
```

## 4. 板端构建与自动测试

沿用已通过 V8.4/V8.5 第一轮的真实 toolchain、sysroot 和库路径：

```sh
cmake -S . -B build-v8-5 \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISIONARM_BUILD_RUNTIME=ON \
  -DVISIONARM_BUILD_TESTS=ON \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=ON \
  -DVISIONARM_BUILD_ACCELERATION_TOOLS=ON \
  -DVISIONARM_ENABLE_OPENCV_PREPROCESS=OFF \
  -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
  -DVISIONARM_ENABLE_MPP_VIDEO=ON \
  -DVISIONARM_ENABLE_ALSA_AUDIO=ON \
  -DVISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON \
  -DVISIONARM_ENABLE_FFMPEG_MP4_MUX=ON \
  -DVISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK=ON \
  -DRKNN_INCLUDE_DIR=<rknn_include> \
  -DRKNN_LIBRARY=<librknnrt.so> \
  -DRGA_INCLUDE_DIR=<rga_include> \
  -DRGA_LIBRARY=<librga.so> \
  -DMPP_INCLUDE_DIR=<mpp_include> \
  -DMPP_LIBRARY=<librockchip_mpp.so> \
  -DFFMPEG_INCLUDE_DIR=<ffmpeg_include> \
  -DFFMPEG_AVCODEC_LIBRARY=<libavcodec.so> \
  -DFFMPEG_AVUTIL_LIBRARY=<libavutil.so> \
  -DFFMPEG_AVFORMAT_LIBRARY=<libavformat.so>

cmake --build build-v8-5 -j"$(nproc)"
ctest --test-dir build-v8-5 --output-on-failure
```

`udp_telemetry_sink_test` 会检查 schema、media PTS、Capture→Result latency、`result_staleness_ms`、UDP loopback 和非有限数拒绝。CMake 找到 Python3 时，CTest 也会自动运行 `v8_5_overlay_core_test`。

PC 上先运行不依赖 GStreamer 的 matcher 测试：

```sh
python3 -m unittest -v tests/v8_5_overlay_core_test.py
python3 -m py_compile \
  tools/receive_v8_5_telemetry.py \
  tools/v8_5_overlay_core.py \
  tools/view_v8_5_overlay.py
```

纯 Python 测试覆盖精确/前向 PTS 匹配、未来结果拒绝、结果过期、Telemetry 过期、bounded capacity、重复/乱序和 sequence gap。

## 5. PC 安装 Overlay 依赖

Ubuntu/Debian PC：

```sh
sudo apt update
sudo apt install \
  python3-gi python3-cairo gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
```

确认关键插件：

```sh
gst-inspect-1.0 tsdemux decodebin h265parse avdec_h265 aacparse avdec_aac cairooverlay
```

实际 MPEG-TS 可能同时出现
`video/x-h265,alignment=nal` 和 `audio/mpeg,mpegversion=2,stream-format=adts`。
Viewer 不再对这些 encoded caps 写死 parser→decoder 协商，而是把音视频分别
交给独立 `decodebin` 自动选择 parser/decoder；只有得到 raw caps 后才连接
Overlay/Audio 输出。启动时应依次看到：

```text
linked video/x-h265 demux pad: video/x-h265, ... alignment=(string)nal
linked audio/mpeg demux pad: audio/mpeg, ... mpegversion=(int)2 ...
linked decoded video/x-raw: video/x-raw, ...
linked decoded audio/x-raw: audio/x-raw, ...
```

不应再出现 `GST_PAD_LINK_NOFORMAT` 或 `reason not-negotiated`。

视频 decoder 常输出 I420，而 `cairooverlay` 只接受有限的 RGB raw format。
Viewer 因此使用固定边界：

```text
decoded I420/NV12
→ videoconvert
→ video/x-raw,format=BGRA
→ cairooverlay
→ videoconvert
→ autovideosink
```

Decodebin raw-video 动态 pad 只检查 element hierarchy，避免 GStreamer 在第一个
`videoconvert` 执行之前，把可转换的 I420 错误判成 `GST_PAD_LINK_NOFORMAT`。

防火墙放行：

```text
UDP/5000 = HEVC+AAC MPEG-TS
UDP/5001 = visionarm.telemetry.v1 JSON
```

两个接收工具不能同时绑定 UDP/5001；自动 validator 和 Overlay 应分两次试验运行。

## 6. PC 端运行

先启动 Overlay，再启动板端：

```sh
python3 tools/view_v8_5_overlay.py \
  --video-port 5000 \
  --telemetry-port 5001 \
  --result-stale-ms 300 \
  --telemetry-stale-ms 1000 \
  --timeline-capacity 512 \
  --telemetry-log v8_5_overlay_telemetry.jsonl
```

Viewer 保留 AAC 播放。按 `Ctrl-C` 退出后会输出 received、invalid、out-of-order、sequence gap 和 retained result 摘要。

单独做 wire validation 时：

```sh
python3 tools/receive_v8_5_telemetry.py \
  v8_5_telemetry_30s.jsonl 30 5001 \
  --stale-result-ms 300 \
  --expect-network enabled \
  --expect-recording enabled
```

Telemetry validator 新增检查 `control.result_staleness_ms` 必须有限且非负，并在 summary 输出 stale sample 数和最大 staleness。PASS 仍要求：

```text
parse_errors=0
schema_errors=0
sequence_gaps=0
out_of_order=0
counter_regressions=0
network_expectation_ok=1
recording_expectation_ok=1
v8_5_telemetry_validation=PASS
```

## 7. 板端运行

将 `<PC_IP>`、Camera、sensor subdev、model 和输出路径替换为实机值：

```sh
./build-v8-5/vision_pipeline_r7_r8_probe \
  --device /dev/videoX \
  --sensor-subdev /dev/v4l-subdev2 \
  --model /path/to/football_960x544.rknn \
  --output /tmp/v8_5_final.h265 \
  --av-output /mnt/sdcard/v8_5_final_30s.mp4 \
  --report /tmp/v8_5_final.report.txt \
  --width 1920 --height 1080 --fps 30 \
  --bitrate 8000000 --gop 30 \
  --buffers 6 --video-queue 2 \
  --audio-device hw:1,0 \
  --audio-rate 48000 --audio-channels 2 \
  --audio-period-frames 1024 --audio-buffer-frames 4096 \
  --audio-queue 16 --audio-encoded-queue 32 \
  --audio-bitrate 128000 \
  --network-url 'udp://<PC_IP>:5000' \
  --network-queue 256 --network-io-timeout-ms 1000 \
  --network-rate-bps 0 --network-burst-bits 0 \
  --network-packet-size 1316 \
  --network-send-buffer-bytes 4194304 \
  --telemetry-host <PC_IP> \
  --telemetry-port 5001 \
  --telemetry-interval-ms 100 \
  --telemetry-send-buffer-bytes 1048576 \
  --duration-sec 30 \
  --control-backend mock
```

先以 `mock` 验收媒体和显示，再使用已经验证的 UART 参数回归；本轮没有修改 wire framing、CRC、SafetyGate、STOP/watchdog 或云台控制算法。

## 8. 板端 report 验收

V8.4 原有 Camera/Audio/MPP/MP4/MPEG-TS/ownership/queue/PTS 条件必须继续全部 PASS，并额外确认：

```text
auxiliary_media_runtime_fault=0
video_frames_dropped=0
video_branch_failed=0

telemetry_enabled=1
telemetry_start_ok=1
telemetry_start_error=
telemetry_runtime_fault=0
telemetry_final_update_ok=1
telemetry_stop_ok=1
telemetry_started=1
telemetry_stopped_cleanly=1
telemetry_fatal_error=0
telemetry_control_updates>0
telemetry_runtime_updates>0
telemetry_datagrams_attempted>0
telemetry_datagrams_attempted=telemetry_datagrams_sent
telemetry_send_failures=0
telemetry_serialization_failures=0
telemetry_oversized_datagrams=0
telemetry_ok=1
vision_pipeline_r7_r8_probe=PASS
```

同时人工核对：

```text
0 < telemetry_control_updates <= state_processed_packets
camera_outstanding_before_stop=0
broker_outstanding_frames_before_camera_stop=0
broker_outstanding_leases_before_camera_stop=0
```

## 9. V8.5 最终验收顺序

1. 运行全部 CTest 和 Python matcher tests。
2. 30 s Network+Recording+Telemetry：先用 validator，确认 JSON 与 sequence PASS。
3. 再运行 30 s Overlay：确认 HEVC 画面、AAC 声音、Camera/NPU FPS、Network/Recording 状态均可见。
4. 足球进入/离开画面：bbox 只在 `DETECTED` 时出现，坐标与原图目标一致；不得画到目标出现前的帧。
5. Viewer 使用错误的 `--telemetry-port 5999`，板端仍向 5001 发送：视频/音频必须继续，窗口显示 `TELEMETRY: WAITING`，不能冻结媒体。
6. 正常 Viewer 中停止 telemetry 输入但保持 MPEG-TS：1 s 后显示红色 `TELEMETRY STALE` 且 bbox 消失，A/V 继续。
7. 人为制造 video FIFO/网络 sink 故障测试：report 必须显式出现 drop/fault 和最终 FAIL；在设定 duration 内 Camera/Inference counter 应继续增长，Camera/Broker lease 最终为 0。
8. Network+Recording+Telemetry+Overlay+mock 连续 1 分钟，确认无花屏、撕裂、明显 A/V 异常和 bbox 时间倒置。
9. 换 UART control，完整组合连续 10 分钟；检查 RSS 无持续增长、所有 queue bounded、Audio 无静默 drop、TS continuity/完整解码和 MP4 playable。
10. 对 10 分钟输出继续运行 V8.4 的 TS/ffprobe/MP4 验证工具。Overlay 人工 PASS 不能替代媒体 packet/timestamp 验证。

完成以上实机项目且 report/PC validator/人工 Overlay 均 PASS 后，V8.5 才可冻结。下一阶段是 V8.6 Full Integration；V7 真实视觉伺服闭环仍未完成，本阶段没有重新调 PID，也不能据此宣称 V7 完成。
