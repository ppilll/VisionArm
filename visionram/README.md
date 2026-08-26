# VisionArm BallTrack V8.6 第一轮：轻量接收与 10 分钟完整集成验收

本轮在已经实机通过的 V8.4、V8.5 基础上做两件事：

1. PC Viewer 使用已经实测正常的 FFmpeg/ffplay 低延迟路径播放 UDP/5000，Python 独立汇总 UDP/5001。
2. 开始 V8.6 Full Integration，把“完整运行 10 分钟、资源全部归还、所有分支正常结束”变成机器可检查的报告契约。

本轮代码完成不代表 V8.6 已通过。RK3588、Camera、RKNN、MPP、ALSA、FFmpeg、网络、存储和 STM32/UART 的 600 秒实机组合测试仍必须由真实环境完成；V8.6 第二轮将依据第一轮实测结果收口。

## 0. 应用本轮 patch

实际仓库应已应用 `V8.6-round1.patch`。本次 Viewer 修复是其增量 patch，在真实仓库根目录执行：

```sh
git apply --check V8.6-round1-viewer-hotfix.patch
git apply V8.6-round1-viewer-hotfix.patch
```

如果第一条命令失败，不要强制应用；先确认实际仓库已经完整应用上一份 V8.6 第一轮 patch，并且没有与 Viewer、README 或 CMake 测试清单重叠的本地修改。

## 1. 为什么截图中没有检测框

旧 Overlay 截图中的关键信息是：

```text
TARGET DETECTED conf=0.920
RESULT age=13906.0 ms RESULT_STALE
TELEMETRY LIVE age=71 ms
```

`TARGET DETECTED` 说明检测成功。没有框是因为旧 GStreamer Viewer 中正在显示的视频已经落后约 13.9 秒，远超原来的 300 ms bbox 时效阈值，因此旧实现主动隐藏了框。

三份实测 Telemetry JSONL 均为连续 `301/301` datagram，Camera 平均约 `28.95–28.97 FPS`、Inference 平均约 `28.87 FPS`，发送端 `result_staleness_ms` 最大约 `110 ms`。所以十几秒延迟是在 PC 的 GStreamer/XVideo 显示链中累积的，不是板端推理或 UDP 发送停顿。

本次降级后不再提供画面内检测框或状态面板。产品画面只显示 A/V；检测、Camera/NPU FPS、Network 和 Recording 状态写入独立 summary JSON。

## 2. 卡顿根因与修复

旧 Viewer 即使关闭 Cairo，仍使用虚拟机中的 GStreamer `tsdemux/decodebin/videoconvert/xvimagesink` 路径；实测三种模式都会累积延迟并出现花屏/撕裂。而相同主机、相同 MPEG-TS/UDP 流使用以下 V8.4 路径完全正常：

```text
ffplay -fflags nobuffer -flags low_delay -framedrop
```

这不能说明虚拟机完全没有 1080p30 解码能力；ffplay 正常已经证明它有能力完成本项目所需的基础播放。它说明当前虚拟机的 GStreamer/XVideo 组合不适合作为产品接收路径，继续调 Overlay 的收益和可靠性都不足。

轻量 Viewer 现在拆成两个互不等待的主机侧分支：

```text
UDP/5000 → ffplay low-latency/framedrop → A/V window

UDP/5001 → Python UDP thread → schema/sequence/counter validation
                             → optional raw JSONL
                             → final visionarm.viewer_summary.v1 JSON
```

Python 通过无 shell 的子进程参数启动 ffplay，播放参数与已经通过测试的 `receive_v8_4_mpegts.sh play-low-latency` 一致。Telemetry 接收不参与视频播放、解码或时钟同步。旧的 `--overlay-mode off/status/bbox` 参数仅为兼容已有命令而保留，三个值现在都使用相同的 ffplay 路径，均不会画框或面板。

## 3. 板端架构和数据流不变

```text
Camera owner / V4L2 DQBUF
        │
CaptureBufferBroker::Publish
   ┌────┴────────────────────────┐
   │                             │
Inference FrameLease         Video FrameLease
   │                             │
latest queue(cap=1)          bounded FIFO / PushLatest
   │                             │
RGA → RKNN → postprocess     MPP HEVC → owning packets
   │                             ├→ raw H.265
TargetStateMachine               ├→ MP4 recorder
   ├→ UART/STM32                  └→ MPEG-TS/UDP worker :5000
   └→ LatestResultStore
             │
 supervisor 只读采样，不在推理同步路径中
             │
       Telemetry mailbox → JSON/UDP worker :5001
```

必须继续成立的隔离原则：

- UDP/视频传输从不读取、轮询或等待 inference result。
- Inference 使用自己的 latest-frame queue；Video 使用自己的 bounded FIFO。
- Capture 向 Video 使用非阻塞 `PushLatest()`，媒体过载不能通过软件队列反压 Inference。
- MPP 消费 Camera lease 后，Mux、网络和文件只持有编码后 bytes，Video sink 不持有 Camera lease。
- Telemetry 不属于 `IControlSink` 同步 fanout；socket I/O 和 JSON 序列化由独立 worker 完成。
- 网络、Recorder 或 Telemetry 故障只锁存故障并令最终报告 FAIL，不篡改推理结果；Camera/Inference 继续运行到 duration，除非推理主流水线自身发生 fatal fault。
- PC Viewer 只消费 UDP，不向板端回传控制，不改变 H.265/AAC/Telemetry 内容。

软件数据流彼此不等待，但 RKNN、RGA、MPP、DDR 和 CPU 仍共享 SoC 物理资源，因此最终 FPS、延迟和稳定性只能用实机数据判断。

## 4. V8.6 第一轮新增验收契约

`vision_pipeline_r7_r8_probe` 新增：

```text
requested_duration_seconds
observed_duration_seconds
completed_requested_duration
terminated_by_signal
broker_outstanding_frames_after_stop
broker_outstanding_leases_after_stop
```

程序只有真正到达 duration deadline 才允许最终 PASS。提前 `Ctrl-C`、Inference pipeline 提前停止或其他提前退出会得到：

```text
completed_requested_duration=0
vision_pipeline_r7_r8_probe=FAIL
```

新增 `tools/validate_v8_6_integration_report.py`，离线检查：

- 确实请求并观察到至少 600 秒，且不是信号提前终止。
- Camera/Broker 在 shutdown 前后的 frame/lease 全为 0。
- Camera、RGA、RKNN、MPP、H.265、Audio、AAC、MP4、Network、Telemetry、UART 无 fatal/error/drop。
- 所有必须队列 `capacity>0`、`high_watermark<=capacity`、退出后 `current_size=0`。
- Audio 与 Network queue 没有 `replaced_oldest` 静默丢包。
- MP4 写 header 并 clean finalize；MPEG-TS/UDP worker clean finalize。
- RSS 增长不超过本次命令明确配置的上限。
- UART link 可用且 parser、read/write、protocol error 全为 0。

Validator 不能证明 MP4 可播放、TS packet 连续、物理 A/V 同步或主机显示流畅，这些仍需后续独立验证。

## 5. 构建与自动测试

沿用已经验证的 RK3588 toolchain、sysroot 和库路径：

```sh
cmake -S . -B build-v8-6 \
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
  -DVISIONARM_ENABLE_UART_CONTROL=ON \
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

cmake --build build-v8-6 -j"$(nproc)"
ctest --test-dir build-v8-6 --output-on-failure
```

CTest 在找到 Python3 时会自动运行：

```text
v8_5_overlay_core_test
v8_6_integration_report_test
v8_6_lightweight_viewer_test
```

PC 也可直接运行纯 Python 测试：

```sh
python3 -m unittest -v \
  tests/v8_5_overlay_core_test.py \
  tests/v8_6_integration_report_test.py \
  tests/v8_6_lightweight_viewer_test.py

python3 -m py_compile \
  tools/receive_v8_5_telemetry.py \
  tools/v8_5_overlay_core.py \
  tools/view_v8_5_overlay.py \
  tools/validate_v8_6_integration_report.py
```

## 6. PC 轻量接收：先启动，再启动板端

### 6.1 依赖

只需要 Python3 和带 `ffplay` 的 FFmpeg，不再需要 PyGObject、GStreamer 或 Cairo：

```sh
sudo apt update
sudo apt install ffmpeg
ffplay -version
```

### 6.2 同时播放 5000 并汇总 5001

```sh
python3 tools/view_v8_5_overlay.py \
  --video-port 5000 \
  --telemetry-port 5001 \
  --telemetry-log v8_6_telemetry.jsonl \
  --summary-json v8_6_view_summary.json
```

随后启动板端。程序会用以下已验证参数启动 ffplay：

```text
-fflags nobuffer
-flags low_delay
-framedrop
-probesize 5000000
-analyzeduration 5000000
```

关闭 ffplay 窗口或在启动该 Python 程序的终端按 `Ctrl-C` 后，程序停止 Telemetry receiver 并写出 `v8_6_view_summary.json`。若要自动运行 600 秒：

```sh
python3 tools/view_v8_5_overlay.py \
  --video-port 5000 --telemetry-port 5001 \
  --duration-seconds 600 \
  --telemetry-log v8_6_telemetry_600s.jsonl \
  --summary-json v8_6_view_summary_600s.json
```

`--overlay-mode status` 和 `--overlay-mode bbox` 仍能被旧命令接受，但会明确提示画面 Overlay 已禁用，并继续使用同一条 ffplay 路径。

### 6.3 Summary JSON 验收

输出 schema 为：

```text
visionarm.viewer_summary.v1
```

重点检查：

```text
video.backend = "ffplay"
video.low_latency = true
video.framedrop = true
video.overlay = "disabled"

telemetry.valid_datagrams > 0
telemetry.parse_errors = 0
telemetry.schema_errors = 0
telemetry.sequence_gaps = 0
telemetry.out_of_order = 0
telemetry.counter_regressions = 0
telemetry.receiver_errors = 0
telemetry.wire_validation = "PASS"
```

Summary 还包含 Camera/Inference FPS、Capture→Result latency、result staleness 的 minimum/maximum/mean/last，target/network/recording 各状态计数，以及最后一条有效 Telemetry。原始 datagram 仍可通过 `--telemetry-log` 保存为 JSONL。

注意：Viewer、Telemetry validator 不能同时绑定 UDP/5001；Viewer、TS capture 或另一个 ffplay 也不能同时绑定 UDP/5000。同类测试要分次运行。

## 7. 板端 600 秒 Full Integration

替换 `<PC_IP>`、Camera、sensor subdev、model、UART 和输出路径。测试期间不要按 `Ctrl-C`：

```sh
./build-v8-6/vision_pipeline_r7_r8_probe \
  --device /dev/videoX \
  --sensor-subdev /dev/v4l-subdev2 \
  --model /path/to/football_960x544.rknn \
  --output /tmp/v8_6_10min.h265 \
  --av-output /mnt/sdcard/v8_6_10min.mp4 \
  --report /tmp/v8_6_10min.report.txt \
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
  --telemetry-port 5001 --telemetry-interval-ms 100 \
  --telemetry-send-buffer-bytes 1048576 \
  --duration-sec 600 \
  --latency-samples 65536 \
  --max-rss-growth-kb 65536 \
  --control-backend uart \
  --uart-device /dev/ttyS3 \
  --uart-baud 115200 \
  --uart-ready-timeout-ms 5000
```

`--max-rss-growth-kb 65536` 是第一轮建议验收上限，不是普适产品指标。请同时记录 `rss.first_kb`、`rss.last_kb`、`rss.maximum_kb`；若接近上限，第二轮基于真实曲线调整或定位，不能只放宽阈值。

## 8. 离线验证板端 report

将报告复制到主机后运行：

```sh
python3 tools/validate_v8_6_integration_report.py \
  v8_6_10min.report.txt \
  --minimum-duration-seconds 600 \
  --expect-network enabled \
  --expect-recording enabled \
  --expect-telemetry enabled \
  --expect-control uart
```

成功必须输出：

```text
validation_error_count=0
v8_6_integration_report_validation=PASS
```

只要有 `validation_error=...`，本次 Full Integration 就不通过。不要手工把报告中的 FAIL 改成 PASS。

## 9. MP4、MPEG-TS 与 Telemetry 独立验证

报告 PASS 之后仍需验证真实媒体内容。

本地 MP4：

```sh
tools/validate_v8_3_av_file.sh \
  /mnt/sdcard/v8_6_10min.mp4 \
  /tmp/v8_6_mp4
```

另起一次板端运行，在 PC 捕获完整 TS：

```sh
tools/receive_v8_4_mpegts.sh capture \
  v8_6_network_600s.ts 600 5000

tools/validate_v8_4_network_capture.sh \
  v8_6_network_600s.ts \
  v8_6_network_600s
```

另起 30 秒 Telemetry wire validation：

```sh
python3 tools/receive_v8_5_telemetry.py \
  v8_6_telemetry_30s.jsonl 30 5001 \
  --stale-result-ms 300 \
  --expect-network enabled \
  --expect-recording enabled
```

最后人工确认：

- MP4 从头到尾可播放并 clean finalize，无花屏、撕裂和明显 A/V 不同步。
- MPEG-TS 完整解码、PTS/DTS 单调、continuity 正常。
- ffplay 网络播放无持续累积延迟；短时抖动后能通过 framedrop 追上实时画面。
- UART/STM32 在 10 分钟内无 watchdog、CRC、parser、read/write 异常。
- 退出后 Camera buffer 全归还，Broker frame/lease 均为 0。

## 10. 第一轮完成边界

本轮已经提供稳定性运行的硬完成语义和自动报告验收，但没有在隔离仓库中宣称以下项目实机 PASS：

- 600 秒全组合稳定性。
- 网络断开、磁盘写满/Recorder error 的真实故障注入。
- 物理声画同步。
- 产品硬件上的 RSS 上限。

请保存以下结果供 V8.6 第二轮使用：板端完整 stderr/stdout、`v8_6_10min.report.txt`、Validator 输出、MP4/TS validation summary、`v8_6_view_summary.json`，以及 ffplay 画面是否仍卡顿的结论。
