# VisionArm Runtime

VisionArm captures NV12 frames from a V4L2 camera, performs RKNN inference,
and sends target state to the configured control backend. Optional media
branches encode H.265 video, capture and encode AAC audio, record MP4, and
stream MPEG-TS over UDP. The runtime emits a final, machine-readable report
for operational acceptance.

## Runtime architecture

```text
V4L2 Camera -> CaptureBufferBroker -> inference latest-frame queue
                                     -> optional video latest-frame queue

inference -> preprocess -> RKNN -> postprocess -> target state -> UART/control
                                                  -> result mailbox -> telemetry
video -> MPP H.265 -> encoded packet fanout -> raw output / MP4 / UDP :5000
ALSA -> media clock -> AAC -> encoded packet fanout -> MP4 / UDP :5000
telemetry mailbox -> independent UDP worker -> UDP :5001
```

The UDP endpoints are intentionally separate: port `5000` carries A/V
MPEG-TS only, while port `5001` carries telemetry JSON only. Telemetry is a
mailbox-driven side channel; it never blocks synchronous control or holds
camera or DMA-BUF resources.

### Resource and shutdown contracts

- V4L2 remains the camera-buffer owner. A buffer is requeued only after every
  issued inference and optional video lease has been released.
- Inference uses a capacity-one latest-frame queue. Video also uses bounded
  latest-frame replacement, so a slow media branch does not backpressure
  capture or inference.
- PCM audio uses bounded blocking backpressure; it is not silently replaced.
- Video and audio PTS originate from the same monotonic media epoch.
- Shutdown follows producer-to-consumer drain order. Encoded media is drained
  before mux/network finalization, and broker/camera outstanding counts must
  reach zero.

## Supported platform and optional features

The board runtime uses RKNN, Rockchip RGA and MPP, V4L2 DMA-BUF, and optionally
ALSA and FFmpeg. UART control can use the production serial backend or the
mock backend for host-oriented checks. Feature switches in
[`cmake/Options.cmake`](cmake/Options.cmake) control optional dependencies.

## Build

Configure with the board toolchain/sysroot and the vendor include/library
paths. A full board build normally enables the media features required by the
deployment:

```sh
cmake -S . -B build-board \
  -DCMAKE_BUILD_TYPE=Release \
  -DRKNN_INCLUDE_DIR=/path/to/rknn/include \
  -DRKNN_LIBRARY=/path/to/librknnrt.so \
  -DVISIONARM_ENABLE_RGA_PREPROCESS=ON \
  -DVISIONARM_ENABLE_MPP_VIDEO=ON \
  -DVISIONARM_ENABLE_ALSA_AUDIO=ON \
  -DVISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON \
  -DVISIONARM_ENABLE_FFMPEG_MP4_MUX=ON \
  -DVISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK=ON \
  -DVISIONARM_ENABLE_UART_CONTROL=ON
cmake --build build-board --target visionarm_runtime -j
```

For host-side regression builds, disable board-only runtime features and keep
tests enabled. This verifies contracts but does not replace a board build or
an end-to-end hardware run.

```sh
cmake -S . -B build-host \
  -DVISIONARM_BUILD_RUNTIME=OFF \
  -DVISIONARM_BUILD_CAPTURE_TOOLS=OFF \
  -DVISIONARM_BUILD_ACCELERATION_TOOLS=OFF \
  -DVISIONARM_ENABLE_ALSA_AUDIO=OFF \
  -DVISIONARM_BUILD_TESTS=ON
cmake --build build-host -j
ctest --test-dir build-host --output-on-failure
python3 -m unittest discover -s tests -p '*_test.py'
```

## Board runtime

Use `visionarm_runtime --help` to view the complete option set. A typical
deployment supplies the camera device, model, output/report paths, and any
enabled media/control destinations:

```sh
./visionarm_runtime \
  --device /dev/videoX \
  --model /path/to/model.rknn \
  --report /tmp/visionarm.report.txt \
  --network-url 'udp://HOST_IP:5000' \
  --telemetry-host HOST_IP --telemetry-port 5001 \
  --control-backend uart --uart-device /dev/ttyS3
```

For a complete acceptance run, use the production duration and retain the
report, runtime logs, and media outputs. A signal before the requested
duration is not a successful acceptance run.

## Host viewer and telemetry

Start the host viewer before the board sender:

```sh
python3 host/visionarm_viewer.py --bind 0.0.0.0
```

It launches `ffplay` with the validated low-latency settings for UDP `5000`,
while an independent receiver validates and summarizes telemetry from UDP
`5001`. The summary defaults to `visionarm_view_summary.json`; pass
`--telemetry-log` to retain received telemetry records.

To capture telemetry alone:

```sh
python3 host/telemetry.py telemetry.jsonl 60 5001 --bind 0.0.0.0
```

`tools/validation/mpegts_continuity.py` can validate a captured raw MPEG-TS
file, and `tools/validation/runtime_report.py` validates a final runtime
report.

## Runtime report

The report schema is `visionarm.runtime_report.v1`. The primary completion
field is `result=PASS` or `result=FAIL`; consumers should not infer result
from file names or historical aliases.

Validate a full-duration report with explicit enabled-module expectations:

```sh
python3 tools/validation/runtime_report.py /tmp/visionarm.report.txt \
  --minimum-duration-seconds 600 \
  --expect-audio enabled \
  --expect-network enabled \
  --expect-recording enabled \
  --expect-telemetry enabled \
  --expect-control uart
```

The validator checks queue draining, broker/camera resource return, media-clock
continuity, telemetry health, mux/network finalization, and control counters.

## Troubleshooting

- Missing vendor headers or libraries: configure with the matching board
  sysroot and required `*_INCLUDE_DIR` and `*_LIBRARY` values.
- No A/V on the host: confirm the board sends MPEG-TS to the host's UDP `5000`
  address and that firewall rules permit it.
- No telemetry: verify the board telemetry destination and host UDP `5001`
  binding; telemetry failures are reported separately from the inference path.
- A failed report: inspect the explicit error, queue, lease, media, telemetry,
  and control fields rather than treating a partial or interrupted run as a
  pass.
