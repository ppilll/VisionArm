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

The canonical build entry point cross-builds the only product executable,
`visionarm_runtime`, into `build/rk3588/bin`:

```sh
./build.sh
```

Use `--clean`, `--debug`, `--jobs N`, `--no-copy`, or `--copy-dir PATH` as
needed. The script accepts `VISIONARM_TOOLCHAIN_ENV`, `VISIONARM_SDK`,
`VISIONARM_SYSROOT`, and `VISIONARM_COPY_DIR` environment overrides. It enables
the board product features and passes the RKNN, RGA, MPP, ALSA, and FFmpeg
locations to CMake once; there are no test, tool, probe, or benchmark targets.

The product feature switches are documented by their descriptions in
[`cmake/Options.cmake`](cmake/Options.cmake). The final target graph and
canonical source layout are summarized in
[`doc/README_STRUCTURE.md`](doc/README_STRUCTURE.md).

## Board runtime

Use `visionarm_runtime --help` to view the complete option set. A typical
deployment supplies the camera device, model, output/report paths, and any
enabled media/control destinations:

```sh
./visionarm_runtime \
  --device /dev/videoX \
  --sensor-subdev /dev/v4l-subdevX \
  --model /path/to/model.rknn \
  --output /tmp/visionarm.h265 \
  --report /tmp/visionarm.report.txt \
  --width 1920 --height 1080 --fps 30 \
  --bitrate 4000000 --gop 60 \
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

## Runtime report

The report schema is `visionarm.runtime_report.v1`. The primary completion
field is `result=PASS` or `result=FAIL`; consumers should not infer result
from file names or historical aliases.

For the final 600-second board acceptance, retain the report, runtime logs,
raw H.265 output, optional MP4 output, and host telemetry summary. A passing
run must report drained queues, zero broker/camera outstanding resources, no
requeue or DMA-BUF synchronization failures, continuous monotonic A/V timing,
successful mux/network finalization, isolated telemetry health, and the
expected UART/control counters.

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
