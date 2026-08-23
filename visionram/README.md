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
