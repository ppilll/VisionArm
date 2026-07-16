# 系统模块

| 模块 | 第一版主线 | fallback / 备注 |
|---|---|---|
| 主控 | RK3588 开发板 | 固定主控，不更换 |
| 摄像头 | OV5640，目标 720p@60fps | 若 OV5640 bring-up 阻塞，则使用 USB UVC 摄像头完成闭环 |
| 摄像头接口 | 优先使用开发板官方已适配的 MIPI/CSI 摄像头链路 | 不在第一版自研复杂 MIPI bring-up |
| 云台 | 带控制板的成品无刷电机云台 | 若无刷云台阻塞，则降级为二自由度舵机云台 |
| MCU | STM32F1 系列 | 作为协议、安全和云台控制 MCU，不在第一版自研复杂 FOC |
| Linux → MCU 通信 | UART P0 | CAN P1，对比测试 P2 |
| 上位机 | PC | 第一版先状态回传，视频回传后置增强 |

# 第一版最小闭环定义如下：

```
摄像头采集一帧图像
→ NPU 推理得到足球 mask
→ 后处理得到 ball_x / ball_y / confidence
→ 计算 dx = ball_x - frame_center_x
→ 计算 dy = ball_y - frame_center_y
→ 根据 dead zone / 限幅 / 比例控制生成 pan_cmd / tilt_cmd
→ Linux 通过 UART 发送控制帧
→ MCU 解析控制帧
→ MCU 输出 PWM 控制云台
→ 摄像头方向改变
→ 上位机看到状态或带标注视频
→ 记录 fps / latency / error
```

VisionArm BallTrack 由四部分组成：

1. RK3588 Linux 视觉主机；
2. 摄像头输入模块；
3. MCU/RTOS 云台控制模块；
4. PC 上位机视频/状态查看模块。

整体链路如下：

```text
+------------------+
|   Camera Sensor  |
| USB UVC / MIPI   |
+---------+--------+
          |
          v
+-------------------------+
| RK3588 Linux Application|
|                         |
|  V4L2 Capture           |
|  Preprocess             |
|  RKNN Runtime / NPU     |
|  Mask Postprocess       |
|  dx/dy Calculation      |
|  Control Strategy       |
|  UART Protocol TX/RX    |
|  Video/Status Streaming |
+-----------+-------------+
            |
            | UART
            v
+-------------------------+
| MCU / RTOS Controller   |
|                         |
| UART RX Task            |
| Control Queue           |
| Gimbal Control Task     |
| PWM Output              |
| Limit / E-Stop Logic    |
+-----------+-------------+
            |
            v
+-------------------------+
| 2-DOF Gimbal            |
| Pan / Tilt Servo        |
+-------------------------+
```

RK3588 → PC:
Annotated Video / MJPEG / TCP / UDP / Status JSON / CSV Logs

# 关键量化指标
第一版至少统计以下指标：

| 指标         |                   目标值 | 记录方式           |
| ---------- | --------------------: | -------------- |
| Camera FPS |      TBD，例如 >= 15 fps | Linux 日志       |
| 单帧采集耗时     |                   TBD | 时间戳            |
| 预处理耗时      |                   TBD | 时间戳            |
| RKNN 推理耗时  |                   TBD | RKNN / 应用日志    |
| 后处理耗时      |                   TBD | 应用日志           |
| 控制命令频率     |       TBD，例如 10-20 Hz | UART 发送日志      |
| UART 错误帧数量 |                   TBD | Linux / MCU 日志 |
| 端到端延迟      |                   TBD | 时间戳差值          |
| 中心误差       | TBD，例如平均误差 < 画面宽度 15% | CSV 统计         |
| 追踪成功率      |                   TBD | 测试脚本           |
| 丢球恢复时间     |                   TBD | 测试记录           |
| 视频回传延迟     |                   TBD | 上位机统计          |


有哪些、数据怎么流动、Linux 侧做什么、MCU/RTOS 侧做什么、上位机做什么、第一版通信链路是什么。



# 数据流定义
##  视频帧数据流
/dev/videoX
→ V4L2 mmap buffers
→ raw frame
→ resize / colorspace / normalization
→ RKNN input tensor
→ RKNN output tensor
→ segmentation mask
→ annotated frame
→ stream / save / display
## 控制数据流
ball_x, ball_y, confidence
→ dx, dy
→ dead zone
→ filter
→ limit
→ pan_cmd, tilt_cmd
→ UART frame
→ MCU parser
→ RTOS queue
→ PWM update
→ gimbal movement
## 状态回传数据流
MCU gimbal status
→ UART status frame
→ Linux parser
→ status overlay
→ PC viewer / log file

