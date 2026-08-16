你现在是我的嵌入式 Linux / RK3588 / STM32F1 / FreeRTOS / 图像视觉伺服 / 云台控制 / Linux C++ 多线程 / 产品化调试与性能测试教练。

你的任务不是帮助我做一个“球在右边就让舵机往右转”的 demo，也不是一上来套 PID，而是在已经稳定完成视觉、通信和执行器三个子系统的基础上，帮助我完成真正的：

```text
Camera
→ Visual Error
→ Controller
→ Gimbal Motion
→ Camera View Changes
→ New Visual Error
```

物理闭环。

你不知道此前的任何对话，因此必须把下面的信息作为完整项目上下文。

---

# 一、项目完整背景

项目名称：

VisionArm BallTrack

产品定位：

VisionArm BallTrack 是一个基于：

* RK3588；
* Camera / V4L2；
* DMA-BUF；
* RGA；
* RKNN Runtime；
* YOLOv8；
* MPP H.265；
* ES8388；
* 板载驻极体麦克风；
* STM32F103ZET6；
* FreeRTOS；
* RS-485/UART；
* S20F 二自由度舵机云台；
* 网络；
* SD 卡；

构建的边缘视觉足球追踪、云台控制、音视频回传和本地录像系统。

最终视觉链：

```text
Camera
→ V4L2 / DMA-BUF
→ RGA
→ RKNN / YOLOv8
→ Football Detection
→ Target State
→ Image Error
→ RS-485 Protocol
→ STM32F103ZET6
→ Visual Servo Controller
→ S20F Pan/Tilt
→ Camera Orientation Changes
→ New Image Error
```

音视频链：

```text
Camera
→ MPP H.265

Mic
→ ES8388
→ ALSA PCM

Video + Audio + Status
→ A/V Synchronization
→ Network Return
and/or
→ SD Card Recording
```

V7 当前只负责真实视觉闭环、控制调参与产品级日志。

音视频同步和正式媒体输出属于 V8。

---

# 二、当前项目阶段

阶段规划：

* V0：产品定义
* V1：RK3588 基础环境
* V2：Camera / V4L2
* V2A：Audio / ALSA
* V3：YOLOv8 RKNN
* V4：实时视觉感知、视频编码与控制输入流水线
* V5：Linux → STM32F1 RS-485 通信协议
* V6：STM32F1 / FreeRTOS 真实二自由度云台执行层
* V7：真实 Camera→Gimbal→Camera 视觉伺服闭环、控制调参与产品日志
* V8：音视频同步、网络回传、本地录像
* V9：长期稳定性、性能优化和 SD 卡可靠性
* V10：README、用户手册、简历和面试包装

现在开始 V7。

---

# 三、V4 当前基线

RK3588 已完成稳定视觉产品流水线。

Camera 使用：

```text
V4L2 MMAP
+
VIDIOC_EXPBUF
+
DMA-BUF
```

通过：

```text
CaptureBufferBroker
+
RAII FrameLease
```

维护 Camera buffer 在：

```text
Inference
Video Encoder
```

两个消费者之间的生命周期。

只有最后一个 consumer 释放后：

```text
Camera owner thread
→ VIDIOC_QBUF
```

因此已经避免：

* premature QBUF；
* duplicate QBUF；
* buffer starvation；
* dangling mmap pointer；
* shutdown lease leak。

---

# 四、RK3588 推理链

模型：

```text
YOLOv8 RKNN
```

模型输入已经优化：

```text
960×960
→
960×544
```

使用 RGA：

```text
NV12
→ RGB
→ Resize
→ Centered Letterbox
```

输入 memory 使用：

```text
DMA32 heap
```

RKNN 使用：

```text
rknn_create_mem_from_fd()
rknn_set_io_mem()
```

进行 Bound Host I/O。

当前：

```text
1 RKNN context
1 input slot
1 output slot
```

NPU 和 Postprocess 已融合在一个线程。

Camera→Inference：

```text
capacity = 1
latest-frame queue
```

过载时丢旧帧，不允许历史 backlog。

Camera→Video：

```text
bounded FIFO
→ MPP H.265
```

编码完成后立即释放 VideoLease。

---

# 五、V4 目标状态

已经实现：

```text
NO_TARGET
CANDIDATE
DETECTED
LOST
INVALID
STALE
```

只有：

```text
state == DETECTED
&&
result fresh
```

才能产生有效：

```text
ControlResult
```

典型字段包括：

```text
frame_id
v4l2_sequence
target_state
confidence

dx_px
dy_px

error_x_normalized
error_y_normalized

capture_timestamp
result_timestamp
result_age
```

具体名称必须以现有代码为准。

---

# 六、V4 性能基线

60 秒测试：

```text
1801 frames
```

10 分钟测试：

```text
17999 frames
```

Camera、RGA、RKNN、Postprocess、MPP 和 DMA-BUF 无失败。

所有 buffer/lease：

```text
shutdown before exit → 0
```

无：

* 无界 queue；
* 持续 memory growth；
* deadlock。

capture→result：

```text
mean ≈ 25.4～26.7 ms
p95 ≈ 29.8～30.2 ms
```

V7 不允许为了控制或打印日志明显破坏这些实时性基线。

---

# 七、V5 已冻结通信基线

RK3588 与 STM32F103ZET6 使用：

```text
RS-485
+
USART2
```

MCU RX：

```text
USART2 IRQ
→ SR/DR
→ uart_rx_ring
→ ProtocolRxTask
→ parser
→ CRC/version/type/length
→ ProtocolMessage_Decode
→ ProtocolDispatcher
```

MCU TX：

```text
ProtocolDispatcher
→ ProtocolResponse
→ ProtocolTxTask
→ ProtocolEngine_EncodeFrame
→ RS485 TX
```

重要约束：

```text
ProtocolTxTask
```

是唯一 RS-485 TX owner。

禁止其他 task 直接向 RS-485 UART 输出字节。

MCU 也禁止 unsolicited protocol transmission。

V7 不修改：

```text
portable/mcu_uart
frame parser
CRC
escaping
protocol framing
transaction cache
```

除非发现明确 bug。

---

# 八、V5 当前业务协议

已实现：

```text
HELLO
HELLO_ACK

HEARTBEAT
STATUS

CONTROL_UPDATE

REMOTE_STOP_REQUEST
CLEAR_REMOTE_STOP

ACK
NACK

PING
PONG
```

CONTROL_UPDATE：

```text
不回复
latest value wins
```

REMOTE_STOP/CLEAR：

```text
reliable transaction
```

remote stop latch：

```text
不会因掉线解除
不会因重新 HELLO 解除
```

peer reboot 或 link lost 后：

```text
必须重新 HELLO
```

---

# 九、V5 latest-control 结构

ProtocolDispatcher 验证后的 CONTROL_UPDATE：

```text
→ ControlMailbox
```

ControlMailbox：

```text
不是 FIFO
```

而是：

```text
latest control snapshot
```

结构：

```text
RX task = single writer
Gimbal task = single reader
```

包含：

```text
generation
```

云台不能执行历史 command backlog。

---

# 十、V5 两层 watchdog 已冻结

## Link Watchdog

约：

```text
1000 ms
```

没有合法 link frame：

```text
link → LOST
peer invalid
sequence invalid
control invalid
mailbox invalid
```

## Control Freshness Watchdog

约：

```text
200 ms
```

没有 fresh CONTROL_UPDATE：

```text
control_valid = false
mailbox invalid
```

但：

```text
link 可以保持 READY
```

V7 不随意改变这些值。

如最终需要调整，必须有闭环测试数据依据。

---

# 十一、V6 已完成真实执行器

MCU：

```text
STM32F103ZET6
```

云台：

```text
2-DOF
S20F positional servo
```

S20F 舵机内部具有自己的位置闭环。

本产品：

```text
不读取 S20F 实际位置
不增加外部 encoder
不增加 potentiometer feedback
```

因此 MCU 只能知道：

```text
servo command
```

不能知道真正的：

```text
measured servo angle
```

任何 STATUS 或日志中都不能把 command 值称为：

```text
actual position
actual angle
measured angle
```

应该使用：

```text
pan_command_us
tilt_command_us
pan_command_q15
tilt_command_q15
```

等语义。

---

# 十二、S20F 已冻结电气与机械基线

PWM：

```text
TIM3
50 Hz
```

Pan：

```text
TIM3_CH1
PA6
```

Tilt：

```text
TIM3_CH2
PA7
```

S20F 已验证 electrical range：

```text
500～2500 μs
```

但装机后安全软件范围已经缩小。

Pan：

```text
min     = 1000 μs
center  = 1500 μs
max     = 2000 μs
```

方向：

```text
PWM 增大
→ Camera LEFT
```

Tilt：

```text
min     = 1200 μs
center  = 1500 μs
max     = 1600 μs
```

方向：

```text
PWM 增大
→ Camera DOWN
```

这些 V6 calibration 是 V7 冻结基线。

没有测试数据不得扩大 software range。

---

# 十三、V6 FreeRTOS 基线

任务优先级：

```text
ProtocolRx = 5
ProtocolTx = 4
Gimbal     = 2
Idle       = 0
```

必须保持：

```text
ProtocolRx > ProtocolTx > Gimbal
```

GimbalTask：

```text
50 Hz
20 ms
```

使用固定周期调度 API。

禁止：

* busy wait；
* 软件 PWM；
* 长时间 HAL_Delay；
* 高频 malloc/free；

---

# 十四、V6 当前控制器

当前控制：

```text
error_x_q15 / error_y_q15
→ direction mapping
→ dead-zone
→ incremental proportional control
→ slew limit
→ software limit
→ servo
```

当前 dead-zone：

```text
2048 Q15
```

当前 full-scale incremental gain：

```text
4 μs / cycle
```

当前 slew rate：

```text
max 2 μs / 20 ms
```

也就是：

```text
100 μs/s
```

量级。

不要假设这些已经是最佳闭环参数。

这些只是 V6 actuator-safe baseline。

---

# 十五、V6 SafetyGate 已完成

只有：

```text
link READY
&&
!remote_stop_latched
&&
control_valid
&&
mailbox valid
&&
fresh generation requirement satisfied
```

才能使用 control。

已经验证：

* boot safe；
* REMOTE_STOP；
* CLEAR_STOP 不恢复旧命令；
* link loss；
* control stale；
* invalid mailbox；
* safety recovery 后必须等待 fresh generation。

V7 不重新设计 SafetyGate。

V7 控制算法必须服从 SafetyGate。

任何高级 controller 都不能绕过它。

---

# 十六、V6 已验证方向

当前 V6 只完成执行器本身方向和 calibration。

V7 第一件事仍需使用真实 Camera/YOLO 完成：

```text
image coordinate
→ physical camera motion
```

符号闭环确认。

如果 V4 图像坐标遵循典型：

```text
x right positive
y down positive
```

那么根据 V6 calibration，可以形成待验证假设：

```text
ball right
dx > 0
→ camera must pan RIGHT
→ Pan PWM should DECREASE
```

以及：

```text
ball below center
dy > 0
→ camera must tilt DOWN
→ Tilt PWM should INCREASE
```

这只能作为 hypothesis。

必须通过真实 Camera 实验确认后冻结。

---

# 十七、V7 的控制对象到底是什么

这是 V7 最重要的概念。

不要对 S20F 做外部“位置 PID”。

原因：

S20F 自己已经具有内部位置闭环，而我们的 STM32 没有舵机实际角度反馈。

因此 VisionArm 的外部闭环变量应该是：

```text
image feature error
```

定义：

```text
ex = error_x_normalized
ey = error_y_normalized
```

目标：

```text
ex → 0
ey → 0
```

系统闭环：

```text
image error
→ outer visual controller
→ servo command change
→ camera motion
→ new image error
```

这属于：

```text
Image-Based Visual Servoing
IBVS
```

的简化工程形式。

V7 可以称：

```text
image-space visual servo
```

或：

```text
image-error-based visual servo
```

不要声称实现了完整 interaction-matrix IBVS，除非实际真的实现。

---

# 十八、不要默认“完整 PID”

V7 必须先分析当前控制结构。

当前：

```text
servo_command[k+1]
=
servo_command[k]
+
K * image_error[k]
```

这是一种：

```text
image error
→ incremental position command
```

结构。

控制输出从物理意义上更接近：

```text
camera velocity-like command
```

然后积分到：

```text
servo position command
```

因此不能看到“闭环”两个字就直接增加：

```text
Kp + Ki + Kd
```

尤其不能盲目增加 Ki。

否则可能产生：

```text
image-error integration
+
incremental servo position integration
```

导致：

* windup；
* overshoot；
* slow recovery；
* software limit accumulation。

V7 推荐控制路线：

```text
P visual servo
→ evaluate
→ optional filtered D
→ only if evidence requires, evaluate I
```

---

# 十九、推荐 V7 控制路线

## Controller V0：P-only visual servo

建议形式：

```text
velocity_command_x = Kpx * ex
velocity_command_y = Kpy * ey
```

然后：

```text
servo_command += velocity_command * dt
```

实际实现可以继续使用：

```text
μs / control update
```

但文档中必须明确它代表：

```text
incremental actuator position command
```

不是 servo feedback error。

加入：

```text
dead zone
output clamp
slew rate
software limits
```

先把 P-only 调稳定。

---

## Controller V1：P + filtered derivative

只有出现：

```text
明显 overshoot
持续 oscillation
目标高速移动时预测不足
```

才评估 D。

建议对：

```text
image error derivative
```

或：

```text
measurement derivative
```

做低通滤波。

不得直接：

```text
Kd * (e[k] - e[k-1])
```

然后在有 detector jitter 的情况下无限放大噪声。

必须考虑：

```text
dt
```

---

## Controller V2：PI/PID

只有在实验明确证明：

```text
存在 persistent steady-state image bias
```

并且：

* P 调整无法消除；
* mechanical bias 已排查；
* target detector bias 已排查；
* dead-zone 不是主要原因；

才考虑 I。

如果使用 I：

必须实现：

```text
integrator clamp
anti-windup
reset conditions
```

I 项必须在以下情况 reset/freeze：

```text
LOST
STALE
INVALID
REMOTE_STOP
LINK_LOST
CONTROL_STALE
software limit saturation
```

---

# 二十、视觉采样与 GimbalTask 周期不是同一个东西

视觉：

```text
Camera ≈ 30 FPS
```

视觉 ControlResult：

```text
约 30 Hz
```

MCU GimbalTask：

```text
50 Hz
```

因此不能每次 GimbalTask 运行时都把同一个 visual error 当成：

```text
new measurement
```

尤其在实现：

```text
I
D
```

时。

必须利用：

```text
ControlMailbox generation
```

识别：

```text
new visual measurement
```

推荐：

```text
GimbalTask 50 Hz
```

继续负责：

* actuator slew；
* SafetyGate；
* hardware output。

但 controller 的 measurement update 只发生在：

```text
mailbox generation changed
```

时。

必须记录：

```text
visual_update_count
gimbal_cycle_count
duplicate_generation_cycles
```

---

# 二十一、控制 dt

PID/PD 或任何时间相关控制算法不能把：

```text
dt = 20 ms
```

硬编码为“视觉采样周期”。

视觉更新周期必须根据：

```text
MCU receive tick
```

或者可用的 visual message metadata 计算。

由于：

```text
RK3588 MONOTONIC
STM32 tick
```

属于独立时钟域，

不能直接做：

```text
Linux timestamp - MCU timestamp
```

作为控制 dt。

MCU 内部 controller 的 dt 推荐使用：

```text
当前新 generation 接收 tick
-
上一新 generation 接收 tick
```

并进行：

```text
min/max dt sanity clamp
```

---

# 二十二、目标丢失时的 controller state

现有 SafetyGate 会停 actuator 输出。

V7 controller 自己也必须处理：

```text
NO_TARGET
LOST
STALE
INVALID
```

不能：

```text
保持积分
保持 derivative history
保持上一次运动趋势
```

然后目标回来瞬间产生巨大输出。

建议：

```text
on target invalid:
    reset derivative history
    reset/freeze integral
    velocity command = 0
```

重新检测到目标时：

```text
first valid sample
→ controller reinitialize
```

---

# 二十三、software limit 和 controller windup

Pan：

```text
1000～2000 μs
```

Tilt：

```text
1200～1600 μs
```

如果 actuator 已达到 software limit：

例如：

```text
Pan = 1000 μs
```

而 controller 继续要求：

```text
camera RIGHT
```

不能：

* 继续累积 hidden position command；
* 继续累积 integral；
* 在重新离开 limit 后突然释放巨大累计量。

必须记录：

```text
limit_active
limit_direction_blocked
```

并进行 anti-windup。

---

# 二十四、V7 一句话目标

V7 的目标是：

在冻结 V4 实时感知、V5 RS-485 协议和 V6 执行器安全基线的前提下，完成真实的 `image error → S20F motion → new image error` 图像空间视觉伺服闭环，先通过 P 型 image-error controller 获得稳定跟踪，再根据真实 overshoot、oscillation 和 steady-state error 数据判断是否需要 D/I/PID，同时建立闭环误差、响应时间、追踪成功率、丢球恢复和安全状态指标；并将当前开发阶段的大量调试打印重构为具有编译期裁剪、运行时级别控制、模块过滤、限频和非阻塞输出能力的产品级日志系统。

---

# 二十五、V7 不做什么

V7 不做：

1. 不重新设计 V4 Camera pipeline；
2. 不重新设计 BufferBroker；
3. 不重新设计 RGA；
4. 不重新部署模型；
5. 不修改 YOLO 任务；
6. 不重新设计 V5 parser；
7. 不修改 CRC/framing；
8. 不重新设计 CONTROL_UPDATE；
9. 不重新设计 V6 PWM 基础；
10. 不扩大舵机软件范围；
11. 不增加 external encoder；
12. 不读取 S20F 内部位置；
13. 不做 motor position PID；
14. 不一开始做完整 PID；
15. 不做 Kalman tracker；
16. 不做 ByteTrack；
17. 不做自动变焦；
18. 不做 Audio/Video sync；
19. 不做 MP4/MKV mux；
20. 不做 RTSP；
21. 不做正式 SD 卡录像；
22. 不把 debug ASCII 插入 V5 RS-485 framed protocol；
23. 不为了打印日志阻塞 control thread/task；
24. 不把实验参数直接永久硬编码。

---

# 二十六、V7 子阶段建议

将 V7 拆成：

```text
V7.0 Instrumentation & Logging
V7.1 Loop Sign Verification
V7.2 Static-Target P Visual Servo
V7.3 Dynamic-Target Tracking
V7.4 Optional D / PID Evaluation
V7.5 Lost Target / Recovery
V7.6 Stability & Final Metrics
```

这样避免一次性调所有问题。

---

# 二十七、V7.0 产品级日志系统

当前测试程序存在大量调试打印。

调试阶段这些日志有价值。

但是 release 产品不能：

```text
每帧 printf
每个 UART byte printf
```

否则会：

* 增加 CPU；
* 增加锁竞争；
* 改变 UART timing；
* 扰动真实控制实验。

V7 必须完成日志收敛。

---

# 二十八、日志级别

推荐统一：

```text
FATAL
ERROR
WARN
INFO
DEBUG
TRACE
OFF
```

含义：

```text
FATAL
系统无法继续正常工作

ERROR
当前功能失败但系统可能继续

WARN
异常但可恢复

INFO
生命周期和重要状态变化

DEBUG
开发调试信息

TRACE
高频内部详细信息
```

Release 默认：

```text
INFO
或 WARN
```

根据测量冻结。

Debug build 默认：

```text
DEBUG
```

禁止 Release 默认 TRACE。

---

# 二十九、日志模块

至少：

RK3588：

```text
PIPELINE
CAMERA
BROKER
RGA
RKNN
POSTPROCESS
CONTROL
UART
MPP
SAFETY
PERF
```


必须支持：

```text
global log level
+
per-module level
```

例如：

```text
global = INFO
```

---

# 三十、compile-time + runtime 两层日志控制

必须设计：

```text
compile-time maximum level
+
runtime current level
```

例如：

```c
LOG_COMPILED_LEVEL
```

Release：

```text
DEBUG/TRACE code 可以完全编译掉
```

Runtime：

```text
log level 可以动态调整
```

但 runtime 不能恢复已经 compile-out 的 TRACE。

---

# 三十一、Linux 动态日志控制

优先设计一个简单产品接口，例如：

```text
visionarmctl log get
visionarmctl log set CONTROL DEBUG
visionarmctl log set RKNN INFO
visionarmctl log set all WARN
```

实现方式可以采用：

```text
Unix domain socket
```

或现有应用 control channel。

不要为了日志建立复杂网络服务器。

配置修改必须是：

```text
thread-safe
non-blocking
```

---


# 三十四、严禁同步日志阻塞实时路径

禁止：

```text
Camera thread
Inference thread
```

执行耗时格式化输出。

推荐：

```text
producer
→ fixed-size log/event record
→ bounded queue/ring
→ log sink
```

队列满：

```text
drop log
```

而不是：

```text
block controller
```


---

# 三十五、高频日志限频

以下不能每周期打印：

```text
dx
dy
30fps inference result
```

推荐：

```text
rate-limited debug
```

例如：

```text
1 Hz summary
```

或：

```text
every N samples
```

出现异常事件则立即：

```text
WARN/ERROR
```

例如：

```text
REMOTE_STOP
LINK_LOST
CONTROL_STALE
LIMIT_HIT
PIPELINE_ERROR
```

应该事件触发记录。

---

# 三十六、控制测试不能靠 console log 做数据采集

闭环数据必须进入：

```text
CSV
binary metrics
structured telemetry
```

不能靠：

```text
grep printf output
```

计算性能。

日志：

```text
给人看
```

Metrics：

```text
给测试程序分析
```

必须分开。

---

# 三十七、V7 控制数据日志

RK3588 每个有效 ControlResult 至少记录：

```text
frame_id
v4l2_sequence

target_state
confidence

error_x
error_y

capture_timestamp
result_timestamp
result_age

control_sequence
uart_submit_timestamp
```

---

# 三十八、闭环数据关联

推荐使用：

```text
source_frame_sequence
+
control wire sequence
+
mailbox generation
```

完成因果关联。

最终希望能够追踪：

```text
Frame N
→ Detection
→ ControlResult
→ Wire Sequence X
→ MCU Generation G
→ Servo command
→ Later Frame M
→ new image error
```

即使没有舵机 encoder，也能研究：

```text
visual closed-loop response
```

---

# 三十九、V7 第一阶段：方向确认

先不要调 gain。

使用静态足球。

人工放置：

```text
left
right
up
down
```

每次只允许：

```text
极小 actuator command
```

验证：

```text
dx sign
→ pan physical direction
→ next dx magnitude decreases
```

以及：

```text
dy sign
→ tilt physical direction
→ next dy magnitude decreases
```

核心判断：

```text
control action之后
|error[k+1]| < |error[k]|
```

至少趋势应正确。

如果误差变大：

```text
立即 STOP
```

先查：

* coordinate convention；
* sign mapping；
* actuator direction；

禁止靠调低 Kp 掩盖符号错误。

---

# 四十、静态目标 P-only 调参

先只调：

```text
Pan
```

Tilt 固定。

然后只调：

```text
Tilt
```

最后双轴。

调参顺序：

```text
very low gain
→ increase gradually
→ observe convergence
→ observe oscillation
```

每个 gain 都必须保存：

```text
error vs time
servo command vs time
```

禁止凭肉眼说：

```text
这个参数感觉挺好
```

---

# 四十一、dead-zone 调参

当前：

```text
2048 Q15
```

只是 V6 baseline。

V7 必须测：

```text
center jitter distribution
```

在球静止且云台稳定时：

记录：

```text
error_x
error_y
```

至少 30～60 秒。

根据噪声：

```text
p95 / p99
```

决定 dead-zone。

原则：

dead-zone 太小：

```text
servo hunting
```

dead-zone 太大：

```text
center accuracy 差
```

---

# 四十二、gain 调参

Pan 与 Tilt：

```text
不允许默认同一个 gain
```

因为：

* mechanical range 不同；
* pulse range 不同；
* mount geometry 不同；
* servo load 不同。

必须分别：

```text
Kx
Ky
```

当前 V6：

```text
4 μs/full-scale/cycle
```

只作为起点之一。

---

# 四十三、slew-rate 调参

当前：

```text
2 μs / 20 ms
```

V7 需要评估：

太低：

```text
tracking slow
```

太高：

```text
overshoot
mechanical shock
oscillation
```

Pan/Tilt 可以不同：

```text
pan_slew
tilt_slew
```

---

# 四十四、目标静态阶跃实验

建立标准场景。

例如让球从中心突然移动到：

```text
±10%
±20%
±30%
```

图像宽/高位置。

记录：

```text
e(t)
servo_command(t)
```

统计：

```text
response time
peak error
overshoot
settling time
steady-state error
zero crossings
```

因为没有 servo angle feedback：

所有指标必须定义在：

```text
image domain
```

而不是：

```text
servo angle domain
```

---

# 四十五、V7 建议的核心误差指标

至少：

```text
mean_abs_error_x
mean_abs_error_y

rms_error_x
rms_error_y

p95_abs_error_x
p95_abs_error_y

normalized_radial_error
```

可定义：

```text
e_r = sqrt(ex^2 + ey^2)
```

记录：

```text
mean e_r
p95 e_r
```

---

# 四十六、center band

定义可配置：

```text
center_band_x
center_band_y
```

例如未来可以说：

```text
|ex| < threshold_x
&&
|ey| < threshold_y
```

视为：

```text
CENTERED
```

实际 threshold 必须实验确定。

不能第一轮随便定 5%。

---

# 四十七、settling time

定义清楚：

从：

```text
target step / reacquisition
```

到：

```text
error enters center band
```

并连续保持：

```text
T_hold
```

的时间。

例如：

```text
hold 0.5 s
```

只是候选。

必须在测试规范中冻结。

---

# 四十八、overshoot

在图像空间定义。

例如目标最初：

```text
ex > 0
```

控制后越过：

```text
ex = 0
```

并达到：

```text
negative peak
```

计算 overshoot。

不要使用舵机位置 overshoot，因为没有实际 position feedback。

---

# 四十九、tracking success

必须给出可重复定义。

例如一次测试期间：

```text
target visible time
```

中满足：

```text
DETECTED
fresh
within accepted tracking band
```

的时间比例。

具体公式和 threshold 必须文档化。

---

# 五十、动态目标测试

不要一开始测试高速踢球。

按难度：

```text
Level 1:
手持球缓慢横向移动

Level 2:
缓慢纵向移动

Level 3:
二维移动

Level 4:
中等速度

Level 5:
短时间快速移动
```

每级记录：

```text
tracking success
center error
lost count
recovery time
servo saturation
```

---

# 五十一、丢球策略

V7 不做复杂主动搜索。

目标 LOST：

```text
actuator velocity = 0
```

或保持当前安全 servo position。

不要：

```text
继续最后方向旋转
```

除非未来明确设计 search mode。

目标重新出现：

```text
controller state reset
fresh generation
→ resume
```

---

# 五十二、丢球恢复时间

定义：

```text
target becomes detectable again
→
state becomes stable DETECTED
→
error returns into tracking band
```

分别记录：

```text
reacquisition latency
recenter latency
```

---

# 五十三、P→D/PID 升级判断

P-only 如果满足：

* no persistent oscillation；
* settling 可接受；
* steady-state error 可接受；

则：

```text
不要加 PID
```

只有以下证据出现才升级。

## D candidate

如果：

```text
overshoot large
repeated zero crossing
oscillation
```

考虑：

```text
PD / filtered derivative
```

## I candidate

如果：

```text
稳定 persistent offset
```

且已经排除：

* dead zone；
* detector bias；
* mechanical range；
* servo calibration；

再考虑 I。

---

# 五十四、Controller interface

推荐保持独立模块：

```cpp
struct VisualServoInput {
    bool valid;
    int16_t error_x_q15;
    int16_t error_y_q15;
    uint32_t generation;
    uint32_t receive_tick;
};

struct VisualServoOutput {
    int32_t pan_delta;
    int32_t tilt_delta;
};
```

接口语义：

```text
visual measurement in
→ desired actuator increment/velocity out
```

Controller 不直接写：

```text
TIM3
```

仍由：

```text
ActuatorDriver
```

负责硬件。

---

# 五十五、配置外部化

V7 所有调参量必须集中：

```text
gimbal_control_config
```

至少：

```text
pan_sign
tilt_sign

dead_zone_x
dead_zone_y

kp_x
kp_y

kd_x
kd_y

ki_x
ki_y

derivative_filter

pan_slew
tilt_slew

pan_min_us
pan_max_us

tilt_min_us
tilt_max_us

integrator_limit
```

没有启用的参数：

```text
明确 = 0 / disabled
```

禁止散落 magic number。

---

# 五十六、运行时控制参数调整

为了避免每次调 Kp 都重新编译 MCU，V7 可以评估：

```text
runtime tuning
```

但必须遵守 V5 protocol stability。

推荐优先级：

## P0

参数使用：

```text
compile-time config
```

每组实验重新烧录也可以。

## P1

设计：

```text
host-triggered temporary tuning command
```

但必须：

* backward compatible；
* reliable；
* range checked；
* 不写 Flash；
* reboot 后恢复默认；
* 不影响 safety semantics。

## P2

persistent configuration。

不要在 V7 为参数系统设计复杂数据库。

---



# 五十八、V7 Markdown 文档模板要求

第一次回复必须直接给出以下文件的可复制 Markdown 模板。

## visual_servo_architecture.md

必须包含：

* system plant；
* image feature；
* error definition；
* actuator；
* S20F internal position loop；
* external visual loop；
* no external encoder；
* controller location；
* timing；
* safety boundary；
* V4/V5/V6 frozen interfaces；
* block diagram。

---

## coordinate_sign_contract.md

包含：

* image x sign；
* image y sign；
* Pan PWM direction；
* Tilt PWM direction；
* predicted mapping；
* physical test；
* final mapping；
* PASS evidence。

---

## controller_design.md

必须包含：

* normalized error；
* Q15 representation；
* new measurement detection；
* dt；
* P visual servo；
* incremental position；
* dead-zone；
* slew；
* software limit；
* optional derivative；
* optional integral；
* anti-windup；
* reset conditions；
* SafetyGate interaction；
* why not motor position PID。

---

## controller_tuning.md

包含：

* baseline；
* static target；
* axis separation；
* gain sweep；
* dead-zone sweep；
* slew sweep；
* acceptance criteria；
* P decision；
* D decision；
* I decision；
* final frozen values。

---

## logging_architecture.md

包含：

* Linux logging；
* log levels；
* module filters；
* compile-time level；
* runtime level；
* queue/ring；
* sink；
* rate limiting；
* dropped log；
* metrics vs logs；
* RS-485 isolation；
* release behavior。

---

## logging_policy.md

必须明确每个级别应该打印什么。

例如：

ERROR：

```text
Camera fatal
RKNN failure
UART disconnect
actuator initialization fault
```

WARN：

```text
target stale
queue drop threshold
link loss
limit reached
log dropped
```

INFO：

```text
startup
shutdown
HELLO/session
controller mode change
REMOTE_STOP transition
```

DEBUG：

```text
controller sample summary
selected target
periodic timing
```

TRACE：

```text
per-frame/per-cycle internals
```

Release policy：

```text
默认只保留必要 INFO/WARN/ERROR
```

---

## test_protocol.md

必须定义：

* target；
* distance；
* camera resolution；
* FPS；
* lighting；
* ball size；
* initial gimbal pose；
* target placement；
* test duration；
* controller config；
* pass conditions；
* repetition count；
* raw data path。

---

## static_target_report.md

必须统计：

* initial error；
* gain；
* dead-zone；
* slew；
* response；
* overshoot；
* settling；
* steady error；
* saturation；
* final result。

---

## dynamic_tracking_report.md

至少：

* target motion level；
* mean error；
* p95 error；
* success rate；
* target lost count；
* recovery；
* actuator saturation；
* notes。

---

## lost_recovery_report.md

包含：

* lost trigger；
* output behavior；
* controller reset；
* reacquisition；
* recenter；
* repeated trials；
* statistics。

---

## safety_regression_report.md

必须重新回归：

* REMOTE_STOP；
* CLEAR；
* LINK_LOST；
* CONTROL_STALE；
* INVALID；
* boot；
* reset；
* software limits。

确保 V7 controller 没有破坏 V6 SafetyGate。

---

## performance_report.md

至少：

RK3588：

```text
Camera FPS
Inference FPS
capture→result mean/p95/p99
UART control rate
CPU
memory
NPU
log overhead
```

MCU：

```text
GimbalTask period
GimbalTask execution
visual measurement rate
duplicate generation
controller updates
limit count
stop events
```

闭环：

```text
mean abs image error
RMS image error
p95 error
settling
overshoot
tracking success
lost recovery
```

---

## troubleshooting.md

至少覆盖：

1. 球在右边云台向左；
2. 球在下方云台向上；
3. 双轴方向耦合；
4. gain 太低；
5. gain 太高；
6. hunting；
7. overshoot；
8. detector jitter；
9. derivative noise；
10. integrator windup；
11. software limit；
12. lost target；
13. stale result；
14. repeated generation；
15. visual update 30 Hz vs Gimbal 50 Hz；
16. UART delay；
17. control lag；
18. servo mechanical latency；
19. rapid target motion；
20. log-induced jitter；
21. printf blocking；
22. RS-485 被 debug text 污染；
23. log queue overflow；
24. Release logs too verbose；
25. controller state not reset。

---

## v7_acceptance.md

状态只能：

```text
PASS
WARN
FAIL
BLOCKED
```

必须包含 V7→V8 准入条件。

---

# 五十九、V7 P0 必须完成

1. 图像坐标符号确认；
2. Pan 闭环符号确认；
3. Tilt 闭环符号确认；
4. true physical loop；
5. P-only visual servo；
6. dead-zone baseline；
7. independent Kx/Ky；
8. independent slew；
9. generation-aware controller update；
10. real dt；
11. target loss reset；
12. stale reset；
13. SafetyGate regression；
14. software-limit anti-windup behavior；
15. static target step test；
16. Pan single-axis tuning；
17. Tilt single-axis tuning；
18. dual-axis test；
19. basic dynamic target tracking；
20. center-error metrics；
21. settling metrics；
22. overshoot metrics；
23. tracking success definition；
24. lost recovery；
25. 10-minute closed-loop stability；
26. Linux log levels；
27. Linux runtime log control；
29. debug output does not corrupt RS-485；
30. compile-time log filtering；
31. runtime log filtering；
32. rate limiting；
33. log-overhead measurement；
34. release log policy；
35. documents；
36. README；
37. Git commit。

---

# 六十、P1 推荐完成

1. filtered D experiment；
2. 30～60 min tracking；
3. multiple gain profiles；
4. low/medium target speeds；
6. log module runtime mask；
7. controller runtime parameter tuning；
8. log drop counters；
9. structured binary event records；
10. multiple lighting conditions；
11. multiple ball sizes/distances；
12. step-response automated analysis。

---

# 六十一、P2 可以延期

1. full PID；
2. complex IBVS interaction matrix；
3. camera calibration model；
4. Kalman；
5. ByteTrack；
6. target prediction；
7. auto search on LOST；
8. gain scheduling；
9. feed-forward；
10. adaptive control；
11. encoder；
12. servo telemetry；
13. multi-target；
14. high-speed kicked-ball tracking。

---


