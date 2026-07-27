# 如何编译以及使用
交叉编译：
source /opt/atk-dlrk3588-toolchain/environment-setup
/usr/bin/cmake \
  -S src/linux_app/camera \
  -B build/v2_camera \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/cmake/rk3588-toolchain.cmake"

/usr/bin/cmake \
  --build build/v2_camera \
  --verbose
rm -rf build/v2_camera
cp -a build/v2_camera/v4l2_camera_capture /home/liu2004/nfs_dir/

./v4l2_capturea \
  --device <VIDEO_DEVICE> \
  --width <WIDTH> \
  --height <HEIGHT> \
  --format <FOURCC> \
  --fps <FPS> \
  --buffers 4 \
  --frames 300 \
  --timeout-ms 2000 \
  --output-dir reports/images/v2 \
  --save-first 1 \
  --csv logs/v2_camera/frames.csv \
  --nonblock \
  | tee logs/v2_camera/smoke_test.log

./v4l2_capture \
  --device /dev/video22 \
  --width 3840 \
  --height 2160 \
  --format NV12 \
  --fps 60 \
  --buffers 4 \
  --frames 300 \
  --timeout-ms 2000 \
  --output-dir reports/images/v2 \
  --save-first 3 \
  --csv logs/v2_camera/frames.csv \
  --nonblock \
  | tee logs/v2_camera/smoke_test.log

# 用户态采集程序。
  完整执行流程

  ```
  解析命令行
    ↓
stat + open
    ↓
VIDIOC_QUERYCAP
    ↓
选择 single-planar 或 multi-planar
    ↓
VIDIOC_G_FMT
VIDIOC_S_FMT
VIDIOC_G_FMT
    ↓
VIDIOC_G_PARM
VIDIOC_S_PARM
VIDIOC_G_PARM
    ↓
VIDIOC_REQBUFS
    ↓
每个 buffer 执行 VIDIOC_QUERYBUF
    ↓
每个 buffer/plane 执行 mmap
    ↓
所有 buffer 执行 VIDIOC_QBUF
    ↓
VIDIOC_STREAMON
    ↓
poll → VIDIOC_DQBUF → 处理数据 → VIDIOC_QBUF
    ↓
VIDIOC_STREAMOFF
    ↓
munmap
    ↓
close
  ```

# MMAP 采集的核心原理
1. MMAP 不等于程序自己分配图像缓冲区

这里的流程是：

用户程序调用 VIDIOC_REQBUFS。
驱动为采集队列分配若干缓冲区。
用户程序调用 VIDIOC_QUERYBUF，取得每个缓冲区的长度和 mmap offset。
用户程序调用 mmap()，把这些驱动缓冲区映射到自己的虚拟地址空间。
摄像头或 DMA 将数据写入这些缓冲区。
驱动通过 VIDIOC_DQBUF 告诉程序：“第几个缓冲区已经填好”。
程序直接从映射地址读取数据。
处理结束后通过 VIDIOC_QBUF 把缓冲区归还驱动。

V4L2 文档将这种方式描述为：应用程序与驱动交换的是缓冲区指针和元数据，而不是通过 read() 将整帧从内核复制到另一个用户缓冲区。

因此它通常被称为“零拷贝采集”，但要准确理解：

避免的是一次传统的“内核缓冲区 → 用户缓冲区”复制。
硬件仍然需要通过 DMA 或其他方式把图像写入缓冲区。
saveFrame() 写磁盘时仍然会发生用户空间到文件系统的复制。
CSV 输出也会产生普通文件 I/O。

内核文档的[说法](https://docs.kernel.org/userspace-api/media/v4l/mmap.html)
```
Input and output devices support this I/O method when the V4L2_CAP_STREAMING flag in the capabilities field of struct v4l2_capability returned by the ioctl VIDIOC_QUERYCAP ioctl is set. There are two streaming methods, to determine if the memory mapping flavor is supported applications must call the ioctl VIDIOC_REQBUFS ioctl with the memory type set to V4L2_MEMORY_MMAP.

Streaming is an I/O method where only pointers to buffers are exchanged between application and driver, the data itself is not copied. Memory mapping is primarily intended to map buffers in device memory into the application’s address space. Device memory can be for example the video memory on a graphics card with a video capture add-on. However, being the most efficient I/O method available for a long time, many other drivers support streaming as well, allocating buffers in DMA-able main memory.

A driver can support many sets of buffers. Each set is identified by a unique buffer type value. The sets are independent and each set can hold a different type of data. To access different sets at the same time different file descriptors must be used. [1]

To allocate device buffers applications call the ioctl VIDIOC_REQBUFS ioctl with the desired number of buffers and buffer type, for example V4L2_BUF_TYPE_VIDEO_CAPTURE. This ioctl can also be used to change the number of buffers or to free the allocated memory, provided none of the buffers are still mapped.

Before applications can access the buffers they must map them into their address space with the mmap() function. The location of the buffers in device memory can be determined with the ioctl VIDIOC_QUERYBUF ioctl. In the single-planar API case, the m.offset and length returned in a struct v4l2_buffer are passed as sixth and second parameter to the mmap() function. When using the multi-planar API, struct v4l2_buffer contains an array of struct v4l2_plane structures, each containing its own m.offset and length. When using the multi-planar API, every plane of every buffer has to be mapped separately, so the number of calls to mmap() should be equal to number of buffers times number of planes in each buffer. The offset and length values must not be modified. Remember, the buffers are allocated in physical memory, as opposed to virtual memory, which can be swapped out to disk. Applications should free the buffers as soon as possible with the munmap() function.
```

2. 缓冲区所有权循环

一个 MMAP buffer 在运行中大致经过以下状态：

```
应用拥有，尚未排队
        │
        │ VIDIOC_QBUF
        ▼
驱动 incoming queue
等待硬件写入
        │
        │ 硬件完成一帧
        ▼
驱动 outgoing queue
缓冲区已完成
        │
        │ VIDIOC_DQBUF
        ▼
应用重新拥有
读取、保存、统计
        │
        └──────────── VIDIOC_QBUF ────────────┐   
                                              │
                                              ▼
                                         驱动继续复用
```

缓冲区通过 QBUF 交给驱动后，应用程序不应再访问它；直到 DQBUF 成功，缓冲区才重新进入应用程序域。V4L2 的 QUEUED、DONE 等标志描述的就是这个状态机。