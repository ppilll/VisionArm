1. 板卡与 SoC

实机证据已经确认：

- Device Tree model：ATK-DLRK3588 Board
- compatible：rockchip,rk3588-evb7-lp4-v10、rockchip,rk3588
- hostname：ATK-DLRK3588
- Kernel：5.10.160
- 架构：aarch64
- 启动参数明确包含 storagemedia=emmc

2. Kernel、RootFS 与 ABI

实机运行：

- Kernel        5.10.160
- Kernel arch   aarch64
- RootFS        Buildroot 2021.11
- libc          glibc 2.37
- userspace     64-bit
- ELF loader    /lib/ld-linux-aarch64.so.1
- PID 1         init

3. 内存与 eMMC

内存实测为：

- MemTotal: 8113036 kB
- free 显示总量：7.7 GiB

eMMC 实测：

- 设备：mmcblk0
- 类型：MMC
- 容量：58.3 GiB
- 分区总容量：61112320 KiB
- 根分区：mmcblk0p6，约 14 GiB
- /oem：mmcblk0p7
- /userdata：mmcblk0p8，约 43 GiB

4. 网络与设备节点

当前网络基线：

- eth1：UP，192.168.50.2/24
- 链路：1 Gbps / Full
- NFS：192.168.50.1 挂载到 /mnt/nfs
- eth0：无载波
- wlan0：DOWN