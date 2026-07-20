# RKNN模型
# 核心组件
1. RNKK-Toolkit2:pc端模型转换工具，负责将ONNX/PyTorch/TensorFlow 等框架模型转换为 NPU 可执行的.rknn格式，同时支持量化、精度分析、性能评估等功能。
2. RKNN Runtime:板端推理运行时，分为Python 版rknn-toolkit-lite2和 C++ 版librknnrt.so，负责在开发板上执行.rknn模型的推理。

# 环境搭建
# PC 端 RKNN-Toolkit2 环境搭建
推荐使用 Ubuntu 20.04/22.04 x86_64 系统，Python 版本选择 3.8/3.10，步骤如下：

1. 下载 RKNN-Toolkit2 安装包

从官方仓库下载对应版本：https://github.com/airockchip/rknn-toolkit2

2. 安装 Python wheel 包

```
# 以2.3.2版本、Python3.8为例
pip install rknn_toolkit2-2.3.2-xxxx-cp38-cp38-linux_x86_64.whl
# 验证安装
python3 -c "from rknn.api import RKNN; print('RKNN-Toolkit2安装成功')"
```

# 板端环境配置
以 RK3588 Debian/Ubuntu 系统为例：

1. Python rknnlite 环境

# 下载对应版本的rknn_toolkit_lite2 wheel包，安装
pip install rknn_toolkit_lite2-2.3.2-xxxx-cp38-cp38-linux_aarch64.whl
# 验证
python3 -c "from rknnlite.api import RKNNLite; print('rknnlite安装成功')"
2. C++ Runtime 环境

板端系统一般已预装librknnrt.so，路径为/usr/lib/librknnrt.so，开发时需从官方 SDK 中获取对应的头文件rknn_api.h，编译时链接该动态库即可。