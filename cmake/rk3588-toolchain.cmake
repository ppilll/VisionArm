set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TOOLCHAIN_BIN
    "/opt/atk-dlrk3588-toolchain/bin")

set(CMAKE_C_COMPILER
    "${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-gcc")

set(CMAKE_CXX_COMPILER
    "${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-g++")