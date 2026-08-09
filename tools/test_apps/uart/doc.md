source /opt/atk-dlrk3588-toolchain/environment-setup

/usr/bin/cmake \
  -S tools/test_apps/uart  \
  -B build/v5-r5 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo

/usr/bin/cmake \
  --build build/v5-r5 \
  --target visionarm_uart_standalone visionarm_uart_wire_qualification \
  --parallel "$(nproc)"

cp -a build/v5-r5/visionarm_uart_standalone /home/liu2004/nfs_dir/
cp -a build/v5-r5/visionarm_uart_wire_qualification /home/liu2004/nfs_dir/
 
./visionarm_uart_standalone \
  --device /dev/ttyS3 \
  --baud 115200 \
  --mode smoke \
  --control-rate-hz 30 \
  --heartbeat-ms 200 \
  --duration-sec 60 \
  --report reports/v5/r5/l2_smoke_60s.json
1

  DEVICE=/dev/ttyS3 \
BAUD=115200 \
CONTROL_RATE_HZ=30 \
HEARTBEAT_MS=200 \
./run_r5_qualification.sh

./visionarm_uart_wire_qualification \
  --device /dev/ttyS3 \
  --baud 115200 \
  --mode transaction-edge \
  --report reports/v5/l3_transaction_edge.json \
  2>&1 | tee reports/v5/logs_l3_transaction_edge.txt

./visionarm_uart_standalone \
--device /dev/ttyS3 \
--baud 115200 \
--mode interactive \
--duration-sec 30 \
--report reports/v5/manual_clear.json