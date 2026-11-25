# Mini V1.5 ROS2 上位机使用说明

## 环境和构建

WSL Ubuntu 24.04 使用 ROS 2 Jazzy 和系统 Python 3.12。新终端执行：

```bash
dev ros2
cd ~/projects/tianbot-mini-v1.5/develop/tianbot_core
./build.sh
source install-wsl/setup.bash
```

构建脚本只扫描当前包，默认限制两个编译任务。生成目录为 `build-wsl/`、`install-wsl/`、`log-wsl/`。
Humble 环境也可使用此脚本。ESP-IDF 终端请先退出，再执行 `dev ros2`。

## 扫描设备

```bash
ros2 run tianbot_core mini_v1_5_scan
```

扫描器向 `255.255.255.255:5003` 广播发现请求，至少扫描 2 秒，连续 1.5 秒无新增设备后结束，最长 10 秒。它只显示设备，不申请控制会话，也不发送速度命令。

扫描器退出后再启动驱动。扫描器和多车驱动不能同时占用 UDP 5003。

## 启动驱动

单车：

```bash
ros2 launch tianbot_core mini_v1_5_multi.launch.py \
  vehicle_ips:=192.168.1.3
```

多车：

```bash
ros2 launch tianbot_core mini_v1_5_multi.launch.py \
  vehicle_ips:=192.168.1.3,192.168.1.4
```

广播发现所有设备：

```bash
ros2 launch tianbot_core mini_v1_5_multi.launch.py vehicle_ips:=*
```

`vehicle_ips` 是驱动的目标车辆列表。单车或指定多车时必须填写；只有扫描器只查看设备时不需要填写。`discovery_address` 通常自动选择，使用 `vehicle_ips:=*` 时可显式指定 `255.255.255.255`。

启动日志按普通白色输出参数、发现和上线信息；只有参数错误、端口绑定失败等错误使用红色。正常格式示例：

```text
[INFO] [tianbot_core_multi] Mini V1.5 multi-driver starting
[INFO] [tianbot_core_multi] Vehicle discovered: Robot5DE0 ip=192.168.1.3 ...
[INFO] [tianbot_core_multi] Control session acknowledged: Robot5DE0
[INFO] [tianbot_core_multi] Vehicle online: Robot5DE0 ip=192.168.1.3 ns=/Robot5DE0
```

## 话题

设备名会生成 ROS 命名空间。例如设备名为 `Robot5DE0`：

```text
/Robot5DE0/cmd_vel
/Robot5DE0/imu
/Robot5DE0/odom
/Robot5DE0/vehicle_state
/Robot5DE0/battery_state
/Robot5DE0/spi_link
/Robot5DE0/scan
```

可选调试雷达话题由 `publish_lidar_debug:=true` 开启：

```text
/Robot5DE0/lidar_points
/Robot5DE0/lidar_raw
```

当前实测话题频率约为：IMU 25 Hz、里程计 25 Hz、车辆状态 50 Hz、电池状态 50 Hz、SPI 链路 4.9 Hz、雷达 `/scan` 约 9.94 Hz。ROS `/scan` 发布 1440 个角度格，C5 网页 `/api/lidar` 使用 90 个角度格聚合数据。

## 控制和安全

架空低速测试：

```bash
ros2 topic pub -r 20 /Robot5DE0/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

停止：

```bash
ros2 topic pub -1 /Robot5DE0/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

驱动将速度限制在 `max_linear_velocity=1.5`、`max_angular_velocity=2.0`，默认 500 ms 没有新速度命令时发送 STOP。测试必须架空或确认车辆周围无障碍物。

## 当前边界

已验证设备发现、会话、核心状态话题、雷达和架空低速控制。尚未完成多车长稳、落地带载和正式安全验收。完整回归记录见两个固件工程的 `docs/test/regression_2026-11-06.md`。
