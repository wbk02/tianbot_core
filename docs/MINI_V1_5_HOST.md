# MINI V1.5 上位机使用与验收说明

## 启动

在本地 ROS2 环境中打开终端，输入 `2` 加载环境，然后执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_study_ws/install/setup.bash --extend
source <工作区>/install/setup.bash
ros2 launch tianbot_core mini_v1_5.launch.py \
  namespace:=tianbot \
  remote_ip:=192.168.4.1 \
  command_port:=5000 \
  management_port:=5003 \
  bind_telemetry_port:=5001 \
  bind_lidar_port:=5002
```

`<工作区>`替换为实际上位机工作区。正式 C5/S3 使用时不要使用测试脚本和回环地址。

## 独立广播扫描

只查看局域网中的 Mini V1.5 设备时，运行独立扫描程序。它不需要填写 IP、MAC
或其他参数，也不需要另开终端读取 ROS 话题：

```bash
source "$(ros2go_switch -v 2)"
cd ~/Desktop/MIni_dev/tianbot-mini-v1.5/develop/tianbot_core
source install/setup.bash
ros2 run tianbot_core mini_v1_5_scan
```

程序每秒向 `255.255.255.255:5003` 广播发现请求，并在当前终端直接打印设备名、
IP、MAC、固件版本、状态和故障。扫描至少运行 2 秒；连续 1.5 秒没有发现新设备后
自动结束，最长运行 10 秒。设备按 MAC 去重，并且只接受对应本次扫描请求的响应。
按 `Ctrl+C` 可提前退出。扫描程序只负责发现，不会申请
控制会话，也不会发送速度或 STOP 指令；它与多车控制节点不能同时占用 UDP 5003。

单车多车节点也可以只填写一个 IP；程序会自动把该 IP 同时作为发现目标：

```bash
ros2 launch tianbot_core mini_v1_5_multi.launch.py \
  vehicle_ips:=192.168.1.4
```

只有使用 `vehicle_ips:=*` 扫描所有设备时，才需要使用默认广播地址，或显式填写
`discovery_address:=255.255.255.255`。

## 话题映射

节点使用启动参数中的 `namespace`，因此默认话题名称如下；下位机数据上传后会直接发布到这些固定名称：

| 数据 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 速度控制 | `tianbot/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`、`angular.z`发送到C5 |
| IMU | `tianbot/imu` | `sensor_msgs/msg/Imu` | 四元数写入 orientation |
| 里程计 | `tianbot/odom` | `nav_msgs/msg/Odometry` | frame 为 `odom`，子 frame 为 `base_link` |
| 雷达 | `tianbot/scan` | `sensor_msgs/msg/LaserScan` | 90点、毫米转米 |
| 车辆状态 | `tianbot/vehicle_state` | `std_msgs/msg/UInt32` | bit 0~7 状态，bit 8~23 故障，bit 24~31 来源 |
| 电压 | `tianbot/voltage` | `std_msgs/msg/Float32` | 单位伏特 |
| 电池 | `tianbot/battery_state` | `sensor_msgs/msg/BatteryState` | 当前填充 voltage |
| 链路 | `tianbot/spi_link` | `std_msgs/msg/Bool` | C5链路在线状态 |

修改 `namespace:=mini` 后，所有话题统一变为 `/mini/...`，不会散落到其他命名空间。

## RViz

固定坐标系使用 `odom`；添加 `Odometry` 选择 `<namespace>/odom`，添加 `LaserScan` 选择 `<namespace>/scan`，并确认 `odom -> base_link` TF。雷达 frame 默认是 `laser`，可通过 `lidar_frame:=...` 修改。

## 控制与安全

上位机完成发现、会话授权后才接受 `/cmd_vel`。速度按 `max_linear_velocity` 和 `max_angular_velocity` 限幅。超过 `command_timeout_ms` 没有新的速度指令会发送 STOP；链路超过 3 秒没有收到遥测也会停止并发布 `spi_link=false`。

## 已修复的问题

- 首次请求 `session_id=0` 时原条件会阻止会话申请，已修正为只判断当前会话是否已建立。
- IMU 32 字节负载原先被误当成加速度和角速度，现按时间戳加四元数解析。
- 接收 socket 原来每 10 ms 只读取一个包，状态、IMU、里程计和雷达分片突发时会积压；现扩大接收缓冲并在一次回调中排空当前队列。
- 测试模拟器只放在 `/tmp`，不属于工程安装内容，也不会参与正式启动。


## 2026-10-31 帧率与恢复修正

正式设计上传要求：IMU、odom、车辆状态各 50 Hz，雷达滚动快照 10 Hz。
`cmd_vel` 由上游按 50 Hz 输入；驱动不重放旧速度，输入超时仍发送 STOP。
心跳不重复发布车辆状态，以免把管理帧混入状态采样率。
STOP 后、闲置租约失效后和断线后重新握手；持续输入时若 S3 状态仍为 READY
超过 500 ms，放弃本地会话并重新申请，后续仅转发新输入。
雷达限制最多 90 个分片，完整帧后的重复分片直接丢弃，避免清空数组越界。
雷达 90 格包含无效距离（NaN），发布帧率不代表物理雷达每格均有效。
实物测试与日志见 S3 工程 `docs/rate_acceptance_2026-10-31/`。


## RViz 雷达显示配置

保留上位机驱动运行，在 ROS2 终端执行（源码路径）：

```bash
rviz2 -d /home/tianbot/Desktop/MIni_dev/tianbot-mini-v1.5/develop/tianbot_core/rviz/mini_lidar.rviz
```

默认 Fixed Frame=laser，PointCloud2=/tianbot/lidar_points，Reliability=Best Effort，绿色点表示原始测量。可启用“建图扫描”查看 /tianbot/scan。两者均为 SensorDataQoS，Reliable 订阅无法连接 Best Effort 发布端。直接在 laser 中看图不需要 base_link→laser TF；正式建图须标定该 TF。
