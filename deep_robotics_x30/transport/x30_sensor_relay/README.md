# X30 105 到 106 被动传感器转发

本目录保存一条不抢占传感器、不关闭原厂 ROS1 链路的跨主机数据通道。

```text
192.168.1.105 感知主机
  原厂 ROS1 驱动和 LIO 保持运行
  /lidar_points
  /imu/data
  /leg_odom
        |
        | x30_sensor_forwarder_ros1，只订阅
        | TCP 56110 / 56111 / 56112
        v
192.168.1.106 开发主机
  x30_ros2_passive 容器
  x30_sensor_receiver
        |
        +-> /x30/lidar_points
        +-> /x30/body_imu
        +-> /x30/leg_odom
```

这不是 `ros1_bridge`。105 节点读取 ROS1 消息并使用项目内的有版本 TCP
协议发送，106 节点还原为 ROS2 消息。该模式不连接 ROS1
`ROS_MASTER_URI`，也不会发布或修改 105 的 ROS1 话题。

## 目录

```text
x30_sensor_forwarder_105/       部署到 192.168.1.105
PROTOCOL.md                     TCP 协议和消息字段
package_transfers.ps1           覆盖生成两台主机的唯一传输包
tests/                          本地静态契约测试
```

106 接收端源码位于：

```text
deep_robotics_x30/docker/x30_livox_ros2_transfer/
  ws/src/x30_sensor_receiver/
  remote/26_run_passive_relay.sh
  remote/27_check_passive_relay.sh
  remote/28_status_passive_relay.sh
  remote/29_stop_passive_relay.sh
```

## 部署顺序

先在 106 启动监听端：

```bash
cd /home/ysc/x30_livox_ros2_transfer
chmod +x remote/*.sh
bash remote/00_build_image.sh
bash remote/26_run_passive_relay.sh
```

再在 105 启动只读转发端：

```bash
cd /home/ysc/x30_sensor_forwarder_105
chmod +x remote/*.sh
bash remote/00_build_105.sh
bash remote/01_check_sources_105.sh
bash remote/02_start_forwarder_105.sh
```

检查：

```bash
# 105
bash remote/03_status_forwarder_105.sh

# 106
bash remote/27_check_passive_relay.sh
```

停止时先停 105，再停 106：

```bash
# 105
bash remote/04_stop_forwarder_105.sh

# 106
bash remote/29_stop_passive_relay.sh
```

## 安全边界

- 105 转发节点只有 ROS1 Subscriber，没有 Publisher 或 Service Client；
  `roslaunch` 只加载自身私有参数，不写原厂参数。
- 不调用 `/lio_enable`，不切换步态，不发送速度或地形数据。
- 不停止原厂 Livox、IMU、LIO、高度图、`plane_seg` 或 `gridmap_port`。
- 106 被动容器不使用 `--privileged`，不挂载 `/dev/ttyS0`，不占用 Livox UDP。
- 点云队列固定为 latest-only 深度 1；网络阻塞时丢弃旧帧，禁止积压后延迟回放。
- 该通道完成的是传感器共享，不等于 45 度楼梯算法或实机闭环已经完成。

## 唯一交付包

在 Windows 工作区运行：

```powershell
cd D:\Desktop\RoboDog
powershell -ExecutionPolicy Bypass -File `
  .\deep_robotics_x30\transport\x30_sensor_relay\package_transfers.ps1
```

脚本覆盖生成：

```text
deep_robotics_x30/artifacts/x30_sensor_forwarder_105.tar.gz
deep_robotics_x30/artifacts/x30_livox_ros2_transfer.tar.gz
```

不另外创建带日期或 `single` 后缀的重复压缩包。
