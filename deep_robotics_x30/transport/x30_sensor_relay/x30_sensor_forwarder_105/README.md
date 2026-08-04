# X30 105 ROS1 只读传感器转发包

部署目标：

```text
192.168.1.105（感知主机，Ubuntu 20.04 aarch64，ROS1 Noetic）
```

本包只订阅：

```text
/lidar_points  sensor_msgs/PointCloud2
/imu/data      sensor_msgs/Imu
/leg_odom      nav_msgs/Odometry
```

并通过三个独立 TCP 连接发送到 `192.168.1.106`：

```text
56110  点云
56111  IMU
56112  里程计
```

它不会发布 ROS1 Topic，不调用 ROS service，不修改原厂 rosparam，也不会发送
运动命令。`roslaunch` 只加载本节点自己的私有连接参数。

上传到 105 后：

```bash
cd /home/ysc
tar -xzf x30_sensor_forwarder_105.tar.gz
cd x30_sensor_forwarder_105
chmod +x remote/*.sh

bash remote/00_build_105.sh
bash remote/01_check_sources_105.sh
```

先在 106 启动被动接收容器，然后在 105 启动转发：

```bash
bash remote/02_start_forwarder_105.sh
bash remote/03_status_forwarder_105.sh
```

停止我们自己的转发节点：

```bash
bash remote/04_stop_forwarder_105.sh
```

停止脚本只操作自己记录的 PID，不停止任何原厂 ROS1 节点。
