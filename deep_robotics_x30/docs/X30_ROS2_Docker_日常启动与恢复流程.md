# X30 ROS2 Docker 日常启动与恢复流程

更新时间：2026-07-08  
项目路径：`D:\Desktop\RoboDog`  
机器狗路径：`/home/ysc/x30_livox_ros2_transfer`

## 1. 每次测试会关闭什么

启动我们的 ROS2 Docker 传感器容器前，需要临时关闭原厂 ROS1 的硬件接收：

```text
1. 原厂 ROS1 Livox 雷达驱动
   脚本：remote/02_factory_livox_off.sh
   作用：调用 /home/ysc/jy_cog/drivers/scripts/lidar_driver_stop.sh livox
   目的：释放 Livox UDP 端口，让 Docker 内 ROS2 Livox 驱动直接接收四雷达数据。

2. 原厂 ROS1 Yesense 本体 IMU 驱动
   脚本：remote/11_factory_imu_off.sh
   作用：调用 imu_driver_stop.sh，或在脚本不存在时 kill /yesense_imu_node 和相关进程
   目的：释放 /dev/ttyS0，让 Docker 内 ROS2 Yesense 驱动直接接收本体 IMU。
```

单容器启动脚本还会清理旧的 ROS2 测试容器：

```text
x30_ros2_sensors
x30_livox_ros2
x30_cloud_merger
x30_body_imu
```

注意：

```text
remote/18_run_ros2_all.sh 不会主动关闭原厂 ROS1 雷达和 IMU。
它只会检查 Livox UDP 端口和 /yesense_imu_node 是否已经释放。
所以每次启动前仍然先手动执行 02 和 11。
```

## 2. 启动 ROS2 单容器

这些命令必须在机器狗宿主机执行，不是在 Docker 容器内部执行。

宿主机提示符通常类似：

```text
ysc@ysc-perception:...$
```

如果看到：

```text
root@ysc-perception:/ws#
```

说明还在 Docker 内部，需要先执行：

```bash
exit
```

日常启动流程：

```bash
cd /home/ysc/x30_livox_ros2_transfer

bash remote/02_factory_livox_off.sh
bash remote/11_factory_imu_off.sh
bash remote/18_run_ros2_all.sh
```

检查当前容器：

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}'
```

期望只看到一个 ROS2 传感器容器：

```text
x30_ros2_sensors
```

## 3. 查看 ROS2 话题

推荐直接运行检查脚本：

```bash
bash remote/19_check_ros2_all.sh
```

期望看到：

```text
/livox/lidar
/livox/imu
/x30/points_merged
/x30/body_imu
```

也可以进入容器手动查看：

```bash
docker exec -it x30_ros2_sensors bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 node list
ros2 topic list
```

查看频率：

```bash
timeout 8 ros2 topic hz /livox/lidar
timeout 8 ros2 topic hz /livox/imu
timeout 8 ros2 topic hz /x30/points_merged
timeout 8 ros2 topic hz /x30/body_imu
```

参考频率：

```text
/livox/lidar        约 40Hz
/livox/imu          约 800Hz
/x30/points_merged  约 8-10Hz
/x30/body_imu       约 200Hz
```

## 4. 测试结束后恢复原厂 ROS1

测试结束必须停止 ROS2 容器，并恢复原厂 ROS1 雷达和 IMU：

```bash
cd /home/ysc/x30_livox_ros2_transfer

bash remote/20_stop_ros2_all.sh
bash remote/12_factory_imu_on.sh
bash remote/06_factory_livox_on.sh
bash remote/01_status.sh
```

恢复后重点确认：

```text
ROS1 /yesense_imu_node 正常
ROS1 /imu/data 有频率
ROS1 Livox 相关节点正常
ROS1 /lidar_points 有频率
```

如果只想看 ROS1 话题：

```bash
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash
rostopic list | grep -E "lidar|livox|imu"
timeout 8 rostopic hz /lidar_points -w 3
timeout 8 rostopic hz /imu/data -w 3
```

## 5. 当前安全边界

当前这个 Docker 只接收传感器数据：

```text
不发布 /cmd_vel
不发送速度控制
不控制机器狗运动
不修改原厂 ROS1 文件
只在测试期间临时停用原厂雷达和 IMU 接收进程
```
