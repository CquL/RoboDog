# X30 Pro ROS2 Docker 接入阶段记录

更新时间：2026-07-06  
项目路径：`D:\Desktop\RoboDog`  
目标机器：X30 Pro 导航/感知主机 `192.168.1.106`

## 1. 项目目标

目标是在不破坏云深处 X30 Pro 原厂 ROS1 系统的前提下，搭建我们自己的 ROS2 Docker 环境，用于接收机器狗传感器数据，并为后续上层算法接入做准备。

当前原则：

```text
1. 原厂 ROS1 系统保留，不改原厂代码。
2. 测试时短时间关闭原厂雷达/IMU 驱动，释放硬件资源。
3. ROS2 Docker 直接接收硬件数据。
4. 测试完成后恢复原厂 ROS1 驱动。
5. 当前阶段只接收数据，不发布 /cmd_vel，不控制机器狗运动。
```

## 2. 机器狗网络和硬件关系

X30 Pro 主要主机：

| 主机 | IP | 作用 |
|---|---:|---|
| 运动主机 | `192.168.1.103` | 底层运动、步态、速度执行 |
| 感知主机 | `192.168.1.105` | 传感器、地形图、停避障 |
| 导航/智能控制主机 | `192.168.1.106` | 建图、定位、导航、Docker 测试 |

当前我们在 `192.168.1.106` 上测试。

106 主机网口：

```text
enp3s0: 192.168.1.106/24, 192.168.9.106/24
eno1:   192.168.2.106/24
```

四个 MID360 雷达：

```text
192.168.2.202
192.168.2.203
192.168.2.204
192.168.2.205
```

关键理解：

```text
192.168.2.106 是 106 主机连接雷达网络的网口 IP，不是交换机 IP。
Docker 使用 --network host 后，可以直接复用 106 主机网络接收雷达 UDP 数据。
```

## 3. 镜像导入与 ROS2 接入镜像构建

我们先在机器狗上导入基础 Docker 镜像包：

```text
D:\Desktop\RoboDog\jezetek_navigation_amd64.tar
```

导入后得到基础镜像：

```text
jezetek:navigation_system_amd64
```

```bash
cd /home/ysc
docker load -i jezetek_navigation_amd64.tar
docker images | grep jezetek
```

这个镜像作为基础环境。我们没有直接修改它，而是基于它构建了自己的派生镜像：

```text
jezetek:navigation_system_amd64
        ↓
x30_livox_ros2:jezetek
```

本地接入包：

```text
D:\Desktop\RoboDog\x30_livox_ros2_transfer
D:\Desktop\RoboDog\x30_livox_ros2_transfer.tar.gz
```

我们在派生镜像中加入了：

```text
1. livox_ros_driver2
2. 四雷达 MID360 配置
3. X30 雷达启动和点云工具包 x30_livox_tools
4. Yesense ROS2 IMU 驱动
5. Euler -> Quaternion 修复
6. remote/*.sh 启动、关闭、检查脚本
```

```bash
cd /home/ysc/ros2_trans
tar -xzf x30_livox_ros2_transfer.tar.gz
cd x30_livox_ros2_transfer
chmod +x remote/*.sh
bash remote/00_build_image.sh
```

构建成功后得到：

```text
x30_livox_ros2   jezetek   约 3.5GB
```

如果旧版 Docker 报错 `unknown flag: --progress`，执行：

```bash
sed -i 's/ --progress=plain//g' remote/00_build_image.sh
bash remote/00_build_image.sh
```

## 4. 已完成工作一：ROS2 Docker 接收四雷达

完成内容：

```text
关闭原厂 ROS1 Livox 驱动，释放雷达 UDP 端口。
启动 ROS2 Docker Livox 驱动，直接接收四个 MID360 雷达数据。
```

```bash
cd /home/ysc/ros2_trans/x30_livox_ros2_transfer
bash remote/02_factory_livox_off.sh
bash remote/03_run_ros2_livox.sh
bash remote/04_check_ros2_topics.sh
```

验证结果：

```text
/livox/lidar   sensor_msgs/msg/PointCloud2   约 40Hz
/livox/imu     sensor_msgs/msg/Imu           约 800Hz
```

```text
/livox/imu 是 MID360 雷达内置 IMU，不是机器狗本体 Yesense IMU。
```

## 5. 已完成工作二：四雷达点云融合

完成内容：

```text
订阅 ROS2 雷达驱动发布的 /livox/lidar。
按 100 ms 时间窗口融合四雷达点云。
发布合并后的 /x30/points_merged。
```

```bash
bash remote/07_start_cloud_merger.sh
bash remote/08_check_merged_cloud.sh
```

验证结果：

```text
/x30/points_merged   sensor_msgs/msg/PointCloud2   约 8-10Hz
```

```text
输入：/livox/lidar
输出：/x30/points_merged
窗口：100 ms
```

## 6. 已完成工作三：ROS2 Docker 接收本体 Yesense IMU

原厂本体 IMU 信息：

```text
ROS1 节点：/yesense_imu_node
ROS1 话题：/imu/data
消息类型：sensor_msgs/Imu
频率：约 200Hz
串口：/dev/ttyS0
波特率：115200
订阅者：/localization_node
```

完成内容：

```text
关闭原厂 ROS1 Yesense 驱动，释放 /dev/ttyS0。
Docker 启动时映射 /dev/ttyS0。
ROS2 Yesense 驱动直接读取本体 IMU。
发布标准 ROS2 IMU 话题 /x30/body_imu。
```

```bash
bash remote/10_factory_imu_status.sh
bash remote/11_factory_imu_off.sh
bash remote/14_run_ros2_body_imu.sh
bash remote/15_check_ros2_body_imu.sh
```

验证结果：

```text
/x30/body_imu   sensor_msgs/msg/Imu   约 200Hz
frame_id: imu_link
```

```text
三轴线加速度 linear_acceleration
三轴角速度 angular_velocity
姿态四元数 orientation
```

## 7. 已完成工作四：修复 IMU 四元数

```text
初次测试发现：
- 加速度正常
- 角速度正常
- 欧拉角 roll/pitch/yaw 正常
- 原始 quaternion 为 0,0,0,0
- /x30/body_imu.orientation 为 0,0,0,0
```

```bash
bash remote/17_probe_ros2_body_imu_raw.sh
```

```text
检查结果：
- /euler_only 有有效 pitch/roll/yaw
- /att_min_vru 有有效 euler
- /att_all 有有效 euler
- /robot_lord 和 /att_all 的 quat 仍为 0
```

```text
修复方式：
在 ROS2 Yesense 驱动中加入 euler -> quaternion fallback。
如果原始 quat 有效，则使用原始 quat。
如果原始 quat 无效但 euler 有效，则用 roll/pitch/yaw 计算 quaternion。
```

```text
修复后验证：
/x30/body_imu.orientation:
  x: 0.0048250287
  y: -0.0055170983
  z: 0.0005613017
  w: 0.9999729825

/x30/body_imu 频率约 200Hz
linear_acceleration.z 约 9.73 m/s^2
```

```text
ROS2 Docker 已经可以直接接收机器狗本体 Yesense IMU，并输出完整标准 IMU 数据。
```

## 8. 当前已完成的结果

已经完成：

```text
1. 离线安装 Docker
2. 导入基础镜像 jezetek:navigation_system_amd64
3. 基于 jezetek 构建 x30_livox_ros2:jezetek
4. ROS2 Docker 直接接收四个 MID360 雷达
5. ROS2 Docker 合并四雷达点云
6. ROS2 Docker 直接接收本体 Yesense IMU
7. IMU Euler -> Quaternion 修复
8. 测试后恢复原厂 ROS1 雷达和 IMU
```

当前 ROS2 关键话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 原始四雷达点云，约 40Hz |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 内置 IMU，约 800Hz |
| `/x30/points_merged` | `sensor_msgs/msg/PointCloud2` | 合并点云，约 8-10Hz |
| `/x30/body_imu` | `sensor_msgs/msg/Imu` | 本体 Yesense IMU，约 200Hz |

## 9. 当前操作步骤：从原厂 ROS1 切到 ROS2 Docker

### 9.1 查看当前状态

```bash
cd /home/ysc/ros2_trans/x30_livox_ros2_transfer
bash remote/01_status.sh
```

查看 Docker 容器：

```bash
docker ps --filter name=x30
```

查看 Docker 镜像：

```bash
docker images | grep -E "jezetek|x30_livox"
```

### 9.2 切换雷达到 ROS2 Docker

关闭原厂 ROS1 雷达：

```bash
bash remote/02_factory_livox_off.sh
```

启动 ROS2 雷达容器：

```bash
bash remote/03_run_ros2_livox.sh
```

检查 ROS2 雷达话题：

```bash
bash remote/04_check_ros2_topics.sh
```

启动点云融合：

```bash
bash remote/07_start_cloud_merger.sh
```

检查合并点云：

```bash
bash remote/08_check_merged_cloud.sh
```

### 9.3 切换本体 IMU 到 ROS2 Docker

检查原厂 ROS1 IMU：

```bash
bash remote/10_factory_imu_status.sh
```

关闭原厂 ROS1 IMU：

```bash
bash remote/11_factory_imu_off.sh
```

启动 ROS2 IMU 容器：

```bash
bash remote/14_run_ros2_body_imu.sh
```

检查 ROS2 IMU：

```bash
bash remote/15_check_ros2_body_imu.sh
```

检查 Yesense 原始扩展话题：

```bash
bash remote/17_probe_ros2_body_imu_raw.sh
```

### 9.4 查看 ROS2 话题

进入任意一个 x30 容器查看：

```bash
docker exec -it x30_body_imu bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic list
'
```

查看频率：

```bash
docker exec -it x30_body_imu bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
timeout 8 ros2 topic hz /x30/body_imu
'
```

如果雷达容器正在运行：

```bash
docker exec -it x30_cloud_merger bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
timeout 8 ros2 topic hz /x30/points_merged
'
```

## 10. 当前操作步骤：恢复原厂 ROS1 系统

测试结束后必须恢复原厂驱动。

停止 ROS2 IMU：

```bash
bash remote/16_stop_ros2_body_imu.sh
```

恢复原厂 ROS1 IMU：

```bash
bash remote/12_factory_imu_on.sh
```

如果上面脚本恢复失败，使用手动恢复：

```bash
nohup bash /home/ysc/jy_cog/drivers/scripts/imu_driver.sh > /tmp/factory_imu_driver_manual.log 2>&1 &
sleep 5

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash
rostopic info /imu/data
timeout 8 rostopic hz /imu/data -w 3
```

停止 ROS2 点云融合：

```bash
bash remote/09_stop_cloud_merger.sh
```

停止 ROS2 雷达：

```bash
bash remote/05_stop_ros2_livox.sh
```

恢复原厂 ROS1 雷达：

```bash
bash remote/06_factory_livox_on.sh
```

恢复后检查：

```bash
bash remote/01_status.sh
```

原厂正常状态应看到：

```text
ROS1 Livox 节点恢复
ROS1 /imu/data 约 200Hz
/yesense_imu_node 正常运行
```

## 11. 迁移到另一台机器狗需要的文件

必需文件：

```text
jezetek_navigation_amd64.tar
x30_livox_ros2_transfer.tar.gz
```

如果新机器没有 Docker，还需要：

```text
docker_offline_focal_amd64_fix1.tar.gz
```

建议携带：

```text
docs/X30_ROS2_Docker_接入阶段记录.md
multi_MID360_config.json
```

迁移流程：

```bash
docker load -i jezetek_navigation_amd64.tar

mkdir -p /home/ysc/ros2_trans
cd /home/ysc/ros2_trans
tar -xzf x30_livox_ros2_transfer.tar.gz
cd x30_livox_ros2_transfer
chmod +x remote/*.sh
bash remote/00_build_image.sh
```

## 12. 后续计划

下一阶段建议：

```text
1. 把雷达、点云融合、本体 IMU 做成一键启动脚本。
2. 接入原厂 /odom 或 /leg_odom，给 ROS2 算法提供定位/里程计。
3. 接入 robot_hardware，把算法输出转换成 /cmd_vel 或目标点。
4. 保持地形图安全层，不直接绕过原厂底层运动控制。
```
