# X30 ROS2 Livox MID360 package

This folder is the minimal transfer package for host 192.168.1.106.

Base image required on 106:

```bash
jezetek:navigation_system_amd64
```

This package adds:

```text
ws/src/livox_ros_driver2
ws/src/x30_livox_bringup
ws/src/x30_pointcloud_tools
ws/src/yesense_interface
ws/src/yesense_std_ros2
config/x30_multi_mid360_ros2.json
```

Suggested remote path:

```bash
/home/ysc/x30_livox_ros2_transfer
```

After upload:

```bash
cd /home/ysc/x30_livox_ros2_transfer
chmod +x remote/*.sh
```

Build derived image:

```bash
bash remote/00_build_image.sh
```

Expected image:

```text
x30_livox_ros2:jezetek
```

Check current status:

```bash
bash remote/01_status.sh
```

Stop factory ROS1 Livox driver:

```bash
bash remote/02_factory_livox_off.sh
```

Start ROS2 Livox driver:

```bash
bash remote/03_run_ros2_livox.sh
```

Watch logs:

```bash
docker logs -f x30_livox_ros2
```

Check ROS2 topics:

```bash
bash remote/04_check_ros2_topics.sh
```

Start 100 ms point cloud merger:

```bash
bash remote/07_start_cloud_merger.sh
```

Check merged output:

```bash
bash remote/08_check_merged_cloud.sh
```

Expected merged output:

```text
/x30/points_merged
sensor_msgs/msg/PointCloud2
about 10 Hz
```

Stop point cloud merger only:

```bash
bash remote/09_stop_cloud_merger.sh
```

Stop ROS2 Livox and restore factory ROS1 Livox:

```bash
bash remote/05_stop_ros2_livox.sh
bash remote/06_factory_livox_on.sh
bash remote/01_status.sh
```

Scope:

```text
ROS2 native Livox MID360 receiving only.
No robot velocity command is sent.
No /cmd_vel is published.
No navigation algorithm is started.
```

## Factory Body IMU

The X30 factory body IMU is a Yesense ROS1 driver. On the tested robot:

```text
Start script: /home/ysc/jy_cog/drivers/scripts/imu_driver.sh
Stop script:  /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh
ROS1 node:    /yesense_imu_node
ROS1 topic:   /imu/data
Subscriber:   /localization_node
Device:       /dev/ttyS0
Baudrate:     115200
```

Use the IMU scripts only during a stationary test window. Stopping the factory
IMU does not modify factory code, but it removes `/imu/data` from the localization
node until the factory driver is restored.

Check factory body IMU:

```bash
bash remote/10_factory_imu_status.sh
```

Stop factory body IMU:

```bash
bash remote/11_factory_imu_off.sh
```

Restore factory body IMU:

```bash
bash remote/12_factory_imu_on.sh
```

Probe factory IMU config:

```bash
bash remote/13_factory_imu_config_probe.sh
```

## ROS2 Native Body IMU Test

The package includes a ROS2 Yesense driver adapted for the X30 factory body IMU:

```text
Source: Avalue-Technology/ros2.humble.amr.avalue avalue_imu/yesense_ros2
Device: /dev/ttyS0
Baudrate: 115200
Frame: imu_link
ROS2 topic: /x30/body_imu
Raw Yesense topic: /x30/body_imu_raw
Serial backend: linux_serial
```

The factory ROS1 Yesense node must be stopped before starting the ROS2 driver,
because only one process can read `/dev/ttyS0`.

```bash
bash remote/10_factory_imu_status.sh
bash remote/11_factory_imu_off.sh
bash remote/14_run_ros2_body_imu.sh
bash remote/15_check_ros2_body_imu.sh
bash remote/17_probe_ros2_body_imu_raw.sh
bash remote/16_stop_ros2_body_imu.sh
bash remote/12_factory_imu_on.sh
```

Do not touch `/dev/ttyS1`; it is used by `RobotServ` on the tested robot.

Current tested status:

```text
/x30/body_imu publishes sensor_msgs/msg/Imu at about 200Hz.
Angular velocity and linear acceleration are valid.
Raw Yesense Euler topics are valid.
Raw Yesense quaternion fields are currently 0,0,0,0 on the tested robot.
The driver now falls back to Euler -> quaternion conversion for /x30/body_imu.orientation
when the raw quaternion is invalid.
```
