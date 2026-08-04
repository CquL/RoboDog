# X30 ROS2 passive sensor and HAL image

This is the production transfer package for host `192.168.1.106`.

It derives one image from:

```text
jezetek:navigation_system_amd64
```

The image contains only:

```text
components/robot_hardware_x30   X30 hardware abstraction library
ws/src/x30_sensor_receiver      passive 105-to-106 sensor receiver
```

Livox and Yesense hardware remain owned by the factory ROS1 system on host
`192.168.1.105`. The Docker image does not open sensor devices, bind Livox
ports, stop factory processes, or reproduce the factory stair terrain chain.

## Data path

```text
105 factory ROS1
  /lidar_points
  /imu/data
  /leg_odom
        |
        v
105 x30_sensor_forwarder (read-only subscriber)
        |
        | TCP 56110 / 56111 / 56112
        v
106 x30_ros2_passive
  /x30/lidar_points
  /x30/body_imu
  /x30/leg_odom
```

The separately deployed 105 forwarder is maintained under:

```text
deep_robotics_x30/transport/x30_sensor_relay/x30_sensor_forwarder_105
```

## Build

The base image must already be loaded on host 106:

```text
jezetek:navigation_system_amd64
```

Build and verify the derived image:

```bash
cd /home/ysc/x30_livox_ros2_transfer
chmod +x remote/*.sh
bash remote/00_build_image.sh
```

Expected image:

```text
x30_livox_ros2:jezetek
```

The build verifies:

- the passive ROS2 receiver;
- the staged X30-only HAL;
- the UDP protocol and factory contracts;
- dynamic linkage to `/opt/x30_robot_hardware/lib/librobot_hardware_x30.so`.

Local source synchronization and static contract tests are maintained outside
the production transfer package:

```text
deep_robotics_x30/tools/x30_passive_image_maintenance/
```

## Start

Start the receiver on host 106 first:

```bash
bash remote/26_run_passive_relay.sh
```

Then start the read-only forwarder on host 105:

```bash
cd /home/ysc/x30_sensor_forwarder_105
bash remote/02_start_forwarder_105.sh
```

Check the receiver:

```bash
bash remote/27_check_passive_relay.sh
bash remote/28_status_passive_relay.sh
```

Stop it:

```bash
bash remote/29_stop_passive_relay.sh
```

## HAL verification

Run the no-network HAL verification:

```bash
bash remote/30_verify_x30_hal_offline.sh
```

The guarded live check sends heartbeat, connection confirmation, and zero
velocity only:

```bash
bash remote/31_test_x30_hal_zero.sh CONFIRM_X30_ZERO_ONLY
```

The live check requires both automatic mode/source configuration flags to
remain `false`.

The installed HAL is located at:

```text
/opt/x30_robot_hardware/include/robot_hardware/
/opt/x30_robot_hardware/lib/librobot_hardware_x30.so
/opt/x30_robot_hardware/bin/robot_test_x30
```

## Current control boundary

The HAL library implements:

```cpp
initRobotHardware()
writeRobotVelocityCommand(...)
writeActionCommand(...)
```

There is not yet a long-running ROS2 `/cmd_vel` adapter in this image.
Navigation software cannot control the robot by publishing `/cmd_vel` until a
single-owner `x30_hardware_controller` node is added.

That node must provide:

- `/cmd_vel` to `writeRobotVelocityCommand`;
- a restricted gait/action service to `writeActionCommand`;
- command timeout and final zero velocity on shutdown;
- low-frequency state and freshness interlocks.

## Offline archives

Code removed from the production build is retained outside this package:

```text
deep_robotics_x30/factory/plane_seg_offline/
deep_robotics_x30/sensors/legacy_direct_docker_snapshot/
```

Those directories are research and recovery material, not production runtime
dependencies.
