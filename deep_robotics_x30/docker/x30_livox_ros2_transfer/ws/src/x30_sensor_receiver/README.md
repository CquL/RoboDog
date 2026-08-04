# X30 passive sensor receiver

This ROS2 package listens for the versioned TCP frames emitted by
`x30_sensor_forwarder_ros1` on host `192.168.1.105`.

Default streams:

```text
TCP 56110 -> /x30/lidar_points  sensor_msgs/msg/PointCloud2
TCP 56111 -> /x30/body_imu      sensor_msgs/msg/Imu
TCP 56112 -> /x30/leg_odom      nav_msgs/msg/Odometry
```

The receiver does not open Livox UDP ports, does not access `/dev/ttyS0`, and
does not publish any robot control command.
