# unitree_h2_sensor_bridge

This ROS 2 Humble node subscribes directly to the H2 native HG DDS channels:

- `rt/lowstate` (`LowState_.imu_state`) for the pelvis IMU;
- `rt/secondary_imu` (`IMUState_`) for the torso/body IMU.

It publishes project-owned standard ROS 2 topics:

- `/h2/imu/pelvis` (`sensor_msgs/msg/Imu`);
- `/h2/imu/torso` (`sensor_msgs/msg/Imu`).

The bridge does not subscribe to PC2 ROS topics and does not publish any H2
control channel. Unitree DDS and project ROS 2 may use separate DDS domains:

```text
H2_DDS_DOMAIN=0
ROS_DOMAIN_ID=20
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Unitree SDK2 uses its bundled CycloneDDS internally. The project ROS 2 side
uses the Fast DDS RMW already present in the offline Humble base image.

The first implementation uses local receive time because `IMUState_` has no
source timestamp. Quaternion input order is `Qw,Qx,Qy,Qz`. Covariance remains
zero (unknown) until robot-specific calibration is supplied. Frame axes and
sensor extrinsics must be verified on each delivered H2 before navigation
fusion is accepted.
