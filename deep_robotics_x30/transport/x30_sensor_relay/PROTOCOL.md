# X30 Sensor Relay Protocol v1

## Transport

105 主动连接 106。每种消息使用独立 TCP 流，避免大点云阻塞 IMU 和里程计：

```text
56110  sensor_msgs/PointCloud2
56111  sensor_msgs/Imu
56112  nav_msgs/Odometry
```

所有整数和 IEEE-754 浮点值均按 little-endian 编码。字符串为
`uint32 length + UTF-8 bytes`。每个 TCP 帧由 32 字节固定头和变长 payload
组成。

## Frame Header

```text
offset  size  field
0       4     magic = "X30R"
4       2     protocol_version = 1
6       2     stream_id: cloud=1, imu=2, odometry=3
8       8     sequence
16      8     source_stamp_ns
24      4     payload_size
28      4     reserved = 0
```

106 只接受默认来源 `192.168.1.105`，payload 上限为 64 MiB。magic、版本、
stream 或长度不符合契约时，当前连接会被拒绝或关闭。

## PointCloud2 Payload

```text
uint32 ros1_header_sequence
string frame_id
uint32 height
uint32 width
uint32 field_count
repeat field_count:
  string field_name
  uint32 offset
  uint8 datatype
  uint32 count
uint8 is_bigendian
uint32 point_step
uint32 row_step
uint8 is_dense
uint32 data_size
uint8 data[data_size]
```

ROS1 的时间戳保存在帧头 `source_stamp_ns`，106 原样恢复。点字段、排列和原始
字节不做点云算法变换。

## Imu Payload

```text
uint32 ros1_header_sequence
string frame_id
float64 orientation_xyzw[4]
float64 orientation_covariance[9]
float64 angular_velocity_xyz[3]
float64 angular_velocity_covariance[9]
float64 linear_acceleration_xyz[3]
float64 linear_acceleration_covariance[9]
```

## Odometry Payload

```text
uint32 ros1_header_sequence
string frame_id
string child_frame_id
float64 position_xyz[3]
float64 orientation_xyzw[4]
float64 pose_covariance[36]
float64 linear_velocity_xyz[3]
float64 angular_velocity_xyz[3]
float64 twist_covariance[36]
```

## Queue And Reconnect Rules

- 点云：队列深度 1，始终保留最新帧。
- IMU、里程计：有界队列，默认深度 100。
- TCP 断开或发送失败：清空该流积压的数据，再重连。
- 106 连续 2 秒没有收到当前帧的任何字节时关闭旧连接，允许 105 重连。
- 不进行历史补发，不允许断线期间的数据在恢复后集中回放。
- 三个流分别统计 received、sent、dropped、failures 和 sequence gaps。
