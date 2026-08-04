#ifndef UNITREE_H2_SENSOR_BRIDGE_IMU_CONVERSION_HPP_
#define UNITREE_H2_SENSOR_BRIDGE_IMU_CONVERSION_HPP_

// H2 IMU 数据转换的纯函数接口。
//
// 数据所有权与边界：
// - 输入来自 Unitree SDK2/HG DDS，只读取消息，不向机器人发布任何 DDS 数据；
// - 输出是 ROS 2 sensor_msgs/Imu，供 Docker 内导航/感知节点订阅；
// - 本文件不初始化 DDS、ROS 2 或运动控制客户端，因而不能产生运动指令。

#include <cmath>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

namespace unitree_h2_sensor_bridge {

using VendorImuState = unitree_hg::msg::dds_::IMUState_;

// 将一帧宇树 HG IMUState 映射成 ROS 2 Imu 消息。
//
// 参数说明：
// - source：DDS 收到的原厂 IMU 数据；
// - stamp：桥接节点收到该帧数据时记录的 ROS 时钟；
// - frame_id：输出数据所属的 TF 坐标系名称；
// - destination：转换成功后写入的 ROS 2 消息。
//
// 返回 false 表示源数据含 NaN/Inf 或四元数不可归一化；调用方必须丢弃
// 该帧，避免无效姿态进入导航算法。
inline bool ConvertImu(
    const VendorImuState &source,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    sensor_msgs::msg::Imu &destination) {
  const auto &quaternion = source.quaternion();
  const auto &gyroscope = source.gyroscope();
  const auto &accelerometer = source.accelerometer();

  // 先验证所有传感器分量为有限数；任何一个异常都拒绝整帧数据。
  for (const float value : quaternion) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const float value : gyroscope) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const float value : accelerometer) {
    if (!std::isfinite(value)) {
      return false;
    }
  }

  // 原厂四元数理论上应为单位长度；这里显式归一化，兼容浮点累计误差。
  // 接近零的四元数没有有效姿态含义，不能写入 ROS 2 消息。
  const double norm = std::sqrt(
      static_cast<double>(quaternion[0]) * quaternion[0] +
      static_cast<double>(quaternion[1]) * quaternion[1] +
      static_cast<double>(quaternion[2]) * quaternion[2] +
      static_cast<double>(quaternion[3]) * quaternion[3]);
  if (!std::isfinite(norm) || norm < 1.0e-6) {
    return false;
  }

  destination = sensor_msgs::msg::Imu{};
  destination.header.stamp = stamp;
  destination.header.frame_id = frame_id;

  // 宇树文档规定源数组顺序为 Qw、Qx、Qy、Qz，而 ROS 字段按名称赋值。
  destination.orientation.w = quaternion[0] / norm;
  destination.orientation.x = quaternion[1] / norm;
  destination.orientation.y = quaternion[2] / norm;
  destination.orientation.z = quaternion[3] / norm;

  // HG gyroscope 三轴值直接映射到 ROS angular_velocity，单位沿用 SDK
  // 契约（rad/s）；此处不交换轴、不改变符号。
  destination.angular_velocity.x = gyroscope[0];
  destination.angular_velocity.y = gyroscope[1];
  destination.angular_velocity.z = gyroscope[2];

  // HG accelerometer 三轴值直接映射到 ROS linear_acceleration，单位沿用
  // SDK 契约（m/s^2）；此处不额外扣除重力。
  destination.linear_acceleration.x = accelerometer[0];
  destination.linear_acceleration.y = accelerometer[1];
  destination.linear_acceleration.z = accelerometer[2];

  // sensor_msgs/Imu 中全零协方差表示“协方差未知”。三个估计量均实际存在，
  // 因此不能用首元素 -1（-1 表示该估计量不存在）。标定后的协方差应由
  // 具体部署配置提供，本桥接层不臆造精度。
  destination.orientation_covariance.fill(0.0);
  destination.angular_velocity_covariance.fill(0.0);
  destination.linear_acceleration_covariance.fill(0.0);
  return true;
}

}  // namespace unitree_h2_sensor_bridge

#endif  // UNITREE_H2_SENSOR_BRIDGE_IMU_CONVERSION_HPP_
