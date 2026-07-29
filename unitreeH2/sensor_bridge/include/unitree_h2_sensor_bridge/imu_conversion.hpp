#ifndef UNITREE_H2_SENSOR_BRIDGE_IMU_CONVERSION_HPP_
#define UNITREE_H2_SENSOR_BRIDGE_IMU_CONVERSION_HPP_

#include <cmath>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

namespace unitree_h2_sensor_bridge {

using VendorImuState = unitree_hg::msg::dds_::IMUState_;

inline bool ConvertImu(
    const VendorImuState &source,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    sensor_msgs::msg::Imu &destination) {
  const auto &quaternion = source.quaternion();
  const auto &gyroscope = source.gyroscope();
  const auto &accelerometer = source.accelerometer();

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

  // Unitree documents the source order as Qw, Qx, Qy, Qz.
  destination.orientation.w = quaternion[0] / norm;
  destination.orientation.x = quaternion[1] / norm;
  destination.orientation.y = quaternion[2] / norm;
  destination.orientation.z = quaternion[3] / norm;

  destination.angular_velocity.x = gyroscope[0];
  destination.angular_velocity.y = gyroscope[1];
  destination.angular_velocity.z = gyroscope[2];

  destination.linear_acceleration.x = accelerometer[0];
  destination.linear_acceleration.y = accelerometer[1];
  destination.linear_acceleration.z = accelerometer[2];

  // Zero covariance means "unknown covariance" in sensor_msgs/Imu.  The
  // orientation, angular velocity and acceleration estimates are present, so
  // -1 must not be used.  Calibrated covariance is a deployment input.
  destination.orientation_covariance.fill(0.0);
  destination.angular_velocity_covariance.fill(0.0);
  destination.linear_acceleration_covariance.fill(0.0);
  return true;
}

}  // namespace unitree_h2_sensor_bridge

#endif  // UNITREE_H2_SENSOR_BRIDGE_IMU_CONVERSION_HPP_
