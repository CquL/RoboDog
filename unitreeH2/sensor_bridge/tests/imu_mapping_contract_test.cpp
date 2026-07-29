#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include "unitree_h2_sensor_bridge/imu_conversion.hpp"

namespace {

bool Near(double left, double right) {
  return std::abs(left - right) < 1.0e-6;
}

int Fail(const std::string &reason) {
  std::cerr << "UNITREE_H2_IMU_MAPPING_CONTRACT_FAIL reason=" << reason
            << '\n';
  return EXIT_FAILURE;
}

}  // namespace

int main() {
  unitree_hg::msg::dds_::IMUState_ source;
  source.quaternion({2.0F, 0.2F, -0.4F, 0.6F});
  source.gyroscope({1.0F, 2.0F, 3.0F});
  source.accelerometer({4.0F, 5.0F, 6.0F});

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;

  sensor_msgs::msg::Imu output;
  if (!unitree_h2_sensor_bridge::ConvertImu(
          source, stamp, "h2_test_imu", output)) {
    return Fail("valid source rejected");
  }

  const double norm = std::sqrt(4.0 + 0.04 + 0.16 + 0.36);
  if (!Near(output.orientation.w, 2.0 / norm) ||
      !Near(output.orientation.x, 0.2 / norm) ||
      !Near(output.orientation.y, -0.4 / norm) ||
      !Near(output.orientation.z, 0.6 / norm)) {
    return Fail("QwQxQyQz mapping or normalization mismatch");
  }
  if (!Near(output.angular_velocity.x, 1.0) ||
      !Near(output.angular_velocity.y, 2.0) ||
      !Near(output.angular_velocity.z, 3.0)) {
    return Fail("gyroscope mapping mismatch");
  }
  if (!Near(output.linear_acceleration.x, 4.0) ||
      !Near(output.linear_acceleration.y, 5.0) ||
      !Near(output.linear_acceleration.z, 6.0)) {
    return Fail("accelerometer mapping mismatch");
  }
  if (output.header.frame_id != "h2_test_imu" ||
      output.header.stamp.sec != 123 ||
      output.header.stamp.nanosec != 456) {
    return Fail("header mapping mismatch");
  }
  if (!Near(output.orientation_covariance[0], 0.0) ||
      !Near(output.angular_velocity_covariance[0], 0.0) ||
      !Near(output.linear_acceleration_covariance[0], 0.0)) {
    return Fail("unknown covariance contract mismatch");
  }

  source.quaternion()[0] = std::numeric_limits<float>::quiet_NaN();
  if (unitree_h2_sensor_bridge::ConvertImu(
          source, stamp, "h2_test_imu", output)) {
    return Fail("non-finite source accepted");
  }

  std::cout << "UNITREE_H2_IMU_MAPPING_CONTRACT_OK\n";
  return EXIT_SUCCESS;
}
