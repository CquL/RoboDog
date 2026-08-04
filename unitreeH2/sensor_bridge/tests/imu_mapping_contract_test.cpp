#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include "unitree_h2_sensor_bridge/imu_conversion.hpp"

// 离线映射契约测试：不初始化 DDS、ROS executor 或运动控制，只验证一帧
// HG IMUState 到 sensor_msgs/Imu 的确定性字段映射和坏数据拒绝策略。
namespace {

// 浮点断言使用固定容差，避免归一化计算的微小舍入差异造成误报。
bool Near(double left, double right) {
  return std::abs(left - right) < 1.0e-6;
}

// 统一失败输出，方便 CTest 和镜像验收脚本定位具体契约。
int Fail(const std::string &reason) {
  std::cerr << "UNITREE_H2_IMU_MAPPING_CONTRACT_FAIL reason=" << reason
            << '\n';
  return EXIT_FAILURE;
}

}  // namespace

int main() {
  // 构造非单位四元数，验证转换函数既保持 Qw/Qx/Qy/Qz 顺序又会归一化。
  unitree_hg::msg::dds_::IMUState_ source;
  source.quaternion({2.0F, 0.2F, -0.4F, 0.6F});
  source.gyroscope({1.0F, 2.0F, 3.0F});
  source.accelerometer({4.0F, 5.0F, 6.0F});

  // 接收时间和 frame_id 必须原样进入 ROS 消息头。
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;

  sensor_msgs::msg::Imu output;
  if (!unitree_h2_sensor_bridge::ConvertImu(
          source, stamp, "h2_test_imu", output)) {
    return Fail("valid source rejected");
  }

  // 姿态映射契约：HG 数组 [w,x,y,z] -> ROS 命名字段并除以模长。
  const double norm = std::sqrt(4.0 + 0.04 + 0.16 + 0.36);
  if (!Near(output.orientation.w, 2.0 / norm) ||
      !Near(output.orientation.x, 0.2 / norm) ||
      !Near(output.orientation.y, -0.4 / norm) ||
      !Near(output.orientation.z, 0.6 / norm)) {
    return Fail("QwQxQyQz mapping or normalization mismatch");
  }

  // 角速度和线加速度不做换轴、缩放或符号变换。
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

  // 头部元数据用于 TF 和多传感器时间关联，必须保持调用方传入值。
  if (output.header.frame_id != "h2_test_imu" ||
      output.header.stamp.sec != 123 ||
      output.header.stamp.nanosec != 456) {
    return Fail("header mapping mismatch");
  }

  // 尚未提供标定协方差时保持全零，表示数值存在但协方差未知。
  if (!Near(output.orientation_covariance[0], 0.0) ||
      !Near(output.angular_velocity_covariance[0], 0.0) ||
      !Near(output.linear_acceleration_covariance[0], 0.0)) {
    return Fail("unknown covariance contract mismatch");
  }

  // 非有限传感器值必须整帧拒绝，禁止 NaN 传播到导航算法。
  source.quaternion()[0] = std::numeric_limits<float>::quiet_NaN();
  if (unitree_h2_sensor_bridge::ConvertImu(
          source, stamp, "h2_test_imu", output)) {
    return Fail("non-finite source accepted");
  }

  // 稳定成功标记供 CTest、Docker 离线验收和发布流程匹配。
  std::cout << "UNITREE_H2_IMU_MAPPING_CONTRACT_OK\n";
  return EXIT_SUCCESS;
}
