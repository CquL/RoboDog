cmake_minimum_required(VERSION 3.8)
project(x30_sensor_receiver)

# 生产接收端只使用标准 ROS2 消息和 POSIX socket，
# 不依赖 Livox/IMU SDK 或机器人控制库。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(x30_sensor_receiver_node
  src/x30_sensor_receiver_node.cpp
)
target_compile_features(x30_sensor_receiver_node PUBLIC cxx_std_17)
target_include_directories(x30_sensor_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(x30_sensor_receiver_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS x30_sensor_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 字节级读取测试在不依赖实时 ROS 图或 TCP 对端的情况下，
  # 校验 version 1 帧合同的 106 侧实现。
  enable_testing()
  add_executable(x30_sensor_wire_protocol_test
    test/wire_protocol_test.cpp
  )
  target_compile_features(x30_sensor_wire_protocol_test PUBLIC cxx_std_17)
  target_include_directories(x30_sensor_wire_protocol_test PRIVATE include)
  add_test(
    NAME x30_sensor_wire_protocol_test
    COMMAND x30_sensor_wire_protocol_test
  )
endif()

ament_package()
