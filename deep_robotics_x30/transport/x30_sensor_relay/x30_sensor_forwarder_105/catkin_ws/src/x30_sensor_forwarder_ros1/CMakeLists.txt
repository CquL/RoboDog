cmake_minimum_required(VERSION 3.0.2)
project(x30_sensor_forwarder_ros1)

# 本包只依赖 ROS 消息传输，不链接原厂传感器 SDK 或机器人控制库。
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(x30_sensor_forwarder_node
  src/x30_sensor_forwarder_node.cpp
)
target_compile_features(x30_sensor_forwarder_node PRIVATE cxx_std_14)
target_link_libraries(x30_sensor_forwarder_node
  ${catkin_LIBRARIES}
)

install(TARGETS x30_sensor_forwarder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # 独立字节级测试用于固定 version 1 帧布局，确保与 106 的 ROS2 接收端一致。
  add_executable(x30_sensor_wire_protocol_writer_test
    test/wire_protocol_test.cpp
  )
  target_compile_features(
    x30_sensor_wire_protocol_writer_test PRIVATE cxx_std_14
  )
  target_include_directories(
    x30_sensor_wire_protocol_writer_test PRIVATE include
  )
  add_test(
    NAME x30_sensor_wire_protocol_writer_test
    COMMAND x30_sensor_wire_protocol_writer_test
  )
endif()
