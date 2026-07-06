cmake_minimum_required(VERSION 3.8)
project(x30_pointcloud_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(time_window_cloud_merger src/time_window_cloud_merger.cpp)
target_compile_features(time_window_cloud_merger PUBLIC cxx_std_17)
ament_target_dependencies(time_window_cloud_merger rclcpp sensor_msgs)

install(TARGETS time_window_cloud_merger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
