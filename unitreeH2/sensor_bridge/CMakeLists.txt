cmake_minimum_required(VERSION 3.16)
project(unitree_h2_sensor_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 2.0.0 EXACT CONFIG REQUIRED)

add_executable(unitree_h2_sensor_bridge
  src/unitree_h2_sensor_bridge.cpp
)
target_include_directories(unitree_h2_sensor_bridge PRIVATE include)
ament_target_dependencies(unitree_h2_sensor_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(unitree_h2_sensor_bridge unitree_sdk2)
target_compile_options(unitree_h2_sensor_bridge PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

add_executable(unitree_h2_imu_mapping_contract_test
  tests/imu_mapping_contract_test.cpp
)
target_include_directories(unitree_h2_imu_mapping_contract_test PRIVATE include)
ament_target_dependencies(unitree_h2_imu_mapping_contract_test sensor_msgs)
target_link_libraries(unitree_h2_imu_mapping_contract_test unitree_sdk2)
target_compile_options(unitree_h2_imu_mapping_contract_test PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

enable_testing()
add_test(
  NAME unitree_h2_imu_mapping_contract
  COMMAND unitree_h2_imu_mapping_contract_test
)

install(
  TARGETS
    unitree_h2_sensor_bridge
    unitree_h2_imu_mapping_contract_test
  RUNTIME DESTINATION bin
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
