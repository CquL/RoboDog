cmake_minimum_required(VERSION 3.16)
project(unitree_h2_sensor_bridge LANGUAGES CXX)

# 使用与 Ubuntu 22.04/ROS 2 Humble 及 SDK2 兼容的 C++17；禁止编译器扩展，
# 使本地构建和 Docker/PC2 构建保持一致。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# 固定 SDK2 2.0.0，避免不同机器人上的 API/IDL ABI 漂移。
find_package(unitree_sdk2 2.0.0 EXACT CONFIG REQUIRED)

# 生产节点：只读订阅 H2 HG DDS IMU，并发布统一 ROS 2 Imu 话题。
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

# 离线契约测试：复用同一个转换头，不连接实机、不创建 DDS 写通道。
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

# 将字段映射测试注册进 CTest，镜像构建阶段可在无机器人环境中执行。
enable_testing()
add_test(
  NAME unitree_h2_imu_mapping_contract
  COMMAND unitree_h2_imu_mapping_contract_test
)

# 安装节点、契约测试和默认参数；运行镜像只需这些编译产物和配置，
# 不依赖工作区源码。
install(
  TARGETS
    unitree_h2_sensor_bridge
    unitree_h2_imu_mapping_contract_test
  RUNTIME DESTINATION bin
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
