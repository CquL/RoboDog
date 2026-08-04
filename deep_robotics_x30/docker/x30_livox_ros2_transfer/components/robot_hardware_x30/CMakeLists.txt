cmake_minimum_required(VERSION 3.15)
project(robot_hardware)
set(CMAKE_CXX_STANDARD 17)
include(CTest)

# X30-only 模式用于可迁移的生产/Docker 构建，并排除其他机器人的厂家库。
# ROS1 反馈与数据包契约测试保留为独立选项，
# 使部署和离线验证可以分别选择。
option(BUILD_X30_ONLY "Build only the DeepRobotics X30 UDP adapter and test." OFF)
option(BUILD_X30_ROS1_TEST "Enable ROS1 state feedback in the unified X30 test." OFF)
option(BUILD_X30_PROTOCOL_TEST "Build the offline X30 UDP packet encoding test." OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

if(NOT BUILD_X30_ONLY)
  # ===================== zsibot 库配置 =====================
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH "aarch64")
  else()
    set(ARCH "x86_64")
  endif()

  message(STATUS "系统处理器架构: ${CMAKE_SYSTEM_PROCESSOR}, 映射 ARCH 为: ${ARCH}")

  set(ZSIBOT_MODEL "zsl_1")
  set(ZSIBOT_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/zsibot/${ARCH}")
  message(STATUS "zsibot 库路径: ${ZSIBOT_LIB_PATH}")

  find_library(
    ZSIBOT_LIB
    NAMES "mc_sdk_${ZSIBOT_MODEL}_${ARCH}"
    PATHS ${ZSIBOT_LIB_PATH}
    NO_DEFAULT_PATH
    REQUIRED
  )

  if(NOT ZSIBOT_LIB)
    message(FATAL_ERROR
      "未在 ${ZSIBOT_LIB_PATH} 找到 libmc_sdk_${ZSIBOT_MODEL}_${ARCH}.so")
  else()
    message(STATUS "找到 zsibot 库: ${ZSIBOT_LIB}")
  endif()
else()
  message(STATUS "BUILD_X30_ONLY=ON: skipping Unitree and Zsibot dependencies.")
endif()

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

if(NOT BUILD_X30_ONLY)
  # H2 的 LocoClient、ChannelFactory 和 CycloneDDS 依赖由 unitree_sdk2 提供。
  # 这里只解析构建依赖，不会启动 DDS 或连接机器人。
  find_package(unitree_sdk2 REQUIRED)
endif()

# ROS1 仅用于现场测试观测器；X30 HAL 本身仍是独立于 middleware 的 UDP 库。
if(BUILD_X30_ROS1_TEST)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    std_msgs
  )
endif()

include_directories(
  include/deep_robotics
)

if(NOT BUILD_X30_ONLY)
  include_directories(
    # include/unitree 含 UnitreeH2 声明；ddscxx 是 SDK2 的 DDS C++ 头文件。
    include/unitree
    include/zsibot
    /usr/local/include/ddscxx
  )
endif()

# 通用库始终包含 X30 适配器和共享 action 字符串；
# 仅完整多机器人构建才追加其他厂家源码。
file(GLOB srcfiles
  src/deep_robotics/*.cpp
  src/robot_hardware_constant.cpp
)

if(NOT BUILD_X30_ONLY)
  # 完整 HAL 把 src/unitree/unitree_h2.cpp 与其他厂商适配器一起编入
  # librobot_hardware，上层只需链接公共库。
  file(GLOB extra_srcfiles
    src/unitree/*.cpp
    src/zsibot/*.cpp
  )
  list(APPEND srcfiles ${extra_srcfiles})
endif()

add_library(${PROJECT_NAME} SHARED ${srcfiles})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

if(BUILD_X30_ONLY)
  # 向 RobotFactory 等依赖方公开编译期 RobotFactory 边界，
  # 避免下游引用不可用的机器人类。
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC ROBOT_HARDWARE_X30_ONLY=1)
  # 基础导航镜像已包含另一份 /usr/local/lib/librobot_hardware.so。
  # 为 X30-only HAL 设置独立 SONAME，防止动态加载器静默绑定到该 ABI。
  set_target_properties(${PROJECT_NAME}
    PROPERTIES OUTPUT_NAME robot_hardware_x30)
endif()

if(NOT BUILD_X30_ONLY)
  target_link_directories(${PROJECT_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${ZSIBOT_LIB_PATH}>
      $<INSTALL_INTERFACE:lib>
  )
endif()

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    yaml-cpp
    pthread
    Threads::Threads
)

if(NOT BUILD_X30_ONLY)
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      mc_sdk_${ZSIBOT_MODEL}_${ARCH}
      ddsc
      ddscxx
      # UnitreeH2 高层运动 RPC 实现。
      unitree_sdk2
  )

  add_executable(robot_test robot_test.cpp)
  target_link_libraries(robot_test PUBLIC ${PROJECT_NAME})
  target_include_directories(robot_test PUBLIC include)

  add_executable(robot_test_unitree_h2 robot_test_unitree_h2.cpp)
  # 实机 CLI 链接公共 HAL，验证与正式上层相同的工厂和抽象接口路径。
  target_link_libraries(robot_test_unitree_h2 PUBLIC ${PROJECT_NAME})
  target_include_directories(robot_test_unitree_h2 PUBLIC include)

  add_executable(unitree_h2_factory_contract_test
    tests/unitree_h2_factory_contract_test.cpp)
  # 离线验证 unitree_h2 工厂选择及前置拒绝契约，不初始化 SDK/DDS。
  target_link_libraries(unitree_h2_factory_contract_test PUBLIC ${PROJECT_NAME})
  target_include_directories(unitree_h2_factory_contract_test PUBLIC include)
  add_test(NAME unitree_h2_factory_contract_test
    COMMAND unitree_h2_factory_contract_test)

  # 优先包含 tests/fakes 中的假 SDK2，在没有 DDS、机器人或 ROS 运行时的
  # 环境中验证 HAL 到 H2 API 的映射、异常归一化和看门狗。
  add_executable(unitree_h2_direct_api_contract_test
    tests/unitree_h2_direct_api_contract_test.cpp
    src/unitree/unitree_h2.cpp
    src/robot_hardware_constant.cpp
  )
  target_include_directories(unitree_h2_direct_api_contract_test BEFORE PRIVATE
    tests/fakes
    include
    include/unitree
  )
  target_link_libraries(unitree_h2_direct_api_contract_test PRIVATE
    yaml-cpp
    Threads::Threads
  )
  add_test(NAME unitree_h2_direct_api_contract_test
    COMMAND unitree_h2_direct_api_contract_test)

  add_executable(unitree_h2_live_motion_plan_test
    tests/unitree_h2_live_motion_plan_test.cpp)
  # 旧版 Stage 06E 单轴计划是纯头文件逻辑，只验证数值和时间包络。
  target_include_directories(unitree_h2_live_motion_plan_test PRIVATE include)
  add_test(NAME unitree_h2_live_motion_plan_test
    COMMAND unitree_h2_live_motion_plan_test)

  # 以下负向契约使用 WILL_FAIL：CLI 正确拒绝请求并返回非零时，CTest 记为通过。
  add_test(NAME unitree_h2_probe_rejects_conflicting_modes
    COMMAND robot_test_unitree_h2 --read-only --zero-stop)
  set_tests_properties(unitree_h2_probe_rejects_conflicting_modes
    PROPERTIES WILL_FAIL TRUE)

  add_test(NAME unitree_h2_velocity_rejects_missing_execute
    COMMAND robot_test_unitree_h2
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/unitree_h2_live.yaml
      --velocity --vx 1.00 --vy 0 --omega 0.70 --duration-ms 1000)
  set_tests_properties(unitree_h2_velocity_rejects_missing_execute
    PROPERTIES WILL_FAIL TRUE)

  add_test(NAME unitree_h2_velocity_rejects_above_project_envelope
    COMMAND robot_test_unitree_h2
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/unitree_h2_live.yaml
      --velocity --vx 1.01 --vy 0 --omega 0 --duration-ms 1000 --execute)
  set_tests_properties(unitree_h2_velocity_rejects_above_project_envelope
    PROPERTIES WILL_FAIL TRUE)

  add_test(NAME unitree_h2_omega_rejects_above_project_envelope
    COMMAND robot_test_unitree_h2
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/unitree_h2_live.yaml
      --velocity --vx 0 --vy 0 --omega 0.71 --duration-ms 1000 --execute)
  set_tests_properties(unitree_h2_omega_rejects_above_project_envelope
    PROPERTIES WILL_FAIL TRUE)

  install(TARGETS
    # 随完整 HAL 安装 H2 CLI 与离线契约二进制，供镜像打包/现场验收。
    robot_test_unitree_h2
    unitree_h2_factory_contract_test
    unitree_h2_direct_api_contract_test
    unitree_h2_live_motion_plan_test
    RUNTIME DESTINATION bin
  )
endif()

# X30 统一现场测试目标链接与上层应用安装版本相同的 HAL 库，
# 避免形成仅供测试使用的独立控制路径。
add_executable(robot_test_x30 robot_test_x30.cpp)
target_link_libraries(robot_test_x30 PUBLIC ${PROJECT_NAME})
target_include_directories(robot_test_x30 PUBLIC include)
set_target_properties(robot_test_x30
  PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")

# 启用反馈的构建只为测试程序增加 ROS1 subscriber；
# 步态和速度命令仍通过 RobotHardwareInterface 发送。
if(BUILD_X30_ROS1_TEST)
  target_compile_definitions(robot_test_x30
    PRIVATE X30_TEST_WITH_ROS1=1)
  target_link_libraries(robot_test_x30
    PUBLIC ${catkin_LIBRARIES})
  target_include_directories(robot_test_x30
    PUBLIC include ${catkin_INCLUDE_DIRS})
endif()

# 数据包与 RobotFactory 测试无需机器人即可运行，
# 在镜像到达目标机前固定字节编码和 X30-only 分配边界。
if(BUILD_X30_PROTOCOL_TEST)
  add_executable(x30_udp_protocol_test tests/x30_udp_protocol_test.cpp)
  target_include_directories(x30_udp_protocol_test PUBLIC include)
  add_test(NAME x30_udp_protocol_test COMMAND x30_udp_protocol_test)

  add_executable(x30_factory_contract_test
    tests/x30_factory_contract_test.cpp)
  target_link_libraries(x30_factory_contract_test PUBLIC ${PROJECT_NAME})
  target_include_directories(x30_factory_contract_test PUBLIC include)
  set_target_properties(x30_factory_contract_test
    PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  add_test(NAME x30_factory_contract_test
    COMMAND x30_factory_contract_test)
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS robot_test_x30
  RUNTIME DESTINATION bin
)

# 仅在配置阶段显式请求时安装可选离线检查；
# 生产构建不强制携带测试可执行文件。
if(BUILD_X30_PROTOCOL_TEST)
  install(TARGETS x30_udp_protocol_test x30_factory_contract_test
    RUNTIME DESTINATION bin
  )
endif()

install(EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(NOT BUILD_X30_ONLY)
  install(FILES ${ZSIBOT_LIB}
    DESTINATION lib
  )
endif()

if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
