cmake_minimum_required(VERSION 3.15)
project(robot_hardware)
set(CMAKE_CXX_STANDARD 17)
include(CTest)
option(BUILD_X30_ONLY "Build only the DeepRobotics X30 UDP adapter and test." OFF)
option(BUILD_X30_ROS1_ODOM_TEST "Build the ROS1 /leg_odom closed-loop X30 UDP distance test." OFF)
option(BUILD_X30_ROS1_GAIT_TEST "Build the ROS1 state-verified X30 gait test." OFF)
option(BUILD_X30_PROTOCOL_TEST "Build the offline X30 UDP packet encoding test." OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()



if(NOT BUILD_X30_ONLY)
  # ===================== zsibot 库配置（仅 ARM/aarch64） =====================
  # 1. 强制指定 ARM 架构（仅支持 aarch64）
  #set(ARCH "aarch64")
  #set(ARCH "x86_64")
  # 自动识别系统架构
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      set(ARCH "aarch64")
  else()
      set(ARCH "x86_64")
  endif()

  message(STATUS "系统处理器架构: ${CMAKE_SYSTEM_PROCESSOR}, 映射 ARCH 为: ${ARCH}")

  # 2. 机型匹配实际库名：zsl_1（对应 libmc_sdk_zsl_1_aarch64.so）
  set(ZSIBOT_MODEL "zsl_1")
  # 3. 库路径：匹配你的实际路径 lib/zsibot/aarch64/
  set(ZSIBOT_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/zsibot/${ARCH}")
  message(STATUS "zsibot 库路径: ${ZSIBOT_LIB_PATH}")

  # 4. 查找 ARM 架构的 zsibot 库（库名：mc_sdk_zsl_1_aarch64，匹配实际文件名）
  find_library(
      ZSIBOT_LIB
      NAMES "mc_sdk_${ZSIBOT_MODEL}_${ARCH}"  # 动态拼接库名，避免硬编码错误
      PATHS ${ZSIBOT_LIB_PATH}
      NO_DEFAULT_PATH
      REQUIRED
  )

  # 验证库是否找到
  if(NOT ZSIBOT_LIB)
      message(FATAL_ERROR "未在 ${ZSIBOT_LIB_PATH} 找到 ARM 架构 zsibot 库，请检查 libmc_sdk_${ZSIBOT_MODEL}_${ARCH}.so 是否存在")
  else()
      message(STATUS "找到 ARM 架构 zsibot 库: ${ZSIBOT_LIB}")
  endif()
else()
  message(STATUS "BUILD_X30_ONLY=ON: skipping Unitree and Zsibot dependencies.")
endif()

# ===================== 原有项目逻辑 =====================

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

if(NOT BUILD_X30_ONLY)
  find_package(unitree_sdk2 REQUIRED)
endif()

if(BUILD_X30_ROS1_ODOM_TEST OR BUILD_X30_ROS1_GAIT_TEST)
  find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs std_msgs)
endif()


include_directories(
	include/deep_robotics
)

if(NOT BUILD_X30_ONLY)
  include_directories(
    include/unitree
    include/zsibot
    /usr/local/include/ddscxx
  )
endif()

file(GLOB srcfiles
	src/deep_robotics/*.cpp
	src/robot_hardware_constant.cpp
)

if(NOT BUILD_X30_ONLY)
  file(GLOB extra_srcfiles
    src/unitree/*.cpp
    src/zsibot/*.cpp
  )
  list(APPEND srcfiles ${extra_srcfiles})
endif()

add_library(${PROJECT_NAME} SHARED  ${srcfiles})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 通过库目录 + 库名链接 zsibot，避免把源码树中的绝对路径导出到安装后的 cmake export 中。
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
      unitree_sdk2
  )

  add_executable(robot_test robot_test.cpp)
  target_link_libraries(robot_test PUBLIC ${PROJECT_NAME})
  target_include_directories(robot_test PUBLIC include)

  add_executable(robot_test_unitree_h2 robot_test_unitree_h2.cpp)
  target_link_libraries(robot_test_unitree_h2 PUBLIC ${PROJECT_NAME})
  target_include_directories(robot_test_unitree_h2 PUBLIC include)

  add_executable(robot_test_unitree_h2_live_motion
    robot_test_unitree_h2_live_motion.cpp)
  target_link_libraries(robot_test_unitree_h2_live_motion PUBLIC ${PROJECT_NAME})
  target_include_directories(robot_test_unitree_h2_live_motion PUBLIC include)

  add_executable(unitree_h2_factory_contract_test tests/unitree_h2_factory_contract_test.cpp)
  target_link_libraries(unitree_h2_factory_contract_test PUBLIC ${PROJECT_NAME})
  target_include_directories(unitree_h2_factory_contract_test PUBLIC include)
  add_test(NAME unitree_h2_factory_contract_test COMMAND unitree_h2_factory_contract_test)

  # Compile UnitreeH2 against a fake SDK seam so the HAL-to-H2 API mapping can
  # be verified without DDS, a robot, or any ROS runtime. This does not alter
  # the production UnitreeH2 class or the inline RobotFactory allocation logic.
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
  target_include_directories(unitree_h2_live_motion_plan_test PRIVATE include)
  add_test(NAME unitree_h2_live_motion_plan_test
    COMMAND unitree_h2_live_motion_plan_test)

  add_test(NAME unitree_h2_probe_rejects_conflicting_modes
    COMMAND robot_test_unitree_h2 --read-only --zero-stop)
  set_tests_properties(unitree_h2_probe_rejects_conflicting_modes
    PROPERTIES WILL_FAIL TRUE)

  add_test(NAME unitree_h2_live_motion_rejects_missing_interlocks
    COMMAND robot_test_unitree_h2_live_motion --axis x-positive)
  set_tests_properties(unitree_h2_live_motion_rejects_missing_interlocks
    PROPERTIES WILL_FAIL TRUE)

  add_test(NAME unitree_h2_live_motion_rejects_static_ack_without_one_time_gate
    COMMAND robot_test_unitree_h2_live_motion
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/unitree_h2.yaml
      --axis x-positive
      --expected-fsm 601
      --live-motion
      --acknowledge
        H2_FALL_ARREST_STAND_FOUR_CASTERS_LOCKED_CLEAR_AREA_SECOND_OPERATOR_REMOTE_HIGH_LEVEL_STOP_CONFIRMED
      --authorization-file
        /home/unitree/p2_unitreeH2/build/h2_control_gate_state/invalid)
  set_tests_properties(
    unitree_h2_live_motion_rejects_static_ack_without_one_time_gate
    PROPERTIES WILL_FAIL TRUE)

  install(TARGETS
    robot_test_unitree_h2
    robot_test_unitree_h2_live_motion
    unitree_h2_factory_contract_test
    unitree_h2_direct_api_contract_test
    unitree_h2_live_motion_plan_test
    RUNTIME DESTINATION bin
  )
endif()

#新加
add_executable(robot_test_x30_udp robot_test_x30_udp.cpp)
target_link_libraries(robot_test_x30_udp PUBLIC ${PROJECT_NAME})
target_include_directories(robot_test_x30_udp PUBLIC include)

if(BUILD_X30_ROS1_ODOM_TEST)
  add_executable(robot_test_x30_udp_odom robot_test_x30_udp_odom.cpp)
  target_link_libraries(robot_test_x30_udp_odom PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(robot_test_x30_udp_odom PUBLIC include ${catkin_INCLUDE_DIRS})
endif()

if(BUILD_X30_ROS1_GAIT_TEST)
  add_executable(robot_test_x30_gait robot_test_x30_gait.cpp)
  target_link_libraries(robot_test_x30_gait PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(robot_test_x30_gait PUBLIC include ${catkin_INCLUDE_DIRS})
endif()

if(BUILD_X30_PROTOCOL_TEST)
  add_executable(x30_udp_protocol_test tests/x30_udp_protocol_test.cpp)
  target_include_directories(x30_udp_protocol_test PUBLIC include)
endif()
#
target_include_directories(${PROJECT_NAME}
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


install(EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY  include/
        DESTINATION include/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)


# 安装 zsibot 库到系统 lib 目录，保持原始文件名与构建期链接名一致
if(NOT BUILD_X30_ONLY)
  install(FILES ${ZSIBOT_LIB}
      DESTINATION lib
  )
endif()

# --- Uninstall target ---
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
