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

add_library(${PROJECT_NAME} SHARED ${srcfiles})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

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

  add_executable(robot_test_unitree_h2_motion_mode
    robot_test_unitree_h2_motion_mode.cpp)
  target_link_libraries(robot_test_unitree_h2_motion_mode PUBLIC ${PROJECT_NAME})
  target_include_directories(robot_test_unitree_h2_motion_mode PUBLIC include)

  add_executable(unitree_h2_factory_contract_test
    tests/unitree_h2_factory_contract_test.cpp)
  target_link_libraries(unitree_h2_factory_contract_test PUBLIC ${PROJECT_NAME})
  target_include_directories(unitree_h2_factory_contract_test PUBLIC include)
  add_test(NAME unitree_h2_factory_contract_test
    COMMAND unitree_h2_factory_contract_test)

  # Compile UnitreeH2 against a fake SDK seam so the HAL-to-H2 API mapping can
  # be verified without DDS, a robot, or any ROS runtime.
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
    robot_test_unitree_h2_motion_mode
    unitree_h2_factory_contract_test
    unitree_h2_direct_api_contract_test
    unitree_h2_live_motion_plan_test
    RUNTIME DESTINATION bin
  )
endif()

# X30 targets.
add_executable(robot_test_x30_udp robot_test_x30_udp.cpp)
target_link_libraries(robot_test_x30_udp PUBLIC ${PROJECT_NAME})
target_include_directories(robot_test_x30_udp PUBLIC include)

if(BUILD_X30_ROS1_ODOM_TEST)
  add_executable(robot_test_x30_udp_odom robot_test_x30_udp_odom.cpp)
  target_link_libraries(robot_test_x30_udp_odom
    PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(robot_test_x30_udp_odom
    PUBLIC include ${catkin_INCLUDE_DIRS})
endif()

if(BUILD_X30_ROS1_GAIT_TEST)
  add_executable(robot_test_x30_gait robot_test_x30_gait.cpp)
  target_link_libraries(robot_test_x30_gait
    PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(robot_test_x30_gait
    PUBLIC include ${catkin_INCLUDE_DIRS})
endif()

if(BUILD_X30_PROTOCOL_TEST)
  add_executable(x30_udp_protocol_test tests/x30_udp_protocol_test.cpp)
  target_include_directories(x30_udp_protocol_test PUBLIC include)
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
