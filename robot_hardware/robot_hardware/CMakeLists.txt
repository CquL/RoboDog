cmake_minimum_required(VERSION 3.15)
project(robot_hardware)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()



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

# ===================== 原有项目逻辑 =====================

find_package(yaml-cpp REQUIRED)
find_package(unitree_sdk2 REQUIRED)


include_directories(
	include/deep_robotics
	include/unitree
	include/zsibot
  /usr/local/include/ddscxx
)

file(GLOB srcfiles 
	src/deep_robotics/*.cpp 
	src/unitree/*.cpp 
	src/zsibot/*.cpp
	src/robot_hardware_constant.cpp
)

add_library(${PROJECT_NAME} SHARED  ${srcfiles})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 通过库目录 + 库名链接 zsibot，避免把源码树中的绝对路径导出到安装后的 cmake export 中。
target_link_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${ZSIBOT_LIB_PATH}>
    $<INSTALL_INTERFACE:lib>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    yaml-cpp
    pthread
    mc_sdk_${ZSIBOT_MODEL}_${ARCH}
    ddsc
    ddscxx
    Threads::Threads
    unitree_sdk2
)

add_executable(robot_test robot_test.cpp)
target_link_libraries(robot_test PUBLIC ${PROJECT_NAME})
target_include_directories(robot_test PUBLIC include)

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
install(FILES ${ZSIBOT_LIB}
    DESTINATION lib
)

# --- Uninstall target ---
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
