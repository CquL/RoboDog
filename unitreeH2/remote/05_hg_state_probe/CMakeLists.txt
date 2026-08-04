cmake_minimum_required(VERSION 3.16)
project(h2_hg_state_read_only_probe LANGUAGES CXX)

# PC2/Ubuntu 22.04 使用 C++17；禁用编译器扩展以保持离线与实机构建一致。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
# 精确固定交付机 SDK2 2.0.0，避免 HG IDL 或 Channel ABI 漂移。
find_package(unitree_sdk2 2.0.0 EXACT CONFIG REQUIRED)

# 单一可执行文件只创建 ChannelSubscriber；源码安全门禁会在构建前后
# 拒绝 publisher、writer、控制客户端及运动话题符号。
add_executable(h2_hg_state_read_only_probe h2_hg_state_read_only_probe.cpp)
target_link_libraries(h2_hg_state_read_only_probe PRIVATE unitree_sdk2)
target_compile_options(h2_hg_state_read_only_probe PRIVATE
  # 把常见类型/可移植性问题提升为构建日志中的明确告警。
  -Wall
  -Wextra
  -Wpedantic
)
