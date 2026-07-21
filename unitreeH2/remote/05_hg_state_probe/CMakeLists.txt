cmake_minimum_required(VERSION 3.16)
project(h2_hg_state_read_only_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(unitree_sdk2 2.0.0 EXACT CONFIG REQUIRED)

add_executable(h2_hg_state_read_only_probe h2_hg_state_read_only_probe.cpp)
target_link_libraries(h2_hg_state_read_only_probe PRIVATE unitree_sdk2)
target_compile_options(h2_hg_state_read_only_probe PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
