cmake_minimum_required(VERSION 3.8)
project(x30_plane_seg_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io search surface)

set(
  X30_PLANE_SEG_UPSTREAM_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/plane_seg"
  CACHE PATH
  "Path to the checksum-pinned ori-drs/plane_seg source snapshot"
)

if(NOT EXISTS "${X30_PLANE_SEG_UPSTREAM_DIR}/UPSTREAM_VERSION.md")
  set(X30_PLANE_SEG_UPSTREAM_DIR "/opt/src/x30-plane-seg-upstream")
endif()

if(NOT EXISTS "${X30_PLANE_SEG_UPSTREAM_DIR}/UPSTREAM_VERSION.md")
  message(FATAL_ERROR
    "Pinned plane_seg source not found. Set X30_PLANE_SEG_UPSTREAM_DIR or "
    "copy third_party/plane_seg to /opt/src/x30-plane-seg-upstream."
  )
endif()

add_library(x30_plane_seg_core SHARED
  src/grid_map_adapter.cpp
  src/plane_seg_core.cpp
  src/quadrangle_geometry.cpp
  src/quadrangle_postprocessing.cpp
  src/factory_block_fitter.cpp
  src/factory_incremental_plane_estimator.cpp
  src/factory_plane_segmenter.cpp
  ${X30_PLANE_SEG_UPSTREAM_DIR}/src/PlaneFitter.cpp
  ${X30_PLANE_SEG_UPSTREAM_DIR}/src/RectangleFitter.cpp
  ${X30_PLANE_SEG_UPSTREAM_DIR}/src/RobustNormalEstimator.cpp
)

target_compile_features(x30_plane_seg_core PUBLIC cxx_std_17)
target_include_directories(x30_plane_seg_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${X30_PLANE_SEG_UPSTREAM_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(x30_plane_seg_core PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(x30_plane_seg_core
  PUBLIC Eigen3::Eigen
  PRIVATE ${PCL_LIBRARIES}
)

install(
  TARGETS x30_plane_seg_core
  EXPORT export_x30_plane_seg_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/x30_plane_seg_core DESTINATION include)
install(FILES
  README.md
  ${X30_PLANE_SEG_UPSTREAM_DIR}/LICENSE
  ${X30_PLANE_SEG_UPSTREAM_DIR}/UPSTREAM_VERSION.md
  ${X30_PLANE_SEG_UPSTREAM_DIR}/SHA256SUMS
  DESTINATION share/${PROJECT_NAME}
)

set(
  X30_PLANE_SEG_REPLAY_FIXTURE
  "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/x30_plane_seg_replay_v1.x30rpl"
)
set(
  X30_PLANE_SEG_REPLAY_METRICS
  "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/x30_plane_seg_replay_v1.metrics.json"
)
install(FILES
  "${X30_PLANE_SEG_REPLAY_FIXTURE}"
  "${X30_PLANE_SEG_REPLAY_FIXTURE}.sha256"
  "${X30_PLANE_SEG_REPLAY_METRICS}"
  DESTINATION share/${PROJECT_NAME}/test/fixtures
  OPTIONAL
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(x30_plane_seg_core_contract_test
    test/core_contract_test.cpp
  )
  target_compile_features(x30_plane_seg_core_contract_test PUBLIC cxx_std_17)
  target_link_libraries(x30_plane_seg_core_contract_test x30_plane_seg_core)
  add_test(
    NAME x30_plane_seg_core_contract_test
    COMMAND x30_plane_seg_core_contract_test
  )

  add_executable(x30_plane_seg_core_grid_map_adapter_test
    test/grid_map_adapter_test.cpp
  )
  target_compile_features(x30_plane_seg_core_grid_map_adapter_test PUBLIC cxx_std_17)
  target_link_libraries(x30_plane_seg_core_grid_map_adapter_test x30_plane_seg_core)
  add_test(
    NAME x30_plane_seg_core_grid_map_adapter_test
    COMMAND x30_plane_seg_core_grid_map_adapter_test
  )

  add_executable(x30_plane_seg_core_quadrangle_geometry_test
    test/quadrangle_geometry_test.cpp
  )
  target_compile_features(x30_plane_seg_core_quadrangle_geometry_test PUBLIC cxx_std_17)
  target_link_libraries(x30_plane_seg_core_quadrangle_geometry_test x30_plane_seg_core)
  add_test(
    NAME x30_plane_seg_core_quadrangle_geometry_test
    COMMAND x30_plane_seg_core_quadrangle_geometry_test
  )

  add_executable(x30_plane_seg_core_quadrangle_postprocessing_test
    test/quadrangle_postprocessing_test.cpp
  )
  target_compile_features(
    x30_plane_seg_core_quadrangle_postprocessing_test PUBLIC cxx_std_17)
  target_link_libraries(
    x30_plane_seg_core_quadrangle_postprocessing_test x30_plane_seg_core)
  add_test(
    NAME x30_plane_seg_core_quadrangle_postprocessing_test
    COMMAND x30_plane_seg_core_quadrangle_postprocessing_test
  )

  add_executable(x30_plane_seg_core_replay_test
    test/replay_fixture_test.cpp
    test/support/replay_file.cpp
    test/support/sha256.cpp
  )
  target_compile_features(x30_plane_seg_core_replay_test PUBLIC cxx_std_17)
  target_include_directories(x30_plane_seg_core_replay_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(x30_plane_seg_core_replay_test x30_plane_seg_core)
  add_test(
    NAME x30_plane_seg_core_replay_test
    COMMAND x30_plane_seg_core_replay_test
      "${X30_PLANE_SEG_REPLAY_FIXTURE}"
      --expected-output-sha256
      42318c156d930052fb213e69078c66399dd806d673ef01ce6c10302ddbab909e
  )

  add_executable(x30_plane_seg_core_replay_format_test
    test/replay_format_test.cpp
    test/support/replay_file.cpp
    test/support/sha256.cpp
  )
  target_compile_features(x30_plane_seg_core_replay_format_test PUBLIC cxx_std_17)
  target_include_directories(x30_plane_seg_core_replay_format_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  add_test(
    NAME x30_plane_seg_core_replay_format_test
    COMMAND x30_plane_seg_core_replay_format_test "${X30_PLANE_SEG_REPLAY_FIXTURE}"
  )
endif()

ament_export_targets(export_x30_plane_seg_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
