cmake_minimum_required(VERSION 3.0.2)
project(x30_sensor_forwarder_workspace)

# 独立 Catkin 顶层文件固定使用原厂 Noetic。构建脚本复制此普通文件，
# 不跟随原厂创建且在迁移工作区内失效的相对软链接。
set(CATKIN_TOPLEVEL TRUE)
set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
find_package(
  catkin
  REQUIRED
  NO_POLICY_SCOPE
  PATHS "/opt/ros/noetic/share/catkin/cmake"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

catkin_workspace()
