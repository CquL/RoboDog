# x30_livox_tools

这是我们自己整理的 X30 ROS2 工具包，用来放和 X30 四雷达启动、点云融合相关的代码。

包内主要文件：

```text
launch/x30_mid360_launch.py
  启动 livox_ros_driver2，并加载 X30 四个 MID360 的配置文件。

launch/cloud_merger_launch.py
  启动点云融合节点，可以配置输入点云、输出点云、融合窗口等参数。

src/time_window_cloud_merger.cpp
  订阅 /livox/lidar，把短时间窗口内的点云合并到一起，
  然后发布 /x30/points_merged。
```

`livox_ros_driver2` 仍然保持为独立的第三方雷达驱动包。这个包只放我们针对 X30 写的启动文件和点云处理代码，避免修改第三方源码。
