# X30 原厂材料归档

本目录不参与当前 X30 生产 Docker、105 传感器转发器、106 运行包或
`robot_hardware` 的构建与部署。

这里保存的是原厂 ROS1 程序、配置、共享库、抓取证据和历史离线复现材料，
用于协议核对、问题追溯和厂商接口验证。不要把本目录复制到新机器人，也不要
删除 105 主机上对应的在用原厂组件。

当前生产链路只需要：

```text
105: x30_sensor_forwarder_105
106: x30_livox_ros2:jezetek + x30_livox_ros2_transfer
X30 HAL: robot_hardware_x30
```

归档边界：

- `binaries/`、`message_transformer_cpp/`、X30 的 105/106 launch 和
  `plane_seg_bundle/`：保留为原厂协议证据。
- `plane_seg_offline/`：旧版楼梯算法复现材料，已经退出生产镜像；不再继续
  该路线时可另行备份后删除。
- 非 X30 平台 launch 和旧 `gridmap_receiver`：可另行归档。

任何清理只针对 Windows 工作区归档，不得据此停止或删除机器人原系统进程。
