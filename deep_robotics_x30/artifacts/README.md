# X30 当前交付物

本目录是 X30 上传和同型号机器人迁移的唯一交付物目录。

```text
x30_sensor_forwarder_105.tar.gz
x30_sensor_forwarder_105.tar.gz.sha256
  -> 上传到 192.168.1.105
  -> ROS1 只读传感器转发器

x30_livox_ros2_transfer.tar.gz
x30_livox_ros2_transfer.tar.gz.sha256
  -> 上传到 192.168.1.106
  -> Docker 构建/运行脚本和源码小包，不是 Docker 镜像

robot_hardware_x30_udp_transfer.tar.gz
robot_hardware_x30_udp_transfer.tar.gz.sha256
  -> 仅在 106 宿主机编译带 ROS1 状态反馈的统一控制测试时上传

x30_livox_ros2_jezetek_amd64.tar
x30_livox_ros2_jezetek_amd64.tar.sha256
  -> 上传到 192.168.1.106
  -> 最终生产 Docker 镜像归档
  -> 当前需先从已验证的 106 执行 docker save 后取得
```

更新规则：

```text
覆盖同名文件
重新生成同名 .sha256
校验后上传
不在本目录保存时间戳旧版本
```

统一更新三个小包：

```powershell
cd D:\Desktop\RoboDog

powershell -ExecutionPolicy Bypass -File `
  .\deep_robotics_x30\tools\package_x30_artifacts.ps1
```

Docker 源码或 HAL 变化后，还需在已验证的 106 重建镜像并执行：

```bash
cd /home/ysc/x30_livox_ros2_transfer
bash remote/32_export_x30_image.sh
```

然后把导出的镜像 tar 和 `.sha256` 覆盖下载到本目录。

完整命令见：

```text
../X30_新机器人_Docker部署与控制测试操作手册_20260731.md
```
