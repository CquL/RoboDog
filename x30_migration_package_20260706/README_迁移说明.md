# X30 ROS2 Docker 迁移包说明

## 包内文件

```text
jezetek_navigation_amd64.tar
  基础 Docker 镜像包，导入后得到 jezetek:navigation_system_amd64。

x30_livox_ros2_transfer.tar.gz
  我们的 ROS2 接入包，用于构建 x30_livox_ros2:jezetek。

docker_offline_focal_amd64_fix1.tar.gz
  离线 Docker 安装包。新机器狗没有 Docker 时使用。

config/multi_MID360_config.json
  四个 MID360 雷达配置备份。

docs/X30_ROS2_Docker_接入阶段记录.md
  阶段记录和操作手册。

docs/README_X30_LIVOX_ROS2.md
  ROS2 接入包说明。
```

## 新机器狗迁移流程

```bash
mkdir -p /home/ysc/ros2_trans
```

如果机器狗没有 Docker，先安装：

```bash
cd /home/ysc
tar -xzf docker_offline_focal_amd64_fix1.tar.gz
cd docker_offline_focal_amd64_fix1
chmod +x install_docker_offline.sh
bash install_docker_offline.sh
docker --version
```

导入基础镜像：

```bash
cd /home/ysc
docker load -i jezetek_navigation_amd64.tar
docker images | grep jezetek
```

构建我们的 ROS2 接入镜像：

```bash
cd /home/ysc/ros2_trans
tar -xzf x30_livox_ros2_transfer.tar.gz
cd x30_livox_ros2_transfer
chmod +x remote/*.sh
bash remote/00_build_image.sh
```

验证：

```bash
docker images | grep -E "jezetek|x30_livox"
```
