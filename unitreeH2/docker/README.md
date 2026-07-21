# H2 EDU HAL-only Docker 方案

## 当前状态

当前 `unitree_h2:amd64-offline` 是 **SDK2 + `robot_hardware` 的 HAL-only
离线构建/验证镜像**，不是最终的 H2 运行镜像。它目前不包含：

- `unitree_h2_sensor_bridge` 或其他传感器接收程序；
- 导航、规划或控制算法；
- 将算法输出直接调用 `RobotHardwareInterface` 的 `h2_runtime` 主程序；
- 自动启动上述组件的 entrypoint。

镜像和 `compose.h2.yaml` 默认都只进入 Bash，并已清除从导航底座继承的
`/ros_entrypoint.sh` 自动入口，不会在 HAL-only 检查时隐式 source ROS 2。离线构建、Factory 合同测试和动态库
链接通过，只能证明 SDK2/HAL 可以在该 amd64 用户空间中构建，不能据此宣称已经完成
实机传感器接入、算法闭环或机器人运动控制。

2026-07-16 的 PC2 只读盘点还确认该机器 **尚未安装 Docker**。因此当前镜像不能直接
在 PC2 启动；部署前必须先单独完成 Docker Engine 的安装、版本核对和离线镜像导入
验证。这不授权修改或停止 PC2 上现有的原厂服务。

## 能否继续使用 `jezetek_navigation_amd64.tar`

可以把它作为 **H2 PC2（Intel x86_64、Ubuntu 22.04、ROS 2 Humble）** 的构建底座。它本身只是基础环境，不包含现成导航算法。先加载：

```bash
docker load -i ../../jezetek_navigation_amd64.tar
```

PC3（Jetson Thor）和 PC4（Jetson Orin NX）是 ARM64，不能直接运行该 amd64 镜像；需要用相同 Dockerfile 思路在 ARM64 上原生构建并验证 SDK2 交付库。

## 离线构建

先在 Windows 工作区执行官方资料抓取，再把整个项目复制到 Linux 构建机：

```powershell
powershell -ExecutionPolicy Bypass -File ..\tools\fetch_official_sources.ps1
```

Linux 构建机执行：

```bash
./unitreeH2/docker/build_h2_amd64.sh
```

规范镜像名为 `unitree_h2:amd64-offline`，容器名为 `unitree-h2-hal`。
早期验证使用过的 `robodog_unitree_h2:amd64-offline` 标签已于 2026-07-16
废弃并删除；这只是镜像标签更名，不改变镜像内容。

2026-07-16 Stage 06A 最终本地镜像/仓库 digest 为
`sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa`，
架构 `amd64`，SDK2 commit 为 `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`。
构建阶段 2/2 CTest 通过；最终只读、断网容器复核输出
`UNITREE_H2_FACTORY_CONTRACT_OK`、`UNITREE_H2_DIRECT_API_CONTRACT_OK` 与
`H2_STAGE06A_IMAGE_OFFLINE_OK`。该 digest 仍只代表 HAL-only 镜像，不代表实机可控。

可重复复核：

```powershell
powershell -ExecutionPolicy Bypass -File `
  unitreeH2\docker\verify_stage06a_image_offline.ps1 `
  -ExpectedImageId sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa
```

成功还会输出 `H2_STAGE06A_HOST_GATE_OK`。

构建使用 Docker BuildKit named contexts，只发送 SDK2 与 `robot_hardware`，不会把根目录 3.5 GB tar 和全部 X30 资产发送进构建上下文；Dockerfile 的 `RUN` 阶段网络为 `none`。首次使用的 BuildKit/Docker 基础组件和基础镜像仍应提前加载或缓存。

由于官方 GitHub ZIP 经 Windows 解压后会把 CycloneDDS 的 `.so.0` 符号链接保存成短文本
文件，Dockerfile 会在 Linux 构建层内恢复 `libddsc.so.0` 和 `libddscxx.so.0` 链接；
`vendor/` 原始快照及其下载哈希不被修改。

`verify_all_adapters_offline.sh` 用仓库的普通完整构建方式在断网容器中同时编译
B2、H2、ZSL-1 和 X30，用于验证 `robot_factory.h` 内联的全部分配分支不会破坏原有
适配器。项目保持原来的无条件 Factory 结构，因此该 H2 镜像构建的是完整
`robot_hardware`，没有新增 H2 专用适配器开关。

当前镜像没有设置 `RMW_IMPLEMENTATION`。SDK2 HAL 直接使用安装在
`/opt/unitree_robotics` 下的 CycloneDDS 库，不需要 ROS 2 RMW。原先写入的
`rmw_cyclonedds_cpp` 已移除，因为当前基础镜像没有安装对应的
`librmw_cyclonedds_cpp.so`。未来如果算法内部需要 ROS 2，必须在最终 runtime 镜像中
明确选择并安装经过验证的 RMW；这与 SDK2 内部 DDS 传输以及最终直接调用 HAL 是不同
边界。

当前镜像内还安装了 `unitree_h2_factory_contract_test` 与
`unitree_h2_direct_api_contract_test`。后者只链接 fake H2 SDK seam，在无 DDS、无机器人、
无 ROS 运行时的条件下核对三个 HAL 接口到 `LocoClient` 方法的准确映射及错误边界。

## 运行边界

- 容器默认只进入 Bash；没有传感器桥、算法或 `h2_runtime`，不自动发现 DDS、不自动
  发控制指令。
- SDK2/CycloneDDS 需要 `network_mode: host` 并绑定连接 H2 的真实有线网卡。
- `network_mode: host` 只是为未来 SDK2 实机 DDS 通信预留；启动当前 Bash 容器不等于
  已经完成实机 DDS 或控制验证。
- 高层 `LocoClient` 不需要照搬 X30 的 `--privileged`、串口映射和 ROS1 停启脚本。
- 默认配置禁用非零速度和状态切换；零速度探针也只能在保护支架、遥控器与硬件急停可用时由人员显式运行。
- `robot_test_unitree_h2 <config> --read-only` 只执行初始化和 `GetFsmId`；只有显式
  `--zero-stop` 才调用零速度与 `StopMove()`。首次系统/DDS 盘点不要使用
  `--zero-stop`。
- 高层 RPC 依赖原厂运动控制服务，不要关闭它。只有未来做 `rt/lowcmd` 低层关节控制时，才按 H2 官方调试模式流程处理命令冲突；本阶段不做低层控制。

## 建议的最终分层

```text
Unitree DDS / SDK2
  -> unitree_h2_sensor_bridge（新包）
  -> 项目统一 Imu / JointState / Odometry / TF / RobotStatus 契约
  -> 通用算法容器
  -> RobotVelocityCommand / typed action
  -> UnitreeH2（robot_hardware）
  -> H2 LocoClient 高层运动服务
```

`x30_livox_ros2_transfer` 只能复用通用 Docker/离线测试经验，以及在 H2 确实安装同型号 Livox 时复用部分标准 `PointCloud2` 处理代码；不能复用 X30 IP、MID360 配置、Yesense `/dev/ttyS0`、ROS1 停启脚本、`/x30/*` 话题或地形 TCP/UDP 协议。
