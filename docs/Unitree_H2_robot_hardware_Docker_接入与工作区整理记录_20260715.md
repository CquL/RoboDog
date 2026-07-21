# Unitree H2 robot_hardware、Docker 接入与工作区整理记录

日期：2026-07-15  
阶段：公开资料核验、HAL 首版、离线构建验证、工作区归档完成；实机验证未开始。

## 1. 本轮目标

1. 核实“Unitree H20”的正式型号、版本与官方开发资料。
2. 按现有 `RobotHardwareInterface` 增加独立宇树 H2 适配器。
3. 明确 H2 的 ROS/DDS、输入数据、原厂服务和 Docker 迁移边界。
4. 下载并固定官方 SDK、ROS、模型和在线文档快照。
5. 将云深处 X30 专属资产集中到一个目录，同时保留根目录共享资产。
6. 全程只做离线构建和静态/合同测试，不连接机器人、不发非零速度。

## 2. 核验结论

- 正式产品名是 **Unitree H2 / H2 EDU**；未发现官方 `H20` 型号。
- 官网只把 H2 EDU 标为支持二次开发，因此采购/实物必须先确认 EDU 版本。
- H2 实机开发主线是 SDK2 + CycloneDDS，并有 ROS 2 通信例程；PC2 文档环境为
  x86_64、Ubuntu 22.04、ROS 2 Humble。官方 `unitree_ros` 中的 H2 内容主要用于
  ROS1 模型/仿真，不能作为 H2 实机控制主线。
- 公开 H2 下载中心没有找到完整用户手册、数据手册或二次开发 PDF；已归档在线文档，
  私有交付手册仍需向厂商索取。
- X30 厂商是云深处科技（Deep Robotics），不是优必选。

## 3. 实际完成内容

### 官方资料归档

归档根目录统一使用官方型号命名 `unitreeH2/`，代码命名使用 `unitree_h2`：

```text
unitreeH2/
  official_docs/   23 个中文与 20 个英文页面快照及 SHA-256 清单
  downloads/       固定 commit 的官方 ZIP
  vendor/          解压后的官方源码与 LICENSE
  audit/           本轮官网/仓库核验材料
  tools/           可重复抓取脚本
  docker/          H2 amd64 离线构建方案
```

固定版本：

| 仓库 | commit |
|---|---|
| `unitree_sdk2` | `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` |
| `unitree_sdk2_python` | `e4cd91f051aaa77a70600e3d2bf7f50889db1980` |
| `unitree_ros2` | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |
| `unitree_ros` | `d96d8f63ae17a7108d4f7229c00ef875ba7129c9` |
| `unitree_model` | `b6a8942b0803b6c137e58cef12beb4b03e4a2fa7` |
| `unitree_mujoco` | `ae6a8403e272733e9996ef59990880330496177f` |

`OFFICIAL_SOURCE_MANIFEST.json` 记录 URL、commit、时间、字节数和 ZIP SHA-256；
`H2_OFFICIAL_DOCS_MANIFEST.json` 对每个页面做同类记录。

### H2 HAL

最初误建的 H20 命名目录不合适：官方型号是 H2，且空目录没有进入构建系统。现已改为：

```text
include/unitree/unitree_h2.h
src/unitree/unitree_h2.cpp
config/unitree_h2.yaml
robot_test_unitree_h2.cpp
```

新增 `UnitreeH2 : RobotHardwareInterface`，通过 H2 `LocoClient` 对接高层运动服务。
工厂的完整分配逻辑保留在公共头文件 `include/robot_factory.h`，其中直接增加
`unitree_h2 -> UnitreeH2` 分支。保持原项目结构，不使用厂商适配器 CMake 开关或
`ROBOT_HARDWARE_HAS_*` 条件宏；普通构建直接包含四个现有适配器。

默认安全策略：

- 非零速度和状态切换均关闭；
- 速度有限值检查、保守限幅、持续时间和超时看门狗；
- `stop_move` 始终允许；
- 初始化可读取 FSM；只有对象实际发过控制命令后，析构才尽力执行 `StopMove()`；
- 探针默认或 `--read-only` 不写控制，只有显式 `--zero-stop` 才发送零速度和
  `StopMove()`；任何模式都不包含非零命令。

### 输入与桥接方案

需要新增独立 `unitree_h2_sensor_bridge`，但本轮只冻结接口合同，没有在未知硬件配置下
虚构 ROS 2 驱动。必须接收：LowState、secondary IMU、BMS、odom、FSM/返回码、遥控器/
急停与消息新鲜度；相机和点云按实际交付配置接入。详细合同见
`unitreeH2/H2_INPUT_AND_BRIDGE_CONTRACT.md`。

本阶段不关闭原厂高层运动服务、遥控器或安全输入，也不照搬 X30 的 ROS1 服务停启
脚本。只有未来做 `rt/lowcmd` 关节级控制时才按官方调试模式处理控制器冲突。

### Docker

根目录 `jezetek_navigation_amd64.tar` 对应 amd64 Ubuntu 22.04 / ROS 2 Humble 基础镜像，
可继续作为 H2 PC2 的底座；不能直接用于 PC3/PC4 的 arm64 计算单元。

H2 Docker 构建使用命名上下文，只复制固定版本 SDK2 与 `robot_hardware`，构建阶段
`--network none`，默认命令只是 Bash，不自动发现 DDS 或运行控制。实机 DDS 预计需要
host network 和真实有线网卡，但必须在现场安全审查后才启用。

`x30_livox_ros2_transfer` 不能直接复用：它绑定 MID360、Yesense 串口、X30 话题/IP、
ROS1 停启脚本和 X30 地形 TCP/UDP。可复用的只有 Docker 组织方式、离线测试方式，以及
在 H2 实际安装同类雷达时的通用 `PointCloud2` 算法。

### X30 工作区整理

X30 专属内容已集中到 `deep_robotics_x30/`：文档、手册、Docker、原厂包、传感器、
数据、迁移包、日志、交付物和临时分析材料。移动前先生成 dry-run 清单，实际移动后对
目录核对文件数/字节数、对单文件核对 SHA-256。历史文档和 fixture 中的旧路径作为
实验来源证据保留；新操作从 `deep_robotics_x30/README.md` 进入。

按要求继续保留在根目录：

- `robot_hardware/`
- `jezetek_navigation_amd64.tar`

`robot_hardware.zip` 是 HAL 的旧快照，也保留在根目录；它不是当前源码真值。

## 4. 验证结果

| 验证 | 结果 |
|---|---|
| `robot_hardware` Python 合同测试 | `12 passed` |
| 搬迁后 X30 离线测试 | `45 passed, 13 subtests passed` |
| H2 SDK2 + HAL Docker 离线编译 | 通过，生成 `librobot_hardware.so` 与 H2 探针 |
| H2 Factory 断网合同测试 | `UNITREE_H2_FACTORY_CONTRACT_OK` |
| 普通完整四适配器联合构建 | `ALL_ADAPTERS_INLINE_FACTORY_BUILD_OK` |
| H2 镜像架构 | `linux/amd64` |
| H2 镜像运行时链接 | `ldd` 无缺失库 |
| H2 配置安全门 | 两个 allow 开关均为 `false` |
| 网络/实机运动 | 未执行 |

当时验证标签：`robodog_unitree_h2:amd64-offline`（已于 2026-07-16 更名并废弃）；当前规范标签：`unitree_h2:amd64-offline`，镜像 ID
`sha256:b4e6c8661d1a6b815e3dd454d40ba177d1effdbbc3f352cbfa915ce66619f727`。

## 4.1 用户复核后的更正

用户要求恢复原来的 Factory 可读形式后，已完成以下更正：

- 顶层资料目录从误命名的 H20 形式整体改为 `unitreeH2/`，4,755 个文件、
  2,925,880,915 字节在改名前后核对一致，工作区内旧目录名引用为零；
- `RobotFactory::RobotAllocate()` 的完整实现恢复到
  `include/robot_factory.h`，可直接看到 B2、H2、ZSL-1、X30 四个分配分支；
- 删除本轮曾额外增加的工厂实现源文件、四个厂商适配器 CMake 开关、全部 `ENABLE_*`
  中间变量和 `ROBOT_HARDWARE_HAS_*` 编译宏；
- Factory 恢复原来的无条件 include 和连续 `if / else if` 形式，只增加
  `unitree_h2 -> UnitreeH2`；普通构建直接包含 B2、H2、ZSL-1、X30；
- 保留 H2 接入前已经存在的四个 `BUILD_X30_*` 选项；它们用于 X30 离线构建和测试，
  不是厂商 Factory 开关；
- 删除不安全的 `lie_down -> Damp()` 猜测映射：H2 对旧 `lie_down` 明确返回不支持，
  只有显式 `damp` 才调用 `Damp()`；
- 增加完全断网的 Factory 合同测试，确认 `unitree_h2` 创建的是 `UnitreeH2`、初始化前
  不接受速度命令、未知型号会被拒绝；
- 新测试首次运行暴露出官方 GitHub ZIP 经 Windows 解压后将 CycloneDDS `.so.0`
  符号链接变成短文本文件，症状为 `libddsc.so.0: file too short`。Docker 构建层现会
  恢复链接，随后 CTest、实际程序加载和 `ldd` 均通过；原始下载包与 SHA-256 未修改。

## 5. 下一阶段（必须按顺序）

1. 从供应商补齐 H2 EDU 交付资料并确认固件/SDK/计算单元版本。
2. 在 H2 上只读枚举 DDS topic、类型、频率、网卡和时钟，不发送控制。
3. 实现 `unitree_h2_sensor_bridge`，录制无运动 rosbag，完成离线回放与时间戳/TF 检查。
4. 在保护支架、清场、双人监护、遥控器和物理急停就绪时做 FSM 读取、零速度和
   `StopMove()`；保留安全门关闭。
5. 审查速度坐标、动作前置状态、故障与失联测试后，才可一次只打开一个安全门，做
   极小速度、短持续时间测试。
6. 最后接导航算法闭环；未经上述验证不得开启低层 `rt/lowcmd`。

## 6. 当前明确未完成

- 未连接 H2 实机、未验证实际网卡/IP/DDS 发现；
- 未运行零速度探针，因为即使零命令也属于实机控制操作；
- 未实现传感器桥接包，等待真实交付接口和标定；
- 未获得私有 H2 EDU PDF、固件恢复包或兼容矩阵；
- 未发送非零速度、未站立/坐下/阻尼、未进入低层控制。
