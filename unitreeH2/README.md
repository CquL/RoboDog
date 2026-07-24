# Unitree H2/H2 EDU 资料与接入工作区

> 本目录与代码统一采用宇树官方产品名 **H2 / H2 EDU**。目录名使用
> `unitreeH2`，代码、类名、配置键和 ROS 包使用 `unitree_h2` / `UnitreeH2`。

## 目录用途

```text
official_docs/   宇树 H2 在线文档的本地快照与来源清单
downloads/       固定到 commit 的官方 GitHub ZIP 归档
vendor/          上述归档的可读源码快照，保留各仓库 LICENSE
audit/           本轮官网与仓库核验的工作材料
docker/          H2 EDU 独立镜像构建与运行边界
tools/           可重复执行的资料抓取脚本
remote/          PC2 审计脚本、回传日志与远端目录合同
```

## 首轮资料抓取

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\fetch_official_sources.ps1
powershell -ExecutionPolicy Bypass -File .\tools\fetch_official_h2_docs.ps1
```

每个下载项都记录来源 URL、完整 commit、字节数、SHA256 和抓取时间。官方在线文档未声明开源再分发许可，只用于本地研发查阅；随 H2 EDU 交付的私有手册、固件和恢复镜像不得默认提交或外发。

输入与算法桥接边界见 `H2_INPUT_AND_BRIDGE_CONTRACT.md`，直接 HAL 控制、SDK2
内部 DDS 与 Docker 解耦边界见
`../docs/Unitree_H2_直接HAL控制与Docker解耦架构记录_20260716.md`。源码总清单是
`OFFICIAL_SOURCE_MANIFEST.json`，网页快照清单是
`official_docs/H2_OFFICIAL_DOCS_MANIFEST.json`。

## 控制出口约束

- 上层算法只依赖 `RobotHardwareInterface`：`initRobotHardware()`、
  `writeRobotVelocityCommand()` 和 `writeActionCommand()`。
- 当前源码只保留一个 H2 实机测试入口 `robot_test_unitree_h2`；已移除长期
  绕过抽象层的 vendor 运动测试入口。安全只读配置与显式实机配置分别是
  `config/unitree_h2.yaml` 和 `config/unitree_h2_live.yaml`。
- H2 控制不通过上层 ROS 2 控制 topic；`UnitreeH2` 直接调用宇树
  `LocoClient`。SDK2 内部使用 DDS 与 PC1 `sport` 服务交互，这是厂商传输实现，
  不是上层 ROS 2 耦合。
- 要实现字面上的 C++ 对象直接调用，最终控制器与 HAL 对象必须在同一
  进程；跨进程时必须定义项目自有的机器人无关 IPC 合同。
- 当前映射已完成 06C getter、06D 零速度/`StopMove()` 和一次 06E 非零
  `SetVelocity()` 的实机 RPC 验证。可以确认高层速度 HAL 链路已打通；物理轴方向、
  连续导航控制和状态切换动作仍未完成标定/验收，不能把这一结果扩大成“完整 H2
  自主控制已完成”。

## 当前安全边界

- 先确认实物/采购型号是 H2 EDU；普通 H2 产品页未标注支持二次开发。
- 当前已完成 PC2 DDS 图、候选输入、SDK2/ABI 审计、Stage 05 r2 的
  15 秒 HG native 纯订阅，以及 Stage 06C/06D/06E 的分级高层 HAL 实机验证。
- Stage 05 r1 因动态库实际位于 `lib/x86_64/` 而被过窄的路径门禁拒绝，
  探针未运行； r2 锁定该目录哈希后通过。
- r2 从 `rt/lowstate` 和 `rt/secondary_imu` 各收到约 1,048 Hz，从
  `rt/lf/lowstate`、`rt/lf/secondary_imu`、`rt/lf/bmsstate` 和
  `rt/lf/mainboardstate` 各收到约 20.158 Hz；`rt/lowstate_raw` 无样本。
- PC2 缺少 `SportModeState_.hpp`，Stage 05 未订阅该数据。
- PC2 没有 H2 Loco 头且 archive 与本地快照不同；不得复制本地头与 PC2 旧 archive
  混编。当前实机测试使用由固定 SDK2 快照构建、携带私有依赖的 native bundle。
- Stage 06A Docker 离线自包含审计已通过：`--network none`、不挂载宿主 SDK2、
  2/2 CTest 通过，镜像内 Factory 与 direct API fake-SDK 合同测试通过；没有初始化实机
  `LocoClient` 或发送 DDS RPC。
- Stage 06B 生产型状态源、导航传感器驱动和正式 `h2_runtime` 仍未完成；PC2 当前未安装
  Docker，本轮未安装。
- 2026-07-22 已新增并离线验证 `unitree_h2:amd64-runtime-candidate`。它把当前统一 HAL
  和 Stage 05 的 HG 状态/IMU 只读订阅器放入同一 Ubuntu 22.04 + ROS 2 Humble 镜像，
  但尚未在 PC2 容器内实机订阅；订阅器仍是一次性探针，不等于 Stage 06B 生产状态源。
- 当前实机确认“常规运控 2”对应 FSM 703 `PhaseWalk`；现有速度安全门仍锁定已经完成
  前向短脉冲验证的 FSM 601，不自动切换或接受 703。
- 不运行官方低层 ankle swing 示例，不切换站立/阻尼/FSM；非零速度仅允许通过分级
  native bundle、单轴短脉冲和现场人工门禁执行。
- 软件 `StopMove()` 不是物理急停；实机阶段必须保留遥控器、硬件急停和防跌倒安全措施。

PC2 项目文件统一放在 `/home/unitree/p2_unitreeH2/`；详细目录约定见
`remote/README.md`。

## 公开资料缺口

截至 2026-07-15，官方公开下载中心未找到 H2/H2 EDU 用户手册、数据手册或完整二次开发 PDF。需要向交付方索取：

- H2 EDU 用户/运维/安全手册与遥控器按键表；
- 固件与 SDK2 兼容矩阵；
- DDS topic/IDL、网络拓扑、IP/端口与计算单元说明；
- 双目相机流接口、内外参、时间戳与坐标系；
- IMU 标定、关节零位/方向/限位、急停优先级；
- 系统恢复镜像以及 Docker、内核、驱动和 CUDA 限制。
