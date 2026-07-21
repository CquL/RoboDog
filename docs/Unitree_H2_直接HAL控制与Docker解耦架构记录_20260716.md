# Unitree H2 直接 HAL 控制与 Docker 解耦架构记录

日期：2026-07-16  
状态：Stage 05 r2 纯订阅、Stage 06A Docker 离线自包含门禁通过；H2 实机运动控制仍未解锁

## 1. 本轮决策

控制出口固定为项目已有的 `RobotHardwareInterface`，上层算法不依赖、不发布
宇树 ROS 2 控制 topic：

```cpp
virtual int32_t initRobotHardware() = 0;
virtual int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) = 0;
virtual int32_t writeActionCommand(std::string action) = 0;
```

目标调用链是：

```text
传感器/原厂输入
  -> H2 输入适配器
  -> 项目统一传感器数据合同
  -> 算法
  -> RobotVelocityCommand / 项目动作常量
  -> RobotHardwareInterface（进程内 C++ 调用）
  -> UnitreeH2
  -> 宇树 SDK2 H2 LocoClient
  -> H2 PC1 原厂 sport 运动服务
```

传感器输入可以在适配层使用 SDK2/DDS、相机流或必要的驱动。如果现有算法内部使用
ROS 2，可将统一的 `Imu` / `Odometry` / `PointCloud2` / TF 作为算法内部输入；这不改变
“最终运动控制只调 HAL”的边界。

## 2. 当前 H2 适配映射

`UnitreeH2` 当前已在代码层完成以下映射：

| 项目接口 | H2 SDK2 映射 | 当前保护 |
|---|---|---|
| `initRobotHardware()` | `ChannelFactory::Init()`、`LocoClient::Init()`、`SetTimeout()`，可选 `GetFsmId()` | 参数检查，默认不允许运动/状态切换 |
| `writeRobotVelocityCommand()` | `SetVelocity(vx, vy, omega, duration)` | NaN/Inf 拒绝、三轴限幅、非零速度联锁、watchdog |
| `writeActionCommand(stop_move)` | `StopMove()` | 允许停止，但不是物理急停 |
| `stand_up` | `StandUp()` | 状态切换默认禁用 |
| `prepare_motion` | `Start()` | 状态切换默认禁用 |
| `damp` / `squat` / `sit` | `Damp()` / `Squat()` / `Sit()` | 状态切换默认禁用 |
| `lie_down` | 无同语义的公开 H2 高层 API | 明确返回不支持，不猜测映射 |

这证明“接口对齐”已存在，但不证明这个 SDK2 快照已与当前 H2 固件完成实机兼容验证。

三个虚函数可以继续作为稳定的最小控制出口。生产运行时还需要就绪/FSM、控制权、最后错误、
命令确认、传感器新鲜度和 deadman 状态；它们可以放在独立的诊断/状态合同中，不需要把
厂商 ROS 2 控制接口重新暴露给上层。

## 3. SDK2 内部 DDS 与 ROS 2 控制耦合的区别

H2 `LocoClient` 是普通 C++ API，但 SDK2 内部使用 CycloneDDS 请求/响应与 PC1 交互：

- 服务名为 `sport`；
- `SetVelocity()` 使用 API ID `7105`；
- `SetFsmId()` 使用 API ID `7101`；
- `Start()`、`StandUp()`、`Damp()` 等高层函数最终是 FSM API。

因此应使用下面的准确表述：

> 上层不依赖、不发布 ROS 2 控制 topic，而是直接调用
> `RobotHardwareInterface`。H2 适配器内部调用宇树官方 SDK2，SDK2 内部的 DDS 是厂商传输实现，
> 不是上层 ROS 2 控制耦合。

不能将这条链说成“完全不经过 DDS”，也不能说成“直接写电机”。本项目当前调用的是
H2 高层运动服务，禁止发布 `rt/lowcmd`。

## 4. “直接调用对象”的进程边界

如果要严格满足算法直接调用三个虚函数，最终控制器和 `RobotHardwareInterface` 对象必须在
同一进程：

```cpp
auto robot = RobotFactory::RobotAllocate(config);
robot->initRobotHardware();

RobotVelocityCommand cmd = algorithmOutput();
robot->writeRobotVelocityCommand(cmd);
```

两个独立进程不能直接共享 C++ 对象。如果未来必须做进程隔离，可在两者之间使用项目自有、
与机器人无关的 Unix Socket/共享内存合同，由 HAL daemon 最终调用三个接口。这仍不是原厂
ROS 2 控制 topic，但不再是字面上的进程内调用。

如果 H2 传感器订阅与 HAL 在同一进程，SDK2 `ChannelFactory` 应由一个统一运行时上下文用同一
网卡和 domain 初始化一次，不能由多个模块使用不同参数重复初始化。

## 5. Docker 可迁移边界

目标 H2 运行镜像应自包含：

- 算法程序和统一数据合同；
- `robot_hardware`、`UnitreeH2` 适配器和准确版本的 SDK2；
- CycloneDDS 运行库、设置、配置和健康检查；
- H2 传感器输入适配器；
- 不依赖宿主 `/opt/unitree_robotics` 或 PC2 ROS 2 workspace 的启动入口。

理想验收方式是：在干净的同架构 PC2 上只需 `docker load`、指定 H2 有线网卡/配置并启动，
不需要安装宿主 SDK2，也不需要 source 宿主 ROS 2 环境。

容器可以隔离用户态环境，但无法消除以下外部合同：

- amd64/ARM64 架构、Linux 内核和 Docker 运行时；
- 物理网卡、IP、路由、MTU、组播、防火墙和 DDS domain；
- H2 固件、PC1 `sport` 服务与 SDK2 API/IDL 兼容性；
- 相机/雷达/串口/USB 的实际设备、驱动、标定和时钟；
- 遥控器、硬件急停、机器人当前模式和现场安全条件。

对不同厂商/架构，最现实的交付是“相同算法基础层 + 轻量机器人运行层”，不应把“上层源码不改”
误解成“所有架构和厂商必须使用字节级完全相同的镜像”。

当前实际状态与目标之间还有两个明确差距：

- 当前 H2 Docker 配方只包含 SDK2、HAL 和离线验证程序，默认只进入 Bash；还没有
  `unitree_h2_sensor_bridge`、算法程序或正式 `h2_runtime`。
- 2026-07-16 PC2 盘点结果为 `docker: not installed`；当前 Stage 05 是在 PC2 宿主原生
  SDK2 环境中执行，不是 Docker 实机验收。

## 6. Stage 05 实际记录

### 6.1 r1：门禁误判后安全退出

日志：`unitreeH2/remote/h2_pc2_hg_state_probe_20260716.log`  
SHA256：`E53CBE877BC0566FF6F0DBFDE9BA0FBDD9BEC0E656DEABAFA033630823A42B89`

- PC2 原生 CMake 配置和编译都成功；
- 源码、SDK2/IDL、CycloneDDS 库和网络配置哈希都通过；
- `ldd` 实际解析到 `/opt/unitree_robotics/lib/x86_64/libddsc*.so.0`；
- r1 门禁只允许了无 `x86_64/` 子目录的路径，因此输出
  `BINARY_DDS_LIBRARY_PATH_MISMATCH`、`STAGE05_RUN_RC=14`；
- 探针没有运行，没有建立实际 DDS 订阅。

这是检查脚本的允许路径过窄，不是 ABI 或 DDS 库解析失败。安全性符合预期：门禁不满足时在运行前退出。

### 6.2 r2：15 秒 native 纯订阅成功

日志：`unitreeH2/remote/h2_pc2_hg_state_probe_20260716_r2.log`  
SHA256：`CFC69F06ED45C861CE1959FD2441F3A9603144216C76CFB0F3D6F0225ABAED0A`

r2 将已哈希锁定的 `lib/x86_64/` 库目录纳入允许集合。二进制审计只发现七个允许的
接收 topic，未发现 publisher/client/写路径符号，然后才启动 15 秒订阅：

| DDS channel | 样本数 | 测得频率 | 备注 |
|---|---:|---:|---|
| `rt/lowstate_raw` | 0 | 0 Hz | 该候选本轮无数据 |
| `rt/lowstate` | 15,718 | 1,048.231 Hz | CRC 15,718 成功，0 失败 |
| `rt/lf/lowstate` | 302 | 20.158 Hz | CRC 302 成功，0 失败 |
| `rt/secondary_imu` | 15,720 | 1,048.254 Hz | 官方示例所称 torso IMU，语义仍需交付资料确认 |
| `rt/lf/secondary_imu` | 302 | 20.158 Hz | 低频镜像 |
| `rt/lf/bmsstate` | 302 | 20.158 Hz | SOC/SOH/电流/电压/温度可解码 |
| `rt/lf/mainboardstate` | 302 | 20.158 Hz | 风扇/温度/状态字段可解码 |

运行结束标记为：

```text
H2_HG_SUBSCRIBE_ONLY_PROBE_OK
READ_ONLY_PC2_HG_STATE_PROBE_OK
STAGE05_RUN_RC=0
```

超时后进程已清理，未留下探针进程。`SportModeState` 因 PC2 HG 头文件缺失仍未订阅。

## 7. r2 能证明与不能证明的内容

r2 证明：

- PC2 自带的 HG IDL、SDK2 archive 和 CycloneDDS 库可原生编译并做 native 纯订阅；
- `rt/lowstate`、secondary IMU、BMS 和 MainBoard 在当前 H2 上有新鲜样本；
- H2 输入适配器可以从上述四类数据开始实现，不需要依赖 PC2 ROS 2 消息包。

r2 不证明：

- 当前 Docker 镜像已具备完整传感器桥、算法和控制 runtime；
- 本地固定 SDK2 中的 H2 `LocoClient` 已与当前 H2 固件/原厂 `sport` 服务兼容；
- `initRobotHardware()`、`GetFsmId()`、`SetVelocity()` 或任何动作 API 已在实机调用；
- H2 已可由项目 HAL 实机驱动。

截止本记录，不得对外声称“H2 已经可控”。

## 8. Stage 06A 结果与下一门禁

Stage 06A 已在本地完成：`unitree_h2:amd64-offline` 使用 `--pull=false` 和构建阶段
`--network=none` 重建，镜像/仓库 digest 为
`sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa`。
镜像构建阶段 2/2 CTest 通过；最终又在 `--network none`、只读根文件系统和 drop-all
capabilities 下输出 `UNITREE_H2_FACTORY_CONTRACT_OK`、
`UNITREE_H2_DIRECT_API_CONTRACT_OK` 与 `H2_STAGE06A_IMAGE_OFFLINE_OK`。`ldd` 无缺失，
HAL 无 `rcl/rmw/ros` 动态库依赖，错误 RMW 环境变量和继承的 ROS entrypoint 均已清除。

完整产物哈希、测试覆盖与实际修改见
`docs/Unitree_H2_HAL直接接口与Stage06A离线验收记录_20260716.md`。

下一步是 **Stage 06B：镜像内生产型状态源纯订阅**。PC2 当前未安装 Docker，本轮没有
安装；开始前需要明确批准 Docker Engine 部署，或改用另一台已安装 Docker 的同网段 amd64
主机。Stage 06B 仍不调用 H2 Loco RPC。

首次 `GetFsmId()` 作为后续独立 **Stage 06C**；它虽然不改变运动状态，但会通过 SDK2
发送 DDS RPC 请求，不能与纯订阅门禁混在一轮。

零速度/`StopMove()` 和受限非零运动属于更后续的实机控制门禁，必须在保护支架、遥控器、
硬件急停和现场人员就位后由人员显式执行。
