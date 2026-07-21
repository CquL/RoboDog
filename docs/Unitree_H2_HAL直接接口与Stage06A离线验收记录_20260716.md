# Unitree H2 HAL 直接接口与 Stage 06A 离线验收记录

日期：2026-07-16  
工作区：`D:\Desktop\RoboDog`  
结论：H2 三接口到 SDK2 高层 API 的代码映射与离线错误边界已通过测试；Stage 06A
自包含镜像门禁通过；尚未执行 H2 `LocoClient` 实机 RPC、零速度、动作或非零运动。

## 1. 本轮目标与不变边界

本轮只保证以下调用链在源码、构建和离线测试层成立：

```text
算法输出
  -> RobotVelocityCommand / 项目动作常量
  -> RobotHardwareInterface 三接口（进程内 C++ 调用）
  -> UnitreeH2
  -> Unitree SDK2 h2::LocoClient
```

没有发布宇树 ROS 2 控制 topic，没有使用 `rt/lowcmd`，没有连接 H2 网络，也没有执行
任何实机控制。`RobotFactory::RobotAllocate()` 仍完整内联保留在
`robot_hardware/robot_hardware/include/robot_factory.h`；本轮没有把 Factory 拆到 `.cpp`，
也没有恢复 `ROBOT_HARDWARE_HAS_UNITREE_H2` 或 H2 构建开关。

## 2. 已核对的直接 API 映射

| 项目调用 | `UnitreeH2` 最终调用 | 安全/错误合同 |
|---|---|---|
| `initRobotHardware()` | `ChannelFactory::Init()`、`LocoClient::Init()`、`SetTimeout()`，可选 `GetFsmId()` | 配置校验、初始化异常归一化、FSM 不可读时拒绝就绪 |
| `writeRobotVelocityCommand()` | `SetVelocity(vx, vy, omega, duration)` | NaN/Inf 拒绝、三轴限幅、默认拒绝非零速度、异常返回 `1005` |
| `writeActionCommand("stop_move")` | `StopMove()` | 失败时保留 watchdog 重试条件，异常返回 `1006` |
| `stand_up` | `StandUp()` | 默认禁用状态动作，异常/厂商失败返回 `1004` |
| `prepare_motion` | `Start()` | 同上 |
| `damp` / `squat` / `sit` | `Damp()` / `Squat()` / `Sit()` | 同上 |
| `lie_down` / 未知动作 | 不调用 SDK | 返回 `1007`；不猜测 H2 不存在的同义 API |

SDK2 内部仍使用 CycloneDDS RPC 与 H2 PC1 的原厂 `sport` 服务通信。这是厂商传输层，
不是上层 ROS 2 控制 topic；因此原厂 `sport` 服务不能关闭。

## 3. 本轮代码加固

修改范围：

- `src/unitree/unitree_h2.cpp`：所有 SDK2 调用增加 `std::exception` 与未知异常边界，保留
  原厂返回码日志，同时维持项目 `int32_t` 错误码合同；
- `include/unitree/unitree_h2.h`：watchdog 线程入口声明为 `noexcept`；
- `tests/fakes/unitree/robot/h2/loco/h2_loco_client.hpp`：增加无 DDS 的 fake
  `ChannelFactory` / `LocoClient` 记录器与故障注入；
- `tests/unitree_h2_direct_api_contract_test.cpp`：新增直接映射、联锁、异常、watchdog、
  FSM 和析构合同测试；
- `CMakeLists.txt`：新增 `unitree_h2_direct_api_contract_test`，它直接编译生产
  `unitree_h2.cpp`，但优先包含 fake SDK 头，不创建 DDS 实体；
- Docker 配方安装并在构建阶段运行上述测试。

修复前，测试真实捕获到 `fake SetVelocity failure` 越过 `int32_t` 接口并失败。修复后，
速度、StopMove、动作、FSM、watchdog 与析构中的 SDK 异常都不会逃逸出项目接口。

## 4. 离线测试结果

测试容器约束：

```text
--network none
--cap-drop ALL
--security-opt no-new-privileges
robot_hardware 源码只读挂载
临时构建仅写入容器 /tmp，--rm 后删除
```

完整 `robot_hardware` Release 构建成功，CTest 结果：

```text
unitree_h2_factory_contract_test ...... Passed
unitree_h2_direct_api_contract_test ... Passed
100% tests passed, 0 tests failed out of 2
```

直接 API 测试最终标记：

```text
UNITREE_H2_DIRECT_API_CONTRACT_OK
```

覆盖项包括：

- Factory 的 `unitree_h2 -> UnitreeH2` 分配；
- 初始化参数、DDS domain/网卡、timeout 转发与二次初始化幂等；
- 默认拒绝非零速度与状态动作；
- 零速度、三轴限幅、持续时间转发；
- 五个支持动作的一一映射；
- `lie_down`/未知动作在到达 SDK 前拒绝；
- SDK 厂商错误码及抛异常的统一返回；
- watchdog 超时 StopMove、失败重试与线程不终止；
- `GetFsmId` 成功、失败、异常与初始化就绪门禁；
- 非有限配置拒绝、析构 StopMove 异常不导致 `std::terminate`。

## 5. Stage 06A 镜像验收

构建方式：Docker BuildKit named contexts，`--pull=false`、构建 `RUN` 网络为 `none`；
没有挂载宿主 `/opt/unitree_robotics`。

最终镜像：

```text
name: unitree_h2:amd64-offline
image/repo digest: sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa
architecture: amd64
size: 3,532,983,559 bytes
base image id: sha256:6429796814f0fdd7a54e7ba2ad2e3387a0342037cb1da874a604f72cc34fc257
SDK2 commit: 21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
entrypoint: empty
cmd: /bin/bash
```

镜像产物哈希：

| 产物 | SHA256 |
|---|---|
| `/opt/robodog/lib/librobot_hardware.so` | `2977a772bc4aa2b6cb2403209c9f89f4a502f35434042fc7fbab147acd60b918` |
| `unitree_h2_factory_contract_test` | `345dc439f9b4695350ecc530674f146d9cc6680ce7e53306ca9bcf5007431cfb` |
| `unitree_h2_direct_api_contract_test` | `db7643140dcf0036df099e91e3d48dcbd1fe124cb24caf44508d3bc6e202d27a` |
| `libunitree_sdk2.a` | `08402aea74150dfbfc3fbfded4ca746916a8d892b54d2bade0cbf392a3be4029` |
| `libddsc.so` | `4038630c231412f7b34a2ea60df192bbdebd0a57f22ac7f35c1b6d28323e695c` |
| `libddscxx.so` | `d3e7c1b03123c2745839f2465041777ded090ad66d62f1372949209254f7ebe5` |

最终运行验收同样使用 `--network none`、只读根文件系统和 drop-all capabilities，输出：

```text
UNITREE_H2_FACTORY_CONTRACT_OK
UNITREE_H2_DIRECT_API_CONTRACT_OK
H2_STAGE06A_IMAGE_OFFLINE_OK
H2_STAGE06A_HOST_GATE_OK
```

可重复执行脚本：

```powershell
powershell -ExecutionPolicy Bypass -File `
  unitreeH2\docker\verify_stage06a_image_offline.ps1 `
  -ExpectedImageId sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa
```

`ldd` 没有 `not found`，H2 HAL 未发现 `librcl*`、`librmw*` 或 `libros*` 运行依赖。
Dockerfile/Compose 已删除错误的 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；镜像还清除了
从导航底座继承的 `/ros_entrypoint.sh`，当前启动只进入 Bash，不隐式 source ROS 2。

## 6. 当前可以和不可以下的结论

现在可以确认：

- Factory 保持原结构且能创建 H2 对象；
- 三个抽象接口已准确落到 H2 高层 SDK2 API；
- 映射、安全联锁和 SDK 异常边界已由 fake SDK 离线执行验证；
- H2 HAL 镜像不依赖宿主 SDK2 或 PC2 ROS 2 workspace 来完成构建/离线运行；
- Stage 05 r2 已独立证明当前 PC2 原生状态输入可读。

仍然不能确认：

- 固定镜像内 SDK2 与当前 H2 固件/PC1 `sport` 服务已完成 RPC 兼容验证；
- 镜像已包含 `unitree_h2_state_source`、算法或正式 `h2_runtime`；
- H2 已经能由该 HAL 实机运动；
- `StopMove()` 等价于硬件急停。

## 7. 下一步计划

1. 先获得对 PC2 安装 Docker Engine 的明确授权，或选择另一台已装 Docker、同网段的
   amd64 主机；当前盘点显示 PC2 没有 Docker，本轮没有安装。
2. Stage 06B：把 Stage 05 的七 topic 纯订阅逻辑升级为生产型
   `unitree_h2_state_source`，放入镜像并在 H2 网络上只读验证，不调用 Loco RPC。
3. Stage 06C：单独审批固定镜像中的 `GetFsmId()` 只读 RPC，验证该 SDK2 快照与当前
   `sport` 服务兼容。它会发送 DDS 请求，因此不能与纯订阅门禁混为一轮。
4. 之后才允许保护支架、遥控器、硬件急停和现场人员就位条件下的零速度与
   `StopMove()`；非零速度和状态动作属于更后门禁。
5. 算法接入时建立同进程 `h2_runtime`：状态源更新统一缓存，算法输出
   `RobotVelocityCommand`，同一进程持有 `RobotHardwareInterface` 对象并直接调用三接口。
