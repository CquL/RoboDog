# robot_hardware 当前架构说明

更新时间：2026-07-15

## 1. 定位

`robot_hardware` 是多机器人硬件抽象层（HAL）。上层算法只依赖
`RobotHardwareInterface`，各厂商适配器负责把统一命令转换为原厂 SDK、ROS 或 UDP
协议。

```text
上层导航/规划/控制算法
  -> RobotHardwareInterface
  -> RobotFactory（按 robot_model 选择适配器）
  -> UnitreeH2 / UnitreeDog / DeepRoboticsX30 / ZsibotZslOne
  -> 原厂 SDK2 / UDP / 厂商库
```

核心接口保持不变：

```cpp
virtual int32_t initRobotHardware() = 0;
virtual int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) = 0;
virtual int32_t writeActionCommand(std::string action) = 0;
```

统一速度语义：`vx` 前后线速度、`vy` 左右线速度、`omega` yaw 角速度。

## 2. 工厂与构建方式

工厂的完整分配逻辑直接保留在 `include/robot_factory.h`，打开头文件即可看到
`unitree_dog`、`unitree_h2`、`zsibot_zsl_one` 和 `deep_robotics_x30` 四个连续的
`if / else if` 分支。它保持项目原来的写法：没有厂商适配器 CMake 开关，也没有
`ROBOT_HARDWARE_HAS_*` 条件宏。普通构建会编译全部现有适配器；H2 与其他机器人在类和
控制实现上彼此独立，但共享同一个工厂和 `robot_hardware` 动态库。

`BUILD_X30_ONLY` 及其他 `BUILD_X30_*` 选项是 H2 接入前已有的 X30 离线构建/测试能力，
不是厂商工厂分配开关。`BUILD_X30_ONLY=ON` 仅供不具备 Unitree/Zsibot SDK 的 X30
测试环境使用，不改变普通完整构建的 Factory 写法。

## 3. 当前适配器

### Unitree H2 EDU

```text
include/unitree/unitree_h2.h
src/unitree/unitree_h2.cpp
config/unitree_h2.yaml
robot_test_unitree_h2.cpp
```

实现使用官方 SDK2 的 `unitree::robot::h2::LocoClient`：

| HAL 操作 | H2 高层 API |
|---|---|
| 速度 `vx/vy/omega` | `SetVelocity(...)` |
| `stop_move` | `StopMove()` |
| `stand_up` | `StandUp()` |
| `prepare_motion` | `Start()` |
| `damp` | `Damp()` |
| 旧 `lie_down` | 明确返回“不支持”；H2 `Damp()` 不等于趴卧动作 |
| `squat` | `Squat()` |
| `sit` | `Sit()` |

默认配置是安全锁定状态：

```yaml
allow_motion_commands: false
allow_state_changing_actions: false
```

非零速度会被拒绝；状态切换会被拒绝；速度经过有限值检查、限幅和超时看门狗。只有
本对象实际发过控制命令后，析构时才执行尽力而为的 `StopMove()`；纯只读初始化不会
写控制。探针默认或显式 `--read-only` 只读取 FSM，只有人员显式指定 `--zero-stop`
才发送零速度和停止合同；两者都不能代替物理急停。

### DeepRobotics X30

```text
include/deep_robotics/deep_robotics_x30.h
include/deep_robotics/x30_udp_protocol.h
src/deep_robotics/deep_robotics_x30.cpp
```

当前已有真实 UDP `0x150` 编码、速度限幅、CRC、序列号、远端校验、零速度看门狗和
状态验证测试，不再是空壳。X30 感知/地形链仍有独立安全边界：离线
`x30_plane_seg_core` 验证不等于已经接通在线 GridMap、TCP 地形发送或机器人运动。

### 其他现有适配器

- `UnitreeDog`：当前代码实质绑定 B2 `SportClient`，不能作为 H2 适配器复用。
- `ZsibotZslOne`：调用随仓库保存的厂商二进制库。

## 4. H2 配置与对象创建

配置：

```yaml
robot_model: unitree_h2
network_interface_card_name: eth0
```

工厂调用：

```cpp
YAML::Node config = YAML::LoadFile("config/unitree_h2.yaml");
auto robot = RobotFactory::RobotAllocate(config);
```

普通完整构建中，`robot_model=unitree_h2` 会直接创建 `UnitreeH2`；未知型号明确报错，
不会悄悄落到其他机器人实现。

## 5. 新机器人接入清单

1. 在 `include/<vendor>/` 和 `src/<vendor>/` 各建声明与实现文件。
2. 继承 `RobotHardwareInterface` 并实现三个纯虚函数。
3. 明确坐标轴、单位、动作语义和不支持能力；不能只做字符串同名映射。
4. 在 `include/robot_factory.h` 直接注册唯一 `robot_model` 分配分支。
5. 确认 CMake 的源文件收集会包含新增实现，并补齐新增厂商 SDK 的 include/link 条件；
   不为每个机器人额外创建工厂宏或适配器开关。
6. 增加单独配置，危险能力默认关闭。
7. 增加离线合同测试和“只读状态 + 零速度”探针。
8. 最后才进入有保护支架、硬件急停和现场监护的实机分阶段验证。

## 6. 当前接口的边界

三个函数足以完成最小运动命令抽象，但还不能完整表达安全状态与能力差异。后续建议用
向后兼容方式增加：

- `readRobotState()`：FSM、关节、IMU、电池、故障、通信新鲜度；
- `getCapabilities()`：支持的动作、速度范围、传感器与控制级别；
- `stop()/shutdown()`：显式、幂等的停止和资源释放；
- 枚举/结构化动作替代自由字符串；
- 命令时间戳、序列号和上层心跳合同。

这些增强不改变当前三接口调用方，但能防止上层把不同机器人的同名动作误认为完全等价。

## 7. 安全验证顺序

```text
源码与接口静态测试
  -> Docker --network none 构建/链接合同
  -> 实机只读状态发现
  -> 零速度 + StopMove
  -> 小速度、短时、架空/防跌倒测试
  -> 感知和算法闭环
```

当前 H2 已完成源码/接口与 Docker 断网合同，也通过 PC2 宿主原生的七 channel 只读状态
发现；该只读探针不是最终 Docker 状态源。尚未执行 H2 `LocoClient` 实机 RPC、零速度、
StopMove、动作或非零运动验证。
