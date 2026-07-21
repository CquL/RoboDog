# Unitree H2 导航传感器现状与 Stage 06B 实机门禁记录

日期：2026-07-16  
工作区：`D:\Desktop\RoboDog`  
输入证据：宇树 H2 官方产品页/开发文档、本地固定 SDK2、PC2 只读盘点与 Stage 05 r2 日志  
状态：完成现状判定和下一阶段设计；本轮未向 H2 发送 RPC、速度、动作或 FSM 命令

## 1. 本轮结论

`unitree_h2:amd64-offline` 当前是 **HAL-only 镜像**：它包含 Unitree SDK2、
`robot_hardware`、H2 适配器和离线合同测试，但没有常驻传感器采集程序、导航算法或
`h2_runtime`。镜像的 `ENTRYPOINT` 为空、默认命令是 `/bin/bash`，所以启动后不会自动
接收、缓存或向算法提供任何 H2 数据。

Stage 05 r2 证明的是：在 PC2 宿主机上临时编译运行一个 15 秒只读探针，可以收到 H2
机身状态。该探针没有安装进 Docker，也没有生产型 getter、快照缓存、数据新鲜度判断或
长期生命周期，因此不能把 Stage 05 表述成“Docker 传感器已接入”。

当前不应立即执行机器人运动测试。正确顺序是先完成生产型状态源和只读实机验证，再单独
批准 `GetFsmId()` / `GetAvailableFsmIds()` 的只读 RPC，之后才是在完整现场安全条件下的
零速度和 `StopMove()`；非零速度及状态动作属于更后阶段。

## 2. “HAL-only”准确含义

当前镜像已经具备：

- 固定版本的 Unitree SDK2 与 CycloneDDS 运行库；
- `librobot_hardware.so` 及 `UnitreeH2` 高层适配器；
- `RobotFactory` 的 `unitree_h2 -> UnitreeH2` 分配；
- 三个抽象接口到 H2 `LocoClient` 的离线映射测试；
- H2 YAML 配置和断网、无机器人条件下的可重复验收程序。

当前镜像尚不具备：

- `unitree_h2_state_source` 或传感器 bridge；
- 类型化 `RobotStateSnapshot`、时间戳、新鲜度和掉线判断；
- 相机、雷达/激光雷达等导航传感器驱动；
- 导航、定位、规划或控制算法；
- 将“状态输入 -> 算法 -> 三个 HAL 接口”闭环起来的常驻 `h2_runtime`；
- 自动启动以上组件的 entrypoint。

`network_mode: host` 只给未来的 DDS/传感器通信提供网络条件。没有接收进程时，开放网络
并不会自动产生传感器数据。

## 3. 目前确认的 H2 数据输入

### 3.1 已在本机 PC2 实测确认

Stage 05 r2 在 15 秒纯订阅中得到：

| 输入 | 样本/频率 | 当前可证明内容 | 尚未冻结内容 |
|---|---:|---|---|
| `rt/lowstate` | 15,718，约 1048 Hz | 35 个电机槽的消息、骨盆 IMU、tick、CRC；全部 LowState CRC 通过 | 31 个实际关节的完整索引、方向、零位、限位和单位 |
| `rt/lf/lowstate` | 302，约 20.16 Hz | LowState 低频镜像 | 主备切换和过期阈值 |
| `rt/secondary_imu` | 15,720，约 1048 Hz | 第二路 IMU 的四元数、角速度、加速度、RPY、温度字段有非零数据 | 精确安装位置、轴向、单位、标定和协方差 |
| `rt/lf/secondary_imu` | 302，约 20.16 Hz | 第二路 IMU 低频镜像 | 主备切换和过期阈值 |
| `rt/lf/bmsstate` | 302，约 20.16 Hz | SOC、SOH、电流、电压、温度等原始状态可解码 | 所有 raw 字段缩放、单位和故障位 |
| `rt/lf/mainboardstate` | 302，约 20.16 Hz | 风扇、温度、value/state 原始数组可解码 | 完整字段语义与故障合同 |

这些是“机身状态/健康状态”，不是完整导航传感器集合。BMS 和 MainBoard 可用于健康门禁，
但不能代替定位、避障所需的点云、图像或可信里程计。

### 3.2 PC2 上存在但尚不能作为导航输入

| 候选 | 只读实测 | 判定 |
|---|---|---|
| `/dog_imu_raw` | 约 500 Hz；样本只有姿态有效，角速度与线加速度均为 0 | 不是当前可用的完整导航 IMU |
| `/dog_odom` | 约 500 Hz；pose/twist 有数值，但协方差全 0，节点/帧仍使用 `dog_*` 旧命名 | 仅为待验证候选，不能直接作为定位真值 |
| `/point_in_map` | 图中有 publisher endpoint，但 10 秒内没有样本 | 当前不可用 |
| `/tf` | 图中有 publisher endpoint，但 10 秒内没有样本，且未形成完整 frame tree | 当前不可用 |
| `/frontvideostream` | 只有 subscriber，没有 publisher | 当前没有可用相机流 |

PC2 文件系统中同时存在 RoboSense、Hesai、Livox、LIO-SAM 等软件，运行进程还带有
`rslidar32`、`Go2`、`dog_*` 等遗留命名。这只能证明系统安装或复用了这些软件，不能证明
本台 H2 实际装有对应品牌/型号的传感器。盘点时 `192.168.124.20` 邻居解析为
`INCOMPLETE`，也没有 PC2 可见的 `/dev/video*`，所以不能据此宣布雷达或相机已在线。

## 4. 官网明确了哪些传感器

宇树 H2 官方产品页当前明确的感知配置只有：

- H2/H2 EDU：大视场角仿人双目相机；
- 阵列式麦克风和大功率扬声器；
- H2 EDU 的 PC2 为 Intel Core i7，用于用户开发。

官方公开页面没有给出双目相机的厂家、型号、分辨率、帧率、编码方式、深度输出合同、
内参或外参；也没有把某个具体雷达/激光雷达型号列为 H2 标配。H2 软件架构页面提到
音视频、雷达点云以及“电机、雷达等传感器”进入 DDS，这描述的是平台软件能力，不能反推
本台交付机器的实际雷达 BOM。

官方 About H2 文档列出了 PC2 可用的 USB 3.0、以太网、CAN、RS-485 等扩展接口；GMSL
视频输入则与选配 PC3 有关。这同样只能说明接口能力，不能确定本台相机/雷达实际接在
PC1、PC2、PC3、USB、GMSL 还是以太网上。

官方来源：

- H2 产品页：<https://www.unitree.com/cn/H2/>
- H2 SDK 开发指南入口：<https://support.unitree.com/home/zh/H2_developer>
- About H2：<https://support.unitree.com/home/zh/H2_developer/About_H2>
- 软件架构说明：<https://support.unitree.com/home/zh/H2_developer/architecture_description>
- 底层服务接口：<https://support.unitree.com/home/zh/H2_developer/basic_services_interface>
- DDS 通信接口：<https://support.unitree.com/home/zh/H2_developer/dds_services_interface>

## 5. 数据手册/PDF 现状

截至本轮核对：

- 官方已经有 [H2 在线手册入口](https://www.unitree.com/app/H2/)、
  [H2 User Manual](https://marketing.unitree.com/article/en/H2/User_Manual.html) 和
  H2 SDK 开发指南；
- 在线用户手册是网页/逐页图片形式，不是可下载的完整 H2 PDF；
- 本地 `unitreeH2/official_docs/` 是官方网页内容的工程快照；
- 在 H2 官网、文档中心、公开下载入口和本地 H2 资料中，没有找到公开的 H2 整机数据表
  PDF，也没有找到本台双目相机、IMU 或交付雷达的独立型号数据手册 PDF。

因此必须向宇树或交付方补齐以下交付资料，不能用软件目录猜测：

1. 本机完整传感器 BOM、厂家、型号、序列号及接线/网口拓扑；
2. 双目相机分辨率、帧率、编码、左右目同步、内参、畸变和外参；
3. 若有雷达/激光雷达：型号、IP、UDP/串口合同、点型、时间戳和驱动/SDK 版本；
4. 两路 IMU 的芯片型号、安装位置、坐标轴、单位、量程、标定、协方差和时间同步；
5. `odommodestate`/`dog_odom` 的来源、坐标系、重置行为、漂移和协方差定义；
6. 传感器到骨盆/机身的静态外参，以及 PC1/PC2/可选 PC3 的时钟同步方法；
7. 当前固件、PC1 `sport` 服务与 SDK2 commit 的兼容矩阵。

## 6. Docker 是否必须植入驱动

答案按输入类型区分：

| 输入类型 | 是否需要新增厂商硬件驱动 | 当前应做什么 |
|---|---|---|
| `rt/lowstate`、两路 IMU、BMS、MainBoard | 不需要额外硬件驱动；SDK2 已能解码 DDS | 实现生产型 `unitree_h2_state_source` |
| 双目相机原始/编码视频 | 通常需要与真实连接方式匹配的驱动、SDK 或流解码器 | 先确认型号、连接到哪台计算单元和输出协议，再放入镜像 |
| 雷达/激光雷达原始数据 | 需要与实际型号严格匹配的厂商驱动/SDK | 未确认 BOM 前不得复制 X30 Livox 配置 |
| 已由宿主发布的标准 ROS 图像/点云/里程计 | 容器可不装原始设备驱动，但会依赖宿主 ROS 版本、topic、QoS、frame 和启动服务 | 可用于过渡验证，不应作为最终可移植合同 |
| 原厂 DDS 中已有且合同明确的状态/里程计 | 不需要 ROS 2 驱动，可用 SDK2/DDS 直接订阅 | 必须先验证 H2 类型、字段语义、活性与时间合同 |

为了实现“把镜像迁移到同型号 H2 后直接运行”，优先方案是把已确认传感器的用户态
驱动/SDK、配置和标定装入 H2 专属镜像；宿主只提供 Linux 内核、Docker、物理设备和网络。
这能减少对 PC2 原厂 ROS workspace 的依赖，但仍无法消除架构、设备节点、网卡、固件、
标定和时间同步这些硬件合同。

## 7. 最终运行架构

最终应使用单容器、单控制进程：

```text
h2_runtime（同一进程）
├─ H2StateSource：SDK2/DDS 只读订阅
├─ Camera/Lidar source：仅装入已确认型号的驱动/SDK
├─ RobotStateSnapshot：统一状态、时间、frame、新鲜度和健康门禁
├─ Algorithm::step(state) -> RobotVelocityCommand / action
├─ RobotFactory::RobotAllocate(config)
└─ RobotHardwareInterface
   └─ UnitreeH2
      └─ H2 LocoClient
         └─ SDK2 内部 CycloneDDS RPC -> PC1 sport 服务
```

三个 C++ 虚函数只能在同一地址空间中被算法“直接调用”。如果算法与 HAL 分成两个进程，
中间就必须新增共享内存、Unix socket、ROS 2 或其他 IPC，不能再称为直接调用 C++ 对象。

`H2StateSource` 与 `UnitreeH2` 还必须共享唯一的 `ChannelFactory` 生命周期所有者：进程只用
同一个 domain/网卡初始化一次 DDS，停止控制和订阅后，最后统一 `Release()`。不能让状态源
退出时释放 HAL 仍在使用的全局 DDS。

## 8. Stage 06B 实施内容

建议新增：

```text
unitreeH2/runtime/
├─ CMakeLists.txt
├─ include/h2_state_snapshot.h
├─ include/unitree_h2_state_source.h
├─ src/unitree_h2_state_source.cpp
├─ src/h2_runtime_main.cpp
├─ config/h2_runtime.yaml
└─ tests/
   ├─ state_source_contract_test.cpp
   ├─ state_source_crc_test.cpp
   ├─ state_source_stale_timeout_test.cpp
   └─ state_source_replay_test.cpp

unitreeH2/docker/
└─ h2_runtime_entrypoint.sh
```

生产状态源最低合同：

- 用固定大小类型化快照保存全部必要字段，不在约 1 kHz 回调内拼字符串；
- 同时记录机器人 tick 和本地单调接收时间；
- CRC 失败不得更新有效 LowState；
- 为每路输入记录频率、最后接收时间、乱序、掉线和 stale 状态；
- 明确高速 `rt/*` 主源、低频 `rt/lf/*` 备用源及切换条件；
- 急停、FSM、控制权或遥控器状态未知时，`healthy_for_motion=false`；
- 状态源只读，不持有 Publisher、控制 Client 或任何写通道；
- Docker 默认 `control_enabled=false`，断流只允许触发安全停止逻辑，不允许继续沿用旧命令。

## 9. 实机控制测试门禁

### Stage 06B-0：交付 BOM 与导航传感器只读核验

- 核对机身铭牌、线缆、端口、网卡邻居、USB/GMSL/以太网拓扑和厂商配置；
- 只读取配置与数据活性，不停止原厂服务，不发布 topic，不调用服务；
- 明确相机和雷达是否实际存在、型号以及由哪台计算单元负责。

### Stage 06B-1：本地生产状态源与离线测试

- 实现 `H2StateSource`、类型化快照和 DDS 生命周期管理；
- 用 fake/replay 覆盖 CRC 损坏、断流、过期、重复、乱序和重启；
- 构建新的 runtime 镜像，但保持控制关闭。

### Stage 06B-2：镜像内实机纯订阅

- 在 PC2 获得 Docker Engine 安装授权后运行，或使用同网段、已装 Docker 的 amd64 主机；
- 使用 `network_mode: host`、明确 H2 有线网卡、只读文件系统和最小权限；
- 只订阅状态，连续运行并与 PC2 宿主探针的频率、CRC 和字段做对照；
- 本阶段不初始化 `LocoClient`，不发送任何控制 RPC。

### Stage 06C：第一项 HAL 实机测试——只读 RPC

- 单独审批后执行 `GetFsmId()` 和 `GetAvailableFsmIds()`；
- 这会发送 DDS 请求，但不应改变运动状态；
- 验证固定镜像 SDK2 与当前 PC1 `sport` 服务的实际兼容性、返回码和超时；
- 失败只记录，不切换服务、不复制 PC2 未核验的库覆盖镜像。

### Stage 06D：零速度与 `StopMove()`

只有保护支架/吊装、手持遥控器、硬件急停、清空区域、双人现场确认和原厂模式确认全部
满足后，才允许显式执行一次零速度与 `StopMove()`。`StopMove()` 是软件运动命令，不等同
于硬件急停。

### Stage 06E：状态动作与非零速度

只有在 06B～06D 的输入健康、SDK ABI、FSM、控制权、方向/单位和零停行为全部通过后，
才设计最小幅度、最短持续时间的非零测试。当前尚未解锁。

## 10. 下一步决策

本轮决策是：**现在去 H2 做只读核验，但不做运动控制。**

下一项代码工作应是 Stage 06B-1 的生产型 `unitree_h2_state_source` 与离线测试；并行向
宇树/交付方索取传感器 BOM、相机/雷达/IMU数据手册和标定。完成后再决定是在 PC2 安装
Docker Engine，还是先用同网段的另一台 amd64 Docker 主机完成镜像内纯订阅。

只有 Stage 06B 通过后，才把 `GetFsmId()` 作为第一项真实 HAL 链路测试；当前不执行
`SetVelocity()`、`StopMove()`、`StandUp()`、`Start()`、`Damp()`、`Squat()` 或 `Sit()`。
