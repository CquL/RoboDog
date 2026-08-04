# Unitree H2 统一迁移与使用说明

本目录是将当前 H2 的 Docker 双 IMU 接收能力和 `robot_hardware` 底层控制能力迁移到另一台兼容 H2 PC2 时的统一交付目录。原来的 `unitreeH2/H2_docker.md` 与 `robot_hardware/H2_test.md` 已合并到本文；迁移时复制本目录即可，不要再分别寻找两个旧文档。

## 1. 当前交付边界

已经包含：

- H2 Linux/amd64 运行镜像归档及校验文件；
- 编译好的 `librobot_hardware.so`、H2 统一测试程序和 Unitree SDK2/CycloneDDS 运行库；
- `unitree_h2_sensor_bridge`，可直接订阅 H2 原厂 DDS 并发布两路 ROS 2 IMU；
- 只读/停止配置和显式运动配置；
- Ubuntu 22.04 amd64 的可选离线 Docker 安装包；
- Windows 上传脚本、H2 端统一管理脚本和完整 SHA256 清单。

当前没有包含：

- `robot_hardware` 完整源码；当前运行不需要源码，修改适配器或重建镜像时才需要原仓库；
- 深度相机、点云、雷达驱动或对应 ROS 2 Bridge；
- 导航、SLAM、定位和路径规划算法；
- 生产型 `h2_runtime_controller`、自动 FSM 切换流程或开机自动运动控制；
- PC1 原厂运动服务或固件。它们仍由机器人原厂系统提供。

因此，本包当前能完成的是“两路 IMU 接收与 ROS 2 发布”和“通过抽象 HAL 执行受门禁的底层控制测试”，不是完整自主导航系统。

## 2. 数据链与控制链

IMU 数据链不经过 PC2 原厂 ROS 2 Topic，也不是 rosbridge：

```text
H2 原厂 DDS rt/lowstate.imu_state
  -> Docker 内 unitree_h2_sensor_bridge
  -> /h2/imu/pelvis [sensor_msgs/msg/Imu]

H2 原厂 DDS rt/secondary_imu
  -> Docker 内 unitree_h2_sensor_bridge
  -> /h2/imu/torso [sensor_msgs/msg/Imu]
```

底层控制链为：

```text
robot_test_unitree_h2 或未来上层控制程序
  -> RobotFactory::RobotAllocate()
  -> RobotHardwareInterface
  -> UnitreeH2
  -> Unitree SDK2 LocoClient
  -> H2 原厂 DDS/RPC 运动服务
```

镜像中的适配器实现了：

```cpp
initRobotHardware();
writeRobotVelocityCommand(cmd);
writeActionCommand(action);
```

ROS 2 只用于容器内统一输出 IMU 话题；H2 运动控制不依赖发布 ROS 2 速度话题。

## 3. 目录内容

```text
H2_migration_bundle/
├── README.md
├── SHA256SUMS
├── h2_bundle.sh
├── transfer_to_h2.ps1
├── config/
│   ├── deployment.env.example
│   ├── unitree_h2_container_safe.yaml
│   └── unitree_h2_container_motion.yaml
├── images/
│   ├── unitree_h2_amd64_runtime_candidate.tar.gz
│   └── unitree_h2_amd64_runtime_candidate.tar.gz.sha256
└── docker_engine/
    ├── docker_offline_jammy_amd64_24.0.7.tar.gz
    └── docker_offline_jammy_amd64_24.0.7.tar.gz.sha256
```

`config/deployment.env` 和 `state/` 是目标机器人本地生成内容，不属于迁移清单，也不应复制到另一台机器人。

## 4. 目标设备前提

- 目标是 H2 PC2，系统为 Ubuntu 22.04、`linux/amd64`；本镜像不能直接用于 ARM64 PC3/PC4；
- H2 PC1/PC2 原厂内部网络正常；当前已验收配置为 PC2 `eth0`、DDS domain `0`、PC1 `192.168.123.161`；
- 外接 Wi-Fi/USB 网卡只负责远程登录时，不要把它误设为 H2 内部 DDS 网卡；
- 建议至少预留 10 GB 空间；
- 运动测试必须有第二操作员持原厂遥控器，并具备空旷场地和停止/阻尼条件。

另一台 H2 的接口名或 PC1 地址不同时，必须修改目标机自己的 `config/deployment.env`，不能直接沿用旧机器生成的 `state/`。

## 5. 从 Windows 上传整个目录

推荐在 PowerShell 执行统一上传脚本：

```powershell
powershell -ExecutionPolicy Bypass -File `
  "D:\Desktop\RoboDog\unitreeH2\H2_migration_bundle\transfer_to_h2.ps1" `
  -VerifyOnly

powershell -ExecutionPolicy Bypass -File `
  "D:\Desktop\RoboDog\unitreeH2\H2_migration_bundle\transfer_to_h2.ps1" `
  -HostAddress 192.168.123.162
```

脚本会在本机校验完整 `SHA256SUMS`，上传到临时目录，在 H2 再次校验后才改为最终目录：

```text
/home/unitree/p2_unitreeH2/H2_migration_bundle
```

为避免新旧文件混合，最终目录已存在时脚本会停止，不会覆盖或删除。请先人工核对旧目录，再自行改名归档；不要直接用 `scp -r` 合并到旧目录。

## 6. H2 上首次部署

以下命令均在 H2 PC2 宿主机执行：

```bash
cd /home/unitree/p2_unitreeH2/H2_migration_bundle
chmod +x h2_bundle.sh
bash h2_bundle.sh verify
```

成功标记：

```text
H2_MIGRATION_BUNDLE_OK
```

为当前 H2 创建本地配置：

```bash
cp config/deployment.env.example config/deployment.env
nano config/deployment.env
bash h2_bundle.sh show-config
bash h2_bundle.sh check-host
```

`check-host` 会确认 amd64、DDS 网卡 UP 且有 IPv4，并要求去往 PC1 的实际路由使用同一 DDS 网卡。当前交付机一般保持：

```text
H2_DDS_INTERFACE=eth0
H2_DDS_DOMAIN=0
H2_ROS_DOMAIN_ID=20
H2_PC1_ADDRESS=192.168.123.161
```

### 6.1 可选：离线安装 Docker Engine

先检查：

```bash
command -v docker || echo DOCKER_NOT_INSTALLED
```

只有未安装 Docker 时才执行：

```bash
bash h2_bundle.sh install-docker
```

离线包是交付 H2 PC2 Ubuntu 22.04 上验收过的 6 个 amd64 包，不是任意 Jammy 系统的完整依赖闭包。若安装失败，脚本不会联网修复；运行 `sudo dpkg --audit` 查看状态后停止部署，不要盲目执行 `apt -f install`。

### 6.2 导入并核验运行镜像

```bash
bash h2_bundle.sh load-image
```

脚本会校验归档、导入镜像，并核对镜像 ID、架构、SDK2 提交号和 HAL/Bridge 运行文件。必须出现：

```text
H2_RUNTIME_ARTIFACTS_OK
H2_RUNTIME_IMAGE_LOAD_OK
```

## 7. 启动和验证双 IMU Bridge

```bash
bash h2_bundle.sh start-imu
bash h2_bundle.sh verify-imu
```

`start-imu` 会完成以下工作：

- 启用 Docker 服务开机启动；
- 根据本机 `deployment.env` 生成目标专用 HAL YAML；
- 同时覆盖 Bridge 的 DDS 网卡/domain 参数，避免镜像内默认值抢占环境变量；
- 使用 `--network host`、只读根文件系统、移除 Linux capabilities；
- 创建 `unitree-h2-imu-bridge` 容器并设置 `--restart unless-stopped`；
- 等待 `H2_IMU_BRIDGE_READY` 后才报告成功。

“Bridge 进程只接收状态”不等于“整个镜像没有控制能力”：镜像还包含 HAL 控制二进制，拥有 Docker 权限的人仍可通过显式 `docker exec` 发控制命令。

查看状态：

```bash
bash h2_bundle.sh status
sudo docker ps -a --format \
  'table {{.Names}}\t{{.Image}}\t{{.Status}}'
```

应存在：

```text
/h2/imu/pelvis [sensor_msgs/msg/Imu]
/h2/imu/torso  [sensor_msgs/msg/Imu]
H2_IMU_LIVE_SAMPLES_OK
```

查看频率或单条样本：

```bash
sudo docker exec unitree-h2-imu-bridge \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS2CLI_DISABLE_DAEMON=1
timeout 10s ros2 topic hz /h2/imu/pelvis
timeout 10s ros2 topic hz /h2/imu/torso
'
```

机器人重启后，先确认 Docker 服务和容器：

```bash
sudo systemctl start docker
sudo docker start unitree-h2-imu-bridge 2>/dev/null || true
bash h2_bundle.sh status
```

Docker 服务启用且容器 restart policy 为 `unless-stopped` 时，容器通常会自动恢复。`docker start` 只启动已有容器，不会创建容器，也不会升级其镜像；换镜像或改配置时重新执行 `bash h2_bundle.sh start-imu`。

停止但保留容器：

```bash
bash h2_bundle.sh stop-imu
```

## 8. 底层控制测试前置门禁

先由人工将 H2 置于已验证的“常规运控 1”，并要求实机查询为 `FSM=601`。不要把 RPC 返回 `0` 理解成机器人必然发生物理位移；它只表示该次 SDK/RPC 调用成功返回。

只读预检：

```bash
bash h2_bundle.sh preflight
```

必须同时出现：

```text
fsm_id=601
H2_GETTER_ONLY_RPC_OK
H2_CONTROL_READ_ONLY_PREFLIGHT_OK fsm_id=601
```

若不是 `601`，不要执行后面的写命令。执行任何非零速度前还必须满足：

- 平整空旷区域内无人和障碍物；
- 第二操作员持原厂遥控器，随时可以停止或进入阻尼；
- 摇杆中立，且没有其他算法、遥控或测试进程同时发送速度；`preflight` 会检查容器内常见进程名，但无法证明遥控器、宿主机或其他 DDS 参与者没有发送；
- 每条命令单独执行，确认机器人停止后才执行下一条；
- 异常运动时优先用原厂遥控器处置，不等待终端命令。

进入干净的容器 Shell：

```bash
bash h2_bundle.sh shell
```

该命令使用 `bash --noprofile --norc`，避免旧 Jezetek 环境在启动时加载 `/my_workspace/...`。

容器内设置一次路径变量：

```bash
BIN=/opt/robodog/bin/robot_test_unitree_h2
SAFE=/opt/robodog/config/unitree_h2_container_safe.yaml
MOTION=/opt/robodog/config/unitree_h2_container_motion.yaml
```

### 8.1 零速度停止检查

```bash
"$BIN" \
  --config "$SAFE" \
  --zero-stop \
  --execute

echo "RC=$?"
```

`--zero-stop` 会发送 `StopMove` 和零速度，是写 RPC，不是只读操作，也不能替代遥控器或物理急停。预期标记：

```text
H2_ZERO_STOP_RPC_OK fsm_id=601
RC=0
```

### 8.2 用原生 HAL 已验证参数验收 Docker 前进

```bash
"$BIN" \
  --config "$MOTION" \
  --velocity \
  --vx 0.50 \
  --vy 0 \
  --omega 0 \
  --duration-ms 1000 \
  --execute

echo "RC=$?"
```

该参数在 PC2 原生 r11、同一 HAL 调用链上已验证为机器人向前移动并停止；当前 runtime Docker 内尚未留下这条前进命令的物理验收记录。迁移到新 H2 后仍必须把它作为一次新的 Docker 实机验收。软件侧应看到 20/20 次发送成功、`StopMove` 和零速度成功、前后均为 FSM 601，并由观察员记录真实位移。

### 8.3 分档验证转向

正负角速度的物理方向尚未标定，第一次只记录为 `yaw-positive`/`yaw-negative`。先从一条较短命令开始：

```bash
"$BIN" \
  --config "$MOTION" \
  --velocity \
  --vx 0 \
  --vy 0 \
  --omega 0.15 \
  --duration-ms 1000 \
  --execute

echo "RC=$?"
```

确认已停止且无异常后，才可以单独测试 `--omega -0.15`。此前 `-0.15 rad/s、1000 ms` 出现过 RPC 全部成功但没有可见转向，因此不要因“没动”就连续重试或直接跳到项目上限。

当前运动 YAML 中的项目侧软件包络是：

```text
max_vx=1.00 m/s
max_vy=0.10 m/s
max_omega=0.70 rad/s
duration=100..3000 ms
```

这些不是宇树公布的 H2 官方机械极限，`1.00/0.70` 也没有完成完整实机分档验收。输入 5000 ms 会被测试程序拒绝并返回 `RC=64`。

### 8.4 再次发送软件停止

```bash
"$BIN" --config "$SAFE" --zero-stop --execute
```

本迁移包的 motion 配置默认 `allow_state_changing_actions: false`，会拒绝未验收的状态动作；安全动作 `stop_move` 仍可使用。本交付包没有经过验收的自动 FSM 生命周期控制，本次迁移不要测试 `stand_up`、`start`、`damp`、`squat` 或 `sit`。

退出容器：

```bash
exit
```

## 9. 上层算法接入时怎么用

上层程序应链接镜像内的 `librobot_hardware.so`，读取目标专用 YAML，通过工厂创建 `robot_model: unitree_h2`，然后按顺序调用：

```text
1. initRobotHardware()
2. 控制循环中持续调用 writeRobotVelocityCommand(cmd)
3. 正常退出或故障时调用 writeActionCommand("stop_move") 并发送零速度
```

当前统一测试程序已经演示了这条调用链，并以 20 Hz 重复发送速度。未来导航控制器仍需自行实现控制循环、命令超时处理、独占发送者管理和进程退出收尾；仅调用一次 `initRobotHardware()` 不会让机器人移动。

若导航算法运行在另一个容器并订阅本包的 ROS 2 IMU，它也应使用 host network、相同 `ROS_DOMAIN_ID=20` 和兼容的 ROS 2 RMW；否则即使 Bridge 正常发布，也可能无法发现 `/h2/imu/*`。

若要修改 H2 适配器、测试程序或限幅，再使用原仓库：

```text
D:\Desktop\RoboDog\robot_hardware\robot_hardware
```

修改源码/YAML 后必须重新编译、重建并重新导出镜像；只改 Windows 源码不会自动改变已经导入 H2 的 Docker 镜像。

## 10. 常见问题

### IMU 话题不存在或无样本

```bash
bash h2_bundle.sh show-config
bash h2_bundle.sh check-host
bash h2_bundle.sh status
sudo docker logs --tail 100 unitree-h2-imu-bridge
```

重点检查：目标 H2 的内部 DDS 网卡/domain、去 PC1 的路由、容器是否为 host network，以及日志是否出现 `H2_IMU_BRIDGE_READY`。外接 ZTE/USB Wi-Fi 网卡取得互联网 IP，不代表它是 PC1 DDS 网卡。

### 容器仍使用旧镜像

```bash
sudo docker inspect --format 'container={{.Image}} ref={{.Config.Image}}' \
  unitree-h2-imu-bridge
sudo docker image inspect --format 'tag={{.Id}} arch={{.Architecture}}' \
  unitree_h2:amd64-runtime-candidate
```

加载新镜像不会自动更新旧容器。确认同名容器确属本项目后，重新执行 `bash h2_bundle.sh start-imu`。

### RPC 返回 0 但机器人没动

返回 0 证明 SDK/RPC 接受了调用，不证明产生位移。检查 FSM 是否仍为 601、现场速度是否低于可见阈值、是否存在另一发送者、机器人是否处于正确的“常规运控 1”，并保留软件日志和观察员记录。不要绕过门禁盲目提高速度。

### `bash: : command not found` 或变量为空

重新进入干净 Shell，并再次设置：

```bash
bash h2_bundle.sh shell
BIN=/opt/robodog/bin/robot_test_unitree_h2
SAFE=/opt/robodog/config/unitree_h2_container_safe.yaml
MOTION=/opt/robodog/config/unitree_h2_container_motion.yaml
```

## 11. 版本与已验证证据

```text
镜像标签：unitree_h2:amd64-runtime-candidate
H2 PC2 镜像配置 ID：
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
镜像归档 SHA256：
ba8025b3250d01544a41272dc50b8b71803d86f582856915ee6f8d7009a1f5a5
SDK2 commit：21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
离线 Docker 包 SHA256：
e331e68c504f249847f238363eeb2e24fb561e2ab9b6a8b94004a3d360d1d224
```

已验证：H2 PC2 Docker Engine 24 加载该 amd64 镜像；两路 IMU 均读取到有效 `sensor_msgs/msg/Imu`；原生 r11 HAL 在 `vx=0.50、1000 ms` 时机器人向前移动并停止。

尚未验证：本统一目录在第二台 H2 的完整部署；雷达/深度相机接入；`vy` 与角速度正负方向标定；项目上限的完整物理效果；生产型导航控制器和自动 FSM 状态机。
