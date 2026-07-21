# Unitree H2 PC2 DDS/SDK2 只读审计记录

日期：2026-07-16  
输入：`unitreeH2/remote/h2_pc2_dds_sdk_audit_20260716.log`  
状态：阶段 04 纯 ABI 审计完成；等待阶段 05 PC2 原生 HG 只读订阅  
安全边界：没有运行 H2 HAL、SDK RPC、速度、动作、`StopMove()` 或 `rt/lowcmd`。

## 日志完整性

```text
size=59407 bytes
SHA256=EDEA93B7A4C017185EE9D5A7605ADACE404833F5A98A6D6F3AE19880CA96110E
end marker=READ_ONLY_PC2_DDS_SDK_AUDIT_OK
```

## 网卡与 DDS

- `eth0` 是 Realtek RTL8153 USB 千兆网卡，驱动 `r8152`，地址
  `192.168.123.162/24`；这是当前交付栈使用的 Unitree DDS 链路。
- `eth1` 是 Intel I219-V 主板网卡，驱动 `e1000e`，地址
  `192.168.124.162/24`。
- CycloneDDS XML：
  `/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml`，SHA256
  `bc977aacd0e44804cb8da24d24d6fe5ed654aad9ed7b15ed3ef36d32f27a1796`。
- XML 使用 `Domain Id="any"`，只绑定 `eth0`，允许 SPDP 组播发现，没有静态
  Peers。它不直接证明实际 Domain 编号或远端 H2 participant 已经被发现。
- `unitree_slam`、`dog_cmd`、`sport_switch`、`bms_ctr` 的实际进程环境均使用
  `rmw_cyclonedds_cpp` 和同一 XML；`dog_cmd` 命令行也显式传入 `eth0`。
- 进程没有暴露 `ROS_DOMAIN_ID`，ROS2 默认预期为 Domain 0，但仍需图发现验证。

## 原厂服务异常

- `unitree_slam.service` 的 systemd 状态为 active，但启动日志显示应用层
  `slam`/`ctl` 未运行、请求失败，`h1_client` 因 `BadCastException` 中止并产生
  core dump。因此不能把 systemd active 等同于 SLAM/雷达健康。
- `shoujuan_server.service` 为 disabled + failed，直接错误是相对路径
  `translate_test.py` 不存在，连续重启后触发 rate limit。审计脚本没有触发该故障。
- `unitree_slam.service` 中把 `Environment=` 误写为 `Enviroment=`；实际进程仍有
  正确环境，说明环境来自其他启动层。当前不修改原厂 unit 文件。
- CycloneDDS XML 权限是 `0777`，也不在运行期间修改。

## SDK2 与 ABI 边界

- `/opt/unitree_robotics` 和 `/usr/local/lib/cmake/unitree_sdk2` 均声明 SDK2
  `2.0.0` 并导出静态 `unitree_sdk2` 目标。
- `/opt/unitree_robotics` 中的 `libunitree_sdk2.a`、DDS ELF 库和 `.so.0`
  软链接存在且结构正常，不存在 Windows ZIP 的短文本软链接问题。
- 当前日志未深入确认 H2 专用
  `include/unitree/robot/h2/loco/h2_loco_client.hpp` 是否存在，也未验证静态库 ABI。
- `ldconfig` 把 `libddsc.so.0` 和 `libddscxx.so.0` 优先解析到
  `/unitree/opt/lib`，而开发 SDK 位于 `/opt/unitree_robotics/lib`。原生编译前必须
  比较两套库的 SONAME/哈希，并在 CMake 中显式固定：

```text
-Dunitree_sdk2_DIR=/opt/unitree_robotics/lib/cmake/unitree_sdk2
```

- 当前没有 Docker、Podman、nerdctl、ctr 或 containerd。

结论：具备继续做原生配置/编译审计的基础，但尚不允许直接构建后运行 H2 探针。

## ROS 2 图发现执行追溯

已运行 `02_pc2_ros2_graph_read_only_audit.sh`。脚本显式固定：

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml
ROS_LOCALHOST_ONLY=0
ROS_DOMAIN_ID=0
ROS2CLI_DISABLE_DAEMON=1
```

已上传并执行的版本只枚举包、接口、node、topic、service 和 action；会发送 DDS
发现报文，但没有发布数据、调用服务或发送 action goal。执行版本 SHA256：

```text
78712811BAD0318CEB66F0AFB06D8D7B6F6CB0504B1386C972ECC9D19FBCA935
```

Humble 的 `ros2 action list` 没有 `--no-daemon`，该执行版本因此留下一个 ROS 2 CLI
图缓存 daemon。它不是机器人运动服务，也没有发送控制。当前本地 `02` 已省略 action
枚举，修正版 SHA256：

```text
339F2165C8DE93BBF4FB901F7C6B6A27323894C84F1252D31E6529FEAE1BA5A6
```

`03_pc2_topic_contract_read_only_audit.sh` 已完成。话题合同结论与当前 ABI 门槛见
`docs/Unitree_H2_PC2_话题合同与SDK2_ABI门槛记录_20260716.md`。阶段 04 已完成；下一轮
只运行加固后的 `05_pc2_build_run_hg_state_read_only.sh`，读取 LowState、secondary IMU、
BMS 和 MainBoard。仍然禁止 H2 HAL、`--zero-stop`、非零速度、状态动作、停止原厂服务或
修复 `shoujuan`/SLAM。PC2 后续文件统一放在 `/home/unitree/p2_unitreeH2/`。
