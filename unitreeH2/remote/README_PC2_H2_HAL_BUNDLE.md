# Unitree H2 PC2 原生 HAL 候选包

该包只用于 H2 EDU 高层 LocoClient 的分级实机验证，固定运行位置为
/home/unitree/p2_unitreeH2/build/<release-name>/。

它从已离线测试的 unitree_h2:amd64-live-test-candidate 镜像导出，携带同一
SDK2 快照的私有 CycloneDDS 动态库；不会把本地 H2 头文件与 PC2 旧
/opt/unitree_robotics/lib/libunitree_sdk2.a 混合链接。PC2 不需要安装 Docker。

## 安全边界

- Stage 06C 调用 getter RPC，会发布 /api/sport/request，但不调用控制 setter。
- Stage 06D 写入零速度与 StopMove()；后者不是硬件急停。
- Stage 06E 一次只发送一个未标定轴的 20 Hz 有界速度 stream，不自动循环方向；r8
  允许逐次指定速度和 stream 时长，但不能越过编译期安全上限。
- 不调用 Start、StandUp、Damp、Squat、Sit，不发 rt/lowcmd，不停止原厂 sport
  或 PC2 服务。
- Stage 06B 生产型传感器状态源仍未完成；本包不包含导航算法或正式 h2_runtime。

## Windows 上传与 PC2 解包

在 Windows PowerShell 中上传 tar.gz 和 tar.gz.sha256，文件名以实际产物为准：

    scp "D:/Desktop/RoboDog/unitreeH2/runtime_bundle/<bundle>.tar.gz" "D:/Desktop/RoboDog/unitreeH2/runtime_bundle/<bundle>.tar.gz.sha256" unitree@192.168.123.162:/home/unitree/p2_unitreeH2/build/

随后在 PC2 的 unitree 用户终端中执行：

    cd /home/unitree/p2_unitreeH2/build
    sha256sum --check --strict <bundle>.tar.gz.sha256
    tar -xzf <bundle>.tar.gz
    cd <bundle>
    bash scripts/06_pc2_h2_getters_rpc_gate.sh

06C 会自动检查：主机、用户、eth0=192.168.123.162/24、PC1 连通性、原厂
CycloneDDS XML 哈希、包内 manifest、文件所有者、符号链接、ldd 私有依赖、
无 ROS 动态库依赖、三个离线合同测试以及当前/可用 FSM 601。失败时不自动切换
FSM。

只有 06C 成功且现场五项物理门禁已落实后，才在交互式终端执行：

    bash scripts/07_pc2_h2_zero_stop_gate.sh

只有观察员确认零速度没有异常运动、06D 成功，并再次确认防坠支架、四脚轮锁定、
净空、第二操作员持原厂遥控器和高层停止/阻尼流程后，才执行首个轴。r8 默认是
`0.05 m/s + 1000 ms + 20 Hz`，即最多 20 次有界速度 RPC：

    bash scripts/08_pc2_h2_single_axis_motion_gate.sh x-positive

如需指定本轮方向观察 profile，使用运行时参数，不手改脚本：

    bash scripts/08_pc2_h2_single_axis_motion_gate.sh \
      x-positive \
      --linear-speed 0.08 \
      --yaw-speed 0.08 \
      --stream-ms 1000

参数合同：线速度 `0.01..0.10 m/s`、角速度 `0.01..0.15 rad/s`、stream
`250..1000 ms` 且以 `50 ms` 为步进；速度步进为 `0.001`。发送频率固定 `20 Hz`、
周期 `50 ms`、最大允许发送间隔 `100 ms`、watchdog `150 ms`、单条厂商 duration
`0.300 s`。预期 RPC 数量为 `stream_ms / 50`，实际 `rpc_count` 必须与其相等，实测
最大发送间隔 `max_observed_send_gap_ms` 也不得超过 `100 ms`。

每次仍只允许一个轴，最终人工授权短语为
`RUN_STREAM_<axis>_L<linear>_Y<yaw>_S<stream_ms>_H<command_hz>`。每次实机尝试的
观察值都会先写入单次日志、`h2_pc2_axis_observations.tsv` 和新的
`h2_pc2_axis_stream_observations.tsv`；只有 C++ 输出
`H2_LIVE_SINGLE_AXIS_STREAM_RPC_OK`、RPC 数量和发送间隔合同通过、明确观察到物理
动作且随后输入安全确认短语 `BOUNDED_STREAM_OBSERVED_SAFE`，才允许写入
`stage06e.ok`。

如果没有看到动作，在 `Record the observed physical direction:` 提示中输入
`NO_PHYSICAL_MOTION_OBSERVED`（也接受 `NO_VISIBLE_MOTION`）。脚本会保留本次观察记录，
输出 `H2_STAGE06E_NO_MOTION_OBSERVED ... stage06e_gate_written=0`，随后以非零状态退出；
它不会询问 `BOUNDED_STREAM_OBSERVED_SAFE`、不会写 `stage06e.ok`，也不会输出 Stage 06E
物理成功标记。不要把“RPC 返回 0”记录成“机器人已经移动”。

`0.08 m/s + 1000 ms` 的理论速度积分约为 `80 mm`，这只是命令积分，不等于机器人
一定产生该实际位移，也不能替代现场观察、发送间隔门禁和停止门禁。

首轮不能直接写 forward、back、left 或 right。必须先把 x-positive 的实物观察
方向写入日志，再决定后续单轴测试。

所有运行日志自动追加到 /home/unitree/p2_unitreeH2/logs/。

成功门禁绑定同一 bundle manifest 与 PC2 boot ID；重启或更换 bundle 后必须从
06C 重新开始。不能只把 r8 脚本复制到 r7，也不能复用 r7 的 gate 文件；r8 的 CLI、
脚本和 manifest 必须作为完整 bundle 一起部署。
