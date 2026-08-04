# Deep Robotics X30 工作区

绝影 X30 / X30 Pro 的厂商是云深处科技（Deep Robotics），不是优必选。本目录集中保存 X30 专属的文档、原厂二进制、ROS1/ROS2 传感器迁移包、抓包、基线、手册与归档。

以下两项按项目约定继续保留在 `D:\Desktop\RoboDog` 根目录，作为多机器人共享资产：

- `robot_hardware/`
- `jezetek_navigation_amd64.tar`

目录规划：

```text
docs/                 X30 研发和操作记录
manuals/              绝影/X30 原厂及项目手册
docker/               x30_livox_ros2_transfer 及其归档
factory/              原厂 launch、二进制和 plane_seg 取证包
sensors/              Livox 与 Yesense 源码/配置
transport/            105/106 跨主机只读传感器转发
data/                 点云、GridMap、步态和 UDP 抓包基线
migration/            旧迁移包与离线安装材料
logs/                 远端探测和构建输出
artifacts/            交付 tar.gz
tmp/                  X30 plane_seg、ELF 与 PDF 临时分析材料
tools/                整理与校验工具
```

`workspace_move_dry_run_20260715.json` 和 `workspace_move_manifest_20260715.json`
分别记录预演与实际移动结果。旧文档、抓包元数据和 fixture 中的旧绝对/相对路径作为
历史来源证据保留，不代表当前 Windows 工作区路径。

安全边界不变：当前 `x30_plane_seg_core` 仍是离线核心与合同验证，不代表已连接在线 GridMap、TCP `192.168.1.103:49999` 地形发送或运动控制。

新 X30 机器人部署入口：

- [Docker、105/106 传感器链与控制测试操作手册](X30_新机器人_Docker部署与控制测试操作手册_20260731.md)

所有用于上传和同型号机器人迁移的当前交付物统一放在：

```text
artifacts/
```

`docker/` 只保存镜像构建上下文，不再保存同名传输压缩包。

## Current X30 Control Entry

- [Unified X30 control test](docs/X30_统一控制测试入口_20260731.md)
- Executable: `robot_test_x30`
- Commands: `status`, `zero`, `gait`, and `move`

`factory/` is offline vendor evidence only. It is not copied into the
production Docker image or deployment archives.
