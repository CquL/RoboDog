# H2 官方 SDK 原版高层 RPC 基准

日期：2026-07-22

## 用途

该目录中的 ELF 二进制直接由宇树官方、未修改的源文件编译：

`unitreeH2/vendor/unitree_sdk2/example/h2/high_level/h2_loco_client_example.cpp`

它用于在 H2 PC2 上绕过 RoboDog HAL、r9 vendor 包装器和 MotionSwitcher
门禁，原样复现宇树文档中的高层速度命令。

这不是新的控制实现，也不证明实机已经运动。

## 来源与哈希

- SDK2 commit：`21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`
- 官方源文件 SHA256：
  `a91791a2511e9c7927d321aaeb2c404c81a5e306701e6cc1d546e62421b3a0f5`
- `libunitree_sdk2.a` SHA256：
  `08402aea74150dfbfc3fbfded4ca746916a8d892b54d2bade0cbf392a3be4029`
- 基准二进制 SHA256：
  `0de8dce74bbf64d95816d0beb5d19ded01f1991f6041449a7e141ced206759c6`

编译平台为 Ubuntu 22.04 x86_64。使用 r9 release 的 `libddscxx.so.0`
和 `libddsc.so.0` 完成了离线 `ldd` 验证，无缺失依赖。

## PC2 运行边界

- 高层 RPC 例程无需进入底层调试模式。
- 先使用常规运控 1，并确认 FSM 601。
- 原版程序不检查 RPC 返回码；`Done!` 和退出码 0 不等于实机已经运动。
- 物理动作、DDS request/response 和人工观察必须分别记录。
- 不要在一次命令中同时传入 `--start` 和 `--set_velocity`；程序使用
  `std::map`，处理次序不是命令行输入次序。
