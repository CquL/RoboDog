# Unitree H2 r11 verified-vx A/B

## 实机依据

在同一台 H2、同一 FSM 601、同一 vendor CLI 下：

- `vx=0.20 m/s, duration=2 s`：RPC 返回 0，但实体不移动；
- `vx=0.50 m/s, duration=2 s`：RPC 返回 0，实体明确移动；
- 宇树未修改官方示例 `vx=0.50 m/s, duration=1 s`：实体明确移动。

r11 因此只把代码允许的 vx 绝对上限和 HAL 实机测试上限提高到已验证的
`0.50 m/s`。正式 `config/unitree_h2.yaml` 默认值仍为 `0.20 m/s`，不会因
升级包自动提高部署速度。

## 一键测试

保持常规运控 1、FSM 601、清空场地并由第二操作员持原厂遥控器：

```bash
bash scripts/09_pc2_h2_velocity_probe.sh vendor 0.50 0 0 1000
bash scripts/09_pc2_h2_velocity_probe.sh hal    0.50 0 0 1000
```

`09` 不再查询或强制 MotionSwitcher `form/name`。宇树 H2 高层 RPC 官方
例程没有该前置条件，两个 backend 自身都会检查 FSM 和 SDK 返回值。

## 结果边界

返回值 0 只代表 RPC 调用成功；必须另外记录实体是否移动。测试结束会显式
停止，紧急情况优先使用原厂遥控器进入阻尼。
