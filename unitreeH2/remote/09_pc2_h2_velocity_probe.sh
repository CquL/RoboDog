#!/usr/bin/env bash

# H2 简化速度实机探针，统一通过 RoboDog HAL 测试入口。
# 数据链：命令行 vx/vy/omega/duration -> RobotFactory ->
# RobotHardwareInterface（动态对象为 UnitreeH2）-> writeRobotVelocityCommand -> SDK2 LocoClient。
# 安全边界：这是会让机器人运动的脚本；不保留直连厂商后端，所有速度和
# FSM/看门狗/停止检查必须经过抽象层。调用前仍需完成现场物理安全条件。

# 参数、管道或未定义变量错误立即终止；测试程序返回码会原样传给调用者。
set -Eeuo pipefail

# 接口固定为四个位置参数，避免缺省值导致非预期速度或持续时间。
if [[ $# -ne 4 ]]; then
  printf 'Usage: %s <vx> <vy> <omega> <duration_ms>\n' "$0" >&2
  printf 'Example: %s 0.50 0 0 1000\n' "$0" >&2
  exit 64
fi

# 用户输入仅转交统一测试二进制，数值范围由二进制和 YAML 双重校验。
vx="$1"
vy="$2"
omega="$3"
duration_ms="$4"

# release_dir 从脚本所在发布包推导，保证 bin/config/lib 来自同一版本。
script_dir="$(cd -- "$(dirname -- "$0")" && pwd -P)"
release_dir="$(cd -- "$script_dir/.." && pwd -P)"
binary="$release_dir/bin/robot_test_unitree_h2"
config="$release_dir/config/unitree_h2_live.yaml"
# 优先加载当前发布包随附库，避免误链接主机上的另一版 DDS/HAL。
export LD_LIBRARY_PATH="$release_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# 每次运行使用唯一日志名；H2_LOG_DIR 只允许改变日志目录，不改变控制参数。
stamp="$(date +%Y%m%d_%H%M%S)_$$"
log_dir="${H2_LOG_DIR:-/home/unitree/p2_unitreeH2/logs}"
mkdir -p "$log_dir"
log="$log_dir/h2_pc2_unified_velocity_${stamp}.log"

printf 'H2_UNIFIED_PROBE vx=%s vy=%s omega=%s duration_ms=%s log=%s\n' \
  "$vx" "$vy" "$omega" "$duration_ms" "$log"

# tee 同时保留现场输出和持久日志；分别捕获二进制与 tee 的返回码。
set +e
"$binary" --config "$config" --velocity \
  --vx "$vx" --vy "$vy" --omega "$omega" \
  --duration-ms "$duration_ms" --execute 2>&1 | tee "$log"
status=("${PIPESTATUS[@]}")
set -e
# 日志写入失败返回 70；否则原样返回 HAL 测试程序结果。
[[ "${status[1]}" -eq 0 ]] || exit 70
printf 'H2_UNIFIED_PROBE_RESULT rc=%s log=%s\n' "${status[0]}" "$log"
exit "${status[0]}"
