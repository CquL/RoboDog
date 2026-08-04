#!/usr/bin/env bash

# Stage 06D：受保护的零速度 + StopMove 写 RPC 门禁。
# 用途：在 06C getter 门禁、物理防护和人工确认均通过后，验证抽象层的
# 高层停止链路，并把观察结果写入 stage06d.ok。
# 安全边界：StopMove 是 SDK2 的高层停止 RPC，不是硬件急停；测试还会另行
# 发送显式零速度。本阶段不发送非零速度，但确实会写控制 RPC，因此必须有原装遥控器操作员。

# 任何命令/管道/未定义变量错误都立即退出，不生成后续门禁文件。
set -Eeuo pipefail
# 从脚本目录加载公共门禁实现，不依赖当前目录。
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd -P)"
# shellcheck source=h2_pc2_hal_gate_common.sh
source "$SCRIPT_DIR/h2_pc2_hal_gate_common.sh"

# 临时捕获文件用于同时保存命令输出和验证成功标记。
zero_capture=""
# 无论正常退出、拒绝还是异常，都删除临时日志；正式阶段日志由公共库 tee 保存。
cleanup_zero_capture() {
  if [[ -n "$zero_capture" ]]; then
    rm -f -- "$zero_capture"
  fi
}
trap cleanup_zero_capture EXIT

# 准备隔离环境并强制验证父阶段 06C 的关键事实。
h2_prepare_gate 06d
h2_require_gate 06c \
  "fsm_id=601" \
  "mode_value_observation_only=1" \
  "control_setter_invoked=0" \
  "getter_only_rpc_ok=1"
# 保存父门禁哈希，06D 证据可追溯到本次实际使用的 06C 文件。
stage06c_gate_sha256="$(
  sha256sum "$H2_STATE_DIR/stage06c.ok" | awk '{print $1}'
)"
# 实机写入前再次执行离线构造/工厂/计划契约。
h2_run_offline_contracts

# h2_prepare_gate 会把 stdout 重定向给 tee，而 FD 3 保留原始终端；
# 两者都必须是交互 TTY，禁止在无人值守管道中绕过物理确认。
[[ -t 0 && -t 3 ]] || h2_die STAGE06D_REQUIRES_INTERACTIVE_TTY 40
h2_section "PHYSICAL ZERO-STOP CHECKLIST"
printf '%s\n' \
  'Required: official fall-arrest/protective stand installed.' \
  'Required: all four stand caster wheels locked.' \
  'Required: clear test area and no person in reach.' \
  'Required: second operator holding the original remote controller.' \
  'Required: delivered-H2 high-level stop/damping procedure confirmed.'
# 确认词必须逐字匹配；这是一道人工物理门禁，不是可执行命令。
read -r -p 'Type ZERO_STOP_PHYSICAL_GATES_CONFIRMED exactly: ' confirmation
[[ "$confirmation" == ZERO_STOP_PHYSICAL_GATES_CONFIRMED ]] ||
  h2_die PHYSICAL_CONFIRMATION_REJECTED 40

h2_section "PRE-WRITE GETTER RECHECK"
# 临近写入时重新确认 FSM，防止 06C 之后机器人状态已变化。
h2_run_getter_audit

h2_section "ZERO VELOCITY AND STOPMOVE"
# 在受控超时内执行统一 HAL 测试入口，并把二进制输出复制到私有临时文件。
zero_capture="$H2_STATE_DIR/.stage06d_zero_stop.$BASHPID.log"
: >"$zero_capture"
chmod 0600 "$zero_capture"
set +e
h2_isolated timeout --foreground --signal=INT --kill-after=3s 25s \
  "$H2_BIN_DIR/robot_test_unitree_h2" \
  --config "$H2_CONFIG" --zero-stop --execute 2>&1 | tee "$zero_capture"
zero_status=("${PIPESTATUS[@]}")
set -e
zero_rc="${zero_status[0]}"
# 分别检查 tee 与测试程序返回码，再验证稳定成功标记，三者缺一不可。
[[ "${zero_status[1]}" -eq 0 ]] || h2_die ZERO_STOP_CAPTURE_TEE_FAILED 41
zero_output="$(<"$zero_capture")"
rm -f -- "$zero_capture"
zero_capture=""
printf 'COMMAND_RC[zero_stop]=%s\n' "$zero_rc"
[[ "$zero_rc" -eq 0 ]] || h2_die "ZERO_STOP_COMMAND_FAILED_RC=$zero_rc" 41
grep -F 'H2_ZERO_STOP_RPC_OK' <<<"$zero_output" >/dev/null ||
  h2_die ZERO_STOP_SUCCESS_MARKER_MISSING 41

h2_section "POST-WRITE GETTER RECHECK"
# 写入后再次读取 FSM，并确认没有残留探针进程。
h2_run_getter_audit
h2_check_no_probe_process

# 现场观察者确认机器人没有意外运动；拒绝任何模糊或自动填入的回答。
read -r -p 'Observer: type NO_UNEXPECTED_MOTION_OBSERVED exactly: ' observation
[[ "$observation" == NO_UNEXPECTED_MOTION_OBSERVED ]] ||
  h2_die ZERO_STOP_OBSERVATION_NOT_ACCEPTED 42

# 仅在命令、前后 getter 和人工观察全部通过后写入 06D 门禁。
h2_write_gate 06d \
  "fsm_id=$H2_FSM_ID" \
  "fsm_mode=$H2_FSM_MODE" \
  "observer=NO_UNEXPECTED_MOTION_OBSERVED" \
  "nonzero_velocity_invoked=0" \
  "zero_stop_rpc_ok=1" \
  "parent_stage06c_sha256=$stage06c_gate_sha256"

# 稳定日志标记供后续单轴运动阶段识别。
printf 'H2_STAGE06D_ZERO_STOP_OBSERVED_OK fsm_id=%s\n' "$H2_FSM_ID"
