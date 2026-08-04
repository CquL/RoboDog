#!/usr/bin/env bash

# Stage 06C：H2 Loco getter-only RPC 兼容性门禁。
# 用途：在离线契约通过后，仅调用 GetFsmId/GetFsmMode/可用 FSM 等读取接口，
# 并把实机结果写入 stage06c.ok，供后续 06D 零速门禁校验。
# 通信边界：会向 /api/sport/request 发送只读 DDS RPC 请求，但不调用速度、
# StopMove、状态切换动作、低层命令或 ROS 控制话题。

# 任一未捕获错误、未定义变量或管道失败都立即终止，禁止生成假阳性门禁。
set -Eeuo pipefail
# 以脚本自身位置加载公共门禁库，避免依赖调用者当前工作目录。
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd -P)"
# shellcheck source=h2_pc2_hal_gate_common.sh
source "$SCRIPT_DIR/h2_pc2_hal_gate_common.sh"

# 建立隔离环境/日志并删除本阶段旧门禁，随后先跑无实机副作用的离线契约。
h2_prepare_gate 06c
h2_run_offline_contracts

h2_section "GETTER-ONLY LOCO RPC"
# getter 结果必须满足公共库中的返回码和 FSM 解析规则。
h2_run_getter_audit
# 确认临时探针进程退出，避免留下额外 DDS participant。
h2_check_no_probe_process

# 仅在上述步骤全部成功后原子写入 06C 门禁证据；明确记录未调用 setter。
h2_write_gate 06c \
  "fsm_id=$H2_FSM_ID" \
  "fsm_mode=$H2_FSM_MODE" \
  "mode_value_observation_only=1" \
  "control_setter_invoked=0" \
  "getter_only_rpc_ok=1"

# 稳定成功标记用于 PC2 日志和上层自动化验收。
printf 'H2_STAGE06C_GETTER_RPC_OK fsm_id=%s fsm_mode=%s\n' \
  "$H2_FSM_ID" "$H2_FSM_MODE"
