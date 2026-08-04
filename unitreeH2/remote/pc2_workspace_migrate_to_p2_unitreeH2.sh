#!/usr/bin/env bash

# 一次性、显式限定范围的 H2 PC2 审计工件迁移脚本。
# 用途：把早期误放在 /home/unitree 根目录的指定脚本和日志，移动到
# /home/unitree/p2_unitreeH2/{scripts,logs}，并输出清单与 SHA256。
# 写入边界：只创建项目目录、移动下面两个白名单中的文件；不触碰厂商、
# 系统、隐藏文件、NoMachine、ROS 工作区或其他用户文件，也不与机器人通信。

# 未定义变量立即报错；单个源文件缺失/冲突由 migrate_one 记录后继续。
set -u

# 目标工作区及其脚本/日志子目录。
BASE=/home/unitree/p2_unitreeH2
SCRIPT_DIR="${BASE}/scripts"
LOG_DIR="${BASE}/logs"

# 只创建本项目约定的五个子目录。
mkdir -p "${SCRIPT_DIR}" "${LOG_DIR}" "${BASE}/src" "${BASE}/build" "${BASE}/config"

# 安全迁移一个白名单文件：
# - 源不存在：记录 SKIP；
# - 目标已存在且相同：保留两者并记录；
# - 目标冲突：绝不覆盖；
# - 仅目标不存在时执行 mv。
migrate_one() {
  local source="$1"
  local destination_dir="$2"
  local destination="${destination_dir}/$(basename "$source")"

  if [[ ! -e "$source" ]]; then
    printf 'SKIP_MISSING %s\n' "$source"
    return 0
  fi

  if [[ -e "$destination" ]]; then
    if cmp -s -- "$source" "$destination"; then
      printf 'SKIP_IDENTICAL_DESTINATION_EXISTS source=%s destination=%s\n' \
        "$source" "$destination"
    else
      printf 'CONFLICT_NOT_MOVED source=%s destination=%s\n' \
        "$source" "$destination" >&2
    fi
    return 0
  fi

  mv -- "$source" "$destination"
  printf 'MOVED source=%s destination=%s\n' "$source" "$destination"
}

# 白名单 1：四个早期 PC2 只读审计脚本。
for name in \
  00_pc2_read_only_inventory.sh \
  01_pc2_dds_sdk_read_only_audit.sh \
  02_pc2_ros2_graph_read_only_audit.sh \
  03_pc2_topic_contract_read_only_audit.sh; do
  migrate_one "/home/unitree/${name}" "${SCRIPT_DIR}"
done

# 白名单 2：四个对应的 2026-07-16 原始日志。
for name in \
  h2_pc2_inventory_20260716.log \
  h2_pc2_dds_sdk_audit_20260716.log \
  h2_pc2_ros2_graph_20260716.log \
  h2_pc2_topic_contract_20260716.log; do
  migrate_one "/home/unitree/${name}" "${LOG_DIR}"
done

# 输出迁移后的相对路径和大小，便于人工确认没有越界文件。
printf '\nWORKSPACE=%s\n' "$BASE"
find "$BASE" -maxdepth 2 -type f -printf '%P\t%s bytes\n' | sort

printf '\nSHA256\n'
# 对脚本和日志生成内容哈希，作为后续复制/归档完整性依据。
find "${SCRIPT_DIR}" "${LOG_DIR}" -maxdepth 1 -type f -print0 \
  | sort -z \
  | xargs -0 -r sha256sum

# 结束标记仅表示迁移函数执行完毕；冲突项仍应查看前面的日志。
echo "P2_UNITREE_H2_WORKSPACE_MIGRATION_OK"
