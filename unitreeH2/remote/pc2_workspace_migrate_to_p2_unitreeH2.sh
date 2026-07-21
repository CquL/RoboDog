#!/usr/bin/env bash

# One-time, explicitly scoped migration of RoboDog H2 audit artifacts from
# /home/unitree into /home/unitree/p2_unitreeH2. It does not touch any vendor,
# system, hidden, NoMachine, ROS workspace or unrelated user file.

set -u

BASE=/home/unitree/p2_unitreeH2
SCRIPT_DIR="${BASE}/scripts"
LOG_DIR="${BASE}/logs"

mkdir -p "${SCRIPT_DIR}" "${LOG_DIR}" "${BASE}/src" "${BASE}/build" "${BASE}/config"

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

for name in \
  00_pc2_read_only_inventory.sh \
  01_pc2_dds_sdk_read_only_audit.sh \
  02_pc2_ros2_graph_read_only_audit.sh \
  03_pc2_topic_contract_read_only_audit.sh; do
  migrate_one "/home/unitree/${name}" "${SCRIPT_DIR}"
done

for name in \
  h2_pc2_inventory_20260716.log \
  h2_pc2_dds_sdk_audit_20260716.log \
  h2_pc2_ros2_graph_20260716.log \
  h2_pc2_topic_contract_20260716.log; do
  migrate_one "/home/unitree/${name}" "${LOG_DIR}"
done

printf '\nWORKSPACE=%s\n' "$BASE"
find "$BASE" -maxdepth 2 -type f -printf '%P\t%s bytes\n' | sort

printf '\nSHA256\n'
find "${SCRIPT_DIR}" "${LOG_DIR}" -maxdepth 1 -type f -print0 \
  | sort -z \
  | xargs -0 -r sha256sum

echo "P2_UNITREE_H2_WORKSPACE_MIGRATION_OK"
