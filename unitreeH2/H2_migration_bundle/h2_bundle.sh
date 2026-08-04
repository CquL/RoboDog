#!/usr/bin/env bash

# Unitree H2 PC2 迁移包统一入口。
# 本脚本负责校验交付物、可选安装 Docker、导入镜像、持久启动双 IMU 桥，
# 以及执行只读 HAL/FSM 预检。它不会自动发送零速度、非零速度或状态动作。
set -Eeuo pipefail

BUNDLE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_CONFIG="${BUNDLE_ROOT}/config/deployment.env.example"
LOCAL_CONFIG="${H2_BUNDLE_CONFIG:-${BUNDLE_ROOT}/config/deployment.env}"

if [[ -f "${LOCAL_CONFIG}" ]]; then
  # deployment.env 是目标 PC2 的本地覆盖，不纳入只读交付清单。
  # shellcheck disable=SC1090
  source "${LOCAL_CONFIG}"
else
  # shellcheck disable=SC1090
  source "${DEFAULT_CONFIG}"
fi

: "${H2_IMAGE_TAG:=unitree_h2:amd64-runtime-candidate}"
: "${H2_CONTAINER_NAME:=unitree-h2-imu-bridge}"
: "${H2_DDS_INTERFACE:=eth0}"
: "${H2_DDS_DOMAIN:=0}"
: "${H2_ROS_DOMAIN_ID:=20}"
: "${H2_PC1_ADDRESS:=192.168.123.161}"
: "${H2_RESTART_POLICY:=unless-stopped}"

IMAGE_ARCHIVE="${BUNDLE_ROOT}/images/unitree_h2_amd64_runtime_candidate.tar.gz"
IMAGE_SHA="${IMAGE_ARCHIVE}.sha256"
DOCKER_ARCHIVE="${BUNDLE_ROOT}/docker_engine/docker_offline_jammy_amd64_24.0.7.tar.gz"
DOCKER_SHA="${DOCKER_ARCHIVE}.sha256"
SAFE_TEMPLATE="${BUNDLE_ROOT}/config/unitree_h2_container_safe.yaml"
MOTION_TEMPLATE="${BUNDLE_ROOT}/config/unitree_h2_container_motion.yaml"
STATE_DIR="${BUNDLE_ROOT}/state"
SAFE_RUNTIME="${STATE_DIR}/config/unitree_h2_container_safe.yaml"
MOTION_RUNTIME="${STATE_DIR}/config/unitree_h2_container_motion.yaml"
EXPECTED_IMAGE_ID="sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451"
EXPECTED_SCOPE="hal-native-hg-state-ros2-imu-candidate"
EXPECTED_SDK2_COMMIT="21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

run_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

docker_root() {
  run_root docker "$@"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

validate_settings() {
  [[ "${H2_DDS_INTERFACE}" =~ ^[[:alnum:]_.:-]+$ ]] ||
    die "invalid H2_DDS_INTERFACE=${H2_DDS_INTERFACE}"
  [[ "${H2_DDS_DOMAIN}" =~ ^[0-9]+$ ]] ||
    die "H2_DDS_DOMAIN must be an integer"
  ((H2_DDS_DOMAIN >= 0 && H2_DDS_DOMAIN <= 232)) ||
    die "H2_DDS_DOMAIN must be in [0, 232]"
  [[ "${H2_ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] ||
    die "H2_ROS_DOMAIN_ID must be an integer"
  ((H2_ROS_DOMAIN_ID >= 0 && H2_ROS_DOMAIN_ID <= 232)) ||
    die "H2_ROS_DOMAIN_ID must be in [0, 232]"
  [[ "${H2_CONTAINER_NAME}" =~ ^[[:alnum:]][[:alnum:]_.-]*$ ]] ||
    die "invalid H2_CONTAINER_NAME=${H2_CONTAINER_NAME}"
  [[ "${H2_PC1_ADDRESS}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] ||
    die "invalid H2_PC1_ADDRESS=${H2_PC1_ADDRESS}; use an IPv4 address"
  [[ "${H2_RESTART_POLICY}" =~ ^(no|always|unless-stopped|on-failure(:[0-9]+)?)$ ]] ||
    die "invalid H2_RESTART_POLICY=${H2_RESTART_POLICY}"
}

check_host() {
  validate_settings
  require_command uname
  require_command ip

  local machine
  machine="$(uname -m)"
  [[ "${machine}" == "x86_64" ]] ||
    die "this bundle requires linux/amd64 (x86_64), found ${machine}"

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    printf 'HOST_OS=%s VERSION_ID=%s ARCH=%s\n' \
      "${ID:-unknown}" "${VERSION_ID:-unknown}" "${machine}"
  fi

  ip link show dev "${H2_DDS_INTERFACE}" >/dev/null 2>&1 ||
    die "DDS interface does not exist: ${H2_DDS_INTERFACE}"
  [[ "$(cat "/sys/class/net/${H2_DDS_INTERFACE}/operstate" 2>/dev/null || true)" == "up" ]] ||
    die "DDS interface is not UP: ${H2_DDS_INTERFACE}"
  ip -4 addr show dev "${H2_DDS_INTERFACE}" | grep -Eq 'inet [0-9]' ||
    die "DDS interface has no IPv4 address: ${H2_DDS_INTERFACE}"
  ip -4 -br addr show dev "${H2_DDS_INTERFACE}" || true
  ip route || true
  df -h "${BUNDLE_ROOT}" /var/lib/docker 2>/dev/null || true

  if command -v ping >/dev/null 2>&1; then
    if ping -c 1 -W 1 "${H2_PC1_ADDRESS}" >/dev/null 2>&1; then
      printf 'H2_PC1_REACHABLE=%s\n' "${H2_PC1_ADDRESS}"
    else
      printf 'WARNING: H2 PC1 did not answer ping at %s\n' \
        "${H2_PC1_ADDRESS}" >&2
    fi
  fi

  local route_line route_dev
  route_line="$(ip route get "${H2_PC1_ADDRESS}" 2>/dev/null | head -n 1 || true)"
  route_dev="$(awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}' <<<"${route_line}")"
  printf 'H2_PC1_ROUTE=%s\n' "${route_line:-unresolved}"
  [[ "${route_dev}" == "${H2_DDS_INTERFACE}" ]] ||
    die "route to PC1 uses ${route_dev:-no-interface}, expected ${H2_DDS_INTERFACE}"

  if command -v docker >/dev/null 2>&1; then
    docker --version || true
  else
    echo 'DOCKER_NOT_INSTALLED'
  fi
  echo 'H2_HOST_CHECK_OK'
}

verify_one_archive() {
  local archive="$1"
  local checksum="$2"
  [[ -f "${archive}" ]] || die "missing archive: ${archive}"
  [[ -f "${checksum}" ]] || die "missing checksum: ${checksum}"
  (
    cd -- "$(dirname -- "${archive}")"
    sha256sum --check --strict "$(basename -- "${checksum}")"
  )
}

verify_bundle() {
  require_command sha256sum
  [[ -f "${BUNDLE_ROOT}/SHA256SUMS" ]] ||
    die "missing ${BUNDLE_ROOT}/SHA256SUMS"
  (
    cd -- "${BUNDLE_ROOT}"
    sha256sum --check --strict SHA256SUMS
  )
  echo 'H2_MIGRATION_BUNDLE_OK'
}

require_docker() {
  require_command docker
  run_root systemctl start docker
  docker_root info >/dev/null
}

install_docker_offline() {
  check_host
  require_command tar
  require_command sha256sum
  require_command dpkg
  require_command systemctl

  if command -v docker >/dev/null 2>&1; then
    run_root systemctl enable --now docker >/dev/null 2>&1 || true
    if docker_root info >/dev/null 2>&1; then
      echo 'Docker Engine is already installed and running; no packages changed.'
      docker_root version
      return 0
    fi
  fi

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
  fi
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] ||
    die "offline Docker packages are only accepted on Ubuntu 22.04 H2 PC2"
  [[ "$(dpkg --print-architecture)" == "amd64" ]] ||
    die "offline Docker packages require dpkg architecture amd64"

  verify_one_archive "${DOCKER_ARCHIVE}" "${DOCKER_SHA}"

  local temp_base temp_dir
  temp_base="$(cd -- "${TMPDIR:-/tmp}" && pwd -P)"
  temp_dir="$(mktemp -d "${temp_base}/h2-docker.XXXXXX")"
  [[ -d "${temp_dir}" && "${temp_dir}" == "${temp_base}"/h2-docker.* ]] ||
    die "refusing unsafe temporary directory: ${temp_dir}"

  (
    # 临时目录已经解析并限制在 TMPDIR 下，函数结束或报错时才递归清理该目录。
    trap 'rm -rf -- "${temp_dir}"' EXIT
    local package_dir="${temp_dir}/docker_offline_jammy_amd64_24.0.7"
    tar -xzf "${DOCKER_ARCHIVE}" -C "${temp_dir}"
    [[ -d "${package_dir}" ]] || die "unexpected Docker package layout"
    (
      cd -- "${package_dir}"
      sha256sum --check --strict SHA256SUMS
    )

    # 这 6 个包已在交付 H2 PC2 Ubuntu 22.04 上验收，但不是任意 Jammy
    # 安装的完整依赖闭包；依赖缺失时拒绝联网修复并输出 dpkg 审计结果。
    if ! run_root dpkg -i \
        "${package_dir}/bridge-utils_1.7-1ubuntu3_amd64.deb" \
        "${package_dir}/pigz_2.6-1_amd64.deb" \
        "${package_dir}/runc_1.1.12-0ubuntu2~22.04.1_amd64.deb" \
        "${package_dir}/containerd_1.7.12-0ubuntu2~22.04.1_amd64.deb" \
        "${package_dir}/ubuntu-fan_0.12.16_all.deb" \
        "${package_dir}/docker.io_24.0.7-0ubuntu2~22.04.1_amd64.deb"; then
      run_root dpkg --audit || true
      die "Docker package dependencies are incomplete on this target; no online repair was attempted"
    fi
  )

  run_root systemctl daemon-reload
  run_root systemctl enable --now containerd docker
  docker_root version
  echo 'H2_DOCKER_OFFLINE_INSTALL_OK'
}

verify_image_identity() {
  local image_id image_arch image_scope sdk_commit
  image_id="$(docker_root image inspect "${H2_IMAGE_TAG}" --format '{{.Id}}')"
  image_arch="$(docker_root image inspect "${H2_IMAGE_TAG}" --format '{{.Architecture}}')"
  image_scope="$(docker_root image inspect "${H2_IMAGE_TAG}" \
    --format '{{index .Config.Labels "io.robodog.h2.runtime.scope"}}')"
  sdk_commit="$(docker_root image inspect "${H2_IMAGE_TAG}" \
    --format '{{index .Config.Labels "io.robodog.unitree_sdk2.commit"}}')"

  printf 'IMAGE=%s\nID=%s\nARCH=%s\nSCOPE=%s\nSDK2=%s\n' \
    "${H2_IMAGE_TAG}" "${image_id}" "${image_arch}" \
    "${image_scope}" "${sdk_commit}"
  [[ "${image_id}" == "${EXPECTED_IMAGE_ID}" ]] || die "unexpected image ID"
  [[ "${image_arch}" == "amd64" ]] || die "unexpected image architecture"
  [[ "${image_scope}" == "${EXPECTED_SCOPE}" ]] || die "unexpected image scope"
  [[ "${sdk_commit}" == "${EXPECTED_SDK2_COMMIT}" ]] || die "unexpected SDK2 revision"
}

verify_image_artifacts() {
  docker_root run --rm \
    --network none \
    --read-only \
    --cap-drop ALL \
    --security-opt no-new-privileges \
    --entrypoint bash \
    "${H2_IMAGE_TAG}" --noprofile --norc -c '
set -e
test -x /opt/robodog/bin/robot_test_unitree_h2
test -x /opt/robodog/bin/unitree_h2_sensor_bridge
test -f /opt/robodog/lib/librobot_hardware.so
test -f /opt/robodog/config/unitree_h2_container_safe.yaml
test -f /opt/robodog/config/unitree_h2_container_motion.yaml
! ldd /opt/robodog/lib/librobot_hardware.so | grep -q "not found"
! ldd /opt/robodog/bin/unitree_h2_sensor_bridge | grep -q "not found"
echo H2_RUNTIME_ARTIFACTS_OK
'
}

load_image() {
  check_host
  require_command gzip
  require_command sha256sum
  require_docker
  verify_one_archive "${IMAGE_ARCHIVE}" "${IMAGE_SHA}"
  gzip -dc "${IMAGE_ARCHIVE}" | docker_root load
  verify_image_identity
  verify_image_artifacts
  echo 'H2_RUNTIME_IMAGE_LOAD_OK'
}

prepare_hal_configs() {
  [[ -f "${SAFE_TEMPLATE}" ]] || die "missing safe config template"
  [[ -f "${MOTION_TEMPLATE}" ]] || die "missing motion config template"
  mkdir -p -- "${STATE_DIR}/config"

  local source target
  for source in "${SAFE_TEMPLATE}" "${MOTION_TEMPLATE}"; do
    target="${STATE_DIR}/config/$(basename -- "${source}")"
    sed -E \
      -e "s|^network_interface_card_name:.*$|network_interface_card_name: \"${H2_DDS_INTERFACE}\"|" \
      -e "s|^dds_domain_id:.*$|dds_domain_id: ${H2_DDS_DOMAIN}|" \
      "${source}" >"${target}"
    grep -Fx "network_interface_card_name: \"${H2_DDS_INTERFACE}\"" \
      "${target}" >/dev/null
    grep -Fx "dds_domain_id: ${H2_DDS_DOMAIN}" "${target}" >/dev/null
  done
  chmod 0644 "${SAFE_RUNTIME}" "${MOTION_RUNTIME}"
}

container_running() {
  [[ "$(docker_root inspect --format '{{.State.Running}}' \
    "${H2_CONTAINER_NAME}" 2>/dev/null || true)" == "true" ]]
}

status() {
  require_docker
  if docker_root image inspect "${H2_IMAGE_TAG}" >/dev/null 2>&1; then
    verify_image_identity
  else
    printf 'IMAGE_NOT_LOADED=%s\n' "${H2_IMAGE_TAG}"
  fi

  if docker_root inspect "${H2_CONTAINER_NAME}" >/dev/null 2>&1; then
    docker_root ps -a --filter "name=^/${H2_CONTAINER_NAME}$" \
      --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}'
    docker_root inspect "${H2_CONTAINER_NAME}" --format \
      'container_image_id={{.Image}} network={{.HostConfig.NetworkMode}} readonly={{.HostConfig.ReadonlyRootfs}} restart={{.HostConfig.RestartPolicy.Name}}'
    docker_root logs --tail 30 "${H2_CONTAINER_NAME}" 2>&1 || true
  else
    printf 'CONTAINER_NOT_CREATED=%s\n' "${H2_CONTAINER_NAME}"
  fi
}

start_imu() {
  check_host
  require_docker
  run_root systemctl enable docker >/dev/null
  verify_image_identity
  prepare_hal_configs

  # 只有显式执行 start-imu 才替换本包同一镜像标签的容器；日常重启由 restart policy 处理。
  if docker_root inspect "${H2_CONTAINER_NAME}" >/dev/null 2>&1; then
    local existing_image_id existing_image_ref
    existing_image_id="$(docker_root inspect --format '{{.Image}}' "${H2_CONTAINER_NAME}")"
    existing_image_ref="$(docker_root inspect --format '{{.Config.Image}}' "${H2_CONTAINER_NAME}")"
    if [[ "${existing_image_id}" != "${EXPECTED_IMAGE_ID}" &&
          "${existing_image_ref}" != "${H2_IMAGE_TAG}" ]]; then
      die "refusing to delete unrelated container ${H2_CONTAINER_NAME}: image=${existing_image_ref} id=${existing_image_id}"
    fi
    printf 'REPLACING_CONTAINER=%s\n' "${H2_CONTAINER_NAME}"
    docker_root rm -f "${H2_CONTAINER_NAME}" >/dev/null
  fi

  docker_root run -d \
    --name "${H2_CONTAINER_NAME}" \
    --restart "${H2_RESTART_POLICY}" \
    --network host \
    --read-only \
    --cap-drop ALL \
    --security-opt no-new-privileges \
    --label io.robodog.h2.bundle=H2_migration_bundle \
    --tmpfs /tmp:rw,noexec,nosuid,size=64m \
    --volume "${SAFE_RUNTIME}:/opt/robodog/config/unitree_h2_container_safe.yaml:ro" \
    --volume "${MOTION_RUNTIME}:/opt/robodog/config/unitree_h2_container_motion.yaml:ro" \
    -e "H2_DDS_INTERFACE=${H2_DDS_INTERFACE}" \
    -e "H2_DDS_DOMAIN=${H2_DDS_DOMAIN}" \
    -e "ROS_DOMAIN_ID=${H2_ROS_DOMAIN_ID}" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ROS_LOG_DIR=/tmp/ros_logs \
    "${H2_IMAGE_TAG}" \
    /opt/robodog/bin/unitree_h2_sensor_bridge \
      --ros-args \
      --params-file \
      /opt/robodog/share/unitree_h2_sensor_bridge/config/unitree_h2_imu_bridge.yaml \
      -p "dds_interface:=${H2_DDS_INTERFACE}" \
      -p "dds_domain:=${H2_DDS_DOMAIN}" >/dev/null

  local ready=0
  for _ in $(seq 1 30); do
    if ! container_running; then
      break
    fi
    if docker_root logs "${H2_CONTAINER_NAME}" 2>&1 |
        grep -Fq 'H2_IMU_BRIDGE_READY'; then
      ready=1
      break
    fi
    sleep 0.5
  done

  if [[ "${ready}" -ne 1 ]]; then
    docker_root logs --tail 100 "${H2_CONTAINER_NAME}" 2>&1 || true
    die "IMU bridge did not reach H2_IMU_BRIDGE_READY"
  fi
  echo 'H2_IMU_BRIDGE_START_OK'
  status
}

verify_imu() {
  require_docker
  container_running || die "container is not running: ${H2_CONTAINER_NAME}"
  docker_root exec "${H2_CONTAINER_NAME}" bash --noprofile --norc -lc '
set -Eeuo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
ros2 topic list -t | grep -E "^/h2/imu/(pelvis|torso) "
timeout 15s ros2 topic echo \
  /h2/imu/pelvis sensor_msgs/msg/Imu --once --no-arr
timeout 15s ros2 topic echo \
  /h2/imu/torso sensor_msgs/msg/Imu --once --no-arr
echo H2_IMU_LIVE_SAMPLES_OK
'
}

preflight_control() {
  require_docker
  container_running || die "container is not running: ${H2_CONTAINER_NAME}"

  local contenders output
  contenders="$(docker_root exec "${H2_CONTAINER_NAME}" \
    bash --noprofile --norc -c \
    "pgrep -af '[r]obot_test_unitree_h2|[h]2_runtime|unitree_h2.*[c]ontroller' || true")"
  if [[ -n "${contenders}" ]]; then
    printf '%s\n' "${contenders}"
    die "another H2 test/controller process is running in the container"
  fi

  if ! output="$(docker_root exec "${H2_CONTAINER_NAME}" \
      /opt/robodog/bin/robot_test_unitree_h2 \
      --config /opt/robodog/config/unitree_h2_container_safe.yaml \
      --getter-audit 2>&1)"; then
    printf '%s\n' "${output}"
    die "read-only H2 getter audit failed"
  fi
  printf '%s\n' "${output}"
  grep -Fq 'H2_GETTER_ONLY_RPC_OK' <<<"${output}" ||
    die "getter success marker missing"
  grep -Eq 'fsm_id=601|FSM ID=601' <<<"${output}" ||
    die "FSM is not confirmed as 601; do not run write tests"
  echo 'H2_CONTROL_READ_ONLY_PREFLIGHT_OK fsm_id=601'
}

open_shell() {
  require_docker
  container_running || die "container is not running: ${H2_CONTAINER_NAME}"
  if [[ "${EUID}" -eq 0 ]]; then
    exec docker exec -it "${H2_CONTAINER_NAME}" bash --noprofile --norc
  else
    exec sudo docker exec -it "${H2_CONTAINER_NAME}" bash --noprofile --norc
  fi
}

stop_imu() {
  require_docker
  docker_root stop "${H2_CONTAINER_NAME}"
  echo 'H2_IMU_BRIDGE_STOPPED'
}

show_config() {
  printf 'CONFIG_SOURCE=%s\n' "$([[ -f "${LOCAL_CONFIG}" ]] && echo "${LOCAL_CONFIG}" || echo "${DEFAULT_CONFIG}")"
  printf 'H2_IMAGE_TAG=%s\n' "${H2_IMAGE_TAG}"
  printf 'H2_CONTAINER_NAME=%s\n' "${H2_CONTAINER_NAME}"
  printf 'H2_DDS_INTERFACE=%s\n' "${H2_DDS_INTERFACE}"
  printf 'H2_DDS_DOMAIN=%s\n' "${H2_DDS_DOMAIN}"
  printf 'H2_ROS_DOMAIN_ID=%s\n' "${H2_ROS_DOMAIN_ID}"
  printf 'H2_PC1_ADDRESS=%s\n' "${H2_PC1_ADDRESS}"
  printf 'H2_RESTART_POLICY=%s\n' "${H2_RESTART_POLICY}"
}

usage() {
  cat <<'EOF'
Usage: bash h2_bundle.sh <command>

Commands:
  verify          Verify the complete migration bundle SHA256 manifest.
  show-config     Show active deployment settings.
  check-host      Validate H2 PC2 architecture, DDS interface and basic network.
  install-docker  Install Docker 24 from the optional Jammy amd64 offline package.
  load-image      Verify, load and inspect the H2 runtime image.
  start-imu       Recreate and persistently start the read-only dual-IMU bridge.
  status          Inspect image/container identity and recent bridge logs.
  verify-imu      Read one live sample from each ROS 2 IMU topic.
  preflight       Run the read-only HAL getter audit and require FSM=601.
  shell           Open a clean interactive shell in the running bridge container.
  stop-imu        Stop the bridge container without deleting it.

No command in this script sends zero/nonzero velocity or state actions.
See README.md for the separately gated control commands.
EOF
}

command_name="${1:-help}"
case "${command_name}" in
  verify) verify_bundle ;;
  show-config) show_config ;;
  check-host) check_host ;;
  install-docker) install_docker_offline ;;
  load-image) load_image ;;
  start-imu) start_imu ;;
  status) status ;;
  verify-imu) verify_imu ;;
  preflight) preflight_control ;;
  shell) open_shell ;;
  stop-imu) stop_imu ;;
  help|-h|--help) usage ;;
  *) usage >&2; die "unknown command: ${command_name}" ;;
esac
