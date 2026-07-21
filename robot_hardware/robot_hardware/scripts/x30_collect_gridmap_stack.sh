#!/usr/bin/env bash

# Read-only inventory of the factory X30 terrain/gridmap data path.
# This script does not publish ROS messages, send network packets, or stop processes.

set -u

timestamp="$(date +%Y%m%d_%H%M%S)"
output_file="${1:-$HOME/x30_gridmap_stack_${timestamp}.log}"
mkdir -p "$(dirname "$output_file")"
exec > >(tee "$output_file") 2>&1

section()
{
    printf '\n===== %s =====\n' "$1"
}

run()
{
    printf '\n$'
    printf ' %q' "$@"
    printf '\n'
    "$@" || true
}

source_if_present()
{
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        # shellcheck disable=SC1090
        source "$setup_file"
        echo "sourced: $setup_file"
    fi
}

section "Safety"
echo "Read-only collection: no ROS publish, no network transmission, no process stop."
echo "Output: $output_file"

section "ROS environment"
source_if_present /opt/ros/noetic/setup.bash
source_if_present /home/ysc/jy_cog/drivers/setup.bash
source_if_present /home/ysc/jy_cog/system/devel/setup.bash
run date --iso-8601=seconds
run hostname
run ip -brief address
run ip route get 192.168.1.103
printf 'ROS_MASTER_URI=%s\n' "${ROS_MASTER_URI-}"
printf 'ROS_IP=%s\n' "${ROS_IP-}"
printf 'ROS_HOSTNAME=%s\n' "${ROS_HOSTNAME-}"

if ! ip -4 -o address show 2>/dev/null | grep -q '192\.168\.1\.105/'; then
    echo "WARNING: this host does not own 192.168.1.105."
    echo "The X30 105 launch starts gridmap_port; the 106 launch does not."
    echo "Use this result only as a 106-side inventory, then repeat on host 192.168.1.105."
fi

section "Candidate ROS nodes"
nodes="$(rosnode list 2>/dev/null | grep -Ei 'grid.?map|height.?map|vmap|fast.?lio|localization|terrain|obstacle|udp_sender|udp_receiver|app_port|pcl_concatenate' || true)"
if [[ -z "$nodes" ]]; then
    echo "No candidate terrain nodes found."
else
    printf '%s\n' "$nodes"
    while IFS= read -r node; do
        [[ -n "$node" ]] || continue
        section "Node $node"
        run timeout 8 rosnode info "$node"
    done <<< "$nodes"
fi

section "Candidate ROS topics"
topics="$(rostopic list 2>/dev/null | grep -Ei 'grid.?map|height.?map|vmap|terrain|obstacle|lidar_points|imu/data' || true)"
if [[ -z "$topics" ]]; then
    echo "No candidate terrain topics found."
else
    printf '%s\n' "$topics"
    while IFS= read -r topic; do
        [[ -n "$topic" ]] || continue
        section "Topic $topic"
        run timeout 5 rostopic type "$topic"
        run timeout 5 rostopic info "$topic"
    done <<< "$topics"
fi

section "Candidate ROS parameters"
rosparam list 2>/dev/null \
    | grep -Ei 'grid.?map|height.?map|vmap|fast.?lio|terrain|obstacle' \
    | head -n 500 \
    || true

section "Candidate processes"
run pgrep -af '(^|/)(gridmap_port|grid_map[^ ]*|vmap[^ ]*|fast[-_]?lio[^ ]*|localization_node|udp_sender|udp_receiver|app_port|pcl_concatenate|[^ ]*height[^ ]*map[^ ]*)([[:space:]]|$)'

gridmap_pids="$(pgrep -f '(^|/)gridmap_port([[:space:]]|$)' || true)"
if [[ -z "$gridmap_pids" ]]; then
    echo "No running gridmap_port process found."
    echo "If this is host 192.168.1.106, this matches message_transformer_106.launch."
    echo "Repeat the inventory on host 192.168.1.105 before concluding that the factory gridmap path is stopped."
else
    for pid in $gridmap_pids; do
        section "gridmap_port pid=$pid"
        executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
        run readlink -f "/proc/$pid/exe"
        run readlink -f "/proc/$pid/cwd"

        if [[ -r "/proc/$pid/cmdline" ]]; then
            printf 'cmdline: '
            tr '\0' ' ' < "/proc/$pid/cmdline"
            printf '\n'
        fi

        if [[ -r "/proc/$pid/environ" ]]; then
            tr '\0' '\n' < "/proc/$pid/environ" \
                | grep -E '^(LD_LIBRARY_PATH|ROS_|CMAKE_PREFIX_PATH)=' \
                || true
        fi

        if [[ -n "$executable" && -r "$executable" ]]; then
            run sha256sum "$executable"
            run ldd "$executable"
        fi

        echo "Selected mapped libraries:"
        grep -E 'grid_map_transformer|message_transformer|libpcl|libboost' "/proc/$pid/maps" 2>/dev/null \
            | awk '{print $NF}' \
            | sort -u \
            || true

        if command -v lsof >/dev/null 2>&1; then
            echo "Selected open files:"
            sudo lsof -p "$pid" 2>/dev/null \
                | grep -E 'grid_map_transformer|message_transformer|\.launch|\.yaml|\.yml|\.toml|\.json' \
                || true
            echo "TCP sockets:"
            sudo lsof -Pan -p "$pid" -iTCP 2>/dev/null || true
        fi
    done
fi

section "TCP port 49999 sockets"
sudo ss -tnap 2>/dev/null | grep -E '49999|gridmap_port' || true

section "Locate libgrid_map_transformer.so"
for root in /home/ysc /usr/local /opt; do
    [[ -d "$root" ]] || continue
    timeout 20 find "$root" -type f -name 'libgrid_map_transformer.so*' -print 2>/dev/null || true
done

section "Factory launch/config references"
for root in /home/ysc/jy_cog /home/ysc/jy_exe; do
    [[ -d "$root" ]] || continue
    timeout 30 grep -R -n -E \
        'gridmap_port|grid_map_transformer|49999|height_map_mode|obstacle_pointcloud' \
        "$root" \
        --include='*.launch' --include='*.xml' --include='*.yaml' --include='*.yml' \
        --include='*.toml' --include='*.json' --include='*.conf' --include='*.ini' \
        --include='*.sh' \
        2>/dev/null \
        | head -n 800 \
        || true
done

section "Done"
echo "Saved read-only gridmap diagnostic log to: $output_file"
