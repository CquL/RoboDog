#!/usr/bin/env bash

# 同步记录厂家 ROS1 地形链及其 TCP 输出。
# 本脚本只读：不发布 ROS 消息，也不发送机器人数据。

set -u

# 场景标签和测量值仅作为 metadata。
# 脚本不会用它们配置地形处理或发送机器人命令。
duration_s="${1:-15}"
label="${2:-mode3_known_step}"
output_root="${3:-$HOME}"
motion_ip="${X30_MOTION_IP:-192.168.1.103}"
gridmap_port="${X30_GRIDMAP_PORT:-49999}"

# 最小时长保证 ROS Topic 和 TCP 输出有足够时间生成有效同步样本；
# 最大时长限制磁盘占用。
if ! [[ "$duration_s" =~ ^[0-9]+$ ]] || (( duration_s < 5 || duration_s > 120 )); then
    echo "duration must be an integer from 5 to 120 seconds" >&2
    exit 2
fi

if ! [[ "$label" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "label may contain only letters, digits, dot, underscore, and hyphen" >&2
    exit 2
fi

for command_name in rosbag rostopic tcpdump timeout sha256sum; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "required command not found: $command_name" >&2
        exit 3
    fi
done

# source 当前机器人镜像中实际存在的厂家 workspace。
source_if_present()
{
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        # shellcheck disable=SC1090
        source "$setup_file"
    fi
}

source_if_present /opt/ros/noetic/setup.bash
source_if_present /home/ysc/jy_cog/drivers/setup.bash
source_if_present /home/ysc/jy_cog/system/devel/setup.bash

# TCP 数据流由 105 单播发送，因此同步网络抓包必须在感知主机执行，
# 不能在 106 开发主机执行。
if ! ip -4 -o address show 2>/dev/null | grep -q '192\.168\.1\.105/'; then
    echo "Run this script on the X30 perception host 192.168.1.105." >&2
    exit 4
fi

# 若三项厂家地形链证据不可见，则立即失败。
for topic in \
    /height_map_mode_state \
    /deeprobotics_local_height_map_mid360/height_map \
    /plane_seg/quadrangels; do
    if ! rostopic type "$topic" >/dev/null 2>&1; then
        echo "required factory topic is unavailable: $topic" >&2
        exit 5
    fi
done

interface="$(ip route get "$motion_ip" 2>/dev/null | awk '{for (i=1; i<=NF; ++i) if ($i=="dev") {print $(i+1); exit}}')"
if [[ -z "$interface" ]]; then
    echo "could not determine the interface used to reach $motion_ip" >&2
    exit 6
fi

# 抓取前后各读取一次地形模式状态，
# 用于发现采样期间处理模式发生变化的基线。
read_mode_state()
{
    timeout 4 rostopic echo -n 1 /height_map_mode_state 2>/dev/null \
        | awk '/data:/ {print $2; exit}'
}

mode_before="$(read_mode_state)"
if [[ -z "$mode_before" ]]; then
    echo "could not read /height_map_mode_state" >&2
    exit 7
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
stem="${label}_${timestamp}"
bag_dir="$output_root/x30_stair_baselines"
pcap_dir="$output_root/x30_gridmap_captures"
bag_file="$bag_dir/${stem}.bag"
info_file="$bag_dir/${stem}.info.yaml"
metadata_file="$bag_dir/${stem}.metadata.txt"
pcap_file="$pcap_dir/${timestamp}_${label}.pcap"
packet_file="$pcap_dir/${timestamp}_${label}.packets.txt"
hex_file="$pcap_dir/${timestamp}_${label}.hex.txt"

mkdir -p "$bag_dir" "$pcap_dir"

# 记录从传感器、里程计输入到高度图和平面分割的完整链路，
# 并记录解释样本所需的控制状态上下文。
topics=(
    /livox/lidar
    /imu/data
    /leg_odom
    /tf
    /tf_static
    /step_z_max
    /height_map_mode
    /height_map_mode_state
    /deeprobotics_local_height_map_mid360/height_map
    /plane_seg/received_cloud
    /plane_seg/voxel_cloud
    /plane_seg/hull_cloud
    /plane_seg/quadrangels
    /plane_seg/look_pose
    /plane_seg/hull_markers
    /plane_seg/hull_marker_array
    /test_rqy
    /control_mode
    /robot_basic_state
    /robot_gait_state
)

# 直接调用 recorder 二进制，避开 ROS Noetic Python wrapper 的
# SIGINT handler 缺陷；该缺陷会在有效 bag 关闭后打印 traceback。
rosbag_record_binary="/opt/ros/noetic/lib/rosbag/record"
if [[ -x "$rosbag_record_binary" ]]; then
    record_command=("$rosbag_record_binary")
else
    record_command=(rosbag record)
fi

bag_pid=""
capture_pid=""

# 两个后台 recorder 都必须收到 SIGINT，
# 以便脚本正常退出、失败或中断时刷新有效 PCAP/bag 索引。
cleanup()
{
    if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
        kill -INT "$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
    fi
    if [[ -n "$capture_pid" ]] && kill -0 "$capture_pid" 2>/dev/null; then
        kill -INT "$capture_pid" 2>/dev/null || true
        wait "$capture_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Read-only synchronized terrain baseline capture."
echo "No ROS publish, gait command, velocity command, or gridmap transmission is performed."
echo "Keep the robot stationary for the full capture."
echo "height_map_mode_state before capture: $mode_before"
echo "Duration: ${duration_s}s"
echo "ROS bag: $bag_file"
echo "TCP pcap: $pcap_file"

# 先启动 TCP 抓包并多保留四秒，
# 使网络证据完整覆盖 ROS bag 时间区间。
sudo -v
capture_duration_s=$((duration_s + 4))
sudo timeout --signal=INT "$capture_duration_s" \
    tcpdump -i "$interface" -nn -s 0 -U -w "$pcap_file" \
    "tcp and src host 192.168.1.105 and dst host $motion_ip and dst port $gridmap_port" &
capture_pid=$!

sleep 2
"${record_command[@]}" --buffsize=2048 -O "$bag_file" "${topics[@]}" &
bag_pid=$!
sleep "$duration_s"
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
bag_pid=""

wait "$capture_pid" 2>/dev/null || true
capture_pid=""
sudo chown "$(id -u):$(id -g)" "$pcap_file" 2>/dev/null || true

# 仅在两个标准 recorder 停止并刷新文件后生成可读汇总。
mode_after="$(read_mode_state)"
rosbag info --yaml "$bag_file" > "$info_file"
tcpdump -nn -tttt -q -r "$pcap_file" > "$packet_file" 2>&1 || true
tcpdump -nn -tttt -r "$pcap_file" -XX > "$hex_file" 2>&1 || true

# 将物理场景说明与机器数据放在一起。
# 显式记录 unknown，避免把未测量物体误认为已标定楼梯工装。
{
    echo "capture_time=$timestamp"
    echo "label=$label"
    echo "duration_s=$duration_s"
    echo "host=$(hostname)"
    echo "interface=$interface"
    echo "height_map_mode_state_before=$mode_before"
    echo "height_map_mode_state_after=$mode_after"
    echo "scene_description=${X30_SCENE_DESCRIPTION:-unknown}"
    echo "scene_step_height_m=${X30_SCENE_STEP_HEIGHT_M:-unknown}"
    echo "scene_step_depth_m=${X30_SCENE_STEP_DEPTH_M:-unknown}"
    echo "scene_step_width_m=${X30_SCENE_STEP_WIDTH_M:-unknown}"
    echo "scene_robot_distance_m=${X30_SCENE_ROBOT_DISTANCE_M:-unknown}"
    echo "robot_motion_command_sent=false"
    echo "gridmap_data_sent_by_script=false"
} > "$metadata_file"

# 独立哈希用于校验大型 bag 和 PCAP 的传输完整性。
sha256sum "$bag_file" > "${bag_file}.sha256"
sha256sum "$pcap_file" > "${pcap_file}.sha256"

echo "Capture complete."
echo "height_map_mode_state after capture: $mode_after"
echo "ROS bag: $bag_file"
echo "ROS bag info: $info_file"
echo "Metadata: $metadata_file"
echo "TCP pcap: $pcap_file"
echo "Packet summary: $packet_file"
echo "Hex dump: $hex_file"
echo "Bag SHA256: $(cut -d' ' -f1 "${bag_file}.sha256")"
echo "PCAP SHA256: $(cut -d' ' -f1 "${pcap_file}.sha256")"

# 即使两份记录本身有效，地形模式变化也会使前后对比产生歧义。
if [[ "$mode_before" != "$mode_after" ]]; then
    echo "WARNING: height_map_mode_state changed during capture: $mode_before -> $mode_after" >&2
    exit 8
fi
