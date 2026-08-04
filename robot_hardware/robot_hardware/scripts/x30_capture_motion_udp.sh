#!/usr/bin/env bash

# 被动抓取 X30 运动主机 UDP 流量，脚本自身不发送任何命令。
# 运行期间请在另一终端启动厂家命令源或测试命令源。

set -u

# 环境变量使被动抓包可直接复用于其他 X30 网络，无需修改脚本；
# 位置参数用于描述本次样本。
motion_ip="${X30_MOTION_IP:-192.168.1.103}"
motion_port="${X30_MOTION_PORT:-43893}"
duration_s="${1:-15}"
label="${2:-capture}"
output_dir="${3:-$HOME/x30_udp_captures}"

# 限制抓包时长，避免无人值守诊断持续占用存储空间。
if ! [[ "$duration_s" =~ ^[0-9]+$ ]] || (( duration_s < 1 || duration_s > 120 )); then
    echo "duration must be an integer from 1 to 120 seconds" >&2
    exit 2
fi

if ! command -v tcpdump >/dev/null 2>&1; then
    echo "tcpdump is required but was not found" >&2
    exit 3
fi

# 通过路由表确定实际到达运动主机的网卡，不假定目标机网卡名称。
interface="$(ip route get "$motion_ip" 2>/dev/null | awk '{for (i=1; i<=NF; ++i) if ($i=="dev") {print $(i+1); exit}}')"
if [[ -z "$interface" ]]; then
    echo "Could not determine the interface used to reach $motion_ip" >&2
    exit 4
fi

mkdir -p "$output_dir"
timestamp="$(date +%Y%m%d_%H%M%S)"
pcap_file="$output_dir/${timestamp}_${label}.pcap"
hex_file="$output_dir/${timestamp}_${label}.hex.txt"

echo "Capture only: this script sends no robot command."
echo "Interface: $interface"
echo "Filter: UDP host $motion_ip port $motion_port"
echo "Duration: ${duration_s}s"
echo "PCAP: $pcap_file"

# 过滤器观测指定主机/端口的双向流量。
# tcpdump 只写入原始证据，不创建命令 socket。
capture=(tcpdump -i "$interface" -nn -s 0 -U -w "$pcap_file" \
    "udp and host $motion_ip and port $motion_port")

if (( EUID == 0 )); then
    timeout --signal=INT "$duration_s" "${capture[@]}" || true
else
    sudo timeout --signal=INT "$duration_s" "${capture[@]}" || true
    sudo chown "$(id -u):$(id -g)" "$pcap_file" 2>/dev/null || true
fi

# 同时保留供机器分析的 PCAP 和便于快速离线查看的 hex 文本。
tcpdump -nn -tttt -r "$pcap_file" -XX > "$hex_file" 2>&1 || true

echo "Capture finished."
echo "PCAP: $pcap_file"
echo "Hex dump: $hex_file"
