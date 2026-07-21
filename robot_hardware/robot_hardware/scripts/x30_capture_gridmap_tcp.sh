#!/usr/bin/env bash

# Passively capture the factory terrain stream sent to the X30 motion host.
# Static analysis of gridmap_receiver confirms that port 49999 uses TCP.
# This script does not publish ROS messages or send robot/control data.

set -u

motion_ip="${X30_MOTION_IP:-192.168.1.103}"
gridmap_port="${X30_GRIDMAP_PORT:-49999}"
duration_s="${1:-15}"
label="${2:-factory_idle}"
output_dir="${3:-$HOME/x30_gridmap_captures}"

if ! [[ "$duration_s" =~ ^[0-9]+$ ]] || (( duration_s < 1 || duration_s > 120 )); then
    echo "duration must be an integer from 1 to 120 seconds" >&2
    exit 2
fi

if ! [[ "$label" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "label may contain only letters, digits, dot, underscore, and hyphen" >&2
    exit 2
fi

if ! command -v tcpdump >/dev/null 2>&1; then
    echo "tcpdump is required but was not found" >&2
    exit 3
fi

interface="$(ip route get "$motion_ip" 2>/dev/null | awk '{for (i=1; i<=NF; ++i) if ($i=="dev") {print $(i+1); exit}}')"
if [[ -z "$interface" ]]; then
    echo "Could not determine the interface used to reach $motion_ip" >&2
    exit 4
fi

mkdir -p "$output_dir"
timestamp="$(date +%Y%m%d_%H%M%S)"
prefix="$output_dir/${timestamp}_${label}"
pcap_file="${prefix}.pcap"
packet_file="${prefix}.packets.txt"
hex_file="${prefix}.hex.txt"

echo "Capture only: this script sends no robot command or gridmap data."
echo "Interface: $interface"
echo "Filter: TCP dst $motion_ip port $gridmap_port"
echo "Duration: ${duration_s}s"
echo "PCAP: $pcap_file"

if ! ip -4 -o address show 2>/dev/null | grep -q '192\.168\.1\.105/'; then
    echo "WARNING: this host does not own 192.168.1.105."
    echo "The factory gridmap_port is started by the 105 launch, not the 106 launch."
    echo "A switched Ethernet interface on 106 normally cannot observe unicast traffic from 105 to 103."
fi

capture=(tcpdump -i "$interface" -nn -s 0 -U -w "$pcap_file" \
    "tcp and dst host $motion_ip and dst port $gridmap_port")

if (( EUID == 0 )); then
    timeout --signal=INT "$duration_s" "${capture[@]}" || true
else
    sudo timeout --signal=INT "$duration_s" "${capture[@]}" || true
    sudo chown "$(id -u):$(id -g)" "$pcap_file" 2>/dev/null || true
fi

tcpdump -nn -tttt -q -r "$pcap_file" > "$packet_file" 2>&1 || true
tcpdump -nn -tttt -r "$pcap_file" -XX > "$hex_file" 2>&1 || true

echo "Capture finished."
packet_count="$(tcpdump -nn -r "$pcap_file" 2>/dev/null | wc -l)"
echo "Packet count: $packet_count"
if [[ "$packet_count" -eq 0 ]]; then
    echo "No matching packet was written. Run this on 192.168.1.105 and confirm gridmap_port is connected."
fi
echo "PCAP: $pcap_file"
echo "Packet summary: $packet_file"
echo "Hex dump: $hex_file"
sha256sum "$pcap_file" "$packet_file" "$hex_file" 2>/dev/null || true
