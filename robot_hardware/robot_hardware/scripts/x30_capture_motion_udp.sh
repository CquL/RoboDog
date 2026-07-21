#!/usr/bin/env bash

# Capture X30 motion-host UDP traffic without sending any command itself.
# Use a second terminal for the factory or test command source while this runs.

set -u

motion_ip="${X30_MOTION_IP:-192.168.1.103}"
motion_port="${X30_MOTION_PORT:-43893}"
duration_s="${1:-15}"
label="${2:-capture}"
output_dir="${3:-$HOME/x30_udp_captures}"

if ! [[ "$duration_s" =~ ^[0-9]+$ ]] || (( duration_s < 1 || duration_s > 120 )); then
    echo "duration must be an integer from 1 to 120 seconds" >&2
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
pcap_file="$output_dir/${timestamp}_${label}.pcap"
hex_file="$output_dir/${timestamp}_${label}.hex.txt"

echo "Capture only: this script sends no robot command."
echo "Interface: $interface"
echo "Filter: UDP host $motion_ip port $motion_port"
echo "Duration: ${duration_s}s"
echo "PCAP: $pcap_file"

capture=(tcpdump -i "$interface" -nn -s 0 -U -w "$pcap_file" \
    "udp and host $motion_ip and port $motion_port")

if (( EUID == 0 )); then
    timeout --signal=INT "$duration_s" "${capture[@]}" || true
else
    sudo timeout --signal=INT "$duration_s" "${capture[@]}" || true
    sudo chown "$(id -u):$(id -g)" "$pcap_file" 2>/dev/null || true
fi

tcpdump -nn -tttt -r "$pcap_file" -XX > "$hex_file" 2>&1 || true

echo "Capture finished."
echo "PCAP: $pcap_file"
echo "Hex dump: $hex_file"
