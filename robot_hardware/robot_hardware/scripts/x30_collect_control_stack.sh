#!/usr/bin/env bash

# Read-only X30 factory control-stack inventory.
# This script does not publish ROS messages and does not send UDP commands.

set -u

timestamp="$(date +%Y%m%d_%H%M%S)"
output_file="${1:-$HOME/x30_control_stack_${timestamp}.log}"
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
echo "Read-only collection: no ROS publish and no UDP transmission."
echo "Output: $output_file"

section "Host"
run date --iso-8601=seconds
run hostname
run uname -a
run ip -brief address
run ip route get 192.168.1.103

section "ROS environment"
source_if_present /opt/ros/noetic/setup.bash
source_if_present /home/ysc/jy_cog/drivers/setup.bash
source_if_present /home/ysc/jy_cog/system/devel/setup.bash
for variable_name in ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_PACKAGE_PATH
do
    printf '%s=%s\n' "$variable_name" "${!variable_name-}"
done

section "ROS topics"
run timeout 8 rostopic list
for topic in \
    /cmd_vel \
    /cmd_vel_corrected \
    /handle_state \
    /robot_velocity \
    /leg_odom \
    /control_mode \
    /robot_basic_state \
    /robot_gait_state \
    /vel_source \
    /vel_source_state
do
    section "Topic $topic"
    run timeout 5 rostopic type "$topic"
    run timeout 5 rostopic info "$topic"
    run timeout 5 rostopic echo -n 1 "$topic"
done

section "ROS nodes"
run timeout 8 rosnode list -a
for node in /udp_sender /udp_receiver /robot_server /DwaLocalPlanner
do
    section "Node $node"
    run timeout 5 rosnode info "$node"
done

section "ROS parameters"
for parameter in /vel_source /brake_mode /control_mode
do
    run timeout 5 rosparam get "$parameter"
done
run timeout 5 rosparam get /udp_sender
run timeout 5 rosparam get /udp_receiver
rosparam list 2>/dev/null \
    | grep -E '^/(udp_sender|udp_receiver|vel_source|brake_mode)' \
    || true

section "Processes"
run pgrep -af 'udp_sender|udp_receiver|robot_server|DwaLocalPlanner|jy_exe|transfer'

section "udp_sender process details"
udp_sender_pids="$(pgrep -f 'udp_sender' || true)"
if [[ -z "$udp_sender_pids" ]]; then
    echo "No process matching udp_sender was found."
else
    for pid in $udp_sender_pids
    do
        echo "--- pid=$pid ---"
        run readlink -f "/proc/$pid/exe"
        run readlink -f "/proc/$pid/cwd"
        if [[ -r "/proc/$pid/cmdline" ]]; then
            tr '\0' ' ' < "/proc/$pid/cmdline"
            printf '\n'
        fi
        executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
        if [[ -n "$executable" && -r "$executable" ]]; then
            run sha256sum "$executable"
            run ldd "$executable"
            echo "Selected strings from $executable:"
            strings "$executable" 2>/dev/null \
                | grep -E 'cmd_vel|cmd_vel_corrected|43893|43899|MotionComplex|network\.toml' \
                | head -n 200 \
                || true
        fi
    done
fi

section "UDP sockets"
ss -lunp 2>/dev/null \
    | grep -E '43893|43897|43899|udp_sender|udp_receiver|robot_server' \
    || true

section "Factory configuration references"
for root in /home/ysc/jy_cog /home/ysc/jy_exe
do
    [[ -d "$root" ]] || continue
    echo "Searching $root"
    timeout 25 grep -R -n -E \
        '192\.168\.1\.103|43893|43897|43899|cmd_vel_corrected|network\.toml|max_forward_vel' \
        "$root" \
        --include='*.toml' --include='*.yaml' --include='*.yml' \
        --include='*.launch' --include='*.xml' --include='*.conf' \
        --include='*.ini' --include='*.sh' --include='*.cpp' --include='*.hpp' \
        2>/dev/null \
        | head -n 500 \
        || true
done

section "Candidate network configuration files"
find /home/ysc/jy_cog /home/ysc/jy_exe \
    -type f \( -iname 'network.toml' -o -iname '*network*.toml' -o -iname '*udp*.toml' \) \
    2>/dev/null \
    | sort \
    | head -n 200 \
    || true

section "Factory message transformer launch files"
for launch_file in \
    /home/ysc/jy_cog/transfer/include/message_transformer_cpp/message_transformer_106.launch \
    /home/ysc/jy_cog/transfer/share/message_transformer_cpp/launch/message_transformer_106.launch \
    /home/ysc/jy_cog/transfer/share/message_transformer_cpp/launch/message_transformer_cpp.launch
do
    if [[ -r "$launch_file" ]]; then
        echo "--- $launch_file ---"
        sed -n '1,220p' "$launch_file"
    fi
done

section "Done"
echo "Saved read-only diagnostic log to: $output_file"
