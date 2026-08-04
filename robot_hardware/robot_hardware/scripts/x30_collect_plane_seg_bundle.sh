#!/usr/bin/env bash

# 厂家 X30 plane_seg 实现的只读采集器。
# 仅查询正在运行的 ROS graph，并复制可读本地文件。
# 不发布 ROS 消息、不发送机器人或地形数据，也不停止进程。

set -u

# 将诊断信息与按绝对路径复制的文件分开保存，
# 使归档后仍保留来源和原始部署布局。
timestamp="$(date +%Y%m%d_%H%M%S)"
bundle_dir="${1:-$HOME/x30_plane_seg_bundle_${timestamp}}"
archive_path="${bundle_dir}.tar.gz"
diagnostics_dir="$bundle_dir/diagnostics"
files_dir="$bundle_dir/files"

mkdir -p "$diagnostics_dir" "$files_dir"

# 只 source 实际存在的 ROS overlay，
# 使采集器适用于 workspace 布局不同的厂家镜像。
source_if_present()
{
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        # shellcheck disable=SC1090
        source "$setup_file"
    fi
}

# 在 files/ 下重建绝对来源路径并解引用符号链接，
# 使归档离开机器人后仍保持自包含。
copy_absolute_file()
{
    local source_path="$1"
    [[ -f "$source_path" && -r "$source_path" ]] || return 0
    local destination="$files_dir$source_path"
    mkdir -p "$(dirname "$destination")"
    cp -L "$source_path" "$destination"
}

# 在输出前记录准确命令。可选工具或文件缺失本身也是证据，
# 因此不会中止整个采集过程。
capture_command()
{
    local output_file="$1"
    shift
    {
        printf '$'
        printf ' %q' "$@"
        printf '\n'
        "$@"
    } > "$diagnostics_dir/$output_file" 2>&1 || true
}

echo "Read-only plane_seg collection."
echo "ROS graph queries are read-only."
echo "No ROS publish, robot/terrain command, or process stop."
echo "Bundle directory: $bundle_dir"

source_if_present /opt/ros/noetic/setup.bash
source_if_present /home/ysc/jy_cog/drivers/setup.bash
source_if_present /home/ysc/jy_cog/system/devel/setup.bash

{
    echo "capture_time=$timestamp"
    echo "host=$(hostname)"
    echo "bundle_dir=$bundle_dir"
    echo "ros_master_uri=${ROS_MASTER_URI-}"
    echo "ros_ip=${ROS_IP-}"
    echo "ros_hostname=${ROS_HOSTNAME-}"
    echo "read_only=true"
} > "$diagnostics_dir/metadata.txt"

capture_command date.txt date --iso-8601=seconds
capture_command host_addresses.txt ip -brief address
capture_command rosnode_plane_seg.txt timeout 10 rosnode info /plane_seg
capture_command rosparam_plane_seg.txt timeout 10 rosparam get /plane_seg
capture_command topic_height_map.txt timeout 10 rostopic info \
    /deeprobotics_local_height_map_mid360/height_map
capture_command topic_quadrangels.txt timeout 10 rostopic info \
    /plane_seg/quadrangels
capture_command topic_mode_state.txt timeout 10 rostopic info \
    /height_map_mode_state
capture_command mode_state_sample.txt timeout 5 rostopic echo -n 1 \
    /height_map_mode_state

# 优先使用 ROS 节点 metadata；若不可用，再按可执行文件名
# 确定实际运行的 plane_seg 进程。
pid="$(rosnode info /plane_seg 2>/dev/null | awk '/Pid:/ {print $2; exit}')"
if [[ -z "$pid" ]]; then
    pid="$(pgrep -f '(^|/)plane_seg_ros([[:space:]]|$)' | head -n 1 || true)"
fi

if [[ -z "$pid" || ! -d "/proc/$pid" ]]; then
    echo "ERROR: running /plane_seg process was not found." >&2
    exit 2
fi

executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
if [[ -z "$executable" || ! -r "$executable" ]]; then
    echo "ERROR: plane_seg executable is not readable for pid $pid." >&2
    exit 3
fi

# 进程 metadata 和过滤后的环境信息用于将复制的二进制、库
# 关联到实时节点，而非未使用的安装副本。
{
    echo "pid=$pid"
    echo "executable=$executable"
    echo "cwd=$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    printf 'cmdline='
    tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true
    printf '\n'
} > "$diagnostics_dir/process.txt"

tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
    | grep -E '^(LD_LIBRARY_PATH|ROS_|CMAKE_PREFIX_PATH)=' \
    > "$diagnostics_dir/process_environment_filtered.txt" \
    || true
cp "/proc/$pid/maps" "$diagnostics_dir/process_maps.txt" 2>/dev/null || true

copy_absolute_file "$executable"
capture_command executable_sha256.txt sha256sum "$executable"
capture_command executable_file.txt file "$executable"
capture_command executable_ldd.txt ldd "$executable"
capture_command executable_readelf_dynamic.txt readelf -d "$executable"
capture_command executable_readelf_symbols.txt readelf -Ws "$executable"
capture_command executable_strings.txt strings -a -n 5 "$executable"

# 合并 ldd 依赖与进程映射文件，因为运行时加载的 plugin
# 可能不在 ELF 依赖表中。
{
    ldd "$executable" 2>/dev/null \
        | awk '/=> \// {print $3} /^\// {print $1}'
    awk '{print $NF}' "/proc/$pid/maps" 2>/dev/null \
        | grep '^/'
} | sort -u > "$diagnostics_dir/runtime_library_paths.txt"

while IFS= read -r library; do
    case "$library" in
        /home/ysc/*|/opt/robot/*|/usr/local/*)
            copy_absolute_file "$library"
            ;;
    esac
done < "$diagnostics_dir/runtime_library_paths.txt"

# 根据二进制中观察到的参数名和公开算法来源，
# 收集影响四边形输出的配置文件。
config_matches="$diagnostics_dir/config_matches.txt"
: > "$config_matches"
for root in \
    /home/ysc/jy_cog/vmap \
    /opt/robot/share/vmap \
    /home/ysc/jy_cog/system; do
    [[ -d "$root" ]] || continue
    timeout 40 grep -R -I -l -E \
        'plane_seg|correctMinSize|ignoreHeight|sideDistanceThreshold|heightMapTopic' \
        "$root" \
        --include='*.launch' --include='*.xml' --include='*.yaml' \
        --include='*.yml' --include='*.json' --include='*.conf' \
        --include='*.ini' --include='*.sh' --include='package.xml' \
        2>/dev/null >> "$config_matches" \
        || true
done

sort -u -o "$config_matches" "$config_matches"
while IFS= read -r config_file; do
    [[ -n "$config_file" ]] || continue
    copy_absolute_file "$config_file"
done < "$config_matches"

# 额外收集按名称识别的 plane-seg 文件，
# 它们可能未被当前进程命令行或配置搜索引用。
find_matches="$diagnostics_dir/plane_seg_named_files.txt"
: > "$find_matches"
for root in /home/ysc/jy_cog/vmap /opt/robot/share/vmap; do
    [[ -d "$root" ]] || continue
    timeout 30 find -L "$root" -type f \
        \( -iname '*plane*seg*' -o -path '*/plane_seg_ros/*' \) \
        -size -100M -print 2>/dev/null >> "$find_matches" \
        || true
done

sort -u -o "$find_matches" "$find_matches"
while IFS= read -r matched_file; do
    [[ -n "$matched_file" ]] || continue
    copy_absolute_file "$matched_file"
done < "$find_matches"

# 归档前为每个复制文件计算哈希，
# 使后续分析能准确证明使用了哪些厂家文件。
find "$files_dir" -type f -print0 \
    | sort -z \
    | xargs -0 -r sha256sum \
    > "$diagnostics_dir/collected_files.sha256" \
    2>/dev/null || true

# 最后统一打包，并输出归档哈希供 Windows 传输校验。
tar -czf "$archive_path" -C "$(dirname "$bundle_dir")" "$(basename "$bundle_dir")"
sha256sum "$archive_path" | tee "${archive_path}.sha256"

echo "Collection complete."
echo "Archive: $archive_path"
echo "SHA256 file: ${archive_path}.sha256"
echo "Copy both files back to the Windows workspace for offline analysis."
