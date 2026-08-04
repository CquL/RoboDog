from pathlib import Path


# 这些源码级检查保护证据采集器的两项契约：
# 收集足够的厂家文件供离线分析，并始终保持只读。
ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "x30_collect_plane_seg_bundle.sh"


# 按打包后的实际形式读取部署脚本。
def script_text() -> str:
    return SCRIPT.read_text(encoding="utf-8")


# 搜索禁用的活动命令前，先移除注释和空行。
def active_lines() -> list[str]:
    return [
        line.strip()
        for line in script_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


# 确保归档包含运行节点、ROS 契约、可执行文件、链接库、
# 符号、字符串和相关配置证据。
def test_plane_seg_collector_captures_required_factory_evidence():
    source = script_text()

    assert "rosnode info /plane_seg" in source
    assert "rosparam get /plane_seg" in source
    assert "/deeprobotics_local_height_map_mid360/height_map" in source
    assert "/plane_seg/quadrangels" in source
    assert "/height_map_mode_state" in source
    assert 'readlink -f "/proc/$pid/exe"' in source
    assert "ldd \"$executable\"" in source
    assert "readelf -Ws" in source
    assert "strings -a -n 5" in source
    assert "correctMinSize" in source
    assert "sideDistanceThreshold" in source
    assert 'tar -czf "$archive_path"' in source
    assert 'sha256sum "$archive_path"' in source


# 即使未来扩大采集范围，也禁止控制、停止进程和发送地形数据的命令。
def test_plane_seg_collector_has_no_active_control_or_process_stop_command():
    source = "\n".join(active_lines())
    forbidden = (
        "rostopic pub",
        "rosservice call",
        "rosnode kill",
        "pkill ",
        "killall ",
        "docker stop",
        "systemctl stop",
        "supervisorctl stop",
        "0x3101040B",
        "0x31010421",
        "192.168.1.103:49999",
    )

    for command in forbidden:
        assert command not in source
