from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "x30_collect_plane_seg_bundle.sh"


def script_text() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def active_lines() -> list[str]:
    return [
        line.strip()
        for line in script_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


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
