from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "analyze_x30_plane_seg_elf.py"


def load_module():
    spec = spec_from_file_location("analyze_x30_plane_seg_elf", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_compact_symbol_name_recovers_factory_method_names():
    module = load_module()

    assert module.compact_symbol_name("_ZN4Pass20elevationMapCallbackEv") == (
        "Pass::elevationMapCallback"
    )
    assert module.compact_symbol_name("_ZN8planeseg11BlockFitter2goEv") == (
        "planeseg::BlockFitter::go"
    )
    assert module.compact_symbol_name("sqrt@plt") == "sqrt@plt"
