import importlib.util
import sys
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
MODULE_PATH = TOOLS_DIR / "analyze_x30_gridmap_layers.py"
SPEC = importlib.util.spec_from_file_location("gridmap_layer_analyzer", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class GridmapLayerAnalyzerTest(unittest.TestCase):
    def make_frame(self):
        dimension_x = MODULE.wire.MultiArrayDimension("column_index", 2, 6)
        dimension_y = MODULE.wire.MultiArrayDimension("row_index", 3, 3)
        layer = MODULE.wire.GridMapLayer(
            name="elevation",
            dimensions=(dimension_x, dimension_y),
            data_offset=0,
            values=(0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
        )
        return MODULE.wire.GridMapFrame(
            record_time_ns=0,
            sequence=1,
            stamp_sec=1,
            stamp_nsec=2,
            frame_id="world",
            resolution=1.0,
            length_x=2.0,
            length_y=3.0,
            position=(10.0, 20.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            layers=(layer,),
            basic_layers=("elevation",),
            outer_start_index=1,
            inner_start_index=2,
        )

    def test_canonical_layer_values_unwraps_circular_buffer(self):
        values = MODULE.canonical_layer_values(self.make_frame(), "elevation")
        self.assertEqual(values, (5.0, 4.0, 1.0, 0.0, 3.0, 2.0))

    def test_cell_position_uses_grid_map_axis_convention(self):
        geometry = MODULE.geometry_from_frame(self.make_frame())
        self.assertEqual(MODULE.cell_position(geometry, 0, 0), (10.5, 21.0))
        self.assertEqual(MODULE.cell_position(geometry, 1, 2), (9.5, 19.0))

    def test_connected_components_uses_four_neighbors(self):
        components = MODULE.connected_components(
            {(0, 0), (1, 0), (1, 1), (5, 5), (6, 6)}
        )
        self.assertEqual([len(component) for component in components], [3, 1, 1])


if __name__ == "__main__":
    unittest.main()
