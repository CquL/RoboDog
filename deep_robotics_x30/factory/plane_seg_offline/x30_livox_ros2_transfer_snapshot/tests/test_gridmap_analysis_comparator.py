import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
MODULE_PATH = TOOLS_DIR / "compare_x30_gridmap_analyses.py"
SPEC = importlib.util.spec_from_file_location("gridmap_analysis_comparator", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def group(frame_index, center_x, center_y, mean_z):
    return {
        "frame_index": frame_index,
        "center_x": center_x,
        "center_y": center_y,
        "mean_z": mean_z,
        "minimum_x": center_x - 0.1,
        "maximum_x": center_x + 0.1,
        "minimum_y": center_y - 0.2,
        "maximum_y": center_y + 0.2,
        "extent_x": 0.2,
        "extent_y": 0.4,
        "z_range": 0.0,
        "area": 0.08,
    }


class GridmapAnalysisComparatorTest(unittest.TestCase):
    def test_local_region_check_uses_local_ground_and_bounds(self):
        candidate = [
            group(1, 1.0, 0.0, 0.21),
            group(2, 1.0, 0.0, 0.22),
            group(3, 1.0, 0.0, 0.23),
            group(4, 5.0, 0.0, 0.22),
        ]
        gridmap = {
            "largest_component": {
                "bounds_m": {"x": [0.8, 1.2], "y": [-0.3, 0.3]},
                "layer_comparison": {
                    "elevation": {"reference": {"median": 0.0}}
                },
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "comparison.json"
            path.write_text(json.dumps(gridmap), encoding="utf-8")
            result = MODULE.local_region_check(
                [],
                candidate,
                path,
                expected_step_height=0.22,
                tolerance=0.03,
                horizontal_z_range=0.03,
                expected_step_depth=0.2,
                expected_step_width=0.4,
            )

        self.assertTrue(result["distinct_local_surface_detected"])
        self.assertEqual(result["candidate"]["groups"], 3)
        self.assertEqual(result["candidate"]["frames"], 3)
        self.assertAlmostEqual(result["candidate"]["top_z_m"]["median"], 0.22)
        self.assertAlmostEqual(result["dimensions"]["depth_error_m"], 0.0)
        self.assertAlmostEqual(result["dimensions"]["width_error_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
