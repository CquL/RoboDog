# plane_seg upstream provenance

This directory contains the reusable C++ core from:

```text
Repository: https://github.com/ori-drs/plane_seg
Commit: f94dc77c684225eded23f488d5b94baf579fd460
License: BSD 3-Clause
Imported: 2026-07-14
```

Imported files:

```text
LICENSE
plane_seg/include/plane_seg/*.hpp -> include/plane_seg/
plane_seg/src/PlaneFitter.cpp
plane_seg/src/RobustNormalEstimator.cpp
plane_seg/src/IncrementalPlaneEstimator.cpp
plane_seg/src/PlaneSegmenter.cpp
plane_seg/src/RectangleFitter.cpp
plane_seg/src/BlockFitter.cpp
```

The ROS1 package metadata, ROS1 wrapper, examples, historical source files, and
test applications were intentionally not imported.

Do not treat this unmodified upstream snapshot as X30-compatible. The factory
X30 fork adds vertical-plane constraints, configurable minimum point counts,
extra rectangle results, and a substantial stair-specific post-processing
pipeline. Those changes must be implemented and validated against recorded
factory outputs before any terrain data is sent to the motion host.
