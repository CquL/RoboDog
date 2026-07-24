# x30_plane_seg_core

`x30_plane_seg_core` is the ROS-independent, offline-first terrain plane
segmentation layer for the X30 stair pipeline.

Current input:

```text
POD GridMap geometry/layer views, or
3D terrain samples + accessibility values + sensor pose
```

Current output:

```text
core rectangle candidates
opt-in computeStairPose + correctQuadPose geometry prefix
```

The package intentionally does **not** contain:

```text
ROS publishers
the complete X30 stair-specific quadrangle post-processing chain
TCP or UDP sockets
192.168.1.103:49999 transmission
robot gait or velocity commands
```

`grid_map_adapter.hpp` provides the ROS-runtime-independent GridMap input path.
Its input is a non-owning POD view containing dimensions, geometry, pose,
circular-buffer start indices, and raw float32 `elevation`/`accessibility`
arrays. The adapter validates the complete geometry and array contract, then
unwraps each layer with the factory index rule:

```text
raw = (x + outer_start_index) % size_x
    + (y + inner_start_index) % size_y * size_x
```

Logical X and Y cell centers decrease from positive to negative map extents and
are transformed by the normalized GridMap pose. Elevation already has the
factory `toPointCloud` absolute-Z meaning, so `center.z` is validated but is not
added to it. Every cell is emitted as a `TerrainSample`; NaN elevation and raw
accessibility values are deliberately preserved for the existing core filter.

Recorded same-stamp GridMap/quadrangle fixtures live in
`tests/fixtures/plane_seg_paired` at the transfer-root level. They preserve
the original float32 layer bytes and circular-buffer metadata. The package
unit test uses synthetic POD input to validate the adapter independently of
JSON and ROS serialization.

`test/fixtures/x30_plane_seg_replay_v1.x30rpl` is the self-contained binary
replay form of the 12 committed real frames. The package-local replay loader
validates fixed little-endian records, every blob range, the body and per-frame
SHA-256 values, GridMap geometry, exact/missing-TF flags, and the recovered
sensor pose contract before any core code runs. This avoids adding a JSON
runtime dependency to the target image.

The replay executable resets `std::rand()` to seed `1` immediately before each
frame. This is a deterministic regression profile for the pinned upstream
RANSAC implementation; it is not evidence that the factory process used seed
`1`. Canonical candidate JSONL is SHA-256 frozen by CTest, while
`x30_plane_seg_replay_v1.metrics.json` records lightweight per-frame metrics.

Current committed replay evidence:

```text
12/12 real frames execute successfully
retained samples: 4087..4186
core candidates: 7..12, 104 total
factory quadrangle groups: 3..9, 83 total
candidate-count equality: 1/12 frames
degenerate core candidates: 0
```

The complete 147-frame measured-step replay also executes deterministically,
but frame 132 exposes one collinear plane cluster. PCL/Qhull cannot build a 2D
convex hull and the current intermediate core emits one zero-area candidate.
That candidate is recorded as a diagnostic and is not silently filtered here;
the X30 final-quadrangle post-processing parity work must resolve it.

The implementation is derived from the checksum-pinned BSD-3-Clause
`ori-drs/plane_seg` snapshot in `third_party/plane_seg`. X30 behavior is added
in separate `factory_*` source files so the immutable upstream snapshot remains
auditable.

Confirmed X30 behavior implemented at this stage:

```text
accessibility > 0.9 removes the corresponding terrain sample
normal minimum plane size = 20 points
factory mode 9 minimum plane size = 40 points
candidate plane must be within 15 degrees of horizontal
factory search radius = 0.08 m and plane error threshold = 0.025 m
the result is labelled core_rectangles, not final X30 quadrangels
```

The factory header contains `setComputeVerticalPlane()` and block type values
for horizontal/vertical output. Offline AArch64 analysis shows that this factory
binary stores but never reads that flag and always emits type `0`. This package
therefore does not claim vertical-plane output.

Before this output can become `/x30/terrain/quadrangels`, the recorded factory
post-processing sequence and all measured-step regression frames must match.
Network transmission stays out of this package even after parity is reached.

`quadrangle_postprocessing.hpp` now contains the first evidence-backed factory
post-processing prefix:

```text
candidate top faces
  -> computeStairPose least-squares fit
  -> edge-direction consensus
  -> correctQuadPose reconstruction from contained plane points
```

The recovered constants are explicit: eligible rectangles require a long edge
greater than `0.8 m` and a short edge greater than `0.1 m`; pose correction
requires at least four quadrangles and filters direction consensus with
`cos(20 deg)`. This stage is still not the factory final output. The later
`cutByX`, same-stair merge, unnecessary-quad filtering, intrusion/reference,
and temporal error-repair stages remain required.
