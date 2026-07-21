# X30 ROS2 Livox MID360 package

This folder is the minimal transfer package for host 192.168.1.106.

Base image required on 106:

```bash
jezetek:navigation_system_amd64
```

This package adds:

```text
third_party/Livox-SDK2 v1.3.1
ws/src/livox_ros_driver2
ws/src/x30_livox_tools
ws/src/yesense_interface
ws/src/yesense_std_ros2
config/x30_multi_mid360_ros2.json
config/x30_multi_mid360_ros2_slave.json
```

The vendored Livox SDK is pinned at commit
`f5d9375f84efe2b15bc0a052d3e18482ed13adf4`. It provides multicast
`master_sdk=false` support and makes the image build independent of the older
SDK bundled in the base image.

Suggested remote path:

```bash
/home/ysc/x30_livox_ros2_transfer
```

After upload:

```bash
cd /home/ysc/x30_livox_ros2_transfer
chmod +x remote/*.sh
```

Build derived image:

```bash
bash remote/00_build_image.sh
```

Expected image:

```text
x30_livox_ros2:jezetek
```

Check current status:

```bash
bash remote/01_status.sh
```

## Recommended Single Container Startup

The recommended runtime is now one ROS2 container:

```text
Container: x30_ros2_sensors
Image:     x30_livox_ros2:jezetek
Launch:    x30_livox_tools/x30_all_sensors_launch.py
```

It starts these ROS2 nodes in the same container:

```text
x30_livox_ros2            -> /livox/lidar, /livox/imu
time_window_cloud_merger  -> /x30/points_merged
x30_body_imu              -> /x30/body_imu, /x30/body_imu_raw
```

Start one container:

```bash
bash remote/02_factory_livox_off.sh
bash remote/11_factory_imu_off.sh
bash remote/18_run_ros2_all.sh
```

Check all ROS2 sensor topics:

```bash
bash remote/19_check_ros2_all.sh
```

Expected topics:

```text
/livox/lidar          sensor_msgs/msg/PointCloud2
/livox/imu            sensor_msgs/msg/Imu
/x30/points_merged    sensor_msgs/msg/PointCloud2
/x30/body_imu         sensor_msgs/msg/Imu
```

Stop the single container and restore factory ROS1 drivers:

```bash
bash remote/20_stop_ros2_all.sh
bash remote/12_factory_imu_on.sh
bash remote/06_factory_livox_on.sh
bash remote/01_status.sh
```

## ROS1 Master + ROS2 Multicast Slave Test

This test keeps the factory ROS1 Livox driver as the only SDK master. The ROS2
container uses `master_sdk=false` and only receives the multicast point cloud.

```bash
bash remote/20_stop_ros2_all.sh
bash remote/06_factory_livox_on.sh
bash remote/21_probe_livox_slave_mode.sh
bash remote/22_run_ros2_livox_slave.sh
bash remote/24_check_livox_dual_receive.sh
bash remote/23_stop_ros2_livox_slave.sh
```

Do not run `remote/02_factory_livox_off.sh` for the slave test. If the probe
reports that `master_sdk` is missing, rebuild the image with
`remote/00_build_image.sh` before starting the slave.

The old split-container scripts below are kept as fallback/debug tools.

Stop factory ROS1 Livox driver:

```bash
bash remote/02_factory_livox_off.sh
```

Start ROS2 Livox driver:

```bash
bash remote/03_run_ros2_livox.sh
```

Watch logs:

```bash
docker logs -f x30_livox_ros2
```

Check ROS2 topics:

```bash
bash remote/04_check_ros2_topics.sh
```

Start 100 ms point cloud merger:

```bash
bash remote/07_start_cloud_merger.sh
```

Check merged output:

```bash
bash remote/08_check_merged_cloud.sh
```

Expected merged output:

```text
/x30/points_merged
sensor_msgs/msg/PointCloud2
about 10 Hz
```

Stop point cloud merger only:

```bash
bash remote/09_stop_cloud_merger.sh
```

Stop ROS2 Livox and restore factory ROS1 Livox:

```bash
bash remote/05_stop_ros2_livox.sh
bash remote/06_factory_livox_on.sh
bash remote/01_status.sh
```

Scope:

```text
ROS2 native Livox MID360 receiving only.
No robot velocity command is sent.
No /cmd_vel is published.
No navigation algorithm is started.
```

## Factory Body IMU

The X30 factory body IMU is a Yesense ROS1 driver. On the tested robot:

```text
Start script: /home/ysc/jy_cog/drivers/scripts/imu_driver.sh
Stop script:  /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh
ROS1 node:    /yesense_imu_node
ROS1 topic:   /imu/data
Subscriber:   /localization_node
Device:       /dev/ttyS0
Baudrate:     115200
```

Use the IMU scripts only during a stationary test window. Stopping the factory
IMU does not modify factory code, but it removes `/imu/data` from the localization
node until the factory driver is restored.

Check factory body IMU:

```bash
bash remote/10_factory_imu_status.sh
```

Stop factory body IMU:

```bash
bash remote/11_factory_imu_off.sh
```

Restore factory body IMU:

```bash
bash remote/12_factory_imu_on.sh
```

Probe factory IMU config:

```bash
bash remote/13_factory_imu_config_probe.sh
```

## ROS2 Native Body IMU Test

The package includes a ROS2 Yesense driver adapted for the X30 factory body IMU:

```text
Source: Avalue-Technology/ros2.humble.amr.avalue avalue_imu/yesense_ros2
Device: /dev/ttyS0
Baudrate: 115200
Frame: imu_link
ROS2 topic: /x30/body_imu
Raw Yesense topic: /x30/body_imu_raw
Serial backend: linux_serial
```

The factory ROS1 Yesense node must be stopped before starting the ROS2 driver,
because only one process can read `/dev/ttyS0`.

```bash
bash remote/10_factory_imu_status.sh
bash remote/11_factory_imu_off.sh
bash remote/14_run_ros2_body_imu.sh
bash remote/15_check_ros2_body_imu.sh
bash remote/17_probe_ros2_body_imu_raw.sh
bash remote/16_stop_ros2_body_imu.sh
bash remote/12_factory_imu_on.sh
```

Do not touch `/dev/ttyS1`; it is used by `RobotServ` on the tested robot.

Current tested status:

```text
/x30/body_imu publishes sensor_msgs/msg/Imu at about 200Hz.
Angular velocity and linear acceleration are valid.
Raw Yesense Euler topics are valid.
Raw Yesense quaternion fields are currently 0,0,0,0 on the tested robot.
The driver now falls back to Euler -> quaternion conversion for /x30/body_imu.orientation
when the raw quaternion is invalid.
```

## Offline Factory Terrain Analysis

These tools are read-only and run on downloaded ROS bag and packet-capture files.
They do not connect to the robot or send a terrain, gait, or velocity command.

```text
tools/analyze_x30_gridmap_baseline.py
tools/analyze_x30_gridmap_layers.py
tools/compare_x30_gridmap_analyses.py
tests/test_gridmap_baseline_analyzer.py
tests/test_gridmap_layer_analyzer.py
```

Compare the factory four-layer GridMap between a flat reference and a candidate
scene:

```powershell
python .\tools\analyze_x30_gridmap_layers.py `
  --reference-bag <flat.bag> `
  --candidate-bag <candidate.bag> `
  --output-dir <analysis-directory>
```

The parser handles `elevation`, packed `color`, `slope`, `accessibility`, and
the GridMap circular-buffer start indices. The generated CSV and JSON files are
diagnostic evidence only; they are not accepted as input for robot movement.

After running the bag/TCP baseline analyzer for both captures, compare support
quadrangles inside the GridMap-localized terrain region:

```powershell
python .\tools\compare_x30_gridmap_analyses.py `
  --reference <flat-analysis-directory> `
  --candidate <candidate-analysis-directory> `
  --expected-step-height <measured-height-m> `
  --expected-step-depth <measured-depth-m> `
  --expected-step-width <measured-width-m> `
  --gridmap-comparison <gridmap-comparison.json> `
  --output <quadrangle-comparison.json>
```

Use the GridMap-localized mode for scene captures. A global dominant-height
comparison can select an unrelated large plane and miss a valid local step.

## Pinned Plane Segmentation Baseline

`third_party/plane_seg` is a checksum-pinned snapshot of the reusable C++ core
from `ori-drs/plane_seg` commit
`f94dc77c684225eded23f488d5b94baf579fd460`, retained under its BSD 3-Clause
license. `UPSTREAM_VERSION.md` records provenance and `SHA256SUMS` protects the
snapshot from accidental edits.

This snapshot is not an X30 terrain node. The factory X30 fork adds API and
behavior for minimum point counts, floor-angle constraints, horizontal versus
vertical planes, rectangle metadata, and stair-specific quadrangle filtering.
Those changes must live in a separate ROS2 package and pass offline regression
against the recorded flat and measured-step datasets before a TCP sender is
implemented or enabled.

The factory AArch64 ELF analysis helper is:

```text
tools/analyze_x30_plane_seg_elf.py
tools/requirements-plane-seg-analysis.txt
```

It is an offline inspection tool and never connects to the robot.

## Offline X30 Plane Segmentation Core

`ws/src/x30_plane_seg_core` now contains the first ROS-runtime-independent
implementation of the recovered X30 core plane segmentation behavior. It links
the pinned upstream snapshot without modifying it and returns only in-memory
`core_rectangles`.

Implemented factory evidence:

```text
accessibility > 0.9 filtering
20 points normally, 40 points for factory mode 9
0.025 m plane error threshold
15 degree floor-angle constraint
0.08 m region search radius
```

The recovered factory binary stores but never reads its vertical-plane flag and
always emits block type 0. The package therefore does not advertise vertical
plane support.

Compact reference fixtures are kept in:

```text
tests/fixtures/plane_seg
tests/fixtures/plane_seg_paired
```

`plane_seg` contains three sampled factory `/plane_seg/quadrangels` output
frames from each of four recorded scenes. `plane_seg_paired` adds the exact
same-stamp factory GridMap input, raw float32 layer blobs, TF audit, and expected
quadrangles for three frames per scene. Every fixture, source bag, sidecar, and
binary blob is SHA-256 recorded.

Regenerate the compact paired fixtures from the downloaded ROS1 bags:

```powershell
python .\tools\export_x30_plane_seg_paired_fixtures.py `
  --output-dir .\tests\fixtures\plane_seg_paired `
  --source-root ..\ `
  --frames-per-bag 3 `
  ..\x30_stair_baselines\mode3_flat_20260714_142726.bag `
  ..\x30_stair_baselines\mode3_flat_repeat_20260714_150324.bag `
  ..\x30_stair_baselines\mode3_measured_step_h022_d029_w035_20260714_160857.bag `
  ..\x30_stair_baselines\mode3_object_probe_20260714_153211.bag
```

The pairing policy is intentionally strict:

```text
GridMap Header.stamp == quadrangles Header.stamp
dynamic TF Header.stamp must match exactly
static TF is valid for the complete source bag
nearest or future TF fallback is disabled
/plane_seg/look_pose stamp 0 is not a pairing key
/height_map_mode and /height_map_mode_state remain separate signals
```

The four committed fixtures contain 12 selected pairs and 1,920,000 exact
layer bytes. A full measured-step export covers all 147 GridMap/quadrangles
pairs; 146 have exact `world -> base_link` dynamic TF and the first pair records
the missing TF explicitly.

`grid_map_adapter.hpp` converts a validated, ROS-independent POD GridMap view
into `TerrainSample` values. It reproduces the circular-buffer index mapping,
cell-center coordinates, pose transform, absolute elevation semantics, and NaN
preservation. The adapter does not parse ROS messages or fixture JSON.

The paired fixture corpus is compiled into a package-local, dependency-free
binary replay pack with:

```powershell
python .\tools\compile_x30_plane_seg_replay.py
```

The committed pack contains 12 real same-stamp frames and is verified by both
Python and C++ SHA-256 checks. `x30_plane_seg_core_replay_test` loads each
GridMap, runs the adapter and core, canonicalizes the intermediate candidates,
and freezes the complete JSONL output hash. The corresponding lightweight
regression manifest is:

```text
ws/src/x30_plane_seg_core/test/fixtures/x30_plane_seg_replay_v1.metrics.json
```

Generate a deterministic manifest from replay output with:

```powershell
python .\tools\summarize_x30_plane_seg_replay.py `
  --input <replay-output.jsonl> `
  --output <metrics.json> `
  --source-pack .\ws\src\x30_plane_seg_core\test\fixtures\x30_plane_seg_replay_v1.x30rpl
```

The replay resets the pinned upstream RANSAC seed before every frame. This
makes regression output reproducible but does not claim factory RNG parity.
The complete measured-step sequence covers all 147 frames. It currently finds
one degenerate candidate at selected frame 132 after Qhull rejects a collinear
plane cluster; this remains explicit input to the final X30 post-processing
work.

Run the offline Python checks with:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

The image build runs every `x30_plane_seg_core_*_test` under
`docker run --network none`. This covers the core contract, GridMap adapter,
strict replay format/corruption checks, and the 12-frame real-data replay.

The package has no ROS publisher and no network sender. It is built into the
image for offline testing but is not part of the default container launch.
