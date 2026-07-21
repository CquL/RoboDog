# RoboDog H2 Stage 06E r9 patch

This patch contains complete replacement/new files for the r9 diagnostic run.

## Safety profile

- Default x/y linear speed: 0.10 m/s
- Stage 06E maximum linear speed: 0.10 m/s
- 0.90 m/s remains rejected
- Command rate: 20 Hz
- Default stream: 1000 ms
- MotionSwitcher is read-only (`CheckMode` only)
- A Stage 06E success gate is written only when:
  - RPC stream succeeds;
  - `/dog_odom` shows a speed response;
  - the operator confirms physical motion;
  - the operator confirms the bounded stream was safe.

## Files

Replace/add these paths in the repository:

- `robot_hardware/robot_hardware/CMakeLists.txt`
- `robot_hardware/robot_hardware/robot_test_unitree_h2_live_motion.cpp`
- `robot_hardware/robot_hardware/robot_test_unitree_h2_motion_mode.cpp`
- `robot_hardware/robot_hardware/include/unitree/unitree_h2_live_motion_plan.h`
- `robot_hardware/robot_hardware/tests/unitree_h2_live_motion_plan_test.cpp`
- `unitreeH2/remote/08_pc2_h2_single_axis_motion_gate.sh`
- `unitreeH2/remote/h2_dog_odom_capture.py`
- `unitreeH2/remote/tests/test_h2_gate_schema_offline.sh`

`unitreeH2/remote/h2_pc2_hal_gate_common.sh` is intentionally unchanged in this minimal r9 patch. Odom evidence is retained in r9 TSV/CSV/summary logs while the existing Stage 06E gate schema stays compatible.

## Local checks

```bash
bash -n unitreeH2/remote/08_pc2_h2_single_axis_motion_gate.sh
bash -n unitreeH2/remote/tests/test_h2_gate_schema_offline.sh
python3 -m py_compile unitreeH2/remote/h2_dog_odom_capture.py
bash unitreeH2/remote/tests/test_h2_gate_schema_offline.sh
```

## Build

Use the same SDK2 commit as r8:

```text
21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
```

Example clean build:

```bash
cd robot_hardware/robot_hardware
rm -rf build-h2-r9

cmake -S . -B build-h2-r9 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH=/opt/unitree_robotics

cmake --build build-h2-r9 -j"$(nproc)" --target \
  robot_hardware \
  robot_test_unitree_h2 \
  robot_test_unitree_h2_live_motion \
  robot_test_unitree_h2_motion_mode \
  unitree_h2_factory_contract_test \
  unitree_h2_direct_api_contract_test \
  unitree_h2_live_motion_plan_test

ctest --test-dir build-h2-r9 --output-on-failure -R 'unitree_h2'
```

## PC2 command after packaging

Do not run the old r8 Stage 06E script. After r9 is packaged and the existing valid Stage 06C/06D gates are accepted for the r9 manifest/boot policy, run:

```bash
bash scripts/08_pc2_h2_single_axis_motion_gate.sh \
  x-positive \
  --linear-speed 0.10 \
  --stream-ms 1000
```
