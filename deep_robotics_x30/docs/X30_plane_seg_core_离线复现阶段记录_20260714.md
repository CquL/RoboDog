# X30 plane_seg_core 离线复现阶段记录

更新时间：2026-07-14  
项目路径：`D:\Desktop\RoboDog`  
实施边界：仅操作本地离线文件，不连接机器狗，不发送 ROS 控制消息，不发送 TCP/UDP 地形数据。

## 1. 本轮计划做什么

```text
1. 固定公开 ori-drs/plane_seg 基线，不直接修改第三方快照。
2. 从原厂 ARM64 二进制复核 X30 核心差异，避免按头文件名称猜功能。
3. 新建 ROS 运行时无关的 x30_plane_seg_core。
4. 从现有真实 bag 导出小型、可校验的离线回归夹具。
5. 把新包接入 Docker 构建，但不加入默认启动链路。
```

## 2. 本轮实际完成

新增包：

```text
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/
  CMakeLists.txt
  package.xml
  README.md
  include/x30_plane_seg_core/plane_seg_core.hpp
  include/plane_seg/*.hpp
  src/plane_seg_core.cpp
  src/factory_block_fitter.cpp
  src/factory_incremental_plane_estimator.cpp
  src/factory_plane_segmenter.cpp
  test/core_contract_test.cpp
```

该包当前只做：

```text
带 accessibility 的三维地形采样
  -> accessibility 过滤
  -> PCL 降采样和法向估计
  -> 平面区域生长
  -> 最小面积矩形候选
  -> 内存中的 core_rectangles
```

该包明确不包含：

```text
ROS publisher
最终 /plane_seg/quadrangels
TCP/UDP socket
192.168.1.103:49999 sender
步态命令或速度命令
```

`RobotHardwareInterface`、`robot_hardware` 的三个抽象控制函数本轮没有修改。

## 3. 原厂二进制精确复核结果

复核对象：

```text
libplane_seg.so
SHA256: 4AFBB750A7C0452B436447E9ACE0C50A8CF1F0430788C6E96D317DA960673A1E

plane_seg_ros
SHA256: 1665140E1989677ACBEA683673F32E0F4C5B7172A9180BC393D067AAAD883B67
Build ID: b443e0118ba90d6105d580d7e28f583192aa2078
```

确认的 X30 核心行为：

| 项目 | 原厂行为 |
|---|---|
| accessibility | `> 0.9` 的单元不进入平面拟合 |
| 普通最小平面点数 | 20 |
| `/height_map_mode == 9` | 最小平面点数 40 |
| PlaneSegmenter 最大误差 | 0.025 m |
| 平面与地面最大夹角 | 15 度 |
| 区域搜索半径 | 0.08 m |
| 平面接受条件 | `abs(normal.z) >= cos(15 deg)` |
| 小平面拒绝条件 | `component_count < min_points`，等于阈值时保留 |

五参数 `IncrementalPlaneEstimator::tryPoint()` 的原厂实现不会使用传入的
`iNormal` 和 `iMaxAngle`。它用全局 Z 方向的 15 度约束替代了公开版的法向一致性判断，然后继续使用公开版的均方误差增量判断。

### 3.1 对垂直平面能力的纠正

原厂头文件虽然存在：

```cpp
setComputeVerticalPlane(bool)
Block::type  // 0 horizontal, 1 vertical
```

但当前原厂二进制中：

```text
setComputeVerticalPlane() 只保存成员值
BlockFitter::go() 从不读取这个成员
所有输出 block.type 都固定写为 0
```

因此当前版本不能声称已经支持垂直平面输出。`x30_plane_seg_core` 保留该内部
ABI，但不向上层暴露“垂直平面已实现”的能力。

## 4. 公开算法基线

固定快照：

```text
repository: https://github.com/ori-drs/plane_seg
commit: f94dc77c684225eded23f488d5b94baf579fd460
license: BSD 3-Clause
path: x30_livox_ros2_transfer/third_party/plane_seg
```

Docker 构建会先执行 `sha256sum --check SHA256SUMS`，然后让 X30 专用源文件
链接该固定快照。第三方快照本身保持不变，原厂差异只写在 `factory_*` 文件中。

## 5. 离线回归夹具

新增：

```text
x30_livox_ros2_transfer/tools/export_x30_plane_seg_fixtures.py
x30_livox_ros2_transfer/tests/test_plane_seg_fixture_exporter.py
x30_livox_ros2_transfer/tests/fixtures/plane_seg/
```

当前包含 4 组真实采集，每组均匀抽取 3 帧：

```text
mode3_flat
mode3_flat_repeat
mode3_object_probe
mode3_measured_step_h022_d029_w035
```

夹具总大小约 52 KB，不复制原始数百 MB bag。每个夹具记录源 bag 路径、源
SHA256、帧索引、时间戳、frame_id、XYZ 和逐帧 payload SHA256。重复导出得到的
manifest SHA256 为：

```text
5465C630BAB1B8A8A18E58BFA7339D9136AA5C6AEF4F9012D2DE3C7E0B4751AD
```

重要边界：这些夹具当前是原厂最终 `/plane_seg/quadrangels` 的小型标准答案，
还不是完整的 GridMap 输入夹具。它们用于保护最终输出格式和顶点数据；下一步仍需
把对应的 elevation、accessibility、GridMap 几何和 TF 配成输入/输出对。

## 6. Docker 接入

已修改：

```text
x30_livox_ros2_transfer/Dockerfile
x30_livox_ros2_transfer/remote/00_build_image.sh
```

构建阶段新增检查：

```text
1. 校验固定 upstream 文件 SHA256。
2. 构建 x30_plane_seg_core。
3. 在 --network none 容器中确认包和共享库存在。
4. 要求 CTest 中确实存在 core contract test。
5. 用 441 点合成水平面进入真实 PCL 法向估计、区域生长和矩形拟合路径。
6. 用 ldd 检查共享库运行时依赖。
```

默认容器 CMD 未修改，仍只启动 Livox、点云合成和本体 IMU。新核心包不会自动运行。

## 7. 本轮验证

本地 Python 测试：

```text
25 passed
```

测试覆盖：

```text
固定 upstream 哈希
原厂 ELF 离线分析工具
GridMap/TCP 分析工具
真实 quadrangels 夹具及其哈希
x30_plane_seg_core 文件和常量契约
核心包无 ROS publisher、无 socket、无 103:49999 地址
Docker 构建安全检查
```

本机已使用离线基础镜像完成 Docker 构建，并在完全断网的容器中完成运行验证：

```text
image: x30_livox_ros2:jezetek
image ID: sha256:ee77faed8e07c7e03240244f4eb52d5f531dcdbf1309119d99546c32d2056d2b
image size: 3519911397 bytes
libeigen3-dev: 3.4.0-2ubuntu2
libpcl-dev: 1.12.1+dfsg-3build1
ros2 pkg prefix: /ws/install/x30_plane_seg_core
shared library: /ws/install/x30_plane_seg_core/lib/libx30_plane_seg_core.so
CTest: 1/1 passed, 0 failed
```

`ldd` 已确认共享库的 PCL、Eigen/C++ 运行时及其传递依赖均能解析，没有
`not found`。验证容器使用 `--network none`，没有接触机器狗网络。

## 8. 当前流水线做到哪里

```text
ROS2 Livox + IMU 接收                     已完成
四雷达时间窗合成 /x30/points_merged       已完成
LIO / 去畸变 / 重力对齐                   未实现
x30_local_height_map                      未实现
GridMap -> 地形采样适配                   仅完成核心输入接口，未接在线 GridMap
x30_plane_seg_core                         第一版已实现，容器编译与合成平面合约测试通过
X30 四边形后处理                           未实现
/x30/terrain/quadrangels                   未发布
x30_gridmap_sender                         未实现
TCP 192.168.1.103:49999                    禁止发送
45 度楼梯运动测试                          禁止执行
```

所以当前位于：

```text
公开核心算法 + 已确认 X30 核心差异
```

还没有到：

```text
原厂完整 quadrangels 等价复现
```

## 9. 下一步

```text
1. 导出时间对齐的 GridMap 输入夹具：elevation、accessibility、几何、循环索引、TF。
2. 将输入夹具转换成与原厂一致的 world 坐标地形采样。
3. 比较核心 rectangle candidates 与原厂中间输出，先完成平地负样本回归。
4. 按已恢复顺序实现 X30 四边形后处理。
5. 对 147 帧实测台阶逐帧检查数量、顶点顺序、高度、特殊第 0 组和时间戳。
6. 达标后才新增 ROS2 只读可视化 wrapper。
7. sender 继续放在独立包中，并保持默认关闭。
```

## 10. 安全结论

本轮没有连接 `192.168.1.103`、`192.168.1.105` 或 `192.168.1.106`，没有停止
原厂 ROS1 节点，没有切换高度图模式或步态，没有发送速度、地形或楼梯数据。

## 11. 本轮传输包

只覆盖原有单一传输包，没有新增日期后缀副本：

```text
D:\Desktop\RoboDog\x30_livox_ros2_transfer.tar.gz
size: 607061 bytes
SHA256: 1C9715C70FC8E5E3B2CD65538592DE37F5829CD25D745AA69592AB9EB96746ED
```

压缩包已确认包含 `x30_plane_seg_core`、固定 upstream、离线夹具和导出工具，
并已清除 `__pycache__` 与 `.pyc` 生成文件。

## 12. 当前能测试什么

### 12.1 先测试离线核心库

这个测试不接收雷达、不访问机器狗网络，也不需要关闭任何 ROS1 节点：

```bash
docker run --rm --network none \
  x30_livox_ros2:jezetek \
  ctest --test-dir /ws/build/x30_plane_seg_core \
  --output-on-failure --no-tests=error \
  -R '^x30_plane_seg_core_contract_test$'
```

期望结果：

```text
1/1 Test #1: x30_plane_seg_core_contract_test ... Passed
100% tests passed, 0 tests failed
```

该测试只证明核心库在目标 AMD64/Humble 镜像中能够运行，并且合成水平面可以进入
真实 PCL 法向估计、区域生长和矩形拟合路径。它不代表在线 GridMap、最终
quadrangels 或楼梯地形发送已经完成。

### 12.2 更新机器狗上的传输目录和镜像

Windows PowerShell：

```powershell
cd D:\Desktop\RoboDog
scp .\x30_livox_ros2_transfer.tar.gz ysc@192.168.1.106:/home/ysc/
```

机器狗 `192.168.1.106`：

```bash
cd /home/ysc
rm -rf x30_livox_ros2_transfer.prev
if [ -d x30_livox_ros2_transfer ]; then
  mv x30_livox_ros2_transfer x30_livox_ros2_transfer.prev
fi
tar -xzf x30_livox_ros2_transfer.tar.gz
cd /home/ysc/x30_livox_ros2_transfer
chmod +x remote/*.sh
bash remote/00_build_image.sh
```

构建完成后，先在机器狗上重复 12.1 的断网核心测试。此过程同样不需要关闭 ROS1。

### 12.3 可选的 ROS2 传感器独占测试

这个测试只验证四雷达、本体 IMU 和 `/x30/points_merged`。它不会运行
`x30_plane_seg_core`，也不会产生或发送楼梯地形数据。

开始前只临时关闭会争用雷达 UDP 端口和 IMU 串口的两个原厂驱动：

```bash
cd /home/ysc/x30_livox_ros2_transfer
bash remote/20_stop_ros2_all.sh
bash remote/02_factory_livox_off.sh
bash remote/11_factory_imu_off.sh
bash remote/18_run_ros2_all.sh

docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
bash remote/19_check_ros2_all.sh
```

测试完成后立即恢复原厂 ROS1：

```bash
cd /home/ysc/x30_livox_ros2_transfer
bash remote/20_stop_ros2_all.sh
bash remote/12_factory_imu_on.sh
bash remote/06_factory_livox_on.sh
bash remote/01_status.sh
```

这里没有关闭整套 ROS1，只关闭原厂 Livox 和 Yesense 输入驱动。测试期间原厂
下游地形节点会因为没有新的 ROS1 传感器输入而不能形成有效实时地形，因此此时
禁止进入楼梯模式或运动。

### 12.4 当前不能测试的部分

完整目标链为：

```text
Docker ROS2 雷达 + IMU
  -> LIO / 去畸变 / 重力对齐
  -> x30_local_height_map
  -> GridMap 地形采样适配
  -> x30_plane_seg_core
  -> X30 四边形后处理
  -> /x30/terrain/quadrangels
  -> x30_gridmap_sender
  -> TCP 192.168.1.103:49999
```

当前只完成了“ROS2 传感器接收和点云合成”以及“独立核心算法库”两端。中间的
LIO、局部高度图、在线输入适配和 X30 后处理仍未完成，TCP sender 也未实现。
因此现在不能把 `/x30/points_merged` 直接交给核心库，更不能向 `103:49999`
发送数据。下一开发步骤是生成配对的 GridMap 输入夹具并完成离线逐帧回归。

## 13. 2026-07-15 下一步与是否更新机器狗

### 13.1 本轮计划

```text
1. 确认传输包是否包含最新 x30_plane_seg_core。
2. 判断更新机器狗是不是继续开发的前置条件。
3. 固定更新后的安全测试边界。
```

### 13.2 实际检查结果

当前传输包：

```text
D:\Desktop\RoboDog\x30_livox_ros2_transfer.tar.gz
大小：607061 bytes
SHA256：1C9715C70FC8E5E3B2CD65538592DE37F5829CD25D745AA69592AB9EB96746ED
```

压缩包生成时间晚于当前传输目录内全部相关源码，并已确认包含：

```text
Dockerfile
remote/00_build_image.sh
ws/src/x30_plane_seg_core
x30_plane_seg_core 合约测试
离线 quadrangels 输出夹具
```

### 13.3 决策

更新机器狗不是继续开发的前置条件。当前核心仍是离线库，GridMap 输入适配尚未完成，
所以配对输入夹具、输入适配和逐帧回归都应先在 Windows/本地 Docker 完成。

可以现在把传输包推送到 106，并进行一次目标机离线构建验收，用来确认机器狗上的
Docker、编译器和依赖与本地结果一致。该验收只运行 `--network none` 合约测试，不需要
关闭 ROS1，不启动传感器容器，不连接运动主机，也不发送控制或地形数据。

### 13.4 更新后的执行顺序

```text
第一步（可立即执行）：推送传输包，在 106 构建镜像并运行断网合约测试。
第二步（真正开发主线）：从现有 bag 导出配对 GridMap elevation/accessibility/geometry/TF 输入夹具。
第三步：实现 GridMap -> x30_plane_seg_core 输入适配和逐帧回归。
第四步：补齐 X30 四边形后处理并达到原厂输出一致性。
第五步：之后才开发默认关闭、只连接本地假服务器的 TCP sender。
```

### 13.5 安全边界

此次更新机器狗只验证构建和离线测试。仍然禁止进入楼梯模式、发送非零速度、启动
自制地形发送器或连接 `192.168.1.103:49999`。

### 13.6 机器狗 106 目标机验收结果

2026-07-15 已在 `ysc-perception` 上完成最新传输包构建和断网测试：

```text
colcon：5 packages finished
新镜像：x30_livox_ros2:jezetek
镜像 ID：a0dc46b9326f
镜像大小：3.56 GB
Livox SDK master_sdk 支持：通过
x30_plane_seg_core 路径：/ws/install/x30_plane_seg_core
PCL：1.12.1
Eigen：3.4.0
```

`remote/00_build_image.sh` 内置的合约测试结果：

```text
1/1 passed
0 failed
```

随后又使用 `docker run --rm --network none` 独立运行一次相同测试，结果仍为：

```text
1/1 passed
0 failed
```

`5 packages had stderr output` 不表示失败；最终 `Summary: 5 packages finished`、镜像构建成功，
且后续验证全部通过。本轮没有启动传感器、没有关闭 ROS1、没有连接运动主机，也没有
发送步态、速度或地形数据。

目标机兼容性验证已经完成，不需要继续重复构建。下一开发步骤回到本地离线主线：导出
配对 GridMap 输入夹具，并实现 `GridMap -> x30_plane_seg_core` 输入适配和逐帧回归。

## 14. 2026-07-15 配对 GridMap 输入夹具与纯 C++ 适配器

### 14.1 本轮计划

```text
1. 对现有四组 ROS1 bag 中的 GridMap、quadrangels 和 TF 做精确时间戳配对。
2. 导出可重复、可校验、保留原始 float32 位模式的紧凑输入夹具。
3. 在 x30_plane_seg_core 内增加不依赖 ROS 的 GridMap 输入适配器。
4. 用合成测试、真实 bag 和断网容器测试验证实现。
5. 不启动在线 sender，不连接 192.168.1.103:49999，不发送任何控制命令。
```

### 14.2 实际完成

新增或更新：

```text
x30_livox_ros2_transfer/tools/analyze_x30_gridmap_baseline.py
x30_livox_ros2_transfer/tools/export_x30_plane_seg_paired_fixtures.py
x30_livox_ros2_transfer/tests/test_plane_seg_paired_fixture_exporter.py
x30_livox_ros2_transfer/tests/fixtures/plane_seg_paired/
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/include/x30_plane_seg_core/grid_map_adapter.hpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/src/grid_map_adapter.cpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/grid_map_adapter_test.cpp
```

bag 解析器现在可以在一次顺序扫描中读取：

```text
GridMap
/plane_seg/quadrangels
/plane_seg/look_pose
/tf
/tf_static
/height_map_mode
/height_map_mode_state
```

配对规则固定为：

```text
GridMap 与 quadrangels：Header.stamp 必须完全相同。
动态 TF：TransformStamped Header.stamp 必须完全相同。
静态 TF：对整个 bag 有效。
禁止用最近 TF 或未来 TF 补帧。
/plane_seg/look_pose 的 stamp=0 不作为配对键。
/height_map_mode 与 /height_map_mode_state 分开记录，不混为同一状态。
```

每个 GridMap 图层以原始 little-endian float32 blob 保存，保留 NaN 的实际位模式，
并记录 dimensions、stride、data_offset、outer_start_index、inner_start_index、
geometry、TF、原厂期望 quadrangels、来源文件和全部 SHA256。

纯 C++ 适配器输入为非拥有型 POD 视图，输出为 `std::vector<TerrainSample>`。其核心映射为：

```text
raw_index = (x + outer_start_index) % size_x
          + ((y + inner_start_index) % size_y) * size_x

local_x = length_x / 2 - (x + 0.5) * resolution
local_y = length_y / 2 - (y + 0.5) * resolution
```

XY 经过规范化 GridMap 四元数和中心位置变换。原厂 GridMap `toPointCloud` 中的
elevation 已经是绝对 Z，因此不能再次叠加 `center.z`。适配器保留全部单元格和 NaN，
现有核心过滤器再处理可访问性和非有限点。

### 14.3 真实 bag 配对结果

| bag | GridMap | quadrangels | 精确 G/Q 对 | 有精确 world->base_link TF | 缺 TF | 孤立 quadrangels |
|---|---:|---:|---:|---:|---:|---:|
| mode3_flat_20260714_142726 | 141 | 142 | 141 | 141 | 0 | 1 |
| mode3_flat_repeat_20260714_150324 | 142 | 142 | 142 | 142 | 0 | 0 |
| mode3_measured_step_h022_d029_w035_20260714_160857 | 147 | 147 | 147 | 146 | 1 | 0 |
| mode3_object_probe_20260714_153211 | 141 | 141 | 141 | 141 | 0 | 0 |

平地首包多出的第一个 quadrangels 帧没有同时间戳 GridMap，夹具不会错误配对它。
实测台阶 bag 的第一对 GridMap/quadrangels 没有同时间戳动态 TF，夹具明确保存
`missing_exact_required_dynamic_tf`，不会使用约 100 ms 后的未来 TF。

提交的紧凑夹具：

```text
4 个 bag
每个 bag 3 个均匀采样的精确 G/Q 对
共 12 帧
共 48 个图层 blob
图层原始数据 1,920,000 bytes
完整夹具树 54 个文件
```

台阶 bag 另做了临时全量导出验证：

```text
147/147 个精确 GridMap/quadrangels 对
146 个精确动态 TF
1 个明确缺失动态 TF 的首帧
588 个图层 blob
23,520,000 bytes 原始图层数据
```

### 14.4 验证结果

```text
Python unittest：21/21 passed
真实夹具独立重导出：54/54 文件 SHA256 完全一致，mismatch=0
断网 Docker CTest：2/2 passed
  x30_plane_seg_core_contract_test
  x30_plane_seg_core_grid_map_adapter_test
```

完整 `pytest` 结果为 `31 passed, 1 failed`。唯一失败是既有 ELF 离线分析测试的
可选 Python 依赖 `capstone` 未安装，与本轮 GridMap、TF、夹具或 C++ 适配器无关；
本轮相关 unittest 全部通过。

镜像构建脚本已从只运行旧 contract test 改为运行全部：

```text
^x30_plane_seg_core_.*_test$
```

### 14.5 当前数据链位置

本轮已经打通的是离线路径：

```text
记录的原厂 ROS1 GridMap + quadrangels + TF
  -> 精确配对夹具
  -> GridMap POD 视图
  -> grid_map_adapter
  -> TerrainSample
  -> x30_plane_seg_core 可接受的输入类型
```

本轮没有完成或声称完成：

```text
ROS2 点云 + IMU -> LIO / 去畸变 / 重力对齐
在线 x30_local_height_map
夹具 JSON/blob 的 C++ 回放执行器
真实夹具逐帧运行 core 后的候选结果基线
core_rectangles -> 原厂最终四边形后处理
/x30/terrain/quadrangels ROS2 发布
x30_gridmap_sender
TCP 192.168.1.103:49999
```

因此现在仍然不能在关闭原厂 ROS1 地形链后进入 45 度楼梯模式。

### 14.6 下一步

```text
1. 增加离线 fixture replay，将真实 elevation/accessibility blob 装入 GridMapInput。
2. 对四组夹具运行 GridMap adapter + PlaneSegCore，记录 retained samples 和 core candidates。
3. 固定 147 帧台阶数据的逐帧核心候选回归。
4. 继续恢复 X30 四边形后处理，比较点数、顺序和坐标，保留原厂退化首组。
5. 上述一致性完成后，才实现在线 ROS2 wrapper；TCP sender 仍保持最后一步且默认关闭。
```

### 14.7 安全边界

本轮全部为本地文件解析、离线夹具和 `--network none` 测试。没有关闭机器狗 ROS1，
没有启动传感器容器，没有发送步态、速度或地形数据，也没有连接
`192.168.1.103:49999`。更新机器狗只用于后续断网构建兼容性检查，不用于楼梯运动测试。

### 14.8 更新后的唯一传输包

已覆盖原传输包，没有新增带日期或 `single` 后缀的重复包：

```text
D:\Desktop\RoboDog\x30_livox_ros2_transfer.tar.gz
大小：785484 bytes
SHA256：97B5DCFDB1A8F08EE47C4FBD396448DA5C9E0209BBC4754AF16D82CE24C0DC5A
```

压缩包检查：

```text
总条目：621
paired fixture 目录条目：75（其中 54 个文件，其余为目录）
GridMap adapter 头文件/实现/测试：已包含
paired fixture exporter：已包含
ROS bag：0
__pycache__/.pytest_cache：0
```

机器狗上的现有镜像仍是上一轮版本。若推送本包，只允许重新构建并运行
`--network none` 的 2 个离线 CTest；不需要关闭 ROS1，也不能进入楼梯模式或发送地形数据。

## 15. 2026-07-15 真实 GridMap 确定性回放与 147 帧核心回归

### 15.1 本轮计划

```text
1. 把 paired fixture 编译成目标镜像可直接读取的自包含二进制回放包。
2. 增加严格 C++ 加载器，不在运行时依赖 JSON 或 ROS1 消息解析。
3. 对 12 帧紧凑夹具运行 GridMap adapter + PlaneSegCore。
4. 对完整 147 帧实测台阶数据运行相同回放，并固定可复现指标。
5. 明确区分“核心能够执行”和“最终四边形已与原厂一致”。
6. 全过程保持断网，不启动 sender，不发送控制命令。
```

### 15.2 实际完成

新增：

```text
x30_livox_ros2_transfer/tools/compile_x30_plane_seg_replay.py
x30_livox_ros2_transfer/tools/summarize_x30_plane_seg_replay.py
x30_livox_ros2_transfer/tests/test_plane_seg_replay_compiler.py
x30_livox_ros2_transfer/tests/test_plane_seg_replay_summary.py
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/support/replay_file.hpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/support/replay_file.cpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/support/sha256.hpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/support/sha256.cpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/replay_fixture_test.cpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/replay_format_test.cpp
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/fixtures/x30_plane_seg_replay_v1.x30rpl
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/fixtures/x30_plane_seg_replay_v1.x30rpl.sha256
x30_livox_ros2_transfer/ws/src/x30_plane_seg_core/test/fixtures/x30_plane_seg_replay_v1.metrics.json
```

回放格式固定为 little-endian、IEEE-754 的自包含二进制。加载器逐项验证：

```text
固定头和帧记录大小
完整文件和每帧输入 SHA256
所有 blob 的范围、大小、无重叠、无空洞和完整覆盖
GridMap 尺寸、分辨率、长度、循环缓冲起始索引
GridMap 与 world->base_link 四元数合法性
HAS_EXACT_TF / EXPECT_MISSING_TF 互斥
固定传感器位姿 origin=(0,0,0), look=(1,0,0)
原厂 quadrangels 点数必须为 4 的倍数
```

第一帧缺少精确 `world -> base_link` TF 仍然执行 core。原因是这批 GridMap 自身已经处于
`world` 坐标系，core 所需变换是 `world <- GridMap.frame_id`，不能误用约 100 ms 后的未来 TF。
缺失状态和 identity 占位仍被保留在回放记录中用于审计。

### 15.3 紧凑 12 帧基线

```text
回放包大小：968720 bytes
回放包 SHA256：e43835f8435b50c2088c1e5bbaef0668d4bcd1907834d87336f97ee8d2d3528d
规范 JSONL SHA256：42318c156d930052fb213e69078c66399dd806d673ef01ce6c10302ddbab909e
metrics SHA256：d19b208c65f19b8955133dc383172f4338ce80f458b08a25701069d9b113ed40
帧数：12/12
retained samples：4087..4186
core candidates：7..12，共 104
原厂 quadrangle groups：3..9，共 83
候选数量与原厂组数相等：1/12 帧
退化核心候选：0
```

CTest 直接校验完整规范 JSONL 的 SHA256。只要候选数量、尺寸、位姿、四元数或 hull
中任意一个规范值变化，测试都会失败；指标清单同时保存每帧轻量记录和逐行哈希。

### 15.4 完整 147 帧实测台阶回归

完整回放包和输出仅保存在本地 `tmp/`，不放进正式传输包：

```text
tmp/x30_plane_seg_measured_step_full_147.x30rpl
  大小：11866592 bytes
  SHA256：2084b459c830abd9bb56e2f210e01a363c757f1e3bf8ce293b3a581b00e0fbe4

tmp/x30_plane_seg_measured_step_full_147.jsonl
  SHA256：28514be409f7ce4f79be3b348c4f022903830a0dba12bc685b67de46f751bf4e

tmp/x30_plane_seg_measured_step_full_147.metrics.json
  SHA256：fe5f1c1f606a73722f569f4407129beb57af588ddd06d9ccc18e47ce1bf9e845
```

回归结果：

```text
帧数：147/147
retained samples：4067..4198
core candidates：6..14，共 1450
原厂 quadrangle groups：3..13，共 1042
候选数量与原厂组数相等：10/147 帧
退化核心候选：1
```

唯一退化候选位于：

```text
selected_index=132
stamp_ns=10428477123846
```

该帧一个 20 点平面簇的 X 坐标全部为 `0.585`，PCL/Qhull 判定输入不足二维，无法生成
凸包。现有中间核心继续执行，并产生一个 `size.x=0, size.y=0, hull=0` 的零面积候选。
本轮没有静默过滤它，也没有把它算作原厂一致；它已进入后续 X30 四边形后处理的明确
失败样本。

### 15.5 确定性与原厂一致性的边界

固定的上游 `RansacGeneric` 使用全局 `std::rand()`。不同随机种子会明显改变候选数量和
几何，因此回放执行器在每帧 core 调用前执行：

```cpp
std::srand(1);
```

这只定义“当前固定源码和镜像的确定性回归配置”，不代表原厂进程也使用种子 1。
两次独立 12 帧运行和两次独立 147 帧运行的 stdout SHA256 均完全相同。

当前结论必须写成：

```text
真实 GridMap -> adapter -> PlaneSegCore 已经逐帧可重复执行。
当前输出仍是 core_rectangles 中间候选，不是原厂最终 quadrangels。
12 帧只有 1 帧候选数量相等，147 帧只有 10 帧数量相等。
因此 X30 四边形后处理仍未完成，不能声称楼梯地形输出已经复现。
```

### 15.6 验证结果

```text
Python unittest：34/34 passed
断网 Docker CTest：4/4 passed
  x30_plane_seg_core_contract_test
  x30_plane_seg_core_grid_map_adapter_test
  x30_plane_seg_core_replay_test
  x30_plane_seg_core_replay_format_test

回放格式负向检查：
  SHA256 标准向量
  body 损坏
  文件截断
  非法 sensor pose
```

所有 Docker 验证均使用 `--network none`，镜像为 `x30_livox_ros2:jezetek`。

正式 Dockerfile 重建结果：

```text
镜像 ID：sha256:6ad9d35013e1705c3159aef440e70b237355e5fc2fae0ecb86ba86178e08ebe1
镜像大小：3520540508 bytes
5 个 ROS2 package 构建成功
镜像内部断网 CTest：4/4 passed，总耗时 51.03 s
安装目录中 replay pack、sidecar 和 metrics 均存在且哈希正确
```

### 15.7 当前链路位置和下一步

现在已完成：

```text
记录的原厂 ROS1 GridMap + quadrangels + TF
  -> 精确配对
  -> 自包含 replay pack
  -> 严格 C++ loader
  -> GridMap adapter
  -> TerrainSample
  -> PlaneSegCore
  -> 可重复的 core_rectangles + 回归指标
```

下一步仍然是纯离线工作：

```text
1. 以 12 帧和 147 帧指标为保护网，恢复 X30 core_rectangles 后处理。
2. 首先分析 selected_index=132 的退化簇和原厂同帧 5 组 quadrangels。
3. 比较原厂的筛选、排序、合并、退化首组和每四点排列规则。
4. 达到逐帧几何一致后，再封装 x30_plane_seg_ros2。
5. ROS2 点云/IMU -> LIO -> local height map 仍是另一条尚未完成的上游链。
6. TCP sender 继续保持最后一步、默认关闭，且先只允许连接本地假服务端。
```

### 15.8 安全边界

本轮没有连接机器狗、没有关闭原厂 ROS1、没有启动传感器容器，也没有发送步态、速度、
地形 TCP 或任何 UDP 控制包。没有连接 `192.168.1.103:49999`。当前产物只允许离线回放和
断网构建验证，不能用于 45 度楼梯运动测试。

### 15.9 更新后的唯一传输包

已覆盖原文件，没有新建日期版或 `single` 版：

```text
D:\Desktop\RoboDog\x30_livox_ros2_transfer.tar.gz
大小：846791 bytes
SHA256：BF76C3BA07179EE6CFC446AA088C74E3C3E158047776AA6CE79BD0DE7033EA28
```

压缩包审计：

```text
总条目：636
replay pack / sidecar / metrics：3
replay Python 工具：2
replay C++ 测试与 support 源文件：6
ROS bag：0
完整 147 帧临时产物：0
__pycache__/.pytest_cache：0
```

机器狗上的镜像尚未因本轮结果而更新。若之后上传此包，第一轮仍只允许重新构建并执行
`--network none` 的 4 个 CTest；不需要关闭 ROS1，不允许切楼梯步态、发送非零速度或连接
地形 TCP 端口。

## 16. 最终四边形后处理逐帧对齐（进行中）

### 16.1 本轮准备做什么

```text
1. 从原厂 plane_seg_ros 二进制恢复 BlockFitter::go() 之后的控制流。
2. 对比公开 plane_seg_ros 包装层与 X30 原厂包装层的差异。
3. 用 12 帧紧凑集和 147 帧完整台阶集分析候选块与原厂四点组的几何对应关系。
4. 只在证据明确后实现无 ROS 依赖的 quadrangle 后处理器。
5. 增加逐帧几何回归，不再仅比较候选数量。
```

### 16.2 本轮允许改动

```text
x30_plane_seg_core 的离线 C++ 源码与测试
离线回放/几何比较工具
README 与本阶段记录
Docker 断网构建验证和唯一传输包
```

### 16.3 本轮禁止事项

```text
不连接机器狗或 192.168.1.103:49999
不关闭原厂 ROS1 雷达、IMU 或地形节点
不启动在线传感器链
不发送步态、非零速度、地形 TCP 或控制 UDP
不把 core_rectangles 误称为最终 quadrangels
```

实际实现、回归指标、残余差异和安全结论将在本节后续小节中补充。
