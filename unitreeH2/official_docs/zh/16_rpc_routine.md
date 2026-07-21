# H2 RPC 例程

本例程通过高层运控的 RPC 接口，实现了 H2 机器人的模式切换，以及站立、运动、手臂SDK开关等控制。

!!! note
LOCO_SERVICE_NAME = "sport"
!!!

程序源代码位于 `unitree_sdk2/example/h2/high_level/h2_loco_client_example.cpp`。运行方式与快速开发类似，但无需进入调试模式。

## 例程解析

## 主要逻辑

通过在启动时指定启动参数，可以实现不同的控制效果。支持同时传入多个参数，程序将依次处理每个命令。

## 参数说明

| 参数 | 说明 | 赋值示例 |
| --- | --- | :---: |
| `--network_interface` | 指定通信的网卡名 | `enp3s0` |
| `--get_fsm_id` | 获取状态机ID | / |
| `--get_fsm_mode` | 获取状态机模式 | / |
| `--get_arm_sdk_status` | 获取手臂SDK开关状态 | / |
| `--set_fsm_id` | 设置状态机ID | `4` |
| `--set_velocity` | 设置运动速度 `[vx vy omega duration(可选)]` | `"0.5 0 0 1"` |
| `--set_punch_api` | 设置拳击API浮点参数序列 | `"0.1 0.2 0.3"` |
| `--enable_arm_sdk` | 启用手臂SDK控制（关闭内置手臂服务） | 暂不可用 |
| `--disable_arm_sdk` | 禁用手臂SDK控制（恢复内置手臂服务） | 暂不可用 |
| `--damp` | 进入阻尼模式 | / |
| `--start` | 进入主运控 | / |
| `--squat` | 蹲下 | / |
| `--sit` | 落座 | / |
| `--stand_up` | 站立 | / |
| `--zero_torque` | 零力矩模式 | / |
| `--stop_move` | 停止运动 | / |
| `--move` | 以一定速度运动 `[vx vy omega]` | `"0.5 0 0"` |

## 启动示例

在 $x$ 方向以 $0.5\,\text{m/s}$ 的速度运动 $1\,\text{s}$：

```bash
./h2_loco_client --network_interface=enp3s0 --set_velocity="0.5 0 0 1"
```

查询当前可用状态机列表：

```bash
./h2_loco_client --network_interface=enp3s0 --get_available_fsm_ids
```

启用手臂SDK（关闭内置手臂服务）后再禁用：

```bash
./h2_loco_client --network_interface=enp3s0 --enable_arm_sdk
./h2_loco_client --network_interface=enp3s0 --disable_arm_sdk
```

## 代码解析

### 解析参数

如果参数带有值，格式为 `--${key}=${value}` 或 `--${key}="${value}"`;
如果参数没有值，格式为 `--${key}`。

```cpp
  std::map<std::string, std::string> args = {{"network_interface", "eth0"}};

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.substr(0, 2) == "--") {
      size_t pos = arg.find("=");
      std::string key, value;
      if (pos != std::string::npos) {
        key = arg.substr(2, pos - 2);
        value = arg.substr(pos + 1);
        if (value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.length() - 2);
        }
      } else {
        key = arg.substr(2);
        value = "";
      }
      args[key] = value;
    }
  }
```

### 初始化 DDS 通信实例

```cpp
unitree::robot::ChannelFactory::Instance()->Init(0, args["network_interface"]);
```

### 初始化上层控制的客户端

```cpp
unitree::robot::h2::LocoClient client;
client.Init();
client.SetTimeout(10.f);
```

### 响应命令

逐个解析命令行参数，并通过调用相应的 API 实现对机器人的上层控制。

```cpp
for (const auto& arg_pair : args) {
  if (arg_pair.first == "network_interface") continue;

  if (arg_pair.first == "get_fsm_id") {
    int fsm_id;
    client.GetFsmId(fsm_id);
    std::cout << "current fsm_id: " << fsm_id << std::endl;
  }

  if (arg_pair.first == "get_arm_sdk_status") {
    bool arm_sdk_status;
    client.GetArmSdkStatus(arm_sdk_status);
    std::cout << "current arm_sdk_status: "
              << (arm_sdk_status ? "enabled" : "disabled") << std::endl;
  }

  if (arg_pair.first == "get_available_fsm_ids") {
    std::vector<int> ids;
    std::vector<std::string> names;
    client.GetAvailableFsmIds(ids, names);
    for (size_t i = 0; i < ids.size(); i++) {
      std::cout << "  " << ids[i] << ": " << names[i] << std::endl;
    }
  }

  if (arg_pair.first == "set_punch_api") {
    std::vector<float> punch_api = stringToFloatVector(arg_pair.second);
    client.SetPunchApi(punch_api);
  }

  if (arg_pair.first == "enable_arm_sdk") {
    client.EnableArmSDK();
  }

  if (arg_pair.first == "disable_arm_sdk") {
    client.DisableArmSDK();
  }
  
  if (arg_pair.first == "switch_move_mode") {
    if (arg_pair.second == "true") {
      client.SwitchMoveMode(true);
    } else if (arg_pair.second == "false") {
      client.SwitchMoveMode(false);
    } else {
      std::cerr << "invalid argument: " << arg_pair.second << std::endl;
      return 1;
    }
  }

  if (arg_pair.first == "move") {
    std::vector<float> param = stringToFloatVector(arg_pair.second);
    if (param.size() != 3) {
      std::cerr << "Invalid param size for Move: " << param.size() << std::endl;
      return 1;
    }
    client.Move(param[0], param[1], param[2]);
  }
  // ... 其余命令处理见源代码
}
```

## H2 相对 G1 新增接口说明

| 接口 | API ID | 说明 |
| --- | --- | --- |
| `GetArmSdkStatus(bool&)` | 7007 | 查询当前手臂SDK开关状态 |
| `GetAvailableFsmIds(vector<int>&, vector<string>&)` | 7008 | 查询当前固件支持的状态机ID及名称 |
| `SetPunchApi(const vector<float>&)` | 7108 | 发送拳击控制参数序列 |
| `SetArmSdkStatus(bool)` | 7109 | 设置手臂SDK开关（true=启用，false=禁用） |
| `EnableArmSDK()` | — | 高层封装，等价于 `SetArmSdkStatus(true)` |
| `DisableArmSDK()` | — | 高层封装，等价于 `SetArmSdkStatus(false)` |
