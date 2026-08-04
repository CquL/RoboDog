// H2 HAL implementation. It maps the common interface to Unitree SDK2
// LocoClient RPCs and keeps motion disabled until configuration and FSM safety
// checks explicitly allow it.
#include "unitree_h2.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>

namespace {

// 项目允许 YAML 配置到达的绝对硬上限，不是宇树公布的 H2 额定速度上限。
// 当前公开 H2 文档和 SDK2 头文件没有给出 SetVelocity 的数值范围，因此这里只
// 表示本项目已经评审/测试过的配置边界；只读配置和分阶段探测仍可使用更小值。
constexpr double kProjectMaxVx = 1.00;
constexpr double kProjectMaxVy = 0.10;
constexpr double kProjectMaxOmega = 0.70;
constexpr float kProjectMaxVelocityDurationS = 0.30f;

// 从 YAML 读取可选键；缺失时保留类成员的安全默认值。类型转换错误由 YAML
// 异常报告给构造调用方，避免悄悄接受格式错误的配置。
template <typename T>
T readYaml(const YAML::Node &node, const std::string &key, const T &default_value)
{
    if (node && node[key]) {
        return node[key].as<T>();
    }
    return default_value;
}

bool isNonZero(const RobotVelocityCommand &cmd)
{
    // Safety classification is exact: an arbitrarily small non-zero value
    // must not bypass the motion interlock or disable the watchdog.
    return cmd.vx != 0.0 || cmd.vy != 0.0 || cmd.omega != 0.0;
}

bool isSupportedStateChangingAction(const std::string &action)
{
    // 停止动作由 writeActionCommand() 单独处理；这里只列出会改变 H2 姿态或
    // 运动 FSM、因而必须受 allow_state_changing_actions_ 门禁保护的动作。
    return action == ACTION_STAND_UP || action == ACTION_PREPARE_MOTION ||
           action == ACTION_DAMP || action == ACTION_SQUAT ||
           action == ACTION_SIT;
}

} // namespace

UnitreeH2::UnitreeH2(YAML::Node config) : RobotHardwareInterface(config)
{
    // DDS 与 SDK RPC 参数。
    network_interface_card_name_ =
        readYaml<std::string>(config, "network_interface_card_name", "");
    dds_domain_id_ = readYaml<int>(config, "dds_domain_id", dds_domain_id_);
    sdk_timeout_s_ = readYaml<float>(config, "sdk_timeout_s", sdk_timeout_s_);
    velocity_command_duration_s_ = readYaml<float>(
        config, "velocity_command_duration_s", velocity_command_duration_s_);

    // 软件速度包络。真正写入 SDK 前还会逐轴 clamp。
    max_vx_ = readYaml<double>(config, "max_vx", max_vx_);
    max_vy_ = readYaml<double>(config, "max_vy", max_vy_);
    max_omega_ = readYaml<double>(config, "max_omega", max_omega_);

    // 命令流看门狗和非零运动所需的 FSM。
    velocity_watchdog_hz_ =
        readYaml<int>(config, "velocity_watchdog_hz", velocity_watchdog_hz_);
    velocity_command_timeout_ms_ = readYaml<int>(
        config, "velocity_command_timeout_ms", velocity_command_timeout_ms_);
    velocity_zero_hold_ms_ =
        readYaml<int>(config, "velocity_zero_hold_ms", velocity_zero_hold_ms_);
    required_motion_fsm_id_ = readYaml<int>(
        config, "required_motion_fsm_id", required_motion_fsm_id_);

    // 安全许可必须由配置显式给出；默认构造值不允许非零运动和状态切换。
    verify_fsm_on_init_ =
        readYaml<bool>(config, "verify_fsm_on_init", verify_fsm_on_init_);
    allow_motion_commands_ =
        readYaml<bool>(config, "allow_motion_commands", allow_motion_commands_);
    allow_state_changing_actions_ = readYaml<bool>(
        config, "allow_state_changing_actions", allow_state_changing_actions_);
}

UnitreeH2::~UnitreeH2()
{
    // 先停止并回收看门狗，避免其与下面的析构 StopMove 同时使用 LocoClient。
    watchdog_running_ = false;
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }

    // RPC 超时或抛异常不代表机器人一定未收到请求。只要对象生命周期内曾尝试
    // 控制，就在退出时尽力停止；析构函数不能把失败继续抛给上层。
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (is_initialized_ && loco_client_ && control_rpc_attempted_) {
        try {
            const int32_t ret = loco_client_->StopMove();
            if (ret != 0) {
                std::cerr << "[UnitreeH2] Best-effort shutdown StopMove returned vendor code: "
                          << ret << std::endl;
            }
        } catch (const std::exception &error) {
            std::cerr << "[UnitreeH2] Best-effort shutdown StopMove threw: "
                      << error.what() << std::endl;
        } catch (...) {
            std::cerr << "[UnitreeH2] Best-effort shutdown StopMove threw an unknown exception."
                      << std::endl;
        }
    }
}

int32_t UnitreeH2::initRobotHardware()
{
    // 初始化具有幂等性：同一对象重复调用不会重建 DDS/SDK 客户端或线程。
    if (is_initialized_) {
        return CMD_SUCCESS;
    }

    // 在接触厂商网络前一次性验证配置。运动许可开启时强制要求初始化 FSM
    // 检查，防止配置通过关闭状态读取而绕过运动门禁。
    if (network_interface_card_name_.empty() || dds_domain_id_ < 0 ||
        !std::isfinite(sdk_timeout_s_) || sdk_timeout_s_ <= 0.0f ||
        !std::isfinite(velocity_command_duration_s_) ||
        velocity_command_duration_s_ <= 0.0f ||
        velocity_command_duration_s_ > kProjectMaxVelocityDurationS ||
        !std::isfinite(max_vx_) ||
        max_vx_ <= 0.0 || !std::isfinite(max_vy_) || max_vy_ <= 0.0 ||
        !std::isfinite(max_omega_) || max_omega_ <= 0.0 ||
        max_vx_ > kProjectMaxVx || max_vy_ > kProjectMaxVy ||
        max_omega_ > kProjectMaxOmega ||
        velocity_watchdog_hz_ <= 0 ||
        velocity_command_timeout_ms_ <= 0 || velocity_zero_hold_ms_ < 0 ||
        required_motion_fsm_id_ <= 0 ||
        (allow_motion_commands_ && !verify_fsm_on_init_)) {
        std::cerr << "[UnitreeH2] Invalid configuration." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    try {
        // SDK2 通信入口：在指定 DDS Domain 和 H2 有线网卡上初始化通道。
        unitree::robot::ChannelFactory::Instance()->Init(
            dds_domain_id_, network_interface_card_name_);

        // LocoClient 是 H2 高层运动 RPC 客户端；SetTimeout 控制 RPC 等待时间，
        // 不表示机器人的运动持续时间。
        loco_client_ = std::make_unique<unitree::robot::h2::LocoClient>();
        loco_client_->Init();
        loco_client_->SetTimeout(sdk_timeout_s_);

    } catch (const std::exception &e) {
        std::cerr << "[UnitreeH2] SDK initialization failed: " << e.what()
                  << std::endl;
        loco_client_.reset();
        return ERROR_ROBOT_HARDWARE_INIT;
    } catch (...) {
        std::cerr << "[UnitreeH2] SDK initialization failed with an unknown exception."
                  << std::endl;
        loco_client_.reset();
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    if (verify_fsm_on_init_) {
        // 只读连接探测：确认 PC1 的原厂运动服务可响应 GetFsmId。若当前配置
        // 允许非零速度，还要求 FSM 精确等于 required_motion_fsm_id_。
        int fsm_id = -1;
        try {
            const int32_t ret = loco_client_->GetFsmId(fsm_id);
            if (ret != 0) {
                std::cerr << "[UnitreeH2] H2 locomotion service did not return FSM state; vendor code: "
                          << ret << std::endl;
                loco_client_.reset();
                return ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY;
            }
            if (allow_motion_commands_ &&
                fsm_id != required_motion_fsm_id_) {
                std::cerr << "[UnitreeH2] Motion-enabled initialization rejected FSM ID "
                          << fsm_id << "; required "
                          << required_motion_fsm_id_ << std::endl;
                loco_client_.reset();
                return ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY;
            }
        } catch (const std::exception &error) {
            std::cerr << "[UnitreeH2] H2 FSM readiness check threw: "
                      << error.what() << std::endl;
            loco_client_.reset();
            return ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY;
        } catch (...) {
            std::cerr << "[UnitreeH2] H2 FSM readiness check threw an unknown exception."
                      << std::endl;
            loco_client_.reset();
            return ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY;
        }
        std::cout << "[UnitreeH2] Connected read-only check passed, FSM ID="
                  << fsm_id << std::endl;
    }

    try {
        // 看门狗在初始化成功后始终运行；只读模式下没有活动速度命令，因此
        // 它不会发出控制 RPC。
        watchdog_running_ = true;
        watchdog_thread_ = std::thread(&UnitreeH2::watchdogLoop, this);
    } catch (const std::exception &error) {
        watchdog_running_ = false;
        loco_client_.reset();
        std::cerr << "[UnitreeH2] Failed to start velocity watchdog: "
                  << error.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    } catch (...) {
        watchdog_running_ = false;
        loco_client_.reset();
        std::cerr << "[UnitreeH2] Failed to start velocity watchdog with an unknown exception."
                  << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }
    is_initialized_ = true;

    std::cout << "[UnitreeH2] Initialized on DDS interface "
              << network_interface_card_name_
              << ". allow_motion_commands=" << std::boolalpha
              << allow_motion_commands_
              << ", allow_state_changing_actions="
              << allow_state_changing_actions_ << std::noboolalpha
              << std::endl;

    return CMD_SUCCESS;
}

int32_t UnitreeH2::writeRobotVelocityCommand(RobotVelocityCommand &cmd)
{
    // 未初始化对象和非有限输入均在进入 SDK 前拒绝。
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }
    if (!std::isfinite(cmd.vx) || !std::isfinite(cmd.vy) ||
        !std::isfinite(cmd.omega)) {
        return ERROR_ROBOT_HARDWARE_MOVE;
    }

    // 对每个轴独立限幅。上层传入对象本身不被修改，safe_cmd 才是 SDK 输入。
    RobotVelocityCommand safe_cmd{
        std::clamp(cmd.vx, -max_vx_, max_vx_),
        std::clamp(cmd.vy, -max_vy_, max_vy_),
        std::clamp(cmd.omega, -max_omega_, max_omega_),
    };

    const bool nonzero = isNonZero(safe_cmd);
    if (nonzero && !allow_motion_commands_) {
        std::cerr << "[UnitreeH2] Non-zero velocity rejected by safety interlock."
                  << std::endl;
        return ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (nonzero) {
        // 非零速度不仅在初始化时检查 FSM，而且每一条命令都重新读取，防止遥控
        // 或原厂服务在运行中切换状态后继续接受旧控制流。
        const int32_t fsm_result = verifyMotionFsmLocked();
        if (fsm_result != CMD_SUCCESS) {
            // No velocity was sent, but explicitly request a stop because the
            // caller attempted motion while the robot state was not ready.
            sendZeroVelocityLocked();
            velocity_command_active_ = false;
            return fsm_result;
        }
        // Arm the watchdog before the RPC. A response error/exception cannot
        // prove that the robot did not receive and execute the request.
        velocity_command_active_ = true;
        last_velocity_command_time_ = std::chrono::steady_clock::now();
    }
    const int32_t ret = sendVelocityLocked(safe_cmd);
    if (ret != CMD_SUCCESS) {
        // SetVelocity 返回错误或抛异常时处于“可能已经投递”的不确定状态。
        // 立即尝试 StopMove；若停止也失败，保留 active 标志让看门狗继续重试。
        if (nonzero) {
            const int32_t stop_result = sendZeroVelocityLocked();
            if (stop_result == CMD_SUCCESS) {
                velocity_command_active_ = false;
            } else {
                std::cerr << "[UnitreeH2] SetVelocity failed in an uncertain delivery state and the immediate StopMove also failed; watchdog remains armed."
                          << std::endl;
            }
        }
        return ret;
    }

    velocity_command_active_ = nonzero;
    last_velocity_command_time_ = std::chrono::steady_clock::now();
    return CMD_SUCCESS;
}

int32_t UnitreeH2::verifyMotionFsmLocked()
{
    // 厂商返回码为 0 且状态值匹配，两个条件同时成立才允许非零运动。
    int fsm_id = -1;
    try {
        const int32_t ret = loco_client_->GetFsmId(fsm_id);
        if (ret == 0 && fsm_id == required_motion_fsm_id_) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] Per-command motion FSM gate rejected: vendor_code="
                  << ret << ", fsm_id=" << fsm_id
                  << ", required_fsm_id=" << required_motion_fsm_id_
                  << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] Per-command GetFsmId threw: "
                  << error.what() << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] Per-command GetFsmId threw an unknown exception."
                  << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY;
}

int32_t UnitreeH2::writeActionCommand(std::string action)
{
    // 所有动作共用与速度相同的初始化门槛和互斥锁。
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (action == ACTION_STOP_MOVE) {
        // stop_move 是安全动作，不受状态动作许可开关限制。
        const int32_t ret = sendZeroVelocityLocked();
        if (ret == CMD_SUCCESS) {
            velocity_command_active_ = false;
        }
        return ret;
    }

    if (!isSupportedStateChangingAction(action)) {
        // 未知动作和 H2 未实现动作必须在任何 SDK 调用前拒绝，禁止猜测性映射。
        std::cerr << "[UnitreeH2] Unsupported action rejected before SDK call: "
                  << action << std::endl;
        return ERROR_ROBOT_HARDWARE_NOT_SUPPORTED;
    }

    if (!allow_state_changing_actions_) {
        std::cerr << "[UnitreeH2] State-changing action rejected by safety interlock: "
                  << action << std::endl;
        return ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK;
    }

    // StandUp/Start/Damp/Squat/Sit 前先停止速度流。只有 StopMove 成功后才执行
    // 状态动作，避免速度 RPC 与姿态/FSM 切换并发。
    if (sendZeroVelocityLocked() != CMD_SUCCESS) {
        return ERROR_ROBOT_HARDWARE_STOP_MOVE;
    }
    velocity_command_active_ = false;
    return runStateChangingActionLocked(action);
}

int32_t UnitreeH2::readFsmId(int &fsm_id)
{
    // Getter 只读原厂状态；厂商非零返回或异常统一转换为 STATE_STALE。
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    try {
        const int32_t ret = loco_client_->GetFsmId(fsm_id);
        if (ret == 0) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] GetFsmId returned vendor code: " << ret
                  << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] GetFsmId threw: " << error.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] GetFsmId threw an unknown exception."
                  << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_STATE_STALE;
}

int32_t UnitreeH2::readFsmMode(int &fsm_mode)
{
    // fsm_mode 的语义由当前原厂固件定义，本适配层只透传数值，不自行解释。
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    try {
        const int32_t ret = loco_client_->GetFsmMode(fsm_mode);
        if (ret == 0) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] GetFsmMode returned vendor code: " << ret
                  << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] GetFsmMode threw: " << error.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] GetFsmMode threw an unknown exception."
                  << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_STATE_STALE;
}

int32_t UnitreeH2::readAvailableFsmIds(
    std::vector<int> &ids, std::vector<std::string> &names)
{
    // 同时读取可用 FSM ID 及原厂名称；两个 vector 的一致性由调用方诊断，
    // 本层只负责把 SDK 调用结果规范化。
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    try {
        const int32_t ret = loco_client_->GetAvailableFsmIds(ids, names);
        if (ret == 0) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] GetAvailableFsmIds returned vendor code: "
                  << ret << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] GetAvailableFsmIds threw: " << error.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] GetAvailableFsmIds threw an unknown exception."
                  << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_STATE_STALE;
}

int32_t UnitreeH2::sendVelocityLocked(const RobotVelocityCommand &cmd)
{
    // 从这一刻开始，即使 SDK 返回错误也按“机器人可能已收到”处理。
    control_rpc_attempted_ = true;
    try {
        // 项目统一 double 输入转换为 SDK2 的 float，参数顺序为
        // (vx, vy, omega, 单条厂商命令持续时间)。
        const int32_t ret = loco_client_->SetVelocity(
            static_cast<float>(cmd.vx), static_cast<float>(cmd.vy),
            static_cast<float>(cmd.omega), velocity_command_duration_s_);
        if (ret == 0) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] SetVelocity returned vendor code: " << ret
                  << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] SetVelocity threw: " << error.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] SetVelocity threw an unknown exception."
                  << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_MOVE;
}

int32_t UnitreeH2::sendZeroVelocityLocked()
{
    // 本项目的停止语义直接映射为原厂 StopMove，而不是仅发送一帧 0 速度。
    control_rpc_attempted_ = true;
    try {
        const int32_t ret = loco_client_->StopMove();
        if (ret == 0) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] StopMove returned vendor code: " << ret
                  << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] StopMove threw: " << error.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] StopMove threw an unknown exception."
                  << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_STOP_MOVE;
}

int32_t UnitreeH2::runStateChangingActionLocked(const std::string &action)
{
    // 到达这里前，动作已通过白名单和安全许可门禁，并且 StopMove 已成功。
    control_rpc_attempted_ = true;
    try {
        int32_t ret = 0;
        // 项目动作字符串到 H2 SDK2 高层动作 RPC 的唯一映射。
        if (action == ACTION_STAND_UP) {
            ret = loco_client_->StandUp();
        } else if (action == ACTION_PREPARE_MOTION) {
            ret = loco_client_->Start();
        } else if (action == ACTION_DAMP) {
            ret = loco_client_->Damp();
        } else if (action == ACTION_SQUAT) {
            ret = loco_client_->Squat();
        } else if (action == ACTION_SIT) {
            ret = loco_client_->Sit();
        } else {
            return ERROR_ROBOT_HARDWARE_NOT_SUPPORTED;
        }

        if (ret == 0) {
            return CMD_SUCCESS;
        }
        std::cerr << "[UnitreeH2] Action " << action
                  << " returned vendor code: " << ret << std::endl;
    } catch (const std::exception &error) {
        std::cerr << "[UnitreeH2] Action " << action
                  << " threw: " << error.what() << std::endl;
    } catch (...) {
        std::cerr << "[UnitreeH2] Action " << action
                  << " threw an unknown exception." << std::endl;
    }
    return ERROR_ROBOT_HARDWARE_ACTION_FAILED;
}

void UnitreeH2::watchdogLoop() noexcept
{
    try {
        // 使用 steady_clock 避免系统时间校准导致命令过期判断倒退或跳变。
        const auto period =
            std::chrono::milliseconds(std::max(1, 1000 / velocity_watchdog_hz_));
        const auto timeout =
            std::chrono::milliseconds(velocity_command_timeout_ms_);
        const auto zero_hold = std::chrono::milliseconds(velocity_zero_hold_ms_);

        while (watchdog_running_) {
            std::this_thread::sleep_for(period);

            // 与正常速度/动作入口共用同一把锁，保证 StopMove 不会和 SetVelocity
            // 并发访问 LocoClient。
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!velocity_command_active_ || !loco_client_) {
                continue;
            }

            const auto elapsed =
                std::chrono::steady_clock::now() - last_velocity_command_time_;
            if (elapsed < timeout) {
                continue;
            }

            // 超时后每个看门狗周期都尝试 StopMove。失败不会清除 active 标志，
            // 下一个周期继续重试。
            const int32_t ret = sendZeroVelocityLocked();
            if (ret != CMD_SUCCESS) {
                std::cerr << "[UnitreeH2] Velocity watchdog failed to send StopMove; retrying."
                          << std::endl;
                continue;
            }

            // 在 zero_hold 窗口内重复 StopMove；超过窗口且本次调用成功后才解除
            // active 状态，确保至少有一次已确认成功的停止 RPC。
            if (elapsed > timeout + zero_hold) {
                velocity_command_active_ = false;
            }
        }
    } catch (const std::exception &error) {
        watchdog_running_ = false;
        std::cerr << "[UnitreeH2] Velocity watchdog stopped after exception: "
                  << error.what() << std::endl;
    } catch (...) {
        watchdog_running_ = false;
        std::cerr << "[UnitreeH2] Velocity watchdog stopped after an unknown exception."
                  << std::endl;
    }
}
