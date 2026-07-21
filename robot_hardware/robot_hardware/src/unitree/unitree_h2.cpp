#include "unitree_h2.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>

namespace {

// Initial H2 live-test ceiling. YAML may lower these values but cannot raise
// them without a reviewed code change and a new offline/live acceptance round.
constexpr double kAbsoluteMaxVx = 0.20;
constexpr double kAbsoluteMaxVy = 0.10;
constexpr double kAbsoluteMaxOmega = 0.30;
constexpr float kAbsoluteMaxVelocityDurationS = 0.30f;

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
    return action == ACTION_STAND_UP || action == ACTION_PREPARE_MOTION ||
           action == ACTION_DAMP || action == ACTION_SQUAT ||
           action == ACTION_SIT;
}

} // namespace

UnitreeH2::UnitreeH2(YAML::Node config) : RobotHardwareInterface(config)
{
    network_interface_card_name_ =
        readYaml<std::string>(config, "network_interface_card_name", "");
    dds_domain_id_ = readYaml<int>(config, "dds_domain_id", dds_domain_id_);
    sdk_timeout_s_ = readYaml<float>(config, "sdk_timeout_s", sdk_timeout_s_);
    velocity_command_duration_s_ = readYaml<float>(
        config, "velocity_command_duration_s", velocity_command_duration_s_);

    max_vx_ = readYaml<double>(config, "max_vx", max_vx_);
    max_vy_ = readYaml<double>(config, "max_vy", max_vy_);
    max_omega_ = readYaml<double>(config, "max_omega", max_omega_);

    velocity_watchdog_hz_ =
        readYaml<int>(config, "velocity_watchdog_hz", velocity_watchdog_hz_);
    velocity_command_timeout_ms_ = readYaml<int>(
        config, "velocity_command_timeout_ms", velocity_command_timeout_ms_);
    velocity_zero_hold_ms_ =
        readYaml<int>(config, "velocity_zero_hold_ms", velocity_zero_hold_ms_);
    required_motion_fsm_id_ = readYaml<int>(
        config, "required_motion_fsm_id", required_motion_fsm_id_);

    verify_fsm_on_init_ =
        readYaml<bool>(config, "verify_fsm_on_init", verify_fsm_on_init_);
    allow_motion_commands_ =
        readYaml<bool>(config, "allow_motion_commands", allow_motion_commands_);
    allow_state_changing_actions_ = readYaml<bool>(
        config, "allow_state_changing_actions", allow_state_changing_actions_);
}

UnitreeH2::~UnitreeH2()
{
    watchdog_running_ = false;
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }

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
    if (is_initialized_) {
        return CMD_SUCCESS;
    }

    if (network_interface_card_name_.empty() || dds_domain_id_ < 0 ||
        !std::isfinite(sdk_timeout_s_) || sdk_timeout_s_ <= 0.0f ||
        !std::isfinite(velocity_command_duration_s_) ||
        velocity_command_duration_s_ <= 0.0f ||
        velocity_command_duration_s_ > kAbsoluteMaxVelocityDurationS ||
        !std::isfinite(max_vx_) ||
        max_vx_ <= 0.0 || !std::isfinite(max_vy_) || max_vy_ <= 0.0 ||
        !std::isfinite(max_omega_) || max_omega_ <= 0.0 ||
        max_vx_ > kAbsoluteMaxVx || max_vy_ > kAbsoluteMaxVy ||
        max_omega_ > kAbsoluteMaxOmega ||
        velocity_watchdog_hz_ <= 0 ||
        velocity_command_timeout_ms_ <= 0 || velocity_zero_hold_ms_ < 0 ||
        required_motion_fsm_id_ <= 0 ||
        (allow_motion_commands_ && !verify_fsm_on_init_)) {
        std::cerr << "[UnitreeH2] Invalid configuration." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    try {
        unitree::robot::ChannelFactory::Instance()->Init(
            dds_domain_id_, network_interface_card_name_);

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
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }
    if (!std::isfinite(cmd.vx) || !std::isfinite(cmd.vy) ||
        !std::isfinite(cmd.omega)) {
        return ERROR_ROBOT_HARDWARE_MOVE;
    }

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
    if (!is_initialized_ || !loco_client_) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (action == ACTION_STOP_MOVE) {
        const int32_t ret = sendZeroVelocityLocked();
        if (ret == CMD_SUCCESS) {
            velocity_command_active_ = false;
        }
        return ret;
    }

    if (!isSupportedStateChangingAction(action)) {
        std::cerr << "[UnitreeH2] Unsupported action rejected before SDK call: "
                  << action << std::endl;
        return ERROR_ROBOT_HARDWARE_NOT_SUPPORTED;
    }

    if (!allow_state_changing_actions_) {
        std::cerr << "[UnitreeH2] State-changing action rejected by safety interlock: "
                  << action << std::endl;
        return ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK;
    }

    if (sendZeroVelocityLocked() != CMD_SUCCESS) {
        return ERROR_ROBOT_HARDWARE_STOP_MOVE;
    }
    velocity_command_active_ = false;
    return runStateChangingActionLocked(action);
}

int32_t UnitreeH2::readFsmId(int &fsm_id)
{
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
    control_rpc_attempted_ = true;
    try {
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
    control_rpc_attempted_ = true;
    try {
        int32_t ret = 0;
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
        const auto period =
            std::chrono::milliseconds(std::max(1, 1000 / velocity_watchdog_hz_));
        const auto timeout =
            std::chrono::milliseconds(velocity_command_timeout_ms_);
        const auto zero_hold = std::chrono::milliseconds(velocity_zero_hold_ms_);

        while (watchdog_running_) {
            std::this_thread::sleep_for(period);

            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!velocity_command_active_ || !loco_client_) {
                continue;
            }

            const auto elapsed =
                std::chrono::steady_clock::now() - last_velocity_command_time_;
            if (elapsed < timeout) {
                continue;
            }

            const int32_t ret = sendZeroVelocityLocked();
            if (ret != CMD_SUCCESS) {
                std::cerr << "[UnitreeH2] Velocity watchdog failed to send StopMove; retrying."
                          << std::endl;
                continue;
            }

            // Repeat StopMove throughout the hold interval. After the interval,
            // deactivate only after at least one final successful SDK call.
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
