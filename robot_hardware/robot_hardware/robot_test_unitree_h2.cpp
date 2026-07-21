#include "robot_factory.h"
#include "unitree/unitree_h2.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

enum class ProbeMode {
    ReadOnly,
    GetterAudit,
    ZeroStop,
};

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " [config.yaml] [--read-only|--getter-audit|--zero-stop]"
              << std::endl;
}

} // namespace

int main(int argc, char *argv[])
{
    std::string config_path = "../config/unitree_h2.yaml";
    bool config_path_set = false;
    bool mode_set = false;
    ProbeMode mode = ProbeMode::ReadOnly;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--read-only") {
            if (mode_set) {
                printUsage(argv[0]);
                return 64;
            }
            mode = ProbeMode::ReadOnly;
            mode_set = true;
            continue;
        }
        if (argument == "--getter-audit") {
            if (mode_set) {
                printUsage(argv[0]);
                return 64;
            }
            mode = ProbeMode::GetterAudit;
            mode_set = true;
            continue;
        }
        if (argument == "--zero-stop") {
            if (mode_set) {
                printUsage(argv[0]);
                return 64;
            }
            mode = ProbeMode::ZeroStop;
            mode_set = true;
            continue;
        }
        if (!argument.empty() && argument.front() == '-') {
            printUsage(argv[0]);
            return 64;
        }
        if (config_path_set) {
            std::cerr << "Only one configuration path may be supplied."
                      << std::endl;
            return 64;
        }
        config_path = argument;
        config_path_set = true;
    }

    YAML::Node config = YAML::LoadFile(config_path);
    std::shared_ptr<RobotHardwareInterface> robot =
        RobotFactory::RobotAllocate(config);
    const std::shared_ptr<UnitreeH2> h2 =
        std::dynamic_pointer_cast<UnitreeH2>(robot);
    if (!h2) {
        std::cerr << "[UnitreeH2Probe] RobotFactory did not return UnitreeH2."
                  << std::endl;
        return 1;
    }

    const int32_t init_ret = robot->initRobotHardware();
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "[UnitreeH2Probe] Initialization/read-only FSM check failed: "
                  << init_ret << std::endl;
        return 1;
    }

    if (mode == ProbeMode::ReadOnly) {
        std::cout << "[UnitreeH2Probe] Read-only initialization and FSM check "
                     "passed. No control command was requested."
                  << std::endl;
        return 0;
    }

    if (mode == ProbeMode::GetterAudit) {
        int fsm_id = -1;
        int fsm_mode = -1;
        std::vector<int> ids;
        std::vector<std::string> names;
        const int32_t fsm_ret = h2->readFsmId(fsm_id);
        const int32_t mode_ret = h2->readFsmMode(fsm_mode);
        const int32_t available_ret =
            h2->readAvailableFsmIds(ids, names);
        std::cout << "H2_GETTER_AUDIT fsm_id=" << fsm_id
                  << " fsm_ret=" << fsm_ret
                  << " fsm_mode=" << fsm_mode
                  << " mode_ret=" << mode_ret
                  << " mode_value_observation_only=1"
                  << " available_ret=" << available_ret
                  << " available_count=" << ids.size();
        for (std::size_t index = 0; index < ids.size(); ++index) {
            std::cout << " [" << ids[index];
            if (index < names.size()) {
                std::cout << ":" << names[index];
            }
            std::cout << "]";
        }
        std::cout << std::endl;
        if (fsm_ret != CMD_SUCCESS || mode_ret != CMD_SUCCESS ||
            available_ret != CMD_SUCCESS) {
            std::cerr << "[UnitreeH2Probe] Getter-only RPC audit failed."
                      << std::endl;
            return 2;
        }
        std::cout << "H2_GETTER_ONLY_RPC_OK" << std::endl;
        return 0;
    }

    int fsm_before = -1;
    if (h2->readFsmId(fsm_before) != CMD_SUCCESS) {
        std::cerr << "[UnitreeH2Probe] Pre-zero FSM read failed." << std::endl;
        return 2;
    }
    RobotVelocityCommand zero_cmd{0.0, 0.0, 0.0};
    const int32_t zero_ret = robot->writeRobotVelocityCommand(zero_cmd);
    if (zero_ret != CMD_SUCCESS) {
        std::cerr << "[UnitreeH2Probe] Zero-velocity command failed: "
                  << zero_ret << std::endl;
        return 2;
    }

    const int32_t stop_ret = robot->writeActionCommand(ACTION_STOP_MOVE);
    if (stop_ret != CMD_SUCCESS) {
        std::cerr << "[UnitreeH2Probe] StopMove failed: " << stop_ret
                  << std::endl;
        return 3;
    }

    int fsm_after = -1;
    if (h2->readFsmId(fsm_after) != CMD_SUCCESS ||
        fsm_after != fsm_before) {
        std::cerr << "[UnitreeH2Probe] FSM changed or became unreadable after "
                     "zero/StopMove. before="
                  << fsm_before << " after=" << fsm_after << std::endl;
        return 4;
    }

    std::cout << "[UnitreeH2Probe] Explicit zero-velocity and StopMove contract "
                 "passed with unchanged FSM ID="
              << fsm_after
              << ". StopMove is a high-level zero command, not a hardware "
                 "emergency stop. No non-zero motion was requested."
              << std::endl;
    std::cout << "H2_ZERO_STOP_RPC_OK fsm_id=" << fsm_after << std::endl;
    return 0;
}
