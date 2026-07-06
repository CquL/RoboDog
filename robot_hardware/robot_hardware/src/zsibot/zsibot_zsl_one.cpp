#include "zsibot_zsl_one.h"

using namespace std;
using namespace mc_sdk::zsl_1;

ZsibotZslOne::ZsibotZslOne(YAML::Node config) : RobotHardwareInterface(config) {
    try {
        dog_ip_ = config["dog_ip"].as<std::string>();
        client_ip_ = config["client_ip"].as<std::string>();
        client_port_ = config["client_port"].as<int>();
        //# 机器狗网络配置
        //dog_ip: "192.168.168.168"
        //client_ip: "192.168.168.19"
        //client_port: 43988
        std::cout << "[ZsibotZslOne] Config Loaded: DogIP=" << dog_ip_ 
                  << ", ClientIP=" << client_ip_ 
                  << ", Port=" << client_port_ << std::endl;
    }catch (const std::exception& e) {
        std::cerr << "[ZsibotZslOne] YAML Config Error: " << e.what() << std::endl;
    }
}

/**
 * @brief 辅助函数：将 SDK 的站立错误码转换为可读信息并返回相应状态
 */
int32_t ZsibotZslOne::parseActionResult(uint32_t ret) {
    if (ret == 0) return CMD_SUCCESS;

    std::string error_msg;
    switch (ret) {
        case 0x3012: error_msg = "电机数据丢失 (Motor data lost)"; break;
        case 0x3010: error_msg = "电机失能 (Motor disabled)"; break;
        case 0x3011: error_msg = "电机故障 (Motor fault)"; break;
        case 0x3009: error_msg = "电机角度超限 (Motor angle out of range)"; break;
        case 0x3007: error_msg = "状态机切换失败 (FSM switch failed)"; break;
        case 0x3013: error_msg = "速度命令过大 (Velocity command too large)"; break;
        default:     error_msg = "未知硬件错误 (Unknown hardware error)"; break;
    }
    
    std::cerr << "[ZsibotZslOne] standUp failed! Error Code: 0x" << std::hex << ret 
              << " | Message: " << error_msg << std::dec << std::endl;
    return ERROR_ROBOT_HARDWARE_ACTION_FAILED; // 返回对应的错误码
}

int32_t ZsibotZslOne::initRobotHardware()
{
    try {
        highlevel_.initRobot(client_ip_, client_port_, dog_ip_);
        is_initialized_ = true;
    } catch (const std::exception& e) {
        is_initialized_ = false;
        std::cerr << "[ZsibotZslOne] Failed to init robot hardware SDK: " << e.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    int timeout_count = 0;
    while(highlevel_.getCurrentCtrlmode() == CTRLMODE_MOTOR_DAMPING && timeout_count < 20) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeout_count++;
    }

    std::cout << "[ZsibotZslOne] Robot hardware initialized successfully." << std::endl;
    return CMD_SUCCESS;
}

int32_t ZsibotZslOne::standUp()
{
    try {
        uint32_t stand_ret = highlevel_.standUp();
            
        // 处理 standUp 的即时返回值
        if (stand_ret != 0) {
            return parseActionResult(stand_ret);
        }
        
        int stand_timeout = 0;
        while(highlevel_.getCurrentCtrlmode() != CTRLMODE_STAND_UP){
            stand_timeout++;
            if (stand_timeout > 100) { // 超时10秒（100×100ms）
                std::cerr << "[ZsibotZslOne] Stand up timeout (10s)!" << std::endl;
                return ERROR_ROBOT_HARDWARE_STAND_UP;
            }
            std::cout << "[ZsibotZslOne] waiting for standing... "
                        << "Ctrlmode: " << highlevel_.getCurrentCtrlmode() 
                        << " | Elapsed: " << stand_timeout * 100 << "ms" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms检测一次，更精准
        }
        std::cout << "[ZsibotZslOne] Robot stand successfully. Ctrlmode: " << highlevel_.getCurrentCtrlmode() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ZsibotZslOne] Failed to stand up: " << e.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_STAND_UP;
    }

    return CMD_SUCCESS;
}

int32_t ZsibotZslOne::lieDown()
{
    try {
        //uint32_t lie_down_ret = highlevel_.lieDown();
        uint32_t lie_down_ret = highlevel_.passive();
        if (lie_down_ret != 0) {
            return parseActionResult(lie_down_ret);
        }
    } catch (const std::exception& e) {
        std::cerr << "[ZsibotZslOne] Failed to lie down: " << e.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_LIE_DOWN;
    }
    return CMD_SUCCESS;
}

int32_t ZsibotZslOne::writeActionCommand(std::string action)
{
    if (!is_initialized_) {
        std::cerr << "[ZsibotZslOne] Error: Robot not initialized." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }
    if (action == ACTION_STAND_UP) {
        return standUp();
    } else if (action == ACTION_LIE_DOWN) {
        return lieDown();
    }
    else
    {

    }

    return CMD_SUCCESS;
}

int32_t ZsibotZslOne::writeRobotVelocityCommand(RobotVelocityCommand &cmd)
{
    if (!is_initialized_) {
        std::cerr << "[ZsibotZslOne] Error: Robot not initialized." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }
    // 调用 SDK 的 move 方法
    // move(float vx, float vy, float yaw_rate)
    // vx: 前后 (m/s)
    // vy: 左右 (m/s)
    // yaw_rate: 旋转 (rad/s)
    int ret = highlevel_.move(static_cast<float>(cmd.vx), 
                              static_cast<float>(cmd.vy), 
                              static_cast<float>(cmd.omega));
    return CMD_SUCCESS;
}