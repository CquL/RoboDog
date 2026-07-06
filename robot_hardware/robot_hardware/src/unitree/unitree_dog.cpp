#include "unitree_dog.h"

UnitreeDog::UnitreeDog(YAML::Node config) : RobotHardwareInterface(config) {
     try {
        network_interface_card_name_ = config["network_interface_card_name"].as<std::string>();
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_card_name_.c_str());

        sport_client_ = std::make_unique<unitree::robot::b2::SportClient>();

        timeout_ = config["timeout"].as<float>();
        

        option_list_ = {
            {"damp", 0},
            {"balance_stand", 1},
            {"stop_move", 2},
            {"stand_down", 3},
            {"recovery_stand", 4},
            {"move", 5},
            {"switch_gait", 6},
            {"speed_level", 7},
            {"hand_stand", 8},
            {"auto_recovery_set", 9},
            {"free_walk", 11},
            {"classic_walk", 12},
            {"fast_walk", 13},
            {"euler", 14},
        };
        is_initialized_ = true;
    } catch (const std::exception& e) {
        std::cerr << "[UnitreeDog] Failed to init robot hardware SDK: " << e.what() << std::endl;
        is_initialized_ = false;
    }
}

int32_t UnitreeDog::initRobotHardware()
{
    if(is_initialized_) {
        sport_client_->SetTimeout(timeout_);
        sport_client_->Init();
        return CMD_SUCCESS;
    }
    else
    {
        return ERROR_ROBOT_HARDWARE_INIT;
    }
}

int32_t UnitreeDog::writeRobotVelocityCommand(RobotVelocityCommand &cmd)
{
    if (!is_initialized_) {
        std::cerr << "[UnitreeDog] Error: Robot not initialized." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }
    int ret = sport_client_->Move(static_cast<float>(cmd.vx), 
                                static_cast<float>(cmd.vy), 
                                static_cast<float>(cmd.omega));
    if (ret == 0) {
        return CMD_SUCCESS;
    }
    else
    {
        cout << "宇树狗移动失败，返回值："<<ret<<endl;
        return ERROR_ROBOT_HARDWARE_MOVE;
    }
}

int32_t UnitreeDog::writeActionCommand(std::string action)
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
    else if (action == ACTION_STOP_MOVE) {
        return stopMove();
    }
    else
    {

    }

    return CMD_SUCCESS;
}

int UnitreeDog::ConvertToInt(const std::string &str)
{
    try
    {
        std::stoi(str);
        return std::stoi(str);
    }
    catch (const std::invalid_argument &)
    {
        return -1;
    }
    catch (const std::out_of_range &)
    {
        return -1;
    }
}

int32_t UnitreeDog::standUp()
{
    try {
        int32_t res = sport_client_->BalanceStand();
        if (res == 0) {
            return CMD_SUCCESS;
        }
        else
        {
            cout << "宇树狗站起来失败，返回值："<<res<<endl;
            return ERROR_ROBOT_HARDWARE_STAND_UP;
        }
    } catch (const std::exception& e) {
        std::cerr << "[UnitreeDog] Failed to stand up: " << e.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_STAND_UP;
    }
}

int32_t UnitreeDog::lieDown()
{
    try {
        int32_t res = sport_client_->Damp();
        if (res == 0) {
            return CMD_SUCCESS;
        }
        else
        {
            cout << "宇树狗躺下失败，返回值："<<res<<endl;
            return ERROR_ROBOT_HARDWARE_LIE_DOWN;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[UnitreeDog] Failed to lie down: " << e.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_LIE_DOWN;
    }
}

int32_t UnitreeDog::stopMove()
{
    try {
        int32_t res = sport_client_->StopMove();
        if (res == 0) {
            return CMD_SUCCESS;
        }
        else
        {
            cout << "宇树狗停止移动失败，返回值："<<res<<endl;
            return ERROR_ROBOT_HARDWARE_STOP_MOVE;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[UnitreeDog] Failed to stop move: " << e.what() << std::endl;
        return ERROR_ROBOT_HARDWARE_STOP_MOVE;
    }
}