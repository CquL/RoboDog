#include "robot_hardware_constant.h"

// 字符串值属于上层与 HAL 的接口契约；集中定义可防止调用方和厂商适配器各自漂移。
// UnitreeH2 只映射其明确支持的子集，其他动作在调用 SDK 前返回 NOT_SUPPORTED。

const std::string ACTION_STAND_UP = "stand_up";
const std::string ACTION_LIE_DOWN = "lie_down";
const std::string ACTION_STOP_MOVE = "stop_move";
const std::string ACTION_START_STOP_MOTION = "start_stop_motion";
const std::string ACTION_GAIT_WALK = "gait_walk";
const std::string ACTION_GAIT_L_WALK = "gait_l_walk";
const std::string ACTION_GAIT_MOUNTAIN = "gait_mountain";
const std::string ACTION_GAIT_STAIRS = "gait_stairs";
// H2: prepare_motion/damp/squat/sit -> SDK2 Start/Damp/Squat/Sit。
const std::string ACTION_PREPARE_MOTION = "prepare_motion";
const std::string ACTION_DAMP = "damp";
const std::string ACTION_SQUAT = "squat";
const std::string ACTION_SIT = "sit";
