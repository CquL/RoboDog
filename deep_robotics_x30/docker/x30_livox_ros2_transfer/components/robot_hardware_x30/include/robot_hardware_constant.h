#ifndef ROBOT_HARDWARE_CONSTANT_H
#define ROBOT_HARDWARE_CONSTANT_H

// RobotHardwareInterface 接受的通用动作词表。具体适配器只映射厂商 API 明确
// 支持的动作，其余动作应拒绝。

#include <string>

// Posture and stop actions. H2 maps stand_up/stop_move to StandUp/StopMove；
// lie_down 在 H2 中明确返回 NOT_SUPPORTED，绝不猜测映射为 Damp。
extern const std::string ACTION_STAND_UP;
extern const std::string ACTION_LIE_DOWN;
extern const std::string ACTION_STOP_MOVE;
extern const std::string ACTION_START_STOP_MOTION;

// X30 适配器和统一实机测试使用的移动步态请求。
extern const std::string ACTION_GAIT_WALK;
extern const std::string ACTION_GAIT_L_WALK;
extern const std::string ACTION_GAIT_MOUNTAIN;
extern const std::string ACTION_GAIT_STAIRS;

// 为支持这些动作的其他适配器保留的跨平台动作。
// H2 maps prepare_motion/damp/squat/sit to SDK2 Start/Damp/Squat/Sit；这些
// 状态动作还必须通过 allow_state_changing_actions 安全门禁。
extern const std::string ACTION_PREPARE_MOTION;
extern const std::string ACTION_DAMP;
extern const std::string ACTION_SQUAT;
extern const std::string ACTION_SIT;
#endif
