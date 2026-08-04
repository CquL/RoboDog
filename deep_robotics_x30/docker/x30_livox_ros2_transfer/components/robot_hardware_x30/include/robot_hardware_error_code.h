#ifndef _ROBOT_HARDWARE_ERROR_CODE_H_
#define _ROBOT_HARDWARE_ERROR_CODE_H_

// 抽象接口和具体适配器共用的返回码。0 表示成功，正值表示 HAL 失败类别，
// -1 表示通用命令传输失败。

#define CMD_SUCCESS 0 //代表命令执行成功

#define ERROR_ROBOT_HARDWARE_INIT 1001 //机器人硬件初始化错误
#define ERROR_ROBOT_HARDWARE_STAND_UP 1002 //机器人站立错误
#define ERROR_ROBOT_HARDWARE_LIE_DOWN 1003 //机器人趴下错误
#define ERROR_ROBOT_HARDWARE_ACTION_FAILED 1004 //机器人动作执行失败
#define ERROR_ROBOT_HARDWARE_MOVE 1005 //机器人移动错误
#define ERROR_ROBOT_HARDWARE_STOP_MOVE 1006 //机器人停止移动错误
#define ERROR_ROBOT_HARDWARE_NOT_SUPPORTED 1007 //当前机器人不支持该命令
#define ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK 1008 //安全门未满足，拒绝命令
#define ERROR_ROBOT_HARDWARE_STATE_STALE 1009 //状态数据过期或不可用
#define ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY 1010 //控制服务尚未就绪
#define CMD_ERROR -1
#endif
