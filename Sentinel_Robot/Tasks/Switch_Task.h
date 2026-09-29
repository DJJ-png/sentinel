#ifndef _SWITCH_TASK_H_
#define _SWITCH_TASK_H_
#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
typedef struct 
{
		uint8_t aim_debug_flag;
		int16_t aim_keep;
		fp32 aim_keep_angle[2];
		uint8_t aim_PID;
        uint8_t last_aim_PID;
}aim_control_t;
typedef struct
{
    fp32 vx;
	fp32 vy;
	fp32 wz;
	uint8_t chassis_follow_gimbal;
	uint8_t chassis_enable;
	uint8_t follow_chassis_direction;
	pid_type_def chassis_psi;
	fp32 angle_error_rad;
	
	fp32 slope_angle;//记录上坡时底盘的仰角值
}chassis_control_t;
extern aim_control_t aim_control;

/**
 * @brief 云台状态枚举
 */
typedef enum {
    GIMBAL_RELAX = 0,    // 失能/无力
    GIMBAL_RC_CTRL,      // 遥控器手动控制
    GIMBAL_AUTO_AIM,     // 视觉自瞄
    GIMBAL_PATROL,       // 自动巡逻扫描
    GIMBAL_COUNTERATTACK, // 受击反击逻辑
    GIMBAL_NUC_CTRL        //NUC控云台（比赛中不巡逻不自瞄的模式）
} Gimbal_State_e;

/**
 * @brief 底盘状态枚举
 */
typedef enum {
    CHASSIS_RELAX = 0,   // 失能
    CHASSIS_RC_CTRL,     // 遥控器手动控制
    CHASSIS_NUC_CTRL,    // 导航控制（含跟头/不跟头）
} Chassis_State_e;

/* 全局状态变量 */
extern Gimbal_State_e gimbal_state;
extern Chassis_State_e chassis_state;

/* 函数原型定义 */
void Step_Gimbal_FSM(void);  // 云台状态机逻辑步进
void Step_Chassis_FSM(void); // 底盘状态机逻辑步进
void Update_System_Event(void); // 事件监听与状态转移判断

#endif
