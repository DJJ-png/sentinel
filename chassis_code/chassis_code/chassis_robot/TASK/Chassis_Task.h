#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "pid.h"
#include "arm_math.h"
#include "main.h" 
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "upboard_transmit_task.h"
/*--base--begin--*/
typedef struct
{
    //底盘系速度设定
    fp32 ch_vx_set;
	fp32 ch_vy_set;
	fp32 ch_wz_set;
    
    fp32 ch_pitch_fdb;//底盘的仰角值
    fp32 ch_angle_err;//底盘位置环角度差(跟头时)
	pid_type_def ch_psi_pid;//位置环pid
    
    //标志位
    uint8_t ch_enable_flag;//底盘使能标志位
    uint8_t ch_follow_gimbal_flag;//底盘跟头标志位
	uint8_t ch_orient_flag;//头跟底盘模式
    
    //功率控制相关
    fp32 chassis_power_buffer;
    fp32 chassis_power_limit;
    fp32 power_limit_deceive;
	
}chassis_control_t;
	




#endif
