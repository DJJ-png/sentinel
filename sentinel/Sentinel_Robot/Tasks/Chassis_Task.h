#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "pid.h"

/*--define--begin--*/

#define CHASSIS_POWER_PID_KP 360.0f		//160
#define CHASSIS_POWER_PID_KI 5.0f
#define CHASSIS_POWER_PID_KD 20.0f
#define CHASSIS_POWER_PID_MAX_OUT 64000.0f
#define CHASSIS_POWER_PID_MAX_IOUT 64000.0f

/*--define--end--*/

/*--base--begin--*/
typedef struct
{
    fp32 vx;
	fp32 vy;
	fp32 wz;
	pid_type_def chassis_psi;
	uint8_t chassis_follow_gimbal;
	uint8_t chassis_enable;
	uint8_t follow_chassis_direction;
	
	fp32 angle_error_rad;
	
	uint8_t slope_mode;//没用
	fp32 slope_angle_last;
	fp32 slope_angle;//记录上坡时底盘的仰角值
	
	fp32 chassis_power;
	fp32 chassis_power_limit;
	fp32 chassis_power_buffer;
	fp32 power_limit_deceive;
	pid_type_def chassis_power_pid;
//	fp32 hlem_power_scale[2];// 6020 3508
}chassis_control_t;
	
/*--base--end--*/

/*--extern--begin--*/
extern chassis_control_t chassis_control;
extern uint8_t rollwheel_mode;
extern uint16_t Autoaim_lose_flag;
/*--extern--end--*/


/*--function--begin--*/
extern void chassis_vector_set(fp32 vx,fp32 vy,fp32 wz,fp32 ang_err,uint8_t flag);
extern void chassis_solve(void);
extern void Autoaim_Buzzer_State_Update(void);

/*--function--end*/




#endif
