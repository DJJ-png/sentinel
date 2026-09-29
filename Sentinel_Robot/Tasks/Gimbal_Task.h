#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "pid.h"
#include "bsp_can.h"

#define FOLLOW_RADAR  0x01
#define FOLLOW_GIMBAL 0x00

typedef struct
{
    fp32 ENC_speed;
    fp32 INS_speed;
    fp32 INS_speed_set;
    fp32 INS_speed_feedforward;
    
    fp32 ENC_angle;
	fp32 INS_angle;
	fp32 BASE_INS_angle_Pitch;
    fp32 INS_angle_set;
	fp32 INS_angle_err;
	

    int16_t give_current;
	int16_t set_current;
	
	pid_type_def speed_pid;
	pid_type_def angle_pid;
	
	uint8_t control_mode;
	uint8_t lock_flag;
    uint8_t last_lock_flag;
	
} gimbal_motor_t;

typedef struct
{
	uint8_t yaw_mode;
	 
	fp32 advanced_gimbal_zero;
	fp32 angle_error_rad;
	fp32 advanced_yaw_angle;
	fp32 pitch_angle_zero;
	fp32 advanced_yaw_angle_init;
	fp32 pitch_angle_init;
	
	fp32 counterattack_angle;
	uint8_t counterattack_flag;
	
	uint8_t follow_change;
	
	uint8_t MODE;
	
	pid_type_def gimbal_psi;
	pid_type_def gimbal_psi_advanced;
	
} gimbal_control_t;

extern gimbal_motor_t   gimbal_motor[3];
extern gimbal_control_t gimbal_control;

extern uint8_t last_aim_PID_flag_switch;
extern void gimbal_vector_set(fp32 yaw_speed,fp32 pitch_speed,fp32 yaw_angle,fp32 pitch_angle,uint8_t yaw_mode,uint8_t pitch_mode,uint8_t gimbal_mode);
extern void Gimbal_Spin(fp32 Pitch_max,fp32 Pitch_min,fp32*gimbal_spin_speed,uint8_t mode);
extern void Gimbal_Motor_Control(gimbal_motor_t *Gimbal_Motor);
extern void CAN_cmd_LK_Motor(CAN_HandleTypeDef* hcan,uint8_t uid, int16_t current);
extern void Normal_Gimbal_Pid_Init();
extern void Aim_Gimbal_Pid_Init();

#endif
