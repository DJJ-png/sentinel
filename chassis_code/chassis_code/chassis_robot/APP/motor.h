#ifndef MOTOR_H
#define MOTOR_H

#include "struct_typedef.h"
#include "pid.h"
#include "can.h"
#include "arm_math.h"

#include "bsp_can.h"

#include "my_math.h"
#include "config_set.h"

#include "Chassis_Task.h"
#include "detect_task.h"
//电机参数(调参地方)
#define CH_PARAM_WHEEL_SPEED_PID_KP             15.0f
#define CH_PARAM_WHEEL_SPEED_PID_KI             0.1f
#define CH_PARAM_WHEEL_SPEED_PID_KD             0.0f
#define CH_PARAM_WHEEL_SPEED_PID_MAXOUT         16000.0f
#define CH_PARAM_WHEEL_SPEED_PID_MAXIOUT        800.0f
 
#ifdef CH_CFG_TYPE_SWERVE

#define CH_PARAM_STEER_SPEED_PID_KP             100.0f
#define CH_PARAM_STEER_SPEED_PID_KI             0.2f
#define CH_PARAM_STEER_SPEED_PID_KD             100.0f
#define CH_PARAM_STEER_SPEED_PID_MAXOUT         16000.0f
#define CH_PARAM_STEER_SPEED_PID_MAXIOUT        800.0f

#define CH_PARAM_STEER_ANGLE_PID_KP             70.0f
#define CH_PARAM_STEER_ANGLE_PID_KI             0.0f
#define CH_PARAM_STEER_ANGLE_PID_KD             100.0f
#define CH_PARAM_STEER_ANGLE_PID_MAXOUT         400.0f
#define CH_PARAM_STEER_ANGLE_PID_MAXIOUT        0.0f

#endif

#define CH_SCALE_V_TO_RMP   (60/(2*PI*CH_PARAM_WHEEL_R_MM)*CH_PARAM_WHEEL_RATIO)
#define SCALE_6020ECD_TO_RAD 1303.797294f  // 8192 / 2pi


//电机数据接口
#define get_djimotor_measure(ptr, data)                                 \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = 	(int16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
    
#define motor_measure_LK(ptr, data)                                 				    \
    {                                                                   				\
        (ptr)->last_ecd = 			(ptr)->ecd;                             		    \
        (ptr)->ecd = 						(uint16_t)((data)[7] << 8 | (data)[6]); 	\
        (ptr)->speed_rpm = 			((int16_t)((data)[5] << 8 | (data)[4]))/60.0f;	    \
        (ptr)->given_current = 	(uint16_t)((data)[3] << 8 | (data)[2]); 				\
        (ptr)->temperate = 			(data)[1];                              		    \
    }	
 
//电机封装结构体
typedef struct
{
	int16_t last_ecd;
    uint16_t ecd;
    fp32 speed_rpm;
    int16_t given_current;
    uint8_t temperate;
}djimot_measure_t;

typedef struct
{     
    #if defined(CH_CFG_WHEEL_MOT_DJI_3508) || defined(CH_CFG_WHEEL_MOT_DJI_2006)
        djimot_measure_t motor_data_raw;
    #endif
    
    #if defined CH_CFG_WHEEL_MOT_LK_6015
       lkmot_measure_t motor_data_raw;
    //缺少翎控电机数据的结构体
    #endif
    
    pid_type_def wheel_speed_pid;
    fp32 speed_set;
    
}wheel_motor_t;//轮电机结构体



#ifdef CH_CFG_TYPE_SWERVE
typedef struct
{     
    #if defined(CH_CFG_STEER_MOT_DJI_6020) || defined(CH_CFG_STEER_MOT_DJI_3508)
        djimot_measure_t motor_data_raw;
    #endif
    
    #ifdef CH_CFG_STEER_MOT_DM_4310
        lkmot_measure_t motor_data_raw;
        //缺少达妙电机电机数据的结构体
    #endif
    
    pid_type_def steer_speed_pid;//速度环结构体
    pid_type_def steer_angle_pid;//角度环结构体
    
    fp32 ecd_offset_rad;//舵轮偏置值
	fp32 last_angle_set;
	fp32 angle_set;
	fp32 angle_err;
    fp32 angle_fdb;
    
	fp32 speed_set;
}steer_motor_t;//舵电机结构体
#endif

extern void  Can_Getmeasure_Chmotor(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8]);
extern void  Pid_Init_motor(pid_type_def *pid, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);
extern void  Pid_Init_CHmotor();
extern void  CH_Solve_motor();
extern void  Ch_Calc_Motorpid();
extern void  Can_Tansmit_Chmotor_current();
extern void  Can_Tansmit_Chmotor_off();
#endif