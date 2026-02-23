#include "Shoot_Task.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "Nmanifold_usart_task.h"
#include "robot_message.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Vofa_send.h"
#include "bsp_can.h"
#include "main.h"
#include "Switch_Task.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "config_set.h"
#include "remote_control.h"
#include "config_set.h"
#include "bsp_math.h"
/*--define--begin--*/
#define DIAL_SPEED_BASE (5400) //(5400/36/60)*8//5400是20HZ//270是1HZ
#define DIAL_SPEED_FAST  (16*270)
#define FRIC_SPEED   5500
#define BARREL_POWER  2500
///**/   /* RPM*/    /*减速比*/
#define SHOOT_MOTOR_SPEED_PID_KP 3.75//3.75//3.0f
#define SHOOT_MOTOR_SPEED_PID_KI 0.01//0.01f
#define SHOOT_MOTOR_SPEED_PID_KD 0.2f
#define SHOOT_MOTOR_SPEED_PID_MAX_OUT 6000.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_IOUT 3000.0f

#define SHOOT_MOTOR_ANGLE_PID_KP 4.5f//4.5//0.2f
#define SHOOT_MOTOR_ANGLE_PID_KI 0.0f
#define SHOOT_MOTOR_ANGLE_PID_KD 1.2//0.2f
#define SHOOT_MOTOR_ANGLE_PID_MAX_OUT 10000.0f
#define SHOOT_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

/*摩擦轮*/
#define FRIC_MOTOR_SPEED_PID_KP 10.0f
#define FRIC_MOTOR_SPEED_PID_KI 0.1f
#define FRIC_MOTOR_SPEED_PID_KD 0.0f
#define FRIC_MOTOR_SPEED_PID_MAX_OUT 4000.0f
#define FRIC_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define FRIC_MOTOR_1 	shoot_m2006[2]
#define FRIC_MOTOR_2 	shoot_m2006[3]

/*热量保护*/
#define BARREL_HEAT_LIMIT  400.0f
#define BARREL_HEAT_COOLING 1000//80.0f
#define BULLET_17MM_HEAT    10.0f

#define CONTRONL_HZ   1000.0f

#define COOLING   (BARREL_HEAT_COOLING/CONTRONL_HZ)
#define ADD       (BULLET_17MM_HEAT/CONTRONL_HZ)
#define SHOOT_HZ(X)  ((fp32)(X))*(20.0f/((fp32)DIAL_SPEED_BASE))

//电机控制变量
#define DIAL_MOTOR 		shoot_m2006[0]  
shoot_motor_t shoot_m2006[4];

//功能控制变量
barrel_control_t barrel_control;
shoot_control_t shoot_control;
RC_ctrl_t last_rc_control;

extern aim_control_t aim_control;

//热量保护

void CAN_cmd_AMMO(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{//电机发送
	 CAN_CMD_BASE(&hcan2,0x200 ,  0,  0,  motor3,  motor4);
	 CAN_CMD_BASE(&hcan1,0x200 ,  motor1,  0,  0,  0);
}
/*--初始化函数--*/
void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {SHOOT_MOTOR_ANGLE_PID_KP, SHOOT_MOTOR_ANGLE_PID_KI, SHOOT_MOTOR_ANGLE_PID_KD};
	memset((uint8_t*)&shoot_m2006[0],0,sizeof(shoot_motor_t));
	PID_init(&shoot_m2006[0].speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006[0].angle_pid,PID_POSITION,shoot_motor_angle_pid,SHOOT_MOTOR_ANGLE_PID_MAX_OUT,SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);
}

void FRIC_Motor_Init(void)
{
	const static fp32 fric_motor_speed_pid[3] = {FRIC_MOTOR_SPEED_PID_KP, FRIC_MOTOR_SPEED_PID_KI, FRIC_MOTOR_SPEED_PID_KD};
	memset((uint8_t*)&FRIC_MOTOR_1,0,sizeof(shoot_motor_t));
	memset((uint8_t*)&FRIC_MOTOR_2,0,sizeof(shoot_motor_t));
	PID_init(&FRIC_MOTOR_1.speed_pid,PID_POSITION,fric_motor_speed_pid,FRIC_MOTOR_SPEED_PID_MAX_OUT,FRIC_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&FRIC_MOTOR_2.speed_pid,PID_POSITION,fric_motor_speed_pid,FRIC_MOTOR_SPEED_PID_MAX_OUT,FRIC_MOTOR_SPEED_PID_MAX_IOUT);
}

/*--数据更新--*/
void Shoot_Motor_Data_Update(void)
{
	/**/
	shoot_m2006[0].speed					=	motor_measure_shoot[0].speed_rpm;
	shoot_m2006[0].give_current		=	motor_measure_shoot[0].given_current;
	shoot_m2006[0].angle					=	dial_angle;
	/**/
	FRIC_MOTOR_1.speed				=	motor_measure_shoot[2].speed_rpm;
	FRIC_MOTOR_1.give_current	=	motor_measure_shoot[2].given_current;
	/**/
	FRIC_MOTOR_2.speed				=	motor_measure_shoot[3].speed_rpm;
	FRIC_MOTOR_2.give_current	=	motor_measure_shoot[3].given_current;
}


/*--摩擦轮控制--*/
void Fric_Motor_Control(void)
{
   if(shoot_control.fric_state==1)
	{
			FRIC_MOTOR_1.speed_set=FRIC_SPEED;
			FRIC_MOTOR_2.speed_set=-FRIC_SPEED;

		PID_calc(&FRIC_MOTOR_1.speed_pid,FRIC_MOTOR_1.speed,FRIC_MOTOR_1.speed_set);
		PID_calc(&FRIC_MOTOR_2.speed_pid,FRIC_MOTOR_2.speed,FRIC_MOTOR_2.speed_set);
		FRIC_MOTOR_1.set_current=FRIC_MOTOR_1.speed_pid.out;
		FRIC_MOTOR_2.set_current=FRIC_MOTOR_2.speed_pid.out;
	}
	else    if(shoot_control.fric_state==0)
	{
//				FRIC_MOTOR_1.speed_set=0;
//				FRIC_MOTOR_2.speed_set=0;
				FRIC_MOTOR_1.set_current=0;
				FRIC_MOTOR_2.set_current=0;
	}
}


/*--拨弹盘控制--*/
void Dial_Motor_Control(void)
{
	if(nuc_receive_data.aim_data_received.shoot_freq == 0)
	{
			nuc_receive_data.aim_data_received.shoot_freq = 25;
	}
	
	/* 导航控制是否进行打符 */
	if(nuc_transmit_data.robot_gimbal_data_send.mode == 2 || nuc_transmit_data.robot_gimbal_data_send.mode == 3)
	{
		shoot_control.dial_mode = 1;
	}
	else
	{
		shoot_control.dial_mode = 0;
	}
	
	if(fifo_s_isempty(&Referee_FIFO)!=0||Robot_Status.power_management_shooter_output==0x01)
		{
		if(shoot_control.fric_state==1)
		{
			/* 自瞄模式且自瞄控制连发模式 */
			if(nuc_receive_data.aim_data_received.fire_mode == 0  && shoot_control.dial_mode==0)
			{
				/* 自瞄开启 and 导航模式 and 摩擦轮开关在中间 */
				if(	nuc_receive_data.aim_data_received.success==1 && nuc_receive_data.aim_data_received.is_fire == 1
					&&(((Switch_Left	==	RC_SW_UP)||(Switch_Left	==	RC_SW_MID))&&nuc_control.action.robot_aim!=0)
				)//&&nuc_receive_data.aim_data_received.target_rate!=0)
					shoot_control.dial_speed
					=	nuc_receive_data.aim_data_received.shoot_freq*36*60/8;//DIAL_SPEED_BASE
				/* 1.摩擦轮开关至上 2.调试自瞄模式 and 摩擦轮开关至上 */
				else if((Switch_Right	==	RC_SW_UP&&aim_control.aim_debug_flag!=1)||
					(Switch_Right	==	RC_SW_UP&&(nuc_receive_data.aim_data_received.is_fire==1&&nuc_receive_data.aim_data_received.success==1)&&aim_control.aim_debug_flag==1))
					shoot_control.dial_speed
					=	nuc_receive_data.aim_data_received.shoot_freq*36*60/8;//DIAL_SPEED_BASE
				else shoot_control.dial_speed=0;
				//控制传给电机
				shoot_m2006[0].speed_set=shoot_control.dial_speed;
				shoot_m2006[0].angle_set=shoot_m2006[0].angle;
			}
			/* 打符 */
			else if(shoot_control.dial_mode == 1)//单发
			{
					/*控 单发脉冲*/
					if(!(last_rc_control.rc.s[0]==RC_SW_UP)&&(rc_ctrl.rc.s[0]==RC_SW_UP)
						&&nuc_receive_data.aim_data_received.success!=1)
					{
						if(shoot_control.if_single_hit==0)
						{
							shoot_control.if_single_hit=1;
							shoot_m2006[0].angle_set=shoot_m2006[0].angle-8192*45/360;//按拨弹盘改
						}
					}
					else if(((Switch_Left	==	RC_SW_UP && Switch_Right	==	RC_SW_MID) 
						||(Switch_Right	==	RC_SW_UP && aim_control.aim_debug_flag==1))
						&& nuc_receive_data.aim_data_received.fire_mode == 1
						&& nuc_receive_data.aim_data_received.success==1)
					{		
						if(shoot_control.if_single_hit==0)
						{
							shoot_control.if_single_hit=1;
							shoot_m2006[0].angle_set=shoot_m2006[0].angle-8192*45/360;//按拨弹盘改
							nuc_receive_data.aim_data_received.fire_mode = 0;
						}
					}
						/*信号执行*/
						if(shoot_control.if_single_hit==1)
						{
							shoot_control.dial_single_cnt++;							
							PID_calc(&shoot_m2006[0].angle_pid,-LIMIT_TO_SET(shoot_m2006[0].angle-shoot_m2006[0].angle_set,4096),0);
							shoot_m2006[0].speed_set=shoot_m2006[0].angle_pid.out;
							if(shoot_control.dial_single_cnt>100)
							{
								shoot_control.if_single_hit=0;
								shoot_control.dial_single_cnt=0;
								shoot_m2006[0].speed_set=0;
							}
						}
															
			}
			else if((((Switch_Left	==	RC_SW_UP) && nuc_control.action.robot_aim!=0)
				||(Switch_Right	==	RC_SW_UP && aim_control.aim_debug_flag==1))
				&& shoot_control.dial_mode == 0 && nuc_receive_data.aim_data_received.fire_mode == 1
				&& nuc_receive_data.aim_data_received.success==1)
			{		
				if(shoot_control.if_single_hit==0)
				{
					shoot_control.if_single_hit=1;
					shoot_m2006[0].angle_set=shoot_m2006[0].angle-8192*45/360;//按拨弹盘改
				}
				/*信号执行*/
				if(shoot_control.if_single_hit==1)
				{
					shoot_control.dial_single_cnt++;
					if(shoot_control.dial_single_cnt>100)
					{
						shoot_control.if_single_hit=0;
						shoot_control.dial_single_cnt=0;
						nuc_receive_data.aim_data_received.fire_mode = 0;
					}
				}
					
				PID_calc(&shoot_m2006[0].angle_pid,-LIMIT_TO_SET(shoot_m2006[0].angle-shoot_m2006[0].angle_set,4096),0);
				shoot_m2006[0].speed_set=shoot_m2006[0].angle_pid.out;
			}
			/* 双发模式 */
			else if((((Switch_Left	==	RC_SW_UP || Switch_Left	==	RC_SW_MID) && nuc_control.action.robot_aim!=0)
				||(Switch_Right	==	RC_SW_UP && aim_control.aim_debug_flag==1))
				&& shoot_control.dial_mode == 0 && nuc_receive_data.aim_data_received.fire_mode == 2
				&& nuc_receive_data.aim_data_received.success==1)
			{		
				if(shoot_control.if_single_hit==0)
				{
					shoot_control.if_single_hit=1;
					shoot_m2006[0].angle_set=shoot_m2006[0].angle-8192*90/360;//按拨弹盘改
				}
				/*信号执行*/
				if(shoot_control.if_single_hit==1)
				{
					shoot_control.dial_single_cnt++;
					if(shoot_control.dial_single_cnt>300)
					{
						shoot_control.if_single_hit=0;
						shoot_control.dial_single_cnt=0;
						nuc_receive_data.aim_data_received.fire_mode = 0;
					}
				}
					
				PID_calc(&shoot_m2006[0].angle_pid,-LIMIT_TO_SET(shoot_m2006[0].angle-shoot_m2006[0].angle_set,4096),0);
				shoot_m2006[0].speed_set=shoot_m2006[0].angle_pid.out;
			}
		}/*--end--fric_on*/
		else 
		{
			shoot_m2006[0].speed_set=0;
		}
	}
/**/
		

	PID_calc(&shoot_m2006[0].speed_pid	,shoot_m2006[0].speed	,shoot_m2006[0].speed_set);
	shoot_m2006[0].set_current=shoot_m2006[0].speed_pid.out;
	/*卡弹保护*/
		if(abs(shoot_m2006[0].give_current)>5000&&fabs(shoot_m2006[0].speed)<100&&shoot_control.dial_stop_cnt<=250)
		{
			shoot_control.dial_stop_cnt++;	
		}
		if(shoot_control.dial_stop_cnt>=250)
		{
				if(shoot_control.wait_time<=250)
				{
					shoot_m2006[0].set_current=-4000;
				}
				shoot_control.wait_time++;
				if(shoot_control.wait_time>=250)
				{
					shoot_control.dial_stop_cnt=0;
					shoot_control.wait_time=0;
				}
				shoot_m2006[0].angle_set=dial_angle;
		}
}

void shoot_Control()
{
	if(barrel_control.barrel_wait_flag==0)
	{
		if(Power_Heat_Data.shooter_17mm_1_barrel_heat >= Robot_Status.shooter_barrel_heat_limit*0.75f)
		{
			shoot_control.dial_speed=0;
			shoot_m2006[0].set_current=0;
			shoot_m2006[0].speed_pid.Iout=0;
			barrel_control.barrel_wait_flag=1;
		}
	}
	if(barrel_control.barrel_wait_flag==1)
	{
				shoot_control.dial_speed=0;
				shoot_m2006[0].set_current=-500; 
				shoot_m2006[0].speed_pid.Iout=0;

				barrel_control.barrel_wait_cnt++;

		if(barrel_control.barrel_wait_cnt>=575 )
		{
			barrel_control.barrel_wait_cnt=0;
			barrel_control.barrel_wait_flag=0;
		}
	}
}

void heat_cooling()
{
	
	barrel_control.barrel_heat[0]+=ADD*SHOOT_HZ(shoot_m2006[0].speed);		
	barrel_control.barrel_heat[0]-=COOLING;
	if(barrel_control.barrel_heat[0]<=0.0f)
		barrel_control.barrel_heat[0]=0.0f;
	Power_Heat_Data.shooter_17mm_1_barrel_heat=barrel_control.barrel_heat[0];
	Robot_Status.shooter_barrel_heat_limit=1000.0f;

}

/*--Task--*/
#if defined  Broad_Gimbal
void Shoot_Task(void const * argument)
{	 

	Shoot_Motor_Init();
	FRIC_Motor_Init();
	
	vTaskDelay(200);
	while(1)
	{
		
		Shoot_Motor_Data_Update();
		   Fric_Motor_Control();
		   Dial_Motor_Control();
			 heat_cooling();
			 shoot_Control();
		if(Switch_Right==RC_SW_MID||Switch_Right==RC_SW_UP)
		{
				CAN_cmd_AMMO(shoot_m2006[0].set_current, 0 ,FRIC_MOTOR_1.set_current,FRIC_MOTOR_2.set_current);
		}
		else
		{
			CAN_cmd_AMMO(0,0,0,0);
			shoot_m2006[0].speed_pid.Iout=0;
		}
		
		last_rc_control = rc_ctrl;
		
		vTaskDelay(1);
	}
}
#else
void Shoot_Task(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

#endif
/*--function--end*/