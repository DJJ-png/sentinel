#include "Chassis_Task.h" 
#include "arm_math.h"
#include "helm_ctrl.h"
#include "main.h"
#include "My_Def.h"
#include "bsp_cap.h" 
#include "Vofa_send.h"
#include "Gimbal_Task.h"
#include "Referee.h"
#include "bsp_buzzer.h"
#include "AHRS_middleware.h"

#define CURRENT_3508   (20.0f/16284.0f)//没用
#define CURRENT_6020   ( 3.0f/16284.0f)//没用

//功率控制相关
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f//无功率控制下功率控制的电流限制
#define POWER_TOTAL_CURRENT_LIMIT       22000.0f//功率控制下功率控制的电流限制
#define WARNING_POWER_BUFF  60.0f
#define POWER_HELM_CURRENT_LIMIT       8000.0f

chassis_control_t chassis_control;

//自瞄丢失标志位
uint16_t Autoaim_lose_flag = 0;

/**
  * @brief  1.底盘模式和速度设置
			2.使用斜坡函数对速度进行更新，防止速度突变
  * @param  ........
  */
void chassis_vector_set(fp32 vx,fp32 vy,fp32 wz,fp32 ang_err,uint8_t flag)
{
	chassis_control.slope_angle = gimbal_motor[BASE_YAW_5010].BASE_INS_angle_Pitch; //实时记录底盘俯仰角度
/***-flag-***/
	chassis_control.chassis_enable				=	(flag&CHASSIS_ENABLE);
	chassis_control.chassis_follow_gimbal		=	(flag&CHASSIS_FLOWE_GIMBAL);
	chassis_control.follow_chassis_direction	=	(flag&FOLLOW_CHASSIS_DIRECTION);
	
/*-- -v_set- --*/
	/* 在上坡时提速 */
		if((vx > 0 && chassis_control.slope_angle > 15.0f) || (vx < 0 && chassis_control.slope_angle < -15.0f))
		{
			vx*=1.5f;
			vy*=1.5f;											
		}	
		//设定上坡时的速度 
		chassis_control.vx=(float)DEADBAND(RAMP_CTRL(chassis_control.vx,vx,0.8),7);//5
		chassis_control.vy=(float)DEADBAND(RAMP_CTRL(chassis_control.vy,vy,0.8),7);//5
		chassis_control.wz=(float)DEADBAND(RAMP_CTRL(chassis_control.wz,wz,0.8),7);//5
		
//	/* 下坡时减速 */
//		if((vx > 0 && chassis_control.slope_angle < -3.0f) || (vx < 0 && chassis_control.slope_angle > 3.0f))
//		{
//			vx*=0.7f;
//			vy*=0.7f;
//			chassis_control.vx=(float)DEADBAND(RAMP_CTRL(chassis_control.vx,vx,0.2),3);
//			chassis_control.vy=(float)DEADBAND(RAMP_CTRL(chassis_control.vy,vy,0.2),3);
//			chassis_control.wz=(float)DEADBAND(RAMP_CTRL(chassis_control.wz,wz,0.2),3);
//		}

	//chassis_control.slope_angle_last = gimbal_motor[BASE_YAW_5010].BASE_INS_angle_Pitch;
	

/*--旋转移动--角度修正--*/
	chassis_control.angle_error_rad=	ang_err + 0.038f*(fp32)chassis_control.wz/RADIUS_CHASSIS;//由于功率控制最后会降低转速,转速与实际不符,效果不好0.11
	chassis_control.angle_error_rad=(float)DEADBAND(chassis_control.angle_error_rad,0.01);
	//chassis_control.angle_error_rad=	ang_err + 0.0105f*(fp32)chassis_control.wz*PI/180.f;//由于功率控制最后会降低转速,转速与实际不符,效果不好
	/**/
	
//	if(chassis_control.follow_chassis_direction == 1)
//	{
//		gimbal_control.MODE = GIMBAL_TO_CHASSIS_MODE;
//	}
	
	chassis_solve();
}


void chassis_solve()
{
	float ang_err=chassis_control.angle_error_rad;	/*云台底盘差角*/

	//设置线速度
	if(chassis_control.follow_chassis_direction) //以底盘朝向为正方向，不进行转换
	{
		chassis_helm.vx = chassis_control.vx;
		chassis_helm.vy = chassis_control.vy;
	}
	else{	/*以云台朝向为正方向，云台系转底盘系*/
		chassis_helm.vx= chassis_control.vx*arm_cos_f32(ang_err)+chassis_control.vy*arm_sin_f32(ang_err);
		chassis_helm.vy=-chassis_control.vx*arm_sin_f32(ang_err)+chassis_control.vy*arm_cos_f32(ang_err);
	}
	//设置角速度
	float ang_err_all[4];
	if(chassis_control.chassis_follow_gimbal)//底盘跟头
	{	//四零点计算
		ang_err_all[0] = limit_pi(0.0f*PI+ang_err);
		ang_err_all[1] = limit_pi(0.0f*PI+ang_err);
		ang_err_all[2] = limit_pi(0.0f*PI+ang_err);
		ang_err_all[3] = limit_pi(0.0f*PI+ang_err);
		float ang_err=ang_err_all[0];
		for(uint8_t i=0;i<4;i++)
			if(fabs(ang_err_all[i])<fabs(ang_err))
				ang_err=ang_err_all[i];
//        if(fabs(ang_err)<PI+0.05&&fabs(ang_err)>PI-0.05)
//            ang_err=0;
		PID_calc(&chassis_control.chassis_psi,ang_err,0);
		DEADBAND(chassis_control.chassis_psi.out,50);
		chassis_helm.wz = chassis_control.chassis_psi.out;			
	}
	else
	{
		chassis_helm.wz= chassis_control.wz;
	}

	//发送给下板
	CAN_CMD_BASE(&CAN_COMMUNITE,CAN_COMMUNITE_ID_1,
	(int16_t)chassis_helm.vx,
	(int16_t)chassis_helm.vy,
	(int16_t)(chassis_helm.wz),
	(int16_t)((chassis_control.chassis_follow_gimbal<<1)|(chassis_control.chassis_enable)));
}

void cap_control()
{
	CAN_Send_Cap(chassis_control.power_limit_deceive,chassis_control.chassis_power,Power_Heat_Data.buffer_energy);
	vTaskDelay(1);
}


void chassis_power_updata()
{
	cap_data.cap_recieve_flag=0;
}


/*power_control*/
void chassis_power_control()
{
	fp32 Wheel_current_limit=0;
	fp32 Helm_current=0,Wheel_current=0;
	
	fp32 power_scale=1.0f;
	fp32 power_buffer_scale=1.0f;
	chassis_control.chassis_power_buffer = 60.0f;
	chassis_control.chassis_power_limit = 100.0f;
	
	/* 如果功率限制不为0，那么根据功率限制百分比来调整轮子电流的限制 */
	if(chassis_control.chassis_power_limit==0)		Wheel_current_limit=NO_JUDGE_TOTAL_CURRENT_LIMIT;
	else																					Wheel_current_limit=POWER_TOTAL_CURRENT_LIMIT*chassis_control.chassis_power_limit/100;
	
	if(chassis_control.chassis_power_buffer < WARNING_POWER_BUFF)
	{
			if(chassis_control.chassis_power_buffer > 20.0f)		power_buffer_scale = (chassis_control.chassis_power_buffer)/ (WARNING_POWER_BUFF);
			else																								power_buffer_scale = 0.0f / WARNING_POWER_BUFF;
	}
	
	for(uint8_t i=0;i<4;i++)		Helm_current+=0.15f*fabs(helm[i].M6020_speed_pid.out);
	if(Helm_current>POWER_HELM_CURRENT_LIMIT){
		for(uint8_t i=0;i<4;i++)			helm[i].M6020_speed_pid.out*=POWER_HELM_CURRENT_LIMIT/Helm_current;
		Helm_current=POWER_HELM_CURRENT_LIMIT;
	}
	for(uint8_t i=0;i<4;i++)			helm[i].M6020_speed_pid.out*=power_buffer_scale*power_scale;

	/*3508*/
	Wheel_current_limit-=Helm_current;	
	for(uint8_t i=0;i<4;i++)				Wheel_current+=fabs(helm[i].M3508_speed_pid.out);
	if(Wheel_current>Wheel_current_limit)
		for(uint8_t i=0;i<4;i++)			helm[i].M3508_speed_pid.out*=Wheel_current_limit/Wheel_current;
		for(uint8_t i=0;i<4;i++)			helm[i].M3508_speed_pid.out*=power_buffer_scale*power_scale;
}


/*只对轮向电机进行了功率控制，没有对舵向电机进行功率控制*/
void chassis_power_control_new(void) 
{
		/*
			超级电容组放电 + 电池的输出（裁判系统检测到的） = 底盘总能量 = 电机所接收到的能量；
			底盘电机所输出的总功率 - 超电放出的能量 = 裁判系统检测到的功率；
			裁判系统检测到的功率越大（已超功率），缓冲能量越小；
			在电容组能量很多且缓冲能量很多时，底盘功率拉高，电容组放电减小，缓冲能量就会减小；
			在电容组能量很多且缓冲能量很少时，电容组放电增加，缓冲能量就会增多；
		*/
		if(Robot_Status.chassis_power_limit < 50 && Game_Status.game_progress == 4)
		{
			chassis_control.chassis_power_limit = 33;//100
			chassis_control.power_limit_deceive = 33;//100
		}else
		{
			chassis_control.chassis_power_limit = 98;//100
			chassis_control.power_limit_deceive = 98;//100
		}
	

		chassis_control.chassis_power = cap_data.chassis_power;
	
		//根据超电剩余电量大小改变限制功率，要测的时候改
		if(cap_data.cap_per > 0.5) 
		{
			if(Power_Heat_Data.buffer_energy > 40.0f)
			{
				chassis_control.power_limit_deceive += (Power_Heat_Data.buffer_energy - 40.0f) * 1.0f;
			}	
			else
			{
				chassis_control.power_limit_deceive += (Power_Heat_Data.buffer_energy - 40.0f) * 1.5f;
			}
			
			chassis_control.chassis_power_limit += 35;
		}
		else if(cap_data.cap_per > 0.3)
		{
			if(Power_Heat_Data.buffer_energy > 40.0f)
			{
				chassis_control.power_limit_deceive += (Power_Heat_Data.buffer_energy - 40.0f) * 2.0f;
			}
			else
			{
				chassis_control.power_limit_deceive += (Power_Heat_Data.buffer_energy - 40.0f) * 1.0f;
			}
			chassis_control.chassis_power_limit += 1;//15
		}
		else
		{			
			chassis_control.chassis_power_limit *= 0.45;
			chassis_control.power_limit_deceive = chassis_control.chassis_power_limit - 15;//10
		}
		
		if(chassis_control.power_limit_deceive < 20 || chassis_control.power_limit_deceive > 300)
		{
			chassis_control.power_limit_deceive = 20;
		}
		
		if(fabs(chassis_helm.vx) < 100 && fabs(chassis_helm.vy) < 100 && fabs(chassis_helm.wz) < 100)
		{
			PID_clear(&chassis_control.chassis_power_pid);
		}
		else
		{
			//PID计算函数中后面那个是限制的功率，也就是期望达到的功率，中间的是当前计算的功率，pid计算得出total_current，即为设定最大电流
			float total_current = 27000+PID_calc(&chassis_control.chassis_power_pid,chassis_control.chassis_power,chassis_control.chassis_power_limit);

			fp32 sum_give_current=0;
			float total_err=0;

			//计算当前将要给的电流总和
			for(uint8_t i=0;i<4;i++)	sum_give_current+=fabs(helm[i].M3508_speed_pid.out);
			
			for(uint8_t i=0;i<2;i++)	total_err+=fabs(-helm[i].speed_set*V_TO_RPM - helm[i].M3508.speed_rpm);
			
			for(uint8_t i=2;i<4;i++)	total_err+=fabs(helm[i].speed_set*V_TO_RPM - helm[i].M3508.speed_rpm);


			//如果将要给的电流大于限定的电流，就按各个轮子与期望转速的误差分配功率，最后能基本上让轮子达到期望的转速
			if(sum_give_current>total_current)
			{
				float k=total_current/sum_give_current;
				for(uint8_t i = 0;i<2;i++)
				{
					float k1=k*fabs(-helm[i].speed_set*V_TO_RPM - helm[i].M3508.speed_rpm)/(total_err/4.0f);
					if(k1 > 1) k1 = 1;

					helm[i].M3508_speed_pid.out *= k1;
				}
				for(uint8_t i = 2;i<4;i++)
				{
					float k1=k*fabs(helm[i].speed_set*V_TO_RPM - helm[i].M3508.speed_rpm)/(total_err/4.0f);
					if(k1 > 1) k1 = 1;

					helm[i].M3508_speed_pid.out *= k1;
				}
			}
		}
}


void Autoaim_Buzzer_State_Update(void)
{	
	Autoaim_lose_flag ++;
	if(Autoaim_lose_flag > 1000 && Game_Status.game_progress == 1)
	{
			SetBuzzerFrequence(1046);
			vTaskDelay(500);
			buzzer_off();
			vTaskDelay(500);			
	}
	else
	{
		buzzer_off();
	}
	
	if(Autoaim_lose_flag > 1500) Autoaim_lose_flag = 1200;
}
#if defined  Broad_Chassis
void Chassis_Task(void const * argument)
{
	vTaskDelay(200);
	helm_pid_init();
	vTaskDelay(50);
	
	/* 初始化上一次6020电机角度值（转换成弧度） 目的是解决电机初始化漂移现象 */
	helm[2].last_angle_set = -40.45/57.3f;
	helm[3].last_angle_set = 40.45/57.3f;

	while(1)
	{
		helm_solve();//底盘系转舵轮系
		cap_control();//底盘发送给超电
		helm_pid_update();//舵轮pid控制
		chassis_power_control_new();//功率控制
		//chassis_power_control();
		if(chassis_control.chassis_enable==0)		helm_current_off();
		else										helm_current_send();//发送给电机
		vTaskDelay(1);
		
	}
}
#else
void Chassis_Task(void const * argument)
{
	PID_init_s(&chassis_control.chassis_psi,0,3000,0,200,1000,0);
	PID_init_s(&chassis_control.chassis_power_pid,0,CHASSIS_POWER_PID_KP,CHASSIS_POWER_PID_KI,CHASSIS_POWER_PID_KD,CHASSIS_POWER_PID_MAX_OUT,CHASSIS_POWER_PID_MAX_IOUT);
	//PID_init_s(&chassis_control.chassis_psi,0,3000,0,1500,4500,0);
  for(;;)
  {		/*--GIMBALE_MODE--*/
		
		/* 自检时丢失自瞄信息蜂鸣器响 */
		//Autoaim_Buzzer_State_Update();
    vTaskDelay(1);
  }
}
#endif

/*--function--end*/