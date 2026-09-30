#include "helm_ctrl.h"
#include "can.h"
#include "arm_math.h"
#include "My_Def.h"
#include "Chassis_Task.h"


helm_state_t helm[4];
chassis_helm_t chassis_helm;

/* 底盘漂移修正标志位 */


/**
  * @brief  转角限制到±PI
  * @param  输入转角
  * @retval 输出转角
  */
fp32 limit_pi(fp32 in)
{
	while(in < -PI || in > PI)
	{
		if (in < -PI)
			in = in + PI + PI;
		if (in > PI)
			in = in - PI - PI;
	}
	return in;
}

/**
  * @brief  PID初始化
  * @param  ........
  */
void PID_init_s(pid_type_def *pid, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
{
	fp32 pid_para[3]={kp,ki,kd};
	PID_init(pid,mode,pid_para,max_out,max_iout);
}

/**
  * @brief  底盘全向轮电机和舵轮电机初始化
  */
void helm_pid_init()
{
	//四零点
	helm[2].ecd_offset=HELM_OFFSET_2;
	helm[3].ecd_offset=HELM_OFFSET_3;
	
	//舵速度环,角度环,轮速度环
	for(uint8_t i=0;i<4;i++)
	{
		PID_init_s(&helm[i].M6020_angle_pid	,0			,180		,0			,40			,400		,0);
		PID_init_s(&helm[i].M6020_speed_pid	,0			,170		,0.5			,0			,M6020_MOTOR_SPEED_PID_MAX_OUT	,M6020_MOTOR_SPEED_PID_MAX_IOUT);
		PID_init_s(&helm[i].M3508_speed_pid	,0			,20		,0.5f		,0			,M3505_MOTOR_SPEED_PID_MAX_OUT	,M3505_MOTOR_SPEED_PID_MAX_IOUT);//16
	}
}

fp32 text;

/**
  * @brief  1.计算底盘电机输出
						2.全向轮和舵轮分开进行计算
						3.底盘v -> 轮子v -> 轮子rpm
						4.将每个轮子的速度设定转换成PID.out
  */
void helm_pid_update()
{
	helm[2].angle_set *=-1;//取反,反装修正
	helm[3].angle_set *=-1;
	
//		helm[1].speed_set *=-1;
		helm[2].speed_set *=-1;
//		helm[3].speed_set	*=-1;
	
	/*由电机实际装车正方向决定*/
	
	for(uint8_t i=2;i<4;i++)
	{
		//找两头离设定角最近的一头
		fp32 angle_err1=limit_pi((helm[i].M6020.ecd-helm[i].ecd_offset)/1303.797294f-helm[i].angle_set);
		fp32 angle_err2=limit_pi((helm[i].M6020.ecd-helm[i].ecd_offset+4096)/1303.797294f-helm[i].angle_set);
		if(fabs(angle_err2)>fabs(angle_err1))
		{
			//若舵反向则速度也反向
			helm[i].angle_err = angle_err1;
			helm[i].speed_set *= -1;
		}
		else helm[i].angle_err=angle_err2;
		
		//6020
		PID_calc(&helm[i].M6020_angle_pid,helm[i].angle_err,0);
		PID_calc(&helm[i].M6020_speed_pid,helm[i].M6020.speed_rpm,helm[i].M6020_angle_pid.out);
		
		text = -helm[0].speed_set*V_TO_RPM;
		
		//3508
		PID_calc(&helm[i].M3508_speed_pid,helm[i].M3508.speed_rpm ,helm[i].speed_set*V_TO_RPM);
	}
	
	/* 全向轮电机输出计算 */ 
	for(uint8_t i=0;i<2;i++)
	{
			PID_calc(&helm[i].M3508_speed_pid,helm[i].M3508.speed_rpm ,-helm[i].speed_set*(60/(2*PI*60)*13.72));	//	15.8     34
	}
	
	text = -helm[1].speed_set*(60/(2*PI*60)*13.72);
}


/* ************************************** 电机底层通信 ******************************************* */

/**
  * @brief  can线发送数据，pid.out控制(3508电流,6020电压)
  */
void helm_current_send()
{
	CAN_CMD_BASE(&hcan2,0x200,helm[0].M3508_speed_pid.out,helm[1].M3508_speed_pid.out,helm[2].M3508_speed_pid.out,helm[3].M3508_speed_pid.out);
	vTaskDelay(1);
	CAN_CMD_BASE(&hcan2,0x1FF,helm[2].M6020_speed_pid.out,0,0,helm[3].M6020_speed_pid.out);
	vTaskDelay(1);
}

/**
  * @brief  底盘电机失能
  */
void helm_current_off()
{
	CAN_CMD_BASE(&hcan2,0x200,0,0,0,0);	vTaskDelay(1);
	CAN_CMD_BASE(&hcan2,0x1FF,0,0,0,0);	vTaskDelay(1);
}

/* ************************************** 底盘姿态解算 ******************************************* */

/*	全向轮 helm[0] helm[1]
				/ - - - - \  	上边310
				-					-		|
	侧		-					-			x
	边		-					-		-y--*----
	368		-					-			|
				-					-		|
				\ - - - - /		逆时针为正方向
			舵轮 helm[3] helm[2]
				
sin( 40.15°) = 310/481 ≈ 0.6445322245322245f
cos( 40.15°) = 368/481 ≈ 0.7650727650727651f
*/

void helm_solve()
{
	//全向轮轮速计算
	helm[0].speed_set = ( chassis_helm.vx * sin_40_15 - chassis_helm.vy * cos_40_15) - chassis_helm.wz;
	helm[1].speed_set = (  -chassis_helm.vx * cos_40_15 - chassis_helm.vy * sin_40_15)  - chassis_helm.wz;
		
	//舵轮轮速解算
	helm[2].speed_set = sqrt(pow(-chassis_helm.vx-chassis_helm.wz*sin_40_15,2.0f)+pow(chassis_helm.vy-chassis_helm.wz*cos_40_15,2.0f));
	helm[3].speed_set = sqrt(pow(chassis_helm.vx-chassis_helm.wz*sin_40_15,2.0f)+pow(chassis_helm.vy-chassis_helm.wz*cos_40_15,2.0f));
				
	//舵轮舵向解算
	arm_atan2_f32(chassis_helm.vy-chassis_helm.wz*cos_40_15,  chassis_helm.vx+chassis_helm.wz*sin_40_15, &helm[2].angle_set );
	arm_atan2_f32(chassis_helm.vy-chassis_helm.wz*cos_40_15,  chassis_helm.vx-chassis_helm.wz*sin_40_15, &helm[3].angle_set );
	
	//零点bug处理
	if(chassis_helm.vx==0 && chassis_helm.vy==0) 
		{		
				helm[2].angle_set = -50/57.3f;
				helm[3].angle_set = 50/57.3f;		
				
				helm[2].speed_set = chassis_helm.wz;
				helm[3].speed_set = -chassis_helm.wz;
		
		}
}

//void helm_solve()
//{
//	//全向轮轮速计算
//	helm[0].speed_set = ( chassis_helm.vx * sin_40_15 - chassis_helm.vy * cos_40_15) * 0.87 + chassis_helm.wz;
//	helm[1].speed_set = (  chassis_helm.vx * sin_40_15 + chassis_helm.vy * cos_40_15) * 0.87 + chassis_helm.wz;
//		
//	//舵轮轮速解算
//	helm[2].speed_set = sqrt(pow(chassis_helm.vx-chassis_helm.wz*cos_40_15,2.0f)+pow(chassis_helm.vy-chassis_helm.wz*sin_40_15,2.0f));
//	helm[3].speed_set = sqrt(pow(chassis_helm.vx+chassis_helm.wz*cos_40_15,2.0f)+pow(chassis_helm.vy-chassis_helm.wz*sin_40_15,2.0f));
//				
//	//舵轮舵向解算
//	arm_atan2_f32(chassis_helm.vy-chassis_helm.wz*sin_40_15,  chassis_helm.vx-chassis_helm.wz*cos_40_15, &helm[2].angle_set );
//	arm_atan2_f32(chassis_helm.vy-chassis_helm.wz*sin_40_15,  chassis_helm.vx+chassis_helm.wz*cos_40_15, &helm[3].angle_set );
//	
//	//零点bug处理
//	if(chassis_helm.vx==0 && chassis_helm.vy==0)
//		{		
//				helm[2].angle_set = -50/57.3f;
//				helm[3].angle_set = 50/57.3f;		
//				
//				helm[2].speed_set = - chassis_helm.wz;
//				helm[3].speed_set = chassis_helm.wz;
//		
//		}
//}

