#include "motor.h"
wheel_motor_t wheel_motor[CH_NUM_WHEEL_MOT];
steer_motor_t steer_motor[CH_NUM_STEER_MOT];
extern chassis_control_t chassis_control;

/**
  * @brief          底盘电机数据接收回调函数，解析CAN总线返回的电机原始数据
  * @param[in]      rx_header: CAN接收报文句柄
  * @param[in]      rx_data: CAN接收数据缓冲区
  * @retval         none
  */
void  Can_Getmeasure_Chmotor(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8])
{
		switch (rx_header.StdId)
		{
             //根据实际情况修改编号情况
			case CAN_ID_WHEEL_MOT_FL:
            {detect_hook(CH_WHEEL_MOT1_TOE);get_djimotor_measure(&wheel_motor[1].motor_data_raw, rx_data);break;}
			case CAN_ID_WHEEL_MOT_FR:
            {detect_hook(CH_WHEEL_MOT2_TOE);get_djimotor_measure(&wheel_motor[0].motor_data_raw, rx_data);break;}
			case CAN_ID_WHEEL_MOT_BL:
            {detect_hook(CH_WHEEL_MOT3_TOE);get_djimotor_measure(&wheel_motor[2].motor_data_raw, rx_data);break;}
			case CAN_ID_WHEEL_MOT_BR:
			{
               detect_hook(CH_WHEEL_MOT4_TOE);get_djimotor_measure(&wheel_motor[3].motor_data_raw, rx_data);break;
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_ID_WHEEL_MOT_FL;
                #ifdef CH_CFG_WHEEL_MOT_LK_6015
				motor_measure_LK(&wheel_motor[i].motor_data_raw, rx_data);
                #endif 
				break;
			}
            #ifdef CH_CFG_TYPE_SWERVE 
            //根据实际情况修改电机编号情况
			case CAN_ID_STEER_MOT_FR :
			{detect_hook(CH_STEER_MOT1_TOE);get_djimotor_measure(&steer_motor[0].motor_data_raw, rx_data);break;}
			case CAN_ID_STEER_MOT_FL :
			{detect_hook(CH_STEER_MOT2_TOE);get_djimotor_measure(&steer_motor[1].motor_data_raw, rx_data);break;}
            case CAN_ID_STEER_MOT_BL:
            {detect_hook(CH_STEER_MOT3_TOE);get_djimotor_measure(&steer_motor[2].motor_data_raw, rx_data);break;}
            case CAN_ID_STEER_MOT_BR:
            {detect_hook(CH_STEER_MOT4_TOE);get_djimotor_measure(&steer_motor[3].motor_data_raw, rx_data);break;}
            #endif 
         }

}

/**
  * @brief          PID参数初始化辅助函数
  * @param[out]     pid: PID结构体指针
  * @param[in]      mode: PID模式（位置式或增量式）
  * @param[in]      kp/ki/kd: PID三项增益参数
  * @param[in]      max_out: 最大输出限制
  * @param[in]      max_iout: 最大积分输出限制
  * @retval         none
  */
void Pid_Init_motor(pid_type_def *pid, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
{
        fp32 pid_para[3]={kp,ki,kd};
        PID_init(pid,mode,pid_para,max_out,max_iout);
}

/**
  * @brief          底盘所有电机（驱动轮与转向电机）的PID控制器初始化
  * @param[in]      none
  * @retval         none
  */
 void Pid_Init_CHmotor()
{
    for(int i=0;i<CH_NUM_WHEEL_MOT;i++)
    {
        Pid_Init_motor(&wheel_motor[i].wheel_speed_pid,0,CH_PARAM_WHEEL_SPEED_PID_KP,CH_PARAM_WHEEL_SPEED_PID_KI,CH_PARAM_WHEEL_SPEED_PID_KD,CH_PARAM_WHEEL_SPEED_PID_MAXOUT,CH_PARAM_WHEEL_SPEED_PID_MAXIOUT);
    }
    
    for(int i=0;i<CH_NUM_STEER_MOT;i++)
    {
        Pid_Init_motor(&steer_motor[i].steer_speed_pid,0,CH_PARAM_STEER_SPEED_PID_KP,CH_PARAM_STEER_SPEED_PID_KI,CH_PARAM_STEER_SPEED_PID_KD,CH_PARAM_STEER_SPEED_PID_MAXOUT,CH_PARAM_STEER_SPEED_PID_MAXIOUT);
        Pid_Init_motor(&steer_motor[i].steer_angle_pid,0,CH_PARAM_STEER_ANGLE_PID_KP,CH_PARAM_STEER_ANGLE_PID_KI,CH_PARAM_STEER_ANGLE_PID_KD,CH_PARAM_STEER_ANGLE_PID_MAXOUT,CH_PARAM_STEER_ANGLE_PID_MAXIOUT);
    }
}


/**
  * @brief          全舵底盘运动学逆解算，将底盘目标速度转换为各轮速和舵向角度
  * @param[in]      chassis_control: 底盘控制目标向量 (vx, vy, wz)
  * @retval         none
  */
void CH_Solve_motor()
{
    steer_motor[0].ecd_offset_rad=530;
    steer_motor[1].ecd_offset_rad=4900;
    steer_motor[2].ecd_offset_rad=610;//4868;
    steer_motor[3].ecd_offset_rad=5010;//4600;
    //0:4683
    //1:880
    //2:4862
    //3:486
    float vx = chassis_control.ch_vx_set;
    float vy = -chassis_control.ch_vy_set;
    float wz = chassis_control.ch_wz_set;
    
    // 轮0 (右前) 分量
    float v0_x = vx + wz * sin_40_15;
    float v0_y = vy - wz * cos_40_15;
    
    // 轮1 (左前) 分量
    float v1_x = vx - wz * sin_40_15;
    float v1_y = vy - wz * cos_40_15;
    
    // 轮2 (左后) 分量
    float v2_x = vx - wz * sin_40_15;
    float v2_y = vy + wz * cos_40_15;
    
    // 轮3 (右后) 分量
    float v3_x = vx +  wz * sin_40_15;
    float v3_y = vy + wz * cos_40_15;

    // 2. 轮速解算 (速度幅值)
    wheel_motor[0].speed_set = sqrt(pow(v0_x, 2.0f) + pow(v0_y, 2.0f));
    wheel_motor[1].speed_set = sqrt(pow(v1_x, 2.0f) + pow(v1_y, 2.0f));
    wheel_motor[2].speed_set = sqrt(pow(v2_x, 2.0f) + pow(v2_y, 2.0f));
    wheel_motor[3].speed_set = sqrt(pow(v3_x, 2.0f) + pow(v3_y, 2.0f));

    // 3. 舵向解算 (角度)
    arm_atan2_f32(v0_y, v0_x, &steer_motor[0].angle_set); // 对应轮0右前
    arm_atan2_f32(v1_y, v1_x, &steer_motor[1].angle_set); // 对应轮1左前
    arm_atan2_f32(v2_y, v2_x, &steer_motor[2].angle_set); // 对应轮2左后
    arm_atan2_f32(v3_y, v3_x, &steer_motor[3].angle_set); // 对应轮3右后

    // 4. 零点/静止状态处理
    if(vx == 0 && vy == 0) 
    {  
        if(wz==0)
        {
        steer_motor[0].angle_set = 0;   // 轮0
        steer_motor[1].angle_set = 0;  // 轮1
        steer_motor[2].angle_set = 0;  // 轮2
        steer_motor[3].angle_set = 0;   // 轮3
        }
    }
    for(int i=0;i<4;i++)
        {
            //找两头离设定角最近的一头
            fp32 angle_err1=limit_pi((steer_motor[i].motor_data_raw.ecd - steer_motor[i].ecd_offset_rad) / SCALE_6020ECD_TO_RAD - steer_motor[i].angle_set);
            steer_motor[i].angle_err=angle_err1;
            if(fabs(steer_motor[i].angle_err)>0.5*PI)
            {
                steer_motor[i].angle_err = steer_motor[i].angle_err - copysignf(PI, steer_motor[i].angle_err);
                wheel_motor[i].speed_set*=-1;
            }
        }
}


/**434
  * @brief          底盘PID控制计算
  * @param[out]     steer_motor: 更新舵机PID输出
  * @param[out]     wheel_motor: 更新轮电机PID输出
  * @retval         none
  */
void Ch_Calc_Motorpid()
{
	for(uint8_t i=0;i<4;i++)
	{
         #ifdef CH_CFG_TYPE_SWERVE 
        //舵电机
		PID_calc(&steer_motor[i].steer_angle_pid , steer_motor[i].angle_err , 0);
		PID_calc(&steer_motor[i].steer_speed_pid , steer_motor[i].motor_data_raw.speed_rpm , steer_motor[i].steer_angle_pid.out);
		#endif
        
		//轮电机
		PID_calc(&wheel_motor[i].wheel_speed_pid , wheel_motor[i].motor_data_raw.speed_rpm , wheel_motor[i].speed_set*CH_SCALE_V_TO_RMP);
	}
}


/**
  * @brief          通过CAN总线发送底盘各电机的控制电流/电压指令
  * @param[in]      wheel_motor: 驱动轮PID输出值
  * @param[in]      steer_motor: 转向电机PID输出值
  * @retval         none
  */
void Can_Tansmit_Chmotor_current()
{
	CAN_CMD_BASE(&CAN_CHMOTOR_MESSAGE,CAN_ID_WHEEL_MOT_ALL,wheel_motor[0].wheel_speed_pid.out , wheel_motor[1].wheel_speed_pid.out , wheel_motor[3].wheel_speed_pid.out , wheel_motor[2].wheel_speed_pid.out);
	vTaskDelay(1);
    #ifdef CH_CFG_TYPE_SWERVE
    CAN_CMD_BASE(&CAN_CHMOTOR_MESSAGE,CAN_ID_STEER_MOT_ALL,steer_motor[3].steer_speed_pid.out , steer_motor[1].steer_speed_pid.out , steer_motor[0].steer_speed_pid.out , steer_motor[2].steer_speed_pid.out);
    vTaskDelay(1);
    #endif
}

/**
  * @brief          底盘电机安全失能，向CAN总线发送零电流控制指令
  * @param[in]      none
  * @retval         none
  */
void  Can_Tansmit_Chmotor_off()
{
	CAN_CMD_BASE(&CAN_CHMOTOR_MESSAGE,CAN_ID_WHEEL_MOT_ALL,0,0,0,0);	vTaskDelay(1);
    #ifdef CH_CFG_TYPE_SWERVE
	CAN_CMD_BASE(&CAN_CHMOTOR_MESSAGE,CAN_ID_STEER_MOT_ALL,0,0,0,0);	vTaskDelay(1);
    #endif
}