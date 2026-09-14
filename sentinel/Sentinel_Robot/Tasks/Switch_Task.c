#include "Switch_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "My_Def.h"

#include "Shoot_Task.h"
#include "Gimbal_Task.h"

#include "remote_control.h"
#include "Nmanifold_usart_task.h"
#include "Vofa_send.h"

#include "referee.h"
#include "robot_message.h"

#include "main.h"
#include "bsp_math.h"

Gimbal_State_e gimbal_state;
Chassis_State_e chassis_state;
Dial_State_e Dial_State=CLOSE;

fp32 gimbal_spin_speed[2];
uint8_t last_power_gimbal_output = 1;
uint16_t power_gimbal_output_cnt = 0;
aim_control_t aim_control;
uint8_t nuc_lose_spin_flag = 0;
uint8_t have_aim_target_flag = 0;//控云台的时候如果开自瞄，就和省赛差不多，要打人就停下来打（的标志位）
chassis_control_t chassis_control;
 

uint8_t aim_debug=0;
void chassis_solve()
{
    float vx,vy,wz;
	float ang_err=chassis_control.angle_error_rad;	/*云台底盘差角*/

	//设置线速度
	if(chassis_control.follow_chassis_direction) //以底盘朝向为正方向，不进行转换
	{
		vx = chassis_control.vx;
		vy = chassis_control.vy;
	}
	else{	/*以云台朝向为正方向，云台系转底盘系*/
		vx= chassis_control.vx*arm_cos_f32(ang_err)+chassis_control.vy*arm_sin_f32(ang_err);
		vy=-chassis_control.vx*arm_sin_f32(ang_err)+chassis_control.vy*arm_cos_f32(ang_err);
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
        if(fabs(ang_err)<0.01) ang_err=0;
		PID_calc(&chassis_control.chassis_psi,ang_err,0);
//		DEADBAND(chassis_control.chassis_psi.out,50);
        if(fabs(chassis_control.chassis_psi.out)<50)
			chassis_control.chassis_psi.out=0;
		wz = chassis_control.chassis_psi.out;
        			
	}
	else
	{
		wz= chassis_control.wz;
	}

	//发送给下板
	CAN_CMD_BASE(&CAN_COMMUNITE,CAN_COMMUNITE_ID_1,
	(int16_t)vx,
	(int16_t)vy,
	(int16_t)(wz),
	(int16_t)((chassis_control.chassis_follow_gimbal<<1)|(chassis_control.chassis_enable)));
}
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
		

///*--旋转移动--角度修正--*/
	chassis_control.angle_error_rad=ang_err+ 0.13f*(fp32)chassis_control.wz/RADIUS_CHASSIS;//由于功率控制最后会降低转速,转速与实际不符,效果不好0.11
	chassis_control.angle_error_rad=(float)DEADBAND(chassis_control.angle_error_rad,0.01);
	chassis_solve();
}


fp32 rc_pitch_speed;
fp32 rc_yaw_spped;
fp32 rc_pitch;
fp32 rc_yaw;
//void Remote_Control_Gimbal()
//{
//    rc_pitch_speed=0.8*rc_pitch_speed+0.2*rc_ctrl.rc.ch[0];
//    rc_yaw_spped=0.8*rc_yaw_spped+0.2*rc_ctrl.rc.ch[1];
//	  gimbal_vector_set(-rc_pitch_speed*Sw_Wz, rc_yaw_spped*Sw_Pc,0,0,SPEED,SPEED,BASE_YAW_MODE);//ADVANCED_YAW_MODE,BASE_YAW_MODE,ANGLE
//	  if(gimbal_motor[PITCH_6015].INS_angle>0.0f)  rc_pitch_speed=0.0;
//		else if(gimbal_motor[PITCH_6015].INS_angle<-25.0f) rc_pitch_speed=0.0;
//}
void Remote_Control_Gimbal()
{
    rc_pitch=gimbal_motor[PITCH_6015].INS_angle+0.02*rc_ctrl.rc.ch[1];
	if(rc_pitch>0.0f) rc_pitch=0.0f;
	if(rc_pitch<-25.0f) rc_pitch=-25.0f;
	
    //rc_yaw=rc_yaw+0.02*rc_ctrl.rc.ch[0];
	//rc_yaw_spped=0.8*rc_yaw_spped+0.2*rc_ctrl.rc.ch[0];
	rc_yaw_spped=0.2*rc_ctrl.rc.ch[0];
	
    gimbal_vector_set( rc_yaw_spped,0,0,rc_pitch,SPEED, ANGLE,BASE_YAW_MODE);
//    rc_pitch_speed=0.8*rc_pitch_speed+0.2*rc_ctrl.rc.ch[0];
//    rc_yaw_spped=0.8*rc_yaw_spped+0.2*rc_ctrl.rc.ch[1];
//    gimbal_vector_set(-rc_pitch_speed*Sw_Wz, rc_yaw_spped*Sw_Pc,0,0,SPEED,SPEED,BASE_YAW_MODE);//ADVANCED_YAW_MODE,BASE_YAW_MODE,ANGLE
}
void Gimbal_Follow_Chassis() 
{
		gimbal_vector_set(0,rc_ctrl.rc.ch[1]*Sw_Pc,0,0,SPEED,SPEED,GIMBAL_TO_CHASSIS_MODE);
}
fp32 yaw_test=10.0f;
fp32 pitch_test=0.0;
float yaw_speed=0.0;
uint16_t debug_cnt=0;
fp32 debug_yaw_max=6.5f;
void Aim_Control_Gimbal()
{
	/* 输出锯齿波进行调试 62是周期 6.5是峰值*/
//	debug_cnt++;
//debug_cnt %= 600;

//// 360为一个周期，幅值为 6.5 的正弦波
//yaw_test = 6.5f * sinf((float)debug_cnt * 2.0f * 3.14159265f / 600.0f);
    
	/*--keep_wait--*/
//	if(nuc_receive_data.aim_data_received.is_fire!=0)
//	{
//		aim_control.aim_keep=1000;
//		aim_control.aim_keep_angle[0]	=	nuc_receive_data.aim_data_received.yaw;
//		aim_control.aim_keep_angle[1]	=	nuc_receive_data.aim_data_received.pitch;
//	}
//	else if(aim_control.aim_keep>=0)
//	{
//		aim_control.aim_keep--;
//		if (aim_control.aim_keep<0)
//		{
//			aim_control.aim_keep=0;
//		}
//		nuc_receive_data.aim_data_received.yaw		=	aim_control.aim_keep_angle[0];
//		nuc_receive_data.aim_data_received.pitch	=	aim_control.aim_keep_angle[1];
//	}

  	gimbal_vector_set(
        0,0,
//        nuc_receive_data.aim_data_received.top_ampl, 
//        nuc_receive_data.aim_data_received.top_freq,   
        yaw_test,
        //nuc_receive_data.aim_data_received.yaw,
        pitch_test,
        //nuc_receive_data.aim_data_received.pitch,  
        ANGLE, ANGLE,                                 
        ADVANCED_YAW_MODE
    );
}


void Nuc_Control_Chassis(){/*导航底盘控制*/
    fp32 rotor=Rotor_speed;
		
    rotor*=1-0.5f*DATA_LIMIT((nuc_control.chassis_v.vx*1000.0f*nuc_control.chassis_v.vx*1000.0f+nuc_control.chassis_v.vy*1000.0f*nuc_control.chassis_v.vy*1000.0f)/(700*700),0,1);
    
    if (nuc_control.action.mode==NUC_CONTROL_CHASSIS_MOVE)//nuc控制底盘，无视nuc的参数spin
    {
			   if(gimbal_state==GIMBAL_AUTO_AIM)//控云台的时候如果开自瞄，就和省赛差不多，要打人就停下来打
        {
            chassis_vector_set(0,0,rotor*nuc_control.action.spin/*根据标志位选择是否小陀螺 */, gimbal_control.angle_error_rad, CHASSIS_ENABLE);
        }
				else
				{
					chassis_vector_set(nuc_control.chassis_v.vx*1000.0f, nuc_control.chassis_v.vy*1000.0f,nuc_control.chassis_v.wz*RADIUS_CHASSIS, gimbal_control.angle_error_rad, FOLLOW_CHASSIS_DIRECTION | CHASSIS_ENABLE);
				} 
		}
    else if (nuc_control.action.mode==NUC_CONTROL_GIMBAL_MOVE)//nuc控制云台底盘跟头
    {
        if(gimbal_state==GIMBAL_AUTO_AIM)//控云台的时候如果开自瞄，就和省赛差不多，要打人就停下来打
        {
            chassis_vector_set(0,0,rotor*nuc_control.action.spin/*根据标志位选择是否小陀螺 */, gimbal_control.angle_error_rad, CHASSIS_ENABLE);
        }
        else if(nuc_control.action.spin)
        {
            chassis_vector_set(nuc_control.chassis_v.vx*1000.0f,nuc_control.chassis_v.vy*1000.0f, rotor*nuc_control.action.spin, gimbal_control.angle_error_rad, CHASSIS_ENABLE);
        }
    }
    else
    {
        chassis_vector_set(0,0,0, 0,0);
    }
}


void Update_System_Event(void) {
    /* --- 0. 全局强制转移 (Highest Priority) --- */
    if (Switch_Left == RC_SW_DOWN) {
        gimbal_state = GIMBAL_RELAX;
        chassis_state = CHASSIS_RELAX;
        return;
    }

    /* --- 1. 云台状态转移处理 (Gimbal State Transition) --- */
    switch (gimbal_state) {
        case GIMBAL_RELAX:
            aim_control.aim_PID=0;
            if      (Switch_Left == RC_SW_MID)      gimbal_state = GIMBAL_AUTO_AIM;// GIMBAL_RC_CTRL;//v//GIMBAL_RC_CTRL;//拨杆中间默认
            else if (Switch_Left == RC_SW_UP)       gimbal_state = GIMBAL_RELAX;//GIMBAL_NUC_CTRL;
            break;

        case GIMBAL_RC_CTRL:
            aim_control.aim_PID=0;
            if      (Switch_Left == RC_SW_UP)       gimbal_state = GIMBAL_RELAX;//GIMBAL_NUC_CTRL;
            else if (RollWheel < -10)               gimbal_state = GIMBAL_AUTO_AIM; // 手动切自瞄调试 
            break;

        case GIMBAL_AUTO_AIM:
            aim_control.aim_PID=1;
            // 退出条件：目标丢失且保活计时耗尽，或者拨杆切换
//            if      (Switch_Left == RC_SW_MID && RollWheel >= -10) 
//                gimbal_state = GIMBAL_RC_CTRL;//拨杆切换
//            else if ((nuc_receive_data.aim_data_received.is_fire == 0 && aim_control.aim_keep == 0)||nuc_control.action.robot_aim==0) {
//                // 目标丢失后的回退路径
//                if((Switch_Left == RC_SW_MID && RollWheel <= -10)||nuc_receive_data.aim_data_received.is_fire==1)
//                    gimbal_state =GIMBAL_AUTO_AIM;
//                else
//                    gimbal_state = (Switch_Left == RC_SW_UP) ? GIMBAL_PATROL : GIMBAL_RC_CTRL;
//            }
            break;

        case GIMBAL_PATROL:
            aim_control.aim_PID=0;
            if      (gimbal_control.counterattack_flag != 0)                                      gimbal_state = GIMBAL_COUNTERATTACK;
            else if (nuc_receive_data.aim_data_received.is_fire&&nuc_control.action.robot_aim!=0) gimbal_state = GIMBAL_AUTO_AIM;
            else if (nuc_control.action.patrol == 0)                                              gimbal_state = GIMBAL_NUC_CTRL;
            else if (Switch_Left == RC_SW_MID)                                                    gimbal_state = GIMBAL_RC_CTRL;
            break;

        case GIMBAL_COUNTERATTACK:
            aim_control.aim_PID=0;
            // 反击结束回归巡逻或自瞄
            if (gimbal_control.counterattack_flag == 0) {
                gimbal_state = (nuc_receive_data.aim_data_received.is_fire && nuc_control.action.robot_aim!=0) ? GIMBAL_AUTO_AIM : GIMBAL_PATROL;
            }
            break;

        case GIMBAL_NUC_CTRL:
            aim_control.aim_PID=0;
            if      (Switch_Left == RC_SW_MID)                                                    gimbal_state = GIMBAL_RC_CTRL;
            else if (nuc_control.action.patrol != 0)                                              gimbal_state = GIMBAL_PATROL;
            else if (nuc_receive_data.aim_data_received.is_fire&&nuc_control.action.robot_aim!=0) gimbal_state = GIMBAL_AUTO_AIM;
            break;
    }

    /* --- 2. 底盘状态转移处理 (Chassis State Transition) --- */
    switch (chassis_state) {
        case CHASSIS_RELAX:
            if      (Switch_Left == RC_SW_MID)     chassis_state = CHASSIS_RC_CTRL;
            else if (Switch_Left == RC_SW_UP)      chassis_state =  CHASSIS_RELAX;//CHASSIS_NUC_CTRL;
            break;

        case CHASSIS_RC_CTRL:
            if (Switch_Left == RC_SW_UP)           chassis_state =  CHASSIS_RELAX;
            break;

        case CHASSIS_NUC_CTRL:
            if (Switch_Left == RC_SW_MID)          chassis_state = CHASSIS_RC_CTRL;
            break;
    }
}

void Step_Gimbal_FSM(void) {
    switch (gimbal_state) {
        case GIMBAL_RELAX:
            gimbal_vector_set(0, 0, 0, 0, SPEED, SPEED, BASE_YAW_MODE);
            break;

        case GIMBAL_RC_CTRL:
            Remote_Control_Gimbal();
            break;

        case GIMBAL_AUTO_AIM:
            Aim_Control_Gimbal();
            break;

        case GIMBAL_PATROL:
            // 使用NUC下发的限位参数进行巡逻
            Gimbal_Spin(nuc_control.action.max_pitch * PI_TO_180,
                        nuc_control.action.min_pitch * PI_TO_180,
                        gimbal_spin_speed,
                        gimbal_control.counterattack_flag);
            break;

        case GIMBAL_COUNTERATTACK://TODO
            // 反击逻辑
            break;

        case GIMBAL_NUC_CTRL:
            // 模式定义：响应NUC的wz速度指令，或者是单纯的“云台跟底盘”
            if (nuc_control.action.mode == NUC_CONTROL_GIMBAL_MOVE) {
                // 响应NUC的wz控制
                gimbal_vector_set(nuc_control.chassis_v.wz / PI * 180, rc_ctrl.rc.ch[1] * Sw_Pc, 0, 0, SPEED, SPEED, BASE_YAW_MODE);
            } else {
                // 默认跟随底盘
                Gimbal_Follow_Chassis();
            }
            break;
    }
}

void Step_Chassis_FSM(void) {
    fp32 rc_spin = 0;
    switch (chassis_state) {
        case CHASSIS_RELAX:
            chassis_vector_set(0, 0, 0, 0, 0);
            break;

        case CHASSIS_RC_CTRL:
            // 手动控制移动 + 滚轮控制小陀螺
            if (RollWheel > 10) rc_spin = Rotor_speed * (RollWheel / 660.0f);
            if(gimbal_state==GIMBAL_AUTO_AIM)
            {
                 chassis_vector_set(0, 
                               0, 
                               0, 
                               0, 
                               0);
            
            }
            else
            {
                chassis_vector_set(RockingBar_Left_V * Sw_CV, 
                               -RockingBar_Left_H * Sw_CV, 
                               rc_spin, 
                               gimbal_control.angle_error_rad, 
                               CHASSIS_ENABLE | (rc_spin == 0 ? CHASSIS_FLOWE_GIMBAL : 0));
            }
            break;

        case CHASSIS_NUC_CTRL:
            // 统一调用NUC控制函数，内部处理 vx, vy, wz 和 nuc_control.action.spin
            if (nuc_lose_spin_flag) {
//                // 异常处理：失去导航信号时强制原地小陀螺
//                chassis_vector_set(0, 0, Rotor_speed * 0.8f, 0, CHASSIS_ENABLE); 
                chassis_vector_set(0, 0, 0, 0, CHASSIS_ENABLE);
            } else {
                Nuc_Control_Chassis();
            }  
            break;
    }
}

void Switch_Right_Down()	
{	shoot_control.fric_state	=	0;
    Dial_State=CLOSE;
}
void Switch_Right_Mid()		
{	
    shoot_control.fric_state	=	1;
     Dial_State=CLOSE;
}
void Switch_Right_Up()		
{	shoot_control.fric_state	=	1;
}
uint8_t test2=0;
void Update_Shoot_Event()
{
     if(Switch_Left != 0 && Switch_Left != RC_SW_DOWN)
		{//控摩擦轮
				switch (Switch_Right)
				{
				case RC_SW_UP:
					Switch_Right_Up();
					break;
				case RC_SW_MID:
					Switch_Right_Mid();
					break;
				case RC_SW_DOWN:
					Switch_Right_Down();
					break;
				default:
					Switch_Right_Down();
					break;
				}		
		}
        switch(Dial_State)
        {//控拨弹盘
            case CLOSE:
                if(Switch_Right==RC_SW_UP && gimbal_state==GIMBAL_AUTO_AIM)                                                                     { Dial_State=RAPID_FIRE;test2=1;break;}
                if(Switch_Right==RC_SW_UP && gimbal_state==GIMBAL_RC_CTRL)                                                                      { Dial_State=SHOOT_TEST;test2=2;break;}
            case RAPID_FIRE:
                if(nuc_transmit_data.robot_gimbal_data_send.mode ==  2 || nuc_transmit_data.robot_gimbal_data_send.mode == 3)                    { Dial_State= SINGLE_FIRE;break;}
                if(Switch_Right!=RC_SW_UP || gimbal_state!=GIMBAL_AUTO_AIM )                                                                    { Dial_State= CLOSE; test2=3;break;}
            case SINGLE_FIRE:
                if(nuc_transmit_data.robot_gimbal_data_send.mode == 0 || nuc_transmit_data.robot_gimbal_data_send.mode == 1)                    { Dial_State= RAPID_FIRE;break;}
                if(Switch_Right!=RC_SW_UP ||  gimbal_state!=GIMBAL_AUTO_AIM )                                                                   { Dial_State= CLOSE;test2=4; break;}
            case SHOOT_TEST: 
                if(gimbal_state!=GIMBAL_RC_CTRL)                                                                                                { Dial_State= CLOSE;test2=5; break;}
        }
            
}

 

static void Robot_Protect_FSM()
{
     if(Robot_Status.power_management_gimbal_output == 1 && last_power_gimbal_output == 0)
		{
				power_gimbal_output_cnt = 1500;
		}
		if(power_gimbal_output_cnt > 0)
		{
			chassis_vector_set(0,0,0,0,0);
			gimbal_motor[PITCH_6015].set_current = 0;
			gimbal_motor[BASE_YAW_5010].set_current = 0;
			PID_clear(&gimbal_motor[BASE_YAW_5010].angle_pid);
			PID_clear(&gimbal_motor[BASE_YAW_5010].speed_pid);
			PID_clear(&gimbal_motor[PITCH_6015].angle_pid);
			PID_clear(&gimbal_motor[PITCH_6015].speed_pid);
			power_gimbal_output_cnt--;
		}
		
		last_power_gimbal_output = Robot_Status.power_management_gimbal_output;
}
void Switch_Task(void const * argument) 
{
    float chassis_psi_pid[3]={1500,30,500};
    PID_init(&chassis_control.chassis_psi,0,chassis_psi_pid,1000,0);
    while(1) 
    {
		if(Game_Status.game_progress == 3)
		{
			nuc_lose_spin_flag = 1;	
		}
		else if(nuc_control.chassis_v.vx!=0 || nuc_control.chassis_v.vy!=0 || nuc_control.chassis_v.wz!=0 || nuc_control.action.mode !=0 || nuc_control.action.patrol!=0 || nuc_control.action.spin!=0)
		{
			nuc_lose_spin_flag = 0;
		}
        Update_System_Event(); // 状态机决策
        Step_Gimbal_FSM();     // 执行云台
        Step_Chassis_FSM();    // 执行底盘
        
        Update_Shoot_Event();  //摩擦轮状态转移
        
        Robot_Protect_FSM();
        vTaskDelay(1);
    }
}