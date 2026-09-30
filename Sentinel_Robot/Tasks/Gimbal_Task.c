#include "Gimbal_Task.h"
#include "arm_math.h"
#include "INS_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Vofa_send.h"
#include "main.h"
#include "stdlib.h"
#include "Switch_Task.h"
#include "dm_imu.h"
#include "robot_message.h"
#include "config_set.h"
#include "AHRS_middleware.h"
#include "remote_control.h"
#include "bsp_math.h"
#include "bsp_can.h"
#include "Nmanifold_usart_task.h"
#define SPIN_PITCH    66
#define SPIN_YAW      90

gimbal_motor_t gimbal_motor[3];
gimbal_control_t gimbal_control;   

fp32 gimbal_psi_pid[3];

fp32 base_yaw_motor_auto_aim_speed_pid[3];
fp32 base_yaw_motor_auto_aim_angle_pid[3];

fp32 advanced_yaw_motor_auto_aim_speed_pid[3];
fp32 advanced_yaw_motor_auto_aim_angle_pid[3];

fp32 pitch_motor_auto_aim_speed_pid[3];
fp32 pitch_motor_auto_aim_angle_pid[3];

fp32 base_yaw_motor_speed_pid[3];
fp32 base_yaw_motor_angle_pid[3];

fp32 advanced_yaw_motor_speed_pid[3];
fp32 advanced_yaw_motor_angle_pid[3];

fp32 pitch_motor_speed_pid[3];
fp32 pitch_motor_angle_pid[3];

uint8_t last_aim_PID_flag_switch;
uint8_t aim_cnt;

extern fp32 yaw_test;
extern float adv_yaw,adv_pitch;
extern float adv_yaw_speed,adv_pitch_speed;
extern float nuc_aim_yaw;
extern float nuc_aim_pitch;
extern nuc_control_t nuc_control;
fp32 k1=1,k2=1;

uint16_t aaa=0;
/*外部控制接口*/
/*基础yaw_pitch控制*/
void gimbal_vector_set(fp32 yaw_speed, fp32 pitch_speed, fp32 yaw_angle, fp32 pitch_angle, uint8_t yaw_mode, uint8_t pitch_mode, uint8_t gimbal_mode)
{	
    gimbal_control.MODE = gimbal_mode;

    /* --- YAW 轴处理 --- */
    gimbal_motor_t *yaw_motor;
    if (gimbal_mode == BASE_YAW_MODE)          yaw_motor = &gimbal_motor[BASE_YAW_5010];
    else if (gimbal_mode == ADVANCED_YAW_MODE) yaw_motor = &gimbal_motor[ADVANCED_YAW_6020];
    else yaw_motor = NULL;
        // 模式切换边缘检测：从 SPEED 切换到 ANGLE 的瞬间同步角度
    if (yaw_mode == ANGLE && yaw_motor->control_mode == SPEED) {
        yaw_motor->INS_angle_set = yaw_motor->INS_angle;
        PID_clear(&yaw_motor->angle_pid); // 防止积分残留导致跳变
    }
        
    yaw_motor->control_mode = yaw_mode;
        
    if (yaw_mode == ANGLE) {
        yaw_motor->INS_angle_set = yaw_angle;
        // 在角度模式下，yaw_speed 作为前馈项（Feedforward）
        yaw_motor->INS_speed_feedforward = yaw_speed; 
    
    } 
    else {
        yaw_motor->INS_angle_set = yaw_angle;
        yaw_motor->INS_speed_set = yaw_speed;
    }
	
    /* --- PITCH 轴处理 --- */
    gimbal_motor_t *pitch_motor = &gimbal_motor[PITCH_6015];
    
    // 模式切换边缘检测
    if (pitch_mode == ANGLE && pitch_motor->control_mode == SPEED) {
        pitch_motor->INS_angle_set = pitch_motor->INS_angle;
        PID_clear(&pitch_motor->angle_pid);
    }
    
    pitch_motor->control_mode = pitch_mode;
    if (pitch_mode == ANGLE) {
        pitch_motor->INS_angle_set = pitch_angle;
        pitch_motor->INS_speed_feedforward = pitch_speed; // 前馈
    } else {
        //pitch_motor->INS_angle_set = pitch_angle;
        pitch_motor->INS_speed_set = pitch_speed;
    }
}
  

/*巡逻模式控制外部接口*/
void Gimbal_Spin(fp32 Pitch_max,fp32 Pitch_min,fp32*gimbal_spin_speed,uint8_t mode)
{
	//赋初值
	if(gimbal_spin_speed[PITCH]==0) 			gimbal_spin_speed[PITCH]=SPIN_PITCH;
	/*---yaw巡逻---*/
	gimbal_spin_speed[BASE_YAW]=DEFAULT_YAW_SPIN_SPEED;
	/*---pitch巡逻---*/
	//限幅
	if(Pitch_max>=32.0)Pitch_max=32.0f;
	if(Pitch_min<=-15.0)Pitch_min=-15.0f; 
	
	if(Pitch_max==Pitch_min)
	{
		// gimbal_spin_speed[PITCH]=0;
		gimbal_vector_set(gimbal_spin_speed[BASE_YAW],gimbal_spin_speed[PITCH],0,Pitch_min,SPEED,ANGLE,BASE_YAW_MODE);
	}
	else{
		//换方向
		if(gimbal_motor[PITCH_6015].INS_angle<Pitch_min+1.0)			gimbal_spin_speed[PITCH]=0.4f*gimbal_motor[PITCH_6015].INS_speed+0.6f*SPIN_PITCH;
		else if(gimbal_motor[PITCH_6015].INS_angle>Pitch_max-1.0)	  	gimbal_spin_speed[PITCH]=0.55f*gimbal_motor[PITCH_6015].INS_speed-0.45f*SPIN_PITCH;
		//保持速度
		else
		{
					if(gimbal_spin_speed[PITCH]>=0)gimbal_spin_speed[PITCH]=SPIN_PITCH*0.65;
			else 	if(gimbal_spin_speed[PITCH]<=0)gimbal_spin_speed[PITCH]=-SPIN_PITCH*0.65;
		}
		gimbal_vector_set(gimbal_spin_speed[BASE_YAW],gimbal_spin_speed[PITCH],0,0,SPEED,SPEED,BASE_YAW_MODE);
	}
}  

float kp_angle=0.5f;//4.4
float kp_speed=-100.0f;//-120.0
void CHANGE_PID_FIXED(gimbal_motor_t *adv_yaw,float INS_angle)//自适应PID速度环
{
	if(INS_angle<-15.0f)
	{
		adv_yaw->speed_pid.Kp=300.0f;
		adv_yaw->speed_pid.Ki=0.8f;
		adv_yaw->speed_pid.Kd=30.0f;
				
		adv_yaw->angle_pid.Kp=-0.2069f*(INS_angle)+4.897f;
		
	}
	else if(INS_angle<=-5.0f&&INS_angle>=-15.0f)
	{
		adv_yaw->speed_pid.Kp=10.0f*(INS_angle)+450.0f;
		adv_yaw->speed_pid.Ki=0.8f;
		adv_yaw->speed_pid.Kd=1.0f*(INS_angle)+45.0f;
		
		adv_yaw->angle_pid.Kp=-0.446f*(INS_angle)+1.31f;
	}
	else if(INS_angle<5.0f&&INS_angle>-5.0f)
	{
		adv_yaw->speed_pid.Kp=-23.0f*(INS_angle)+285.0f;
		adv_yaw->speed_pid.Ki=0.8f;
		adv_yaw->speed_pid.Kd=-2.5f*(INS_angle)+27.5f;
		
		adv_yaw->angle_pid.Kp=-0.134f*(INS_angle)+2.87f;
	}
	else if(INS_angle<=15.0f&&INS_angle>=5.0f)
	{
		adv_yaw->speed_pid.Kp=-2.0f*(INS_angle)+180.0f;
		adv_yaw->speed_pid.Ki=0.2f;
		adv_yaw->speed_pid.Kd=15.0f;
		
		adv_yaw->angle_pid.Kp=0.2f*(INS_angle)+1.2f;
	}
	else 
	{
		adv_yaw->speed_pid.Kp=150.0f;
		adv_yaw->speed_pid.Ki=0.2f;
		adv_yaw->speed_pid.Kd=15.0f;
		
		adv_yaw->angle_pid.Kp=4.1f;
	}
	adv_yaw->angle_pid.Ki=0.0f;	
	adv_yaw->angle_pid.Kd=30.0f;
	if(gimbal_control.MODE == ADVANCED_YAW_MODE)//自瞄补偿
	{
		adv_yaw->angle_pid.Kp+=kp_angle;
		adv_yaw->speed_pid.Kp+=kp_speed;
		adv_yaw->speed_pid.Kd+=10.0f;
	}
	if(gimbal_control.MODE == BASE_YAW)//自瞄补偿
	{
		adv_yaw->angle_pid.Kp+=kp_angle;
		adv_yaw->speed_pid.Kp+=kp_speed;
		adv_yaw->speed_pid.Kd+=10.0f;
	}
}	
	
//	adv_yaw->angle_pid.Kp=0.0089f*(pitch->INS_angle)*(pitch->INS_angle)-0.0958f*(pitch->INS_angle)+3.2088f;
//	adv_yaw->angle_pid.Ki=0;
//	adv_yaw->angle_pid.Kd=-0.0009f*(pitch->INS_angle)*(pitch->INS_angle)-0.0081f*(pitch->INS_angle)+0.3915f;
//	if(adv_yaw->angle_pid.Kd<=0.5)
//	{
//		adv_yaw->angle_pid.Kd=0.5f;
//	}
//	adv_yaw->speed_pid.Kp=-0.2773f*(pitch->INS_angle)*(pitch->INS_angle)-6.8986f*(pitch->INS_angle)+290.18f;
//	if(adv_yaw->speed_pid.Kp<=150.0f)
//	{
//		adv_yaw->speed_pid.Kp=150.0f;
//	}
//	adv_yaw->speed_pid.Ki=0.6f;
//	adv_yaw->speed_pid.Kd=-0.026f*(pitch->INS_angle)*(pitch->INS_angle)-0.6955f*(pitch->INS_angle)+28.23f;
//	if(adv_yaw->speed_pid.Kd<=8.0f)
//	{
//		adv_yaw->speed_pid.Kd=8.0f;
//	}
//}

/* 云台电机初始化	*/
void Gimbal_Motor_Init(void)
{
    gimbal_psi_pid[0]=800;
    gimbal_psi_pid[1]=0; 
    gimbal_psi_pid[2]=100;
	/* yaw电机pid参数 */
	base_yaw_motor_speed_pid[0] = BASE_YAW_MOTOR_SPEED_PID_KP;
	base_yaw_motor_speed_pid[1] = BASE_YAW_MOTOR_SPEED_PID_KI;
	base_yaw_motor_speed_pid[2] = BASE_YAW_MOTOR_SPEED_PID_KD;

	base_yaw_motor_angle_pid[0] = BASE_YAW_MOTOR_ANGLE_PID_KP;
	base_yaw_motor_angle_pid[1] = BASE_YAW_MOTOR_ANGLE_PID_KI;
	base_yaw_motor_angle_pid[2] = BASE_YAW_MOTOR_ANGLE_PID_KD;
	
	pitch_motor_speed_pid[0] = PITCH_MOTOR_SPEED_PID_KP;
	pitch_motor_speed_pid[1] = PITCH_MOTOR_SPEED_PID_KI;     
	pitch_motor_speed_pid[2] = PITCH_MOTOR_SPEED_PID_KD;

	pitch_motor_angle_pid[0] = PITCH_MOTOR_ANGLE_PID_KP;
	pitch_motor_angle_pid[1] = PITCH_MOTOR_ANGLE_PID_KI;
	pitch_motor_angle_pid[2] = PITCH_MOTOR_ANGLE_PID_KD;
	
	base_yaw_motor_auto_aim_speed_pid[0] = BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KP;
	base_yaw_motor_auto_aim_speed_pid[1] = BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KI;
	base_yaw_motor_auto_aim_speed_pid[2] = BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KD;

	base_yaw_motor_auto_aim_angle_pid[0] = BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KP;
	base_yaw_motor_auto_aim_angle_pid[1] = BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KI;
	base_yaw_motor_auto_aim_angle_pid[2] = BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KD;
	
	advanced_yaw_motor_auto_aim_speed_pid[0] = ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KP;
	advanced_yaw_motor_auto_aim_speed_pid[1] = ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KI;
	advanced_yaw_motor_auto_aim_speed_pid[2] = ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KD;

	advanced_yaw_motor_auto_aim_angle_pid[0] = ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KP;
	advanced_yaw_motor_auto_aim_angle_pid[1] = ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KI;
	advanced_yaw_motor_auto_aim_angle_pid[2] = ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KD;
	
	pitch_motor_auto_aim_speed_pid[0] = PITCH_MOTOR_SPEED_AUTO_AIM_PID_KP;
	pitch_motor_auto_aim_speed_pid[1] = PITCH_MOTOR_SPEED_AUTO_AIM_PID_KI;
	pitch_motor_auto_aim_speed_pid[2] = PITCH_MOTOR_SPEED_AUTO_AIM_PID_KD;

	pitch_motor_auto_aim_angle_pid[0] = PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KP;
	pitch_motor_auto_aim_angle_pid[1] = PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KI;
	pitch_motor_auto_aim_angle_pid[2] = PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KD;
	
	advanced_yaw_motor_speed_pid[0] = ADVANCED_YAW_MOTOR_SPEED_PID_KP;
	advanced_yaw_motor_speed_pid[1] = ADVANCED_YAW_MOTOR_SPEED_PID_KI;
	advanced_yaw_motor_speed_pid[2] = ADVANCED_YAW_MOTOR_SPEED_PID_KD;
	
	advanced_yaw_motor_angle_pid[0] = ADVANCED_YAW_MOTOR_ANGLE_PID_KP;
	advanced_yaw_motor_angle_pid[1] = ADVANCED_YAW_MOTOR_ANGLE_PID_KI;
	advanced_yaw_motor_angle_pid[2] = ADVANCED_YAW_MOTOR_ANGLE_PID_KD;
	
	/* yaw轴电机数据初始化为0 */
	for(uint8_t i=0;i<3;i++)	memset((uint8_t*)&gimbal_motor[i],0,sizeof(gimbal_control_t));
    
    PID_init(&gimbal_control.gimbal_psi, PID_POSITION,gimbal_psi_pid,4500,0);
	
	PID_init(&gimbal_motor[BASE_YAW_5010].speed_pid,PID_POSITION,base_yaw_motor_speed_pid,BASE_YAW_MOTOR_SPEED_PID_MAX_OUT,BASE_YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[BASE_YAW_5010].angle_pid,PID_POSITION,base_yaw_motor_angle_pid,BASE_YAW_MOTOR_ANGLE_PID_MAX_OUT,BASE_YAW_MOTOR_ANGLE_PID_MAX_IOUT);

	PID_init(&gimbal_motor[ADVANCED_YAW_6020].speed_pid,PID_POSITION,advanced_yaw_motor_speed_pid,ADVANCED_YAW_MOTOR_SPEED_PID_MAX_OUT,ADVANCED_YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[ADVANCED_YAW_6020].angle_pid,PID_POSITION,advanced_yaw_motor_angle_pid,ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_OUT,ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_IOUT);

	PID_init(&gimbal_motor[PITCH_6015].speed_pid,PID_POSITION,pitch_motor_speed_pid,PITCH_MOTOR_SPEED_PID_MAX_OUT,PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[PITCH_6015].angle_pid,PID_POSITION,pitch_motor_angle_pid,PITCH_MOTOR_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ANGLE_PID_MAX_IOUT);
	
	/* yaw零点 */	
	gimbal_control.advanced_gimbal_zero = ADVANCED_GIMBAL_ANGLE_ZERO;
	gimbal_control.pitch_angle_zero = PITCH_ANGLE_ZERO;

	
	/* 根据大yaw陀螺仪和小yaw编码器，算出小yaw当前角度；相对于初始化上电的坐标系 */
	gimbal_control.advanced_yaw_angle_init = LIMIT_TO_SET( (gimbal_control.advanced_gimbal_zero - motor_measure_gimbal[ADVANCED_YAW].ecd)/((fp32)GIMBAL_MOTOR_ADVANCED_ECD_RANGE)*2*180,180);
}


void FL_Fourlink_angle_solve2(fp32 theta)  
{
	uint16_t l0=45;uint16_t l1=65;uint16_t l2=57;uint16_t l3=75;
	fp32 lAC = sqrt(l3*l3 + l2*l2 - 2*l2*l3*arm_cos_f32((theta+45.72)*PI/180));
	fp32 OAC = acos((l3*l3 + lAC*lAC - l2*l2) / (2*l3*lAC)); 
	fp32 CAB = acos((l0*l0 +lAC*lAC - l1*l1)/ (2*l0*lAC));
	gimbal_motor[PITCH_6015].INS_angle = ((OAC + CAB)/PI*180 - 113) - 17.86 + gimbal_motor[BASE_YAW_5010].BASE_INS_angle_Pitch;
}




fp32 pitch_angle_solve(uint16_t ecd)
{
    int32_t relative_ecd;

    // 1. 将当前 ecd 转换到以 PITCH_ENC_AT_MIN 为起点的线性空间
    if (ecd <= PITCH_ENC_MIN) {
        // 情况 A: 编码器在 56830 ~ 65535 之间
        relative_ecd = -ecd + PITCH_ENC_MIN;
    } else {
        // 情况 B: 编码器已过零点，在 0 ~ 3110 之间
        relative_ecd = ( GIMBAL_MOTOR_PITCH_ECD_RANGE-ecd) + PITCH_ENC_MIN;
    }

    // 2. 线性映射公式: 角度 = 起始角度 + (当前相对位移 / 总位移) * 总角度范围
    fp32 angle = PITCH_ANGLE_MIN + ((fp32)relative_ecd / PITCH_ECD_TOTAL) * PITCH_ANGLE_TOTAL;

    // 3. 安全限幅
    if (angle > PITCH_ANGLE_MAX) angle = PITCH_ANGLE_MAX;
    if (angle < PITCH_ANGLE_MIN) angle = PITCH_ANGLE_MIN;

    return angle;
}

//云台电机数据更新

float check=0.0f;

void Gimbal_Motor_Data_Update(void)
{
	// yaw ----- 陀螺仪角速度 角度 编码器 电流
   
	gimbal_motor[BASE_YAW_5010].INS_speed		=	 	(bmi088_real_data.gyro[2]*RAD_TO_ANGLE)*0.3f	+	(gimbal_motor[BASE_YAW_5010].INS_speed)*0.7f;
	gimbal_motor[BASE_YAW_5010].INS_angle		=		INS_angle_deg[BASE_YAW_5010];
	gimbal_motor[BASE_YAW_5010].BASE_INS_angle_Pitch		=		-INS_angle_deg[1];
	gimbal_motor[BASE_YAW_5010].ENC_angle		=		motor_measure_gimbal[BASE_YAW].ecd;
	gimbal_motor[BASE_YAW_5010].give_current	=		motor_measure_gimbal[BASE_YAW].given_current;
		
	/* 修改低通滤波，可抑制pitch轴抖  */
	gimbal_motor[PITCH_6015].INS_speed			=		adv_pitch_speed;
	gimbal_motor[PITCH_6015].give_current		=		motor_measure_gimbal[PITCH].given_current;
	gimbal_motor[PITCH_6015].ENC_angle = LIMIT_TO_SET((PITCH_ANGLE_ZERO - motor_measure_gimbal[PITCH].ecd)/((fp32)GIMBAL_MOTOR_PITCH_ECD_RANGE)*2*180,180);
	/* 平行四连杆结合电机编码器解算pitch轴角度 */
	 gimbal_motor[PITCH_6015].INS_angle         =  adv_pitch;
	
	CHANGE_PID_FIXED(&gimbal_motor[ADVANCED_YAW_6020],gimbal_motor[PITCH_6015].INS_angle);
	
	
	gimbal_motor[ADVANCED_YAW_6020].ENC_angle   = LIMIT_TO_SET((ADVANCED_GIMBAL_ANGLE_ZERO - motor_measure_gimbal[ADVANCED_YAW].ecd)/((fp32)GIMBAL_MOTOR_ADVANCED_ECD_RANGE)*2*180,180);
	if(adv_pitch<=0)
	{
		gimbal_motor[ADVANCED_YAW_6020].INS_speed	= adv_yaw_speed;
	}
	else
	{
		gimbal_motor[ADVANCED_YAW_6020].INS_speed	= -adv_yaw_speed;
	}
   	gimbal_motor[ADVANCED_YAW_6020].INS_angle	= adv_yaw;
	  
	/*底盘跟头/偏差角*/
	gimbal_control.angle_error_rad = (LIMIT_TO_SET((CHASSIS_FOLLOW_BASE_GIMBAL_ANGLE_ZERO-motor_measure_gimbal[BASE_YAW].ecd)/((fp32)GIMBAL_MOTOR_BASE_ECD_RANGE)*2*PI,PI))*0.5+gimbal_control.angle_error_rad*0.5;
//	check=gimbal_motor[ADVANCED_YAW_6020].INS_speed*cosf(gimbal_motor[PITCH_6015].INS_angle)+gimbal_motor[PITCH_6015].INS_speed*sinf(gimbal_motor[PITCH_6015].INS_angle);
}

/**
 * @brief 通用电机 PID 计算算子
 * @param motor 电机结构体指针
 * @note  这个函数不处理业务逻辑，只负责数学计算
 */

float speed_loop_add = 0.0f;
float temp_target_speed = 0.0f;  

static void Gimbal_BaseYaw_Motor_Operator(gimbal_motor_t *motor) 
{
    static fp32 lock_angle;
    if (motor->control_mode == SPEED) 
    {
        // 单速度环
       PID_calc(&motor->speed_pid, motor->INS_speed, motor->INS_speed_set);
        motor->set_current = motor->speed_pid.out;
    } 
    else if (motor->control_mode == ANGLE) 
    {
        //角度环计算
        PID_calc(&motor->angle_pid, motor->INS_angle_err, 0);
        fp32 target_speed = motor->angle_pid.out - gimbal_motor[ADVANCED_YAW_6020].INS_speed;
        // 速度环计算
        PID_calc(&motor->speed_pid, motor->INS_speed, target_speed);
        motor->set_current = motor->speed_pid.out;
    }
    
}

 int fric_feedforward=0;
static void Gimbal_AdvYaw_Motor_Operator(gimbal_motor_t *motor) 
{
    static fp32 lock_angle;

    if (motor->control_mode == SPEED) 
    {
        // 单速度环
       PID_calc(&motor->speed_pid, motor->INS_speed, motor->INS_speed_set);
        motor->set_current = motor->speed_pid.out;
    } 
    else if (motor->control_mode == ANGLE) 
    {			
        //角度环计算
        PID_aim_calc(&motor->angle_pid, motor->INS_angle_err, 0);
        fp32 target_speed =-motor->angle_pid.out + -motor->INS_speed_feedforward ; 
              float set_target_percent = 1.0f;
        
        if(fabs(motor->INS_angle_err) > 15.0f){
            set_target_percent = 0.25f;
        }else{
            set_target_percent = (15.0f - fabs(motor->INS_angle_err)) / 15.0f;
        }
		if(fabs(motor->INS_angle_err) < 0.1f)
		{
			set_target_percent=0;
		}
       // 速度环计算
		
        PID_calc(&motor->speed_pid, motor->INS_speed, target_speed*set_target_percent);
        if(fabs(target_speed)>0.5)
            fric_feedforward=(target_speed>0)? -350:350;
        else
            fric_feedforward=0;
        motor->set_current =motor->speed_pid.out;
    }
    
}


float gravity_feedforward=160.0f;
float friction_feedfoward=0;
float real_friction_feedforword=0;

static void Gimbal_Pitch_Motor_Operator(gimbal_motor_t *motor) 
{
    static fp32 lock_angle;
    if(rc_ctrl.rc.s[1] == 0x02){
        PID_clear(&motor->speed_pid);
        PID_clear(&motor->angle_pid);
        return;
    }
    if (motor->control_mode == SPEED) 
    {
       PID_calc(&motor->speed_pid, motor->INS_speed, motor->INS_speed_set);
//        gravity_feedforward=-240*cos((2*-fabs(motor->INS_angle)+49)*0.0174533); //可疑补偿
		gravity_feedforward=-0.0629f*(motor->INS_angle)*(motor->INS_angle)-0.3043f*(motor->INS_angle)-200.74f;
		real_friction_feedforword=(motor->INS_speed_set>0)? -friction_feedfoward: friction_feedfoward;
		if(real_friction_feedforword>70.0f)
		{
			real_friction_feedforword=70.0f;
		}
		else if(real_friction_feedforword<-70.0f)
		{
			real_friction_feedforword=-70.0f;
		}
        motor->set_current =-motor->speed_pid.out+gravity_feedforward;
    } 
    else if (motor->control_mode == ANGLE) 
    {
        //角度环计算
        PID_calc(&motor->angle_pid, motor->INS_angle_err, 0);
        fp32 target_speed = -k1*motor->angle_pid.out - k2*motor->INS_speed_feedforward ;
        // 速度环计算
        PID_calc(&motor->speed_pid, motor->INS_speed, target_speed);
//      gravity_feedforward=-240*cos((2*-fabs(motor->INS_angle)+49)*0.0174533); //可疑补偿
		friction_feedfoward=-0.0181f*(motor->INS_angle)*(motor->INS_angle)+0.7598f*(motor->INS_angle)+73.686f;
		gravity_feedforward=-0.0629f*(motor->INS_angle)*(motor->INS_angle)-0.3043f*(motor->INS_angle)-200.74f;
		real_friction_feedforword=(target_speed>0)? -friction_feedfoward: friction_feedfoward;
		if(fabsf(target_speed)>1.8f)
		{
			if(real_friction_feedforword>100.0f)
			{
				real_friction_feedforword=100.0f;
			}
			else if(real_friction_feedforword<-100.0f)
			{
				real_friction_feedforword=-100.0f;
			}
			
		}
		else
		{
			if(real_friction_feedforword>25.0f)
			{
				real_friction_feedforword=25.0f;
			}
			else if(real_friction_feedforword<-25.0f)
			{
				real_friction_feedforword=-25.0f;
			}
		}
		if(fabsf(motor->INS_angle_err)<0.2f)
		{
			real_friction_feedforword=0.0f;
			if(fabsf(motor->INS_angle_err)<0.12f)
			{
				motor->speed_pid.out=motor->speed_pid.out*0.9f;
			}
		}
        motor->set_current =-motor->speed_pid.out+gravity_feedforward+0.0f;
    }
    
}




void Gimbal_Yaw_Calculate(gimbal_motor_t *base_yaw,gimbal_motor_t *adv_yaw)
{

    /* --- 模式 1: 手动/大轴主控模式 (BASE_YAW_MODE) --- */
    if (gimbal_control.MODE == BASE_YAW_MODE)
    {
        base_yaw->lock_flag = 0; // 解锁 
        if(base_yaw->INS_speed_set<0.5f&&base_yaw->INS_speed_set>-0.5f)
        {
            base_yaw->INS_angle_set=base_yaw->INS_angle;
						base_yaw->INS_speed_set = 0.0f; 
						base_yaw->control_mode=ANGLE;//停止
				}
         if (base_yaw->control_mode == ANGLE) 
        {
            base_yaw->INS_angle_err = LIMIT_TO_SET(base_yaw->INS_angle_set - base_yaw->INS_angle, 180);
            base_yaw->INS_speed_feedforward = 0; 
        }
        adv_yaw->INS_angle_err = LIMIT_TO_SET(adv_yaw->ENC_angle, 180);
        adv_yaw->INS_speed_feedforward=0.0f;
        adv_yaw->control_mode = ANGLE;
       
        Gimbal_BaseYaw_Motor_Operator(base_yaw); // (vector_set 设定的模式和目标) 
        Gimbal_AdvYaw_Motor_Operator(adv_yaw);
        
    }
    /* --- 模式 2: 自瞄/小轴主控模式 (ADVANCED_YAW_MODE) --- */
    else if (gimbal_control.MODE == ADVANCED_YAW_MODE)
    {
		//CHANGE_PID_FIXED(gimbal_motor_t *adv_yaw,gimbal_motor_t *pitch);
         base_yaw->control_mode=ANGLE;//停止
        if(adv_yaw->INS_speed_set<0.5f&&adv_yaw->INS_speed_set>-0.5f&&adv_yaw->control_mode==SPEED)
        {
            adv_yaw->control_mode=ANGLE;//停止
        }
        if (adv_yaw->control_mode == ANGLE) { 
            // 自瞄时：误差 = 设定绝对角度 - 当前绝对角度
            adv_yaw->INS_angle_err = LIMIT_TO_SET(adv_yaw->INS_angle_set - adv_yaw->INS_angle, 180);
        }
        if(adv_yaw->ENC_angle>= ADVANCED_YAW_ANFLE_MAX || adv_yaw->ENC_angle <= ADVANCED_YAW_ANFLE_MIN)
        {
            adv_yaw->lock_flag=1;
            base_yaw->INS_angle_err = -LIMIT_TO_SET(-adv_yaw->ENC_angle, 180);
            base_yaw->control_mode=ANGLE;   
            base_yaw->lock_flag = 0; // 解锁 
        }
        else
        {
            if (base_yaw->lock_flag == 0 && adv_yaw->ENC_angle<ADVANCED_YAW_ANFLE_MAX && adv_yaw->ENC_angle>ADVANCED_YAW_ANFLE_MIN) 
            {
                adv_yaw->lock_flag=0;
                base_yaw->INS_angle_set = base_yaw->INS_angle; 
                base_yaw->control_mode=ANGLE;//停止
                base_yaw->lock_flag = 1; // 上锁
            }
            base_yaw->INS_angle_err =-LIMIT_TO_SET(base_yaw->INS_angle_set - base_yaw->INS_angle, 90);
        }
        base_yaw->last_lock_flag = base_yaw->lock_flag;
        adv_yaw->last_lock_flag = adv_yaw->lock_flag;
        Gimbal_BaseYaw_Motor_Operator(base_yaw); // (vector_set 设定的模式和目标)
        Gimbal_AdvYaw_Motor_Operator(adv_yaw);
    }
    
    /* --- 模式 3: 跟随底盘 (GIMBAL_TO_CHASSIS_MODE) --- */
    else if (gimbal_control.MODE == GIMBAL_TO_CHASSIS_MODE)
    {
        PID_calc(&gimbal_control.gimbal_psi,gimbal_control.angle_error_rad,0);
        DEADBAND(gimbal_control.gimbal_psi.out,90);
        base_yaw->INS_speed_set=-gimbal_control.gimbal_psi.out;
        adv_yaw->INS_angle_err=LIMIT_TO_SET(adv_yaw->ENC_angle,180);
        Gimbal_BaseYaw_Motor_Operator(base_yaw); // (vector_set 设定的模式和目标)
        Gimbal_AdvYaw_Motor_Operator(adv_yaw);
    }
}

/**
 * @brief Pitch 轴控制计算函数
 * @param pitch_motor Pitch 电机结构体指针 (&gimbal_motor[PITCH_6015])
 */
void Gimbal_Pitch_Calculate(gimbal_motor_t *pitch_motor)
{
     if(pitch_motor->INS_speed_set<0.5f&&pitch_motor->INS_speed_set>-0.5f&&pitch_motor->control_mode==SPEED)
        {
           pitch_motor->control_mode=ANGLE;//停止
        }
    if (pitch_motor->control_mode == ANGLE) 
    {
        //if (pitch_motor->INS_angle_set >  PITCH_ANGLE_MAX)  pitch_motor->INS_angle_set =  PITCH_ANGLE_MAX;
        //if (pitch_motor->INS_angle_set <  PITCH_ANGLE_MIN) pitch_motor->INS_angle_set =  PITCH_ANGLE_MIN;
        
        pitch_motor->INS_angle_err = pitch_motor->INS_angle_set - pitch_motor->INS_angle;
        
    }
    else
    {
        pitch_motor->INS_angle_set=pitch_motor->INS_angle;
    }

    /* 2. 执行通用 PID 算子 */
    Gimbal_Pitch_Motor_Operator(pitch_motor);
    
    if (pitch_motor->set_current > 25000) pitch_motor->set_current = 25000;
    if (pitch_motor->set_current < -25000) pitch_motor->set_current = -25000;
}
void Aim_Gimbal_Pid_Init()
{
//    PID_clear(&gimbal_motor[BASE_YAW_5010].speed_pid);
//    PID_clear(&gimbal_motor[BASE_YAW_5010].angle_pid);
    
    PID_clear(&gimbal_motor[ADVANCED_YAW_6020].speed_pid);
    PID_clear(&gimbal_motor[ADVANCED_YAW_6020].angle_pid);
    
    PID_clear(&gimbal_motor[PITCH_6015].speed_pid);
    PID_clear(&gimbal_motor[PITCH_6015].angle_pid);
    
    PID_init(&gimbal_motor[BASE_YAW_5010].speed_pid,PID_POSITION,base_yaw_motor_auto_aim_speed_pid,BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT,BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT);
    PID_init(&gimbal_motor[BASE_YAW_5010].angle_pid,PID_POSITION,base_yaw_motor_auto_aim_angle_pid,BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT,BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT);
			
    PID_init(&gimbal_motor[ADVANCED_YAW_6020].speed_pid,PID_POSITION,advanced_yaw_motor_auto_aim_speed_pid,ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT,ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT);
	PID_init(&gimbal_motor[ADVANCED_YAW_6020].angle_pid,PID_POSITION,advanced_yaw_motor_auto_aim_angle_pid,ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT,ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT);
		
	PID_init(&gimbal_motor[PITCH_6015].speed_pid,PID_POSITION,pitch_motor_auto_aim_speed_pid,PITCH_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT);
	PID_init(&gimbal_motor[PITCH_6015].angle_pid,PID_POSITION,pitch_motor_auto_aim_angle_pid,PITCH_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT);
			
}

void Normal_Gimbal_Pid_Init()
{
    PID_clear(&gimbal_motor[BASE_YAW_5010].speed_pid);
    PID_clear(&gimbal_motor[BASE_YAW_5010].angle_pid);
    
    PID_clear(&gimbal_motor[ADVANCED_YAW_6020].speed_pid);
    PID_clear(&gimbal_motor[ADVANCED_YAW_6020].angle_pid);
    
    PID_clear(&gimbal_motor[PITCH_6015].speed_pid);
    PID_clear(&gimbal_motor[PITCH_6015].angle_pid);
    
    PID_init(&gimbal_motor[BASE_YAW_5010].speed_pid,PID_POSITION,base_yaw_motor_speed_pid,BASE_YAW_MOTOR_SPEED_PID_MAX_OUT,BASE_YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[BASE_YAW_5010].angle_pid,PID_POSITION,base_yaw_motor_angle_pid,BASE_YAW_MOTOR_ANGLE_PID_MAX_OUT,BASE_YAW_MOTOR_ANGLE_PID_MAX_IOUT);
			
	PID_init(&gimbal_motor[ADVANCED_YAW].speed_pid,PID_POSITION,advanced_yaw_motor_speed_pid,ADVANCED_YAW_MOTOR_SPEED_PID_MAX_OUT,ADVANCED_YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[ADVANCED_YAW].angle_pid,PID_POSITION,advanced_yaw_motor_angle_pid,ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_OUT,ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_IOUT);
		
	PID_init(&gimbal_motor[PITCH_6015].speed_pid,PID_POSITION,pitch_motor_speed_pid,PITCH_MOTOR_SPEED_PID_MAX_OUT,PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[PITCH_6015].angle_pid,PID_POSITION,pitch_motor_angle_pid,PITCH_MOTOR_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ANGLE_PID_MAX_IOUT);

}

void Stop_Base_Yaw_Pid_Init()
{
//    PID_clear(&gimbal_motor[BASE_YAW_5010].speed_pid);
//    PID_clear(&gimbal_motor[BASE_YAW_5010].angle_pid);

    PID_init(&gimbal_motor[BASE_YAW_5010].speed_pid,PID_POSITION,base_yaw_motor_speed_pid,BASE_YAW_MOTOR_SPEED_PID_MAX_OUT,BASE_YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_motor[BASE_YAW_5010].angle_pid,PID_POSITION,base_yaw_motor_angle_pid,BASE_YAW_MOTOR_ANGLE_PID_MAX_OUT,BASE_YAW_MOTOR_ANGLE_PID_MAX_IOUT);
}


void Gimbal_solve()
{
	if (aim_control.aim_PID!=aim_control.last_aim_PID)
	{
        if(aim_control.aim_PID==0)
		{
			Normal_Gimbal_Pid_Init();
		}
		if(gimbal_state == GIMBAL_AUTO_AIM)
		{
			Aim_Gimbal_Pid_Init();
		}
	}
	if (aim_control.set_angle_flag!=aim_control.last_set_angle_flag)
	{
		if(aim_control.set_angle_flag==1)
		{
			nuc_aim_pitch=LIMIT_TO_SET(gimbal_motor[PITCH_6015].INS_angle+nuc_control.aimcontrol.pitch_nuc_gimbal,180);
			nuc_aim_yaw=LIMIT_TO_SET(gimbal_motor[BASE_YAW_5010].INS_angle+nuc_control.aimcontrol.yaw_nuc_gimbal,180);
		}
	}
	aim_control.last_aim_PID=aim_control.aim_PID;	
	aim_control.last_set_angle_flag=aim_control.set_angle_flag;
	//base	
	Gimbal_Pitch_Calculate(&gimbal_motor[PITCH_6015]);
	
	Gimbal_Yaw_Calculate(&gimbal_motor[BASE_YAW_5010],&gimbal_motor[ADVANCED_YAW_6020]);
}

static void Gimbal_Send_Current(int16_t yaw_base, int16_t yaw_adv, int16_t pitch)
{
    // 发送 LK 电机 (Base Yaw ID:1, Pitch ID:3)
    CAN_cmd_LK_Motor(&hcan1,1, yaw_base);
    vTaskDelay(1);
    CAN_cmd_LK_Motor(&hcan2,2, pitch);
    vTaskDelay(1);
    // 发送 DJI GM6020 (Advanced Yaw ID:3)
    CAN_CMD_BASE(&hcan2, 
                 0x1FE, 
                 0,     // ID 1
                 0,     // ID 2
                 yaw_adv, // ID 3
                 0);    // ID 4
}

int16_t current=0;
void Gimbal_Task(void const * argument)
{
    /* 1. 硬件/参数初始化 */
    CAN_cmd_LK_init(&hcan2,2);
    Gimbal_Motor_Init();
    
    // 初始状态强制为 DOWN，确保安全
    rc_ctrl.rc.s[1] = RC_SW_DOWN; 
    
    vTaskDelay(100); // 等待传感器稳定

    // 初始化目标角度为当前角度，防止上电猛甩
     Gimbal_Motor_Data_Update(); // 先更一次数据
    gimbal_motor[PITCH_6015].INS_angle_set = gimbal_motor[PITCH_6015].INS_angle;
    gimbal_motor[BASE_YAW_5010].INS_angle_set = gimbal_motor[BASE_YAW_5010].INS_angle;
    gimbal_motor[ADVANCED_YAW_6020].INS_angle_set = gimbal_motor[ADVANCED_YAW_6020].INS_angle;

    while(1)
    {
        /* 2. 数据更新 */
        Gimbal_Motor_Data_Update();
        
        /* 3. 核心解算  */
        Gimbal_solve();

        /* 4. 安全判断与输出执行(冗余设计) */
        if(rc_ctrl.rc.s[1] == RC_SW_MID || rc_ctrl.rc.s[1] == RC_SW_UP)
        {   
            // 正常输出模式
            Gimbal_Send_Current(
                gimbal_motor[BASE_YAW_5010].set_current,
              gimbal_motor[ADVANCED_YAW_6020].set_current,
//		0,
             gimbal_motor[PITCH_6015].set_current
//			current
//			0
            );
        }
        else
        {
            // 放松/无力模式
            Gimbal_Send_Current(0, 0, 0);

            // 清理状态，防止切回正常模式时猛甩
            PID_clear(&gimbal_motor[PITCH_6015].angle_pid);
            PID_clear(&gimbal_motor[PITCH_6015].speed_pid);
            gimbal_motor[PITCH_6015].set_current = 0;
            // 同步目标，防止回弹
            gimbal_motor[PITCH_6015].INS_angle_set = gimbal_motor[PITCH_6015].INS_angle;
            
            PID_clear(&gimbal_motor[BASE_YAW_5010].angle_pid);
            PID_clear(&gimbal_motor[BASE_YAW_5010].speed_pid);
            gimbal_motor[BASE_YAW_5010].set_current = 0;
            gimbal_motor[BASE_YAW_5010].INS_angle_set = gimbal_motor[BASE_YAW_5010].INS_angle;
            
            PID_clear(&gimbal_motor[ADVANCED_YAW_6020].angle_pid);    
            PID_clear(&gimbal_motor[ADVANCED_YAW_6020].speed_pid);
            gimbal_motor[ADVANCED_YAW_6020].set_current = 0;
            gimbal_motor[ADVANCED_YAW_6020].INS_angle_set = gimbal_motor[ADVANCED_YAW_6020].INS_angle;
        }

        /* 5. 统一的系统延时 */
        //Vofa_Send_Data4(gimbal_motor[PITCH_6015].INS_speed  , gimbal_motor[PITCH_6015].speed_pid.set , gimbal_motor[PITCH_6015].INS_angle_err , gimbal_motor[PITCH_6015].INS_angle_set);
        //Vofa_Send_Data4(gimbal_motor[BASE_YAW_5010].speed_pid.fdb, gimbal_motor[BASE_YAW_5010].speed_pid.set,gimbal_motor[BASE_YAW_5010].INS_angle,gimbal_motor[BASE_YAW_5010].INS_angle_set);
//				Vofa_Send_Data4(gimbal_motor[ADVANCED_YAW_6020].INS_speed , gimbal_motor[ADVANCED_YAW_6020].INS_speed_set , gimbal_motor[ADVANCED_YAW_6020].INS_angle , gimbal_motor[ADVANCED_YAW_6020].INS_angle_set);
				Vofa_Send_Data8(gimbal_motor[PITCH_6015].INS_speed  , gimbal_motor[PITCH_6015].speed_pid.set , gimbal_motor[PITCH_6015].INS_angle_err , gimbal_motor[PITCH_6015].INS_angle_set,gimbal_motor[PITCH_6015].set_current,gimbal_motor[PITCH_6015].speed_pid.out,gimbal_motor[PITCH_6015].angle_pid.out,gimbal_motor[PITCH_6015].INS_angle);
//		Vofa_Send_Data8(gimbal_motor[BASE_YAW_5010].INS_speed  , gimbal_motor[BASE_YAW_5010].speed_pid.set , gimbal_motor[BASE_YAW_5010].INS_angle_err , gimbal_motor[BASE_YAW_5010].INS_angle_set,gimbal_motor[BASE_YAW_5010].set_current,gimbal_motor[BASE_YAW_5010].speed_pid.out,gimbal_motor[BASE_YAW_5010].angle_pid.out,gimbal_motor[BASE_YAW_5010].give_current);
//		Vofa_Send_Data8(gimbal_motor[ADVANCED_YAW_6020].INS_speed  , gimbal_motor[ADVANCED_YAW_6020].speed_pid.set , gimbal_motor[ADVANCED_YAW_6020].INS_angle_err , gimbal_motor[ADVANCED_YAW_6020].INS_angle_set,gimbal_motor[ADVANCED_YAW_6020].set_current,gimbal_motor[ADVANCED_YAW_6020].speed_pid.out,gimbal_motor[ADVANCED_YAW_6020].angle_pid.out,gimbal_motor[ADVANCED_YAW_6020].INS_angle);
				vTaskDelay(1);
    }
}