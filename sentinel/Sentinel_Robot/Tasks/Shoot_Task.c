#include "Shoot_Task.h"
#include "referee.h"
//#include "referee_usart_task.h"
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
#define SHOOT_MOTOR_SPEED_PID_KP 5.75//3.75//3.0f
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

//电机控制变量
Dial_Motor_t dial_motor;
Shoot_Motor_t shoot_motor[2];
#define FRIC_MOTOR_1 	 shoot_motor[0]
#define FRIC_MOTOR_2 	 shoot_motor[1]
#define DIAL_MOTOR 		dial_motor

/*热量保护*/
#define BARREL_HEAT_LIMIT   400.0f
#define BARREL_HEAT_COOLING 130.0f//80.0f
#define BULLET_17MM_HEAT    10.0f

#define CONTRONL_HZ   1000.0f

#define COOLING   (BARREL_HEAT_COOLING/CONTRONL_HZ)
#define ADD       (BULLET_17MM_HEAT/CONTRONL_HZ)
#define SHOOT_HZ(X)  ((fp32)(X))*(20.0f/((fp32)DIAL_SPEED_BASE))
#define FAST 1.5;
uint16_t  max_cnt = 500;

//功能控制变量
barrel_control_t barrel_control;
shoot_control_t shoot_control;
RC_ctrl_t last_rc_control;
extern Dial_State_e Dial_State;
extern aim_control_t aim_control;

uint8_t shoot_test_freg=10;
//热量保护

void CAN_cmd_AMMO(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{//电机发送
	 CAN_CMD_BASE(&hcan2,0x200 ,  0,  0,  motor3,  motor4);
	 CAN_CMD_BASE(&hcan1,0x200 ,  motor1,  0,  0,  0);
}


/*--初始化函数--*/
void Dial_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {SHOOT_MOTOR_ANGLE_PID_KP, SHOOT_MOTOR_ANGLE_PID_KI, SHOOT_MOTOR_ANGLE_PID_KD};
	memset((uint8_t*)&DIAL_MOTOR,0,sizeof(Dial_Motor_t));
	PID_init(&DIAL_MOTOR.speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&DIAL_MOTOR.angle_pid,PID_POSITION,shoot_motor_angle_pid,SHOOT_MOTOR_ANGLE_PID_MAX_OUT,SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);
}

void Shoot_Motor_Init()
{
	uint8_t i;
	PID_clear(&shoot_motor[0].pid_speed);
	PID_clear(&shoot_motor[1].pid_speed);


	for(i=0;i<2;i++){
		fp32 motor_speed_pid[3]={8.0,0.050000007,0.0f};
			PID_init(&shoot_motor[i].pid_speed,PID_POSITION,motor_speed_pid,12000,1200);
        weighted_filter_init(&shoot_motor[i].fric_filter,0.2,5);
	}
    
}
/*--数据更新--*/
void Shoot_Motor_Data_Update(void)
{
	/**/
	DIAL_MOTOR.speed					=	motor_measure_shoot[0].speed_rpm;
	DIAL_MOTOR.give_current		=	motor_measure_shoot[0].given_current;
	DIAL_MOTOR.angle					=	dial_angle;
	/**/
    ewma_filter_update(&FRIC_MOTOR_1.fric_filter,motor_measure_shoot[2].speed_rpm);
	/**/
	ewma_filter_update(&FRIC_MOTOR_2.fric_filter,motor_measure_shoot[3].speed_rpm);
}


/**
 * @brief 拨弹盘电机 PID 计算算子
 * @param motor 电机结构体指针
 * @note  这个函数不处理业务逻辑，只负责数学计算
 */
static void Dial_Motor_Operator(Dial_Motor_t *motor) 
{
    if (motor->mode == SPEED) 
    {
        // 单速度环
        PID_calc(&motor->speed_pid, motor->speed, motor->speed_set);
        motor->set_current = motor->speed_pid.out;
    } 
    else if (motor->mode == ANGLE) 
    {
        //角度环计算
        PID_calc(&motor->angle_pid, -LIMIT_TO_SET(motor->angle - motor->angle_set,4096), 0);
        fp32 target_speed =  motor->angle_pid.out;
        
        // 速度环计算
        PID_calc(&motor->speed_pid, motor->speed, target_speed);
        motor->set_current = motor->speed_pid.out;
    }
    
}
void Dial_Close_Control()
{
    DIAL_MOTOR.speed_set=0;
    DIAL_MOTOR.mode=SPEED;
    Dial_Motor_Operator(&DIAL_MOTOR);
    DIAL_MOTOR.set_current=0;

}
void Shoot_Test_Control()
{
    DIAL_MOTOR.speed_set=shoot_test_freg*36*60/8;
    DIAL_MOTOR.mode=SPEED;//电机设置为speed模式
    Dial_Motor_Operator(&DIAL_MOTOR);
     
}
void Rapid_Fire_Control()
{
    if(nuc_receive_data.aim_data_received.is_fire!=0 && nuc_receive_data.aim_data_received.is_fire == 1)
    {
        if(Power_Heat_Data.shooter_17mm_1_barrel_heat < Robot_Status.shooter_barrel_heat_limit*0.75f)
            DIAL_MOTOR.speed_set= nuc_receive_data.aim_data_received.shoot_freq*36*60/8;//DIAL_SPEED_BASE
    }
    else
    {
        DIAL_MOTOR.speed_set=0;
    }
     DIAL_MOTOR.mode=SPEED;//电机设置为speed模式
     Dial_Motor_Operator(&DIAL_MOTOR);
}

void Single_Fire_Control()
{
     if(nuc_receive_data.aim_data_received.is_fire!=0 && nuc_receive_data.aim_data_received.is_fire == 1)
    {
        if(Power_Heat_Data.shooter_17mm_1_barrel_heat < Robot_Status.shooter_barrel_heat_limit*0.85f)
            DIAL_MOTOR.angle_set= DIAL_MOTOR.angle-8192*45/360;
             DIAL_MOTOR.mode=ANGLE;//电机设置为angle模式
             Dial_Motor_Operator(&DIAL_MOTOR);
    }

}
void Dial_FSM(void)
{
    switch(Dial_State)
    {
        case CLOSE:          {Dial_Close_Control();break;}
        case SINGLE_FIRE:    {Single_Fire_Control();break;}
        case RAPID_FIRE:     {Rapid_Fire_Control();break;}
        case SHOOT_TEST:     {Shoot_Test_Control();break;}
    
    }
        

}

fp32 SHOOT_PID_calc(pid_type_def *pid, Shoot_Motor_t* fric_ctrl,fp32 id)
{

	
    if (pid == NULL)
    {
        return 0.0f;
    }
	
		fric_ctrl->fric_sp.reference[2] = fric_ctrl->fric_sp.reference[1];
    fric_ctrl->fric_sp.reference[1] = fric_ctrl->fric_sp.reference[0];
		fric_ctrl->fric_sp.reference[0] = fric_ctrl->fric_filter.ewma_value;
		pid->fdb = (fric_ctrl->fric_sp.reference[0] + fric_ctrl->fric_sp.reference[1] + fric_ctrl->fric_sp.reference[2]) / 3.0f;
	
		if(pid->last_set != fric_ctrl->rpm_set)
		{
			fric_ctrl->fric_sp.start_flag =1;
			pid->last_set = fric_ctrl->rpm_set;
		}
		if(fric_ctrl->fric_sp.start_flag)
		{
			if(pid->cnt > max_cnt){
				pid->cnt = 0;
				fric_ctrl->fric_sp.start_flag = 0;
			}
			else
				pid->cnt++;
				pid->fdb = fric_ctrl->fric_filter.buffer[fric_ctrl->fric_filter.index-1] ;
		}
		else{
		if(fabs(fric_ctrl->rpm_set - fric_ctrl->fric_filter.buffer[fric_ctrl->fric_filter.index-1] )> 65 && pid->cnt == 0&& pid->last_set!=0){
			fric_ctrl->fric_sp.k= 0;
			pid->cnt ++;
		}
		else if(fric_ctrl->fric_sp.k != 1 && pid->cnt < max_cnt/50 && id > 2 )
			pid->cnt ++;

		else if(fric_ctrl->fric_sp.k != 1 && pid->cnt < max_cnt/25)
		{
			pid->cnt ++;			fric_ctrl->fric_sp.k= FAST;
			
		}
		else 
			{
			fric_ctrl->fric_sp.k= 1;
			pid->cnt = 0;
		}
		}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = fric_ctrl->rpm_set;
		
//    pid->fdb = ref;
//    pid->error[0] = set - ref;

    pid->error[0] = fric_ctrl->rpm_set - pid->fdb;
		
    if (pid->mode == PID_POSITION)
    {
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
//				if(fabs(pid->Dbuf[0]) >= 800)
//					pid->error[0] = pid->error[1];
				pid->Iout += pid->Ki * pid->error[0]*(fric_ctrl->fric_sp.start_flag == 1?1:0);;//		
			
        pid->Pout = pid->Kp *fric_ctrl->fric_sp.k* pid->error[0];

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        	
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
				pid->out = fric_ctrl->fric_sp.k == 0?0:(pid->Pout + pid->Iout + pid->Dout);
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


void shoot_Control()
{
	if(barrel_control.barrel_wait_flag==0)
	{
		if(Power_Heat_Data.shooter_17mm_1_barrel_heat >= Robot_Status.shooter_barrel_heat_limit*0.75f)
		{
			shoot_control.dial_speed=0;
			DIAL_MOTOR.set_current=0;
			DIAL_MOTOR.speed_pid.Iout=0;
			barrel_control.barrel_wait_flag=1;
		}
	}
	if(barrel_control.barrel_wait_flag==1)
	{
				shoot_control.dial_speed=0; 
				DIAL_MOTOR.set_current=0; 
				DIAL_MOTOR.speed_pid.Iout=0;

				barrel_control.barrel_wait_cnt++;

		if(barrel_control.barrel_wait_cnt>=575 )
		{
			barrel_control.barrel_wait_cnt=0;
			barrel_control.barrel_wait_flag=0;
		} 
	}
    if(shoot_control.fric_state==1)
    {
        FRIC_MOTOR_1.rpm_set=-5450;
        FRIC_MOTOR_2.rpm_set=5450;
         SHOOT_PID_calc(&FRIC_MOTOR_1.pid_speed,&FRIC_MOTOR_1,3);
         SHOOT_PID_calc(&FRIC_MOTOR_2.pid_speed,&FRIC_MOTOR_2,4);
        CAN_CMD_BASE(&hcan2,0x200 ,  0,  0,  FRIC_MOTOR_1.pid_speed.out,  FRIC_MOTOR_2.pid_speed.out);
    }
}

void heat_cooling()
{
	
	barrel_control.barrel_heat[0]+=(fp32)ADD*SHOOT_HZ(DIAL_MOTOR.speed);		
	barrel_control.barrel_heat[0]-=(fp32)COOLING;
	if(barrel_control.barrel_heat[0]<=0.0f)
		barrel_control.barrel_heat[0]=0.0f;
	Power_Heat_Data.shooter_17mm_1_barrel_heat=barrel_control.barrel_heat[0];
	Robot_Status.shooter_barrel_heat_limit=1000.0f;

}

void Shoot_Task(void const * argument)
{	 
    Dial_Motor_Init();
	Shoot_Motor_Init();
	vTaskDelay(200); 
	while(1)
	{
		Shoot_Motor_Data_Update();
        Dial_FSM();
        heat_cooling();
        shoot_Control();
        CAN_CMD_BASE(&hcan1,0x200 ,DIAL_MOTOR.set_current,  0,  0,  0);
		vTaskDelay(1);
	}
}
/*--function--end*/