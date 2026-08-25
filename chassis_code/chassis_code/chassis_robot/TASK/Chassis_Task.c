#include "Chassis_Task.h" 

chassis_control_t chassis_control;

CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8];

extern wheel_motor_t wheel_motor[CH_NUM_WHEEL_MOT];
extern steer_motor_t steer_motor[CH_NUM_STEER_MOT];
void chassis_power_control()
{
	fp32 Wheel_current_limit=0;
	fp32 Helm_current=0,Wheel_current=0;
	
	fp32 power_scale=1.0f;
	fp32 power_buffer_scale=1.0f;
	chassis_control.chassis_power_buffer =Power_Heat_Data.buffer_energy;
	chassis_control.chassis_power_limit =Robot_Status.chassis_power_limit;
    
	if(Robot_Status.chassis_power_limit < 50 && Game_Status.game_progress == 4)
		{
			chassis_control.chassis_power_limit = 33;//100
			chassis_control.power_limit_deceive = 33;//100
		}else
		{
			chassis_control.chassis_power_limit = 98;//100
			chassis_control.power_limit_deceive = 98;//100
		}

	/* 如果功率限制不为0，那么根据功率限制百分比来调整轮子电流的限制 */
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
		
	if(chassis_control.chassis_power_limit==0)		Wheel_current_limit=NO_JUDGE_TOTAL_CURRENT_LIMIT;
	else											Wheel_current_limit=POWER_TOTAL_CURRENT_LIMIT*chassis_control.power_limit_deceive/100;
	
	if(chassis_control.chassis_power_buffer < WARNING_POWER_BUFF)
	{
			if(chassis_control.chassis_power_buffer > 20.0f)		power_buffer_scale = (chassis_control.chassis_power_buffer)/ (WARNING_POWER_BUFF);
			else																								power_buffer_scale = 0.0f / WARNING_POWER_BUFF;
	}
	
	for(uint8_t i=0;i<4;i++)		Helm_current+=0.15f*fabs(wheel_motor[i].wheel_speed_pid.out);
	if(Helm_current>POWER_HELM_CURRENT_LIMIT){
		for(uint8_t i=0;i<4;i++)			wheel_motor[i].wheel_speed_pid.out*=POWER_HELM_CURRENT_LIMIT/Helm_current;
		Helm_current=POWER_HELM_CURRENT_LIMIT;
	}
	//for(uint8_t i=0;i<4;i++)			wheel_motor[i].wheel_speed_pid.out*=power_buffer_scale*power_scale;

	/*3508*/
	Wheel_current_limit-=Helm_current;	
	for(uint8_t i=0;i<4;i++)				Wheel_current+=fabs(wheel_motor[i].wheel_speed_pid.out);
	if(Wheel_current>Wheel_current_limit)
		for(uint8_t i=0;i<4;i++)			wheel_motor[i].wheel_speed_pid.out*=Wheel_current_limit/Wheel_current;
		for(uint8_t i=0;i<4;i++)			wheel_motor[i].wheel_speed_pid.out*=power_buffer_scale*power_scale;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data);
	if(hcan==&CAN_CHMOTOR_MESSAGE)	Can_Getmeasure_Chmotor(can_rx_header,can_rx_data);
	if(hcan==&CAN_UPTRANS_MESSAGE)	Can_Getmeasure_Upboard(can_rx_header.StdId,can_rx_data);
}
void Chassis_Task(void const * argument) 
{
    Pid_Init_CHmotor();//pid初始化
  while(1) 
  {		
    CH_Solve_motor();  //底盘解算
    Ch_Calc_Motorpid();//pid输出
    //CAN_Send_Cap(chassis_control.power_limit_deceive,Power_Heat_Data.buffer_energy);
   //chassis_power_control();
    if(chassis_control.ch_enable_flag==0)		 Can_Tansmit_Chmotor_off();
	else								         Can_Tansmit_Chmotor_current();//发送给电机
    
    vTaskDelay(1);
  }
}   