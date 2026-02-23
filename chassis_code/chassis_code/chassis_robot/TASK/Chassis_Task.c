#include "Chassis_Task.h" 

chassis_control_t chassis_control;

CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8];

extern wheel_motor_t wheel_motor[CH_NUM_WHEEL_MOT];
extern steer_motor_t steer_motor[CH_NUM_STEER_MOT];


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
    if(chassis_control.ch_enable_flag==0)		 Can_Tansmit_Chmotor_off();
	else								         Can_Tansmit_Chmotor_current();//发送给电机
    vTaskDelay(1);
  }
}   