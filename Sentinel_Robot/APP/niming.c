#include "niming.h"
#include "pid.h"
#include "arm_math.h"
#include "Gimbal_Task.h"
#include "bsp_can.h"
#include "Vofa_send.h"

extern UART_HandleTypeDef huart6;
extern motor_measure_t motor_measure_digal;


uint8_t identify_flag=1; 
uint8_t BUFF_niming[30]; 
void niming_sent_data(float A,float B,float time)//匿名地面端打包函数
{ 
 int i; 
 uint8_t sumcheck = 0; 
 uint8_t addcheck = 0; 
 uint8_t _cnt=0; 
 BUFF_niming[_cnt++]=0xAA; 
 BUFF_niming[_cnt++]=0xFF; 
 BUFF_niming[_cnt++]=0XF1; 
 BUFF_niming[_cnt++]=12;
 BUFF_niming[_cnt++]=Get_BYTE0(A); 
 BUFF_niming[_cnt++]=Get_BYTE1(A); 
 BUFF_niming[_cnt++]=Get_BYTE2(A); 
 BUFF_niming[_cnt++]=Get_BYTE3(A);  
 BUFF_niming[_cnt++]=Get_BYTE0(B); 
 BUFF_niming[_cnt++]=Get_BYTE1(B); 
 BUFF_niming[_cnt++]=Get_BYTE2(B); 
 BUFF_niming[_cnt++]=Get_BYTE3(B); 
 BUFF_niming[_cnt++]=Get_BYTE0(time); 
 BUFF_niming[_cnt++]=Get_BYTE1(time); 
 BUFF_niming[_cnt++]=Get_BYTE2(time); 
 BUFF_niming[_cnt++]=Get_BYTE3(time); 
 for(i=0;i<BUFF_niming[3]+4;i++)  
 { 
  sumcheck+=BUFF_niming[i]; 
  addcheck+=sumcheck; 
 } 
 BUFF_niming[_cnt++]=sumcheck;  
 BUFF_niming[_cnt++]=addcheck;  
  
 HAL_UART_Transmit_DMA(&huart6,BUFF_niming,_cnt); 
} 
 
void Yaw_Motor_Identification()//系统辨识函数
{ 
 static float sin_time = 0;  
 static float last_sin_time = 0; 
 static uint8_t i_sin = 0; 
 static double phtic = 0; 
 static float f[64] = {1.000000, 1.499250, 2.000000, 2.500000, 3.003003, 3.496503, 4.000000, 4.504505,  
  5.000000, 5.494505, 5.988024, 6.493506, 6.993007, 7.518797, 8.000000, 8.474576, 9.009009, 9.523810,  
  10.000000, 10.526316, 10.989011, 11.494253, 12.048193, 12.500000, 12.987013, 13.513514, 14.084507,  
  14.492754, 14.925373, 15.384615, 15.873016, 16.393443, 16.949153, 17.543860, 17.857143, 18.518519,  
  18.867925, 19.607843, 20.000000, 20.408163, 20.833333, 21.276596, 22.222222, 23.809524, 26.315789,  
   27.777778, 30.303030, 32.258065, 34.482759, 35.714286, 38.461538, 40.000000, 50.0000, 58.8235, 71.4286,  
  76.9231, 90.9091, 100.0000, 111.1111, 125.0000, 200.0000, 250.0000, 333.3333, 500.0000}; 
 
 if(identify_flag) 
 { 
  phtic += f[i_sin] * 0.002; 
  float speed_set = 500 * sin(phtic);  
  sin_time += 0.002; 
//	Gimbal_Motor_Data_Update();
	//Gimbal_Motor_Control(&gimbal_motor[BASE_YAW_5010],(int16_t)speed_set);//需更换为被调参机器的云台电机控制代码   
  //niming_sent_data(speed_set,gimbal_motor[BASE_YAW_5010].INS_speed,sin_time); 
	CAN_cmd_LK_Motor(1,(int16_t)speed_set);
	Vofa_Send_Data4(speed_set,gimbal_motor[BASE_YAW_5010].INS_speed,sin_time,0);
	 
  if(last_sin_time + 3.1415926*4/(float)f[i_sin] <= sin_time) 
  { 
   i_sin++; 
   last_sin_time = sin_time; 
  } 
  if(i_sin > 63) 
  { 
   identify_flag = 0; 
  } 
 } 
 
}
