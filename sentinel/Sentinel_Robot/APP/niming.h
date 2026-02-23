#ifndef __NIMING_H__
#define __NIMING_H__

#include "main.h"

extern uint8_t identify_flag; 
#define Get_BYTE0(dwTemp)   (*(char *)(&dwTemp))    
#define Get_BYTE1(dwTemp)   (*((char *)(&dwTemp) + 1))   
#define Get_BYTE2(dwTemp)   (*((char *)(&dwTemp) + 2))   
#define Get_BYTE3(dwTemp)   (*((char *)(&dwTemp) + 3))   
  
void Yaw_Motor_Identification(void);  
void niming_sent_data(float A,float B,float time);


#endif