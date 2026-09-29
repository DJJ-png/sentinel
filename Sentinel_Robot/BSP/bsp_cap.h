#ifndef BSP_CAP_H
#define BSP_CAP_H

#include "can.h"
#define CAP_HCAN hcan1  

typedef struct
{
	/*send*/
	float max_power;
	float actual_power;
	float buffer_power;
	/*resove*/
	float cap_per;
	float chassis_power;
	
uint8_t cap_recieve_flag;

}cap_measure_t;

extern cap_measure_t cap_data;
extern  void CAN_Send_Cap(uint16_t cap_limt ,uint16_t power ,uint16_t power_buffer);
extern void update_cap(uint8_t * data);

#endif
