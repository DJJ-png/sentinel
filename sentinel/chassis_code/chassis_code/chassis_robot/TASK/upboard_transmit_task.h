#ifndef __UPTANSMIT_H__
#define __UPTANSMIT_H__

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "can.h"
#include "bsp_cap.h"
#include "config_set.h"
#include "Referee.h"
#include "robot_send_referee.h"
#include "struct_typedef.h"
#include "Chassis_Task.h"

#define REFEREE_USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH     1024
//对方车辆数据
typedef struct
{
	uint8_t id;
	uint16_t x;
	uint16_t y;
	uint16_t hp;
	uint8_t attack_enhance;
}enemy_state_data_t;

extern void Can_Getmeasure_Upboard(uint32_t Get_Id,uint8_t rx_data[8]);
extern void update_cap(uint8_t * data);
#endif /* __REFEREETASK_H__ */
