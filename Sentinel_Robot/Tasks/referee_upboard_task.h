#ifndef __REFEREETASK_H__
#define __REFEREETASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "can.h"
#include "struct_typedef.h"
extern void CAN_CMD_BASE_Referee_32bit(CAN_HandleTypeDef* hcan, uint32_t id, int16_t referee_1, int16_t referee_2, uint32_t referee_3);
extern void CAN_CMD_BASE_Referee_8bit(CAN_HandleTypeDef* hcan,uint32_t id, uint16_t position, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6);
extern void CAN_CMD_BASE_Referee_float(CAN_HandleTypeDef* hcan,uint32_t id, uint16_t position, uint8_t data1, uint8_t data2, fp32 data3);

#endif /* __REFEREETASK_H__ */
