#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "can.h"
#include "struct_typedef.h"
#include "arm_math.h"

extern void can_filter_init(void);
extern void CAN_CMD_BASE(CAN_HandleTypeDef* hcan,uint32_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_CMD_f32(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,fp32 data2);


#endif
