#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "struct_typedef.h"
/* *********************************************************************** */

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204

#define CAN_helm_M3_ID 0x205
#define CAN_helm_M4_ID 0x208

#define SEND_NX_ID_1 0x401
#define SEND_NX_ID_2 0x402
#define RECV_NX_ID_1 0x411
#define RECV_NX_ID_2 0x412

#define CAN_LK_BASE_YAW_ID 0x141
#define CAN_ADVANCED_YAW_ID 0x207
#define CAN_LK_PITCH_ID 0x143

#define CAN_COMMUNITE_ID_1 0x301
#define CAN_COMMUNITE_ID_2 0x302
#define CAN_COMMUNITE    hcan1

/*上板向下板传输数据*/
#define UP_TO_DOWN_ID1 0x303
#define UP_TO_DOWN_ID2 0x304

//视觉需要的数据ID
#define MY_POSITION1_ID 0x30D
#define MY_POSITION2_ID 0x30E
#define ENEMT_STATE_ID 0x30F

#define DM_IMU_ID 0x011

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* *********************************************************************** */

typedef struct
{
	int16_t last_ecd;
    uint16_t ecd;
    fp32 speed_rpm;
    int16_t given_current;
    uint8_t temperate;
}motor_measure_t;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = 	(int16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#define motor_measure_LK(ptr, data)                                 						\
    {                                                                   				\
        (ptr)->last_ecd = 			(ptr)->ecd;                             				\
        (ptr)->ecd = 						(uint16_t)((data)[7] << 8 | (data)[6]); 				\
        (ptr)->speed_rpm = 			((int16_t)((data)[5] << 8 | (data)[4]))/60.0f;	\
        (ptr)->given_current = 	(uint16_t)((data)[3] << 8 | (data)[2]); 				\
        (ptr)->temperate = 			(data)[1];                              				\
    }		
		
/* *********************************************************************** */
		
extern void can_filter_init(void);
extern void CAN_CMD_BASE(CAN_HandleTypeDef* hcan,uint32_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_LK_Motor(uint8_t uid, int16_t current);
void Can1_Manage(uint32_t Get_Id);
void Can2_Manage(uint32_t Get_Id);
extern void CAN_CMD_f32_f8(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,uint8_t data2,uint8_t data3,uint8_t data4,uint8_t data5);
extern void CAN_CMD_f32(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,fp32 data2);		
void CAN_CMD_f16_3(CAN_HandleTypeDef* hcan,uint32_t id, uint8_t position, uint16_t data1, uint16_t data2, uint16_t data3);
void CAN_CMD_f8_7(CAN_HandleTypeDef* hcan,uint32_t id, uint8_t position, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);
/* *********************************************************************** */

extern motor_measure_t motor_measure_gimbal[3];
extern motor_measure_t motor_measure_shoot[4];// 0:dial

extern int32_t barrel_angle;
extern int32_t dial_angle;
extern uint16_t receive_upboard_cnt;
extern uint8_t receive_0x0308_begin_flag;
extern uint8_t receive_0x0301_begin_flag;


		
/****   NX通信需要   ****/

extern uint8_t ins_cnt;
extern uint8_t ins_buff[14];

/***********************/
		
#endif
