#ifndef __ROBOT_SEND_REFEREE_H
#define __ROBOT_SEND_REFEREE_H

#include "struct_typedef.h"
#include "cmsis_os.h"
#include "CRCs.h"

typedef struct __attribute__((packed)) 
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;


typedef struct{
	uint8_t  SOF;			//帧头
	uint16_t data_length;			//帧长（包含帧头与校验位）
	uint8_t  seq;			//命令字
	uint8_t  CRC8;
}__attribute__((__packed__)) Referee_Protocol_Head_Data;//裁判系统通信协议数据流前段数据

typedef struct 					__attribute__((packed))
{
    uint16_t data_cmd_id;     // 数据/命令标识
    uint16_t sender_id;       // 发送方ID
    uint16_t receiver_id;     // 接收方ID
    uint32_t decision;      // nuc发来的决策数据
} Sentry_Decision_0x0120_t;			//哨兵自主决策

extern void Sentry_Referee_data_update();
extern void Sentry_Send_Referee_0x0301();
extern void Sentry_Send_Referee_0x0308();
extern void referee_copy(uint16_t cmid,uint16_t length,uint8_t * data,uint8_t* uart_send);

extern Sentry_Decision_0x0120_t Sentry_Decision_0x0120;
extern custom_info_t custom_info_0x0308;


#endif