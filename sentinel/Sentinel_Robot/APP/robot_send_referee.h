#ifndef __ROBOT_SEND_REFEREE_H
#define __ROBOT_SEND_REFEREE_H

#include "struct_typedef.h"



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

extern Sentry_Decision_0x0120_t Sentry_Decision_0x0120;



#endif