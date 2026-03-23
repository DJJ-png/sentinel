#include "bsp_can.h"
#include "Nmanifold_usart_task.h"
extern uint8_t Custom_info_CAN_send_flag;
custom_info_t custom_info_0x0308;
extern  Sentry_Decision_0x0120_t Sentry_Decision_0x0120;
//发给下板的数据
void Custom_info0x0308_CAN_send()
{
	CAN_CMD_f8_7(&hcan1,UP_TO_DOWN_ID2,1,custom_info_0x0308.sender_id>>8,custom_info_0x0308.sender_id,custom_info_0x0308.receiver_id>>8,custom_info_0x0308.receiver_id,custom_info_0x0308.user_data[0],custom_info_0x0308.user_data[1],custom_info_0x0308.user_data[2]);
	vTaskDelay(2);
	CAN_CMD_f8_7(&hcan1,UP_TO_DOWN_ID2,2,	custom_info_0x0308.user_data[3],
																custom_info_0x0308.user_data[4],
																custom_info_0x0308.user_data[5],
																custom_info_0x0308.user_data[6],
																custom_info_0x0308.user_data[7],
																custom_info_0x0308.user_data[8],
																custom_info_0x0308.user_data[9]);
	vTaskDelay(2);
	CAN_CMD_f8_7(&hcan1,UP_TO_DOWN_ID2,3,	custom_info_0x0308.user_data[10],
																custom_info_0x0308.user_data[11],
																custom_info_0x0308.user_data[12],
																custom_info_0x0308.user_data[13],
																custom_info_0x0308.user_data[14],
																custom_info_0x0308.user_data[15],
																custom_info_0x0308.user_data[16]);
	vTaskDelay(2);
	CAN_CMD_f8_7(&hcan1,UP_TO_DOWN_ID2,4,	custom_info_0x0308.user_data[17],
																custom_info_0x0308.user_data[18],
																custom_info_0x0308.user_data[19],
																custom_info_0x0308.user_data[20],
																custom_info_0x0308.user_data[21],
																custom_info_0x0308.user_data[22],
																custom_info_0x0308.user_data[23]);
	vTaskDelay(2);
	CAN_CMD_f8_7(&hcan1,UP_TO_DOWN_ID2,5,	custom_info_0x0308.user_data[24],
																custom_info_0x0308.user_data[25],
																custom_info_0x0308.user_data[26],
																custom_info_0x0308.user_data[27],
																custom_info_0x0308.user_data[28],
																custom_info_0x0308.user_data[29],
																0);
	vTaskDelay(2);
}


void uptodown_task(void const * argument)
{
    while(1)
    {
        if (Custom_info_CAN_send_flag==1)
        {
             Custom_info0x0308_CAN_send();
            Custom_info_CAN_send_flag = 0;
        }
        if(Custom_info_CAN_send_flag==2)
        {
            CAN_CMD_BASE_Referee_32bit(&hcan1,UP_TO_DOWN_ID1,0,0,Sentry_Decision_0x0120.decision);
            Custom_info_CAN_send_flag = 0;
        }
        vTaskDelay(10);
        
    }
    
}