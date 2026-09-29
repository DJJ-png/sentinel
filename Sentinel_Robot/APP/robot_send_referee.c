#include "robot_send_referee.h"
#include "Nmanifold_usart_task.h"
#include "usart.h"
#include "referee.h"

Sentry_Decision_0x0120_t Sentry_Decision_0x0120;

uint8_t Usart6_Dma_Txbuf[64];

void Sentry_Referee_data_update()		//发给裁判系统或者其他机器人的数据更新
{
	/*哨兵自主决策*/
	Sentry_Decision_0x0120.data_cmd_id = 0x0120;	Sentry_Decision_0x0120.receiver_id = 0x8080;		
	Sentry_Decision_0x0120.sender_id = Robot_Status.robot_id;				
}

/*串口发送函数*/

void Sentry_Send_Referee_0x0301(){
	referee_copy(0x0301,10,(uint8_t *)&Sentry_Decision_0x0120,Usart6_Dma_Txbuf);
	HAL_UART_Transmit_DMA(&huart6, Usart6_Dma_Txbuf, 5+2+10+2);
	vTaskDelay(2);
}

void Sentry_Send_Referee_0x0308(){
	referee_copy(0x0308,34,(uint8_t *)&custom_info_0x0308,Usart6_Dma_Txbuf);
	HAL_UART_Transmit_DMA(&huart6, Usart6_Dma_Txbuf, 5+2+34+2);
	vTaskDelay(2);
}
