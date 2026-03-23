#include "robot_send_referee.h"
#include "usart.h"
#include "referee.h"

Sentry_Decision_0x0120_t Sentry_Decision_0x0120 ;
custom_info_t custom_info_0x0308;

uint8_t Usart6_Dma_Txbuf[64];

Referee_Protocol_Head_Data 		Referee_Protocol_Head = {0};

/*数据转载成裁判系统格式*/
void referee_copy(uint16_t cmid,uint16_t length,uint8_t * data,uint8_t* uart_send)
{
	/*head_copy*///这里看裁判系统串口协议
	Referee_Protocol_Head.SOF=0xA5;
	Referee_Protocol_Head.seq+=1;
	Referee_Protocol_Head.data_length = length ;
	Referee_Protocol_Head.CRC8=CRC08_Calculate((uint8_t*)&Referee_Protocol_Head,4);
	memcpy(uart_send,(uint8_t *)(&Referee_Protocol_Head), 5);
	/*cmid_copy*/
	uart_send[5]=cmid&0x0f;
	uart_send[6]=(cmid>>8)&0x0f;
	/*data_copy*/
	memcpy(&uart_send[7], (uint8_t *)(data), length);
	/*crc_16*/
	*((uint16_t*)(uart_send+5+2+length))=CRC16_Calculate(uart_send, 5+2+length);
}


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
