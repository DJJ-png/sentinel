#include "bsp_cap.h"
#include "main.h"


cap_measure_t cap_data;
void update_cap(uint8_t * data)
{
	cap_data.cap_per=(float)((data)[1]<<8|(data)[0])/32768.0f;
	cap_data.chassis_power=(float)((data)[3]<<8|(data)[2])/100.0f;
	cap_data.cap_recieve_flag=1;
}


 void CAN_Send_Cap(uint16_t cap_limt ,uint16_t power_buffer)
{
		uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  cap_tx_message = {0};
	cap_tx_message.StdId = 0x140;
	cap_tx_message.IDE = CAN_ID_STD;
	cap_tx_message.RTR = CAN_RTR_DATA;
	cap_tx_message.DLC = 0x08;
	
	cap_limt*=100;
	power_buffer*=100;

	uint8_t cap_send[8];
	cap_send[0]=(uint8_t)(cap_limt>>0);	
	cap_send[1]=(uint8_t)(cap_limt>>8);
	cap_send[2]=0;
	cap_send[3]=0;
	cap_send[4]=(uint8_t)(power_buffer>>0);
	cap_send[5]=(uint8_t)(power_buffer>>8);
	
	HAL_CAN_AddTxMessage(&CAP_HCAN, &cap_tx_message, cap_send, &send_mail_box);

}

