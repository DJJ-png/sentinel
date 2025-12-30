#include "DJIMotoDriver.h"

void UpdateMotoState(DJIMotoStateTD *MotoState,float ratio);
void UpdateMotoAngle(DJIMotoStateTD *MotoState,float ratio);
void SaveMotoCurrent(CAN_HandleTypeDef *hcan, uint32_t RxFifo);

uint8_t rx_data_lk_and_mt[8]; // FIFO接收缓存区

void DJIMoto_State_Init(DJIMotoStateTD *motostate,double k[6])
{
	motostate->speed = 0;
	motostate->encoder_angle = 0;
	motostate->current = 0;
	motostate->temperature = 0;

	motostate->speed_last = 0;
	motostate->angle_last = 0;

	motostate->current_desired = 0;
	motostate->speed_desired = 0;
	motostate->angle_desired = 0;

	motostate->turns = 0;
	motostate->totol_encoder_angle = 0;

	motostate->original_position = 0;
	motostate->first_run = true;
}

// 大疆电机给电流
// 参数：电机5678电流
void SetMoto_Current(int16_t C1, int16_t C2, int16_t C3, int16_t C4)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	Tx_Msg.StdId = 0x200;
	Tx_Msg.IDE = CAN_ID_STD;   // 不使用扩展标识符
	Tx_Msg.RTR = CAN_RTR_DATA; // 消息类型为数据帧，
	Tx_Msg.DLC = 8;			   // 一帧8字节

	TX_Data[0] = C1 >> 8;
	TX_Data[1] = C1;
	TX_Data[2] = C2 >> 8;
	TX_Data[3] = C2;
	TX_Data[4] = C3 >> 8;
	TX_Data[5] = C3;
	TX_Data[6] = C4 >> 8;
	TX_Data[7] = C4;

	HAL_CAN_AddTxMessage(&DJI_MOTO_CAN_HANDEL, &Tx_Msg, TX_Data, &send_mail_box); // 将数据储存进邮箱FIFOx
}

// 接收电机反馈消息
void DJI_SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo,DJIMotoStateTD *motostate)
{
	CAN_RxHeaderTypeDef Rx_Msg;

	HAL_CAN_GetRxMessage(hcan, RxFifo, &Rx_Msg, rx_data_lk_and_mt);

	switch (Rx_Msg.StdId) // can1 motor message decode
	{
	case CAN_Motor1_ID:
	case CAN_Motor2_ID:
	case CAN_Motor3_ID:
	case CAN_Motor4_ID:
	case CAN_Motor5_ID:
	case CAN_Motor6_ID:
	case CAN_Motor7_ID:
	case CAN_Motor8_ID:
	{
		static uint8_t i = 0;
		// get motor id
		i = Rx_Msg.StdId - CAN_Motor1_ID; // 组内编号
		Update_MotoState(&motostate[i],36.0);//2006
		break;
	}
	default:
		break;
	}
}

// 更新电机状态
void Update_MotoState(DJIMotoStateTD *MotoState,float ratio )
{
    int16_t I_filtered_prev;
    int16_t I_raw;
	MotoState->angle_last = MotoState->encoder_angle;
	MotoState->speed_last = MotoState->speed;

	MotoState->encoder_angle = (uint16_t)(rx_data_lk_and_mt[0] << 8 | rx_data_lk_and_mt[1]);
    
    I_filtered_prev=MotoState->speed;
	MotoState->speed = ((int16_t)(rx_data_lk_and_mt[2] << 8 | rx_data_lk_and_mt[3]))/ratio;
    I_filtered_prev=MotoState->current;
	I_raw = ((int16_t)(rx_data_lk_and_mt[4] << 8 | rx_data_lk_and_mt[5]))*20.0/16384.0f*1000;
    MotoState->current= 0.01* I_raw + (1 - 0.01) * I_filtered_prev;
    
	MotoState->temperature = rx_data_lk_and_mt[6];

	if (MotoState->first_run == true) // first_run为true时，记下初始位置
	{
		MotoState->original_position = MotoState->encoder_angle;
		MotoState->turns = 0;
		MotoState->totol_encoder_angle=0;
		MotoState->first_run=false;
	}
	else
		Update_MotoAngle(MotoState,ratio);
}

void Update_MotoAngle(DJIMotoStateTD *MotoState,float ratio)
{
	int16_t d_angle = MotoState->encoder_angle - MotoState->angle_last;
	if (d_angle <= -4096)
		MotoState->turns += 1;
	else if (d_angle >= 4096)
		MotoState->turns -= 1;
	MotoState->totol_encoder_angle = MotoState->turns * 8192 + MotoState->encoder_angle - MotoState->original_position;
	MotoState->totol_float_angle=MotoState->totol_encoder_angle/8192.0*360.0/ratio;
}
