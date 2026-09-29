#include "Nmanifold_usart_task.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "Shoot_Task.h"
#include "task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "Gimbal_Task.h"
#include "Vofa_send.h"
#include "CRCs.h"
#include "robot_message.h"
#include "bsp_can.h"
#include "referee_upboard_task.h"
#include "config_set.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
// 串口接收的双缓冲（Ping-Pong Buffer），用于DMA接收，防止数据覆盖
uint8_t Usart1_Buf[2][USART_RX_BUF_LENGHT];
// 用于处理解析的接收缓冲区
uint8_t NUC_USART_RxBuf[USART_RX_BUF_LENGHT];
// DMA发送缓冲区
uint8_t Usart1_Dma_Txbuf[64];
// 发送频率控制计数器
uint16_t Frequency_Control_cnt = 1;

Protocol_Head_Data 		Protocol_Head = {0};

nuc_control_t 			nuc_control;

uint8_t 	data_unpack(uint8_t* data_in,uint8_t** data_out,uint16_t* lenth,uint16_t* cmid);
uint16_t 	lenth,cmid;
uint8_t  	data_send[64];

uint8_t 	nuc_work[5];			//0 1
uint16_t 	nuc_off_wait[5];  //5000

uint8_t Switch_Data;

extern uint16_t attack_detection_cnt;

extern custom_info_t custom_info_0x0308;
Sentry_Decision_0x0120_t Sentry_Decision_0x0120;

NUC_Used_Message_200Hz_0606_t 	NUC_Used_Message_200Hz_0606;	//28+1+4+4+8=45byte
NUC_Used_Message_1Hz_0607_t 	NUC_Used_Message_1Hz_0607;		//3+4+2+12+2+32+6+9+26=96byte
uint8_t Custom_info_CAN_send_flag=0;
ext_hurt_data_t	    Last_Hurt_Data; 
//nuc决策需要的数据
void USART_Send_referee_Init(void)
{
    if(Robot_Status.robot_id==107)//蓝色
    {
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_1_robot_HP = Game_Robot_HP.red_1_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_2_robot_HP = Game_Robot_HP.red_2_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_3_robot_HP = Game_Robot_HP.red_3_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_4_robot_HP = Game_Robot_HP.red_4_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_7_robot_HP = Game_Robot_HP.red_7_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_base_HP = Game_Robot_HP.red_base_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_outpost_HP = Game_Robot_HP.red_outpost_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_1_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_2_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_3_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_4_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_7_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_base_HP    = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_outpost_HP = 0xffff;
    
    
    }
    else
    {
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_1_robot_HP = Game_Robot_HP.red_1_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_2_robot_HP = Game_Robot_HP.red_2_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_3_robot_HP = Game_Robot_HP.red_3_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_4_robot_HP = Game_Robot_HP.red_4_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_7_robot_HP = Game_Robot_HP.red_7_robot_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_base_HP = Game_Robot_HP.red_base_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.red_outpost_HP = Game_Robot_HP.red_outpost_HP;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_1_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_2_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_3_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_4_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_7_robot_HP = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_base_HP    = 0xffff;
        NUC_Used_Message_200Hz_0606.NUC_Game_Robot_HP_0003.blue_outpost_HP = 0xffff;
    
    
    }
	NUC_Used_Message_1Hz_0607.NUC_Game_Status_0001.game_progress = Game_Status.game_progress;
	NUC_Used_Message_1Hz_0607.NUC_Game_Status_0001.stage_remain_time = Game_Status.stage_remain_time;
	
	
	NUC_Used_Message_200Hz_0606.Hurt_Data.armor_id = Hurt_Data.armor_id;
	NUC_Used_Message_200Hz_0606.Hurt_Data.HP_deduction_reason = Hurt_Data.HP_deduction_reason;
		
	NUC_Used_Message_1Hz_0607.Event_Data_0101 = Event_Data;	
	NUC_Used_Message_1Hz_0607.dart_info_0105.dart_info = Dart_Information.dart_info;	
	
	NUC_Used_Message_1Hz_0607.robot_pos_0203.x = Robot_Pos.x;
	NUC_Used_Message_1Hz_0607.robot_pos_0203.y = Robot_Pos.y;
	NUC_Used_Message_1Hz_0607.robot_pos_0203.angle = Robot_Pos.angle;

	NUC_Used_Message_1Hz_0607.buff_0204.defence_buff = Buff_Energy.defence_buff;	
	NUC_Used_Message_1Hz_0607.buff_0204.remaining_energy = Buff_Energy.remaining_energy;	

	NUC_Used_Message_200Hz_0606.NUC_Projectile_Allowance_0208.projectile_allowance_17mm = Projectile_Allowance.projectile_allowance_17mm;
	NUC_Used_Message_200Hz_0606.NUC_Projectile_Allowance_0208.remaining_gold_coin = Projectile_Allowance.remaining_gold_coin;
	NUC_Used_Message_200Hz_0606.RFID_Status_0209 = RFID_Status;		

	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.hero_x = Ground_Robot_Position.hero_x;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.hero_y = Ground_Robot_Position.hero_y;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.engineer_x = Ground_Robot_Position.engineer_x;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.engineer_y = Ground_Robot_Position.engineer_y;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.standard_3_x = Ground_Robot_Position.standard_3_x;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.standard_3_y = Ground_Robot_Position.standard_3_y;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.standard_4_x = Ground_Robot_Position.standard_4_x;
	NUC_Used_Message_1Hz_0607.ground_robot_position_020B.standard_4_y = Ground_Robot_Position.standard_4_y;

	NUC_Used_Message_1Hz_0607.sentry_info_020D.sentry_info = Sentry_Info.sentry_info;
	NUC_Used_Message_1Hz_0607.sentry_info_020D.sentry_info_2 = Sentry_Info.sentry_info_2;
	
	NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.radar_enhance = (Robot_Interaction_User_Data.radar_enhance & 0x80);
	for(uint8_t i=0;i<4;i++)
	{
		NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.robot_x[i] = enemy_state_data[i].x;
		NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.robot_y[i] = enemy_state_data[i].y;
		NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.robot_z[i] = Robot_Interaction_User_Data.enemy[i].z;
	}
	NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.robot_x[4] = Robot_Interaction_User_Data.enemy[5].x;
	NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.robot_y[4] = Robot_Interaction_User_Data.enemy[5].y;
	NUC_Used_Message_1Hz_0607.Robot_Interaction_User_Data_0301.radar_data.robot_z[4] = Robot_Interaction_User_Data.enemy[5].z;
	
	NUC_Used_Message_1Hz_0607.map_command_0303.cmd_keyboard = Map_Command.cmd_keyboard;
	NUC_Used_Message_1Hz_0607.map_command_0303.target_position_x = Map_Command.target_position_x;
	NUC_Used_Message_1Hz_0607.map_command_0303.target_position_y = Map_Command.target_position_y;

	NUC_Used_Message_200Hz_0606.NUC_Parameter_Of_Car_0601.angle_between_gimbal_and_chassis = gimbal_control.angle_error_rad+0.07*gimbal_motor[BASE_YAW_5010].INS_speed*0.01745f;
	NUC_Used_Message_200Hz_0606.NUC_Parameter_Of_Car_0601.advanced_yaw_angle = gimbal_motor[ADVANCED_YAW_6020].INS_angle + gimbal_motor[ADVANCED_YAW_6020].ENC_angle;
	
	if(NUC_Used_Message_1Hz_0607.NUC_Game_Status_0001.game_progress != 4)
	{ 
		NUC_Used_Message_1Hz_0607.map_command_0303.cmd_keyboard = 0;
		NUC_Used_Message_1Hz_0607.map_command_0303.target_position_x = 0;
		NUC_Used_Message_1Hz_0607.map_command_0303.target_position_y = 0;
	}
}
void manifold_usart_task(void)
	{	
	memset(&Last_Hurt_Data, 0x00, 1);
	memset(NUC_USART_RxBuf, 0x00, USART_RX_BUF_LENGHT);
	usart1_init(Usart1_Buf[0], Usart1_Buf[1], USART_RX_BUF_LENGHT);
	vTaskDelay(200);	
	while(1){		
		USART_Send_referee_Init();
		if(Frequency_Control_cnt == 3000) Frequency_Control_cnt = 0;
		if(Frequency_Control_cnt % 101  == 0){
			/*1HZ发送数据*/
			NUC_Usart_Tx_1Hz();
		} 
        if(Frequency_Control_cnt % 5 == 0)
		{
			/*200HZ发送数据*/
			NUC_Usart_Tx_200Hz();
		}
        Frequency_Control_cnt+=1;
        vTaskDelay(10);
	}
}


/*end*/

/* sove bigin*/
/*中断接口,以及原始数据接受=收*/
void USART1_IRQHandler_1(void)
{
	static uint16_t RX_Len_Now = 0;
	if(USART1->SR & UART_FLAG_IDLE)
		{
			__HAL_UART_CLEAR_PEFLAG(&huart1);//清标志位
			__HAL_DMA_DISABLE(huart1.hdmarx);//dma失能
			RX_Len_Now = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);//数据长度
			__HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
		if((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
			memcpy(NUC_USART_RxBuf, (char*)Usart1_Buf[0], RX_Len_Now);
			memset((char*)Usart1_Buf[0],0,USART_RX_BUF_LENGHT);
		}
		else
		{
			huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			memcpy(NUC_USART_RxBuf, (char*)Usart1_Buf[1], RX_Len_Now);
			memset((char*)Usart1_Buf[1],0,USART_RX_BUF_LENGHT);
		}
			__HAL_DMA_ENABLE(huart1.hdmarx);
		/*数据处理*/
			NUC_Data_Unpack();
	}
}

/*数据处理函数:nuc一帧发送多种数据,因此得先分离处理*/
//难道不能自己手动分频吗(
uint8_t NUC_Data_Unpack()
{	
	uint8_t	*p_unpack	=	NUC_USART_RxBuf;//已解包数据地址
	uint8_t *p_wait		=	NUC_USART_RxBuf;//待解包数据地址//指针给初值怕乱飞
	//循环解包
	//输入解包检测地址,下一数据指针地址(直接传指针并不能拿出数据),用来存储数据长度的变量的地址,储存指令变量的地址
	while(data_unpack(p_unpack,&p_wait,&lenth,&cmid))
	{ 
		switch(cmid)
		{//  #define NUC_DATA &p_unpack[7]   数据转录 ,检测地址后移 (好像用break会打断循环来着(之前改的忘了))
			case  0x0501 : memcpy(&nuc_control.chassis_v,NUC_DATA,12);										p_unpack=p_wait;nuc_work[0]=1;nuc_off_wait[0]=5000;break;
			case  0x0502 : memcpy(&nuc_control.action,NUC_DATA,sizeof(nuc_control.action)); 	p_unpack=p_wait;nuc_work[1]=1;nuc_off_wait[1]=5000;break;
			case  0x0506 : memcpy(&nuc_control.list,NUC_DATA,sizeof(nuc_control.list)); 	p_unpack=p_wait;nuc_work[2]=1;nuc_off_wait[2]=5000;break;
			case  0x0301 : memcpy(&Sentry_Decision_0x0120,NUC_DATA,sizeof(Sentry_Decision_0x0120)); 	p_unpack=p_wait;nuc_work[3]=1;nuc_off_wait[3]=5000;break;
			case  0x0308 : memcpy(&custom_info_0x0308,NUC_DATA,sizeof(custom_info_0x0308)); 	p_unpack=p_wait;nuc_work[4]=1;nuc_off_wait[4]=5000;break; 
			default ://nuc消息转发给裁判系统
				// memcpy(data_send,p_unpack,5+2+lenth+2);
//			HAL_UART_Transmit_DMA(&huart6,data_send , 5+2+lenth+2);         
			break;	
		}		
		if(cmid == 0x0301){Custom_info_CAN_send_flag = 2;}//发送0102
		if(cmid == 0x0308){Custom_info_CAN_send_flag=1;}//小端发送
		p_unpack=p_wait;
	}
	//缓冲区清零//不清零可能会使用之前指令
		memset((char*)NUC_USART_RxBuf,0,USART_RX_BUF_LENGHT);
		//死机检测
	return 0;
}
/*数据分离处理*/
uint8_t data_unpack(uint8_t* data_in,uint8_t** data_out,uint16_t* lenth,uint16_t* cmid)
{
	if(data_in[0]!=0xA5)  return 0;//头判断
	else if(data_in[0]==0xA5)//分离摘录
	{
			*lenth=data_in[1]|(data_in[2]<<8);
			*cmid	=data_in[5]|(data_in[6]<<8);
			*data_out=data_in+5+2+*lenth +2;
			return 1;
	}
	return 0;
}

/*send begin */
void NUC_Usart_Tx_200Hz(){
	referee_copy(0x0606,45,(uint8_t *)&NUC_Used_Message_200Hz_0606,Usart1_Dma_Txbuf);//决策要求的裁判系统格式(真想摆了)
	HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, 5+2+45+2);
}

void NUC_Usart_Tx_1Hz(){
	referee_copy(0x0607,96,(uint8_t *)&NUC_Used_Message_1Hz_0607,Usart1_Dma_Txbuf);
	HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, 5+2+96+2);
}
/*数据转载成裁判系统格式*/
void referee_copy(uint16_t cmid,uint16_t length,uint8_t * data,uint8_t* uart_send)
{
	/*head_copy*///这里看裁判系统串口协议
	Protocol_Head.SOF=0xA5;
	Protocol_Head.seq+=1;
	Protocol_Head.data_length = length ;
	Protocol_Head.CRC8=CRC_Calculation((uint8_t*)&Protocol_Head,4);
	memcpy(uart_send,(uint8_t *)(&Protocol_Head), 5);
	/*cmid_copy*/ 
	uart_send[5]=cmid&0x0f;
	uart_send[6]=(cmid>>8)&0x0f;
	/*data_copy*/
	memcpy(&uart_send[7], (uint8_t *)(data), length);
	/*crc_16*/
	*((uint16_t*)(uart_send+5+2+length))=CRC16_Calculate(uart_send, 5+2+length);
}

