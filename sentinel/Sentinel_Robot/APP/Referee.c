 /**
  ******************************************************************************
  * @file    referee.c
  * @author  圣灵亮欣
  * @version V1.7.0
  * @date    2025/01/08
  * @brief   
  ******************************************************************************
  * @attention
	*   
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "referee.h"
#include "protocol.h"
#include "string.h"
#include "usart.h"
#include "fifo.h"
#include "crcs.h"
#include <stdarg.h>

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* protocol包头结构体 */
frame_header_struct_t Referee_Receive_Header;

/* 0x00XX */
ext_game_status_t							 		Game_Status;
ext_game_result_t							 		Game_Result;
ext_game_robot_HP_t						 		    Game_Robot_HP;

/* 0x01XX */
ext_event_data_t                	                Event_Data;
ext_referee_warning_t           	                Referee_Warning;
ext_dart_info_t								 		Dart_Information;
ext_sentry_cmd_t									Sentry_Cmd;
ext_radar_cmd_t										Radar_Cmd;

/* 0x02XX */
ext_robot_status_t 						 		Robot_Status;
ext_power_heat_data_t 					 	Power_Heat_Data;
ext_robot_pos_t   							 	Robot_Pos;
ext_buff_energy_t       					Buff_Energy;
ext_hurt_data_t	     					 		Hurt_Data;
ext_shoot_data_t      					 	Shoot_Data;
ext_projectile_allowance_t			 	Projectile_Allowance;
ext_rfid_status_t      				 		RFID_Status;
ext_dart_client_cmd_t					 		Dart_Client_Cmd;
ext_ground_robot_position_t		 		Ground_Robot_Position;
ext_radar_mark_data_t					 		Radar_Mark_Data;
ext_sentry_info_t							 		Sentry_Info;
ext_radar_info_t								 	Radar_Info;

/* 0x03XX */
ext_robot_interaction_header_t		Robot_Interaction_Header_Data;
ext_robot_interaction_user_data_t	Robot_Interaction_User_Data;
ext_custom_robot_data_t						Custom_Robot_Data;
ext_map_command_t									Map_Command;
ext_remote_control_t							Remote_Control;
ext_map_robot_data_t							Map_Robot_Data;
ext_custom_client_data_t					Custom_Client_Data;
ext_map_data_t										Map_Data;
ext_custom_info_t									Custom_Info;
ext_robot_custom_data_t						Robot_Custom_Data;

/* Functions -----------------------------------------------------------------*/
/*==============================================================================
              ##### 裁判系统初始化函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 裁判系统结构体初始化函数 Referee_StructInit
			(+) 裁判系统串口初始化函数 Referee_UARTInit
*/
void Referee_StructInit(void)
{
	memset(&Referee_Receive_Header,						0, sizeof(Referee_Receive_Header));
	
	/* 0x00XX */
	memset(&Game_Status,											0, sizeof(Game_Status));
	memset(&Game_Result,											0, sizeof(Game_Result));
	memset(&Game_Robot_HP,										0, sizeof(Game_Robot_HP));
	
	/* 0x01XX */
	memset(&Event_Data,												0, sizeof(Event_Data));
	memset(&Referee_Warning,									0, sizeof(Referee_Warning));
	memset(&Dart_Information,									0, sizeof(Dart_Information));
	memset(&Sentry_Cmd,												0, sizeof(Sentry_Cmd));
	memset(&Radar_Cmd,												0, sizeof(Radar_Cmd));
	
	/* 0x02XX */
	memset(&Robot_Status,											0, sizeof(Robot_Status));
	memset(&Power_Heat_Data,									0, sizeof(Power_Heat_Data));
	memset(&Robot_Pos,												0, sizeof(Robot_Pos));
	memset(&Buff_Energy,											0, sizeof(Buff_Energy));
	memset(&Hurt_Data,												0, sizeof(Hurt_Data));
	memset(&Shoot_Data,												0, sizeof(Shoot_Data));
	memset(&Projectile_Allowance,							0, sizeof(Projectile_Allowance));
	memset(&RFID_Status, 											0, sizeof(RFID_Status));
	memset(&Dart_Client_Cmd,									0, sizeof(Dart_Client_Cmd));
	memset(&Ground_Robot_Position,						0, sizeof(Ground_Robot_Position));
	memset(&Radar_Mark_Data,									0, sizeof(Radar_Mark_Data));
	memset(&Sentry_Info,											0, sizeof(Sentry_Info));
	memset(&Radar_Info,												0, sizeof(Radar_Info));
	
	/* 0x03XX */
	memset(&Robot_Interaction_Header_Data,		0, sizeof(Robot_Interaction_Header_Data));
	memset(&Robot_Interaction_User_Data,			0, sizeof(Robot_Interaction_User_Data));
	memset(&Custom_Robot_Data,								0, sizeof(Custom_Robot_Data));
	memset(&Map_Command,											0, sizeof(Map_Command));
	memset(&Remote_Control,										0, sizeof(Remote_Control));
	memset(&Map_Robot_Data,										0, sizeof(Map_Robot_Data));
	memset(&Custom_Client_Data,								0, sizeof(Custom_Client_Data));
	memset(&Map_Data,													0, sizeof(Map_Data));
	memset(&Custom_Info,											0, sizeof(Custom_Info));
	memset(&Robot_Custom_Data,								0, sizeof(Robot_Custom_Data));
}

void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength)
{
	/* 使能串口DMA */
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAT);
	
	/* 使能串口空闲中断 */
	__HAL_UART_ENABLE_IT(&Referee_UART, UART_IT_IDLE);
	
	/* 确保DMA RX失能 */
	while(Referee_UART.hdmarx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmarx);
	}
	
	/* 清空标志位 */
	__HAL_DMA_CLEAR_FLAG(Referee_UART.hdmarx, DMA_LISR_TCIF1);

	/* 设置接收双缓冲区 */
	Referee_UART.hdmarx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
	Referee_UART.hdmarx->Instance->M0AR = (uint32_t)(Buffer0);
	Referee_UART.hdmarx->Instance->M1AR = (uint32_t)(Buffer1);
	
	/* 设置数据长度 */
	__HAL_DMA_SET_COUNTER(Referee_UART.hdmarx, BufferLength);
	
	/* 使能双缓冲区 */
	SET_BIT(Referee_UART.hdmarx->Instance->CR, DMA_SxCR_DBM);
	
	/* 使能DMA RX */
	__HAL_DMA_ENABLE(Referee_UART.hdmarx);
	
	/* 确保DMA TX失能 */
	while(Referee_UART.hdmatx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmatx);
	}
	
	Referee_UART.hdmatx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
}

/*==============================================================================
              ##### 裁判系统数据解析函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 裁判系统队列数据解压函数 Referee_UnpackFifoData
      (+) 裁判系统队列数据处理函数 Referee_SolveFifoData
*/
void Referee_UnpackFifoData(unpack_data_t *referee_unpack_obj, fifo_s_t *referee_fifo)
{
  uint8_t byte = 0;
  uint8_t sof  = HEADER_SOF;
	
  while(fifo_s_used(referee_fifo))
  {
    byte = fifo_s_get(referee_fifo);
    switch(referee_unpack_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          referee_unpack_obj->unpack_step = STEP_LENGTH_LOW;
          referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        }
        else
        {
          referee_unpack_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        referee_unpack_obj->data_len = byte;
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        referee_unpack_obj->data_len |= (byte << 8);
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          referee_unpack_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
        }
      }break;
			
      case STEP_FRAME_SEQ:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_HEADER_CRC8;
      }break;
			
      case STEP_HEADER_CRC8:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if(CRC08_Verify(referee_unpack_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
          {
            referee_unpack_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
            referee_unpack_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if(referee_unpack_obj->index <  (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
           referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;  
        }
        if(referee_unpack_obj->index >= (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
          if(CRC16_Verify(referee_unpack_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
          {
            Referee_SolveFifoData(referee_unpack_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
        referee_unpack_obj->index = 0;
      }break;
    }
  }
}

float robo_temp_x=0,robo_temp_y=0;
uint8_t send_flag;
uint16_t cmd_id_text = 0;

//受击检测
uint8_t if_hurted = 0;

void Referee_SolveFifoData(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t  index  = 0;
	
	memcpy(&Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);
	cmd_id_text = cmd_id;
	
/* 此处根据各兵种所能接收的命令码和数据以及程序运行需要决定哪些可以被注释 */
	switch(cmd_id)
	{
/* 0x00XX */
		case GAME_STATUS_CMD_ID:									memcpy(&Game_Status,									frame + index, sizeof(ext_game_status_t));									break;
		//case GAME_RESULT_CMD_ID:									memcpy(&Game_Result,									frame + index, sizeof(ext_game_result_t));									break;
		case GAME_ROBOT_HP_CMD_ID:								memcpy(&Game_Robot_HP,								frame + index, sizeof(ext_game_robot_HP_t));								break;

/* 0x01XX */
    case EVENT_DATA_CMD_ID:										memcpy(&Event_Data,										frame + index, sizeof(ext_event_data_t));										break;
		//case REFEREE_WARNING_CMD_ID:							memcpy(&Referee_Warning,							frame + index, sizeof(ext_referee_warning_t));							break;
		case DART_INFO_CMD_ID:										memcpy(&Dart_Information,							frame + index, sizeof(ext_dart_info_t));										break;
		
/* 0x02XX */
		case ROBOT_STATUS_CMD_ID:									memcpy(&Robot_Status,									frame + index, sizeof(ext_robot_status_t));									break;
		case POWER_HEAT_DATA_CMD_ID:							memcpy(&Power_Heat_Data,							frame + index, sizeof(ext_power_heat_data_t));							break;
		case ROBOT_POS_CMD_ID:										memcpy(&Robot_Pos,										frame + index, sizeof(ext_robot_pos_t));										break;
		case BUFF_ENERGY_CMD_ID:									memcpy(&Buff_Energy,									frame + index, sizeof(ext_buff_energy_t));									break;
		case HURT_DATA_CMD_ID:
		{
			memcpy(&Hurt_Data,										frame + index, sizeof(ext_hurt_data_t));										
			if_hurted = 1;
			break;
		}
		case SHOOT_DATA_CMD_ID:										memcpy(&Shoot_Data,										frame + index, sizeof(ext_shoot_data_t));										break;
		case PROJECTILE_ALLOWANCE_CMD_ID:					memcpy(&Projectile_Allowance,					frame + index, sizeof(ext_projectile_allowance_t));					break;
		case RFID_STATE_CMD_ID:										memcpy(&RFID_Status,									frame + index, sizeof(ext_rfid_status_t));									break;
		//case DART_CLIENT_CMD_ID:									memcpy(&Dart_Client_Cmd,							frame + index, sizeof(ext_dart_client_cmd_t));							break;
		case GROUND_ROBOT_POSITION_CMD_ID:				memcpy(&Ground_Robot_Position,				frame + index, sizeof(ext_ground_robot_position_t));				break;
		//case RADAR_MARK_DATA_CMD_ID:							memcpy(&Radar_Mark_Data,							frame + index, sizeof(ext_radar_mark_data_t));							break;
		case SENTRY_INFO_CMD_ID:									memcpy(&Sentry_Info,									frame + index, sizeof(ext_sentry_info_t));									break;
		//case RADAR_INFO_CMD_ID:										memcpy(&Radar_Info,										frame + index, sizeof(ext_radar_info_t));										break;
		
/* 0x03XX */
		case ROBOT_INTERACTION_DATA_CMD_ID:				memcpy(&Robot_Interaction_User_Data,	frame + index, sizeof(ext_robot_interaction_user_data_t));	break;
		//case CUSTOM_ROBOT_DATA_CMD_ID:						memcpy(&Custom_Robot_Data,						frame + index, sizeof(ext_custom_robot_data_t));						break;
		case MAP_COMMAND_CMD_ID:									memcpy(&Map_Command,									frame + index, sizeof(ext_map_command_t));									break;
		//case REMOTE_CONTROL_CMD_ID:								memcpy(&Remote_Control,								frame + index, sizeof(ext_remote_control_t));								break;
		//case ROBOT_CUSTOM_DATA_CMD_ID:						memcpy(&Robot_Custom_Data,						frame + index, sizeof(ext_robot_custom_data_t));						break;
		
		default:																																																																		break;
	}
}