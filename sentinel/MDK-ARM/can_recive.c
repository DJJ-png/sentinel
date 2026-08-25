#include "bsp_can.h"
#include "main.h"
#include "bsp_cap.h"
#include "Referee.h"
#include "robot_message.h"
#include "robot_send_referee.h"
#include "referee.h"
#include "dm_imu.h"
#include "FreeRTOS.h"
#include "Nmanifold_usart_task.h"
#include "config_set.h"
#include "bsp_math.h"
#include "config_set.h"
CAN_RxHeaderTypeDef rx_header;

motor_measure_t motor_measure_gimbal[3];
motor_measure_t motor_measure_shoot[4];

uint8_t rx_data[8];

int32_t dial_angle;

int8_t dial_cnt=0;

uint16_t position_cnt_receive = 0;

uint16_t receive_upboard_cnt = 0;
uint8_t receive_0x0308_begin_flag = 0;
uint8_t receive_0x0301_begin_flag = 0;

/**
 *@brief	回馈函数,用于接收电机返回的数据
 *@param	hcan
 *@return	none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan==&hcan1)	Can1_Manage(rx_header.StdId);
	if(hcan==&hcan2)	Can2_Manage(rx_header.StdId);
}

//Can1
void Can1_Manage(uint32_t Get_Id)
{
	switch (Get_Id)
		{
			
				case CAN_LK_BASE_YAW_ID://141
				{
					motor_measure_LK(&motor_measure_gimbal[BASE_YAW], rx_data);		break;
					break;
				}

				/* 拨弹盘电机 */
				case CAN_DIAL_MOTOR_ID:
				{
					get_motor_measure(&motor_measure_shoot[0], rx_data);
					Motor_2006_Ecd(motor_measure_shoot[0],&dial_cnt,&dial_angle);			break;				
					break;
				}

				
				/* 0x0001 */
				case CAN_ID_GAME_ENV_POS:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case CMD_601_GAME_STATUS:
						{
							Game_Status.game_progress =((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							// Game_Status.game_progress = 1;
							Game_Status.stage_remain_time = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Dart_Information.dart_info = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
							break;
						}
						case CMD_601_EVENT_DATA:
						{
							Event_Data.event_type = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
						case CMD_601_PROJECTILE_ALLOW:
						{
							Projectile_Allowance.projectile_allowance_17mm =((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Projectile_Allowance.remaining_gold_coin = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							break;
						}
						case CMD_601_SENTRY_INFO:
						{
							Sentry_Info.sentry_info_2 =((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Sentry_Info.sentry_info = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
						case CMD_601_POS_HERO_X :
						{
							memcpy((uint8_t*)&Ground_Robot_Position.hero_x,		rx_data+4,	4);
							break;
						}
						case CMD_601_POS_HERO_Y :
						{
							memcpy((uint8_t*)&Ground_Robot_Position.hero_y,		rx_data+4,	4);
							break;
						}
						case CMD_601_POS_ENGINEER_X:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.engineer_x,	rx_data+4,	4);
							break;
						}
						case CMD_601_POS_ENGINEER_Y:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.engineer_y,	rx_data+4,	4);
							break;
						}
						case CMD_601_POS_STND_3_X :
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_3_x,rx_data+4,	4);
							break;
						}
						case CMD_601_POS_STND_3_Y:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_3_y,rx_data+4,	4);
							break;
						}
						case CMD_601_POS_STND_4_X :
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_4_x,rx_data+4,	4);
							break;
						}
						case CMD_601_POS_STND_4_Y:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_4_y,rx_data+4,	4);
							break;
						}
						case CMD_601_RADAR_MARK:
						{
							Radar_Mark_Data.mark_progress = rx_data[2];
						}
					}
					break;
				}
				
				/* 0x0003 */
				case CAN_ID_GLOBAL_HP_RFID:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case  CMD_602_HP_ALLY_123:
						{
							Game_Robot_HP.red_1_robot_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.red_2_robot_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Game_Robot_HP.red_3_robot_HP = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
							break;
						}
						case CMD_602_HP_ALLY_47_BASE:
						{
							Game_Robot_HP.red_4_robot_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.red_7_robot_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Game_Robot_HP.red_base_HP = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);						
							break;
						}
						case CMD_602_HP_ALLY_BASE_OUT :
						{
							Game_Robot_HP.red_base_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.red_outpost_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							break;
						}
						case CMD_602_RFID_STATUS:
						{
							RFID_Status.rfid_status = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
					}
					break;
				}
				
				case  CAN_ID_SELF_STATUS_MAP:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case CMD_603_HURT_DATA :
						{
							Hurt_Data.armor_id = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Hurt_Data.HP_deduction_reason = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							break;
						}
						case CMD_603_ROBOT_STATUS:
						{
							Robot_Status.power_management_shooter_output = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Robot_Status.shooter_barrel_heat_limit = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Robot_Status.robot_id = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);	
							break;
						}
						case CMD_603_POWER_OUTPUT:
						{
							Robot_Status.power_management_chassis_output = rx_data[2];
							Robot_Status.power_management_gimbal_output = rx_data[3];
							Robot_Status.power_management_shooter_output = rx_data[4];					
						}
						case CMD_603_POWER_HEAT:
						{
							Power_Heat_Data.shooter_17mm_1_barrel_heat = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Power_Heat_Data.buffer_energy = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Map_Command.cmd_keyboard = rx_data[6];
							break;
						}
						case CMD_603_MAP_TARGET_X:
						{
							Map_Command.target_position_x = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
						case CMD_603_MAP_TARGET_Y:
						{
							Map_Command.target_position_y = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
					}
					break;
				}

				case CAN_ID_BUFF_INTERACT:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case CMD_604_BUFF_ENERGY:
						{
							Buff_Energy.remaining_energy = rx_data[2];
							Buff_Energy.defence_buff = rx_data[3];
							break;
						}
						case CMD_604_ENEMY_Z :
						{
							Robot_Interaction_User_Data.enemy[0].z = rx_data[2];Robot_Interaction_User_Data.enemy[1].z = rx_data[3];Robot_Interaction_User_Data.enemy[2].z = rx_data[4];Robot_Interaction_User_Data.enemy[3].z = rx_data[5];Robot_Interaction_User_Data.enemy[4].z = rx_data[6];Robot_Interaction_User_Data.enemy[5].z = rx_data[7];
							break;
						}
						case CMD_604_ALLIANCE_Z :
						{
							Robot_Interaction_User_Data.alliance[0].z = rx_data[2];Robot_Interaction_User_Data.alliance[1].z = rx_data[3];Robot_Interaction_User_Data.alliance[2].z = rx_data[4];Robot_Interaction_User_Data.alliance[3].z = rx_data[5];Robot_Interaction_User_Data.alliance[5].z = rx_data[6];Robot_Interaction_User_Data.radar_enhance = rx_data[7];
							break;
						}
					}
					break;
				}

				case CAN_ID_ENEMT_STATE ://敌方车辆数据
				{
					enemy_state_data[rx_data[0]].id = rx_data[0];
					enemy_state_data[rx_data[0]].x = (rx_data[2] << 8 | rx_data[3]);
					enemy_state_data[rx_data[0]].y = (rx_data[4] << 8 | rx_data[5]);
					enemy_state_data[rx_data[0]].hp = (rx_data[6] << 8 | rx_data[7]);
					break;
				}
				case CAN_ID_MY_POSITION1 :   //自己车位置数据
				{
					memcpy((uint8_t*)&Robot_Pos.x,		rx_data,	4);
					memcpy((uint8_t*)&Robot_Pos.y,		rx_data+4,	4);
					break;
				} 
				case CAN_ID_MY_POSITION2:
				{
					memcpy((uint8_t*)&Robot_Pos.angle,		rx_data,	4);
					break;
				}
		}

}
 

float adv_yaw,adv_pitch;
float adv_yaw_speed,adv_pitch_speed;
void Can2_Manage(uint32_t Get_Id)
{
		switch (rx_header.StdId)
		{
			
				/* 摩擦轮电机 */
				case CAN_FRIC_MOTOR1_ID:
				{
					get_motor_measure(&motor_measure_shoot[2], rx_data);							break;
				}
				
				case CAN_FRIC_MOTOR2_ID:	
				{
					get_motor_measure(&motor_measure_shoot[3], rx_data);							break;
				}
				case CAN_ADVANCED_YAW_ID://207
				{
					get_motor_measure(&motor_measure_gimbal[ADVANCED_YAW], rx_data);		break;
				}
				
				case CAN_LK_PITCH_ID:
				{
  					motor_measure_LK(&motor_measure_gimbal[PITCH],rx_data);			break;
				}
				case CAN_IMU_ID1:
                {
                   adv_yaw = *(float*)(&rx_data[0]);                                     
                   adv_pitch= *(float*)(&rx_data[4]);     break;
                }
                case CAN_IMU_ID2:
                {
                   adv_yaw_speed = *(float*)(&rx_data[0]); 
                   adv_pitch_speed= *(float*)(&rx_data[4]);     break;
                }
		}
}
