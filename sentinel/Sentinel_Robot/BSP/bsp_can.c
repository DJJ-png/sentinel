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
/****   NX通信需要   ****/

uint8_t ins_cnt;
uint8_t ins_buff[14];
uint8_t pitch_error;

/***********************/

/*数据接收*/

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

/*数据处理函数*/

/*can发送控制4个同一类id电机*/
void CAN_CMD_BASE(CAN_HandleTypeDef* hcan,uint32_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message ={0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = id;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;
  
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_CMD_f16_3(CAN_HandleTypeDef* hcan,uint32_t id, uint8_t position, uint16_t data1, uint16_t data2, uint16_t data3)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message ={0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = id;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = position;
	chassis_can_send_data[1] = data1 >> 8;
	chassis_can_send_data[2] = data1;
	chassis_can_send_data[3] = data2 >> 8;
	chassis_can_send_data[4] = data2;
	chassis_can_send_data[5] = data3 >> 8;
	chassis_can_send_data[6] = data3;
	chassis_can_send_data[7] = 0;
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_CMD_f8_7(CAN_HandleTypeDef* hcan,uint32_t id, uint8_t position, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message ={0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = id;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = position;
	chassis_can_send_data[1] = data1;
	chassis_can_send_data[2] = data2;
	chassis_can_send_data[3] = data3;
	chassis_can_send_data[4] = data4;
	chassis_can_send_data[5] = data5;
	chassis_can_send_data[6] = data6;
	chassis_can_send_data[7] = data7;
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_CMD_f32(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,fp32 data2)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef  chassis_tx_message ={0};
    uint8_t              chassis_can_send_data[8];
    chassis_tx_message.StdId = id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    memcpy(chassis_can_send_data, (uint8_t*)&data1 ,4);
    memcpy(chassis_can_send_data+4, (uint8_t*)&data2 ,4);
  
    HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_CMD_f32_f8(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,uint8_t data2,uint8_t data3,uint8_t data4,uint8_t data5)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef  chassis_tx_message ={0};
    uint8_t              chassis_can_send_data[8];
    chassis_tx_message.StdId = id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    memcpy(chassis_can_send_data, (uint8_t*)&data1 ,4);
    chassis_can_send_data[4] = data2;
    chassis_can_send_data[5] = data3;
    chassis_can_send_data[6] = data4;
    chassis_can_send_data[7] = data5;
  
    HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//2006编码器计算处理
void Motor_2006_Ecd(motor_measure_t data_in,int8_t *data_con,int32_t *data_out)
{
	//过0判断
				if(data_in.ecd>8192-900&&data_in.last_ecd<900)				(*data_con)++;
				else 	if(data_in.ecd<900&&data_in.last_ecd>8192-900)	(*data_con)--;
	//减速比控制
				if((*data_con)<0)(*data_con)+=36;
				if((*data_con)>35)(*data_con)-=36;
	//计算结果
				(*data_out)=(8192/36.0f)*((*data_con)+data_in.ecd/8192.0f);
			(*data_out)=LIMIT_TO_SET((*data_out),4096);
}


//滤波器初始化
void can_filter_init(void)
{
		CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000; 
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* LK电机单电机发送 */
void CAN_cmd_LK_Motor(uint8_t uid, int16_t current)
{
	if(current > 1300) current = 1300;
	else if(current < -1300) current = -1300;
	
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message = {0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = 0x140+uid;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] =0xA1; 
	memcpy(&chassis_can_send_data[4],(uint8_t*)&current,2);
	
	/*	yaw: can1接瓴控电机	*/
	if(uid==1)		HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	if(uid==3)		HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
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
				case 0x201:
				{
					get_motor_measure(&motor_measure_shoot[0], rx_data);
					Motor_2006_Ecd(motor_measure_shoot[0],&dial_cnt,&dial_angle);			break;				
					break;
				}

				
				/* 0x0001 */
				case 0x601:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case 1:
						{
							Game_Status.game_progress =((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							// Game_Status.game_progress = 1;
							Game_Status.stage_remain_time = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Dart_Information.dart_info = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
							break;
						}
						case 101:
						{
							Event_Data.event_type = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
						case 208:
						{
							Projectile_Allowance.projectile_allowance_17mm =((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Projectile_Allowance.remaining_gold_coin = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							break;
						}
						case 0x20D:
						{
							Sentry_Info.sentry_info_2 =((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Sentry_Info.sentry_info = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
						case 0x210:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.hero_x,		rx_data+4,	4);
							break;
						}
						case 0x211:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.hero_y,		rx_data+4,	4);
							break;
						}
						case 0x212:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.engineer_x,		rx_data+4,	4);
							break;
						}
						case 0x213:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.engineer_y,		rx_data+4,	4);
							break;
						}
						case 0x214:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_3_x,		rx_data+4,	4);
							break;
						}
						case 0x215:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_3_y,		rx_data+4,	4);
							break;
						}
						case 0x216:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_4_x,		rx_data+4,	4);
							break;
						}
						case 0x217:
						{
							memcpy((uint8_t*)&Ground_Robot_Position.standard_4_y,		rx_data+4,	4);
							break;
						}
						case 0x20C:
						{
							Radar_Mark_Data.mark_progress = rx_data[2];
						}
					}
					break;
				}
				
				/* 0x0003 */
				case 0x602:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case 3:
						{
							Game_Robot_HP.blue_1_robot_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.blue_2_robot_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Game_Robot_HP.blue_3_robot_HP = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
							break;
						}
						case 4:
						{
							Game_Robot_HP.blue_4_robot_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.blue_7_robot_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Game_Robot_HP.blue_base_HP = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);						
							break;
						}
						case 5:
						{
							Game_Robot_HP.blue_outpost_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.red_1_robot_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Game_Robot_HP.red_2_robot_HP = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);						
							break;
						}
						case 6:
						{
							Game_Robot_HP.red_3_robot_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.red_4_robot_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Game_Robot_HP.red_7_robot_HP = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);						
							break;
						}
						case 7:
						{
							Game_Robot_HP.red_base_HP = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Game_Robot_HP.red_outpost_HP = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							break;
						}
						case 209:
						{
							RFID_Status.rfid_status = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							//memcpy(&RFID_Status.rfid_status,&rx_data[4],4);
							break;
						}
					}
					break;
				}
				
				case 0x603:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case 206:
						{
							Hurt_Data.armor_id = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Hurt_Data.HP_deduction_reason = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							break;
						}
						case 201:
						{
							Robot_Status.power_management_shooter_output = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Robot_Status.shooter_barrel_heat_limit = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Robot_Status.robot_id = ((int16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);	
							break;
						}
						case 211:
						{
							Robot_Status.power_management_chassis_output = rx_data[2];
							Robot_Status.power_management_gimbal_output = rx_data[3];
							Robot_Status.power_management_shooter_output = rx_data[4];					
						}
						case 202:
						{
							Power_Heat_Data.shooter_17mm_1_barrel_heat = ((int16_t)rx_data[2]<<8)|((int16_t)rx_data[3]<<0);
							Power_Heat_Data.buffer_energy = ((int16_t)rx_data[4]<<8)|((int16_t)rx_data[5]<<0);
							Map_Command.cmd_keyboard = rx_data[6];
							break;
						}
						case 303:
						{
							Map_Command.target_position_x = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
						case 304:
						{
							Map_Command.target_position_y = ((uint32_t)rx_data[4] << 24) | ((uint32_t)rx_data[5] << 16) | ((uint32_t)rx_data[6] << 8) | (uint32_t)rx_data[7];
							break;
						}
					}
					break;
				}

				case 0x604:
				{
					position_cnt_receive =((int16_t)rx_data[0]<<8)|((int16_t)rx_data[1]<<0);
					switch (position_cnt_receive)
					{
						case 204:
						{
							Buff_Energy.remaining_energy = rx_data[2];
							Buff_Energy.defence_buff = rx_data[3];
							break;
						}
//						case 301:
//						{
//							Robot_Interaction_User_Data.alliance[0].x = ((uint16_t)rx_data[2]<<8)|((uint16_t)rx_data[3]<<0);Robot_Interaction_User_Data.alliance[0].y = ((uint16_t)rx_data[4]<<8)|((uint16_t)rx_data[5]<<0);Robot_Interaction_User_Data.alliance[1].x = ((uint16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
//						  break;
//						}
//						case 302:
//						{
//							Robot_Interaction_User_Data.alliance[1].y = ((uint16_t)rx_data[2]<<8)|((uint16_t)rx_data[3]<<0);Robot_Interaction_User_Data.alliance[2].x = ((uint16_t)rx_data[4]<<8)|((uint16_t)rx_data[5]<<0);Robot_Interaction_User_Data.alliance[2].y = ((uint16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
//							break;
//						}
//						case 303:
//						{
//							Robot_Interaction_User_Data.alliance[3].x = ((uint16_t)rx_data[2]<<8)|((uint16_t)rx_data[3]<<0);Robot_Interaction_User_Data.alliance[3].y = ((uint16_t)rx_data[4]<<8)|((uint16_t)rx_data[5]<<0);Robot_Interaction_User_Data.alliance[4].x = ((uint16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
//						  break;
//						}
//						case 304:
//						{
//							Robot_Interaction_User_Data.alliance[4].y = ((uint16_t)rx_data[2]<<8)|((uint16_t)rx_data[3]<<0);Robot_Interaction_User_Data.alliance[5].x = ((uint16_t)rx_data[4]<<8)|((uint16_t)rx_data[5]<<0);Robot_Interaction_User_Data.alliance[5].y = ((uint16_t)rx_data[6]<<8)|((int16_t)rx_data[7]<<0);
//							break;
//						}
						case 305:
						{
							Robot_Interaction_User_Data.enemy[0].z = rx_data[2];Robot_Interaction_User_Data.enemy[1].z = rx_data[3];Robot_Interaction_User_Data.enemy[2].z = rx_data[4];Robot_Interaction_User_Data.enemy[3].z = rx_data[5];Robot_Interaction_User_Data.enemy[4].z = rx_data[6];Robot_Interaction_User_Data.enemy[5].z = rx_data[7];
							break;
						}
						case 306:
						{
							Robot_Interaction_User_Data.alliance[0].z = rx_data[2];Robot_Interaction_User_Data.alliance[1].z = rx_data[3];Robot_Interaction_User_Data.alliance[2].z = rx_data[4];Robot_Interaction_User_Data.alliance[3].z = rx_data[5];Robot_Interaction_User_Data.alliance[5].z = rx_data[6];Robot_Interaction_User_Data.radar_enhance = rx_data[7];
							break;
						}
					}
					break;
				}

				case ENEMT_STATE_ID://敌方车辆数据
				{
					enemy_state_data[rx_data[0]].id = rx_data[0];
					enemy_state_data[rx_data[0]].x = (rx_data[2] << 8 | rx_data[3]);
					enemy_state_data[rx_data[0]].y = (rx_data[4] << 8 | rx_data[5]);
					enemy_state_data[rx_data[0]].hp = (rx_data[6] << 8 | rx_data[7]);
					break;
				}
				case MY_POSITION1_ID:   //自己车位置数据
				{
					memcpy((uint8_t*)&Robot_Pos.x,		rx_data,	4);
					memcpy((uint8_t*)&Robot_Pos.y,		rx_data+4,	4);
					break;
				} 
				case MY_POSITION2_ID:
				{
					memcpy((uint8_t*)&Robot_Pos.angle,		rx_data,	4);
					break;
				}
		}

}

//Can2
uint8_t text1=0;
void Can2_Manage(uint32_t Get_Id)
{
    if(rx_header.StdId==CAN_3508_M1_ID)
    {
        text1=1;
    }
		switch (rx_header.StdId)
		{
			
				/* 摩擦轮电机 */
				case 0x203:
				{
					get_motor_measure(&motor_measure_shoot[2], rx_data);							break;
				}
				
				case 0x204:	
				{
					get_motor_measure(&motor_measure_shoot[3], rx_data);							break;
				}
				
				/*接收达妙IMU数据*/
				case DM_IMU_ID:
				{
					IMU_UpdateData(rx_data);												break;
				}

				case CAN_ADVANCED_YAW_ID://207
				{
					get_motor_measure(&motor_measure_gimbal[ADVANCED_YAW], rx_data);		break;
				}
				
				case CAN_LK_PITCH_ID:
				{
					pitch_error = rx_data[7];
					motor_measure_LK(&motor_measure_gimbal[PITCH],rx_data);			break;
				}
				//接收NX自瞄信息
        //  case RECV_NX_ID_1:        //0x411
        //  {
        //      memcpy((uint8_t*)&nuc_receive_data.aim_data_received.pitch,        rx_data,    4);                
        //      memcpy((uint8_t*)&nuc_receive_data.aim_data_received.yaw,        rx_data+4,    4);                
        //      break;
        //   }    
        //   case RECV_NX_ID_2:        //0x412
        //   {
        //      memcpy((uint8_t*)&nuc_receive_data.aim_data_received.distance,        rx_data,    4);            
        //      nuc_receive_data.aim_data_received.is_fire = rx_data[4];
        //      nuc_receive_data.aim_data_received.target_number = rx_data[5];
        //      if(nuc_receive_data.aim_data_received.distance>0) 
        //         nuc_receive_data.aim_data_received.success=1;
        //      else  
        //         nuc_receive_data.aim_data_received.success=0;
        //      break;
		// 			}
					
//					case RECV_NX_ID_1:        //0x411
//					{
//							memcpy((uint8_t*)&ins_buff[rx_data[0]*6],rx_data+2,rx_data[1]);
//					}					
					
		}
}

