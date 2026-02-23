#include "referee_upboard_task.h"
#include "main.h"
#include "can.h"
#include "Referee.h"
#include "robot_send_referee.h"
#include "bsp_can.h"
#include "referee.h"
#include "robot_message.h"

uint16_t position_cnt_send = 0;
uint16_t Frequency_Control = 1;

void CAN_CMD_BASE_Referee_8bit(CAN_HandleTypeDef* hcan,uint32_t id, uint16_t position, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  referee_tx_message ={0};
	uint8_t              referee_can_send_data[8];
	referee_tx_message.StdId = id;
	referee_tx_message.IDE = CAN_ID_STD;
	referee_tx_message.RTR = CAN_RTR_DATA;
	referee_tx_message.DLC = 0x08;
	referee_can_send_data[0] = position >> 8;
	referee_can_send_data[1] = position;
	referee_can_send_data[2] = data1;
	referee_can_send_data[3] = data2;
	referee_can_send_data[4] = data3;
	referee_can_send_data[5] = data4;
	referee_can_send_data[6] = data5;
	referee_can_send_data[7] = data6;
  
	HAL_CAN_AddTxMessage(hcan, &referee_tx_message, referee_can_send_data, &send_mail_box);
}

void CAN_CMD_BASE_Referee_16bit(CAN_HandleTypeDef* hcan,uint32_t id, int16_t referee_1, int16_t referee_2, int16_t referee_3, int16_t referee_4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  referee_tx_message ={0};
	uint8_t              referee_can_send_data[8];
	referee_tx_message.StdId = id;
	referee_tx_message.IDE = CAN_ID_STD;
	referee_tx_message.RTR = CAN_RTR_DATA;
	referee_tx_message.DLC = 0x08;
	referee_can_send_data[0] = referee_1 >> 8;
	referee_can_send_data[1] = referee_1;
	referee_can_send_data[2] = referee_2 >> 8;
	referee_can_send_data[3] = referee_2;
	referee_can_send_data[4] = referee_3 >> 8;
	referee_can_send_data[5] = referee_3;
	referee_can_send_data[6] = referee_4 >> 8;
	referee_can_send_data[7] = referee_4;
  
	HAL_CAN_AddTxMessage(hcan, &referee_tx_message, referee_can_send_data, &send_mail_box);
}

void CAN_CMD_BASE_Referee_32bit(CAN_HandleTypeDef* hcan, uint32_t id, int16_t referee_1, int16_t referee_2, uint32_t referee_3)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef referee_tx_message = {0};
    uint8_t referee_can_send_data[8];

    referee_tx_message.StdId = id;
    referee_tx_message.IDE = CAN_ID_STD;
    referee_tx_message.RTR = CAN_RTR_DATA;
    referee_tx_message.DLC = 0x08; // 数据长度为 8 字节

    // 打包 referee_1（16 位整数）
    referee_can_send_data[0] = (uint8_t)(referee_1 >> 8);
    referee_can_send_data[1] = (uint8_t)(referee_1);

    // 打包 referee_2（16 位整数）
    referee_can_send_data[2] = (uint8_t)(referee_2 >> 8);
    referee_can_send_data[3] = (uint8_t)(referee_2);

    // 打包 referee_3（32 位整数）
    referee_can_send_data[4] = (uint8_t)(referee_3 >> 24);
    referee_can_send_data[5] = (uint8_t)(referee_3 >> 16);
    referee_can_send_data[6] = (uint8_t)(referee_3 >> 8);
    referee_can_send_data[7] = (uint8_t)(referee_3);

    // 发送 CAN 消息
    HAL_CAN_AddTxMessage(hcan, &referee_tx_message, referee_can_send_data, &send_mail_box);
}

void CAN_CMD_BASE_Referee_float(CAN_HandleTypeDef* hcan,uint32_t id, uint16_t position, uint8_t data1, uint8_t data2, fp32 data3)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  referee_tx_message ={0};
	uint8_t              referee_can_send_data[8];
	referee_tx_message.StdId = id;
	referee_tx_message.IDE = CAN_ID_STD;
	referee_tx_message.RTR = CAN_RTR_DATA;
	referee_tx_message.DLC = 0x08;
	referee_can_send_data[0] = position >> 8;
	referee_can_send_data[1] = position;
	referee_can_send_data[2] = data1;
	referee_can_send_data[3] = data2;
	memcpy(referee_can_send_data+4, (uint8_t*)&data3 ,4);
  
	HAL_CAN_AddTxMessage(hcan, &referee_tx_message, referee_can_send_data, &send_mail_box);
}

/* 将下板接收到的裁判系统的信息发送给上板 */ 


void refree_to_upboard_1HZ(void)
{
	/* 0x0001&0x0105 */
	position_cnt_send = 1;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x601,position_cnt_send,Game_Status.game_progress,Game_Status.stage_remain_time,Dart_Information.dart_info);	
	/* 0x0101 */
	position_cnt_send = 101;
	CAN_CMD_BASE_Referee_32bit(&hcan1,0x601,position_cnt_send,0,Event_Data.event_type);
	vTaskDelay(1);
	
	/* 0x020D */
	position_cnt_send = 0x20D;
	CAN_CMD_BASE_Referee_32bit(&hcan1,0x601,position_cnt_send,Sentry_Info.sentry_info_2,Sentry_Info.sentry_info);
	
	/* 0x020B */
	position_cnt_send = 0x210;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.hero_x);	
	vTaskDelay(1);
	
	position_cnt_send = 0x211;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.hero_y);	
	position_cnt_send = 0x212;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.engineer_x);	
	vTaskDelay(1);
	
	position_cnt_send = 0x213;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.engineer_y);	
	position_cnt_send = 0x214;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.standard_3_x);	
	vTaskDelay(1);
	
	position_cnt_send = 0x215;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.standard_3_y);
	position_cnt_send = 0x216;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.standard_4_x);	
	vTaskDelay(1);
	
	position_cnt_send = 0x217;
	CAN_CMD_BASE_Referee_float(&hcan1,0x601,position_cnt_send,0,0,Ground_Robot_Position.standard_4_y);
	/* 0x020C */
	position_cnt_send = 0x20C;
	CAN_CMD_BASE_Referee_8bit(&hcan1,0x601,position_cnt_send,Radar_Mark_Data.mark_progress,0,0,0,0,0);
	vTaskDelay(1);
}

void refree_to_upboard_10HZ(void)
{
	/* 0x0208 */
	position_cnt_send = 208;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x601,position_cnt_send,Projectile_Allowance.projectile_allowance_17mm,Projectile_Allowance.remaining_gold_coin,0);	
	
	/* 0x0201 */
	position_cnt_send = 201;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x603,position_cnt_send,Robot_Status.power_management_shooter_output,Robot_Status.shooter_barrel_heat_limit,Robot_Status.robot_id);	
	vTaskDelay(1);
	position_cnt_send = 211;
	CAN_CMD_BASE_Referee_8bit(&hcan1,0x603,position_cnt_send,Robot_Status.power_management_chassis_output,Robot_Status.power_management_gimbal_output,Robot_Status.power_management_shooter_output,0,0,0);
	
	/* 0x0202&0x0303 */
	position_cnt_send = 202;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x603,position_cnt_send,Power_Heat_Data.shooter_17mm_1_barrel_heat,Power_Heat_Data.buffer_energy,(uint16_t)(Map_Command.cmd_keyboard << 8 | 0));	
	vTaskDelay(1);
	
	/* 0x0303 */
	position_cnt_send = 303;
	CAN_CMD_BASE_Referee_32bit(&hcan1,0x603,position_cnt_send,0,Map_Command.target_position_x);
	position_cnt_send = 304;
	CAN_CMD_BASE_Referee_32bit(&hcan1,0x603,position_cnt_send,0,Map_Command.target_position_y);
	vTaskDelay(1);
	
}

void refree_to_upboard_3fenzhi1HZ(void)
{
	/* 0x0003 */
	position_cnt_send = 3;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x602,position_cnt_send,Game_Robot_HP.blue_1_robot_HP,Game_Robot_HP.blue_2_robot_HP,Game_Robot_HP.blue_3_robot_HP);
	position_cnt_send = 4;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x602,position_cnt_send,Game_Robot_HP.blue_4_robot_HP,Game_Robot_HP.blue_7_robot_HP,Game_Robot_HP.blue_base_HP);
	vTaskDelay(1);
	
	position_cnt_send = 5;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x602,position_cnt_send,Game_Robot_HP.blue_outpost_HP,Game_Robot_HP.red_1_robot_HP,Game_Robot_HP.red_2_robot_HP);
	position_cnt_send = 6;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x602,position_cnt_send,Game_Robot_HP.red_3_robot_HP,Game_Robot_HP.red_4_robot_HP,Game_Robot_HP.red_7_robot_HP);
	position_cnt_send = 7;
	vTaskDelay(1);
	
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x602,position_cnt_send,Game_Robot_HP.red_base_HP,Game_Robot_HP.red_outpost_HP,0);
	/* 0x0209 */
	position_cnt_send = 209;
	CAN_CMD_BASE_Referee_32bit(&hcan1,0x602,position_cnt_send,0,RFID_Status.rfid_status);
	vTaskDelay(1);

}

void refree_to_upboard_3HZ(void)
{
	/* 0x0204 */
	position_cnt_send = 204;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x604,position_cnt_send,(uint16_t)(Buff_Energy.remaining_energy << 8 | Buff_Energy.defence_buff),0,0);
	
	/* 0x0301的一部分 */
//	position_cnt_send = 301;
//	CAN_CMD_BASE_Referee_16bit(&hcan1,0x604,position_cnt_send,Robot_Interaction_User_Data.alliance[0].x,Robot_Interaction_User_Data.alliance[0].y,Robot_Interaction_User_Data.alliance[1].x);
//	vTaskDelay(1);
//	
//	position_cnt_send = 302;	
//	CAN_CMD_BASE_Referee_16bit(&hcan1,0x604,position_cnt_send,Robot_Interaction_User_Data.alliance[1].y,Robot_Interaction_User_Data.alliance[2].x,Robot_Interaction_User_Data.alliance[2].y);

//	position_cnt_send = 303;	
//	CAN_CMD_BASE_Referee_16bit(&hcan1,0x604,position_cnt_send,Robot_Interaction_User_Data.alliance[3].x,Robot_Interaction_User_Data.alliance[3].y,Robot_Interaction_User_Data.alliance[4].x);
//	vTaskDelay(1);
//	
//	position_cnt_send = 304;	
//	CAN_CMD_BASE_Referee_16bit(&hcan1,0x604,position_cnt_send,Robot_Interaction_User_Data.alliance[4].y,Robot_Interaction_User_Data.alliance[5].x,Robot_Interaction_User_Data.alliance[5].y);

	position_cnt_send = 305;
	CAN_CMD_BASE_Referee_8bit(&hcan1,0x604,position_cnt_send,Robot_Interaction_User_Data.enemy[0].z,Robot_Interaction_User_Data.enemy[1].z,Robot_Interaction_User_Data.enemy[2].z,Robot_Interaction_User_Data.enemy[3].z,Robot_Interaction_User_Data.enemy[4].z,Robot_Interaction_User_Data.enemy[5].z);
	vTaskDelay(1);
	
	position_cnt_send = 306;
	CAN_CMD_BASE_Referee_8bit(&hcan1,0x604,position_cnt_send,Robot_Interaction_User_Data.alliance[0].z,Robot_Interaction_User_Data.alliance[1].z,Robot_Interaction_User_Data.alliance[2].z,Robot_Interaction_User_Data.alliance[3].z,Robot_Interaction_User_Data.alliance[5].z,Robot_Interaction_User_Data.radar_enhance);
	vTaskDelay(1);
}

void refree_to_upboard_random(void)
{
	/* 0x0206 */
	position_cnt_send = 206;
	CAN_CMD_BASE_Referee_16bit(&hcan1,0x603,position_cnt_send,Hurt_Data.armor_id,Hurt_Data.HP_deduction_reason,0);
}

/*******************  NX需要的USB数据  ********************/

void enemy_state_data_update()
{
	enemy_state_data[0].id = Robot_ID_Red_Hero;   enemy_state_data[0].x = Robot_Interaction_User_Data.enemy[0].x;    enemy_state_data[0].y = Robot_Interaction_User_Data.enemy[0].y;     
	enemy_state_data[1].id = Robot_ID_Red_Engineer;   enemy_state_data[1].x = Robot_Interaction_User_Data.enemy[1].x;    enemy_state_data[1].y = Robot_Interaction_User_Data.enemy[1].y;  
	enemy_state_data[2].id = Robot_ID_Red_Infantry3;   enemy_state_data[2].x = Robot_Interaction_User_Data.enemy[2].x;    enemy_state_data[2].y = Robot_Interaction_User_Data.enemy[2].y; 
	enemy_state_data[3].id = Robot_ID_Red_Infantry4;   enemy_state_data[3].x = Robot_Interaction_User_Data.enemy[3].x;    enemy_state_data[3].y = Robot_Interaction_User_Data.enemy[3].y; 
	enemy_state_data[4].id = Robot_ID_Red_Infantry5;   enemy_state_data[4].x = Robot_Interaction_User_Data.enemy[4].x;    enemy_state_data[4].y = Robot_Interaction_User_Data.enemy[4].y; 
	enemy_state_data[5].id = Robot_ID_Red_Sentry;   enemy_state_data[5].x = Robot_Interaction_User_Data.enemy[5].x;    enemy_state_data[5].y = Robot_Interaction_User_Data.enemy[5].y;    
	if(Robot_Status.robot_id>100)   //大于100是蓝方
	{
		enemy_state_data[0].hp = Game_Robot_HP.red_1_robot_HP;
		enemy_state_data[1].hp = Game_Robot_HP.red_2_robot_HP;
		enemy_state_data[2].hp = Game_Robot_HP.red_3_robot_HP;
		enemy_state_data[3].hp = Game_Robot_HP.red_4_robot_HP;
		enemy_state_data[4].hp = 0;
		enemy_state_data[5].hp = Game_Robot_HP.red_7_robot_HP;
	}
	else
	{
		enemy_state_data[0].hp = Game_Robot_HP.blue_1_robot_HP;
		enemy_state_data[1].hp = Game_Robot_HP.blue_2_robot_HP;
		enemy_state_data[2].hp = Game_Robot_HP.blue_3_robot_HP;
		enemy_state_data[3].hp = Game_Robot_HP.blue_4_robot_HP;
		enemy_state_data[4].hp = 0;
		enemy_state_data[5].hp = Game_Robot_HP.blue_7_robot_HP;
	}
}

void refree_to_upboard_enemy_state(void)
{	
	for(uint8_t enemy_state_cnt = 0;enemy_state_cnt <6;enemy_state_cnt ++)
	{
		CAN_CMD_BASE(&hcan1,ENEMT_STATE_ID,(uint16_t)(enemy_state_cnt <<8 | 0x0000),enemy_state_data[enemy_state_cnt].x,enemy_state_data[enemy_state_cnt].y,enemy_state_data[enemy_state_cnt].hp);
		if(enemy_state_cnt%2 == 0) vTaskDelay(1);
	}
}

void refree_to_upboard_Robot_Pos(void)
{
	/* 0x0203 */
	vTaskDelay(1);
	CAN_CMD_f32(&hcan1,MY_POSITION1_ID,Robot_Pos.x,Robot_Pos.y);
	CAN_CMD_f32(&hcan1,MY_POSITION2_ID,Robot_Pos.angle,0);

}

/*********************************************************/

void referee_upboard_task(void const * argument)
{
	while(1)
	{
		for(uint8_t i=0;i<6;i++)
		{
			Robot_Interaction_User_Data.enemy[i].x = enemy_state_data[i].x;
			Robot_Interaction_User_Data.enemy[i].y = enemy_state_data[i].y;
		}
   		//CAN_CMD_BASE_Referee_32bit(&hcan1,UP_TO_DOWN_ID1,0,0,Sentry_Decision_0x0120.decision);
		vTaskDelay(1);
	}
}

