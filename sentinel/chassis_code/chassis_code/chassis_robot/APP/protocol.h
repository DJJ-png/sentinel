/**
  ******************************************************************************
  * @file    protocol.h
  * @author  圣灵亮欣
  * @version V1.7.0
  * @date    2025/01/08
  * @brief   依据裁判系统 串口协议附录 V1.7.0
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF                  0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
	GAME_STATUS_CMD_ID								= 0x0001,  //*比赛状态数据（服务器→全体机器人，常规链路）
	GAME_RESULT_CMD_ID								= 0x0002,  //比赛结果数据（服务器→全体机器人，常规链路）
	GAME_ROBOT_HP_CMD_ID							= 0x0003,  //*机器人血量数据（服务器→全体机器人，常规链路）
	
	EVENT_DATA_CMD_ID									= 0x0101,  //*场地事件数据（服务器→己方全体机器人，常规链路）
	REFEREE_WARNING_CMD_ID						= 0x0104,  //裁判警告信息（服务器→被判罚方全体机器人，常规链路）
	DART_INFO_CMD_ID									= 0x0105,  //*飞镖发射数据（服务器→己方全体机器人，常规链路）
	SENTRY_CMD_CMD_ID									= 0x0120,  //哨兵自主决策指令（哨兵机器人→服务器，常规链路）
	RADAR_CMD_CMD_ID									= 0x0121,  //雷达自主决策指令（雷达系统→服务器，常规链路）
	
	ROBOT_STATUS_CMD_ID								= 0x0201,  //机器人性能体系数据（主控模块→对应机器人，常规链路）
	POWER_HEAT_DATA_CMD_ID						= 0x0202,  //*实时底盘缓冲能量和射击热量（主控模块→对应机器人，常规链路）
	ROBOT_POS_CMD_ID									= 0x0203,  //机器人位置（主控模块→对应机器人，常规链路）
	BUFF_ENERGY_CMD_ID								= 0x0204,  //*机器人增益和底盘能量（服务器→对应机器人，常规链路）
	HURT_DATA_CMD_ID									= 0x0206,  //伤害状态（主控模块→对应机器人，常规链路）
	SHOOT_DATA_CMD_ID									= 0x0207,  //实时射击信息（主控模块→对应机器人，常规链路）
	PROJECTILE_ALLOWANCE_CMD_ID				= 0x0208,  //*允许发弹量（服务器→己方英雄、步兵、哨兵、空中机器人，常规链路）
	RFID_STATE_CMD_ID									= 0x0209,  //*机器人RFID状态（服务器→己方装有RFID的机器人，常规链路）
	DART_CLIENT_CMD_ID								= 0x020A,  //飞镖选手端指令数据（服务器→己方飞镖系统，常规链路）
	GROUND_ROBOT_POSITION_CMD_ID			= 0x020B,  //*地面机器人位置数据（服务器→己方哨兵机器人，常规链路）
	RADAR_MARK_DATA_CMD_ID						= 0x020C,  //雷达标记进度数据（服务器→己方雷达系统，常规链路）
	SENTRY_INFO_CMD_ID								= 0x020D,  //*哨兵自主决策信息（服务器→己方哨兵机器人，常规链路）
	RADAR_INFO_CMD_ID									= 0x020E,  //雷达自主决策信息（服务器→己方雷达系统，常规链路）
	
	ROBOT_INTERACTION_DATA_CMD_ID			= 0x0301,  //*机器人交互数据
	CUSTOM_ROBOT_DATA_CMD_ID					= 0x0302,  //自定义控制器与机器人交互数据（自定义控制器→机器人，图传链路）
  MAP_COMMAND_CMD_ID								= 0x0303,  //*选手端小地图交互数据（选手端→服务器→选择的己方机器人，常规链路）
  REMOTE_CONTROL_CMD_ID							= 0x0304,  //键鼠遥控数据（选手端→机器人，图传链路）
  MAP_ROBOT_DATA_CMD_ID							= 0x0305,  //选手端小地图接收雷达数据（雷达→服务器→己方所有选手端，常规链路）
  CUSTOM_CLIENT_DATA_CMD_ID					= 0x0306,  //自定义控制器与选手端交互数据（自定义控制器→选手端）
  MAP_DATA_CMD_ID										= 0x0307,  //-选手端小地图接收哨兵数据（哨兵/半自动机器人→对应操作手选手端，常规链路）
  CUSTOM_INFO_CMD_ID								= 0x0308,  //-选手端小地图接收机器人数据（己方机器人→己方选手端，常规链路）
  ROBOT_CUSTOM_DATA_CMD_ID					= 0x0309,  //自定义控制器接收机器人数据（己方机器人→自定义控制器，图传链路）
	//IDCustomData,
}referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct __attribute__((packed))
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef struct __attribute__((packed))
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
