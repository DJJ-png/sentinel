/**
  *****************************************************************************
  * @file       Nmanifold_usart_task.c/h
  * @brief      MANIFOLD data solve. MANIFOLD数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0			Mar-3-2023			圣灵亮欣					1.Done
  *  V2.1.0			May-16-2023			圣灵亮欣					1.Add data transmission for dial switch
  @verbatim
  ==============================================================================
	
  ==============================================================================
  @endverbatim
  *****************************************************************************
*/
#ifndef NMANIFOLD_USART_TASK_H
#define NMANIFOLD_USART_TASK_H
#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"

#include "bsp_usart.h"

#include "INS_task.h"
#include "arm_math.h"
#include "referee.h"

#define NUC_DATA &p_unpack[7]



/****************************************常量定义段********************************************/
#define USART_RX_BUF_LENGHT		256
//命令字ID
#define CMD_ID_AUTOAIM_DATA_RX	 		0x81			//接收自瞄数据（上→下）
#define CMD_ID_AUTOAIM_DATA_TX	 		0x14			//发送自瞄数据（下→上）
#define CMD_ID_WORKING_MODE			 		0x15			//操作模式（下→上）
#define CMD_ID_DIAL_SWITCH			 		0x17			//拨码开关（下→上）
/*烧饼控制宏*/
#define CMD_ID_CHASSIS_DATA_RX     0x82			//底盘控制数据(UP-DOWN)    （哨兵专用）
#define CMD_ID_SENTINEL_SCAN        0x83
#define CMD_ID_ROTATE_DATA_RX       0x85      //接收小陀螺速度（UP-DOWN）（哨兵专用）
#define CMD_ID_ROBOT_DATA             0x18      //发送比赛各兵种血量(用于结算
//#define CMD_ID_CHASSIS_DATA_TX	 		0x12      //上传底盘轮速计数据（下->上）（哨兵专用）
//#define CMD_ID_CHASSIA_GIMBAL_ANGLE 0X16      //发送底盘相对云台角度（下->上）（哨兵专用）



//数据段长度，以字节为单位
#define LENGTH_AUTOAIM_DATA_RX		18			//接收自瞄数据（上→下）
#define LENGTH_AUTOAIM_DATA_TX		13			//发送自瞄数据（下→上）
#define LENGTH_WORKING_MODE			 1				//操作模式（下→上）
#define LENGTH_DIAL_SWITCH				 4				//拨码开关（下→上）
/*烧饼控制宏*/
#define LENGTH_CHASSIS_DATA_RX    6       //接收底盘控制数据（上->下）（哨兵专用）
#define LENGTH_ROTATE_DATA_RX     2       //接收小陀螺速度（上->下）（哨兵专用）
////#define LENGTH_CHASSIS_DATA_TX    24      //上传底盘轮速计数据（下->上）（哨兵专用）
//#define LENGTH_CHASSIS_GIMBAL_ANGLE 4     //发送底盘相对云台角度（下->上）（哨兵专用）

//自瞄模式
#define AUTOAIM_MODE_NORMAL						0x00		//自瞄模式：普通
#define AUTOAIM_MODE_SMALL_ENERGY			0x01		//自瞄模式：小能量机关
#define AUTOAIM_MODE_BIG_ENERGY				0x02		//自瞄模式：大能量机关
#define AUTOAIM_MODE_ANTI_TOP					0x03		//自瞄模式：反小陀螺

//自瞄装甲
#define AUTOAIM_ARMOR_AUTO					0x00		//自瞄装甲：自动
#define AUTOAIM_ARMOR_SMALL					0x04		//自瞄装甲：小装甲模块
#define AUTOAIM_ARMOR_BIG						0x08		//自瞄装甲：大装甲模块

//拨码开关
#define SWITCH_OFF					1				//不启用拨码开关
#define SWITCH_ON						2				//启用拨码开关
#define ENEMY_INFANTRY_ARMOR_SMALL	0				//敌方步兵装甲：小装甲
#define ENEMY_INFANTRY_ARMOR_BIG		1				//敌方步兵装甲：大装甲
/*******************************************END**********************************************/
typedef struct
{
	float vx,vy,wz;
}chassis_helm_t;

typedef struct __attribute__((packed)) 
{
		uint8_t radar_enhance;
		uint16_t robot_x[5];
		uint16_t robot_y[5];
		uint8_t robot_z[5];
} radar_data_t;


typedef struct __attribute__((packed)) 
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

typedef struct __attribute__((packed))
{
	uint8_t game_progress;
	uint16_t stage_remain_time;
	
} NUC_Game_Status_0001_t;//比赛进程（3byte）

typedef struct __attribute__((packed))
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} NUC_Game_Robot_HP_0003_t;//各机器人血量（28byte）

typedef struct __attribute__((packed))
{
	uint8_t defence_buff;
	uint8_t remaining_energy;
}buff_0204_t;					 //2byte

typedef struct __attribute__((packed))
{
	uint16_t projectile_allowance_17mm;
	uint16_t remaining_gold_coin;
} NUC_Projectile_Allowance_0208_t;//4byte

typedef struct __attribute__((packed))
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
}ground_robot_position_020B_t;		//32byte

typedef struct __attribute__((packed))
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
}sentry_info_020D_t;				//6byte

typedef struct __attribute__((packed))
{
	radar_data_t	radar_data;
} robot_interaction_data_t;			//31byte
typedef struct __attribute__((packed))
{
	uint32_t target_position_x;
	uint32_t target_position_y;
	uint8_t cmd_keyboard;
}map_command_0303_t;				//9byte

typedef struct __attribute__((packed))
{
	fp32 angle_between_gimbal_and_chassis;
 	fp32 advanced_yaw_angle;
	
} NUC_Parameter_Of_Car_0601_t;//4byte




/*************************************发送结构体定义段****************************************/
typedef struct{
	uint8_t  SOF;			//帧头
	uint16_t data_length;			//帧长（包含帧头与校验位）
	uint8_t  seq;			//命令字
	uint8_t  CRC8;
}__attribute__((__packed__)) Protocol_Head_Data;//通信协议数据流前段数据


typedef struct{
	uint8_t switch_off;				//拨码开关是否启用（2：启用 ；1：不启用）
	uint8_t Infantry_Armor[3];	//依次为敌方3、4、5号步兵的装甲类型（0:小装甲；1:大装甲）
}__attribute__((__packed__)) Dial_Switch_Data;//拨码开关

typedef struct __attribute__((packed))
{
	NUC_Game_Robot_HP_0003_t 			NUC_Game_Robot_HP_0003;					//28byte	3Hz
	
	ext_hurt_data_t	     			    Hurt_Data; //1Byte
	
	NUC_Projectile_Allowance_0208_t 	NUC_Projectile_Allowance_0208;			//4byte		10Hz
	ext_rfid_status_t      				RFID_Status_0209;						//4byte		3Hz
		
	NUC_Parameter_Of_Car_0601_t 		NUC_Parameter_Of_Car_0601;				//8byte	
} NUC_Used_Message_200Hz_0606_t;

typedef struct __attribute__((packed))
{
	NUC_Game_Status_0001_t 				NUC_Game_Status_0001;					//3byte		1Hz
	ext_event_data_t                	Event_Data_0101;						//4byte		1Hz
	
	ext_dart_info_t						dart_info_0105;							//2byte		1Hz
	ext_robot_pos_t					    robot_pos_0203;							//12byte	1Hz
	buff_0204_t							buff_0204;								//2byte		3Hz
	ground_robot_position_020B_t		ground_robot_position_020B;				//32byte	1Hz
	ext_sentry_info_t				    sentry_info_020D;						//6byte		1Hz
	
	robot_interaction_data_t			Robot_Interaction_User_Data_0301;		//26byte	1Hz
	map_command_0303_t 					map_command_0303;						//9byte		
		
} NUC_Used_Message_1Hz_0607_t;
/*******************************************END**********************************************/




/*************************************接收结构体定义段****************************************/
typedef struct __attribute__((packed)) 
{
	struct __attribute__((packed))//0x0501
	{
		fp32 vx;
		fp32 vy;
		fp32 wz;
	}chassis_v;
	
	struct __attribute__((packed))//0506
	{
		uint8_t robt[11];
	}list;
	
		struct __attribute__((packed))//0502
	{
		fp32 max_pitch;
		fp32 min_pitch;
		uint8_t mode;//0：导航控底盘1：导航控云台
		uint8_t spin;//0：不小陀螺 1：小陀螺
		uint8_t patrol;//0：云台不巡逻 1：云台巡逻 nuc_control.action.patrol
		uint8_t robot_aim;//0:不自瞄 1:自瞄 nuc_control.action.robot_aim
		uint8_t chasing;//0:不在追击状态 1：追击状态（边自瞄边导航）
	}action;
	
} nuc_control_t;
/*******************************************END**********************************************/




extern nuc_control_t nuc_control;
extern void referee_copy(uint16_t cmid,uint16_t length,uint8_t* data,uint8_t* uart_send);

extern custom_info_t custom_info_0x0308;



//对外接口
extern Dial_Switch_Data Dial_Switch;


//函数声明
void manifold_usart_task(void );
void USART1_IRQHandler_1(void);
uint8_t NUC_Data_Unpack(void);
void NUC_Usart_Tx_200Hz();
void NUC_Usart_Tx_1Hz();
extern uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len);

//8位版本CRC表
#endif
