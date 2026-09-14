#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/****************************************电机参数配置********************************************/
#define GIMBAL_MOTOR_BASE_ECD_RANGE 65535
#define GIMBAL_MOTOR_ADVANCED_ECD_RANGE 8191
#define GIMBAL_MOTOR_PITCH_ECD_RANGE 65535
/****************************************零点配置********************************************/
#define CHASSIS_FOLLOW_BASE_GIMBAL_ANGLE_ZERO 11188
#define ADVANCED_GIMBAL_ANGLE_ZERO 2751 
#define PITCH_ANGLE_ZERO 34579
//四连杆解算相关零点
#define PITCH_ENC_MIN         6850
#define PITCH_ANGLE_MIN       -30.0f
#define PITCH_ANGLE_MAX       -10.0f
#define PITCH_ENC_AT_MAX      65535-58200
// 计算得出的常量
#define PITCH_ECD_TOTAL       14185
#define PITCH_ANGLE_TOTAL     70.0f

/* 小YAW阈值  */
#define ADVANCED_YAW_ANFLE_MAX 65
#define ADVANCED_YAW_ANFLE_MIN -65
/****************************************阈值配置********************************************/

#define DEFAULT_YAW_SPIN_SPEED      -200

/* ************************************* 遥控器宏 ******************************************** */
#define RollWheel							rc_ctrl.rc.ch[4]			//遥控器左侧滚轮，左→右：660→-660
#define RockingBar_Left_V			rc_ctrl.rc.ch[3]			//遥控器左摇杆（垂直），上→下：660→-660
#define RockingBar_Left_H			rc_ctrl.rc.ch[2]			//遥控器左摇杆（水平），左→右：-660→660
#define RockingBar_Right_V		rc_ctrl.rc.ch[1]			//遥控器右摇杆（垂直），上→下：660→-660
#define RockingBar_Right_H		rc_ctrl.rc.ch[0]			//遥控器右摇杆（水平），左→右：-660→660
#define Switch_Left						rc_ctrl.rc.s[1]				//遥控器左拨杆，上1，中3，下2
#define Switch_Right					rc_ctrl.rc.s[0]				//遥控器右拨杆，上1，中3，下2
/****************************************pid配置********************************************/
//base yaw speed
#define BASE_YAW_MOTOR_SPEED_PID_KP 8.2f
#define BASE_YAW_MOTOR_SPEED_PID_KI 0.1f
#define BASE_YAW_MOTOR_SPEED_PID_KD 0.0f
#define BASE_YAW_MOTOR_SPEED_PID_MAX_OUT 1800.0f
#define BASE_YAW_MOTOR_SPEED_PID_MAX_IOUT 110.0f
//base yaw angle
#define BASE_YAW_MOTOR_ANGLE_PID_KP 20.2f
#define BASE_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define BASE_YAW_MOTOR_ANGLE_PID_KD 40.0f
#define BASE_YAW_MOTOR_ANGLE_PID_MAX_OUT 1500.0f
#define BASE_YAW_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

//base yaw stop speed 
#define BASE_YAW_MOTOR_SPEED_STOP_PID_KP 8.2f
#define BASE_YAW_MOTOR_SPEED_STOP_PID_KI 0.1f
#define BASE_YAW_MOTOR_SPEED_STOP_PID_KD 0.0f
#define BASE_YAW_MOTOR_SPEED_STOP_PID_MAX_OUT 1800.0f
#define BASE_YAW_MOTOR_SPEED_STOP_PID_MAX_IOUT 110.0f
//base yaw stop angle
#define BASE_YAW_MOTOR_ANGLE_STOP_PID_KP 20.2f
#define BASE_YAW_MOTOR_ANGLE_STOP_PID_KI 0.0f
#define BASE_YAW_MOTOR_ANGLE_STOP_PID_KD 40.0f
#define BASE_YAW_MOTOR_ANGLE_STOP_PID_MAX_OUT 1500.0f
#define BASE_YAW_MOTOR_ANGLE_STOP_PID_MAX_IOUT 0.0f


//advanced yaw speed
#define ADVANCED_YAW_MOTOR_SPEED_PID_KP 175.0f//300.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_KI 0.2f//0.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_KD 200.0f//100.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_MAX_OUT 15000.0f//15000.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_MAX_IOUT 8000.0f//8000.0f

//advanced yaw angle
#define ADVANCED_YAW_MOTOR_ANGLE_PID_KP 8.0f//9.5f//8.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_KI 0.00f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_KD 200.0f//200.0f//1.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_OUT 500.0f//500.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

//pitch yaw speed
#define PITCH_MOTOR_SPEED_PID_KP 5.35f//2.5f//8.5f			
#define PITCH_MOTOR_SPEED_PID_KI 0.00875f//0.06f//0.026f				
#define PITCH_MOTOR_SPEED_PID_KD 0.1f//50.0f				 
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 1500.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 130.0f

//pitch yaw angle
#define PITCH_MOTOR_ANGLE_PID_KP 4.0f//5.0f//5.0f			 
#define PITCH_MOTOR_ANGLE_PID_KI 0.001f//0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 0.011//10.0f   	
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 700.0f	
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 0.0f



/*aim_pid*/
//base yaw aim_speed
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KP 7.0//10.2;//8.2f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KI 0.1f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KD 0.0f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT 1800.0f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT 110.0f

//base yaw aim_angle
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KP 10.2f//10.2f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KI 0.05f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KD 70.0f//70.0f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT 1500.0f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT 0.0f

//advanced yaw_aim_speed
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KP 160.0f//450.0f//160.0f //50.0f//100
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KI 0.02f   //0.35//0
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KD 50.0f//500.0f
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT 16384.0f  //16384
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT 1300.0f

//advanced yaw_aim_angle
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KP 0.75f//8.0f//20
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KI 0.03f
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KD 500.0f//1500
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT 320.0f//320
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT 200.0f

//pitch _aim_speed
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_KP 4.25f//2.5f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_KI 0.016f//0.06f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_KD 0.12f//50.0f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT 2500.0f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT 130.0f//100.0f

//pitch _aim_angle
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KP 4.0//5.2f   
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KI 0.0f//0.0f
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KD 0.011f//20.0f    
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT 2500.0f
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT 0.0f
/****************************************状态枚举********************************************/
enum GIMBAL_JOINT{
    BASE_YAW,
    PITCH,
    ADVANCED_YAW,
};

enum MOTOR_MODE{
    SPEED,
    ANGLE,
};

enum GIMBAL_MOTOR{
    BASE_YAW_5010,
    PITCH_6015,
    ADVANCED_YAW_6020,
};

enum GIMBAL_MODE_CNOTROL {
    BASE_YAW_MODE,
		ADVANCED_YAW_MODE,
		GIMBAL_TO_CHASSIS_MODE,
};
enum NUC_CONTROL_MODE{
    NUC_CONTROL_CHASSIS_MOVE,
    NUC_CONTROL_GIMBAL_MOVE,
};
/****************************************can通讯********************************************/
//电机ID
#define CAN_LK_BASE_YAW_ID 0x141
#define CAN_ADVANCED_YAW_ID 0x207
#define CAN_LK_PITCH_ID 0x142 
#define CAN_DIAL_MOTOR_ID 0x201
#define CAN_FRIC_MOTOR1_ID 0x203
#define CAN_FRIC_MOTOR2_ID 0x204
#define CAN_IMU_ID2 0x012
#define CAN_IMU_ID1 0x011
//can数据通讯ID
#define CAN_COMMUNITE_ID_1 0x301
#define CAN_COMMUNITE_ID_2 0x302
#define CAN_COMMUNITE    hcan1

#define CAN_ID_GAME_ENV_POS       0x601  // 游戏、环境、敌我位置
#define CAN_ID_GLOBAL_HP_RFID     0x602  // 全场血量、RFID
#define CAN_ID_SELF_STATUS_MAP    0x603  // 自身状态、伤害、地图目标
#define CAN_ID_BUFF_INTERACT      0x604  // 增益、小地图交互数据
#define CAN_ID_MY_POSITION1       0x30D
#define CAN_ID_MY_POSITION2       0x30E
#define CAN_ID_ENEMT_STATE        0x30F
/* --- Sub-ID: 0x601 (Game & Environment) --- */
#define CMD_601_GAME_STATUS       1//比赛进程
#define CMD_601_EVENT_DATA        101//比赛事件
#define CMD_601_PROJECTILE_ALLOW  208//允许发弹量
#define CMD_601_SENTRY_INFO       0x20D//哨兵自主决策
#define CMD_601_POS_HERO_X        0x210
#define CMD_601_POS_HERO_Y        0x211
#define CMD_601_POS_ENGINEER_X    0x212
#define CMD_601_POS_ENGINEER_Y    0x213
#define CMD_601_POS_STND_3_X      0x214
#define CMD_601_POS_STND_3_Y      0x215
#define CMD_601_POS_STND_4_X      0x216
#define CMD_601_POS_STND_4_Y      0x217
#define CMD_601_RADAR_MARK        0x20C//雷达标记进度数据
/* --- Sub-ID: 0x602 (HP & RFID) --- */
#define CMD_602_HP_ALLY_123       3
#define CMD_602_HP_ALLY_47_BASE   4
#define CMD_602_HP_ALLY_BASE_OUT  5
#define CMD_602_RFID_STATUS       209


/* --- Sub-ID: 0x603 (Self Status & Map) --- */
#define CMD_603_ROBOT_STATUS      201//
#define CMD_603_POWER_HEAT        202//发射机构热量
#define CMD_603_HURT_DATA         206//伤害状态
#define CMD_603_POWER_OUTPUT      211//
#define CMD_603_MAP_TARGET_X      303
#define CMD_603_MAP_TARGET_Y      304

/* --- Sub-ID: 0x604 (Buff & Interaction) --- */
#define CMD_604_BUFF_ENERGY       204//缓冲能量
#define CMD_604_ENEMY_Z           305
#define CMD_604_ALLIANCE_Z        306
#endif 