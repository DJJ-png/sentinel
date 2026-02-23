#ifndef _MY_DEF_
#define _MY_DEF_

#include "main.h"
#include "remote_control.h"
#include "tim.h"

extern RC_ctrl_t rc_ctrl_last;

#define sin_40_15 					0.6445322245322245f
#define cos_40_15 					0.7650727650727651f

#define DEFAULT_YAW_SPIN_SPEED      -200
/* *************************************** 底盘相关变量 ****************************************** */

/* 舵轮偏置 轮子安装的位置不同则偏置不同 */
#define HELM_OFFSET_2 	4880
#define HELM_OFFSET_3  4600

/* 限制3508电机的输出 */
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 800.0f

/* 限制6020电机的输出 */
#define M6020_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 8000.0f

/* 设置底盘模式 */ 
#define CHASSIS_ENABLE 0x01 //0000 0001
#define CHASSIS_FLOWE_GIMBAL  0x02 //0000 0010

#define FOLLOW_CHASSIS_DIRECTION 0x04 //0000 0100
/* 一弧度约等于57度 */
#define PI_TO_180 57.2957779

/* rpm=v/(2Rpi) *60 *减速比 将底盘的速度转换成3508电机的转速*/
#define V_TO_RPM (60/(2*PI*65)*13.72)//mm-rmp

/* 底盘轮所在圆半径 */
#define RADIUS_CHASSIS 240.6f



/* ************************************** AUTO_AIM ******************************************* */



/* ************************************** LED ******************************************* */

//
#define LED_PIN_PORT		GPIOH	

#define LED_RED_PIN			GPIO_PIN_12		
#define LED_GREEN_PIN		GPIO_PIN_11		
#define LED_BLUE_PIN		GPIO_PIN_10		

/* ************************************* 运算宏 ******************************************** */

#define SIGNAL(n)										((n)>=0 ? 1 : -1)
#define ABS(n)											((n)>=0 ? (n) : -(n))
#define DATA_LIMIT(n,min,max)				((n)>(max) ? (max) : ((n)<(min) ? (min) : (n)))
#define DEADBAND(n,db)							(ABS(n) >= ABS(db) ? (n) : 0)
#define RAMP_CTRL(ref,set,acc)	((ref) + DATA_LIMIT((acc),0,1) * ((set) - (ref)))
#define LIMIT_TO_SET(data,set)  	(((data)>(set))?((data)-(set)*2):(((data)<-(set))?((data)+(set)*2):(data)))

/* ************************************* RM遥控器 ******************************************** */

/* 3000 表示机器人最大行驶速度，单位是 mm/s */
#define Sw_CV (3000.0f/660.0f)
#define Sw_Wz ( 320.0f/660.0f)
#define Sw_Pc (  90.0f/660.0f)
#define Rotor_speed (660*2.8f*1.5)//2.8

#define RollWheel							rc_ctrl.rc.ch[4]			//遥控器左侧滚轮，左→右：660→-660
#define RockingBar_Left_V			rc_ctrl.rc.ch[3]			//遥控器左摇杆（垂直），上→下：660→-660
#define RockingBar_Left_H			rc_ctrl.rc.ch[2]			//遥控器左摇杆（水平），左→右：-660→660
#define RockingBar_Right_V		rc_ctrl.rc.ch[1]			//遥控器右摇杆（垂直），上→下：660→-660
#define RockingBar_Right_H		rc_ctrl.rc.ch[0]			//遥控器右摇杆（水平），左→右：-660→660
#define Switch_Left						rc_ctrl.rc.s[1]				//遥控器左拨杆，上1，中3，下2
#define Switch_Right					rc_ctrl.rc.s[0]				//遥控器右拨杆，上1，中3，下2

/* ************************************* 客户端键鼠 ******************************************** */

#define Mouse_Left					(rc_ctrl.mouse.press_l)		//鼠标左键
#define Mouse_Right					(rc_ctrl.mouse.press_r)		//鼠标右键
#define MOUSE_X							rc_ctrl.mouse.x						//鼠标指针X
#define MOUSE_Y							rc_ctrl.mouse.y						//鼠标指针Y
#define MOUSE_Z							rc_ctrl.mouse.z						//鼠标指针Z
#define KEY_W								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)				//键盘按键W
#define KEY_S								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)				//键盘按键S
#define KEY_A								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)				//键盘按键A
#define KEY_D								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)				//键盘按键D
#define KEY_SHIFT						(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)		//键盘按键SHIFT
#define KEY_CTRL						(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)			//键盘按键CTRL
#define KEY_Q								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)				//键盘按键Q
#define KEY_E								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)				//键盘按键E
#define KEY_R								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)				//键盘按键R
#define KEY_F								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)				//键盘按键F
#define KEY_G								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)				//键盘按键G
#define KEY_Z								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)				//键盘按键Z
#define KEY_X								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)				//键盘按键X
#define KEY_C								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)				//键盘按键C
#define KEY_V								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)				//键盘按键V
#define KEY_B								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)				//键盘按键B
#define KEY_W_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_W)				//前一时刻键盘按键W
#define KEY_S_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_S)				//前一时刻键盘按键S
#define KEY_A_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_A)				//前一时刻键盘按键A
#define KEY_D_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_D)				//前一时刻键盘按键D
#define KEY_Q_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_Q)				//前一时刻键盘按键Q
#define KEY_E_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_E)				//前一时刻键盘按键E
#define KEY_R_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_R)				//前一时刻键盘按键R
#define KEY_F_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_F)				//前一时刻键盘按键F
#define KEY_G_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_G)				//前一时刻键盘按键G
#define KEY_Z_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_Z)				//前一时刻键盘按键Z
#define KEY_X_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_X)				//前一时刻键盘按键X
#define KEY_C_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_C)				//前一时刻键盘按键C
#define KEY_V_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_V)				//前一时刻键盘按键V
#define KEY_B_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_B)				//前一时刻键盘按键B

/* ************************************* 键盘功能 ******************************************** */

#define KEYB_TURN_BACK		(!KEY_Z_LAST && KEY_Z)									//调头
#define KEYB_AIM_MODE			(!KEY_Q_LAST && KEY_Q)									//自瞄模式
#define KEYB_AIM_ARMOR		(!KEY_F_LAST && KEY_F)									//自瞄装甲板
#define KEYB_BOX_LID			(!KEY_CTRL && KEY_R)										//弹舱盖
#define KEYB_FLASH_UI			(KEY_CTRL && KEY_R)											//刷新UI
#define KEYB_SPEED_GEAR		(!KEY_V_LAST && KEY_V)									//速度挡位
#define KEYB_FALL_RESUME	(!KEY_CTRL && KEY_B)										//起立(保护后)
#define KEYB_MCU_RESET		(KEY_CTRL && KEY_B)											//复位
#define KEYB_FLY					(KEY_X)																	//飞坡
#define KEYB_TALL_STAND		(!KEY_C_LAST && KEY_C)									//站立
#define KEYB_JUMP_OVER		(!KEY_G_LAST && KEY_G)									//跳

/* ************************************* 枚举 ******************************************** */

enum NUC_CONTROL_MODE{
    NUC_CONTROL_CHASSIS_MOVE,
    NUC_CONTROL_GIMBAL_MOVE,
};
enum MOTOR_MODE{
    SPEED,
    ANGLE,
};
enum GIMBAL_JOINT{
    BASE_YAW,
    PITCH,
    ADVANCED_YAW,
};
enum GIMBAL_MOTOR{
    BASE_YAW_5010,
    PITCH_6015,
    ADVANCED_YAW_6020,
};
enum SHOOT_MOTOR{
    SHOOT_DIAL,
};
enum NUC_CONTROL_LIST_ROBT {
    AUTO_AIM,
};
enum GIMBAL_MODE_CNOTROL {
    BASE_YAW_MODE,
		ADVANCED_YAW_MODE,
		GIMBAL_TO_CHASSIS_MODE,
};
#endif
