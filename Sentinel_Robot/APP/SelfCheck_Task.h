//V1.4版本。

#ifndef _SELFCHECK_TASK
#define _SELFCHECK_TASK

#include "main.h"
#include "tim.h"

#define DEVICE_TIMEOUT_LIMIT		10  		//外设通信超时计数值

//设置C板LED的状态（宏版本）
#define __LED_R_SET(s)	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, (s) ? 65535 : 0)
#define __LED_G_SET(s)	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, (s) ? 65535 : 0)
#define __LED_B_SET(s)	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, (s) ? 65535 : 0)

//读取外设状态（宏版本）
#define __DEVICE_STATE_READ(id)		((device_state_flag & (1 << (id))) ? DEVICE_NORMAL : DEVICE_DISORDER)

//条件编译：限制DEVICE_TIMEOUT_LIMIT的取值范围，超出范围会强制报错
#if DEVICE_TIMEOUT_LIMIT > 50
	#error "DEVICE_TIMEOUT_LIMIT is too big!"
#elif DEVICE_TIMEOUT_LIMIT < 3
	#error "DEVICE_TIMEOUT_LIMIT is too small!"
#endif

//外设编号名称格式：通信方式_设备类型(_所属机构)；全大写
//注意：其中的编号数值与该外设在状态变量中所占的标志位一一对应，务必严格确保其数值的准确性！
typedef enum
{
		USB_NUC										= 0,	//NUC，USB通信方式
		UART_RC										= 1,	//遥控器接收机，DBUS(UART)通信方式
		UART_IMAGE								= 2,	//图传链路，UART通信方式
		CAN_MCU_DOWN							= 3,	//底盘C板，CAN通信方式
		CAN_MG5010_GIMBAL_YAW			= 4,	//云台MG5010电机，CAN通信方式
		CAN_GM6020_GIMBAL_PITCH		= 5,	//云台GM6020电机，CAN通信方式
		CAN_M2006_SHOOT_DIAL			= 6,	//发射机构M2006电机，CAN通信方式
		CAN_M3508_SHOOT_FRIC_1		= 7,	//发射机构M3508电机，CAN通信方式
		CAN_M3508_SHOOT_FRIC_2		= 8,	//发射机构M3508电机，CAN通信方式
}device_id;

//外设状态
typedef enum{
		DEVICE_DISORDER			= 0,	//设备（通信）异常
		DEVICE_NORMAL				= 1,	//设备（通信）正常
}device_state;

extern uint16_t device_state_flag;
extern uint8_t lost_count[16];
extern float voltage;
extern uint32_t buzzer_cnt;

void SelfCheck_Task(void const* argument);
device_state Device_State_Read(device_id id);
void Device_State_Update(void);
void LED_State_Update(void);
void Low_Battery_Voltage_Warning(void);

#endif
