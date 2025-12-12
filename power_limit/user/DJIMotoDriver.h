#ifndef __DJIMOTODRIVER_H__
#define __DJIMOTODRIVER_H__
#include "main.h"
#include "can.h"
#include "stdbool.h"
#include "math.h"
#include "ina226.h"
#include "struct_typedef.h"

#define DJI_MOTO_CAN_HANDEL hcan1

// 大疆电机ID
typedef enum
{
	CAN_Motor_ALL_ID = 0x200,
	CAN_Motor1_ID = 0x201,
	CAN_Motor2_ID = 0x202,
	CAN_Motor3_ID = 0x203,
	CAN_Motor4_ID = 0x204,
	CAN_Motor5_ID = 0x205,
	CAN_Motor6_ID = 0x206,
	CAN_Motor7_ID = 0x207,
	CAN_Motor8_ID = 0x208,
} DJI_CAN_ID;


// 大疆电机状态
typedef struct
{
	// 电机反馈数据(真实值)
	int16_t speed;//(换算过减速比)
	uint16_t encoder_angle;
	int16_t current;//融入低通滤波，换算到成ma
	int8_t temperature;//温度
	// 上一次
	int16_t speed_last;
	uint16_t angle_last; 
	// 控制数据(目标值)
	int16_t current_desired;
	int16_t speed_desired;
	int32_t angle_desired;
    fp32 power_set;
	// 处理后数据
	int16_t turns;				 // 转过的圈数
	int32_t totol_encoder_angle; // 套圈后总角度	（编码器值）						
	float totol_float_angle;    //总角度（度）

	// 第一次反馈数据记录
	int16_t original_position;
	uint16_t encoder_angle_first;
	bool first_run;
    
    //功率控制
    double k[6];
    float pre_power;//预测功率
    float real_power;//真实功率
    float v_eat;

} DJIMotoStateTD;

void DJIMoto_State_Init(DJIMotoStateTD *motostate,double k[6]);

void SetMoto_Current(int16_t C1, int16_t C2, int16_t C3, int16_t C4);

// can接收回调函数用
void DJI_SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo,DJIMotoStateTD *motostate);

void Update_MotoAngle(DJIMotoStateTD *MotoState,float ratio);
void Update_MotoState(DJIMotoStateTD *MotoState,float ratio);
void Update_Motopower(DJIMotoStateTD *MotoState);
#endif
