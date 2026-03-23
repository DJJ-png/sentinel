#include "bsp_can.h"
#include "main.h"
#include "bsp_cap.h"
#include "Referee.h"
#include "robot_message.h"

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

/*can数据发送函数*/
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


/*dji电机*/
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

void CAN_cmd_LK_init(CAN_HandleTypeDef* hcan,uint8_t uid)
{
	
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message = {0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = 0x140+uid;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] =0x88; 
	for(int i=1;i<8;i++)
    {
        chassis_can_send_data[i]=0x00;
    }
	
	/*	yaw: can1接瓴控电机	*/
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/* LK电机单电机发送 */
void CAN_cmd_LK_Motor(CAN_HandleTypeDef* hcan,uint8_t uid, int16_t current)
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
    // 1. 定义滤波器配置结构体
    CAN_FilterTypeDef can_filter_st;

    // ==================== 配置 CAN1 滤波器 ====================
    can_filter_st.FilterActivation = ENABLE;       // 使能滤波器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; // 模式：掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; // 位宽：32位
    can_filter_st.FilterIdHigh = 0x0000;     // ID 高16位
    can_filter_st.FilterIdLow = 0x0000;      // ID 低16位
    can_filter_st.FilterMaskIdHigh = 0x0000;// 掩码高16位
    can_filter_st.FilterMaskIdLow = 0x0000; // 掩码低16位
    can_filter_st.FilterBank = 0;           // 滤波器组编号：CAN1用0~13
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; // 收到数据放进FIFO0

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);  // 把配置写入CAN1
    HAL_CAN_Start(&hcan1);                         // 启动CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启接收中断

    // ==================== 配置 CAN2 滤波器 ====================
    can_filter_st.SlaveStartFilterBank = 14; // CAN2 从第14组滤波器开始使用
    can_filter_st.FilterBank = 14;           // CAN2 使用第14组滤波器

    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);  // 把配置写入CAN2
    HAL_CAN_Start(&hcan2);                         // 启动CAN2
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启接收中断
}



