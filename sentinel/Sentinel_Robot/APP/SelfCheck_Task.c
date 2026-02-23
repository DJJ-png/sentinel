//V1.4版本。

#include "SelfCheck_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "Referee.h"

//device_state_flag为16位整型，其中的各个二进制位均表示对应外设的状态，1表示通信正常，0表示通信异常
uint16_t device_state_flag = 0x0000;
//lost_timeout的各个字节为对应外设通信断联计数值，达到DEVICE_TIMEOUT_LIMIT时便认为其出现异常
uint8_t lost_count[16] = {DEVICE_TIMEOUT_LIMIT};
//电池电压。
float voltage = 0;
//蜂鸣器计数器。
uint32_t buzzer_cnt = 0;

#if defined  Broad_Chassis
void SelfCheck_Task(void const* argument)
{
	buzzer_off();
	vTaskDelay(100);
	while(1)
	{
			vTaskDelay(99);					//考虑到其它代码消耗的时钟周期
	}
}
#else
void SelfCheck_Task(void const* argument)
{
		vTaskDelay(100);
		
		while(1)
		{
//				Low_Battery_Voltage_Warning();
//				Device_State_Update();
//				LED_State_Update();
			
				/* 自检时丢失自瞄信息蜂鸣器响 */
//				Autoaim_Buzzer_State_Update();
//				Autoaim_lose_flag ++;
			
				vTaskDelay(99);					//考虑到其它代码消耗的时钟周期
		}
}
#endif
/**
  * @brief				读取外设状态（函数版本）
	* @param[in]		id:外设编号
	* @retval				device_state:外设状态，1表示通信正常，0表示通信异常
  */
device_state Device_State_Read(device_id id)
{
		return ((device_state_flag & (1 << (id))) ? DEVICE_NORMAL : DEVICE_DISORDER);
}

/**
  * @brief				外设状态更新
	* @param				none
  * @retval				none
  */
void Device_State_Update(void)
{
		static uint8_t i;
	
		for(i = 0; i < 16; i++)
		{
				if(lost_count[i] < DEVICE_TIMEOUT_LIMIT)
				{
						device_state_flag |= (1 << i);
						lost_count[i]++;
				}
				else
						device_state_flag &= ~(1 << i);
		}
}

/**
  * @brief				三色LED状态更新
	* @param				none
  * @retval				none
  */
void LED_State_Update(void)
{
		static uint8_t t = 0;
		uint8_t red_state = 0, green_state = 0, blue_state = 0;		//0：熄灭，1~4：闪烁:1~4次，5：常亮

		//红灯状态设置
		if(__DEVICE_STATE_READ(CAN_MCU_DOWN) == DEVICE_DISORDER)
			red_state = 5;
		else if(__DEVICE_STATE_READ(UART_RC) == DEVICE_DISORDER)
			red_state = 4;
		else if(__DEVICE_STATE_READ(UART_IMAGE) == DEVICE_DISORDER)
			red_state = 1;
		else
			red_state = 0;
		
		//绿灯状态设置
		if(__DEVICE_STATE_READ(CAN_MG5010_GIMBAL_YAW) == DEVICE_DISORDER)
			green_state = 0;
		else if(__DEVICE_STATE_READ(CAN_GM6020_GIMBAL_PITCH) == DEVICE_DISORDER)
			green_state = 4;
		else if(__DEVICE_STATE_READ(CAN_M2006_SHOOT_DIAL) == DEVICE_DISORDER)
			green_state = 3;
		else if(__DEVICE_STATE_READ(CAN_M3508_SHOOT_FRIC_1) == DEVICE_DISORDER || __DEVICE_STATE_READ(CAN_M3508_SHOOT_FRIC_2) == DEVICE_DISORDER)
			green_state = 2;
		else
			green_state = 5;
		
		//蓝灯状态设置
		if(__DEVICE_STATE_READ(USB_NUC) == DEVICE_DISORDER)
			blue_state = 0;
		else
			blue_state = 5;
	
		//LED状态更新
		switch(t)
		{
				case 0:
				case 2:
				case 4:
				case 6:
						__LED_R_SET(red_state > t/2);
						__LED_G_SET(green_state > t/2);
						__LED_B_SET(blue_state > t/2);
				break;
				
				case 1:
				case 3:
				case 5:
				case 7:
				case 8:
				case 9:
						__LED_R_SET(red_state == 5);
						__LED_G_SET(green_state == 5);
						__LED_B_SET(blue_state == 5);
				break;
				
				default:
				break;
		}
	
		if(++t > 9)
				t = 0;
}

/**
  * @brief			电池低压报警。
  * @param[in]		none。
  * @retval			none。
  */
 void Low_Battery_Voltage_Warning(void)
 {
	 voltage = get_battery_voltage();
	 if(voltage >15 && voltage <= 22.0f)
	 {
		 buzzer_cnt++;
		 if(buzzer_cnt > 100)
		 {
			 SetBuzzerFrequence(1046);
			 buzzer_cnt = 0;
		 }
	 }
	 else
	 {
		 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		 buzzer_cnt = 0;
	 }	
 }
