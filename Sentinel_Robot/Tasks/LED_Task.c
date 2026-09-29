#include "LED_Task.h"
#include "main.h"
#include "bsp_buzzer.h"

/* 测试代码运行是否正常 */

#include "Vofa_send.h"

/**
 *@brief	LED灯闪烁函数
 *@param	GPIOx	GPIO_Pin	接口名
					delay	延时时间
 *@return	none
 */
void LED_Flash(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t delay)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);vTaskDelay(delay);
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);vTaskDelay(delay);
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
}


/**
 *@brief	LED灯闪烁和蜂鸣器初始化任务
 *@param	none
 *@return	none
 */
void LED_Task(void const * argument)
{
//		// 蜂鸣器初始化
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 150);
//		vTaskDelay(50);
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 100);
//		vTaskDelay(150);
		buzzer_off();

		while(1)
		{
			#ifdef Broad_Gimbal
								LED_Flash(LED_PIN_PORT,LED_RED_PIN,100);
			#endif
			#ifdef Broad_Chassis
								LED_Flash(LED_PIN_PORT,LED_BLUE_PIN,100);
			#endif
			
			vTaskDelay(1);
			
		}
}
