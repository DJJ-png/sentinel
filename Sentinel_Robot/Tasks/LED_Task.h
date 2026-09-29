#ifndef _LED_TASK
#define _LED_TASK

#include "main.h"

void LED_Task(void const * argument);

void LED_Flash(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t delay);

#endif
