#ifndef _WIT_DATA_TASK_
#define _WIT_DATA_TASK_

#include "main.h"
#include "cmsis_os.h"

#define WIT_RX_BUF_LENGHT 1

extern void WIT_data_task(void const * argument);
extern void USART6_IRQHandler_WIT(void);

#endif
