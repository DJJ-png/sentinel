#include "referee_usart_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "shoot_task.h"
#include "Gimbal_Task.h"
#include "remote_control.h"

/* Private define ------------------------------------------------------------*/

#define Referee_FIFOInit fifo_s_init
#define Max(a,b) ((a) > (b) ? (a) : (b))
//#define Robot_ID_Current Robot_ID_Blue_Infantry4
#define Robot_ID_Current Robot_ID_Red_Infantry4

/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx; 
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 裁判系统接收数据队列 */

fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;  

extern uint8_t if_predict;
extern uint8_t Nuc_FIFO_Buffer[2][128];

#if defined  Broad_Chassis

void referee_usart_task(void const * argument)
{
	
	fifo_s_init(&Referee_FIFO, Referee_FIFO_Buffer, REFEREE_FIFO_BUF_LENGTH);
	
	
	Referee_StructInit();
	Referee_UARTInit(Referee_Buffer[0], Referee_Buffer[1], REFEREE_USART_RX_BUF_LENGHT);

	vTaskDelay(300);

	while(1)
	{
		/* 解析裁判系统数据 */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
	
	}
}

# endif

uint16_t this_time_rx_len = 0;
void USART6_IRQHandler_1(void)
{
		if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
           __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
           __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
					return;
        }
    }
		HAL_UART_IRQHandler(&huart6);
}


