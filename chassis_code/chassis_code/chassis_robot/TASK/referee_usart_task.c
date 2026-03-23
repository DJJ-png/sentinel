#include "referee_usart_task.h"
/* Private define ------------------------------------------------------------*/
#include "struct_typedef.h"
#define Referee_FIFOInit fifo_s_init
#define Max(a,b) ((a) > (b) ? (a) : (b))
//#define Robot_ID_Current Robot_ID_Blue_Infantry4
#define Robot_ID_Current Robot_ID_Red_Infantry4

/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

// 裁判系统FIFO缓存：缓存裁判系统接收数据
fifo_s_t       Referee_FIFO;

// 裁判串口双缓冲区
uint8_t        Referee_buffer[2][REFEREE_USART_RX_BUF_LENGHT];

// 裁判FIFO缓存数组
uint8_t        Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

// 0x0308数据接收标志：标记是否开始接收该类型裁判数据
uint8_t        receive_0x0308_begin_flag = 0;

// 0x0301数据接收标志：标记是否开始接收该类型裁判数据
uint8_t        receive_0x0301_begin_flag = 0;

// 裁判解包数据：存储解包后的裁判系统数据
unpack_data_t  Referee_Unpack_OBJ; 

// 上板数据接收计数：用于0x0308数据发送的延时计数
uint16_t receive_upboard_cnt = 0;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx; 
extern DMA_HandleTypeDef hdma_usart6_tx;


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
        if(receive_0x0308_begin_flag == 1)
		{
			receive_upboard_cnt++ ;
			if(receive_upboard_cnt>350){
				Sentry_Send_Referee_0x0308();
				receive_0x0308_begin_flag = 0;
				receive_upboard_cnt = 0;
			}
		}
		if(receive_0x0301_begin_flag == 1){
			Sentry_Referee_data_update();
			Sentry_Send_Referee_0x0301();
			receive_0x0301_begin_flag =0;
			vTaskDelay(1);
		}
	}
}


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




