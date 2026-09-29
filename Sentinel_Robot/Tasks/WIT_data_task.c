#include "WIT_data_task.h"
#include "bsp_usart.h"
#include "hwt901b.h"
#include "usart.h"
#include "dm_imu.h"
#include "Referee.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;


void WIT_data_task(void const * argument)
{
//		 // 初始化HWT901B模块
//		 HWT901B_DMA_Init(&huart6, &hdma_usart6_rx);
     vTaskDelay(200);
     while(1)
     {
//				IMU_RequestData(0x01,0x01);
//        HAL_Delay(1);
        IMU_RequestData(0x01,0x02);
				vTaskDelay(1);
        IMU_RequestData(0x01,0x03);
//        IMU_RequestData(0x01,0x04);
//        HAL_Delay(1);  
//				 if(sensor_data.data_ready)
//        {
//            HWT901B_Data *data = HWT901B_GetData();
//           
//        }
        vTaskDelay(1);
				/* 由于C板复位时，外置陀螺仪不会复位，所以0点不会更新，需要代码里减去一个偏置 */
				if(imu.imu_reset_flag == 1)
				{
						if(imu.imu_reset_delay > 400)
						{
							imu.yaw_error = imu.yaw;
							//imu.pitch_error = imu.pitch;
							imu.imu_reset_flag = 0;	
							imu.imu_reset_delay = 0;							
						}
						imu.imu_reset_delay++;
				}
     }
}


