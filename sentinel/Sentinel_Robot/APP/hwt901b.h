#ifndef __HWT901B_DMA_H
#define __HWT901B_DMA_H

#include "stm32f4xx_hal.h"

#define HWT901B_RX_BUFFER_SIZE 256  // DMA接收缓冲区大小

typedef struct {
    float acc[3];       // 加速度 (g)					x  y  z		
    float gyro[3];      // 角速度 (°/s)				x  y  z
    float angle[3];     // 角度 (°)						pitch  roll  yaw
    float temp;         // 温度 (°C)
    uint32_t last_update; // 最后更新时间戳
    uint8_t data_ready; // 数据就绪标志
} HWT901B_Data;

// 初始化函数
void HWT901B_DMA_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);

// 处理接收到的数据
void HWT901B_ProcessBuffer(uint16_t size);

// 获取传感器数据
HWT901B_Data* HWT901B_GetData(void);

// 发送命令函数
void HWT901B_SendCommand(uint8_t cmd);

// 设置波特率到115200
void HWT901B_SetBaudRate115200(void);

extern HWT901B_Data sensor_data;

#endif