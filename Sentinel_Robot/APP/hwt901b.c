#include "hwt901b.h"
#include <string.h>
#include <math.h>

static UART_HandleTypeDef *hwt901b_huart = NULL;
HWT901B_Data sensor_data = {0};
static uint8_t rx_buffer[HWT901B_RX_BUFFER_SIZE];
static uint16_t buffer_pos = 0;

// 帧结构体
#pragma pack(push, 1)
typedef struct {
    uint8_t header;     // 0x55
    uint8_t cmd;        // 命令字
    int16_t data[3];    // 数据(小端模式)
    uint8_t checksum;   // 校验和
} HWT901B_Frame;
#pragma pack(pop)

void HWT901B_DMA_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
    hwt901b_huart = huart;
    
    // 关联DMA到UART
    __HAL_LINKDMA(huart, hdmarx, *hdma_usart_rx);
    
    // 启动DMA接收
    HAL_UART_Receive_DMA(hwt901b_huart, rx_buffer, HWT901B_RX_BUFFER_SIZE);
    
    memset(&sensor_data, 0, sizeof(HWT901B_Data));
}

void HWT901B_ProcessBuffer(uint16_t size)
{
    for(uint16_t i = 0; i < size; i++)
    {
        static uint8_t frame_buffer[11];
        static uint8_t frame_index = 0;
        static uint8_t state = 0;  // 0:等待帧头, 1:接收数据
        
        uint8_t data = rx_buffer[i];
        
        if(state == 0 && data == 0x55)
        {
            frame_index = 0;
            frame_buffer[frame_index++] = data;
            state = 1;
        }
        else if(state == 1)
        {
            frame_buffer[frame_index++] = data;
            
            if(frame_index >= 11)  // 完整帧
            {
                uint8_t sum = 0;
                for(uint8_t j = 0; j < 10; j++)
                {
                    sum += frame_buffer[j];
                }
                
                if(sum == frame_buffer[10])  // 校验通过
                {
                    HWT901B_Frame *frame = (HWT901B_Frame*)frame_buffer;
                    
                    switch(frame->cmd)
                    {
                        case 0x51:  // 加速度
                            sensor_data.acc[0] = frame->data[0] / 32768.0f * 16.0f;
                            sensor_data.acc[1] = frame->data[1] / 32768.0f * 16.0f;
                            sensor_data.acc[2] = frame->data[2] / 32768.0f * 16.0f;
                            break;
                            
                        case 0x52:  // 角速度
                            sensor_data.gyro[0] = frame->data[0] / 32768.0f * 2000.0f;
                            sensor_data.gyro[1] = frame->data[1] / 32768.0f * 2000.0f;
                            sensor_data.gyro[2] = frame->data[2] / 32768.0f * 2000.0f;
                            break;
                            
                        case 0x53:  // 角度
                            sensor_data.angle[0] = frame->data[0] / 32768.0f * 180.0f;
                            sensor_data.angle[1] = frame->data[1] / 32768.0f * 180.0f;
                            sensor_data.angle[2] = frame->data[2] / 32768.0f * 180.0f;
                            break;
                            
                        case 0x54:  // 温度
                            sensor_data.temp = frame->data[0] / 100.0f;
                            break;
                    }
                    
                    sensor_data.last_update = HAL_GetTick();
                    sensor_data.data_ready = 1;
                }
                
                state = 0;
            }
        }
    }
    
    // 处理完成后重新启动DMA接收
    HAL_UART_Receive_DMA(hwt901b_huart, rx_buffer, HWT901B_RX_BUFFER_SIZE);
}

HWT901B_Data* HWT901B_GetData(void)
{
    sensor_data.data_ready = 0;
    return &sensor_data;
}

void HWT901B_SendCommand(uint8_t cmd)
{
    uint8_t buffer[2] = {0xFF, cmd};
    HAL_UART_Transmit(hwt901b_huart, buffer, 2, 100);
}

void HWT901B_SetBaudRate115200(void)
{
    // 发送波特率设置命令
    HWT901B_SendCommand(0x05);
    HAL_Delay(100);
    
    // 停止当前DMA传输
    HAL_UART_DMAStop(hwt901b_huart);
    
    // 重新配置串口
    hwt901b_huart->Init.BaudRate = 115200;
    if(HAL_UART_Init(hwt901b_huart) != HAL_OK)
    {
        // 错误处理
    }
    
    // 重新启动DMA接收
    HAL_UART_Receive_DMA(hwt901b_huart, rx_buffer, HWT901B_RX_BUFFER_SIZE);
    HAL_Delay(100);
}

// DMA接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == hwt901b_huart)
    {
        HWT901B_ProcessBuffer(HWT901B_RX_BUFFER_SIZE);
    }
}

// DMA接收半满回调（可选）
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == hwt901b_huart)
    {
        HWT901B_ProcessBuffer(HWT901B_RX_BUFFER_SIZE/2);
    }
}