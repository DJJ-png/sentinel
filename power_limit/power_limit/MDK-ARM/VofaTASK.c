#include "FreeRTOS.h"
#include "task.h"
#include "Vofa_send.h"
#include "ina226.h"
#include "DJIMotoDriver.h"

extern float power_ina226[4];
extern float motor_speed[4];
extern float power_set;
extern float v_set;
extern float give_current;
extern float pre_power[4];
extern uint8_t start;
extern DJIMotoStateTD motor[4];
void VofaTask()
{  
    while(1)
    {
        if(start==1)
        {
            Vofa_Send_Data3(power_ina226[0], motor[0].current,motor_speed[0]);//收集数据集
        }
        else
        {
            Vofa_Send_Data3(power_ina226[0], power_set,pre_power[0]);
        }
        
        vTaskDelay(1);
    }
    
}