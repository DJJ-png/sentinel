#include "FreeRTOS.h"
#include "task.h"
#include "Vofa_send.h"
#include "ina226.h"

extern float power_ina226;
extern float motor_speed;
extern float power_set;
extern float v_set;
extern float motor_current;
extern float give_current;
extern float pre_power;
extern uint8_t start;

void VofaTask()
{  
    while(1)
    {
        if(start==1)
        {
            Vofa_Send_Data3(power_ina226, motor_current,motor_speed);//收集数据集
        }
        else
        {
            Vofa_Send_Data3(power_ina226, power_set,pre_power);
        }
        
        vTaskDelay(1);
    }
    
}