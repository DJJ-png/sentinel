#ifndef POWER_CONTROL
#define POWER_CONTROL

#include "main.h"
#include "math.h"
#include "ina226.h"
#include "struct_typedef.h"

typedef struct
{
    int16_t *speed;//(换算过减速比)
    fp32 *current;//融入低通滤波，换算到成ma
    uint8_t ina226_addr;
    
    double k[6];
    float pre_power;//预测功率
    float real_power;//真实功率
    float v_eat;
    uint16_t cnt;
    float per_power_sum;
    
}power_control;

void power_control_init(power_control *power_control,double k[6],int16_t *speed,fp32 *current,uint16_t ina226_addr);
void power_limit(power_control *power_control,fp32 power_set);

#endif