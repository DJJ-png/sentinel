/**
  ****************************(C)2025 ckyf****************************
  * @file      power_control.h/.c
  * @brief     进行电机的功率。
  * @note       
  * @history
  *  Version    Date            Author         
  *  V1.0.0    30-12-2025        kuyu        
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C)2025 ckyf****************************
  */

#include "power_control.h"
/**
  * @brief          power_control_init
  * @param[in]      power_control: power_control结构体
  * @param[in]      k[6]: 对应电机的拟合参数
  * @param[in]      speed: 电机速度的指针
  * @param[in]      current: 电调电流的指针
  * @param[in]      ina226_addr: ina226地址
  * @retval         none
  */
void power_control_init(power_control *power_control,double k[6],int16_t *speed,fp32*current,uint16_t ina226_addr)
{
    for(int i=0;i<6;i++)
    {
        power_control->k[i]=k[i];
    }
    
    power_control->v_eat=1;
    power_control->cnt=0;
    power_control->per_power_sum=0.0f;
    power_control->current=current;
    power_control->speed=speed;
    power_control->ina226_addr=ina226_addr;
    INA226_init(ina226_addr);
}
/**
  * @brief          power_limit
  * @param[in]      power_control: power_control结构体
  * @param[in]      power_set: 电机的设定功率
  */
void power_limit(power_control *power_control,fp32 power_set)
{
     power_control->per_power_sum+=power_control->k[0] + power_control->k[1]*(*power_control->current) 
                                    +power_control->k[2]*(*power_control->speed) + power_control->k[3]*(*power_control->current)*(*power_control->speed) 
                                    + power_control->k[4]*pow(*power_control->current,2) + power_control->k[5]*pow(*power_control->speed,2);
    power_control->cnt++;
    if(power_control->cnt==64)
    {
        power_control->pre_power=power_control->per_power_sum/power_control->cnt;
        power_control->cnt=0;
        power_control->per_power_sum=0.0f;
    }
    power_control->real_power=INA226_GetPower(power_control->ina226_addr);
    if(fabs(power_control->pre_power-power_control->real_power)<5.0f)
    {
         double a=power_control->k[5]*pow(*power_control->speed,2) ;
         double b=power_control->k[2]*(*power_control->speed )+ power_control->k[3]*(*power_control->speed)*(*power_control->current);
         double c=power_control->k[0]+power_control->k[1]*(*power_control->current)+ power_control->k[4]*pow(*power_control->current,2)-power_set;
        if(power_control->real_power>power_set)
        {
            if((pow(b,2)-4*a*c)>0)
            {
                power_control->v_eat=fabs((-b+sqrt(pow(b,2)-4*a*c))/(2*a));
                if(power_control->v_eat>1)
                 power_control->v_eat=1;
                
            }
            else if((pow(b,2)-4*a*c)<0)
            {
                power_control->v_eat=power_control->v_eat;
            }
        }
        else
        {
            power_control->v_eat=1;
        }

    }
}