#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "main.h"
#include "DJIMotoDriver.h"
#include "ina226.h"
#include "power_control.h"
#include "math.h"
/*
k0: 0.8452750915335764
k1: 0.00044039452423288305
k2: -0.00020911720414779812
k3: 2.855789810345799e-05
k4: 8.375542551852454e-05
k5: -1.4024588258022384e-06
*/
double k[4][6]={
            { 0.8452750915335764,
             0.00044039452423288305,
             -0.00020911720414779812,
             2.855789810345799e-05,
           8.375542551852454e-05,
             -1.4024588258022384e-06,
            },//k[0]
            {0.8446929913945093,
             0.0009444653935039717,
             0.0004524617259854866,
             1.206900861676191e-05,
             -8.544096003416771e-07,
             7.159362529695892e-05,
            },//k[1]
            {0.8446929913945093,
             0.0009444653935039717,
             0.0004524617259854866,
             1.206900861676191e-05,
             -8.544096003416771e-07,
             7.159362529695892e-05,
            },//k[2]
            {0.8446929913945093,
             0.0009444653935039717,
             0.0004524617259854866,
             1.206900861676191e-05,
             -8.544096003416771e-07,
             7.159362529695892e-05,
            }//k[3]


            };//拟合数据参数
#define PI 3.14159265859
extern CAN_HandleTypeDef hcan2;
            
#define DEV_ADDR0 0x80
#define DEV_ADDR1 0x81
#define DEV_ADDR2 0x82
#define DEV_ADDR3 0x83

pid_type_def motor_power_pid;
#define MOTOR_POWER_KP 30.0f
#define MOTOR_POWER_KI 0.5f
#define MOTOR_POWER_KD 10.0f
#define MOTOR_POWER_MAX_OUT 10000
#define MOTOR_POWER_MAX_IOUT 3000

pid_type_def motor_speed_pid[4];
#define MOTOR_SPEED_KP 100.0f
#define MOTOR_SPEED_KI 0.01f
#define MOTOR_SPEED_KD 10.0f
#define MOTOR_SPEED_MAX_OUT 10000   
#define MOTOR_SPEED_MAX_IOUT 3000

pid_type_def motor_current_pid;
#define MOTOR_CURRENT_KP 10.0f
#define MOTOR_CURRENT_KI 0.00f
#define MOTOR_CURRENT_KD 10.0f
#define MOTOR_CURRENT_MAX_OUT 10000
#define MOTOR_CURRENT_MAX_IOUT 3000

DJIMotoStateTD motor[4];


//数据采集与设定
float power_ina226[4];
float motor_speed[4];
float power_set=5.0f;
float v_set=150.0;
float motor_current=0;
float current_set=2000;
fp32 give_current[4]={0};
uint8_t start=0;
uint32_t time_cnt=0;

//拟合相关
float pre_power[4];
power_control power_2006[4];
uint16_t ina226_addr[4]={0x80,0x82,0x84,0x86};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    for(int i=0;i<4;i++)
    {
        DJI_SaveMotoMsg(&hcan1,CAN_RX_FIFO0,motor);
    }
     
}
void motorTask()
{
    //初始化
    fp32 motor_pid_power[3]={MOTOR_POWER_KP,MOTOR_POWER_KI,MOTOR_POWER_KD};
    fp32 motor_pid_speed[3]={MOTOR_SPEED_KP,MOTOR_SPEED_KI,MOTOR_SPEED_KD};
    fp32 motor_pid_current[3]={MOTOR_CURRENT_KP,MOTOR_CURRENT_KI,MOTOR_CURRENT_KD};
    PID_init(&motor_power_pid,PID_POSITION,motor_pid_power,MOTOR_POWER_MAX_OUT,MOTOR_POWER_MAX_IOUT);
    for(int i=0;i<4;i++)
    {
         PID_init(&motor_speed_pid[i],PID_POSITION,motor_pid_speed,MOTOR_SPEED_MAX_OUT,MOTOR_SPEED_MAX_IOUT);
    }
    PID_init(&motor_current_pid,PID_POSITION,motor_pid_current,MOTOR_CURRENT_MAX_OUT,MOTOR_CURRENT_MAX_IOUT);
    INA226_init(DEV_ADDR);
    for(int i=0;i<4;i++)
        DJIMoto_State_Init(&motor[i],k[i]);
    for(int i=0;i<4;i++)
        power_control_init(&power_2006[i],k[i],&motor->speed,&motor->current,ina226_addr[i]);
    
    while(1)
    {
        //初始化
        for(int i=0;i<4;i++)
        {
            motor[i].power_set=power_set;
            power_limit(&power_2006[i],power_set);
            power_ina226[i]=power_2006[i].real_power;
            motor_speed[i]=fabs((float)(motor[i].speed));
            pre_power[i]=power_2006[i].pre_power;
        }
        //功率拟合测试数据（这个数据量不够，建议直接上车测）
        if(start==1)
        {
            if(time_cnt<4000)
            {
                v_set=-200;
                time_cnt++;
            }
            else if(time_cnt<8000)
            {
                v_set=-100;
                time_cnt++;
            }
            else if(time_cnt<12000)
            {
                v_set=-50;
                time_cnt++;
            }
            else if(time_cnt<16000)
            {
                v_set=-20;
                time_cnt++;
            }
            else if(time_cnt<20000)
            {
                v_set=-10;
                time_cnt++;
            }
            else if(time_cnt<22000)
            {
                v_set=10;
                time_cnt++;
            }
            else if(time_cnt<28000)
            {
                v_set=20;
                time_cnt++;
            }
            else if(time_cnt<32000)
            {
                v_set=50;
                time_cnt++;
            }
            else if(time_cnt<36000)
            {
                v_set=100;
                time_cnt++;
            }
            else if(time_cnt<40000)
            {
                v_set=200;
                time_cnt++;
            }
            else
                v_set=0;
            if(time_cnt<40000)
            {
                for(int i=0;i<4;i++)
                {
                    give_current[i]=PID_calc(&motor_speed_pid[i],motor[i].speed,v_set);
                }
            }     
        }
        else if(start==2)//功率控制测试
        {
            for(int i=0;i<4;i++)
            {
                give_current[i]=PID_calc(&motor_speed_pid[i], motor[i].speed , v_set*power_2006[i].v_eat);
            }
        }
        else if(start==0)//停止
        {
            for(int i=0;i<4;i++)
            {
                give_current[i]=0;
            }
        }
        SetMoto_Current(give_current[0],give_current[1],give_current[2],give_current[3]);
        vTaskDelay(1);
        
    }
    
}