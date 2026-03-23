#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "pid.h"
#include "config_set.h"
#include "Filter.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

typedef struct{
    uint8_t start_flag;
    uint16_t cnt;
    float k;
		fp32 reference[3];
}fric_spealist_t;

typedef struct
{
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;
    fp32 ENC_angle;
    
    int16_t give_current;
    int16_t set_current;

    uint8_t temperate;
	
    pid_type_def speed_pid;
	pid_type_def angle_pid;
    uint8_t mode;
} Dial_Motor_t;

typedef struct{
    WeightedFilter fric_filter;
    pid_type_def pid_speed;
    fric_spealist_t fric_sp;
    int16_t rpm_set;
}Shoot_Motor_t;

typedef struct
{  
	uint8_t barrel_wait_flag;
	uint16_t barrel_wait_cnt;
	
	fp32 barrel_heat[2];
	
} barrel_control_t;



typedef struct
{
    //·¢Éä»ú¹¹
	uint8_t fric_state;
	
	uint8_t dial_mode;
	fp32 dial_speed;
	
	uint8_t if_single_hit;//base
	uint16_t dial_single_cnt;
	
	uint8_t shoot_number;
	uint8_t shoot_get_flag;
	uint8_t shoot_on_flag;
	
} shoot_control_t;


void Barrel_Init_Flash();


extern barrel_control_t barrel_control;
extern shoot_control_t shoot_control;

typedef enum {
    CLOSE=0,
    SINGLE_FIRE,
    RAPID_FIRE,
    SHOOT_TEST,
} Dial_State_e;
#endif


