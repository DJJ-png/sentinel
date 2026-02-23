#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "pid.h"

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
} shoot_motor_t;


typedef struct
{
	uint8_t barrel_stare;
	int32_t barrel_angle;
	int32_t barrel_zore_point;
	uint16_t barrel_init_cnt;
	
	uint8_t barrel_wait_flag;
	uint16_t barrel_wait_cnt;
	
	fp32 barrel_heat[2];
	
} barrel_control_t;


typedef struct
{
	uint8_t fric_state;
	
	uint8_t dial_mode;
	fp32 dial_speed;
	
	uint8_t dial_stop_cnt;
	uint16_t wait_time;
	
	uint8_t if_single_hit;//base
	int8_t single_rc;   //control
	uint8_t single_aim; //control
	uint16_t dial_single_cnt;
	
	uint8_t shoot_number;
	uint8_t shoot_get_flag;
	uint8_t shoot_on_flag;
	
} shoot_control_t;


void Barrel_Init_Flash();


extern barrel_control_t barrel_control;
extern shoot_control_t shoot_control;

#endif


