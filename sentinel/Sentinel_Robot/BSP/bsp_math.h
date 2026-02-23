#ifndef __BSP_MATH_H
#define __BSP_MATH_H

#include "struct_typedef.h"
#include "arm_math.h"
#include "main.h"
/* ************************************* ÔËËãºê ******************************************** */

#define SIGNAL(n)										((n)>=0 ? 1 : -1)
#define ABS(n)											((n)>=0 ? (n) : -(n))
#define DATA_LIMIT(n,min,max)				((n)>(max) ? (max) : ((n)<(min) ? (min) : (n)))
#define DEADBAND(n,db)							(ABS(n) >= ABS(db) ? (n) : 0)
#define RAMP_CTRL(ref,set,acc)	((ref) + DATA_LIMIT((acc),0,1) * ((set) - (ref)))
#define LIMIT_TO_SET(data,set)  	(((data)>(set))?((data)-(set)*2):(((data)<-(set))?((data)+(set)*2):(data)))
extern fp32 limit_pi(fp32 in);
#endif