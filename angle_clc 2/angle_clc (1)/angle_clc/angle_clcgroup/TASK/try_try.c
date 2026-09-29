#include "main.h"
#include "INS_Task.h"
#include "try_try.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "AHRS.h"
#include "bsp_can.h"
#include "i2c.h"
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
#include "bsp_can.h"
extern osThreadId try_tryHandle;
 const fp32 *gyro_data;

typedef struct{
	uint32_t ID_1;
	uint32_t ID_2;
}CANID;
CANID can_ID={
	.ID_1=0x011,
	.ID_2=0x012
};
void Try_Try(void const * argument){		
	while(1){
           gyro_data= get_gyro_data_point();
			CAN_CMD_f32(&hcan2,can_ID.ID_1,INS_angle_deg[0],INS_angle_deg[2]);
			CAN_CMD_f32(&hcan2,can_ID.ID_2,gyro_data[0] * 180.0f / 3.141592653589f,gyro_data[1] * 180.0f / 3.141592653589f);
		  vTaskDelay(5);
	}
}