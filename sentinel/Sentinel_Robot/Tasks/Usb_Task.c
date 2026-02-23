#include "Usb_Task.h"
#include "bsp_can.h"
#include "Nmanifold_usart_task.h"
#include "Shoot_task.h"

//#define CAN_SEND_NX
#define USB_SEND_NX


void usb_task(void const * argument)
{
    
   /*USBÍ¨ÐÅ*/
   cmd_id_init();
   cmd_id_task_create(GIMBAL_AND_CONFIG_SEND_ID,843);//237
	 vTaskDelay(10);
//	 cmd_id_task_create(ENEMY_STATE_SEND_ID,51);
//	 vTaskDelay(10);
//	 cmd_id_task_create(SELF_DATA_SEND_ID,11);
//	 vTaskDelay(10);
 //                cmd_id_task_create(0x21,1);
   for(;;)
   {						
         Nuc_data_unpacked();
         cmd_id_queue_handle();
     vTaskDelay(1);
   }
	

}

