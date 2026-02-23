#include "robot_message.h"
#include "config_set.h"
#include "Gimbal_Task.h"
#include "referee.h"
#include "INS_Task.h"
#include "bsp_can.h"
#include "Switch_Task.h"
#include "dm_imu.h"
#include "SelfCheck_Task.h"
#include "arm_math.h"
#include "Nmanifold_usart_task.h"
#include "CRCs.h"

//small low high
nuc_receive_data_t	nuc_receive_data;
nuc_transmit_data_t 	nuc_transmit_data;
cmd_id_queue_t cmd_id_queue;
uint8_t aim_pressing=0;
extern aim_control_t aim_control;
fp32 ADVANCED_quat[4];
extern fp32 INS_accel[4];

CAN_send_NX_t CAN_Transmit_NX;
CAN_receive_NX_t CAN_Receive_NX;

//敌方车数据
enemy_state_data_t enemy_state_data[6];

/*************************** SEND ********************************/

void data_update(uint8_t cmd_id)
{
    switch(cmd_id){
        case GIMBAL_AND_CONFIG_SEND_ID:
				/*8.1*/
                          //测试打符用的，这行代码，在导航没开的情况下。
						 //nuc_control.action.robot_aim = 2;
                        nuc_transmit_data.robot_gimbal_data_send.head[0]='S';
                        nuc_transmit_data.robot_gimbal_data_send.head[1]='P';
        
                        if(Robot_Status.robot_id==107 && nuc_control.action.robot_aim != 2)
							nuc_transmit_data.robot_gimbal_data_send.mode	=		0;// 自瞄红
						else if(Robot_Status.robot_id == 7 && nuc_control.action.robot_aim != 2)  
							nuc_transmit_data.robot_gimbal_data_send.mode	=		1;// 自瞄蓝
						else if(Robot_Status.robot_id==107 && nuc_control.action.robot_aim == 2)
							nuc_transmit_data.robot_gimbal_data_send.mode	=		3;// 小符蓝
						else if(Robot_Status.robot_id == 7 && nuc_control.action.robot_aim == 2)  
							nuc_transmit_data.robot_gimbal_data_send.mode	=		2;// 小符红
                        
						
                        
                        nuc_transmit_data.robot_gimbal_data_send.pitch      =	    gimbal_motor[PITCH_6015].INS_angle;
                        nuc_transmit_data.robot_gimbal_data_send.pitch_vel  =	    gimbal_motor[PITCH_6015].INS_speed;
                        
                        nuc_transmit_data.robot_gimbal_data_send.yaw	    =		gimbal_motor[ADVANCED_YAW_6020].INS_angle;
                        nuc_transmit_data.robot_gimbal_data_send.yaw_vel    =       gimbal_motor[ADVANCED_YAW_6020].INS_speed;
						
                        nuc_transmit_data.robot_gimbal_data_send.bullet_speed=      Shoot_Data.initial_speed;
                        nuc_transmit_data.robot_gimbal_data_send.bullet_count=      Shoot_Data.shooter_number;
                        
                        ADVANCED_quat[0]=cos((nuc_transmit_data.robot_gimbal_data_send.yaw/2)*PI/180)*cos((nuc_transmit_data.robot_gimbal_data_send.pitch/2)*PI/180);
                        ADVANCED_quat[1]=sin((nuc_transmit_data.robot_gimbal_data_send.yaw/2)*PI/180)*sin((nuc_transmit_data.robot_gimbal_data_send.pitch/2)*PI/180);
                        ADVANCED_quat[2]=cos((nuc_transmit_data.robot_gimbal_data_send.yaw/2)*PI/180)*sin((nuc_transmit_data.robot_gimbal_data_send.pitch/2)*PI/180);
                        ADVANCED_quat[3]=sin((nuc_transmit_data.robot_gimbal_data_send.yaw/2)*PI/180)*cos((nuc_transmit_data.robot_gimbal_data_send.pitch/2)*PI/180);
                        memcpy(nuc_transmit_data.robot_gimbal_data_send.quat,ADVANCED_quat,sizeof(fp32)*4);
            break;
        default:
            break;
    }
}


void send_data_to_nuc(uint8_t cmd_id)
{
    uint16_t len;
//    uint8_t t[10][4];
//		usb_cdc_data.usb_cdc_send_buf[0]=0xAA;
//		enemy_state_data[0].attack_enhance = ((Radar_Mark_Data.mark_progress >> 0) & 1);
//		enemy_state_data[1].attack_enhance = ((Radar_Mark_Data.mark_progress >> 1) & 1);
//		enemy_state_data[2].attack_enhance = ((Radar_Mark_Data.mark_progress >> 2) & 1);
//		enemy_state_data[3].attack_enhance = ((Radar_Mark_Data.mark_progress >> 3) & 1);
//	  enemy_state_data[5].attack_enhance = ((Radar_Mark_Data.mark_progress >> 4) & 1);
	
    switch (cmd_id)
    {
        case GIMBAL_AND_CONFIG_SEND_ID:
                            len=43;
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[0],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.head,2);
							usb_cdc_data.usb_cdc_send_buf[2]= nuc_transmit_data.robot_gimbal_data_send.mode;
							memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[3],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.quat,4*4);
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[19],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.yaw,4);
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[23],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.yaw_vel,4);
							memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[27],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.pitch,4);
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[31],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.pitch_vel,4);
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[35],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.bullet_speed,4);
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[39],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.bullet_count,2);
                            nuc_transmit_data.robot_gimbal_data_send.crc16=CRC16_Calculate(usb_cdc_data.usb_cdc_send_buf,41);
                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[41],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.crc16,2);
//                            len=57;
//                            usb_cdc_data.usb_cdc_send_buf[0]=0x55;
//                            usb_cdc_data.usb_cdc_send_buf[1]=0xAA;
//                            usb_cdc_data.usb_cdc_send_buf[3]=0x01;
//                            usb_cdc_data.usb_cdc_send_buf[4]=0x01;
//                            memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[5],(uint8_t*)&nuc_transmit_data.imu_data_send.x_accel,4);
                            
						break;
//				 case SELF_DATA_SE  ND_ID://0x16
//            len = 16;
//            usb_cdc_data.usb_cdc_send_buf[0] = 0xff;
//            usb_cdc_data.usb_cdc_send_buf[1] = len;
//            usb_cdc_data.usb_cdc_send_buf[2] = SELF_DATA_SEND_ID;
//            memcpy(usb_cdc_data.usb_cdc_send_buf + 3,    (uint8_t*)&Robot_Pos.x,        sizeof(float));
//            memcpy(usb_cdc_data.usb_cdc_send_buf + 7,    (uint8_t*)&Robot_Pos.y,        sizeof(float));
//            memcpy(usb_cdc_data.usb_cdc_send_buf + 11,    (uint8_t*)&Robot_Pos.angle,    sizeof(float));
//            usb_cdc_data.usb_cdc_send_buf[15] = 0x0D;
//            break;
//				 case ENEMY_STATE_SEND_ID://0x15
//            len = 52;
//            usb_cdc_data.usb_cdc_send_buf[0] = 0xff;
//            usb_cdc_data.usb_cdc_send_buf[1] = len;
//            usb_cdc_data.usb_cdc_send_buf[2] = ENEMY_STATE_SEND_ID;
//						for(uint8_t i=0;i<6;i++)
//						{
//							memcpy(usb_cdc_data.usb_cdc_send_buf+i*8+3 , (uint8_t*)&enemy_state_data[i].id,                sizeof(uint8_t));
//							memcpy(usb_cdc_data.usb_cdc_send_buf+i*8+4 , (uint8_t*)&enemy_state_data[i].x,                sizeof(uint16_t));
//							memcpy(usb_cdc_data.usb_cdc_send_buf+i*8+6 , (uint8_t*)&enemy_state_data[i].y,                sizeof(uint16_t));
//							memcpy(usb_cdc_data.usb_cdc_send_buf+i*8+8 , (uint8_t*)&enemy_state_data[i].hp,                sizeof(uint16_t));
//							memcpy(usb_cdc_data.usb_cdc_send_buf+i*8+10 , (uint8_t*)&enemy_state_data[i].attack_enhance,                sizeof(uint8_t));
//						}
//            usb_cdc_data.usb_cdc_send_buf[51] = 0x0D;
//            break;
				 
        default:
            break;
			}
    usb_data_send(usb_cdc_data.usb_cdc_send_buf,len);
}

float debug_angle_nx = 0; 
float debug_jump = 0;


/*************************** RECV ********************************/ 
void Nuc_data_unpacked()
{	
			if(usb_cdc_data.usb_cdc_rx_buf[0]=='S'&&usb_cdc_data.usb_cdc_rx_buf[1]=='P')
			{
                memcpy(&nuc_receive_data.aim_data_received.crc16,&usb_cdc_data.usb_cdc_rx_buf[27],2);
                if(nuc_receive_data.aim_data_received.crc16==CRC16_Calculate(usb_cdc_data.usb_cdc_rx_buf,27))
                {
                    if(usb_cdc_data.usb_cdc_rx_buf[2]==0)
                    {
                        nuc_receive_data.aim_data_received.success=0;
                    }
                    else if((usb_cdc_data.usb_cdc_rx_buf[2]==1))
                    {
                        nuc_receive_data.aim_data_received.is_fire=0;
                        nuc_receive_data.aim_data_received.success=1;
                        
                    }
                    else
                    {
                        nuc_receive_data.aim_data_received.is_fire=1;
                        nuc_receive_data.aim_data_received.success=1;
                    }
                }
				memcpy(&nuc_receive_data.aim_data_received.yaw,&usb_cdc_data.usb_cdc_rx_buf[3],4);
				memcpy(&nuc_receive_data.aim_data_received.yaw_vel,&usb_cdc_data.usb_cdc_rx_buf[7],4);
                memcpy(&nuc_receive_data.aim_data_received.yaw_acc,&usb_cdc_data.usb_cdc_rx_buf[11],4);
                memcpy(&nuc_receive_data.aim_data_received.pitch,&usb_cdc_data.usb_cdc_rx_buf[15],4);
				memcpy(&nuc_receive_data.aim_data_received.pitch_vel,&usb_cdc_data.usb_cdc_rx_buf[19],4);
                memcpy(&nuc_receive_data.aim_data_received.pitch_acc,&usb_cdc_data.usb_cdc_rx_buf[23],4);                
				
				debug_angle_nx = nuc_receive_data.aim_data_received.yaw - nuc_transmit_data.robot_gimbal_data_send.yaw;
				
				if(nuc_receive_data.aim_data_received.shoot_freq == 0)
				{
					nuc_receive_data.aim_data_received.shoot_freq = 20;
				}
				/* 优化自瞄击打距离,参数根据实际击打情况修改 */
//				if((nuc_receive_data.aim_data_received.distance>0 && nuc_receive_data.aim_data_received.distance<6.5 && nuc_control.action.robot_aim != 2)
//						|| (nuc_receive_data.aim_data_received.distance>0 && nuc_control.action.robot_aim == 2))
//				{
//					nuc_receive_data.aim_data_received.success=1;
//				}
//				else  
//				{
//					nuc_receive_data.aim_data_received.success=0;
//				}
				
				/* 上升沿触发打弹（单发模式） */
				if(nuc_receive_data.aim_data_received.last_fire == 0 && nuc_receive_data.aim_data_received.is_fire == 10)
				{
					nuc_receive_data.aim_data_received.fire_mode = 1;
				}
				else if(nuc_receive_data.aim_data_received.last_fire == 0 && nuc_receive_data.aim_data_received.is_fire == 20)
				{
					nuc_receive_data.aim_data_received.fire_mode = 2;
				}
				else if(nuc_receive_data.aim_data_received.last_fire == 10 && nuc_receive_data.aim_data_received.is_fire == 20)
				{
					nuc_receive_data.aim_data_received.fire_mode = 2;
				}				
				nuc_receive_data.aim_data_received.last_fire=nuc_receive_data.aim_data_received.is_fire;
				
			}
}

/*************************** RECV ********************************/
void float_to_u8(float* float_in,uint8_t* u8_out)
{
    uint8_t farray[4];
    *(float*)farray=*float_in;
    u8_out[3]=farray[3];
    u8_out[2]=farray[2];
    u8_out[1]=farray[1];
    u8_out[0]=farray[0];
}
void u8_to_float(uint8_t* datain,float* dataout)
{
    *dataout=*(float *)datain;
}

void cmd_id_task_create(uint8_t cmd_id,uint16_t freq)
{
	cmd_id_queue.cmd_id_queue[cmd_id_queue.total_num]=cmd_id;
	cmd_id_queue.cmd_id_frq[cmd_id_queue.total_num]=1000/(freq%1000);
  cmd_id_queue.total_num+=1;
}
void cmd_id_queue_handle()
{
		for(int i=0;i<cmd_id_queue.total_num;i++){
				if(cmd_id_queue.now_pos%cmd_id_queue.cmd_id_frq[i]==0){
					data_update(cmd_id_queue.cmd_id_queue[i]);
					send_data_to_nuc(cmd_id_queue.cmd_id_queue[i]);
				}
		}
    cmd_id_queue.now_pos++;
    if(cmd_id_queue.now_pos>=1000){
        cmd_id_queue.now_pos=0;
    }
}

void cmd_id_init()
{
    cmd_id_queue.total_num=0;
    cmd_id_queue.now_pos=0;
}
uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len) 
{
	uint8_t crc = 0xff;
	while (len--) {
		crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}
