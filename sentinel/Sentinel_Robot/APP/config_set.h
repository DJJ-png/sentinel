#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/****************************************µç»ú²ÎÊýÅäÖÃ********************************************/
#define GIMBAL_MOTOR_BASE_ECD_RANGE 65535
#define GIMBAL_MOTOR_ADVANCED_ECD_RANGE 8191
#define GIMBAL_MOTOR_PITCH_ECD_RANGE 65535
/****************************************ÁãµãÅäÖÃ********************************************/
#define CHASSIS_FOLLOW_BASE_GIMBAL_ANGLE_ZERO 15355
#define ADVANCED_GIMBAL_ANGLE_ZERO 8177
#define PITCH_ANGLE_ZERO 17572

/* Ð¡YAWãÐÖµ  */
#define ADVANCED_YAW_ANFLE_MAX 25
#define ADVANCED_YAW_ANFLE_MIN -25
/****************************************ãÐÖµÅäÖÃ********************************************/

#define DEFAULT_YAW_SPIN_SPEED      -200

/* ************************************* Ò£¿ØÆ÷ºê ******************************************** */
#define RollWheel							rc_ctrl.rc.ch[4]			//Ò£¿ØÆ÷×ó²à¹öÂÖ£¬×ó¡úÓÒ£º660¡ú-660
#define RockingBar_Left_V			rc_ctrl.rc.ch[3]			//Ò£¿ØÆ÷×óÒ¡¸Ë£¨´¹Ö±£©£¬ÉÏ¡úÏÂ£º660¡ú-660
#define RockingBar_Left_H			rc_ctrl.rc.ch[2]			//Ò£¿ØÆ÷×óÒ¡¸Ë£¨Ë®Æ½£©£¬×ó¡úÓÒ£º-660¡ú660
#define RockingBar_Right_V		rc_ctrl.rc.ch[1]			//Ò£¿ØÆ÷ÓÒÒ¡¸Ë£¨´¹Ö±£©£¬ÉÏ¡úÏÂ£º660¡ú-660
#define RockingBar_Right_H		rc_ctrl.rc.ch[0]			//Ò£¿ØÆ÷ÓÒÒ¡¸Ë£¨Ë®Æ½£©£¬×ó¡úÓÒ£º-660¡ú660
#define Switch_Left						rc_ctrl.rc.s[1]				//Ò£¿ØÆ÷×ó²¦¸Ë£¬ÉÏ1£¬ÖÐ3£¬ÏÂ2
#define Switch_Right					rc_ctrl.rc.s[0]				//Ò£¿ØÆ÷ÓÒ²¦¸Ë£¬ÉÏ1£¬ÖÐ3£¬ÏÂ2
/****************************************pidÅäÖÃ********************************************/
//base yaw speed
#define BASE_YAW_MOTOR_SPEED_PID_KP 10.2f
#define BASE_YAW_MOTOR_SPEED_PID_KI 0.1f
#define BASE_YAW_MOTOR_SPEED_PID_KD 0.0f
#define BASE_YAW_MOTOR_SPEED_PID_MAX_OUT 1800.0f
#define BASE_YAW_MOTOR_SPEED_PID_MAX_IOUT 110.0f
//base yaw angle
#define BASE_YAW_MOTOR_ANGLE_PID_KP 20.2f
#define BASE_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define BASE_YAW_MOTOR_ANGLE_PID_KD 70.0f
#define BASE_YAW_MOTOR_ANGLE_PID_MAX_OUT 1500.0f
#define BASE_YAW_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

//advanced yaw speed
#define ADVANCED_YAW_MOTOR_SPEED_PID_KP 300.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_KI 0.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_KD 0.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_MAX_OUT 15000.0f
#define ADVANCED_YAW_MOTOR_SPEED_PID_MAX_IOUT 8000.0f

//advanced yaw angle
#define ADVANCED_YAW_MOTOR_ANGLE_PID_KP 8.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_KD 1.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_OUT 500.0f
#define ADVANCED_YAW_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

//pitch yaw speed
#define PITCH_MOTOR_SPEED_PID_KP 15.0f			
#define PITCH_MOTOR_SPEED_PID_KI 0.026f				
#define PITCH_MOTOR_SPEED_PID_KD 0.0f				
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 1500.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 100.0f

//pitch yaw angle
#define PITCH_MOTOR_ANGLE_PID_KP 20.2f			 
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 100.0f    	
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 1500.0f	
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 0.0f
/*aim_pid*/

//base yaw aim_speed
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KP 8.2f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KI 0.1f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_KD 0.0f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT 1800.0f
#define BASE_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT 110.0f

//base yaw aim_angle
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KP 16.2f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KI 0.0f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KD 70.0f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT 1500.0f
#define BASE_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT 0.0f

//advanced yaw_aim_speed
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KP 225.0f//250.0f
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KI 1.0f
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_KD 0.0f
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT 15000.0f
#define ADVANCED_YAW_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT 8000.0f

//advanced yaw_aim_angle
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KP 22.5f//22.5//27//20
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KI 0.0f
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_KD 15.0f
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT 1000.0f//1000//2500
#define ADVANCED_YAW_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT 0.0f

//pitch yaw_aim_speed
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_KP 16.0f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_KI 0.046f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_KD 0.0f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_MAX_OUT 2500.0f
#define PITCH_MOTOR_SPEED_AUTO_AIM_PID_MAX_IOUT 100.0f

//pitch yaw_aim_angle
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KP 18.2f   
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_KD 80.0f    
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_MAX_OUT 2500.0f
#define PITCH_MOTOR_ANGLE_AUTO_AIM_PID_MAX_IOUT 0.0f
/****************************************×´Ì¬Ã¶¾Ù********************************************/
enum GIMBAL_JOINT{
    BASE_YAW,
    PITCH,
    ADVANCED_YAW,
};

enum MOTOR_MODE{
    SPEED,
    ANGLE,
};

enum GIMBAL_MOTOR{
    BASE_YAW_5010,
    PITCH_6015,
    ADVANCED_YAW_6020,
};

enum GIMBAL_MODE_CNOTROL {
    BASE_YAW_MODE,
		ADVANCED_YAW_MODE,
		GIMBAL_TO_CHASSIS_MODE,
};
#endif 