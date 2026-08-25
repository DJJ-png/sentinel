#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* =================================================================================
 * 1. 底盘构型选择 (Chassis Configuration Selection)
 * ================================================================================= */

// 只能开启其中一个，决定了运动学解算函数的算法逻辑
//#define CH_CFG_TYPE_MECANUM             // 麦克纳姆轮底盘
#define CH_CFG_TYPE_SWERVE            // 舵轮底盘 (Swerve Drive)
//#define CH_CFG_TYPE_OMNI              // 全向轮底盘 (Omni Drive)


/* =================================================================================
 * 2. 底盘物理参数 (Chassis Physical Parameters) 
 * ================================================================================= */

#define CH_PARAM_WHEEL_TRACK_MM     400.0f  // 轮距: 左右轮中心间距
#define CH_PARAM_WHEEL_BASE_MM      450.0f  // 轴距: 前后轮中心间距
#define CH_PARAM_WHEEL_RADIUS_MM    76.2f   // 轮半径 (如麦轮通常为 152.4mm 直径)
#define CH_PARAM_REDUCTION_RATIO    19.0f   // 底盘电机减速比 (如3508是19:1)

/* =================================================================================
 * 3. 电机配置 (Motor Quantity & Brand)
 * ================================================================================= */

// 轮电机 (Driving Motors)
    #define CH_NUM_WHEEL_MOT            4       // 轮电机总数
    #define CH_CFG_WHEEL_MOT_DJI_3508           // 使用大疆 3508 电机
    //#define CH_CFG_WHEEL_MOT_LK_6015  1       // 若换成达妙则启用此项
 
    //电机物理参数
    #define CH_PARAM_WHEEL_RATIO        13.72   //轮电机减速比
    #define CH_PARAM_WHEEL_R_MM            60      //轮电机半径(mm)
    
    //电机通讯参数
    #define CAN_ID_WHEEL_MOT_ALL              0x200
    #define CAN_ID_WHEEL_MOT_FL               0x202   // Front Left
    #define CAN_ID_WHEEL_MOT_FR               0x201   // Front Right
    #define CAN_ID_WHEEL_MOT_BL               0x204   // Back Left
    #define CAN_ID_WHEEL_MOT_BR               0x203   // Back Right
    
// 舵电机 (Steering Motors - 仅在 SWERVE 模式有效)
#ifdef CH_CFG_TYPE_SWERVE
    #define CH_NUM_STEER_MOT          4
    #define CH_CFG_STEER_MOT_DJI_6020 1     // 用6020作为舵电机
    
    //电机物理参数
    #define CH_PARAM_STEER_RATIO        1   //轮电机减速比
    
    //电机通讯参数
    #define CAN_ID_STEER_MOT_ALL              0x1ff
    #define CAN_ID_STEER_MOT_FL               0x206   // Front Left
    #define CAN_ID_STEER_MOT_FR               0x207   // Front Right
    #define CAN_ID_STEER_MOT_BL               0x208   // Back Left
    #define CAN_ID_STEER_MOT_BR               0x205  // Back Right
#endif

/* =================================================================================
 * 4. 软件限幅与安全阈值 (Limits & Safety)
 * ================================================================================= */

#define CH_LIMIT_MAX_VX_MS          3.5f    // 最大前后线速度 (m/s)
#define CH_LIMIT_MAX_VY_MS          3.5f    // 最大左右线速度 (m/s)
#define CH_LIMIT_MAX_WZ_RADS        6.0f    // 最大自转角速度 (rad/s)

#define CH_LIMIT_MAX_MOTOR_CURR     16000   // 电机控制最大原始电流值
#define CH_LIMIT_OFFLINE_MS         100     // 判定电机离线的超时时间 (ms)

//功率控制相关
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f//无功率控制下功率控制的电流限制
#define POWER_TOTAL_CURRENT_LIMIT       22000.0f//功率控制下功率控制的电流限制
#define WARNING_POWER_BUFF  60.0f
#define POWER_HELM_CURRENT_LIMIT       8000.0f

#endif 

/* =================================================================================
 * 5. CAN 配置 (Communication Identifier)
 * ================================================================================= */
#define CAN_CHMOTOR_MESSAGE          hcan2    //底盘电机挂载的can线
#define CAN_UPTRANS_MESSAGE          hcan1    //上下板挂载的can线
 
// 上下板通讯
#define CAN_ID_BOARD_COM_CH        0x301   // 板间通讯ID 1(底盘相关)
#define CAN_ID_BOARD_COM_REFEREE_1 0x303   // 板间通讯ID 2（裁判系统相关）
#define CAN_ID_BOARD_COM_REFEREE_2 0x304   // 板间通讯ID 3（裁判系统相关）

#define CAN_ID_CAP_COM             0x130   // 超电通讯ID

//视觉需要的数据ID
#define MY_POSITION1_ID 0x30D
#define MY_POSITION2_ID 0x30E
#define ENEMT_STATE_ID 0x30F