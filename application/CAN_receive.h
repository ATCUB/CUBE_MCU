

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

//#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

#define READY_RESTORE  0
#define FINISH_RESTORE 1
#define RUNING_RESTORE 2


/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

/**
  * 模式
**/
typedef enum{
	/*****************************************************外部输入量***********信号发生器(可控的运动参数)*****/
	//测试
	Motor_Mode_Debug_Location		= 0x00,	//调试位置				NULL									多功能调试信号源						
	Motor_Mode_Debug_Speed			= 0x01,	//调试速度				NULL									多功能调试信号源						
	//停止
	Control_Mode_Stop						= 0x10,	//停止						NULL									NULL												NULL
	//DIG(CAN/RS485)
	Motor_Mode_Digital_Location	= 0x20,	//DIG位置					目标位置							位置跟踪器(速度,双加速度)		
	Motor_Mode_Digital_Speed		= 0x21,	//DIG速度					目标速度							速度跟踪器(双加速度)				
	Motor_Mode_Digital_Current	= 0x22,	//DIG电流					目标电流							电流跟踪器(双电梯度)				
	Motor_Mode_Digital_Track		= 0x23,	//DIG轨迹					多指令								运动重构器(运动自寻)
	Motor_Mode_Digital_Debug = 0x24, //Debug模式
	//MoreIO(PWM/PUL)
	Motor_Mode_PWM_Location			= 0x30,	//PWM位置_舵机		目标位置							位置跟踪器(速度,双加速度)		
	Motor_Mode_PWM_Speed				= 0x31,	//PWM速度_调速机	目标速度							速度跟踪器(双加速度)			
	Motor_Mode_PWM_Current			= 0x32,	//PWM电流_直流机	目标电流							电流跟踪器(双电梯度)			
	Motor_Mode_PULSE_Location		= 0x33,	//PULSE位置				目标位置							位置插补器(无)						
}Motor_Mode;

/**
  * 控制器状态
**/
typedef enum{
	Control_State_Stop				= 0x00,	//停止
	Control_State_Finish			= 0x01,	//任务完成
	Control_State_Running			= 0x02,	//任务执行中
	Control_State_Overload		= 0x03,	//过载
	Control_State_Stall				= 0x04,	//堵转
}Motor_State;


//rm motor data
typedef struct
{
    int32_t  est_perlap_speed;
    int32_t  real_lap_location;
    Motor_Mode   mode_run;
    Motor_State   state;
    bool_t  rectify_valid;
} motor_measure_t;


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_motor(int32_t motor, int8_t speed);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
