

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
  * ģʽ
**/
typedef enum{
	/*****************************************************�ⲿ������***********�źŷ�����(�ɿص��˶�����)*****/
	//����
	Motor_Mode_Debug_Location		= 0x00,	//����λ��				NULL									�๦�ܵ����ź�Դ						
	Motor_Mode_Debug_Speed			= 0x01,	//�����ٶ�				NULL									�๦�ܵ����ź�Դ						
	//ֹͣ
	Control_Mode_Stop						= 0x10,	//ֹͣ						NULL									NULL												NULL
	//DIG(CAN/RS485)
	Motor_Mode_Digital_Location	= 0x20,	//DIGλ��					Ŀ��λ��							λ�ø�����(�ٶ�,˫���ٶ�)		
	Motor_Mode_Digital_Speed		= 0x21,	//DIG�ٶ�					Ŀ���ٶ�							�ٶȸ�����(˫���ٶ�)				
	Motor_Mode_Digital_Current	= 0x22,	//DIG����					Ŀ�����							����������(˫���ݶ�)				
	Motor_Mode_Digital_Track		= 0x23,	//DIG�켣					��ָ��								�˶��ع���(�˶���Ѱ)
	Motor_Mode_Digital_Debug = 0x24, //Debugģʽ
	//MoreIO(PWM/PUL)
	Motor_Mode_PWM_Location			= 0x30,	//PWMλ��_���		Ŀ��λ��							λ�ø�����(�ٶ�,˫���ٶ�)		
	Motor_Mode_PWM_Speed				= 0x31,	//PWM�ٶ�_���ٻ�	Ŀ���ٶ�							�ٶȸ�����(˫���ٶ�)			
	Motor_Mode_PWM_Current			= 0x32,	//PWM����_ֱ����	Ŀ�����							����������(˫���ݶ�)			
	Motor_Mode_PULSE_Location		= 0x33,	//PULSEλ��				Ŀ��λ��							λ�ò岹��(��)						
}Motor_Mode;

/**
  * ������״̬
**/
typedef enum{
	Control_State_Stop				= 0x00,	//ֹͣ
	Control_State_Finish			= 0x01,	//�������
	Control_State_Running			= 0x02,	//����ִ����
	Control_State_Overload		= 0x03,	//����
	Control_State_Stall				= 0x04,	//��ת
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
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
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
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_motor(int32_t motor, int8_t speed);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
