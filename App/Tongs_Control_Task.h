#ifndef TONG_CONTROL_TASK_H
#define TONG_CONTROL_TASK_H
#include "RemotDbus.h"
#include "user_lib.h"
#include "pid.h"
#include "MotorCAN.h"
#include "Key.h"


//任务开始空闲一段时间
#define TOOGS_TASK_INIT_TIME 3000
// 任务控制间隔 2ms
#define TOOGS_CONTROL_TIME_MS 2
//底盘电机速度环PID
#define M3510_MOTOR_SPEED_PID_KP_3510 20000.0f
#define M3510_MOTOR_SPEED_PID_KI_3510 1200.0f
#define M3510_MOTOR_SPEED_PID_KD_3510 0.0f
#define M3510_MOTOR_SPEED_PID_MAX_OUT 8000
#define M3510_MOTOR_SPEED_PID_MAX_IOUT 5000.0f


//m3510转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3510_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define TONGS_MOTOR_RPM_TO_VECTOR_SEN M3510_MOTOR_RPM_TO_VECTOR

#define TONGS_CONTROL_FREQUENCE 500.0f

typedef struct _POSITION_PID
{
  float position_kp ;
	float velocity_kp;
	float ange_get;
	float ange_set;
	float  init_angle;
	float lang_init_angle;
	int16_t  encoder_val_new_data;
	int16_t  encoder_val_old_data;
	long int encoder_count;
	float increment;
	float position_limit;
	int16_t speed_get;
	int16_t out;
}position_pid ,*p_position_pid;


typedef struct
{
  const motor_measure_t *toogs_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Tongs_Motor_t;

typedef struct
{
  const RC_ctrl_t *tongs_RC;                                  
  Tongs_Motor_t motor_tongs[3];          
  PidTypeDef motor_speed_pid[3];
  position_pid motor_position_pid[3];
  p_Key_ide	key_p;

} tongs_crtl_t;


void ToogsControl(void *params);
#endif

