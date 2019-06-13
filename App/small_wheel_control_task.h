#ifndef _SMALL_WHEEL_CONTROL_TASK_H
#define _SMALL_WHEEL_CONTROL_TASK_H

#include "RemotDbus.h"
#include "user_lib.h"
#include "pid.h"
#include "MotorCAN.h"
#include "Key.h"



//任务开始空闲一段时间
#define SMALLWHEEL_TASK_INIT_TIME 400
// 任务控制间隔 2ms
#define SMALLWHEEL_CONTROL_TIME_MS 2

//底盘电机速度环PID
#define M2006_MOTOR_SPEED_PID_KP 20000.0f
#define M2006_MOTOR_SPEED_PID_KI 2500.0f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT 8000
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


//m3510转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3510_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define TONGS_MOTOR_RPM_TO_VECTOR_SEN M3510_MOTOR_RPM_TO_VECTOR

#define TONGS_CONTROL_FREQUENCE 500.0f


typedef struct
{
  const motor_measure_t *smallwheel_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Small_wheel_Motor_t;

typedef struct
{
  const RC_ctrl_t *smallwheel_RC;                                  
  Small_wheel_Motor_t motor_tongs[2];          
  PidTypeDef motor_speed_pid[2];
  p_Key_ide	key_p;
} smallwheel_crtl_t;

void smallwheelControl(void *params);
#endif
