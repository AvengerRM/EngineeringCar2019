#include "small_wheel_control_task.h"
#include "MotorCan.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "timer.h"
#include "math.h"
#include "string.h"

smallwheel_crtl_t smallwheel_crtl; 
void smallwheel_feedback_update(smallwheel_crtl_t *tongs_init);
void smallwheel_set_contorl(smallwheel_crtl_t *tongs_init);
static void smallwheel_control_loop(smallwheel_crtl_t *tongs_crtl_loop);

void smallwheel_init(smallwheel_crtl_t *smallwheel_init)
{
    if (smallwheel_init == NULL)
    {
        return;
    }
	
	const static fp32 motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};	
	smallwheel_init->smallwheel_RC =  get_remote_control_point();
	smallwheel_init->key_p = GetKeyStatus(0); 
	 for (char i = 0; i < 2; i++)
    {
        smallwheel_init->motor_tongs[i].smallwheel_motor_measure = get_Small_Whell_Motor_Measure_Point(i); 
        PID_Init(&smallwheel_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT);
    }
	
	smallwheel_feedback_update(smallwheel_init);
}

void smallwheel_feedback_update(smallwheel_crtl_t *smallwheel_fdb)
{
    if (smallwheel_fdb == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 2; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        smallwheel_fdb->motor_tongs[i].speed = TONGS_MOTOR_RPM_TO_VECTOR_SEN * smallwheel_fdb->motor_tongs[i].smallwheel_motor_measure->speed_rpm;
        smallwheel_fdb->motor_tongs[i].accel = smallwheel_fdb->motor_speed_pid[i].Dbuf[0] * TONGS_CONTROL_FREQUENCE;
    }	
}

void smallwheelControl(void *params)
{
    //空闲一段时间
    vTaskDelay(SMALLWHEEL_TASK_INIT_TIME);
	
	smallwheel_init(&smallwheel_crtl);
	
    //判断底盘电机是否都在线
    while (toe_is_error(DBUSTOE))
    {
        vTaskDelay(SMALLWHEEL_CONTROL_TIME_MS);
    }	
	
	while(1)
	{
        //数据更新
        smallwheel_feedback_update(&smallwheel_crtl);	
		//控制量设置
       // smallwheel_set_contorl(&smallwheel_crtl);
		//底盘控制PID计算
         smallwheel_control_loop(&smallwheel_crtl);
		
		 
		 int16_t smallwheel_set_current[2];
		 if (toe_is_error(DBUSTOE))
		 {
		     memset(smallwheel_set_current,0,sizeof(smallwheel_set_current));
		 }
		 else
		 {
			for(char i = 0;i<2;i++)
			{
				smallwheel_set_current[i] = smallwheel_crtl.motor_tongs[i].give_current;
			}		 
		 }	
		 CanSendMess(CAN2,SEND_ID201_204,smallwheel_set_current); 
		 vTaskDelay(2);
	}
}

void smallwheel_set_contorl(smallwheel_crtl_t *smallwheel_set)
{  
    //短按或长按R
   if(smallwheel_set->key_p[2].Key_Event == KEY_ONE_CLICK ||smallwheel_set->key_p[2].Key_Event == KEY_LONG_CLICK)
   {
		smallwheel_set->motor_tongs[0].speed_set =   0;
	    smallwheel_set->motor_tongs[1].speed_set =   0;	
   }
   else if(smallwheel_set->key_p[2].Key_Event == KEY_DOUBLE_CLICK)
   {
		smallwheel_set->motor_tongs[0].speed_set =   2.0f;
	    smallwheel_set->motor_tongs[1].speed_set =  -2.0f;		
   } 
}

static void smallwheel_control_loop(smallwheel_crtl_t * smallwheel_crtl_loop)
{
    //计算pid
    for (char i = 0; i < 2; i++)
    {
        PID_Calc(&smallwheel_crtl_loop->motor_speed_pid[i], smallwheel_crtl_loop->motor_tongs[i].speed, smallwheel_crtl_loop->motor_tongs[i].speed_set);
    }

    //赋值电流值
    for (char i = 0; i < 2; i++)
    {
        smallwheel_crtl_loop->motor_tongs[i].give_current = (int16_t)(smallwheel_crtl_loop->motor_speed_pid[i].out);
    }	
}



