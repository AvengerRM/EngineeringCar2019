#include "Tongs_Control_Task.h"
#include "MotorCan.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "timer.h"
#include "math.h"
#include "string.h"

tongs_crtl_t tongs_crtl; 
void tongs_feedback_update(tongs_crtl_t *tongs_init);
void toogs_set_contorl(tongs_crtl_t *tongs_init);
static void toogs_control_loop(tongs_crtl_t *tongs_crtl_loop);

void tongs_init(tongs_crtl_t *tongs_init)
{
    if (tongs_init == NULL)
    {
        return;
    }
	
	const static fp32 motor_speed_pid[3] = {M3510_MOTOR_SPEED_PID_KP_3510, M3510_MOTOR_SPEED_PID_KI_3510, M3510_MOTOR_SPEED_PID_KD_3510};	
	tongs_init->tongs_RC =  get_remote_control_point();
	tongs_init->key_p = GetKeyStatus(0); 
	
	for(char i = 0;i<3;i++)
	{
		memset(&tongs_init->motor_position_pid[i],0,sizeof(tongs_init->motor_position_pid[i]));
	}
	 for (char i = 0; i < 3; i++)
    {
		tongs_init->motor_position_pid[i].position_kp  = 0.1f;
        tongs_init->motor_tongs[i].toogs_motor_measure = get_Tongs_Motor_Measure_Point(i); 
        PID_Init(&tongs_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3510_MOTOR_SPEED_PID_MAX_OUT, M3510_MOTOR_SPEED_PID_MAX_IOUT);
    }
	
	tongs_feedback_update(tongs_init);
}

long int encoder_value_for_Moter(p_position_pid encoder,int16_t input_data)
{
	encoder->encoder_val_old_data = encoder->encoder_val_new_data;
	encoder->encoder_val_new_data = input_data;
	
	if((encoder->encoder_val_new_data - encoder->encoder_val_old_data) < -6000)
		encoder->encoder_count++;
	else if((encoder->encoder_val_new_data - encoder->encoder_val_old_data) > 6000)
		encoder->encoder_count--;
	return encoder->encoder_val_new_data + encoder->encoder_count * 8191;
}

void tongs_feedback_update(tongs_crtl_t *tongs_init)
{
    if (tongs_init == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        tongs_init->motor_tongs[i].speed = TONGS_MOTOR_RPM_TO_VECTOR_SEN * tongs_init->motor_tongs[i].toogs_motor_measure->speed_rpm;
        tongs_init->motor_tongs[i].accel = tongs_init->motor_speed_pid[i].Dbuf[0] * TONGS_CONTROL_FREQUENCE;
		
		//转化为弧度
		tongs_init->motor_position_pid[i].ange_get = encoder_value_for_Moter(&tongs_init->motor_position_pid[i],tongs_init->motor_tongs[i].toogs_motor_measure->ecd) * 0.000767;
    }	
}

void ToogsControl(void *params)
{
    //空闲一段时间
    vTaskDelay(TOOGS_TASK_INIT_TIME);
	Key_RC_Init();
	tongs_init(&tongs_crtl);
	tongs_feedback_update(&tongs_crtl);	
	for(char i = 0;i<3;i++)
	{
		tongs_crtl.motor_position_pid[i].ange_set = tongs_crtl.motor_position_pid[i].ange_get;
		tongs_crtl.motor_position_pid[i].init_angle = tongs_crtl.motor_position_pid[i].ange_get;
	}
	
    //判断底盘电机是否都在线
//    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE) || toe_is_error(DBUSTOE))
//    {
//        vTaskDelay(TOOGS_CONTROL_TIME_MS);
//    }	
	
	while(1)
	{
        //数据更新

        tongs_feedback_update(&tongs_crtl);	
		//控制量设置
        toogs_set_contorl(&tongs_crtl);
		//底盘控制PID计算
        toogs_control_loop(&tongs_crtl);
		
		 
		 int16_t toogs_set_current[3];
		 if (toe_is_error(DBUSTOE))
		 {
		     memset(toogs_set_current,0,sizeof(toogs_set_current));
		 }
		 else
		 {
			for(char i = 0;i<3;i++)
			{
				toogs_set_current[i] = tongs_crtl.motor_tongs[i].give_current;
			}		 
		 }
		// toogs_set_current[0] = 500; 
		CanSendMess(CAN1,SEND_ID205_207,toogs_set_current); 
		vTaskDelay(1);
	}
}


typedef enum{
init_status_e,
clamp_caisson_e,
clamp_caisson1_e,	
take_back_toogs_e,
throw_bullet_e,
lay_back_caisson_e	
}Toogs_Status;

Toogs_Status toogs_sta = init_status_e;
 int16_t timer_cnt = 0;
uint8_t auto_clamp = 0;
void toogs_set_contorl(tongs_crtl_t *tongs_set)
{
   tongs_set->motor_position_pid[0].position_limit = -100.0f;
   tongs_set->motor_position_pid[1].position_limit = 50.0f;
	
  static uint8_t catch_mode  = 0;
	if(tongs_set->key_p[5].Key_Event == KEY_DOUBLE_CLICK)
	{
		catch_mode = 1;
	}
	else
	{
		catch_mode = 0;
	}
		
	   switch(toogs_sta)
	   {
		   case init_status_e:
			    if(tongs_set->motor_position_pid[0].ange_set <= tongs_set->motor_position_pid[0].init_angle)
				{
					tongs_set->motor_position_pid[0].ange_set += 5.0f * 0.017f;	
				}
				auto_clamp = 0;
				
				if(catch_mode == 1)
				{
					if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_Q)
					{
						auto_clamp = 1;
						toogs_sta = clamp_caisson_e;
					}				
				}
				else
				{
					if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_Q)
					{
						auto_clamp = 1;
						toogs_sta = clamp_caisson1_e;						
					}
				
				}

			   break;
		   case clamp_caisson1_e:
		   {
			   static uint16_t cnt = 0,flag = 0;
			    if(tongs_set->motor_position_pid[0].ange_get >= tongs_set->motor_position_pid[0].init_angle - 130.0f)
				{
					tongs_set->motor_position_pid[0].ange_set -= 5.0f * 0.017f;
				}
				else
				{
					
					if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_E)
					{
						flag = 1;
						GPIO_SetBits(GPIOA,GPIO_Pin_4);
					}
					
					if(flag == 1)
					{
						cnt++;
						if(cnt >= 800)
						{
							flag = 0;
							cnt = 0;
							toogs_sta = throw_bullet_e;
						}
					}	
				}
				
			   if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_R)
				{
					cnt  = 0;
					flag = 0;
					GPIO_ResetBits(GPIOI,GPIO_Pin_9);
					GPIO_ResetBits(GPIOA,GPIO_Pin_4);
					toogs_sta = init_status_e;
				}					
		   }
		   break;
		   case clamp_caisson_e:
		   {
			   static uint16_t cnt = 0,flag = 0;
		   
			    if(tongs_set->motor_position_pid[0].ange_get >= tongs_set->motor_position_pid[0].init_angle - 130.0f)
				{
					tongs_set->motor_position_pid[0].ange_set -= 5.0f * 0.017f;
				}
				else
				{
					//到达夹块位置，按下E进行夹块动作
					 if(flag == 0)
					 {
						  if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_E)
						  {
							 flag = 1;
						  }					 
					 }
					 else if(flag == 1)
					 {
						cnt++;
						if(cnt>= 500)
						{
							cnt = 0;
							flag = 2;
							//伸出抓手
							GPIO_SetBits(GPIOI,GPIO_Pin_9);
						}					 
					 }
					 else if(flag == 2)
					 {
						cnt++;
						 if(cnt >= 500)
						 {
							cnt  = 0;
							flag = 3;
							 //夹紧物块
							GPIO_SetBits(GPIOA,GPIO_Pin_4);
						 }
					 }
					else
					{
						 cnt++;
						 if(cnt >= 500)
						 {
							cnt  = 0;
							flag = 0;
							toogs_sta = take_back_toogs_e;
						 }						
					}
				}
				
			   if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_R)
				{
					cnt  = 0;
					flag = 0;
					GPIO_ResetBits(GPIOI,GPIO_Pin_9);
					GPIO_ResetBits(GPIOA,GPIO_Pin_4);
					toogs_sta = init_status_e;
				}		   
		   }	
			   break;
		   case take_back_toogs_e:
		   {		   
				if(tongs_set->motor_position_pid[0].ange_get <= tongs_set->motor_position_pid[0].init_angle - 100.0f)
				{
					tongs_set->motor_position_pid[0].ange_set += 5.0f * 0.017f;
				}
				else
				{ 
					//收回抓手
					GPIO_ResetBits(GPIOI,GPIO_Pin_9);
					timer_cnt++;
					if(timer_cnt >= 800)
					{
						timer_cnt = 0;
						toogs_sta = throw_bullet_e;
					}
				}
		   }
		   break;
		   case  throw_bullet_e:
		   {
				if(tongs_set->motor_position_pid[0].ange_get <= tongs_set->motor_position_pid[0].init_angle)
				{
					tongs_set->motor_position_pid[0].ange_set += 5.0f * 0.017f;
				}
				else
				{
					static uint16_t cnt = 0;
					cnt++;
					if(cnt >= 1000)
					{
						cnt = 0;
						toogs_sta = lay_back_caisson_e;
					}	
				}
			}
			   break;
		   case lay_back_caisson_e:
		   {
			   static uint16_t cnt = 0;
				if(tongs_set->motor_position_pid[0].ange_get >= tongs_set->motor_position_pid[0].init_angle - 130.0f)
				{
					tongs_set->motor_position_pid[0].ange_set -= 5.0f * 0.017f;
				}
				else
				{
					cnt++;
					if(cnt>= 800)
					{
						cnt =0;
						GPIO_ResetBits(GPIOA,GPIO_Pin_4);
						toogs_sta = init_status_e;
					}
				}				
		   }
		  break;
		   default:
			   break;
	   }
   
	if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_SHIFT)
	{
		tongs_set->motor_position_pid[1].increment += 5.0f * 0.017f;
	}
	else if(tongs_crtl.tongs_RC->key.v == KEY_PRESSED_OFFSET_CTRL)
	{
		tongs_set->motor_position_pid[1].increment -= 5.0f * 0.017f;
	}
	 
	 tongs_set->motor_position_pid[1].ange_set =   tongs_set->motor_position_pid[1].init_angle + tongs_set->motor_position_pid[1].increment; 		 
}

static void toogs_control_loop(tongs_crtl_t *tongs_crtl_loop)
{
	for(char i = 0;i<3;i++)
	{
		tongs_crtl_loop->motor_tongs[i].speed_set = (tongs_crtl_loop->motor_position_pid[i].ange_set - tongs_crtl_loop->motor_position_pid[i].ange_get) *tongs_crtl_loop->motor_position_pid[i].position_kp;
	}
	
    //计算pid
    for (char i = 0; i < 3; i++)
    {
        PID_Calc(&tongs_crtl_loop->motor_speed_pid[i], tongs_crtl_loop->motor_tongs[i].speed, tongs_crtl_loop->motor_tongs[i].speed_set);
    }

    //赋值电流值
    for (char i = 0; i < 3; i++)
    {
        tongs_crtl_loop->motor_tongs[i].give_current = (int16_t)(tongs_crtl_loop->motor_speed_pid[i].out);
    }	
}



