#include "Steering_gear_control_task.h" 
#include "RemotDbus.h"
#include "Key.h"
#include "timer.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

RC_ctrl_t *steering_rc;
p_Key_ide  steering_key;


void Steering_Gear_Control_Task(void *params)
{
	steering_rc = (RC_ctrl_t *)get_remote_control_point();
	steering_key = GetKeyStatus(0); 
	for(;;)
	{
		static uint16_t camera_crtl = 500;
		TIM_SetCompare1(TIM2, camera_crtl);
		
		if((steering_rc->key.v & KEY_PRESSED_OFFSET_G))
		{
			//if(camera_crtl >= 2500)
				camera_crtl = 1700;
			//else
			//	camera_crtl += 10;	
		}
		else if(steering_rc->key.v & KEY_PRESSED_OFFSET_B)
		{
		//	if(camera_crtl <= 500)
				camera_crtl = 500;
			//else
			//	camera_crtl -= 10;				
		}
		vTaskDelay(5);
	}
}
