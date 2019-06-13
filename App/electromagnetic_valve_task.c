#include "electromagnetic_valve_task.h"
#include "RemotDbus.h"
#include "Key.h"
#include "timer.h"
#include "BasicPeripherals.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

RC_ctrl_t *electromagnetic_rc;
p_Key_ide electromagnetic_key;

extern uint8_t auto_clamp ;
void Electromagnetic_valve_control_task(void *prams)
{
	
	electromagnetic_rc = (RC_ctrl_t *)get_remote_control_point();
	electromagnetic_key = GetKeyStatus(0); 
	
	for(;;)
	{

	   
		if(auto_clamp == 0)
		{
				//V	 
			  if(electromagnetic_key[8].Key_Event== KEY_DOUBLE_CLICK)
			   {
					 GPIO_SetBits(GPIOA,GPIO_Pin_4);	   
			   } 
			   else
			   {
					GPIO_ResetBits(GPIOA,GPIO_Pin_4);	 
			   }			
		}
		//X
	   if(electromagnetic_key[6].Key_Event== KEY_DOUBLE_CLICK)
	   {
		 	 GPIO_SetBits(GPIOF,GPIO_Pin_10);	 
	   } 
	   else
	   {
			GPIO_ResetBits(GPIOF,GPIO_Pin_10);			   
	   }
	   
	   	//C
	  if(electromagnetic_key[7].Key_Event== KEY_DOUBLE_CLICK)
	   {
		 	 GPIO_SetBits(GPIOA,GPIO_Pin_5);	 
	   }
	   else
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);	
		}

	
	    //B			 
		if(electromagnetic_key[9].Key_Event== KEY_DOUBLE_CLICK)
	   {
		 	 GPIO_SetBits(GPIOC,GPIO_Pin_1);	 	
	   }
	   else
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_1);
		}
	 
		vTaskDelay(2);
	}
}

