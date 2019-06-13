#include "Key.h"
#include "RemotDbus.h"

RC_ctrl_t *Key_RC;

void Key_RC_Init()
{
	Key_RC = (RC_ctrl_t *)get_remote_control_point();
}

void Key_Scan(uint16_t Key_Value,p_Key_ide key_id)
{
	
	switch(key_id->Key_State)
	{
		case KEY_UP :
			if(Key_RC->key.v == Key_Value)
				key_id->Key_State =    KEY_DOWN;			
		break;

		case KEY_DOWN :
			if(Key_RC->key.v  == Key_Value)
				key_id->Key_State =    KEY_HOLE;
			else
				key_id->Key_State =    KEY_UP; 	//¸ÉÈÅÐÅºÅ							 											
		break;

		case KEY_HOLE :
			key_id->cnt++;
			if(key_id->cnt < 300)
			{
				if(Key_RC->key.v  != Key_Value)
				{ 
					key_id->cnt = 0;
					key_id->Key_State =    KEY_CLICK;       
				}
			}
			else 
			{
				key_id->cnt = 0;
				key_id->Key_State =    KEY_LONG;									 
		}
		break;

		case KEY_CLICK:
			(key_id->cnt)++;
			if(key_id->cnt < 30)
			{
				if(Key_RC->key.v  == Key_Value)
				{
					key_id->cnt = 0;
					key_id->Key_State =    KEY_TWO;  
				}
			}
			else
			{
				key_id->cnt = 0;
				key_id->Key_State =    KEY_ONE;  
			}
		break;


		case KEY_ONE :
			key_id->Key_Event = KEY_ONE_CLICK;
			key_id->Key_State =    KEY_UP;							
			break;								 

		case KEY_TWO :
			key_id->Key_Event = KEY_DOUBLE_CLICK;
			if(Key_RC->key.v  != Key_Value)
				key_id->Key_State =    KEY_UP;							
			break;


		case KEY_LONG :
			key_id->Key_Event = KEY_LONG_CLICK;
			if(Key_RC->key.v != Key_Value)
			key_id->Key_State =    KEY_UP;								
			break;					
	}				
}