#ifndef _KEY_H_
#define _KEY_H_

#include "sys.h"


typedef enum{
	  KEY_NO_CLICK = 0 ,
      KEY_ONE_CLICK    ,
	  KEY_DOUBLE_CLICK ,
	  KEY_LONG_CLICK   ,
}key_event;

typedef enum{
      KEY_UP = 0,
	  KEY_DOWN ,
	  KEY_HOLE ,
	  KEY_CLICK,
	  KEY_ONE,
	  KEY_TWO,
	  KEY_LONG
}key_state;

typedef struct KEY_
{
	key_event Key_Event;
	key_state Key_State;
	int16_t cnt;
}Key_ide,*p_Key_ide;

void Key_Scan(uint16_t Key_Value,p_Key_ide key_id);
void Key_RC_Init(void);

#endif

