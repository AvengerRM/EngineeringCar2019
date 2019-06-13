#ifndef TIMER_H
#define TIMER_H
#include "main.h"
#include "Key.h"
extern void TIM1_Init(uint16_t arr, uint16_t psc);
extern void TIM3_Init(uint16_t arr, uint16_t psc);
extern void TIM6_Init(uint16_t arr, uint16_t psc);
extern void TIM12_Init(uint16_t arr, uint16_t psc);
void TIM5_Config(void);
Key_ide *GetKeyStatus(char index);
#endif
