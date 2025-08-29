#ifndef DELAY
#define DELAY

#include "stm32f303xc.h"

void TIM3_Init(void);
void Start_Delay_TIM3(uint32_t ms);
void TIM3_IRQHandler(void);
uint8_t Delay_Done(void);

#endif // DELAY
