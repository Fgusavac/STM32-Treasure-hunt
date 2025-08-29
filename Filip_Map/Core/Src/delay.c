#include "delay.h"

volatile uint8_t delay_done = 0;

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 71999 - 1;  // 10kHz
    // TIM3->ARR = 1000 - 1; // 1 second delay (10000 counts of 0.1ms = 1s)

    TIM3->DIER |= TIM_DIER_UIE;  // Enable update interrupt

    NVIC_SetPriority(TIM3_IRQn, 1);
    NVIC_EnableIRQ(TIM3_IRQn);

}

void Start_Delay_TIM3(uint32_t ms) {
    delay_done = 0;

    TIM3->CR1 &= ~TIM_CR1_CEN;  // Stop timer first
    TIM3->ARR = ms - 1;  // Set new ARR

    TIM3->CNT = 0;
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->CR1 |= TIM_CR1_CEN;   // Start timer
}

void TIM3_IRQHandler(void) {
	TIM3->CR1 &= ~TIM_CR1_CEN;  // Stop timer

	delay_done = 1;  // Mark delay done

	TIM3->SR &= ~TIM_SR_UIF;  // Clear flag

}

uint8_t Delay_Done(void) {
    return delay_done;
}
