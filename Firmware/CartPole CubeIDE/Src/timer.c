#include "timer.h"

TIMER1_Callback cbTimer1;
long timer_resets = 0;
unsigned int periodMS = 0;

// Timer base is configured to be 1MHz (resolution of 1us)
// periodMS sets the period of the interrupt in ms (max value is 6553 ms)
void TIMER1_Init(unsigned int _periodMS)
{
	periodMS = _periodMS;

	cbTimer1 = 0;

	RCC->APB2ENR	|= 1<<11;					// TIM1 clock enable
 	TIM1->ARR		 = (periodMS * 1000) - 1;	// Setting counter automatic reload value
	TIM1->PSC		 = 72;						// The prescaler 7200 gets the count clock of 1Mhz.
	TIM1->DIER		|= 1<<0;   					// Enable update interrupt
	TIM1->DIER		|= 1<<6;   					// Enable triggered interrupt
	TIM1->CR1		|= 0x01;					// Enable timer
	SYS_NVIC_Init(1, 1, TIM1_UP_IRQn, 2);
}  

void TIMER1_ChangePeriod(unsigned int _periodMS)
{
	timer_resets = 0;
	periodMS = _periodMS;
 	TIM1->ARR		 = (periodMS * 1000) - 1;	// Setting counter automatic reload value
}

void TIMER1_SetCallback(TIMER1_Callback cb)
{
	cbTimer1 = cb;
}

// Timer base is configured to be 1MHz (resolution of 1us)
float TIMER1_getSystemTime() {
    return ((float)timer_resets + (TIM1->SR ? 1.0 : 0.0)) * 1e-3 * periodMS + ((float)TIM1->CNT) * 1e-6;
}

// unsigned long max value: 4'294'967'295us = 4'294.967'295s ~ 60min
unsigned int TIMER1_getSystemTime_Us() {
    return (timer_resets + (TIM1->SR ? 1 : 0)) * 1000 * periodMS + TIM1->CNT;
}

// Timer 1 interrupt handler
void TIM1_UP_IRQHandler(void)
{
	if(TIM1->SR & 0x0001)
	{
        timer_resets += 1;
		TIM1->SR &= ~(1<<0);
		if (cbTimer1) {
			cbTimer1();
		}
	}
}
