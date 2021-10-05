#include "timer.h"

TIMER1_Callback cbTimer1;

// Timer base is configured to be 10 kHz
// periodMS sets the period of the interrupt in ms (max value is 6553 ms)
void TIMER1_Init(unsigned int periodMS)  
{
	periodMS = (periodMS * 10) - 1;

	cbTimer1 = 0;

	RCC->APB2ENR	|= 1<<11;		// TIM1 clock enable    
 	TIM1->ARR		 = periodMS;	// Setting counter automatic reload value  
	TIM1->PSC		 = 7199;		// The prescaler 7200 gets the count clock of 10Khz.
	TIM1->DIER		|= 1<<0;   		// Enable update interrupt				
	TIM1->DIER		|= 1<<6;   		// Enable triggered interrupt	   
	TIM1->CR1		|= 0x01;		// Enable timer
	SYS_NVIC_Init(1, 1, TIM1_UP_IRQn, 2);
}  

void TIMER1_SetCallback(TIMER1_Callback cb)
{
	cbTimer1 = cb;
}

// Timer 1 interrupt handler
void TIM1_UP_IRQHandler(void)
{
	if(TIM1->SR & 0x0001)
	{   
		TIM1->SR &= ~(1<<0);
		if (cbTimer1)
		{
			cbTimer1();
		}
	}
}
