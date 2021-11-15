#include "encoder.h"

// Uses Timer 4 configured in encoder mode
void ENCODER_Init(void)
{
	RCC->APB1ENR	|= 1<<2;				// TIM4 clock enable
	RCC->APB2ENR	|= 1<<3;				// Enable PORTB clock
	GPIOB->CRL		&= 0x00FFFFFF;			// PB6 PB7
	GPIOB->CRL		|= 0x44000000;			// Floating space input
	
	// Enable the TIM4 update interrupt, to clear overflows. Tobi: section was commented out earlier
	TIM4->DIER		|= 1<<0;				// Enable update interrupt			
	TIM4->DIER		|= 1<<6;				// Enable triggered interrupt
	SYS_NVIC_Init(1, 3, TIM4_IRQn, 1);

	// Configure timer for encoder mode
	TIM4->PSC		 = 0x0;
	TIM4->ARR		 = 65535;				// Setting counter automatic reload value
	TIM4->CR1		&=~(3<<8);				// Select clock divider: no frequency division
	TIM4->CR1		&=~(3<<5);				// Select counting mode: 00: edge alignment mode.  Counter counts up/down according to DIR flag
	
	TIM4->CCMR1		|= 1<<0; 				// 14.4.7: CC1S='01' IC1FP1 Mapping to TI1
	TIM4->CCMR1 	|= 1<<8; 				// 14.4.7: CC2S='01' IC2FP2 Mapping to TI2
	TIM4->CCER 		&= ~(1<<1);	 			// 14.4.9: CC1P='0'	 IC1FP1 Not opposite IC1FP1=TI1
	TIM4->CCER		&= ~(1<<5);	 			// 14.4.9: CC2P='0'	 IC2FP2 Not opposite IC2FP2=TI2
	TIM4->CCMR1		|= 0xf<<4; 				// 14.4.7: IC1F='0000' Input capture 1 filter
	TIM4->SMCR		|= 3<<0;	 			// 15.4.3: SMS='011' All inputs are valid on the rising and falling edges. Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input
	TIM4->CNT		 = ENCODER_ZERO;
	TIM4->CR1		|= 0x01;				// CEN=1 Enabling timer
}

short ENCODER_Read(void)
{
	return -(TIM4->CNT);
}

void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)	// Overflow interruption
	{				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);	// Clear interrupt flag bit	    
}
