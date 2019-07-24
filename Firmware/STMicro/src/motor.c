#include "motor.h"

// PWM base frequency is 10 kHz
void MOTOR_Init(void)
{
	RCC->APB2ENR	|= 1<<3;		// PORTB Clock enable  
	GPIOB->CRH		&= 0x0000FFFF;	// PORTB12 13 14 15 push-pull
	GPIOB->CRH		|= 0x22220000;	// PORTB12 13 14 15 push-pull

	RCC->APB1ENR	|= 1<<1;		// TIM3 clock enable    
	RCC->APB2ENR	|= 1<<3;		// PORTB clock enable   
	GPIOB->CRL		&= 0XFFFFFF00;	// PORTB0 1 multiplexing output
	GPIOB->CRL		|=0X000000BB;   // PORTB0 1 Multiplexed output
	TIM3->ARR		 = 7199;		// Setting counter automatic reload value
	TIM3->PSC		 = 0;			// Prescaler non frequency division
	TIM3->CCMR2		|= 6<<12;		// CH4 PWM1 mode	
	TIM3->CCMR2		|= 6<<4;		// CH3 PWM1 mode	
	TIM3->CCMR2		|= 1<<11;		// CH4 Pre load enable energy	 
	TIM3->CCMR2		|= 1<<3; 		// CH3 Pre load enable energy
	TIM3->CCER		|= 1<<12;		// CH4 Output enable   
	TIM3->CCER		|= 1<<8;		// CH3 Output enable	
	TIM3->CR1		 = 0x80;		// ARPE Make energy 
	TIM3->CR1		|= 0x01;		// Enabling timer 3 

	PB_OUT(13) 		= 0;			// AIN1
	PB_OUT(12) 		= 0;			// AIN2
	PB_OUT(14) 		= 0;			// BIN1
	PB_OUT(15) 		= 0;			// BIN2
	TIM3->CCR3 		= 0;
	TIM3->CCR4 		= 0;
}

void MOTOR_Stop(void)
{
    TIM3->CCR4 = 0;
	PB_OUT(13) = 0;					// AIN1
	PB_OUT(12) = 0;					// AIN2
}

void MOTOR_SetSpeed(int speed)
{
    if (speed == 0)
    {
        PB_OUT(13) = 0;					// AIN1
        PB_OUT(12) = 0;					// AIN2
    }
	// Set direction (values less than 0 indicate left)
	else if (speed < 0)
	{
		PB_OUT(13) = 0;				// AIN1
		PB_OUT(12) = 1;				// AIN2
		speed = -speed;
	}
	else
	{
		PB_OUT(13) = 1;				// AIN1
		PB_OUT(12) = 0;				// AIN2
	}
	
    speed = (speed > MOTOR_FULL_SCALE) ? MOTOR_FULL_SCALE : speed;
	TIM3->CCR4 = speed;
}
