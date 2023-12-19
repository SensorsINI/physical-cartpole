#include "motor_stm.h"
#include <stdlib.h>

void set_direction(int pwm_duty_cycle_in_clock_cycles);
void set_power_magnitude(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);

// PWM base frequency is 10 kHz
void Motor_Init_STM(int pwm_period_in_clock_cycles)
{
	RCC->APB2ENR	|= 1<<3;		// PORTB Clock enable  
	GPIOB->CRH		&= 0x0000FFFF;	// PORTB12 13 14 15 push-pull
	GPIOB->CRH		|= 0x22220000;	// PORTB12 13 14 15 push-pull

	RCC->APB1ENR	|= 1<<1;		// TIM3 clock enable    
	RCC->APB2ENR	|= 1<<3;		// PORTB clock enable   
	GPIOB->CRL		&= 0XFFFFFF00;	// PORTB0 1 multiplexing output
	GPIOB->CRL		|=0X000000BB;   // PORTB0 1 Multiplexed output
	TIM3->ARR		 = pwm_period_in_clock_cycles-1;		// Setting counter automatic reload value
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

void Motor_SetPwmPeriod_STM(int pwm_period_in_clock_cycles){
	TIM3->ARR		 = pwm_period_in_clock_cycles-1;		// Setting counter automatic reload value
}

void Motor_Stop_STM(void)
{
    TIM3->CCR4 = 0;
	PB_OUT(13) = 0;					// AIN1
	PB_OUT(12) = 0;					// AIN2
}


void Motor_SetPower_STM(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles)
{
	set_direction(pwm_duty_cycle_in_clock_cycles);
	set_power_magnitude(pwm_duty_cycle_in_clock_cycles, pwm_period_in_clock_cycles);
}


void set_direction(int pwm_duty_cycle_in_clock_cycles){

	if (pwm_duty_cycle_in_clock_cycles == 0)
    {
        PB_OUT(13) = 0;					// AIN1
        PB_OUT(12) = 0;					// AIN2
    }
	// Set direction (values less than 0 indicate left)
	else if (pwm_duty_cycle_in_clock_cycles < 0)
	{
		PB_OUT(13) = 0;				// AIN1
		PB_OUT(12) = 1;				// AIN2
	}
	else
	{
		PB_OUT(13) = 1;				// AIN1
		PB_OUT(12) = 0;				// AIN2
	}
}

void set_power_magnitude(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles){
	pwm_duty_cycle_in_clock_cycles = abs(pwm_duty_cycle_in_clock_cycles);
	pwm_duty_cycle_in_clock_cycles = (pwm_duty_cycle_in_clock_cycles > pwm_period_in_clock_cycles-1) ? pwm_period_in_clock_cycles-1 : pwm_duty_cycle_in_clock_cycles;
	TIM3->CCR4 = (uint16_t)pwm_duty_cycle_in_clock_cycles;
}
