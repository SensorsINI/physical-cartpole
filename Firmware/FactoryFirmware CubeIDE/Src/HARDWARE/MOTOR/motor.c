#include "motor.h"
  /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
	RCC->APB2ENR|=1<<3;       //PORTB Clock enable  
	GPIOB->CRH&=0X0000FFFF;   //PORTB12 13 14 15 push-pull
	GPIOB->CRH|=0X22220000;   //PORTB12 13 14 15 push-pull
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
MiniBalance_Motor_Init(); //Initializing IO required for motor control
	RCC->APB1ENR|=1<<1;       //TIM3 clock enable    
	RCC->APB2ENR|=1<<3;       //PORTB clock enable   
	GPIOB->CRL&=0XFFFFFF00;   //PORTB0 1 multiplexing output
	GPIOB->CRL|=0X000000BB;   //PORTB0 1 Multiplexed output
	TIM3->ARR=arr;//Setting counter automatic reload value
	TIM3->PSC=psc;//Prescaler non frequency division
	TIM3->CCMR2|=6<<12;//CH4 PWM1 mode	
	TIM3->CCMR2|=6<<4; //CH3 PWM1 mode	
	TIM3->CCMR2|=1<<11;//CH4 Pre load enable energy	 
	TIM3->CCMR2|=1<<3; //CH3 Pre load enable energy
	TIM3->CCER|=1<<12; //CH4 Output enable   
	TIM3->CCER|=1<<8;  //CH3 Output enable	
	TIM3->CR1=0x80;  //ARPE Make energy 
	TIM3->CR1|=0x01;   //Enabling timer 3 	
} 

