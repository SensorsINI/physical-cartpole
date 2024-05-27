#include "timer.h"
#include "led.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
/**************************************************************************
Function function: timing interrupt initialization
Entry parameters: arr: automatic reload value PSC: clock preset frequency
Return value: None
**************************************************************************/
void Timer1_Init(u16 arr,u16 psc)  
{  
	RCC->APB2ENR|=1<<11;//TIM1 clock enable    
 	TIM1->ARR=arr;      //Setting counter automatic reload value  
	TIM1->PSC=psc;      //The prescaler 7200 gets the count clock of 10Khz.
	TIM1->DIER|=1<<0;   //Enable update interrupt				
	TIM1->DIER|=1<<6;   //Enable triggered interrupt	   
	TIM1->CR1|=0x01;    //Enabling timer
	MY_NVIC_Init(1,1,TIM1_UP_IRQn,2);
}  
