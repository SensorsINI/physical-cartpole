#include "led.h"

// LED is connected to PA4

void LED_Init(void)
{
	RCC->APB2ENR	|= 1<<2;
	GPIOA->CRL		&= 0XFFF0FFFF;
	GPIOA->CRL		|= 0X00030000;  // PA4 push-pull output
	GPIOA->ODR		|= 1<<4;        // PA4 High output
}

void Led_Enable(bool en)
{
	PA_OUT(4) = !en;
}
