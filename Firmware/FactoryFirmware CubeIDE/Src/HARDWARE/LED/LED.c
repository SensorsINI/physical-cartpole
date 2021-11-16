#include "led.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
/**************************************************************************
Function: initialization of LED interface
Entry parameters: None
Return value: None
**************************************************************************/
void LED_Init(void)
{
RCC->APB2ENR|=1<<2;      // 
GPIOA->CRL&=0XFFF0FFFF;
GPIOA->CRL|=0X00030000;  //PA4 push-pull output
GPIOA->ODR|=1<<4;        //PA4 High output
}

/**************************************************************************
Function: LED flicker
Entry parameter: Scintillation frequency
Return value: None
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED=0;
	  else		if(++temp==time)	LED=~LED,temp=0;
}
