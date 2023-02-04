#include "key.h"

KEY_Callback cbKey2;
KEY_Callback cbKey5;
KEY_Callback cbKey7;
KEY_Callback cbKey11;
KEY_Callback cbKey12;

void KEY_Init(void)
{
	cbKey2  = 0;
	cbKey5  = 0;
	cbKey7  = 0;
	cbKey11 = 0;
	cbKey12 = 0;

	RCC->APB2ENR	|= 1<<2;    // Enable PORTA clock

	GPIOA->CRH		&= 0xFFF00FFF; 
	GPIOA->CRH		|= 0x00088000;
	GPIOA->CRL		&= 0x0F0FF0FF; 
	GPIOA->CRL		|= 0x80800800;
	GPIOA->ODR		|= 0x000018A4;
	
	SYS_EXTI_Config(SYS_GPIO_A,  2, SYS_TRIG_FALLING);
	SYS_EXTI_Config(SYS_GPIO_A,  5, SYS_TRIG_FALLING);
	SYS_EXTI_Config(SYS_GPIO_A,  7, SYS_TRIG_FALLING);
	SYS_EXTI_Config(SYS_GPIO_A, 11, SYS_TRIG_FALLING);
	SYS_EXTI_Config(SYS_GPIO_A, 12, SYS_TRIG_FALLING); 

	SYS_NVIC_Init(2, 2, EXTI2_IRQn,     2);
	SYS_NVIC_Init(2, 2, EXTI9_5_IRQn,   2);
	SYS_NVIC_Init(2, 2, EXTI15_10_IRQn, 2);
}

void KEY_SetCallback(unsigned int key, KEY_Callback cb)
{
	switch (key)
	{
		case KEY_2:		cbKey2  = cb; break;
		case KEY_5:		cbKey5  = cb; break;
		case KEY_7:		cbKey7  = cb; break;
		case KEY_11:	cbKey11 = cb; break;
		case KEY_12:	cbKey12 = cb; break;
	}
}

// External interrupt 2 service function
void EXTI2_IRQHandler(void)
{
	SYS_DelayMS(5);		// Debounce

	if (PA_IN(KEY_2) == 0)
	{
		if (cbKey2)
		{
			cbKey2();
		}
		EXTI->PR = 1<<2;
	}
}

// External interrupt 9~5 service function
void EXTI9_5_IRQHandler(void)
{			
	SYS_DelayMS(5);		// Debounce

	if (PA_IN(KEY_5) == 0)
	{
		if (cbKey5)
		{
			cbKey5();
		}
		EXTI->PR = 1<<5;
	}

	if (PA_IN(KEY_7) == 0)
	{
		if (cbKey7)
		{
			cbKey7();
		}
		EXTI->PR = 1<<7;
	}
}

// External interrupt 15-10 service function
void EXTI15_10_IRQHandler(void)
{			
	SYS_DelayMS(5);		// Debounce

	if (PA_IN(KEY_11) == 0)
	{
		if (cbKey11)
		{
			cbKey11();
		}
		EXTI->PR = 1<<11;
	}

	if (PA_IN(KEY_12) == 0)
	{
		if (cbKey12)
		{
			cbKey12();
		}
		EXTI->PR = 1<<12;
	}
}
