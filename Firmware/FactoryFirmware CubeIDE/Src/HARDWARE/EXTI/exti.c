#include "exti.h"
#include "key.h"
  /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
/**************************************************************************
Function: external interrupt initialization
Entry parameters: None
Return value: None
**************************************************************************/
void EXTI_Init(void)
{
	KEY_Init();	
	Ex_NVIC_Config(GPIO_A,7,FTIR);		//Falling edge trigger
	Ex_NVIC_Config(GPIO_A,5,FTIR);		//Falling edge trigger
	Ex_NVIC_Config(GPIO_A,11,FTIR);		//Falling edge trigger
	Ex_NVIC_Config(GPIO_A,12,FTIR);		//Falling edge trigger

	MY_NVIC_Init(2,2,EXTI9_5_IRQn,2);  	//Preemptive 2, sub priority 2, group 2
	MY_NVIC_Init(2,2,EXTI15_10_IRQn,2);	//Preemptive 2, sub priority 2, group 2	  
}


// external interrupt 9~5 service function
void EXTI9_5_IRQHandler(void)
{			
	delay_ms(5);   //Shake off			 
   if(KEY5==0)	// 
	{
		Flag_Stop=!Flag_Stop;

	}		
	if(KEY7==0)	///Menu 
	{
		if(Menu++==4) Menu=1;
	}		
 		EXTI->PR=1<<5;     //Clear the interrupt flag bit on LINE5.  
		EXTI->PR=1<<7;     //Clear the interrupt flag bit on LINE7.
}
//External interrupt 15~10 service program
void EXTI15_10_IRQHandler(void)
{			
	delay_ms(5);   //Shake off			 
  if(KEY11==0)	//PID-
	{
		if(Menu==1)        Balance_KP-=Amplitude1;
	  else	if(Menu==2)  Balance_KD-=Amplitude2;
		else  if(Menu==3)  Position_KP-=Amplitude3;
		else  if(Menu==4)  Position_KD-=Amplitude4;
	}		
	 if(KEY12==0)	//PID+ 
	{
			    if(Menu==1)  Balance_KP+=Amplitude1;
	  else	if(Menu==2)  Balance_KD+=Amplitude2;
		else  if(Menu==3)  Position_KP+=Amplitude3;
		else  if(Menu==4)  Position_KD+=Amplitude4;
	}		
	if(Balance_KP<=0) Balance_KP=0;
	if(Balance_KD<=0) Balance_KD=0;
	if(Position_KP<=0) Position_KP=0;
	if(Position_KD<=0) Position_KD=0;
  EXTI->PR=1<<11; //Clear the interrupt flag bit on LINE11.  
	EXTI->PR=1<<12; //Clear the interrupt flag bit on LINE12.
}





