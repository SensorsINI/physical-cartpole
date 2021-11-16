#include "encoder.h"
  /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
/**************************************************************************
Function: initialize TIM2 to encoder interface mode.
Entry parameters: None
Return value: None
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	RCC->APB1ENR|=1<<0;     //TIM2 clock enable
	RCC->APB2ENR|=1<<2;    //Enable PORTA clock
	GPIOA->CRL&=0XFFFFFF00;//PA0 PA1
	GPIOA->CRL|=0X00000044;//Floating space input
	/* Initialize timer to encoder mode */ 
	TIM2->DIER|=1<<0;   //Enable update interrupt				
	TIM2->DIER|=1<<6;   //Enable triggered interrupt
	MY_NVIC_Init(1,3,TIM2_IRQn,1);

	/* Timer configuration in Encoder mode */ 
	TIM2->PSC = 0x0;//prescaler
	TIM2->ARR = ENCODER_TIM_PERIOD;//Setting counter automatic reload value
	TIM2->CR1 &=~(3<<8);// Select clock divider: no frequency division.
	TIM2->CR1 &=~(3<<5);// Select counting mode: edge alignment mode.
		
	TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1 mapping to TI1
	TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2 mapping to TI2
	TIM2->CCER &= ~(1<<1);	 //CC1P='0'	IC1FP1 does not invert, IC1FP1=TI1
	TIM2->CCER &= ~(1<<5);	 //CC2P='0'	IC2FP2 does not invert, IC2FP2=TI2
	TIM2->CCMR1 |= 3<<4; //	IC1F='1000' Input capture 1 filter
	TIM2->SMCR |= 3<<0;	 //SMS='011' All inputs are valid on the rising and falling edges.
	TIM2->CNT = 10000;
	TIM2->CR1 |= 0x01;    //CEN=1£¬Enabling timer
}
/**************************************************************************
Function: initialize TIM4 to encoder interface mode.
Entry parameters: None
Return value: None
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	RCC->APB1ENR|=1<<2;     //TIM4 clock enable
	RCC->APB2ENR|=1<<3;     //Enable PORTB clock
	GPIOB->CRL&=0X00FFFFFF; //PB6 PB7
	GPIOB->CRL|=0X44000000; //Floating space input
//	/* Initialize timer to encoder mode */ 
//	TIM4->PSC = 0x0;//prescaler
//	TIM4->ARR = ENCODER_TIM_PERIOD-1;//Setting counter automatic reload value
//  TIM4->CCMR1 |= 1<<0;          //Input mode, IC1FP1 map to TI1.
//  TIM4->CCMR1 |= 1<<8;          //Input mode, IC2FP2 map to TI2.
//  TIM4->CCER |= 0<<1;           //IC1 No reverse
//  TIM4->CCER |= 0<<5;           //IC2 No reverse
//	TIM4->SMCR |= 3<<0;	          //SMS='011' All inputs are valid on the rising and falling edges.
//	TIM4->CNT=10000;
//	TIM4->CR1 |= 0x01;            //CEN=1£¬Enabling timer
	
		/* Enable the TIM3 Update Interrupt */
	//These two items must be installed at the same time before they can be interrupted.
	TIM4->DIER|=1<<0;   //Enable update interrupt			
	TIM4->DIER|=1<<6;   //Enable triggered interrupt
	MY_NVIC_Init(1,3,TIM4_IRQn,1);

	/* Timer configuration in Encoder mode */ 
	TIM4->PSC = 0x0;//Ô¤·ÖÆµÆ÷
	TIM4->ARR = ENCODER_TIM_PERIOD;//Setting counter automatic reload value
	TIM4->CR1 &=~(3<<8);// Select clock divider: no frequency division.
	TIM4->CR1 &=~(3<<5);// Select counting mode: edge alignment mode.
		
	TIM4->CCMR1 |= 1<<0; //CC1S='01' IC1FP1 Mapping to TI1
	TIM4->CCMR1 |= 1<<8; //CC2S='01' IC2FP2 Mapping to TI2
	TIM4->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1 Not opposite£¬IC1FP1=TI1
	TIM4->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2 Not opposite£¬IC2FP2=TI2
	TIM4->CCMR1 |= 3<<4; //	IC1F='1000' Input capture 1 filter
	TIM4->SMCR |= 3<<0;	 //SMS='011' All inputs are valid on the rising and falling edges.
	TIM4->CNT = 10000;
	TIM4->CR1 |= 0x01;    //CEN=1£¬Enabling timer
}
/**************************************************************************
Function: unit time read encoder count
Entry parameters: timer
Return value: speed value
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT; break;
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT; break;	
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}

void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//Spillover interruption
	{
			    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//Clear interrupt flag bit	    
}
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//Spillover interruption
	{
			    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//Clear interrupt flag bit   
}
