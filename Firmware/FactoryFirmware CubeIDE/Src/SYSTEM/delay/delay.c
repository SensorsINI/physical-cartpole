#include "delay.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"	//for ucos  	  
#endif
////////////////////////////////////////////////////////////////////////////////// 	 
static u8  fac_us=0;//Us delay multiplier			   
static u16 fac_ms=0;//MS delay multiplier, under UCOS, represents the MS number of each beat.

#ifdef OS_CRITICAL_METHOD 	//If OS_CRITICAL_METHOD is defined, it means ucosII is used.
//Systick interrupt service function, used when using UCOS
void SysTick_Handler(void)
{				   
	OSIntEnter();		//Enter the interruption
  OSTimeTick();       //Calling the clock service program of UCOS               
  OSIntExit();        //Trigger task switching soft interrupt
}
#endif
			   
//Initialization delay function
//When using UCOS, this function initializes the clock beat of UCOS.
//The SYSTICK clock is fixed to the 1/8 of the HCLK clock.
//SYSCLK: system clock
void delay_init(u8 SYSCLK)
{
#ifdef OS_CRITICAL_METHOD 	//If OS_CRITICAL_METHOD is defined, it means using ucosII..
	u32 reload;
#endif
 	SysTick->CTRL&=~(1<<2);	//SYSTICK uses external clock source.
	fac_us=SYSCLK/8;		//Whether or not UCOS is used, fac_us needs to be used.
	    
#ifdef OS_CRITICAL_METHOD 	//If OS_CRITICAL_METHOD is defined, it means ucosII is used.
	reload=SYSCLK/8;		//The number of counts per second is K.   
	reload*=1000000/OS_TICKS_PER_SEC;//Set spillover time according to OS_TICKS_PER_SEC
							//Reload is a 24 bit register with a maximum value of 16777216. At 72M, about 1.86s.	
	fac_ms=1000/OS_TICKS_PER_SEC;//Represents the minimum unit of delay for uCOS.	   
	SysTick->CTRL|=1<<1;   	//Open SYSTICK interrupt
	SysTick->LOAD=reload; 	//Interrupts every 1/OS_TICKS_PER_SEC seconds.
	SysTick->CTRL|=1<<0;   	//Open SYSTICK  
#else
	fac_ms=(u16)fac_us*1000;//The number of systick clocks required for each MS is not UCOS.
#endif
}								    

#ifdef OS_CRITICAL_METHOD 	//If OS_CRITICAL_METHOD is defined, it means using ucosII..
//Delay NUS
//NUS is the US number to be delayed..		    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;	//The value of LOAD	    	 
	ticks=nus*fac_us; 			//Required beat count	  		 
	tcnt=0;
	OSSchedLock();				//Prevent UCOS scheduling and prevent interruption of us delay
	told=SysTick->VAL;        	//Counter values just entered
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//Notice that SYSTICK is a decreasing counter.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;//When time exceeds / equal to the time to delay, exit.
		}  
	};
	OSSchedUnlock();			//Turn on uCOS scheduling									    
}
//Delay NMS
//nms:MS number to be delayed
void delay_ms(u16 nms)
{	
	if(OSRunning==OS_TRUE)//If OS is running away   
	{		  
		if(nms>=fac_ms)//The delay time is greater than UCOS's minimum time period. 
		{
   			OSTimeDly(nms/fac_ms);//UCOS delay
		}
		nms%=fac_ms;			//UCOS has been unable to provide such a small delay, using a common mode delay.    
	}
	delay_us((u32)(nms*1000));	//Common mode delay 
}
#else//   no UCOS
//delay nus		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //Time loading	  		 
	SysTick->VAL=0x00;        //Empty counter
	SysTick->CTRL=0x01 ;      //Start the countdown 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//Waiting time to arrive   
	SysTick->CTRL=0x00;       //Close counter
	SysTick->VAL =0X00;       //Empty counter	 
}
// delay NMS
// pay attention to the scope of NMS.
//SysTick->LOAD is a 24 bit register, so the maximum delay is:nms<=0xffffff*8*1000/SYSCLK
//The SYSCLK unit is Hz, and the NMS unit is Ms.
// for 72M, nms<=1864
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//Time loading(SysTick->LOADÎª24bit)
	SysTick->VAL =0x00;           //Empty counter
	SysTick->CTRL=0x01 ;          //Start the countdown  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//Waiting time to arrive 
	SysTick->CTRL=0x00;       //Close counter
	SysTick->VAL =0X00;       //Empty counter	  	    
} 
#endif
			 



































