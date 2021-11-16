#include "key.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
/**************************************************************************
Function: initialization of keys
Entry parameters: None
Return value: None
**************************************************************************/
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;    //Enable PORTA clock
	GPIOA->CRH&=0XFFF00FFF; 
	GPIOA->CRH|=0X00088000;
	
	GPIOA->CRL&=0X0F0FF0FF; 
	GPIOA->CRL|=0X80800800;
	
  GPIOA->ODR|=1<<2; //PA 2 Pull up
  GPIOA->ODR|=1<<7; //PA 7 Pull up	
  GPIOA->ODR|=1<<5; //PA5 Pull up
	GPIOA->ODR|=3<<11; //PA11 12  Pull up	
} 
/**************************************************************************
Function: key scan
Entry parameter: double click wait time.
Return value: keystroke status 0: no action 1: click 2: double click.
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	  if(KEY2==0)  Forever_count++;   // long mark position is not set at 1.
     else        Forever_count=0;
		if(0==KEY2&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2;//Double click the execution instruction.
				}
		}
		if(1==KEY2)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1;//Click execute instructions
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
///**************************************************************************
// function: key scan
// entry parameters: None
// return value: keystroke state 0: no action 1: Click
//**************************************************************************/
//u8 click(void)
//{
//			static u8 flag_key=1;//Press button to loosen the mark.
//			if(flag_key&&(KEY1==0||KEY2==0||KEY3==0||KEY4==0))
//			{
//			flag_key=0;
//			if(KEY1==0)return 1;
//			if(KEY2==0)return 2;
//			if(KEY3==0)return 3;
//			if(KEY4==0)return 4;				
//			}
//			else if(1==KEY1&&1==KEY2&&1==KEY3&&1==KEY4)			flag_key=1;
//			return 0;// no press button.
//}
///**************************************************************************
// function: long press check
// entry parameters: None
// return value: keystroke status 0: no action 1: long press 2S
//**************************************************************************/
//u8 Long_Press(void)
//{
//			static u16 Long_Press_count,Long_Press;
//	    if(Long_Press==0&&KEY==0)  Long_Press_count++;  // long mark position is not set at 1.
//      else                       Long_Press_count=0; 
//		  if(Long_Press_count>200)		
//			{
//				Long_Press=1;	
//				Long_Press_count=0;
//				return 1;
//			}				
//			 if(Long_Press==1)     //Long press mark location 1
//			{
//				  Long_Press=0;
//			}
//			return 0;
//}
