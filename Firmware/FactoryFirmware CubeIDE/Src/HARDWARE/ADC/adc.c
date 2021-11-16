#include "adc.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
/**************************************************************************
Function: ADC initializes battery voltage detection
Entry parameters: None
Return value: None
**************************************************************************/
void  Baterry_Adc_Init(void)
{    
  //Initialize IO port first.
 	RCC->APB2ENR|=1<<2;    //Enable PORTA clock 
	GPIOA->CRL&=0XF0FFFFFF;//pa6 anologÊäÈë
	RCC->APB2ENR|=1<<9;    //ADC1 clock enable
	RCC->APB2RSTR|=1<<9;   //ADC1 reduction
	RCC->APB2RSTR&=~(1<<9);//Resetting end	    
	RCC->CFGR&=~(3<<14);   //Zero frequency factor clearing
//SYSCLK/DIV2=12M ADC clock set to 12M, ADC maximum clock can not exceed 14M!
// otherwise, the accuracy of ADC will be reduced.
	RCC->CFGR|=2<<14;      	 
	ADC1->CR1&=0XF0FFFF;   //Zero working mode
	ADC1->CR1|=0<<16;      //Independent working mode  
	ADC1->CR1&=~(1<<8);    //Non scanning mode	  
	ADC1->CR2&=~(1<<1);    //Single conversion mode
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //Software control conversion
	ADC1->CR2|=1<<20;      //Using external triggers (SWSTART)!!! Must be triggered by an event.
	ADC1->CR2&=~(1<<11);   //Right alignment	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //The 1 transformation is in the rule sequence, that is to transform only the rule sequence 1. 			   
// set the sampling time of channel 6.
	ADC1->SMPR2&=0XF0FFFFFF; //Sampling time emptied	  
	ADC1->SMPR2|=7<<18;      // 239.5 Cycle, increasing sampling time can improve accuracy.	 

	ADC1->CR2|=1<<0;	    //Turn on AD converter	 
	ADC1->CR2|=1<<3;        //Enable reset calibration 
	while(ADC1->CR2&1<<3);  //Waiting for calibration to end			 
// this bit is set up by software and cleared by hardware. The bit will be cleared after the calibration register is initialized.
	ADC1->CR2|=1<<2;        //Open AD calibration	   
	while(ADC1->CR2&1<<2);  //Waiting for calibration to end
}		

/**************************************************************************
Function: AD sampling
Entry parameters: ADC1 channel
Return value: AD conversion result
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	// set conversion sequence		 
	ADC1->SQR3&=0XFFFFFFE0;//Regular sequence 1 channel Ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //Start rule conversion channel 
	while(!(ADC1->SR&1<<1));//Wait for conversion end   
	return ADC1->DR;		//Return the ADC value	
}

/**************************************************************************
Function: read battery voltage
Entry parameters: None
Return value: battery voltage unit MV
**************************************************************************/
int Get_battery_volt(void)   
{  
	int Volt;                                       //Battery voltage
	Volt=Get_Adc(Battery_Ch)*3.3*11*100/1.0/4096;		 //Resistance divider can be obtained based on simple analysis of schematic diagram.
	return Volt;
}

/**************************************************************************
Function function: angular displacement sensor initialization
Entry parameters: None
Return value: None
**************************************************************************/
void  Angle_Adc_Init(void)
{    
// first, initialize IO port.
 	RCC->APB2ENR|=1<<2;    //Enable PORTA clock 
	GPIOA->CRL&=0XFFFF0FFF;//PA3 anolog input 	 		 
	RCC->APB2ENR|=1<<9;    //ADC1 Clock enable	  
	RCC->APB2RSTR|=1<<9;   //ADC1 reset
	RCC->APB2RSTR&=~(1<<9);//Resetting end	    
	RCC->CFGR&=~(3<<14);   //Zero frequency factor clearing	
//SYSCLK/DIV2=12M ADC clock set to 12M, ADC maximum clock can not exceed 14M!
// otherwise, the accuracy of ADC will be reduced.
	RCC->CFGR|=2<<14;      	 

	ADC1->CR1&=0XF0FFFF;   //Zero working mode
	ADC1->CR1|=0<<16;      //Independent working mode
	ADC1->CR1&=~(1<<8);    //Non scanning mode 
	ADC1->CR2&=~(1<<1);    //Single conversion mode
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //Software control conversion  
	ADC1->CR2|=1<<20;      //Using external triggers (SWSTART)!!! Must be triggered by an event.
	ADC1->CR2&=~(1<<11);   //Right alignment	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //The 1 transformation is in the rule sequence, that is to transform only the rule sequence 1.	   
	//Set the sampling time of channel 7.
	ADC1->SMPR2&=0XFFFF0FFF;//Channel sampling time emptied  
	ADC1->SMPR2|=7<<9;      //Channel 239.5 cycle, increasing sampling time can improve accuracy.

	ADC1->CR2|=1<<0;	    //Turn on AD converter
	ADC1->CR2|=1<<3;        //Enable reset calibration  
	while(ADC1->CR2&1<<3);  //Waiting for calibration to end 			 
  // this bit is set up by software and cleared by hardware. The bit will be cleared after the calibration register is initialized.	 
	ADC1->CR2|=1<<2;        //Open AD calibration   
	while(ADC1->CR2&1<<2);  //Waiting for calibration to end
	delay_ms(1);
}		

// get the conversion value of channel ch, take times times, and then average.
//ch: channel number
//times: acquisition times
// return value: average value of times conversion result of channel ch.
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_us(200);
	}
	return temp_val/times;
} 
	 








