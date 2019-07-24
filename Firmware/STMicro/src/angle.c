#include "angle.h"

void ANGLE_Init(void)
{
	// Initialise IO port.
 	RCC->APB2ENR	|= 1<<2;			// Enable PORTA clock 
	GPIOA->CRL		&= 0xFFFF0FFF;		// PA3 anolog input 	 		 
	RCC->APB2ENR	|= 1<<9;			// ADC1 Clock enable	  
	RCC->APB2RSTR	|= 1<<9;			// ADC1 reset
	RCC->APB2RSTR	&= ~(1<<9);			// Resetting end	    
	RCC->CFGR		&= ~(3<<14);		// Zero frequency factor clearing
	
	// SYSCLK/DIV2 = 12 MHz ADC clock set to 12 MHz, ADC maximum clock can not exceed 14M!
	// otherwise, the accuracy of ADC will be reduced.
	RCC->CFGR		|= 2<<14; 

	ADC1->CR1		&=  0xF0FFFF;		// Zero working mode
	ADC1->CR1		|=  0<<16;			// Independent working mode
	ADC1->CR1		&=~ (1<<8);			// Non scanning mode 
	ADC1->CR2		&=~ (1<<1);			// Single conversion mode
	ADC1->CR2		&=~ (7<<17);	   
	ADC1->CR2		|=  7<<17;			// Software control conversion  
	ADC1->CR2		|=  1<<20;			// Using external triggers (SWSTART)!!! Must be triggered by an event.
	ADC1->CR2		&=~ (1<<11);		// Right alignment	 
	ADC1->SQR1		&=~ (0x0F<<20);
	ADC1->SQR1		&=  0<<20;			// The 1 transformation is in the rule sequence, that is to transform only the rule sequence 1.	   

	// Set the sampling time of channel 3
	ADC1->SMPR2		&= 0xFFFF0FFF;		// Channel sampling time emptied  
	ADC1->SMPR2		|= 7<<9;			// Channel 239.5 cycle, increasing sampling time can improve accuracy.

	ADC1->CR2		|= 1<<0;			// Turn on AD converter
	ADC1->CR2		|= 1<<3;        	// Enable reset calibration  
	while(ADC1->CR2 & 1<<3);  			// Waiting for calibration to end 		
	
  	// This bit is set up by software and cleared by hardware.
	// The bit will be cleared after the calibration register is initialized.	 
	ADC1->CR2 |= 1<<2;					// Open AD calibration   
	while (ADC1->CR2 & 1<<2);			// Waiting for calibration to end
}

unsigned short ANGLE_Read(void)
{
	// Set conversion sequence		 
	ADC1->SQR3		&= 0xFFFFFFE0;		// Regular sequence 1 channel 3
	ADC1->SQR3		|= 3;		  			    
	ADC1->CR2		|= 1<<22;			// Start rule conversion channel 
	while (!(ADC1->SR & 1<<1));			// Wait for conversion end   
	return ADC1->DR;					// Return the ADC value	
}

unsigned short ANGLE_ReadAvergage(unsigned int n)
{
	unsigned int i;
	unsigned int average = 0;
	
	for (i = 0; i < n; i++)
	{
		// Set conversion sequence		 
		ADC1->SQR3		&= 0xFFFFFFE0;		// Regular sequence 1 channel 3
		ADC1->SQR3		|= 3;		  			    
		ADC1->CR2		|= 1<<22;			// Start rule conversion channel 
		while(!(ADC1->SR & 1<<1));			// Wait for conversion end
		
		average += ADC1->DR;
		//SYS_DelayUS(200);
	}
	
	return (unsigned short)(average/n);
}
