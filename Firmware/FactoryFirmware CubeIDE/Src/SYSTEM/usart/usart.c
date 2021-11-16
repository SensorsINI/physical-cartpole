#include "usart.h"	  
 /**************************************************************************
Author��Minibalance
Our aliexpress��https://minibalance.aliexpress.com
**************************************************************************/
//Add the following code to support the printf function without having to select use MicroLIB  
#if 1
#pragma import(__use_no_semihosting)             
//Support functions required by the standard library                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//Define _sys_exit() to avoid using semihosting mode    
void _sys_exit(int x)
{ 
	x = x; 
} 
//Redefine the fputc function
int fputc(int ch, FILE *f)
{      
	
	while((USART1->SR&0X40)==0);//Flag_Show!=0  Use serial port 1   
	USART1->DR = (u8) ch;      

	return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////
/**************************Implementation function*************************************
*Function: usart1 sends a byte
*********************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//get USARTDIV
	mantissa=temp;				 //Get the integer part
	fraction=(temp-mantissa)*16; //Get the fractional part
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //Enable PORTA port clock  
	RCC->APB2ENR|=1<<14;  //Enable serial clock
	GPIOA->CRH&=0XFFFFF00F;//IO status setting
	GPIOA->CRH|=0X000008B0;//IO status setting
		  
	RCC->APB2RSTR|=1<<14;   //Reset serial port 1
	RCC->APB2RSTR&=~(1<<14);//Stop reset   	   
	//Baud rate setting
 	USART1->BRR=mantissa; // Baud rate setting	 
	USART1->CR1|=0X200C;  //1 bit stops, no check digit.

}
