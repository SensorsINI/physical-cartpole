#include "sys.h" 

// Initialise system		 
void SYS_Init(void)
{
	unsigned int temp = 0;

	// Reset all clock registers.	  
 	RCC->APB1RSTR	 = 0x00000000;		 
	RCC->APB2RSTR	 = 0x00000000; 
  	RCC->AHBENR		 = 0x00000014;	  
  	RCC->APB2ENR 	 = 0x00000000;		   
  	RCC->APB1ENR	 = 0x00000000;   
	RCC->CR 		|= 0x00000001;  // Enable internal high-speed clock HSION	 															 
	RCC->CFGR 		&= 0xF8FF0000;  // Reset SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR			&= 0xFEF6FFFF;  // Reset HSEON,CSSON,PLLON
	RCC->CR			&= 0xFFFBFFFF;  // Reset HSEBYP	   	  
	RCC->CFGR		&= 0xFF80FFFF;	// Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR 		 = 0x00000000;	// Close all interrupts	

	// Configure vector table (RAM or Code area)			  
	#ifdef  VECT_TAB_RAM
	SCB->VTOR = 0x20000000|(0x0 & 0x1FFFFF80UL);
	#else
	SCB->VTOR = 0x08000000|(0x0 & 0x1FFFFF80UL);
	#endif

	// System clock -- PLL on with 72 MHz system clock
 	RCC->CR			|= 0x00010000;	// HSEON
	while(!(RCC->CR>>17));

	RCC->CFGR		 = 0x00000400;	// APB1=DIV2, APB2=DIV1, AHB=DIV1
	RCC->CFGR		|= 7<<18;		// PLL multiplication 9
	RCC->CFGR		|= 1<<16;	 	// PLLSRC on 
	FLASH->ACR		|= 0x32;

	RCC->CR			|= 0x01000000;	// PLLON
	while(!(RCC->CR>>25));

	RCC->CFGR		|= 0x00000002; 
	while (temp != 0x02)
	{   
		temp = (RCC->CFGR >> 2) & 0x03UL;
	}

	// Initalise Delay Timer (clock runs at 72/8 = 9 MHz)
	SysTick->CTRL &= ~(1<<2);		// SYSTICK uses external clock source.
}

void SYS_JTAG_Set(unsigned int mode)
{
	mode <<= 25;
	RCC->APB2ENR	|= 1<<0;  
	AFIO->MAPR		&= 0xF8FFFFFF;
	AFIO->MAPR		|= mode;
}

// Configure NVIC
// priority: 	Preemptive priority
// subPriority: Response priority
// channel: 	Interrupt number
// group: 		Interrupt packet 0~4
// Group division:
// 		group 0:0 preemptive priority, 4 bit response priority.
// 		group 1:1 preemptive priority, 3 bit response priority.
// 		group 2:2 preemptive priority, 2 bit response priority.
// 		group 3:3 preemptive priority, 1 bit response priority.
// 		group 4:4 preemptive priority, 0 bit response priority.
// The principle of priority and priority is that the smaller the value, the more priority.   
void SYS_NVIC_Init(unsigned int priority, unsigned int subPriority, unsigned int channel, unsigned int group)	 
{ 
	unsigned int temp1, temp2;  

	temp1 = (~group) & 0x07;
	temp1 <<= 8;
	temp2  = SCB->AIRCR; // Read previous settings
	temp2 &= 0x0000F8FF; // Empty previous group
	temp2 |= 0x05FA0000; // Write the key
	temp2 |= temp1;
	SCB->AIRCR = temp2;  // Set groupings
	
	temp2  = priority << (4 - group);	  
	temp2 |= subPriority & (0x0FUL >> group);
	temp2 &= 0x0FUL;  
	NVIC->ISER[channel/32] |= (1 << channel%32);
	NVIC->IP[channel] |= temp2 << 4;				   
} 

// External interrupt configuration for GPIOA~G pins.
//	port	Select GPIOA~G
//	bit		Which bit to enable
//	trig	Trigger mode: rising/falling/both
void SYS_EXTI_Config(unsigned int port, unsigned int bit, unsigned int trig) 
{
	unsigned int addr;
	unsigned int offset;

	addr   = bit/4;								// Get the number of interrupt register group.
	offset = (bit%4) * 4;

	RCC->APB2ENR		|=0x01;					// Enable IO to reuse clock			 
	AFIO->EXTICR[addr]	&= ~(0x0FUL << offset);
	AFIO->EXTICR[addr]	|= port << offset;		// EXTI.bit mapping to GPIOx.bit

	EXTI->IMR			|= 1 << bit;			// Open the interrupt on line BITx

	if (trig & 0x01) EXTI->FTSR |= 1 << bit;	// Event descending edge triggering on line BITx
	if (trig & 0x02) EXTI->RTSR |= 1 << bit;	// Event rising and falling edge triggering on line BITx
} 

void SYS_DelayUS(unsigned int us)
{
	unsigned int temp;	  

	SysTick->LOAD	= us * 9UL;		// Time loading	  		 
	SysTick->VAL	= 0x00;			// Empty counter
	SysTick->CTRL	= 0x01;			// Start the countdown 	 
	do
	{
		temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1<<16)));  
	SysTick->CTRL	= 0x00;       // Close counter
	SysTick->VAL 	= 0X00;       // Empty counter
}

// Max delay is 1864 ms
void SYS_DelayMS(unsigned int ms)
{	 		  	  
	unsigned int temp;

	SysTick->LOAD	= ms * 9000UL;	// Time loading	  		 
	SysTick->VAL	= 0x00;			// Empty counter
	SysTick->CTRL	= 0x01;			// Start the countdown 	 

	do
	{
		temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1<<16)));  
	SysTick->CTRL	= 0x00;       // Close counter
	SysTick->VAL 	= 0X00;       // Empty counter  	    
}  	    
