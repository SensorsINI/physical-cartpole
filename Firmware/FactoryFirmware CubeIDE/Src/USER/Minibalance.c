#include "sys.h"
  /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
u8 Flag_Stop=1,delay_50,delay_flag;         //Stop sign bit 50ms precise demo sign.
int Encoder,Position_Zero=10000;            //Pulse count of encoder
int Moto;                                   //The motor PWM variable should be Motor, paying tribute to Moto company	
int Voltage;                                //Battery voltage sampling related variables
float Angle_Balance;                        //Angular displacement sensor data
float Balance_KP=400,Balance_KD=400,Position_KP=20,Position_KD=300;  //PID coefficient
float Menu=1,Amplitude1=5,Amplitude2=20,Amplitude3=1,Amplitude4=10; //PID debug related parameters
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====System clock settings
	delay_init(72);                 //=====Delay initialization
	JTAG_Set(JTAG_SWD_DISABLE);     //=====Close the JTAG interface.
	JTAG_Set(SWD_ENABLE);           //=====The SWD interface can be debugged by using the SWD interface of the motherboard.
	delay_ms(1000);                 //=====Delay startup, waiting for system stability
	delay_ms(1000);                 //=====Delay startup, wait for system stability 
	LED_Init();                     //=====Initializing the hardware interface with LED connection
	EXTI_Init();                    //=====Initialization of keys (form of external interruption)
	OLED_Init();                    //=====OLED initialization
	uart_init(72,128000);           //=====Initializing serial port 1
  MiniBalance_PWM_Init(7199,0);   //=====Initialize PWM 10KHZ for driving motor 
	Encoder_Init_TIM4();            //=====Initialization encoder 
	Angle_Adc_Init();               //=====Angular displacement sensor analog data acquisition initialization
	Baterry_Adc_Init();             //=====Battery voltage analog acquisition initialization
	Timer1_Init(49,7199);           //=====Timing interrupt initialization 
	while(1)
		{      
				DataScope();	            //===Upper computer
				delay_flag=1;	            //===50ms interrupt precise delay flag
				oled_show();              //===Display screen open  	
				while(delay_flag);        //===50ms interrupt precision delay is mainly waveform display, upper computer needs strict 50ms transmission cycle. 							
		} 
}
