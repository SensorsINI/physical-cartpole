#include "usart.h"

XUartPs UartPs	;		/* Instance of the UART Device */

bool with_interrupt = false;

void USART_Init(unsigned int baud, bool interruptEn)
{

    with_interrupt = interruptEn;

	XUartPs_Config *Config;

	Config = XUartPs_LookupConfig(UART_DEVICE_ID);

	XUartPs_CfgInitialize(&UartPs, Config, Config->BaseAddress);


}

void USART_SendBuffer(unsigned char * SendBuffer, unsigned int buffer_size){

	XUartPs_Send(&UartPs, SendBuffer, buffer_size);

}



bool USART_ReceiveAsync(unsigned char * c) {

	if (XUartPs_IsReceiveData(UART_BASEADDR))
	{
		*c = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
		return true;
	} else {
		return false;
	}
}




