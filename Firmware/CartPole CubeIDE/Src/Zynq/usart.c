#include "usart.h"

XUartPs UartPs	;		/* Instance of the UART Device */

void PC_Connection_INIT(unsigned int baud)
{

	XUartPs_Config *Config;

	Config = XUartPs_LookupConfig(UART_DEVICE_ID);

	XUartPs_CfgInitialize(&UartPs, Config, Config->BaseAddress);

	XUartPs_SetBaudRate(&UartPs, baud);


}

void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size){

	XUartPs_Send(&UartPs, SendBuffer, buffer_size);

}



int Message_GetFromPC(unsigned char * c) {

	int newDataCount = XUartPs_Recv(&UartPs, c, SERIAL_MAX_PKT_LENGTH);
	return newDataCount;

}




