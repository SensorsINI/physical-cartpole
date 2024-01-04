#ifndef __USART_H_
#define __USART_H_

#include "xparameters.h"
#include "xuartps.h"
#include "stdbool.h"
#include "../hardware_bridge.h"

/************************** Constant Definitions ***************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define UART_BASEADDR		XPAR_XUARTPS_0_BASEADDR
#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#define UART_CLOCK_HZ		XPAR_XUARTPS_0_CLOCK_HZ

#define SERIAL_MAX_PKT_LENGTH		32

void PC_Connection_INIT(unsigned int baud);
void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size);
int Message_GetFromPC(unsigned char * c);

#endif /*__USART_H_*/

