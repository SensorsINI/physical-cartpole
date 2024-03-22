#ifndef __USART_H_
#define __USART_H_

#include "xparameters.h"
#include "stdbool.h"
#include "../hardware_bridge.h"

#define USART_RX_BUFFER_SIZE		256UL

void PC_Connection_INIT(unsigned int baud);
void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size);
void Message_SendToPC_blocking(unsigned char * SendBuffer, unsigned int buffer_size);
int Message_GetFromPC(unsigned char * c);

#endif /*__USART_H_*/

