#ifndef __USART_H_
#define __USART_H_

#include "sys.h"

#define USART_RX_BUFFER_SIZE    256UL

void            PC_Connection_INIT(unsigned int baud);
int 			Message_GetFromPC(unsigned char * c);
void            Message_SendToPC(const unsigned char * buff, unsigned int len);
void 			Message_SendToPC_blocking(const unsigned char * buff, unsigned int len);

#endif	 /*__USART_H_*/   
