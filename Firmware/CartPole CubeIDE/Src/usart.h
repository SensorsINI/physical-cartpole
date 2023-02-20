#ifndef __USART_H_
#define __USART_H_

#include "sys.h"

#define USART_RX_BUFFER_SIZE    256UL

void            USART_Init(unsigned int baud, bool interruptEn);
unsigned char   USART_Receive(void);
bool            USART_ReceiveNonBlocking(unsigned char * c);
bool            USART_ReceiveAsync(unsigned char * c);
void            USART_Send(unsigned char c);
void            USART_SendBuffer(const unsigned char * buff, unsigned int len);

#endif	 /*__USART_H_*/   
