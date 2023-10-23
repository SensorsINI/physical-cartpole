#include "usart.h"	  

// Rx pin is PA10
// Tx pin is PA9

static unsigned char usartRxBuffer[USART_RX_BUFFER_SIZE];
static unsigned int  rxInp;
static unsigned int  rxOut;
static unsigned int  rxCnt;

void USART_Init(unsigned int baud, bool interruptEn)
{
	unsigned short mantissa;
	unsigned short fraction;
	float temp = 72e6;

    rxInp = 0;
    rxOut = 0;
    rxCnt = 0;
    
	// Calculate baud rate divisor
	temp 	 /= 16.0 * baud;
	mantissa  = (unsigned short)temp;
	fraction  = (unsigned short)((temp - mantissa) * 16);

	RCC->APB2ENR 	|=	1<<2;   					// Enable PORTA clock  
	RCC->APB2ENR 	|=	1<<14;						// Enable serial clock
	GPIOA->CRH   	&= 0xFFFFF00F;					// IO status setting
	GPIOA->CRH   	|= 0x000008B0;					// IO status setting

	RCC->APB2RSTR	|= 1<<14;						// Reset serial port 1
	RCC->APB2RSTR	&=~(1<<14);						// Stop reset

	USART1->BRR		 = (mantissa << 4) + fraction; 	// Baud rate setting

    if (interruptEn)
    {
        USART1->CR1 |= 0x202C;						// Configure UART to 8N1, enable Rx interrupt, enable Rx/Tx
        SYS_NVIC_Init(2, 2, USART1_IRQn, 1);
    }
    else
    {
        USART1->CR1 |= 0x200C;						// Configure UART to 8N1, enable Rx/Tx
    }
}

unsigned char USART_Receive(void)
{
	while ((USART1->SR & 0x0020) == 0);		// Wait for RXNE
	return (unsigned char)USART1->DR;
}

bool USART_ReceiveNonBlocking(unsigned char * c)
{
	if (USART1->SR & 0x0020)	// Wait for RXNE
	{
		*c = (unsigned char)USART1->DR;
		return true;
	}
	return false;
}

bool USART_ReceiveAsync(unsigned char * c)
{
	if (rxCnt > 0)
	{
		*c = (unsigned char)usartRxBuffer[rxOut++];
        if (rxOut == USART_RX_BUFFER_SIZE)
        {
            rxOut = 0;
        }
        
        __disable_irq();
        rxCnt--;
        __enable_irq();
        
		return true;
	}
	return false;
}

void USART1_IRQHandler(void)
{
    unsigned int sr = USART1->SR;
    
    if (sr & 0x0020)
    {
        if (rxCnt < USART_RX_BUFFER_SIZE)
        {
            usartRxBuffer[rxInp++] = (unsigned char)USART1->DR;
            if (rxInp == USART_RX_BUFFER_SIZE)
            {
                rxInp = 0;
            }
            
            rxCnt++;
        }
        else
        {
            rxInp = 0;
            rxOut = 0;
            rxCnt = 0;
        }
    }
    // Over flow
    else
    {
        USART1->DR;
        rxInp = 0;
        rxOut = 0;
        rxCnt = 0;
    }
}

void USART_Send(unsigned char c)
{
	while ((USART1->SR & 0x0080) == 0);		// Wait for TXE
	USART1->DR = c;
}

void USART_SendBuffer(const unsigned char * buff, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i++)
	{
		while ((USART1->SR & 0x0080) == 0);		// Wait for TXE
		USART1->DR = buff[i];
	}
}
