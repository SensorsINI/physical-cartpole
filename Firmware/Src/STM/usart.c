#include "usart.h"	  

// Rx pin is PA10
// Tx pin is PA9

static unsigned char usartRxBuffer[USART_RX_BUFFER_SIZE];
static unsigned int  rxInp;
static unsigned int  rxOut;
static unsigned int  rxCnt;

void PC_Connection_INIT(unsigned int baud)
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

	USART1->CR1 |= 0x202C;						// Configure UART to 8N1, enable Rx interrupt, enable Rx/Tx
	SYS_NVIC_Init(2, 2, USART1_IRQn, 1);

}


int Message_GetFromPC(unsigned char * c)
{
    int count = 0;

    while (rxCnt > 0)
    {
        c[count] = (unsigned char)usartRxBuffer[rxOut++];
        if (rxOut == USART_RX_BUFFER_SIZE)
        {
            rxOut = 0;
        }

        __disable_irq();
        rxCnt--;
        __enable_irq();

        count++;
    }

    return count;
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


void Message_SendToPC(const unsigned char * buff, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i++)
	{
		while ((USART1->SR & 0x0080) == 0);		// Wait for TXE
		USART1->DR = buff[i];
	}
}


// TODO: Not tested yet. Just added a plausible code for compatibility with Zynq, not to crash at compilation
// Not needed for basic operations.
void Message_SendToPC_blocking(const unsigned char * buff, unsigned int len)
{
    Message_SendToPC(buff, len);  // Use the non-blocking function to send all data

    while (!(USART1->SR & 0x0040));  // Wait for TC (Transmission Complete)
}
