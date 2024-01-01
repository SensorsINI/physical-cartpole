#include "xparameters.h"
#include "xuartps.h"
#include "usart.h"
#include "xuartps_hw.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xplatform_info.h"

#define UART_BASEADDR XPAR_XUARTPS_0_BASEADDR

#define INTC		XScuGic
#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_XUARTPS_1_INTR



XUartPs UartPs	;		/* Instance of the UART Device */
INTC InterruptController;	/* Instance of the Interrupt Controller */

bool with_interrupt = false;

void USART_Init(unsigned int baud, bool interruptEn)
{

    rxInp = 0;
    rxOut = 0;
    rxCnt = 0;

    with_interrupt = interruptEn;

	int Status;
	XUartPs_Config *Config;
	u32 IntrMask;


	Config = XUartPs_LookupConfig(UART_DEVICE_ID);
	if (NULL == Config) {
		return XST_FAILURE;
	}

	Status = XUartPs_CfgInitialize(&UartPs, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XUartPs_SelfTest(&UartPs);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	if (with_interrupt){

		XScuGic_Config *IntcConfig; /* Config for interrupt controller */

			/* Initialize the interrupt controller driver */
			IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
			if (NULL == IntcConfig) {
				return XST_FAILURE;
			}

			Status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
							IntcConfig->CpuBaseAddress);
			if (Status != XST_SUCCESS) {
				return XST_FAILURE;
			}


			Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
						(Xil_ExceptionHandler) XScuGic_InterruptHandler,
						&InterruptController);

			Status = XScuGic_Connect(&InterruptController, UART_INT_IRQ_ID,
						  (Xil_ExceptionHandler) XUartPs_InterruptHandler,
						  (void *) &UartPs);
			if (Status != XST_SUCCESS) {
				return XST_FAILURE;
			}

			/* Enable the interrupt for the device */
			XScuGic_Enable(&InterruptController, UART_INT_IRQ_ID);

			Xil_ExceptionEnable();


			XUartPs_SetHandler(&UartPs, (XUartPs_Handler)Handler, &UartPs);

			IntrMask = XUARTPS_IXR_RXOVR;

			XUartPs_SetInterruptMask(&UartPs, IntrMask);
			XUartPs_SetFifoThreshold(&UartPs, 1);
			XUartPs_SetOperMode(&UartPs, XUARTPS_OPER_MODE_NORMAL);

	}


}

void USART_SendBuffer(unsigned char * SendBuffer, unsigned int buffer_size){

	XUartPs_Send(&UartPs, SendBuffer, buffer_size);
//	unsigned int i;
//	/* Send the entire transmit buffer. */
//	for (i = 0; i < buffer_size; i++)
//	{
//		/* Wait until there is space in TX FIFO */
//		 while (XUartPs_IsTransmitFull(UART_BASEADDR));
//		/* Write the byte into the TX FIFO */
//		XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, SendBuffer[i]);
//	}
}


//bool USART_ReceiveAsync(unsigned char * rxBuffer, unsigned int* rxCnt)
//{
//	if (with_interrupt){
//	    if (rxCnt > 0) {
//	        if (rxOut == SERIAL_MAX_PKT_LENGTH)
//	        {
//	            rxOut = 0;
//	        }
//
//	        Xil_ExceptionDisable();
//	        rxCnt--;
//	        Xil_ExceptionEnable();
//
//	        return true;
//	    }
//	    return false;
//	} else {
//
//        int received_count = XUartPs_Recv(&UartPs, (u8 *)rxBuffer, sizeof(rxBuffer));
//
//	}
//
//}

bool USART_ReceiveAsync(unsigned char * rxBuffer, unsigned int* rxCnt) {
    // Calculate the space left in the buffer
    unsigned int spaceLeft = SERIAL_MAX_PKT_LENGTH - *rxCnt;

    // Receive data, ensuring we don't exceed the buffer size
    int received_count = XUartPs_Recv(&UartPs, (u8 *)(rxBuffer + *rxCnt), spaceLeft);

    // Update the count of received bytes
    if (received_count > 0) {
        *rxCnt += received_count;
    }

    return true; // Or return false in case of an error
}





void Handler(void *CallBackRef, u32 Event, unsigned int EventData)
{
    XUartPs *UartInstPtr = (XUartPs *)CallBackRef;
    u32 IrqStatus = XUartPs_ReadReg(UartInstPtr->Config.BaseAddress, XUARTPS_ISR_OFFSET);
    XUartPs_WriteReg(UartInstPtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, IrqStatus);


    if ((Event & (XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXFULL)) != 0) {
        if (rxCnt < SERIAL_MAX_PKT_LENGTH) {
        	rxBuffer[rxInp++] = XUartPs_ReadReg(UartInstPtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
            if (rxInp == SERIAL_MAX_PKT_LENGTH) {
                rxInp = 0;
            }
            rxCnt++;
        } else {
            rxInp = 0;
            rxOut = 0;
            rxCnt = 0;
        }
    } else {
//         Clear the buffer for any other types of interrupts or errors
        rxInp = 0;
        rxOut = 0;
        rxCnt = 0;
    }
}





