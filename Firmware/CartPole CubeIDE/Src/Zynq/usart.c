/*
// Communication using UART PS
#include "usart.h"
#include "xuartps.h"

#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID

XUartPs UartPs	;		// Instance of the UART Device

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

	int newDataCount = XUartPs_Recv(&UartPs, c, USART_RX_BUFFER_SIZE);
	return newDataCount;

}

*/



///*
// Communication using UART PS
#include "usart.h"
#include "xuartps.h"

#include "xuartps_hw.h"
#include "xil_exception.h"
#include "xscugic.h"

#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID

#define INTC				XScuGic
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_XUARTPS_1_INTR

XUartPs UartPs	;		// Instance of the UART Device
INTC InterruptController;	// Instance of the Interrupt Controller
void Handler(XUartPs *InstancePtr);

static unsigned char usartRxBuffer[USART_RX_BUFFER_SIZE];

static unsigned int  rxInp;
static unsigned int  rxOut;
static unsigned int  rxCnt;

void PC_Connection_INIT(unsigned int baud)
{

    rxInp = 0;
    rxOut = 0;
    rxCnt = 0;

	XUartPs_Config *Config;

	Config = XUartPs_LookupConfig(UART_DEVICE_ID);

	XUartPs_CfgInitialize(&UartPs, Config, Config->BaseAddress);

	XUartPs_SetBaudRate(&UartPs, baud);


	int Status;
	XScuGic_Config *IntcConfig; // Config for interrupt controller

	// Initialize the interrupt controller driver
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
				  (Xil_ExceptionHandler) Handler,
				  (void *) &UartPs);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

//	Enable the interrupt for the device
	XScuGic_Enable(&InterruptController, UART_INT_IRQ_ID);

	Xil_ExceptionEnable();


	u32 IntrMask = 	IntrMask =
			XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
			XUARTPS_IXR_RXOVR;

	XUartPs_SetInterruptMask(&UartPs, IntrMask);
	XUartPs_SetFifoThreshold(&UartPs, 1);
	XUartPs_SetOperMode(&UartPs, XUARTPS_OPER_MODE_NORMAL);


}

void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size){

	XUartPs_Send(&UartPs, SendBuffer, buffer_size);

}



int Message_GetFromPC(unsigned char * c) {
    int count = 0;

    while (rxCnt > 0)
    {
        c[count] = (unsigned char)usartRxBuffer[rxOut++];
        if (rxOut == USART_RX_BUFFER_SIZE)
        {
            rxOut = 0;
        }

        disable_irq();
        rxCnt--;
        enable_irq();

        count++;
    }

    return count;

}

extern u32 XUartPs_ReceiveBuffer(XUartPs *InstancePtr);
extern u32 XUartPs_SendBuffer(XUartPs *InstancePtr);
void Handler(XUartPs *InstancePtr)
{
	u32 IsrStatus;

	Xil_AssertVoid(InstancePtr != NULL);
	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);


//	 * Read the interrupt ID register to determine which
//	 * interrupt is active

	IsrStatus = XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
				   XUARTPS_IMR_OFFSET);

	IsrStatus &= XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
				   XUARTPS_ISR_OFFSET);

	//Dispatch an appropriate handler.
	if((IsrStatus & ((u32)XUARTPS_IXR_RXOVR |
			(u32)XUARTPS_IXR_RXFULL)) != (u32)0) {
		//Received data interrupt
		while(XUartPs_IsReceiveData(InstancePtr->Config.BaseAddress))
		{
	        if (rxCnt < USART_RX_BUFFER_SIZE)
	        {
	            usartRxBuffer[rxInp++] = (unsigned char)XUartPs_ReadReg(InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
	            if (rxInp == USART_RX_BUFFER_SIZE)
	            {
	                rxInp = 0;
	            }

	            rxCnt++;
	        }
	        else
			{
	        	// Over flow
	        	(unsigned char)XUartPs_ReadReg(InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
				rxInp = 0;
				rxOut = 0;
				rxCnt = 0;
			}
		}
	}

	if((IsrStatus & ((u32)XUARTPS_IXR_TXEMPTY | (u32)XUARTPS_IXR_TXFULL))
									 != (u32)0) {
		 // Transmit data interrupt

//		 If there are not bytes to be sent from the specified buffer then disable
//		 the transmit interrupt so it will stop interrupting as it interrupts
//		 any time the FIFO is empty

		if (InstancePtr->SendBuffer.RemainingBytes == (u32)0) {
			XUartPs_WriteReg(InstancePtr->Config.BaseAddress,
					XUARTPS_IDR_OFFSET,
					((u32)XUARTPS_IXR_TXEMPTY | (u32)XUARTPS_IXR_TXFULL));
		}

		// If TX FIFO is empty, send more.
		else if((IsrStatus & ((u32)XUARTPS_IXR_TXEMPTY)) != (u32)0) {
			(void)XUartPs_SendBuffer(InstancePtr);
		}
	}

//	Clear the interrupt status.
	XUartPs_WriteReg(InstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET,
		IsrStatus);

}
//*/







/*
// Communication using UARTNS550 IP (Info: In current hardware platform hpf_v2024_4 connected at JD)
// The simple version like this for UART PS is not applicable as the buffer of UARTNS550 IP has only 16 bytes.
// Hence we must use interrupts
#include "usart.h"
#include "xuartns550.h"


#include "xil_exception.h"
#include "xscugic.h"

#define UART_DEVICE_ID		XPAR_UARTNS550_0_DEVICE_ID

#define INTC				XScuGic
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_FABRIC_AXI_UART16550_0_IP2INTC_IRPT_INTR

XUartNs550 UartNs550;		// Instance of the UART Device
INTC InterruptController;	// Instance of the Interrupt Controller
void Handler(XUartNs550 *InstancePtr);

static unsigned char usartRxBuffer[USART_RX_BUFFER_SIZE];

static unsigned int  rxInp;
static unsigned int  rxOut;
static unsigned int  rxCnt;

extern int XUartNs550_SetBaudRate(XUartNs550 *InstancePtr, u32 BaudRate);
void PC_Connection_INIT(unsigned int baud)
{

    rxInp = 0;
    rxOut = 0;
    rxCnt = 0;

    XUartNs550_Initialize(&UartNs550, UART_DEVICE_ID);

    XUartNs550_SetBaudRate(&UartNs550, baud);


	int Status;
	XScuGic_Config *IntcConfig; // Config for interrupt controller

	// Initialize the interrupt controller driver
	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(&InterruptController, UART_INT_IRQ_ID,
					0xA0, 0x3);


	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				&InterruptController);

	Status = XScuGic_Connect(&InterruptController, UART_INT_IRQ_ID,
				  (Xil_ExceptionHandler) Handler,
				  (void *) &UartNs550);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

//	Enable the interrupt for the device
	XScuGic_Enable(&InterruptController, UART_INT_IRQ_ID);

	Xil_ExceptionEnable();

	u16 Options = XUN_OPTION_DATA_INTR|XUN_OPTION_FIFOS_ENABLE;
	XUartNs550_SetOptions(&UartNs550, Options);

	XUartNs550_SetFifoThreshold(&UartNs550, XUN_FIFO_TRIGGER_01);

}

void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size){

	XUartNs550_Send(&UartNs550, SendBuffer, buffer_size);

}



int Message_GetFromPC(unsigned char * c) {
    int count = 0;

    while (rxCnt > 0)
    {
        c[count] = (unsigned char)usartRxBuffer[rxOut++];
        if (rxOut == USART_RX_BUFFER_SIZE)
        {
            rxOut = 0;
        }

        disable_irq();
        rxCnt--;
        enable_irq();

        count++;
    }

    return count;

}

extern u32 XUartNs550_ReceiveBuffer(XUartNs550 *InstancePtr);
extern u32 XUartNs550_SendBuffer(XUartNs550 *InstancePtr);
void Handler(XUartNs550 *InstancePtr)
{
	u8 IsrStatus;

	Xil_AssertVoid(InstancePtr != NULL);

//	Read the interrupt ID register to determine which, only one,
//	interrupt is active
	IsrStatus = (u8)XUartNs550_ReadReg(InstancePtr->BaseAddress,
					XUN_IIR_OFFSET) &
					XUN_INT_ID_MASK;

	//Dispatch an appropriate handler.
	if(IsrStatus == 4) {
		//Received data interrupt
		while(XUartNs550_IsReceiveData(InstancePtr->BaseAddress))
		{
	        if (rxCnt < USART_RX_BUFFER_SIZE)
	        {
	            usartRxBuffer[rxInp++] = (unsigned char)XUartNs550_ReadReg(InstancePtr->BaseAddress, XUN_RBR_OFFSET);
	            if (rxInp == USART_RX_BUFFER_SIZE)
	            {
	                rxInp = 0;
	            }

	            rxCnt++;
	        }
	        else
			{
	        	// Over flow
	        	XUartNs550_ReadReg(InstancePtr->BaseAddress, XUN_RBR_OFFSET);
				rxInp = 0;
				rxOut = 0;
				rxCnt = 0;
			}
		}
	}

	else if(IsrStatus == 2) {
		 // Transmit data interrupt

//		 If there are not bytes to be sent from the specified buffer then disable
//		 the transmit interrupt so it will stop interrupting as it interrupts
//		 any time the FIFO is empty

		if (InstancePtr->SendBuffer.RemainingBytes == (u32)0) {
			u32 IerRegister = XUartNs550_ReadReg(InstancePtr->BaseAddress,
								XUN_IER_OFFSET);
			XUartNs550_WriteReg(InstancePtr->BaseAddress, XUN_IER_OFFSET,
					 IerRegister & ~XUN_IER_TX_EMPTY);
		}

		// If TX FIFO is empty, send more.
		else {
			XUartNs550_SendBuffer(InstancePtr);
		}
	}
	else{
		// Reading the ID register clears the currently asserted interrupts
		XUartNs550_GetLineStatusReg(InstancePtr->BaseAddress);
	}

}
*/











/*
// Communication using UART LITE IP (Info: In current hardware platform hpf_v2024_4 connected at JC)
#include "usart.h"
#include "xuartlite.h"
#include "xuartlite_l.h"

#include "xil_exception.h"
#include "xscugic.h"

#define UART_DEVICE_ID		XPAR_UARTLITE_0_DEVICE_ID

#define INTC				XScuGic
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_FABRIC_AXI_UARTLITE_0_INTERRUPT_INTR

XUartLite UartLite;		// Instance of the UART Device
INTC InterruptController;	// Instance of the Interrupt Controller
void Handler(XUartLite *InstancePtr);

static unsigned char usartRxBuffer[USART_RX_BUFFER_SIZE];

static unsigned int  rxInp;
static unsigned int  rxOut;
static unsigned int  rxCnt;

void PC_Connection_INIT(unsigned int baud)
{

    rxInp = 0;
    rxOut = 0;
    rxCnt = 0;

	XUartLite_Initialize(&UartLite, UART_DEVICE_ID);

	// Baud rate cannot be set for UART LITE. 5.01.2024 with hpf_v2024_4 it is set to 230400


	int Status;
	XScuGic_Config *IntcConfig; // Config for interrupt controller

	// Initialize the interrupt controller driver
	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(&InterruptController, UART_INT_IRQ_ID,
					0xA0, 0x3);

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				&InterruptController);

	Status = XScuGic_Connect(&InterruptController, UART_INT_IRQ_ID,
				  (Xil_ExceptionHandler) Handler,
				  (void *) &UartLite);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

//	Enable the interrupt for the device
	XScuGic_Enable(&InterruptController, UART_INT_IRQ_ID);

	Xil_ExceptionEnable();

	XUartLite_EnableInterrupt(&UartLite);

	// There are only two interrupts for UART lite, now mask or fifo threshold can be set

}

void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size){

	XUartLite_Send(&UartLite, SendBuffer, buffer_size);

}



int Message_GetFromPC(unsigned char * c) {
    int count = 0;

    while (rxCnt > 0)
    {
        c[count] = (unsigned char)usartRxBuffer[rxOut++];
        if (rxOut == USART_RX_BUFFER_SIZE)
        {
            rxOut = 0;
        }

        disable_irq();
        rxCnt--;
        enable_irq();

        count++;
    }

    return count;

}

extern u32 XUartLite_ReceiveBuffer(XUartLite *InstancePtr);
extern u32 XUartLite_SendBuffer(XUartLite *InstancePtr);
void Handler(XUartLite *InstancePtr)
{
	u32 IsrStatus;

	Xil_AssertVoid(InstancePtr != NULL);


//	Read the status register to determine which, coulb be both
//	interrupt is active

	IsrStatus = XUartLite_ReadReg(InstancePtr->RegBaseAddress,
					XUL_STATUS_REG_OFFSET);

	//Dispatch an appropriate handler.
	if ((IsrStatus & (XUL_SR_RX_FIFO_FULL | XUL_SR_RX_FIFO_VALID_DATA)) != 0)
	{
		//Received data interrupt
		while(XUartLite_GetStatusReg(InstancePtr->RegBaseAddress) & XUL_SR_RX_FIFO_VALID_DATA)
		{
	        if (rxCnt < USART_RX_BUFFER_SIZE)
	        {
	            usartRxBuffer[rxInp++] = (unsigned char)XUartLite_ReadReg(InstancePtr->RegBaseAddress, XUL_RX_FIFO_OFFSET);
	            if (rxInp == USART_RX_BUFFER_SIZE)
	            {
	                rxInp = 0;
	            }

	            rxCnt++;
	        }
	        else
			{
	        	// Over flow
	        	XUartLite_ReadReg(InstancePtr->RegBaseAddress, XUL_RX_FIFO_OFFSET);
				rxInp = 0;
				rxOut = 0;
				rxCnt = 0;
			}
		}
	}

	if(((IsrStatus & XUL_SR_TX_FIFO_EMPTY) != 0)&&(InstancePtr->SendBuffer.RemainingBytes != (u32)0)) {
		 // Transmit data interrupt
		// If TX FIFO is empty, send more.
		XUartLite_SendBuffer(InstancePtr);
	}

}
*/










// The below is not working probably because buffer in fpga is to small 16 bytes compared to 64 on PS
// I leave it here in case at some point we want to only use very small data packages and prefer simpler solution

/*

// Communication using UART IP (Info: In current hardware platform hpf_v2024_4 connected at JD)


#include "usart.h"
#include "xuartns550.h"
#include "xuartns550_l.h"

#define UARTNS550_DEVICE_ID		XPAR_UARTNS550_0_DEVICE_ID


XUartNs550 UartNs550;			// Instance of the UART Device


void PC_Connection_INIT(unsigned int baud)
{
	XUartNs550_Initialize(&UartNs550, UARTNS550_DEVICE_ID);

	XUartNs550_SetBaud(XPAR_AXI_UART16550_0_BASEADDR, 100000000, 230400);

//	u16 Options = XUN_OPTION_FIFOS_ENABLE;
//	XUartNs550_SetOptions(&UartNs550, Options);	// Not sure if needed
}


void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size)
{
	XUartNs550_Send(&UartNs550, SendBuffer, buffer_size);
}


int Message_GetFromPC(unsigned char * c)
{
	int newDataCount = XUartNs550_Recv(&UartNs550, c, SERIAL_MAX_PKT_LENGTH);
	return newDataCount;
}


*/














/*


// Communication using UART Lite IP (Info: In current hardware platform hpf_v2024_4 connected at JC)


#include "usart.h"
#include "xuartlite.h"


#define UARTLITE_DEVICE_ID		XPAR_UARTLITE_0_DEVICE_ID


XUartLite UartLite;			// Instance of the UART Device


void PC_Connection_INIT(unsigned int baud)
{
    XUartLite_Initialize(&UartLite, UARTLITE_DEVICE_ID);
}


void Message_SendToPC(unsigned char * SendBuffer, unsigned int buffer_size)
{
	XUartLite_Send(&UartLite, SendBuffer, buffer_size);
}


int Message_GetFromPC(unsigned char * c)
{
	int newDataCount = XUartLite_Recv(&UartLite, c, SERIAL_MAX_PKT_LENGTH);
	return newDataCount;
}


*/










