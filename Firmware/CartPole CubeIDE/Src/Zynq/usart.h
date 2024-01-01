#ifndef __USART_H_
#define __USART_H_

#include "xparameters.h"
#include "xstatus.h"
#include "xil_types.h"
#include "xil_assert.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "xil_printf.h"
#include "stdbool.h"
#include "xscugic.h"
#include "../hardware_bridge.h"

/************************** Constant Definitions ***************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define UART_BASEADDR		XPAR_XUARTPS_0_BASEADDR
#define UART_CLOCK_HZ		XPAR_XUARTPS_0_CLOCK_HZ

#define SERIAL_MAX_PKT_LENGTH		32

extern XScuGic XScuGicInstance; // The Instance of the Interrupt Controller Driver

void USART_Init(unsigned int baud, bool interruptEn);
void USART_SendBuffer(unsigned char * SendBuffer, unsigned int buffer_size);
bool USART_ReceiveAsync(unsigned char * c);
void Handler(void *CallBackRef, u32 Event, unsigned int EventData);


#endif /*__USART_H_*/

