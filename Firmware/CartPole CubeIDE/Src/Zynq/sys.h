#ifndef __SYS_H_
#define __SYS_H_

#include "xscugic.h"
#include "xil_exception.h"

#define INTC_DEVICE_ID      XPAR_SCUGIC_SINGLE_DEVICE_ID

XScuGic XScuGicInstance; // The Instance of the Interrupt Controller Driver

void Sleep_ms(unsigned int ms);
void General_Init(void);

#define POLOLU_MOTOR // define if using replacement Pololu motor with reversed polarity; see also encoder.c

#endif /*__SYS_H_*/
