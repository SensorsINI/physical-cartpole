#ifndef TIMER_INTERRUPT_H
#define TIMER_INTERRUPT_H

#include "xparameters.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xtime_l.h"

#define TIMER_LOAD_VALUE_1s	333333343 // 1s
#define TIMER_LOAD_VALUE_1us	333 // 1s

#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR

typedef void (*TIMER1_Callback)(void);
TIMER1_Callback timer_interrupt;

void TIMER1_Init(unsigned int _periodMS);

void SetControlUpdatePeriod(unsigned int _periodMS);

unsigned long TIMER1_getSystemTime_Us();

void Interrupt_Set(TIMER1_Callback cb);

unsigned long TIMER1_getSystemTime_Us();

/************************** Variable Definitions *****************************/

extern XScuTimer TimerInstance;	/* Cortex A9 Scu Private Timer Instance */
extern XScuGic XScuGicInstance;		/* Interrupt Controller Instance */
extern XTime MyTime;


#endif
