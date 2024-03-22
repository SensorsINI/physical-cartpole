#include "timer_interrupt.h"

XScuTimer TimerInstance;	/* Cortex A9 Scu Private Timer Instance */
XTime MyTime;

unsigned long TIMER1_getSystemTime_Us() {
		XTime_GetTime(&MyTime);
		return (unsigned long)(1000000.0*(float)MyTime/TIMER_LOAD_VALUE_1s);

}

static void Timer_Intr_Handler(void *CallBackRef);

void TIMER1_Init(unsigned int _periodMS){

	XScuTimer_Config *ConfigPtr;

	ConfigPtr = XScuTimer_LookupConfig(TIMER_DEVICE_ID);

	XScuTimer_CfgInitialize(&TimerInstance, ConfigPtr,
					ConfigPtr->BaseAddr);

	XScuTimer_SelfTest(&TimerInstance);


	XScuGic_Connect(&XScuGicInstance, TIMER_IRPT_INTR,
				(Xil_ExceptionHandler)Timer_Intr_Handler,
				(void *)&TimerInstance);

	XScuGic_Enable(&XScuGicInstance, TIMER_IRPT_INTR);

	XScuTimer_EnableInterrupt(&TimerInstance);

	XScuTimer_EnableAutoReload(&TimerInstance);

	SetControlUpdatePeriod(_periodMS);

}


void SetControlUpdatePeriod(unsigned int _periodMS){

	int _period;
	_period = (int)((float)_periodMS*TIMER_LOAD_VALUE_1s/1000.0);

	XScuTimer_Stop(&TimerInstance);

	XScuTimer_LoadTimer(&TimerInstance, _period);

	XScuTimer_Start(&TimerInstance);

}


TIMER1_Callback timer_interrupt = NULL;

static void Timer_Intr_Handler(void *CallBackRef){
	XScuTimer *my_Timer_LOCAL = (XScuTimer *) CallBackRef;
	if (XScuTimer_IsExpired(my_Timer_LOCAL)) {
		if (timer_interrupt != NULL) {
			timer_interrupt(); // Call the function pointed to by timer_interrupt
		    }
		XScuTimer_ClearInterruptStatus(my_Timer_LOCAL);
	}

}

void Interrupt_Set(TIMER1_Callback cb){

	timer_interrupt = cb;
}
