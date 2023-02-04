#ifndef __TIMER_H_
#define __TIMER_H_

#include "sys.h"	 

typedef void (*TIMER1_Callback)(void);

void TIMER1_Init(unsigned int periodMS);
void TIMER1_SetCallback(TIMER1_Callback cb);
float TIMER1_getSystemTime(void);
unsigned int TIMER1_getSystemTime_Us(void);

#endif /*__TIMER_H_*/
