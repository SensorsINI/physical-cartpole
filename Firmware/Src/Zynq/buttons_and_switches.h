#ifndef __BUTTONS_AND_SWITCHES_H_
#define __BUTTONS_AND_SWITCHES_H_

#include "xgpiops.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"

extern XGpioPs GpioPS;
extern XGpio Gpio;

extern XScuGic XScuGicInstance; // The Instance of the Interrupt Controller Driver

#define PS_BTN_4            50 // Button GPIO
#define PS_BTN_5            51 // Button GPIO

typedef void (*ActionHandler)(void);

void Buttons_And_Switches_Init(void);
void Button_SetAction(unsigned int key, ActionHandler action);
u32 Switches_GetState();
u32 Switch_GetState(u32 switch_number);

#endif // __BUTTONS_AND_SWITCHES_H_

