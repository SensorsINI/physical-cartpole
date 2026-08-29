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

#define PS_BTN_4            50 // Button GPIO (PS MIO)
#define PS_BTN_5            51 // Button GPIO (PS MIO)

/* Zybo PL buttons BTN0-BTN3. Distinct from PS MIO numbers so Button_SetAction
 * can bind any of them from firmware after the bitstream wires the GPIO. */
#define PL_BTN_0            0
#define PL_BTN_1            1
#define PL_BTN_2            2
#define PL_BTN_3            3

typedef void (*ActionHandler)(void);

void Buttons_And_Switches_Init(void);
void Button_SetAction(unsigned int key, ActionHandler action);
u32 Switches_GetState();
u32 Switch_GetState(u32 switch_number);

#endif // __BUTTONS_AND_SWITCHES_H_
