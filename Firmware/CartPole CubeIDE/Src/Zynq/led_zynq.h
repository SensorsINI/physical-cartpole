#ifndef __LED_ZYNQ_H_
#define __LED_ZYNQ_H_

#include <stdbool.h>
#include "xparameters.h"
#include "xgpiops.h"
#include "xgpio.h"

extern XGpioPs GpioPS;

void Led_Init(void);
void Led_Switch(bool en);
void Leds_over_switches_Update(u32 leds_state);

#endif /*__LED_ZYNQ_H_*/
