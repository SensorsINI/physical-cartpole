#ifndef __LED_ZYNQ_H_
#define __LED_ZYNQ_H_

#include <stdbool.h>

#include "xparameters.h"
#include "xgpiops.h"
#include "xgpio.h"

extern XGpioPs GpioPS;
extern XGpio Gpio;
XGpio GpioRGB;

void Led_Init(void);
void Led_Switch(bool en);
void Leds_over_switches_Update(u32 leds_state);
void indicate_target_position_with_leds(float* target_position);

#endif /*__LED_ZYNQ_H_*/
