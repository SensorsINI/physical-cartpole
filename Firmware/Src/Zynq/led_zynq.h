#ifndef __LED_ZYNQ_H_
#define __LED_ZYNQ_H_

#include <stdbool.h>

#include "xparameters.h"
#include "xgpiops.h"
#include "xgpio.h"

extern XGpioPs GpioPS;
extern XGpio Gpio;
extern XGpio GpioRGB;

void Led_Init(void);
void Led_Switch(bool en);
void Leds_over_switches_Update(u32 leds_state);
void Led_RgbConfirmFlash(void);
void Led_RgbUprightCaptureStart(void);
void Led_RgbUprightCaptureSuccess(void);
void Led_RgbUprightCaptureError(void);
void indicate_target_position_with_leds(float* target_position, bool dead_zone_warning);

#endif /*__LED_ZYNQ_H_*/
