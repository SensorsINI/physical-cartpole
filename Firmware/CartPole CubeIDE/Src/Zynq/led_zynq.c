#include "led_zynq.h"

#define LED_PIN 7 // Replace with the correct MIO pin number for your LED

void Led_Init(void)
{
    // This function need to be called after Buttons_And_Switches_Init
    // Both Gpio for PL and PS are initialised there and only reused here.

	// Set the GPIO direction for the LED pin to be output.
	XGpioPs_SetDirectionPin(&GpioPS, LED_PIN, 1);
	XGpioPs_SetOutputEnablePin(&GpioPS, LED_PIN, 1);

	// Set GPIO for LEDs over switches
	XGpio_SetDataDirection(&Gpio, 2, 0);  // LED, output

}

void Led_Switch(bool en)
{
	XGpioPs_WritePin(&GpioPS, LED_PIN, (u32)(!en));
}

void Leds_over_switches_Update(u32 leds_state)
{
	XGpio_DiscreteWrite(&Gpio, 2, leds_state);
}
