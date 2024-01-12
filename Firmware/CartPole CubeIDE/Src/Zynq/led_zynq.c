#include "led_zynq.h"

#define LED_PIN 7 // Replace with the correct MIO pin number for your LED

XGpio GpioLEDs; // Initialize LEDs over switches on Zybo Z7-20, accessible through PL

void Led_Init(void)
{
	XGpioPs_Config *ConfigPtr;

	// Initialize the GPIO driver
	ConfigPtr = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);

	XGpioPs_CfgInitialize(&GpioPS, ConfigPtr, ConfigPtr->BaseAddr);

	// Set the GPIO direction for the LED pin to be output.
	XGpioPs_SetDirectionPin(&GpioPS, LED_PIN, 1);
	XGpioPs_SetOutputEnablePin(&GpioPS, LED_PIN, 1);

	// Set GPIO for LEDs over switches
	XGpio_Initialize(&GpioLEDs, XPAR_GPIO_0_DEVICE_ID);

	XGpio_SetDataDirection(&GpioLEDs, 2, 0);  // LED, output
}

void Led_Switch(bool en)
{
	XGpioPs_WritePin(&GpioPS, LED_PIN, (u32)(!en));
}

void Leds_over_switches_Update(u32 leds_state)
{
	XGpio_DiscreteWrite(&GpioLEDs, 2, leds_state);
}
