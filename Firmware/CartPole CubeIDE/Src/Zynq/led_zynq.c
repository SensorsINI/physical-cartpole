#include "led_zynq.h"

#include "xgpiops.h"
#include "xparameters.h"

#define LED_PIN 7 // Replace with the correct MIO pin number for your LED

XGpioPs Gpio; // Instance of the GPIO Driver

void Led_Init(void)
{
	XGpioPs_Config *ConfigPtr;
	int status;

	// Initialize the GPIO driver
	ConfigPtr = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Set the GPIO direction for the LED pin to be output.
	XGpioPs_SetDirectionPin(&Gpio, LED_PIN, 1);
	XGpioPs_SetOutputEnablePin(&Gpio, LED_PIN, 1);
}

void Led_Switch(bool en)
{
	XGpioPs_WritePin(&Gpio, LED_PIN, (u32)(!en));
}
