#include "led_zynq.h"

#include "../hardware_bridge.h"

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

	// Set GPIO for RGB LEDs
#ifdef ZYBO_Z720
    XGpio_Initialize(&GpioRGB, XPAR_RGB_LED_GPIO_DEVICE_ID);
    XGpio_SetDataDirection(&GpioRGB, 1, 0);
#endif

}

void Led_Switch(bool en)
{
	XGpioPs_WritePin(&GpioPS, LED_PIN, (u32)(!en));
}

void Leds_over_switches_Update(u32 leds_state)
{
	XGpio_DiscreteWrite(&Gpio, 2, leds_state);
}

void get_rgb_diodes_input(u32 diode_1, u32 diode_2, u32* diodes)
{
    // Ensure only the 3 LSBs are set
    diode_1 &= 0x7; // 0x7 is 0000 0111 in binary, ensuring only 3 LSBs are considered
    diode_2 &= 0x7; // Same for diode_2

    // Shift diode_2 to its correct position, which is 3 bits to the left
    diode_2 <<= 3; // Now diode_2 is prepared for the next 3 bits position

    *diodes = diode_1 | diode_2;

}

u32 rgb_diodes_input_previous;
const u32 dimmer_counter_max = 10;
u32 dimmer_counter = 0;
void indicate_target_position_with_leds(float* target_position)
{
#ifdef ZYBO_Z720
	// 1 - blue,
	// 2 - green,
	// 3 - cyan,
	// 4 - red,
	// 5 - magenta,
	// 6 - yellow, looks greenisch
	// 7 - white

	u32 diode_1;
	u32 diode_2;

	if (*target_position == 0.0){
		diode_1 = 4;
		diode_2 = 4;
	} else if (*target_position > 0){
		diode_1 = 0;
		diode_2 = 2;
	} else if (*target_position < 0.0){
		diode_1 = 1;
		diode_2 = 0;
	} else {
		diode_1 = 4;
		diode_2 = 0;
	}

	u32 rgb_diodes_input;
	get_rgb_diodes_input(diode_1, diode_2, &rgb_diodes_input);

	++dimmer_counter;
	if (rgb_diodes_input!=rgb_diodes_input_previous){
		XGpio_DiscreteWrite(&GpioRGB, 1, rgb_diodes_input);
		dimmer_counter = 0;
	} else if(dimmer_counter%dimmer_counter_max!=0){
		XGpio_DiscreteWrite(&GpioRGB, 1, 0);
	} else {
		XGpio_DiscreteWrite(&GpioRGB, 1, rgb_diodes_input);
		dimmer_counter = 0;
	}

	rgb_diodes_input_previous = rgb_diodes_input;
#endif
// You can also use diodes over switches
// This is what I did while in hurry for IROS submission
// It would be a solution for Zedboard which does not have RGB diodes
//	if (*target_position == 0.0){
//		Leds_over_switches_Update(1);
//	} else if (*target_position>0){
//		Leds_over_switches_Update(3);
//	} else if (*target_position<0.0){
//		Leds_over_switches_Update(7);
//	} else {
//		Leds_over_switches_Update(0);
//	}

}
