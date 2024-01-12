#include "hardware_bridge.h"
#include "control.h"


int main(void)
{
	General_Init();
	PC_Connection_Init();
	Buttons_And_Switches_Init();
	Led_Init();
	Button_SetAction(BUTTON_1, CONTROL_ToggleState);
	Goniometer_Init();
	Encoder_Init();
	Motor_Init();
	CONTROL_Init();

	Interruput_Init();
	SetControlUpdatePeriod(CONTROL_LOOP_PERIOD_MS);			// Not needed for STM, introduced for consistency with Zynq
	Interrupt_Set(CONTROL_Loop);

	while (1)
	{
		CONTROL_BackgroundTask();
	}
}
