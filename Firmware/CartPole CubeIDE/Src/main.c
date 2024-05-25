#include "hardware_bridge.h"
#include "control.h"


int main(void)
{
	General_Init();
	PC_Connection_Init();
	Buttons_And_Switches_Init();
#ifdef USE_EXTERNAL_INTERFACE
	ExternalInterfaceInit();
#endif
	Led_Init();
	Button_SetAction(BUTTON_1, CONTROL_ToggleState);
	Button_SetAction(BUTTON_2, cmd_Calibrate);  // Not implemented yet for STM
	Goniometer_Init();
	Encoder_Init();
	Motor_Init();
	CONTROL_Init();
	Neural_Imitator_Init();  // Doing nothing for STM, introduced for consistency with Zynq

	Interruput_Init();
	SetControlUpdatePeriod(CONTROL_LOOP_PERIOD_MS);			// Not needed for STM, introduced for consistency with Zynq
	Interrupt_Set(CONTROL_Loop);

	while (1)
	{
		CONTROL_BackgroundTask();
	}
}
