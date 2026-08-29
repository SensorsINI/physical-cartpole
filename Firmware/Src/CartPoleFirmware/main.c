#include "hardware_bridge.h"
#include "control.h"
#ifdef ZYNQ
#include "Zynq/qspi_nvparams.h"
#endif


int main(void)
{
	General_Init();
	PC_Connection_Init();
#ifdef ZYNQ
	QspiNv_Init();
#endif
	Buttons_And_Switches_Init();
#ifdef USE_EXTERNAL_INTERFACE
#ifdef ZYNQ
	ExternalInterfaceInit();
#endif
#endif
	Led_Init();
	Button_SetAction(BUTTON_1, CONTROL_ToggleState);
	Button_SetAction(BUTTON_2, CONTROL_ToggleCalibration);  // Not implemented yet for STM
	Button_SetAction(BUTTON_3, CONTROL_SetHangingFromCurrentReading);
	Goniometer_Init();
	Encoder_Init();
	Motor_Init();
	CONTROL_Init();
	Neural_Imitator_Init();  // Doing nothing for STM, introduced for consistency with Zynq

	Interruput_Init();
	SetControlUpdatePeriod(POLLING_PERIOD_MS);			// Not needed for STM, introduced for consistency with Zynq
	Interrupt_Set(CONTROL_Loop);

	while (1)
	{
		CONTROL_BackgroundTask();
	}
}
