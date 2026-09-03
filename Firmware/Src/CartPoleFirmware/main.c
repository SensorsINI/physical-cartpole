#include "hardware_bridge.h"
#include "control.h"
#ifdef ZYNQ
#include "Zynq/qspi_nvparams.h"
#endif
#if defined(RPGD_DUAL_CORE) && !defined(RPGD_WORKER_ONLY)
#include "Zynq/amp_ipc.h"
#endif
#ifdef RPGD_ON_TARGET_TEST
#include "rpgd_on_target_test.h"
#endif


int main(void)
{
	General_Init();
#ifdef RPGD_ON_TARGET_TEST
	Motor_Init();
	Motor_Stop();
	PC_Connection_Init();
#ifdef ZYNQ
	Gic_AmpEnableDistributor();
#endif
#if defined(RPGD_DUAL_CORE) && !defined(RPGD_WORKER_ONLY)
	amp_ipc_load_and_start_cpu1();
#endif
	rpgd_on_target_test_run();
	for (;;) { }
#else
	PC_Connection_Init();
#if defined(RPGD_DUAL_CORE) && !defined(RPGD_WORKER_ONLY)
	amp_ipc_load_and_start_cpu1();
#endif
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
	Button_SetAction(BUTTON_4, CONTROL_SetUprightFromCurrentReading);
	Goniometer_Init();
	Encoder_Init();
	Motor_Init();
	CONTROL_Init();
	Neural_Imitator_Init();  // Doing nothing for STM, introduced for consistency with Zynq

	Interruput_Init();
	SetControlUpdatePeriod(POLLING_PERIOD_MS);			// Not needed for STM, introduced for consistency with Zynq
	Interrupt_Set(CONTROL_Loop);
#ifdef ZYNQ
	Gic_AmpEnableDistributor();
#endif

	while (1)
	{
		CONTROL_BackgroundTask();
	}
#endif
}
