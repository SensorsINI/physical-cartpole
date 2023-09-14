#include "sys.h"
#include "led.h"
#include "key.h"
#include "angle.h"
#include "encoder.h"
#include "usart.h"
#include "timer.h"
#include "control.h"

#define UART_BAUD 115200 	// 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000

void enable_control(void);

int main(void)
{
	SYS_Init();
	SYS_JTAG_Set(SYS_JTAG_SWD_DISABLE);
	SYS_JTAG_Set(SYS_SWD_ENABLE);
	USART_Init(UART_BAUD, true);
	LED_Init();
	KEY_Init();
	KEY_SetCallback(KEY_5, CONTROL_ToggleState);
	ANGLE_Init();
	ENCODER_Init();
	MOTOR_Init();
	CONTROL_Init();

	TIMER1_Init(CONTROL_LOOP_PERIOD_MS);
	TIMER1_SetCallback(CONTROL_Loop);

	while (1)
	{
		CONTROL_BackgroundTask();
	}
}
