#include "encoder_zynq.h"

#include <unistd.h>

int direction_sign = 1;

// Uses Timer 4 configured in encoder mode
void Encoder_Init_Zynq(void)
{
	Xil_Out32(ENCODER_RESET_ADDR, (u32)(1));
	usleep(1);
	Xil_Out32(ENCODER_RESET_ADDR, (u32)(0));
}

short Encoder_Read_Zynq()
{
	EncoderValue = Xil_In32(ENCODER_COUNT_ADDR);
	return EncoderValue;
}
