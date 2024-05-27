#include "encoder_zynq.h"

#include <unistd.h>

short EncoderDirection = 1;

// Uses Timer 4 configured in encoder mode
void Encoder_Init(void)
{
	Xil_Out32(ENCODER_RESET_ADDR, (u32)(1));
	usleep(1);
	Xil_Out32(ENCODER_RESET_ADDR, (u32)(0));
}

short Encoder_Read()
{
	EncoderValue = EncoderDirection * Xil_In32(ENCODER_COUNT_ADDR);
	return EncoderValue;
}

void Encoder_Set_Direction(short new_direction)
{
	EncoderDirection = new_direction;
}
