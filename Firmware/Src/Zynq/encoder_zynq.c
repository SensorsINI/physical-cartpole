#include "encoder_zynq.h"

#include <unistd.h>

int EncoderValue;
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
	// Cast to signed int first to correctly interpret negative values from FPGA
	int raw_count = (int)Xil_In32(ENCODER_COUNT_ADDR);
	EncoderValue = EncoderDirection * raw_count;
	return (short)EncoderValue;
}

void Encoder_Set_Direction(short new_direction)
{
	EncoderDirection = new_direction;
}
