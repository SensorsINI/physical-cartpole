#include "mblaze_nt_types.h"
#include "xil_types.h"
#include "math.h"

// Function
float fixed_to_float_32(int32_t input, int coma_pos)
{
	return ((float) input / (float) (1 << coma_pos));
}

float fixed_to_float_16(short input, int coma_pos)
{
	return ((float) input / (float) (1 << coma_pos));
}



int32_t float_to_fixed_32(float input, int coma_pos)
{
	return (int32_t) round((input * (1 << coma_pos)));
}

short float_to_fixed_16(float input, int coma_pos)
{
	return (short) round((input * (1 << coma_pos)));
}



int32_t byte_float_to_fixed_32(byte input[], int coma_pos)
{
	int32_t result;
	float concat;
	concat = (float)((input[3] << 24) & (input[2] << 16) & (input[1] << 8)  & input[0]);
	result = float_to_fixed_32(concat, coma_pos);
	return result;
}

short byte_float_to_fixed_16(byte input[], int coma_pos)
{
	short result;
	float concat;
	concat = (float)((input[3] << 24) & (input[2] << 16) & (input[1] << 8)  & input[0]);
	result = float_to_fixed_16(concat, coma_pos);
	return result;
}



int32_t extend_sign_32(int32_t input, int sign_bit_pos)
{
	int32_t result, sign_bit;
	sign_bit = (input >> sign_bit_pos) & 1;
	result = input;
	if (sign_bit) {
		int mask = ((1 << sign_bit_pos) - 1);
		mask = ~mask;
		result |= mask;
	}
	return result;
}

short extend_sign_16(short input, int sign_bit_pos)
{
	short result, sign_bit;
	sign_bit = (input >> sign_bit_pos) & 1;
	result = input;
	if (sign_bit) {
		int mask = ((1 << sign_bit_pos) - 1);
		mask = ~mask;
		result |= mask;
	}

	return result;
}
