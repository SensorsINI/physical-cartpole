
#include "mblaze_nt_types.h"

// Define the fractional bits used in your representation
#define FIXED_POINT_FRACTIONAL_BITS 14
#define SIGN_BIT_POSITION 20

// This fixed point representation is: 32-FRACTIONAL_BITS . FRACTIONAL_BITS
typedef int32_t fixed_point_t;

// Function definitions
float fixed_to_float(fixed_point_t input);

fixed_point_t float_to_fixed(float input);
fixed_point_t byte_float_to_fixed(byte input[]);

fixed_point_t extend_sign(fixed_point_t input);


// Convert from float to Fixed Point
inline float fixed_to_float(fixed_point_t input) {
	return ((float) input / (float) (1 << FIXED_POINT_FRACTIONAL_BITS));
}


// Convert from Fixed Point to float
inline fixed_point_t float_to_fixed(float input) {
	return (fixed_point_t) (input * (1 << FIXED_POINT_FRACTIONAL_BITS));
}

fixed_point_t byte_float_to_fixed(byte input[]) {
	fixed_point_t result;
	float concat;
	concat = (float)((input[3] << 24) & (input[2] << 16) & (input[1] << 8)  & input[0]);
	result = float_to_fixed(concat);
	return result;
}


// Extend the sign to 32bits
inline fixed_point_t extend_sign(fixed_point_t input) {
	fixed_point_t result;
	if ((input & 0x00080000) != 0) {
		result = input | 0xFFF80000;
	} else {
		result = input & 0x000FFFFF;
	}
	return result;
}
