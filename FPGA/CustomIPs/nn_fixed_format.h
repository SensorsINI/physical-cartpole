#ifndef NN_FIXED_FORMAT_H
#define NN_FIXED_FORMAT_H

/*
 * Compile-time fixed-point quantize / dequantize helpers for nn_marshal.
 *
 * Width and fractionality come from the deployed network's nn_marshal_config.h.
 */

#include <cmath>
#include <ap_int.h>

#ifndef NN_IO_INPUT_TOTAL_BITS
#error "Include nn_marshal_config.h before nn_fixed_format.h"
#endif

#define NN_FIXED_FRAC_BITS(TOTAL, INT) ((TOTAL) - (INT))
#define NN_FIXED_MAX_POS(TOTAL)        ((1 << ((TOTAL) - 1)) - 1)
#define NN_FIXED_MIN_NEG(TOTAL)        (-(1 << ((TOTAL) - 1)))
#define NN_FIXED_MASK(TOTAL)           ((unsigned)((TOTAL) >= 32 ? 0xFFFFFFFFu : ((1u << (TOTAL)) - 1u)))

static inline ap_uint<32> nn_fixed_quantize(
    float normalized, int total_bits, int frac_bits)
{
    const float scaled = normalized * (float)(1 << frac_bits);
    int code = (int)::roundf(scaled);
    const int max_pos = NN_FIXED_MAX_POS(total_bits);
    const int min_neg = NN_FIXED_MIN_NEG(total_bits);
    if (code > max_pos) code = max_pos;
    if (code < min_neg) code = min_neg;
    return (ap_uint<32>)(((unsigned)code) & NN_FIXED_MASK(total_bits));
}

static inline ap_uint<32> nn_fixed_quantize_input(float normalized)
{
    return nn_fixed_quantize(
        normalized,
        NN_IO_INPUT_TOTAL_BITS,
        NN_IO_INPUT_FRAC_BITS);
}

static inline float nn_fixed_dequantize_output(ap_uint<32> code)
{
    ap_int<NN_IO_OUTPUT_TOTAL_BITS> raw = code(NN_IO_OUTPUT_TOTAL_BITS - 1, 0);
    return (float)raw * (1.0f / (float)(1 << NN_IO_OUTPUT_FRAC_BITS));
}

static inline ap_uint<32> nn_fixed_extract_output_code(ap_uint<32> beat_data)
{
    return beat_data(NN_IO_OUTPUT_TOTAL_BITS - 1, 0);
}

#endif /* NN_FIXED_FORMAT_H */
