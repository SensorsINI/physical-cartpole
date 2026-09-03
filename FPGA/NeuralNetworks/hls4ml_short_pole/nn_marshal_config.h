#ifndef NN_MARSHAL_CONFIG_H
#define NN_MARSHAL_CONFIG_H

/*
 * IROS24 short pole: Experiment-29-30 Dense-7IN-32H1-32H2-1OUT-1
 * PL VHDL FPGA/NeuralNetworks/hls4ml_short_pole (packed 7x14 in, 14 out).
 * Firmware twin: hpf_v2024_cpp_x3232_14_3_short_v1 (commit ae2d3371).
 *
 * Re-synthesize nn_marshal_hls when this file changes.
 * controller_io_parameters.vhd must use BITS_PER_* = 14.
 */

#define NN_IO_INPUT_COUNT        7
#define NN_IO_OUTPUT_COUNT       1

#define NN_IO_INPUT_TOTAL_BITS   14
#define NN_IO_INPUT_INT_BITS     3

#define NN_IO_OUTPUT_TOTAL_BITS  14
#define NN_IO_OUTPUT_INT_BITS    3

#define NN_IO_INPUT_FRAC_BITS    (NN_IO_INPUT_TOTAL_BITS - NN_IO_INPUT_INT_BITS)
#define NN_IO_OUTPUT_FRAC_BITS   (NN_IO_OUTPUT_TOTAL_BITS - NN_IO_OUTPUT_INT_BITS)

/* Input wire order: angleD, angle_cos, angle_sin, position, positionD,
 *                   target_equilibrium, target_position */

static const float NN_NORM_A[NN_IO_INPUT_COUNT] = {
    0.03312271f, 1.00000000f, 1.00000000f, 5.43271589f,
    1.19410825f, 1.00000000f, 6.31313133f
};

static const float NN_NORM_B[NN_IO_INPUT_COUNT] = {
    -0.01278764f, 0.00000000f, 0.00000000f, -0.00592160f,
    -0.01507568f, 0.00000000f, 0.00000000f
};

static const float NN_DENORM_A[NN_IO_OUTPUT_COUNT] = { 1.00000000f };
static const float NN_DENORM_B[NN_IO_OUTPUT_COUNT] = { 0.00000000f };

#endif /* NN_MARSHAL_CONFIG_H */
