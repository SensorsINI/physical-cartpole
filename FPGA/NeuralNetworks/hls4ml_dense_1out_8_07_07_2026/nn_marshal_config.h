#ifndef NN_MARSHAL_CONFIG_H
#define NN_MARSHAL_CONFIG_H

/*
 * Per-network compile-time configuration for nn_marshal HLS.
 *
 * Regenerate with:
 *   python FPGA/scripts/generate_nn_marshal_config.py \\
 *       --vhd-dir FPGA/NeuralNetworks/hls4ml_dense_1out_8_07_07_2026 \\
 *       --norm-csv-dir <model>/norm_vectors
 *
 * Re-synthesize nn_marshal_hls when this file changes.
 * controller_io_parameters.vhd must match the bit widths below.
 */

#define NN_IO_INPUT_COUNT        7
#define NN_IO_OUTPUT_COUNT       1

#define NN_IO_INPUT_TOTAL_BITS   12
#define NN_IO_INPUT_INT_BITS     2

#define NN_IO_OUTPUT_TOTAL_BITS  12
#define NN_IO_OUTPUT_INT_BITS    2

#define NN_IO_INPUT_FRAC_BITS    (NN_IO_INPUT_TOTAL_BITS - NN_IO_INPUT_INT_BITS)
#define NN_IO_OUTPUT_FRAC_BITS   (NN_IO_OUTPUT_TOTAL_BITS - NN_IO_OUTPUT_INT_BITS)

/* Input wire order: angleD, angle_cos, angle_sin, position, positionD,
 *                   target_equilibrium, target_position */

static const float NN_NORM_A[NN_IO_INPUT_COUNT] = {
    0.04595453f, 1.00000000f, 1.00000000f, 5.21186209f,
    0.82011247f, 1.00000000f, 6.31313133f
};

static const float NN_NORM_B[NN_IO_INPUT_COUNT] = {
    0.02537525f, 0.00000000f, 0.00000000f, 0.01761615f,
    -0.05823207f, 0.00000000f, 0.00000000f
};

static const float NN_DENORM_A[NN_IO_OUTPUT_COUNT] = { 1.00000000f };
static const float NN_DENORM_B[NN_IO_OUTPUT_COUNT] = { 0.00000000f };

#endif /* NN_MARSHAL_CONFIG_H */
