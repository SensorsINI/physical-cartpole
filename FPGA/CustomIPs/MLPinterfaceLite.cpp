#include "ap_int.h"

#define MLP_INPUT_NEURONS    7   // Adjust as per your actual configuration
#define MLP_INPUT_DATA_BITS  12
#define MLP_OUTPUT_NEURONS   1   // Adjust as per your actual configuration
#define MLP_OUTPUT_DATA_BITS 12

#define MLP_INPUT_BIT_WIDTH  (MLP_INPUT_NEURONS * MLP_INPUT_DATA_BITS)
#define MLP_OUTPUT_BIT_WIDTH (MLP_OUTPUT_NEURONS * MLP_OUTPUT_DATA_BITS)

void MLPinterfaceLite(
    ap_uint<32> input_data[MLP_INPUT_NEURONS],
    ap_uint<32> output_data[MLP_OUTPUT_NEURONS],
    bool* start,
    bool* done,
    bool* idle,
    bool* ready,
    // Interface to myproject
    ap_uint<MLP_INPUT_BIT_WIDTH>& input_1_V,
    bool& input_1_V_ap_vld,
    bool& ap_start_r,
    bool& ap_done_r,
    bool& ap_ready_r,
    bool& ap_idle_r,
    ap_uint<MLP_OUTPUT_BIT_WIDTH>& output_out_0_V
    ) {
#pragma HLS INTERFACE s_axilite port=input_data bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=output_data bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=start bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=done bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=idle bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=ready bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL_BUS

#pragma HLS INTERFACE ap_none port=input_1_V
#pragma HLS INTERFACE ap_none port=input_1_V_ap_vld
#pragma HLS INTERFACE ap_none port=ap_start_r
#pragma HLS INTERFACE ap_none port=ap_done_r
#pragma HLS INTERFACE ap_none port=ap_ready_r
#pragma HLS INTERFACE ap_none port=ap_idle_r
#pragma HLS INTERFACE ap_none port=output_out_0_V

    // Declare buffers
    ap_uint<MLP_INPUT_DATA_BITS> input_buffer[MLP_INPUT_NEURONS];
#pragma HLS ARRAY_PARTITION variable=input_buffer complete
    ap_uint<MLP_OUTPUT_DATA_BITS> output_buffer[MLP_OUTPUT_NEURONS];
#pragma HLS ARRAY_PARTITION variable=output_buffer complete

    // Initialize status signals
    *idle = ap_idle_r;
    *ready = ap_ready_r;
    *done = false;

    // Wait for start signal and MLP to be idle
    if (*start && ap_idle_r) {
        // Read input data
        for (int i = 0; i < MLP_INPUT_NEURONS; i++) {
#pragma HLS PIPELINE II=1
            ap_uint<32> input_word = input_data[i];
            input_buffer[i] = input_word.range(MLP_INPUT_DATA_BITS - 1, 0);
        }

        // Assemble input data into single vector
        ap_uint<MLP_INPUT_BIT_WIDTH> temp_input = 0;
        for (int i = 0; i < MLP_INPUT_NEURONS; i++) {
#pragma HLS UNROLL
            temp_input.range((i + 1) * MLP_INPUT_DATA_BITS - 1, i * MLP_INPUT_DATA_BITS) = input_buffer[i];
        }
        input_1_V = temp_input;
        input_1_V_ap_vld = true;
        ap_start_r = true;

        // Wait for MLP to be ready
        do {
#pragma HLS PIPELINE II=1
            *idle = ap_idle_r;
            *ready = ap_ready_r;
        } while (!ap_ready_r);
        ap_start_r = false;
        input_1_V_ap_vld = false;

        // Wait for MLP to be done
        do {
#pragma HLS PIPELINE II=1
            *done = ap_done_r;
        } while (!ap_done_r);

        // Read output data
        ap_uint<MLP_OUTPUT_BIT_WIDTH> temp_output = output_out_0_V;
        for (int i = 0; i < MLP_OUTPUT_NEURONS; i++) {
#pragma HLS UNROLL
            output_buffer[i] = temp_output.range((i + 1) * MLP_OUTPUT_DATA_BITS - 1, i * MLP_OUTPUT_DATA_BITS);
        }

        // Write output data
        for (int i = 0; i < MLP_OUTPUT_NEURONS; i++) {
#pragma HLS PIPELINE II=1
            output_data[i] = output_buffer[i];
        }

        *done = true;
    }
}