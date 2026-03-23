#include "ap_axi_sdata.h"  // Needed for AXI Stream
#include "hls_stream.h"
#include "ap_int.h"        // Provides ap_uint and ap_int data types
#include "ap_axi_sdata.h"

// Define the number of registers statically (compile-time constant)
#define NR_INPUTS 1  // Set the number of registers here

// Define AXI Stream Data Packet with 32-bit data and 1-bit user field
typedef ap_axiu<32, 0, 0, 0> axis_t;

// Top-level function
void S2MM(
    hls::stream<axis_t> &stream_in,
    volatile ap_uint<32> *reg_out
) {
    #pragma HLS INTERFACE s_axilite port=reg_out bundle=control
    #pragma HLS INTERFACE s_axilite port=return bundle=control
    #pragma HLS INTERFACE axis port=stream_in

    for (int i = 0; i < NR_INPUTS; i++) {
        #pragma HLS PIPELINE
        axis_t tmp = stream_in.read();
        reg_out[i] = tmp.data;
        if (tmp.last == 1) {
            break;
        }
    }
}
