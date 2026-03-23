#include "ap_axi_sdata.h"  // Needed for AXI Stream
#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

#define NR_INPUTS 7  // Define the number of registers

// Define AXI Stream Data Packet with 32-bit data and 1-bit user field
typedef ap_axiu<32, 0, 0, 0> axis_t;

// Top-level function
void MM2S(
    volatile ap_uint<32> *reg_in,
    hls::stream<axis_t> &stream_out
) {
    #pragma HLS INTERFACE s_axilite port=reg_in bundle=control
    #pragma HLS INTERFACE s_axilite port=return bundle=control
    #pragma HLS INTERFACE axis port=stream_out

    axis_t tmp;
    for (int i = 0; i < NR_INPUTS; i++) {
        #pragma HLS PIPELINE
        tmp.data = reg_in[i];
        tmp.last = (i == NR_INPUTS - 1) ? 1 : 0;
        stream_out.write(tmp);
    }
}
