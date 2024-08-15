#include "quadrature_encoder.h"

// Quadrature encoding table for state transitions
// Index into this table using [lastA, currentA, lastB, currentB]
const int quad_encoding_table[16] = {
    0,  // 0000 INVALID OK
    1,  // 0001 OK
    -1, // 0010 OK
    0,  // 0011 INVALID OK
    -1, // 0100 OK
    0,  // 0101 INVALID OK
    0,  // 0110 INVALID
    1,  // 0111 OK
    1,  // 1000 OK
    0,  // 1001 INVALID OK
    0,  // 1010 INVALID OK
    -1, // 1011
    0,  // 1100 INVALID OK
    -1, // 1101 OK
    1,  // 1110 OK
    0   // 1111 INVALID OK
};


void quadrature_encoder(bool A, bool B, bool reset, volatile int *count) {
#pragma HLS INTERFACE s_axilite port=count bundle=ENCODER_AXI
#pragma HLS INTERFACE s_axilite port=reset bundle=ENCODER_AXI
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE ap_none port=A
#pragma HLS INTERFACE ap_none port=B

    static int lastA = 0; // Previous state of A
    static int lastB = 0; // Previous state of B
    static int curr_count = 0; // Current count value

    if (reset){
    	curr_count = 0;
    }

    int index = (lastA << 3) | (A << 2) | (lastB << 1) | B;
    curr_count += quad_encoding_table[index];

    lastA = A;
    lastB = B;

    // Set the count value via AXI Lite
    *count = curr_count;
}

