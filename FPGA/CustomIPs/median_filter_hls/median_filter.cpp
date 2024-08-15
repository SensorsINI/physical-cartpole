#include "median_filter.h"
#include "median_functions.h"

void median_filter(hls::stream<stream_type> &xadc_stream, volatile unsigned short* median, short window_size) {
	#pragma HLS INTERFACE ap_ctrl_none port=return  //Comment it out and resynthetize to use in RTL cosimulation
	#pragma HLS INTERFACE axis port=xadc_stream
    #pragma HLS INTERFACE s_axilite port=median bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=window_size bundle=MEDIAN

	unsigned short new_adc_value = 0;

	static short window_size_input = 1;

    static unsigned short median_value = 0;

    // Check if the stream has data available
    if (!xadc_stream.empty()) {
        stream_type sample = xadc_stream.read();
        new_adc_value = sample.data & 0xFFFF;

        median_value = get_median(new_adc_value, window_size_input);

//        median_value = new_adc_value & 0xFFFF;
        // Assuming a 32-bit wide AXI Lite interface. Masking to ensure data fits into 32 bits.
    } else {
    	window_size_input = window_size;
    }
    *median = median_value;


}
