#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"

// Define the stream type using ap_axiu for a 16-bit wide data path
typedef ap_axiu<16, 1, 1, 1> stream_type;


void median_filter(hls::stream<stream_type> &xadc_stream, volatile unsigned short* median, short window_size);

#endif // MEDIAN_FILTER_H
