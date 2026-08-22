#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"

// Define the stream type using ap_axiu for a 16-bit wide data path
typedef ap_axiu<16, 1, 1, 1> stream_type;

// dz_status bits
#define DZ_STATUS_LOW_RAIL  0x1  // latest raw sample <= rail_low
#define DZ_STATUS_HIGH_RAIL 0x2  // latest raw sample >= rail_high

void median_filter(
    hls::stream<stream_type> &xadc_stream,
    volatile unsigned short* filtered,
    volatile unsigned short* raw,
    short window_size,
    short trim_count,
    short filter_mode,
    unsigned short rail_low,
    unsigned short rail_high,
    volatile unsigned short* dz_status,
    volatile unsigned short* dz_window,
    volatile unsigned short* dz_age,
    volatile unsigned int* dz_low_count,
    volatile unsigned int* dz_high_count
);

#endif // MEDIAN_FILTER_H