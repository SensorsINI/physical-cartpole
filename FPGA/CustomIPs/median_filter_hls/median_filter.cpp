#include "median_filter.h"
#include "median_functions.h"

// Saturation value for dz_age: at ~450 kS/s this is ~145 ms, far beyond the
// control polling period, so "saturated" simply means "no recent rail contact".
#define DZ_AGE_MAX 0xFFFF

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
) {
	#pragma HLS INTERFACE ap_ctrl_none port=return  //Comment it out and resynthetize to use in RTL cosimulation
	#pragma HLS INTERFACE axis port=xadc_stream
    #pragma HLS INTERFACE s_axilite port=filtered bundle=MEDIAN
    #pragma HLS INTERFACE s_axilite port=raw bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=window_size bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=trim_count bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=filter_mode bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=rail_low bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=rail_high bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=dz_status bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=dz_window bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=dz_age bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=dz_low_count bundle=MEDIAN
	#pragma HLS INTERFACE s_axilite port=dz_high_count bundle=MEDIAN

	unsigned short new_adc_value = 0;

	static short window_size_input = 1;
    static short trim_count_input = 0;
    static short filter_mode_input = FILTER_MODE_TRIMMED_MEAN;
    // Rail thresholds disabled by default (only exact 0 / 0xFFFF would match);
    // the firmware sets sensor-specific values at init.
    static unsigned short rail_low_input = 0;
    static unsigned short rail_high_input = 0xFFFF;

    static unsigned short filtered_value = 0;
    static unsigned short raw_value = 0;
    static unsigned short dz_status_value = 0;
    static unsigned short dz_window_value = 0;
    static unsigned short dz_age_value = DZ_AGE_MAX;
    static unsigned int dz_low_count_value = 0;
    static unsigned int dz_high_count_value = 0;

    // Check if the stream has data available
    if (!xadc_stream.empty()) {
        stream_type sample = xadc_stream.read();
        new_adc_value = sample.data & 0xFFFF;
        raw_value = new_adc_value;

        unsigned short rail_window_count = 0;
        filtered_value = get_filtered_value(new_adc_value, window_size_input, trim_count_input, filter_mode_input,
                                            rail_low_input, rail_high_input, &rail_window_count);
        dz_window_value = rail_window_count;

        // Dead-zone tracking on the latest raw sample. Low and high rails are
        // counted separately so the PS can tell from which side the pole
        // entered the potentiometer gap (crossing vs turning point).
        bool at_low_rail = (new_adc_value <= rail_low_input);
        bool at_high_rail = (new_adc_value >= rail_high_input);
        dz_status_value = (at_low_rail ? DZ_STATUS_LOW_RAIL : 0)
                        | (at_high_rail ? DZ_STATUS_HIGH_RAIL : 0);
        if (at_low_rail) {
            dz_low_count_value++;
        }
        if (at_high_rail) {
            dz_high_count_value++;
        }
        if (at_low_rail || at_high_rail) {
            dz_age_value = 0;
        } else if (dz_age_value != DZ_AGE_MAX) {
            dz_age_value++;
        }
    } else {
    	window_size_input = window_size;
    	trim_count_input = trim_count;
    	filter_mode_input = filter_mode;
    	rail_low_input = rail_low;
    	rail_high_input = rail_high;
    }
    *filtered = filtered_value;
    *raw = raw_value;
    *dz_status = dz_status_value;
    *dz_window = dz_window_value;
    *dz_age = dz_age_value;
    *dz_low_count = dz_low_count_value;
    *dz_high_count = dz_high_count_value;
}