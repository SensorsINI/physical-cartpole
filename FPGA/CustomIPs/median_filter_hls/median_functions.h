#ifndef MEDIAN_FUNCTIONS_H
#define MEDIAN_FUNCTIONS_H

#define MAX_WINDOW_SIZE 64
#define FILTER_MODE_RAW 0
#define FILTER_MODE_MEDIAN 1
#define FILTER_MODE_TRIMMED_MEAN 2

// rail_low/rail_high define the "near rail" bands (16-bit filter domain,
// inclusive: v <= rail_low or v >= rail_high). rail_window_count returns how
// many samples of the current window fall in those bands, i.e. how
// contaminated the filtered output is by dead-zone samples.
unsigned short get_filtered_value(unsigned short newValue, short window_size, short trim_count, short filter_mode,
                                  unsigned short rail_low, unsigned short rail_high,
                                  unsigned short *rail_window_count);

#endif // MEDIAN_FUNCTIONS_H
