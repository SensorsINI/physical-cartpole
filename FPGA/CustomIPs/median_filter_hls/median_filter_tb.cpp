#include "median_filter.h"
#include "median_functions.h"
#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <vector>

// Reference model mirroring the DUT arithmetic exactly (integer truncation and rounding).
static unsigned short reference_filtered(const std::deque<unsigned short> &samples,
                                         unsigned short new_value,
                                         short trim_count,
                                         short filter_mode) {
    if (filter_mode == FILTER_MODE_RAW) {
        return new_value;
    }

    std::vector<unsigned short> sorted(samples.begin(), samples.end());
    std::sort(sorted.begin(), sorted.end());
    short n = (short)sorted.size();

    if (filter_mode == FILTER_MODE_MEDIAN) {
        if (n % 2 != 0) {
            return sorted[n / 2];
        }
        return (unsigned short)((sorted[(n - 1) / 2] + sorted[n / 2]) / 2);
    }

    // Trimmed mean, same clamping as calculate_trimmed_mean()
    if (trim_count < 0) {
        trim_count = 0;
    }
    if (2 * trim_count >= n) {
        trim_count = (n - 1) / 2;
    }
    short kept_count = n - 2 * trim_count;
    unsigned int sum = 0;
    for (short i = trim_count; i < n - trim_count; i++) {
        sum += sorted[i];
    }
    return (unsigned short)((sum + kept_count / 2) / kept_count);
}

static unsigned short reference_rail_window(const std::deque<unsigned short> &samples,
                                            unsigned short rail_low,
                                            unsigned short rail_high) {
    unsigned short count = 0;
    for (unsigned short v : samples) {
        if (v <= rail_low || v >= rail_high) {
            count++;
        }
    }
    return count;
}

int main() {
    hls::stream<stream_type> test_stream;
    stream_type input_sample;

    volatile unsigned short filtered_output;
    volatile unsigned short raw_output;
    volatile unsigned short dz_status_output;
    volatile unsigned short dz_window_output;
    volatile unsigned short dz_age_output;
    volatile unsigned int dz_low_count_output;
    volatile unsigned int dz_high_count_output;

    std::ofstream outfile("test_output.txt");
    if (!outfile.is_open()) {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    int test_length = 12000;
    int data_period = 10;

    // Requested parameters (written by "AXI Lite" every call)
    short window_size = 63;
    short trim_count = 7;
    short filter_mode = FILTER_MODE_TRIMMED_MEAN;
    // 12-bit codes 20 / 4090 in the left-aligned 16-bit filter domain
    unsigned short rail_low = 20 * 16;
    unsigned short rail_high = 4090 * 16;

    // Parameters as latched by the DUT (it latches only on calls with no stream data)
    short latched_window_size = window_size;
    short latched_trim_count = trim_count;
    short latched_filter_mode = filter_mode;
    unsigned short latched_rail_low = rail_low;
    unsigned short latched_rail_high = rail_high;
    bool params_latched_once = false;

    // Reference window state
    std::deque<unsigned short> model_samples;
    short model_window_size = 0;

    // Reference dead-zone state
    unsigned int model_low_count = 0;
    unsigned int model_high_count = 0;
    unsigned short model_age = 0xFFFF;

    int sample_count = 0;
    int error_count = 0;

    for (int cycle = 0; cycle < test_length; cycle++) {
        // Exercise all modes plus a runtime window-size change
        if (cycle == 1000) {
            filter_mode = FILTER_MODE_MEDIAN;
        }
        if (cycle == 2000) {
            filter_mode = FILTER_MODE_TRIMMED_MEAN;
            trim_count = 0; // pure average
        }
        if (cycle == 2600) {
            window_size = 31; // shrink window at runtime
            trim_count = 3;
        }
        if (cycle == 3300) {
            filter_mode = FILTER_MODE_RAW;
        }
        if (cycle == 3900) {
            window_size = 63; // grow window back at runtime
            trim_count = 7;
            filter_mode = FILTER_MODE_TRIMMED_MEAN;
        }
        if (cycle == 5500) {
            filter_mode = FILTER_MODE_MEDIAN;
        }
        if (cycle == 6500) {
            filter_mode = FILTER_MODE_TRIMMED_MEAN;
        }
        if (cycle == 10500) {
            // Runtime threshold change: widen the rail bands
            rail_low = 100 * 16;
            rail_high = 4000 * 16;
        }

        bool new_data = (cycle % data_period == 0);
        unsigned short value = 0;
        if (new_data) {
            if (cycle < 3900) {
                // Noisy signal with duplicates and occasional large glitches
                value = (unsigned short)(30000 + ((cycle * 7) % 64) - 32);
                if ((cycle / data_period) % 13 == 0) {
                    value += 5000; // outlier spike
                }
            } else if (cycle < 6500) {
                // Heavy-duplicate stress segment: only 8 distinct values, so the
                // remove-one-instance and insert-tie logic gets exercised hard.
                value = (unsigned short)(100 + (cycle * 31) % 8);
                // NOTE: these values are <= rail_low, so this segment also
                // stress-tests a long dwell at the low rail (turning point
                // where the pole parks inside the dead zone).
            } else if (cycle < 8000) {
                // DEAD-ZONE CROSSING: mid-range -> high rail -> RC-like decay
                // through mid values -> low rail -> resume mid-range.
                int k = (cycle - 6500) / data_period;
                if (k < 40)       value = (unsigned short)(35000 + (k * 13) % 32);
                else if (k < 70)  value = 65500;                    // railed high
                else if (k < 76)  value = (unsigned short)(62000 >> (k - 70)); // fast decay
                else if (k < 110) value = 96;                       // railed low (dwell in gap)
                else              value = (unsigned short)(400 + (k - 110) * 50); // back on track
            } else if (cycle < 10500) {
                // TURNING POINT: approach the high rail, dwell, and leave the
                // SAME side without ever touching the low rail. Checks that
                // low/high counters separate the two cases.
                int k = (cycle - 8000) / data_period;
                if (k < 60)        value = (unsigned short)(50000 + k * 260);  // approaching
                else if (k < 130)  value = 65530;                              // parked at high rail
                else               value = (unsigned short)(65440 - (k - 130) * 260); // retreat same side
            } else {
                // Signal near the OLD low rail band, after thresholds were
                // widened at runtime: 1000 (12-bit ~62) is inside the new
                // rail_low=1600 band but outside the old 320 band.
                value = (unsigned short)(1000 + (cycle * 17) % 16);
            }
            input_sample.data = value;
            test_stream.write(input_sample);
        }

        median_filter(test_stream, &filtered_output, &raw_output,
                      window_size, trim_count, filter_mode,
                      rail_low, rail_high,
                      &dz_status_output, &dz_window_output, &dz_age_output,
                      &dz_low_count_output, &dz_high_count_output);

        if (new_data) {
            if (!params_latched_once) {
                // DUT has not latched anything yet: it still uses its power-on defaults.
                // Skip checking this sample (only happens if data arrives before any idle call).
                model_samples.clear();
                model_window_size = 0;
            } else {
                // Mirror DUT behavior: window size change resets the window
                if (latched_window_size != model_window_size) {
                    model_samples.clear();
                    model_window_size = latched_window_size;
                }
                model_samples.push_back(value);
                if ((short)model_samples.size() > model_window_size) {
                    model_samples.pop_front();
                }

                unsigned short expected =
                    reference_filtered(model_samples, value, latched_trim_count, latched_filter_mode);

                // Dead-zone reference model
                bool low = (value <= latched_rail_low);
                bool high = (value >= latched_rail_high);
                unsigned short expected_status = (low ? DZ_STATUS_LOW_RAIL : 0)
                                               | (high ? DZ_STATUS_HIGH_RAIL : 0);
                if (low) model_low_count++;
                if (high) model_high_count++;
                if (low || high) model_age = 0;
                else if (model_age != 0xFFFF) model_age++;
                unsigned short expected_window =
                    reference_rail_window(model_samples, latched_rail_low, latched_rail_high);

                sample_count++;
                if (raw_output != value) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": raw=" << raw_output
                            << " expected " << value << std::endl;
                }
                if (filtered_output != expected) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": filtered=" << filtered_output
                            << " expected " << expected
                            << " (mode " << latched_filter_mode
                            << ", trim " << latched_trim_count
                            << ", window " << model_samples.size() << ")" << std::endl;
                }
                if (dz_status_output != expected_status) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": dz_status=" << dz_status_output
                            << " expected " << expected_status << std::endl;
                }
                if (dz_window_output != expected_window) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": dz_window=" << dz_window_output
                            << " expected " << expected_window << std::endl;
                }
                if (dz_age_output != model_age) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": dz_age=" << dz_age_output
                            << " expected " << model_age << std::endl;
                }
                if (dz_low_count_output != model_low_count) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": dz_low_count=" << dz_low_count_output
                            << " expected " << model_low_count << std::endl;
                }
                if (dz_high_count_output != model_high_count) {
                    error_count++;
                    outfile << "FAIL cycle " << cycle << ": dz_high_count=" << dz_high_count_output
                            << " expected " << model_high_count << std::endl;
                }
            }
        } else {
            // Idle call: DUT latches the current register values
            latched_window_size = window_size;
            latched_trim_count = trim_count;
            latched_filter_mode = filter_mode;
            latched_rail_low = rail_low;
            latched_rail_high = rail_high;
            params_latched_once = true;
        }

        outfile << "Cycle: " << cycle
                << ", Input: " << (new_data ? (int)value : -1)
                << ", Raw: " << raw_output
                << ", Filtered: " << filtered_output
                << ", DZstat: " << dz_status_output
                << ", DZwin: " << dz_window_output
                << ", DZage: " << dz_age_output
                << ", DZlo: " << dz_low_count_output
                << ", DZhi: " << dz_high_count_output
                << std::endl;
    }

    // Scenario-level checks: the crossing segment must have touched BOTH
    // rails; the turning-point segment must have touched only the high rail.
    // (Counters are cumulative; the turning-point segment adds only high hits.)
    if (dz_low_count_output == 0 || dz_high_count_output == 0) {
        error_count++;
        outfile << "FAIL: expected both rails to be hit over the test "
                << "(low=" << dz_low_count_output << ", high=" << dz_high_count_output << ")" << std::endl;
    }

    outfile << "Checked " << sample_count << " samples, " << error_count << " errors." << std::endl;
    outfile.close();

    std::cout << "Checked " << sample_count << " samples, " << error_count << " errors." << std::endl;
    if (error_count != 0) {
        std::cout << "TEST FAILED" << std::endl;
        return 1;
    }
    std::cout << "TEST PASSED" << std::endl;
    return 0;
}