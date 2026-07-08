#include "median_functions.h"

// Sorted window (ascending) and age-ordered circular buffer of the same samples.
static unsigned short window[MAX_WINDOW_SIZE];
static unsigned short buffer[MAX_WINDOW_SIZE];
static short currentWindowSize = 0;
static short bufferIndex = 0;

static short clamp_window_size(short window_size) {
    if (window_size < 1) {
        return 1;
    }
    if (window_size > MAX_WINDOW_SIZE) {
        return MAX_WINDOW_SIZE;
    }
    return window_size;
}

// Single-pass update of the sorted window: removes one instance of the oldest
// sample, merges in the new sample, and accumulates the trimmed sum and median
// picks on the fly. One loop at II=1 (~MAX_WINDOW_SIZE cycles) instead of the
// previous three sequential passes (~3x MAX_WINDOW_SIZE cycles), so the block
// sustains the XADC rate of 450 kS/s (2.22 us/sample) without unrolling.
unsigned short get_filtered_value(unsigned short newValue, short window_size, short trim_count, short filter_mode,
                                  unsigned short rail_low, unsigned short rail_high,
                                  unsigned short *rail_window_count) {

    // Safe: within one pass, reads always target original entries at indices
    // strictly above every index written so far (writes go to i, reads to i+1).
    #pragma HLS DEPENDENCE variable=window inter false

    static short lastWindowSize = 0;

    window_size = clamp_window_size(window_size);

    // Any window size change invalidates the circular age tracking in buffer[]
    // (bufferIndex was cycling modulo the old size), so restart from an empty
    // window. Growing without a reset would strand stale samples forever.
    if (window_size != lastWindowSize) {
        currentWindowSize = 0;
        bufferIndex = 0;
        lastWindowSize = window_size;
    }

    bool removal = (currentWindowSize == window_size);
    unsigned short oldValue = buffer[bufferIndex];
    buffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1 >= window_size) ? 0 : (short)(bufferIndex + 1);

    short n_old = currentWindowSize;                       // valid entries before this sample
    short n_out = removal ? n_old : (short)(n_old + 1);    // valid entries after this sample
    currentWindowSize = n_out;

    if (trim_count < 0) {
        trim_count = 0;
    }
    if (2 * trim_count >= n_out) {
        trim_count = (n_out - 1) / 2;
    }
    short first_index = trim_count;
    short last_index = n_out - trim_count;
    short kept_count = last_index - first_index;           // >= 1 after clamping
    short mid_lo = (n_out - 1) / 2;
    short mid_hi = n_out / 2;

    unsigned int sum = 0;                                  // max 64 * 65535 < 2^32
    unsigned short med_lo = newValue;
    unsigned short med_hi = newValue;
    unsigned short rail_hits = 0;                          // near-rail samples in the window

    bool skip_done = false;   // one instance of oldValue has been dropped
    bool inserted = false;    // newValue has been placed

    // a, b, c hold the ORIGINAL window values at indices i-1, i, i+1. They are
    // prefetched two iterations ahead of the write to window[i], which keeps the
    // BRAM read out of the one-cycle decision recurrence (timing) and guarantees
    // writes never clobber a value that is still needed (correctness).
    unsigned short a = 0;
    unsigned short b = window[0];
    unsigned short c = window[1];

    for (short i = 0; i < MAX_WINDOW_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        short prefetch_idx = (i + 2 < MAX_WINDOW_SIZE) ? (short)(i + 2) : (short)(MAX_WINDOW_SIZE - 1);
        unsigned short d = window[prefetch_idx];

        if (i < n_out) {
            // The output position maps to original index i + skip_done - inserted,
            // always within {i-1, i, i+1}, i.e. the a/b/c pipeline registers.
            // All comparisons below depend only on i and loop-invariant values,
            // so they evaluate in parallel; the (skip_done, inserted) state bits
            // only steer small muxes at the end. Keeping the adds/compares out of
            // that one-cycle recurrence is what lets the loop meet 10 ns at II=1.
            bool valid_m1 = ((short)(i - 1) < n_old);
            bool valid_0 = (i < n_old);
            bool valid_p1 = ((short)(i + 1) < n_old);
            bool eq_a = (a == oldValue);
            bool eq_b = (b == oldValue);
            bool le_a = (newValue <= a);
            bool le_b = (newValue <= b);
            bool le_c = (newValue <= c);

            unsigned short out;
            if (!inserted && !skip_done) {
                // Drop exactly the first occurrence of the outgoing sample.
                bool skip_now = removal && valid_0 && eq_b;
                bool cand_valid = skip_now ? valid_p1 : valid_0;
                bool take_new = !cand_valid || (skip_now ? le_c : le_b);
                out = take_new ? newValue : (skip_now ? c : b);
                inserted = take_new;
                skip_done = skip_now && !take_new;  // old sample only consumed if its slot was output
            } else if (inserted && !skip_done) {
                bool skip_now = removal && valid_m1 && eq_a;
                out = skip_now ? b : a;
                skip_done = skip_now;
            } else if (!inserted) {  // skip_done, not yet inserted
                bool take_new = !valid_p1 || le_c;
                out = take_new ? newValue : c;
                inserted = take_new;
            } else {                 // both done: plain copy
                out = b;
            }

            window[i] = out;
            if (i >= first_index && i < last_index) {
                sum += out;
            }
            if (i == mid_lo) {
                med_lo = out;
            }
            if (i == mid_hi) {
                med_hi = out;
            }
            // Dead-zone contamination: count window samples near either ADC
            // rail. Two compares against loop-invariant thresholds, off the
            // decision recurrence, so no timing impact.
            if (out <= rail_low || out >= rail_high) {
                rail_hits++;
            }
        }

        a = b;
        b = c;
        c = d;
    }

    *rail_window_count = rail_hits;

    if (filter_mode == FILTER_MODE_RAW) {
        return newValue;
    }
    if (filter_mode == FILTER_MODE_MEDIAN) {
        return (n_out % 2 != 0) ? med_lo : (unsigned short)((med_lo + med_hi) / 2);
    }

    // Rounded division sum / kept_count as bit-serial restoring division:
    // a combinational 22/6-bit divider was the critical path (~14.7 ns);
    // this loop is a shift-compare-subtract per cycle and meets 10 ns.
    // Numerator fits in 23 bits (max 64 * 65535 + 32 < 2^23).
    unsigned int numerator = sum + (unsigned int)(kept_count / 2);
    unsigned int quotient = 0;
    unsigned int remainder = 0;
    for (short bit = 22; bit >= 0; bit--) {
        #pragma HLS PIPELINE II=1
        remainder = (remainder << 1) | ((numerator >> bit) & 1u);
        if (remainder >= (unsigned int)kept_count) {
            remainder -= (unsigned int)kept_count;
            quotient |= (1u << bit);
        }
    }
    return (unsigned short)quotient;
}
