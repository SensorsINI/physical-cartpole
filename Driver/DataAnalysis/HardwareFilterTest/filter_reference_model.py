"""Bit-exact Python reference model of the FPGA angle filter block.

Mirrors FPGA/CustomIPs/median_filter_hls/median_functions.cpp (same model as the
HLS testbench): sliding window that grows from empty to window_size, median with
integer averaging of the two middle elements for even counts, trimmed mean with
rounded integer division, trim clamping when 2*trim_count >= count.

Values are the full 16-bit filter-domain codes (12-bit ADC left-aligned, x16).

Note on replaying recorded data: the hardware consumes every XADC conversion
(~2.2 us apart) while recordings are subsampled (>=100 us apart), so replaying a
recording through this model gives statistically equivalent but not
sample-identical output compared to the hardware filtered stream.
"""

from collections import deque

import numpy as np

FILTER_MODE_RAW = 0
FILTER_MODE_MEDIAN = 1
FILTER_MODE_TRIMMED_MEAN = 2

MAX_WINDOW_SIZE = 64


def filter_stream(raw_values, window_size, trim_count, filter_mode):
    """Run the reference filter over a 1-D sequence of raw 16-bit codes.

    Returns a numpy int array of the same length (output for every input
    sample, starting from a freshly reset window, as the hardware does after a
    window-size change).
    """
    window_size = int(np.clip(window_size, 1, MAX_WINDOW_SIZE))
    window = deque(maxlen=window_size)
    out = np.empty(len(raw_values), dtype=np.int64)

    for i, value in enumerate(raw_values):
        value = int(value)
        window.append(value)

        if filter_mode == FILTER_MODE_RAW:
            out[i] = value
            continue

        ordered = sorted(window)
        n = len(ordered)

        if filter_mode == FILTER_MODE_MEDIAN:
            if n % 2 != 0:
                out[i] = ordered[(n - 1) // 2]
            else:
                out[i] = (ordered[n // 2 - 1] + ordered[n // 2]) // 2
            continue

        trim = max(0, int(trim_count))
        if 2 * trim >= n:
            trim = (n - 1) // 2
        kept = ordered[trim:n - trim]
        out[i] = (sum(kept) + len(kept) // 2) // len(kept)

    return out


DZ_AGE_MAX = 0xFFFF


def deadzone_stream(raw_values, window_size, rail_low, rail_high):
    """Reference model of the hardware dead-zone tracking.

    For each raw sample (16-bit filter-domain code) returns arrays:
      status      bit0 = at low rail (v <= rail_low), bit1 = at high rail (v >= rail_high)
      window      number of near-rail samples currently inside the filter window
      age         samples since last rail contact (saturates at 0xFFFF; starts saturated)
      low_count   cumulative low-rail hits
      high_count  cumulative high-rail hits
    """
    window_size = int(np.clip(window_size, 1, MAX_WINDOW_SIZE))
    window = deque(maxlen=window_size)
    n = len(raw_values)
    status = np.zeros(n, dtype=np.int64)
    win_count = np.zeros(n, dtype=np.int64)
    age = np.zeros(n, dtype=np.int64)
    low_count = np.zeros(n, dtype=np.int64)
    high_count = np.zeros(n, dtype=np.int64)

    cur_age = DZ_AGE_MAX
    lo_total = 0
    hi_total = 0
    for i, value in enumerate(raw_values):
        value = int(value)
        window.append(value)
        low = value <= rail_low
        high = value >= rail_high
        status[i] = (1 if low else 0) | (2 if high else 0)
        if low:
            lo_total += 1
        if high:
            hi_total += 1
        if low or high:
            cur_age = 0
        elif cur_age != DZ_AGE_MAX:
            cur_age += 1
        win_count[i] = sum(1 for v in window if v <= rail_low or v >= rail_high)
        age[i] = cur_age
        low_count[i] = lo_total
        high_count[i] = hi_total

    return status, win_count, age, low_count, high_count
