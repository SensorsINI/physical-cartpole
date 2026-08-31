"""Dead-zone-near-vertical warning geometry.

Must stay in sync with wrap_adc_circle / dead_zone_near_vertical in
Firmware/Src/CartPoleFirmware/control.c (Zybo Z7-20 constants from parameters.c).
"""
from __future__ import annotations

import math

# Firmware/Src/CartPoleFirmware/parameters.c, #else  // ZYBO_Z720
ANGLE_360_DEG_IN_ADC_UNITS = 4066.88
DEAD_ZONE_VERTICAL_WARN_DEG = 20.0
ANGLE_HANGING_POLOLU = 1014


def wrap_adc_circle(adc: float, circle: float = ANGLE_360_DEG_IN_ADC_UNITS) -> float:
    while adc < 0.0:
        adc += circle
    while adc >= circle:
        adc -= circle
    return adc


def adc_dist_to_deadzone(adc: float, circle: float = ANGLE_360_DEG_IN_ADC_UNITS) -> float:
    adc = wrap_adc_circle(adc, circle)
    return min(adc, circle - adc)


def dead_zone_near_vertical(
    hanging_adc: float,
    circle: float = ANGLE_360_DEG_IN_ADC_UNITS,
    warn_deg: float = DEAD_ZONE_VERTICAL_WARN_DEG,
) -> bool:
    thresh = circle * (warn_deg / 360.0)
    dist_down = adc_dist_to_deadzone(hanging_adc, circle)
    dist_up = adc_dist_to_deadzone(hanging_adc + circle * 0.5, circle)
    return (dist_down < thresh) or (dist_up < thresh)


def wrap_aware_mean(samples, circle: float = ANGLE_360_DEG_IN_ADC_UNITS) -> float:
    """Same circular mean as hanging_capture_feed (after skip / abort checks)."""
    wrapped = [wrap_adc_circle(float(s), circle) for s in samples]
    ref = wrapped[0]
    total = 0.0
    for sample in wrapped:
        d = sample - ref
        if d > circle * 0.5:
            d -= circle
        elif d < -circle * 0.5:
            d += circle
        total += d
    return wrap_adc_circle(ref + total / len(wrapped), circle)


def test_known_good_hanging_does_not_warn():
    assert dead_zone_near_vertical(ANGLE_HANGING_POLOLU) is False


def test_dead_zone_at_hanging_down_warns():
    assert dead_zone_near_vertical(0.0) is True
    assert dead_zone_near_vertical(150.0) is True


def test_dead_zone_at_upright_warns():
    # Hanging at half-circle puts ADC wrap (0) at upright.
    assert dead_zone_near_vertical(ANGLE_360_DEG_IN_ADC_UNITS * 0.5) is True
    assert dead_zone_near_vertical(2033.44) is True


def test_hanging_at_old_upright_is_still_horizontal_dead_zone():
    # 1063.779 + 180deg is the usual upright ADC, not a vertical dead zone.
    hanging_at_old_up = ANGLE_HANGING_POLOLU + ANGLE_360_DEG_IN_ADC_UNITS * 0.5
    assert dead_zone_near_vertical(hanging_at_old_up) is False


def test_threshold_is_twenty_degrees():
    thresh = ANGLE_360_DEG_IN_ADC_UNITS * (DEAD_ZONE_VERTICAL_WARN_DEG / 360.0)
    assert math.isclose(thresh, 4066.88 * 20.0 / 360.0)
    just_inside = thresh * 0.99
    just_outside = thresh * 1.01
    assert dead_zone_near_vertical(just_inside) is True
    assert dead_zone_near_vertical(just_outside) is False


def test_wrap_aware_mean_keeps_upper_half_adc():
    # Regression: wrapLocal then 4096-fold turned 3000 into ~3047.
    mean = wrap_aware_mean([3000, 3001, 2999, 3000])
    assert abs(mean - 3000.0) < 1.0


def test_wrap_aware_mean_across_electrical_wrap():
    # 4090 folds onto the ANGLE_360 circle (~40.6); mean stays near wrap, not mid-scale.
    mean = wrap_aware_mean([4090, 4091, 5, 10])
    assert mean < 80.0 or mean > ANGLE_360_DEG_IN_ADC_UNITS - 80.0
    assert abs(mean - (ANGLE_360_DEG_IN_ADC_UNITS / 2.0)) > 1000.0
