"""Firmware wrapLocal and PC STATE angle must share the same ADC circle.

The chip wraps the raw ADC into ±ANGLE_360/2 *before* the state packet.
The PC then applies ANGLE_DEVIATION (which also uses ANGLE_360/2) and 2π/ANGLE_360.
If the two circles differ, upright is a constant offset, not a scale error.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
GLOBALS = (REPO / "Driver" / "globals.py").read_text()
PARAMS = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "parameters.c").read_text()

# Live two-pose fit 2026-09-03 (new analog chain, FPGA median-63): 360 = 2*|upright−hanging|.
UPRIGHT_ADC = 1238.988
WORKING_HANG = 3273.353
WORKING_CIRCLE = 4068.73
BAD_CIRCLE = 4143.32
OLD_CIRCLE = 4049.44


def wrap_local(angle: float, circle: float) -> float:
    if angle > circle / 2:
        return angle - circle
    if angle <= -circle / 2:
        return angle + circle
    return angle


def deviation(hanging: float, circle: float) -> float:
    if hanging < circle / 2:
        return -hanging - circle / 2
    return -hanging + circle / 2


def wrap_rad(x: float) -> float:
    while x > math.pi:
        x -= 2 * math.pi
    while x <= -math.pi:
        x += 2 * math.pi
    return x


def state_angle(adc: float, hang: float, circle_fw: float, circle_pc: float | None = None) -> float:
    circle_pc = circle_fw if circle_pc is None else circle_pc
    sent = wrap_local(adc, circle_fw)
    return wrap_rad((sent + deviation(hang, circle_pc)) * (2 * math.pi / circle_pc))


def _zybo_float(text: str, name: str) -> float:
    # ZYBO block is the last assignment of these names in each file.
    env_matches = re.findall(
        rf"{name}\s*=\s*float\(os\.environ\.get\([^,]+,\s*[\"']([0-9.]+)[\"']\)\)",
        text,
    )
    if env_matches:
        return float(env_matches[-1])
    matches = re.findall(rf"{name}\s*=\s*([0-9.]+)", text)
    assert matches, name
    return float(matches[-1])


def test_globals_and_firmware_share_working_circle():
    assert _zybo_float(GLOBALS, "ANGLE_360_DEG_IN_ADC_UNITS") == WORKING_CIRCLE
    assert _zybo_float(PARAMS, "ANGLE_360_DEG_IN_ADC_UNITS") == WORKING_CIRCLE
    assert _zybo_float(GLOBALS, "ANGLE_HANGING_POLOLU") == WORKING_HANG
    assert _zybo_float(PARAMS, "ANGLE_HANGING_POLOLU") == WORKING_HANG


def test_working_pair_puts_live_upright_at_zero():
    assert state_angle(UPRIGHT_ADC, WORKING_HANG, WORKING_CIRCLE) == pytest.approx(0.0, abs=0.001)
    hang = state_angle(WORKING_HANG, WORKING_HANG, WORKING_CIRCLE)
    assert abs(abs(hang) - math.pi) < 0.001


def test_stretched_circle_is_the_minus_0_145_offset():
    # Historical Aug-31 bug: firmware wrap 4143, PC still 1020/4049.
    old_upright = 3038.9200
    got = state_angle(old_upright, 1020.0, BAD_CIRCLE, OLD_CIRCLE)
    assert got == pytest.approx(-0.145, abs=0.01)
    # Both sides on 4143 still miss zero: true upright then was ~3047, not 3088.5.
    assert abs(state_angle(old_upright, 1016.84, BAD_CIRCLE)) > 0.05
