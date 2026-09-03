"""CartPoleFirmware slider affine map stays in sync with tools/slider_pmod."""
from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "slider_pmod"))

from slider_curve import (  # noqa: E402
    ADC_FULL_SCALE,
    SLIDER_ADC_LEFT,
    SLIDER_ADC_RIGHT,
    SLIDER_TARGET_HALF_LENGTH,
    adc_from_normed,
    adc_to_from_left,
    from_left_to_normed,
    normed_from_adc,
    normed_to_from_left,
)

C_PATH = REPO_ROOT / "Firmware" / "Src" / "Zynq" / "external_interface.c"


def _firmware_c_float(name: str) -> float:
    text = C_PATH.read_text()
    m = re.search(rf"#define {name}\s+([0-9.]+)f", text)
    assert m, f"{name} not found in {C_PATH}"
    return float(m.group(1))


def test_c_rails_match_python():
    assert _firmware_c_float("SLIDER_ADC_LEFT") == pytest.approx(SLIDER_ADC_LEFT, abs=1e-4)
    assert _firmware_c_float("SLIDER_ADC_RIGHT") == pytest.approx(
        SLIDER_ADC_RIGHT, abs=1e-4
    )


def test_slider_target_half_length_is_twelve_cm():
    text = (REPO_ROOT / "Firmware" / "Src" / "CartPoleFirmware" / "parameters.c").read_text()
    m = re.search(r"SliderTargetHalfLength\s*=\s*([0-9.]+)", text)
    assert m and float(m.group(1)) == pytest.approx(0.12)
    assert SLIDER_TARGET_HALF_LENGTH == pytest.approx(0.12)
    profiles = (REPO_ROOT / "Firmware" / "Src" / "CartPoleFirmware" / "controller_profiles.c").read_text()
    assert "SLIDER_TARGET_HALF_LENGTH_SHOW 0.12f" in profiles
    assert "0.1125" not in profiles
    assert "0.14f" not in profiles


def test_map_is_single_affine():
    text = C_PATH.read_text()
    assert "2.0f * from_left - 1.0f" in text
    assert "SLIDER_CENTER_FROM_LEFT" not in text
    assert "slider_adc_lut" not in text
    assert "SLIDER_HALF_N" not in text


def test_measured_rails_saturate_12bit():
    assert SLIDER_ADC_LEFT == pytest.approx(0.0)
    assert SLIDER_ADC_RIGHT == pytest.approx(ADC_FULL_SCALE)


def test_rails_and_electrical_mid():
    assert normed_from_adc(SLIDER_ADC_LEFT) == pytest.approx(-1.0)
    assert normed_from_adc(SLIDER_ADC_RIGHT) == pytest.approx(1.0)
    mid_adc = 0.5 * (SLIDER_ADC_LEFT + SLIDER_ADC_RIGHT)
    assert adc_to_from_left(mid_adc) == pytest.approx(0.5)
    assert normed_from_adc(mid_adc) == pytest.approx(0.0, abs=1e-6)


def test_affine_between_rails():
    lo, hi = SLIDER_ADC_LEFT, SLIDER_ADC_RIGHT
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        adc = lo + frac * (hi - lo)
        assert adc_to_from_left(adc) == pytest.approx(frac)
        assert normed_from_adc(adc) == pytest.approx(2.0 * frac - 1.0)


def test_normed_roundtrip():
    for n in (-1.0, -0.5, 0.0, 0.5, 1.0):
        assert from_left_to_normed(normed_to_from_left(n)) == pytest.approx(n, abs=1e-6)
        assert normed_from_adc(adc_from_normed(n)) == pytest.approx(n, abs=1e-6)
