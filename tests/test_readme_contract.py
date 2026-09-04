"""README must stay aligned with the Zybo show firmware and driver."""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
README = (REPO / "README.md").read_text()
HARDWARE = (REPO / "Docs" / "hardware.md").read_text()
FIRMWARE = (REPO / "Docs" / "firmware-and-flash.md").read_text()
SLIDER = (REPO / "tools" / "slider_pmod" / "README.md").read_text()
CALIBRATION = (REPO / "Docs" / "calibration.md").read_text()
OPERATING = (REPO / "Docs" / "operating.md").read_text()


def test_show_mux_and_arming_are_documented():
    assert "SW0" in README and "AMP RPGD" in README
    assert "SW1" in README and "Dense-8" in README
    assert "SW2" in README and "LSTM" in README
    assert "SW3" in README and "Short-pole PL" in README
    assert "BTN4" in README
    assert "BTN0" in README
    assert "BTN5" in README
    assert "program_rpgd_amp_production.sh" in README
    assert "program_show_qspi.sh" in README
    assert "CartPoleFirmware_rpgd_amp_cpu0.elf" in README


def test_slider_half_length_is_twelve_cm():
    assert "0.12 m" in HARDWARE
    assert "±0.14" not in HARDWARE
    assert "0.14 m" not in HARDWARE
    assert "±0.12 m" in SLIDER
    assert "0.14 m" not in SLIDER


def test_setup_paths_and_tools_version():
    assert "2020.1" in README
    assert "Firmware/CubeIDE/CartPoleFirmware" in FIRMWARE
    assert "CartpoleDriverZynq_AXIS_12_09_2025.tcl" in FIRMWARE
    assert "cartpole_zybo_secloc2026" in FIRMWARE


def test_angle_pair_persistence_is_documented():
    for text in (README, CALIBRATION, OPERATING):
        assert "ANGLE_HANGING" in text
        assert "ANGLE_360_DEG_IN_ADC_UNITS" in text
        assert "QSPI" in text
        assert "startup" in text
    assert "complete pair" in README
    assert "session-only" not in CALIBRATION
    assert "QSPI is not loaded" not in OPERATING


def test_obsolete_factory_and_zynq_folder_docs_are_gone():
    for text in (README, HARDWARE, FIRMWARE):
        assert "pendulum.py" not in text
        assert "zynq/zybo.tcl" not in text
        assert "source ./zybo.tcl" not in text
        assert "zybo_vitis_pot_motor_test.zip" not in text
        assert "BELOW NOT FULLY UPDATED YET" not in text
        assert "petalinux" not in text.lower()
