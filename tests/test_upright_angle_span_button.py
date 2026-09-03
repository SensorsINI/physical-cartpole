"""BTN1 upright capture recalibrates the full ADC angle span safely."""
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
CONTROL = (REPO / "Firmware/Src/CartPoleFirmware/control.c").read_text()
CONTROL_H = (REPO / "Firmware/Src/CartPoleFirmware/control.h").read_text()
HARDWARE_BRIDGE = (
    REPO / "Firmware/Src/CartPoleFirmware/hardware_bridge.h"
).read_text()
MAIN = (REPO / "Firmware/Src/CartPoleFirmware/main.c").read_text()
PARAMETERS = (REPO / "Firmware/Src/CartPoleFirmware/parameters.c").read_text()
PARAMETERS_H = (REPO / "Firmware/Src/CartPoleFirmware/parameters.h").read_text()
LED = (REPO / "Firmware/Src/Zynq/led_zynq.c").read_text()
INTERFACE = (REPO / "Driver/DriverFunctions/interface.py").read_text()
DRIVER = (REPO / "Driver/DriverFunctions/PhysicalCartPoleDriver.py").read_text()


def full_circle_from_opposite_poses(hanging, upright):
    return 2.0 * abs(upright - hanging)


def test_known_upright_and_hanging_recover_working_circle():
    assert full_circle_from_opposite_poses(3273.353, 1238.988) == pytest.approx(
        4068.73
    )


def test_btn1_is_bound_to_upright_capture():
    assert "#define BUTTON_4" in HARDWARE_BRIDGE
    assert "PL_BTN_1  /* Zybo PL BTN1: upright/full-circle capture */" in HARDWARE_BRIDGE
    assert "CONTROL_SetUprightFromCurrentReading" in CONTROL_H
    assert "Button_SetAction(BUTTON_4, CONTROL_SetUprightFromCurrentReading);" in MAIN


def test_upright_capture_disarms_and_averages_before_applying():
    handler = CONTROL.split(
        "void CONTROL_SetUprightFromCurrentReading(void)", 1
    )[1].split("#ifdef ZYNQ", 1)[0]
    assert "Motor_DisableOutput();" in handler
    assert "motor_command = 0;" in handler
    assert "ControlOnChip_Enabled = false;" in handler
    assert "PCControl_Enabled = false;" in handler

    capture = CONTROL.split("static void upright_capture_feed", 1)[1].split(
        "int clip(", 1
    )[0]
    assert "upright_capture_sum / (float)upright_capture_count" in capture
    assert "2.0f * fabsf(upright - ANGLE_HANGING)" in capture
    assert "0.8f * ANGLE_360_DEG_IN_ADC_UNITS" in capture
    assert "1.2f * ANGLE_360_DEG_IN_ADC_UNITS" in capture
    assert "ANGLE_NORMALIZATION_FACTOR = (2.0f * M_PI)" in capture
    assert "AngleSpanSetOnChip = true;" in capture


def test_angle_circle_is_runtime_mutable():
    assert "extern float ANGLE_360_DEG_IN_ADC_UNITS;" in PARAMETERS_H
    assert "const float ANGLE_360_DEG_IN_ADC_UNITS" not in PARAMETERS


def test_upright_capture_has_distinct_rgb_feedback():
    assert "Led_RgbUprightCaptureStart" in LED
    assert "Led_RgbFeedback(1, 0)" in LED
    assert "Led_RgbUprightCaptureSuccess" in LED
    assert "Led_RgbFeedback(2, 700000UL)" in LED
    assert "Led_RgbUprightCaptureError" in LED
    assert "Led_RgbFeedback(4, 1000000UL)" in LED


def test_connected_driver_adopts_runtime_circle():
    assert "CMD_GET_ANGLE_CALIBRATION = 0xD5" in INTERFACE
    assert "def get_angle_calibration(self):" in INTERFACE
    assert "incoming_data_processor_module.ANGLE_360_DEG_IN_ADC_UNITS" in DRIVER
    assert "incoming_data_processor_module.ANGLE_NORMALIZATION_FACTOR" in DRIVER
