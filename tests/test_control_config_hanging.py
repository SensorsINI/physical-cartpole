"""CMD_SET/GET_CONTROL_CONFIG packing must match firmware.

Firmware/Src/CartPoleFirmware/communication_with_PC.c:
  SET 16: period/sync/hanging/avg/motor/timesteps
  SET 17: plus force_angle_hanging
  GET 17: same fields as SET 16 plus hanging status, CRC last

Hanging status bit 0 is hanging_set_on_chip, bits 1..4 carry the calibration
revision also sent in STATE telemetry bits 4..7, bit 5 is chip/QSPI span
ownership, and bit 7 advertises support for querying the runtime angle span.
"""
from __future__ import annotations

import struct
from pathlib import Path

CONTROL_C = Path(__file__).resolve().parents[1] / "Firmware/Src/CartPoleFirmware/control.c"
DRIVER = (
    Path(__file__).resolve().parents[1]
    / "Driver/DriverFunctions/PhysicalCartPoleDriver.py"
)


def test_packed_control_config_sizes():
    assert struct.calcsize("=H?fH?H") == 12
    assert struct.calcsize("=H?fH?H?") == 13
    native_may_pad = struct.calcsize("H?fH?H")
    assert native_may_pad >= 12


def test_set_force_byte_is_thirteenth_payload_byte():
    payload = struct.pack("=H?fH?H", 5, True, 1063.779, 1, False, 2)
    forced = payload + struct.pack("=?", True)
    assert len(payload) == 12
    assert len(forced) == 13
    assert forced[12] == 1


def test_get_unpack_hanging_status():
    revision = 9
    wire_status = 0x80 | 0x20 | (revision << 1) | 1
    payload = struct.pack("=H?fH?HB", 5, False, 1100.5, 1, True, 2, wire_status)
    period, sync, hanging, avg, motor, steps, status = struct.unpack("=H?fH?HB", payload)
    on_chip = bool(status & 1)
    assert period == 5
    assert sync is False
    assert abs(hanging - 1100.5) < 1e-4
    assert on_chip is True
    assert (status >> 1) & 0x0F == revision
    assert status & 0x20
    assert status & 0x80
    assert steps == 2
    assert motor is True
    assert avg == 1


def test_btn0_hanging_capture_is_averaged_not_single_sample():
    src = CONTROL_C.read_text()
    assert "#define HANGING_CAPTURE_SAMPLES 50" in src
    assert "HANGING_CAPTURE_SKIP_TICKS" not in src
    assert "hanging_capture_sum / (float)hanging_capture_count" in src


def test_btn0_revision_notifies_connected_driver():
    firmware = CONTROL_C.read_text()
    driver = DRIVER.read_text()
    assert "AngleCalibrationRevision = (AngleCalibrationRevision + 1u) & 0x0Fu;" in firmware
    assert "((AngleCalibrationRevision & 0x0Fu) << 4)" in firmware
    assert "self._sync_angle_calibration_if_changed()" in driver
    assert "revision != self._angle_calibration_revision_seen" in driver


def test_btn0_immediately_disarms_all_control_before_capture():
    src = CONTROL_C.read_text()
    handler = src.split("void CONTROL_SetHangingFromCurrentReading(void)", 1)[1]
    handler = handler.split("#ifdef ZYNQ", 1)[0]
    assert "Motor_DisableOutput();" in handler
    assert "motor_command = 0;" in handler
    assert "ControlOnChip_Enabled = false;" in handler
    assert "PCControl_Enabled = false;" in handler
    assert handler.index("Motor_DisableOutput();") < handler.index(
        "set_hanging_requested = true;"
    )

    capture = src.split("static void hanging_capture_feed", 1)[1]
    capture = capture.split("static void upright_capture_abort", 1)[0]
    assert capture.count(
        "ControlOnChip_Enabled || PCControl_Enabled || calibrate"
    ) == 2


def test_boot_loads_paired_qspi_calibration_before_pc_config():
    src = CONTROL_C.read_text()
    init = src.split("void CONTROL_Init(void)", 1)[1].split(
        "void CONTROL_ToggleState(void)", 1
    )[0]
    assert "QspiNv_LoadCalibration(&stored_hanging, &stored_angle_360)" in init
    assert "ANGLE_HANGING = stored_hanging;" in init
    assert "ANGLE_360_DEG_IN_ADC_UNITS = stored_angle_360;" in init
    assert "ANGLE_NORMALIZATION_FACTOR = (2.0f * M_PI)" in init
    assert "AngleHangingSetOnChip = true;" in init
    assert "AngleSpanSetOnChip = true;" in init
    assert "PcHangingApplied" in src
    assert "chip/QSPI calibration" in src


def test_all_calibration_changes_queue_paired_qspi_save():
    src = CONTROL_C.read_text()
    btn0 = src.split("static void hanging_capture_feed", 1)[1].split(
        "static void upright_capture_abort", 1
    )[0]
    btn1 = src.split("static void upright_capture_feed", 1)[1].split(
        "int clip(", 1
    )[0]
    pc_config = src.rsplit("void cmd_SetControlConfig", 1)[1].split(
        "void cmd_SetSeclocConfig", 1
    )[0]
    assert "queue_angle_calibration_save();" in btn0
    assert "queue_angle_calibration_save();" in btn1
    assert "if (force_angle_hanging)" in pc_config
    assert "queue_angle_calibration_save();" in pc_config
    assert "QspiNv_SaveCalibration(angle_calibration_nv_hanging," in src
