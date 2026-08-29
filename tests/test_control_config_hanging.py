"""CMD_SET/GET_CONTROL_CONFIG packing must match firmware.

Firmware/Src/CartPoleFirmware/communication_with_PC.c:
  SET 16: period/sync/hanging/avg/motor/timesteps
  SET 17: plus force_angle_hanging
  GET 17: same fields as SET 16 plus hanging_set_on_chip, CRC last
"""
from __future__ import annotations

import struct
from pathlib import Path

CONTROL_C = Path(__file__).resolve().parents[1] / "Firmware/Src/CartPoleFirmware/control.c"


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


def test_get_unpack_hanging_on_chip_flag():
    payload = struct.pack("=H?fH?H?", 5, False, 1100.5, 1, True, 2, True)
    period, sync, hanging, avg, motor, steps, on_chip = struct.unpack("=H?fH?H?", payload)
    assert period == 5
    assert sync is False
    assert abs(hanging - 1100.5) < 1e-4
    assert on_chip is True
    assert steps == 2
    assert motor is True
    assert avg == 1


def test_btn0_hanging_capture_is_averaged_not_single_sample():
    src = CONTROL_C.read_text()
    assert "#define HANGING_CAPTURE_SAMPLES 50" in src
    assert "#define HANGING_CAPTURE_SKIP_TICKS 4" in src
    assert "hanging_capture_sum / (float)hanging_capture_count" in src
