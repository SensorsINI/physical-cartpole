"""Slider/button own target_position and target_equilibrium when
USE_EXTERNAL_INTERFACE is on.

Firmware wrapLocal/STATE already carry the chip targets. The PC must not send
CMD_SET_TARGET_POSITION or CMD_SET_TARGET_EQUILIBRIUM, and the chip must ignore
those commands so a running GUI cannot overwrite the pot or button.
"""
from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BRIDGE = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "hardware_bridge.h").read_text()
CONTROL = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "control.c").read_text()
GLOBALS = (REPO / "Driver" / "globals.py").read_text()
DRIVER = (REPO / "Driver" / "DriverFunctions" / "PhysicalCartPoleDriver.py").read_text()


def _firmware_slider_on() -> bool:
    return bool(re.search(r"(?m)^#define\s+USE_EXTERNAL_INTERFACE\b", BRIDGE))


def _globals_slider_on() -> bool:
    match = re.search(r"(?m)^USE_EXTERNAL_INTERFACE\s*=\s*(True|False)\b", GLOBALS)
    assert match, "USE_EXTERNAL_INTERFACE missing from Driver/globals.py"
    return match.group(1) == "True"


def test_pc_flag_matches_firmware_define():
    assert _firmware_slider_on() == _globals_slider_on()


def test_chip_ignores_pc_target_when_slider_on():
    if not _firmware_slider_on():
        return
    set_case = CONTROL.split("case CMD_SET_TARGET_POSITION:", 1)[1]
    set_case = set_case.split("case ", 1)[0]
    assert "USE_EXTERNAL_INTERFACE" in set_case
    assert "ignore" in set_case.lower() or "JB slider owns" in set_case


def test_chip_ignores_pc_equilibrium_when_button_on():
    if not _firmware_slider_on():
        return
    set_case = CONTROL.split("case CMD_SET_TARGET_EQUILIBRIUM:", 1)[1]
    set_case = set_case.split("case ", 1)[0]
    assert "USE_EXTERNAL_INTERFACE" in set_case
    assert "ignore" in set_case.lower() or "JB button owns" in set_case


def test_slider_applied_before_state_packet():
    if not _firmware_slider_on():
        return
    apply = CONTROL.find("target_position = get_normed_slider_state()")
    state = CONTROL.find("prepare_message_to_PC_state(")
    assert apply != -1 and state != -1
    assert apply < state


def test_button_applied_before_state_packet():
    if not _firmware_slider_on():
        return
    apply = CONTROL.find("get_target_equilibrium_from_external_button()")
    state = CONTROL.find("prepare_message_to_PC_state(")
    assert apply != -1 and state != -1
    assert apply < state


def test_slider_target_is_capped_at_fourteen_cm():
    params = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "parameters.c").read_text()
    assert re.search(r"SliderTargetHalfLength\s*=\s*0\.14", params)
    assert "get_normed_slider_state() * SliderTargetHalfLength" in CONTROL
    assert "get_normed_slider_state() * TrackHalfLength" not in CONTROL


def test_driver_adopts_chip_target_when_slider_on():
    if not _globals_slider_on():
        return
    fn = DRIVER.split("def set_target_position(self):", 1)[1]
    branch = fn.split("if USE_EXTERNAL_INTERFACE:", 1)[1]
    branch = branch.split("if self.epm", 1)[0]
    assert "apply_target_position_from_chip" in branch
    assert "apply_target_equilibrium_from_chip" in branch
    assert "InterfaceInstance.set_target_position" not in branch
    assert "InterfaceInstance.set_target_equilibrium" not in branch
