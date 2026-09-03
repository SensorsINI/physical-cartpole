"""Zybo SW0–SW3 one-hot on-chip controller mux."""
from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CONTROL = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "control.c").read_text()
PROFILES = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "controller_profiles.c").read_text()
GLOBALS = (REPO / "Driver" / "globals.py").read_text()
DRIVER = (REPO / "Driver" / "DriverFunctions" / "PhysicalCartPoleDriver.py").read_text()
PROGRAM_TCL = (REPO / "Firmware" / "Scripts" / "program_rpgd_amp_production.tcl").read_text()
SLIDER_SH = (REPO / "tools" / "slider_pmod" / "program_cartpole_slider.sh").read_text()
RPGD = (REPO / "Firmware" / "Src" / "General" / "rpgd_controller.c").read_text()
IMITATOR = (REPO / "Firmware" / "Src" / "Zynq" / "neural_imitator.c").read_text()
POSTPROCESS = (
    REPO / "Firmware" / "Src" / "CartPoleFirmware" / "control_signal_postprocessing.c"
).read_text()
MOTOR_ZYNQ = (REPO / "Firmware" / "Src" / "Zynq" / "motor_zynq.c").read_text()


def test_show_mux_flag_on():
    match = re.search(r"(?m)^SHOW_SWITCH_MUX\s*=\s*(True|False)\b", GLOBALS)
    assert match and match.group(1) == "True"


def test_boot_idle_before_rpgd_dual_core():
    boot = CONTROL.split("unsigned short current_controller", 1)[0]
    zybo = boot.split("defined(ZYNQ) && defined(ZYBO_Z720)", 1)[1]
    zybo = zybo.split("#elif", 1)[0]
    assert "OnChipController_NONE" in zybo
    rpgd = boot.split("defined(ZYNQ) && defined(RPGD_DUAL_CORE)", 1)[1]
    rpgd = rpgd.split("#elif", 1)[0]
    assert "OnChipController_RPGD" in rpgd
    assert boot.find("ZYBO_Z720") < boot.find("RPGD_DUAL_CORE")


def test_all_four_switches_are_inputs():
    buttons = (
        REPO / "Firmware" / "Src" / "Zynq" / "buttons_and_switches.c"
    ).read_text()
    assert "XGpio_SetDataDirection(&Gpio, 1, 0xF)" in buttons
    assert "XGpio_SetDataDirection(&Gpio, 1, 1)" not in buttons


def test_switch_decode_is_one_hot():
    header = (
        REPO / "Firmware" / "Src" / "CartPoleFirmware" / "controller_profiles.h"
    ).read_text()
    ids = (
        REPO / "Firmware" / "Src" / "CartPoleFirmware" / "onchip_controllers.h"
    ).read_text()
    assert "SHOW_SWITCH_MUX_MASK 0x0Fu" in header
    assert "SHOW_MUX_IDLE" in ids
    assert "return OnChipController_RPGD" in PROFILES
    assert "return OnChipController_neural_controller_C" in PROFILES
    assert "return OnChipController_neural_controller_LSTM_C" in PROFILES
    assert "return OnChipController_NeuralImitator" in PROFILES
    assert "bits & (bits - 1u)" in PROFILES
    assert "show_mux_debounced_controller" in PROFILES


def test_mux_lives_in_background_task_not_isr():
    bg = CONTROL.split("void CONTROL_BackgroundTask(void)", 1)[1]
    loop = CONTROL.split("void CONTROL_Loop(void)", 1)[1].split(
        "void CONTROL_BackgroundTask(void)", 1
    )[0]
    assert "show_mux_debounced_controller" in bg
    assert "show_mux_debounced_controller" not in loop
    assert "apply_show_profile_for_controller" in bg
    assert "Q = 0.0" in bg
    assert "Motor_Stop" in bg
    assert "run_onchip_controller = 0" in bg


def test_switch_change_skips_eval_this_tick():
    bg = CONTROL.split("void CONTROL_BackgroundTask(void)", 1)[1]
    change = bg.split("selected != current_controller", 1)[1]
    change = change.split("if (run_onchip_controller)", 1)[0]
    assert "apply_show_profile_for_controller(selected)" in change
    assert "run_onchip_controller = 0" in change


def test_u_applies_profile_from_switches():
    fn = CONTROL.split("void cmd_ControlMode(bool en)", 1)[1]
    fn = fn.split("void cmd_PCControlMode", 1)[0]
    assert "show_mux_controller_from_switches" in fn
    assert "apply_show_profile_for_controller" in fn
    assert "enable_irq();" in fn


def test_k_parks_amp_worker():
    fn = CONTROL.split("void cmd_PCControlMode(bool en)", 1)[1]
    fn = fn.split("void cmd_SetControlConfig", 1)[0]
    assert "CB_Reset(&g_cb)" in fn
    assert "Motor_DisableOutput" in fn


def test_pc_cannot_overwrite_period_or_n_when_mux_on():
    fn = CONTROL.split("void cmd_SetControlConfig", 1)[1]
    fn = fn.split("void cmd_SetSeclocConfig", 1)[0]
    assert "if (!show_switch_mux_enabled())" in fn
    assert fn.count("if (!show_switch_mux_enabled())") >= 2


def test_uart_poll_uses_slowest_show_period():
    assert "0.020 if SHOW_SWITCH_MUX" in DRIVER


def test_k_does_not_restore_1khz_when_mux_on():
    fn = DRIVER.split("def hardware_controller_on_off(self):", 1)[1]
    fn = fn.split("def run_hardware_experiment", 1)[0]
    assert "not SHOW_SWITCH_MUX" in fn
    assert "ON_CHIP_NEURAL_POLLING_PERIOD_MS" in fn


def test_amp_programmer_loads_short_pole_bit():
    assert "cartpole_short_pole_secloc.bit" in PROGRAM_TCL
    assert "program_rpgd_amp_production.sh" in SLIDER_SH


def test_rpgd_does_not_own_timing():
    body = RPGD.split("int rpgd_controller_owns_timing(void)", 1)[1]
    body = body.split("int rpgd_controller_take_deadline_grace", 1)[0]
    assert "return 0;" in body
    init = RPGD.split("static void RPGD_Init(void)", 1)[1].split(
        "static void RPGD_Release(void)", 1
    )[0]
    assert "RPGD_CONTROL_PERIOD_MS" in init


def test_sw3_is_not_a_pl_network_switch():
    assert "Switch_GetState" not in IMITATOR
    assert "NeuralNetworkType active_network = NETWORK_HLS4ML;" in IMITATOR


def test_motor_stops_before_hard_limit_and_on_stall():
    init = CONTROL.split("void CONTROL_Init(void)", 1)[1].split(
        "void CONTROL_BackgroundTask(void)", 1
    )[0]
    assert "POSITION_ENCODER_RANGE / 2.0f" in init
    assert "positionCentre - halfTrack" in init
    assert "positionCentre + halfTrack" in init
    assert "POSITION_LIMIT_BRAKE_MARGIN_COUNTS 300" in POSTPROCESS
    assert "MOTOR_STALL_TIMEOUT_MS 100u" in POSTPROCESS
    assert "motor_stall_safety_check" in CONTROL
    assert "Motor_Stop();" in CONTROL


def test_control_off_latches_motor_output_disabled():
    chip_mode = CONTROL.split("void cmd_ControlMode(bool en)", 1)[1].split(
        "void cmd_PCControlMode(bool en)", 1
    )[0]
    pc_mode = CONTROL.split("void cmd_PCControlMode(bool en)", 1)[1].split(
        "void cmd_SetControlConfig", 1
    )[0]
    set_motor = CONTROL.split("case CMD_SET_MOTOR:", 1)[1].split(
        "case CMD_SET_TARGET_POSITION:", 1
    )[0]
    assert chip_mode.count("Motor_DisableOutput();") >= 2
    assert "Motor_EnableOutput();" in chip_mode
    assert pc_mode.count("Motor_DisableOutput();") >= 2
    assert "Motor_EnableOutput();" in pc_mode
    assert "if (!PCControl_Enabled)" in set_motor
    assert "static volatile int Motor_OutputEnabled" in MOTOR_ZYNQ
    set_power = MOTOR_ZYNQ.split("void Motor_SetPower", 1)[1]
    assert "if (!Motor_OutputEnabled)" in set_power
    assert "Motor_Stop();" in set_power
