import ctypes
import subprocess
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_GENERAL = REPO_ROOT / "Firmware" / "Src" / "General"


class PythonPidReference:
    def __init__(self):
        self.time_last = None
        self.position_error_previous = None
        self.angle_error_previous = None
        self.position_error_integral = 0.0
        self.angle_error_integral = 0.0

    def step(self, angle, angle_d, position, position_d, target_position, time):
        del angle_d, position_d

        if self.time_last is None:
            time_difference = 0.0
        else:
            time_difference = time - self.time_last

        if time_difference > 0.1:
            time_difference = 0.0

        self.time_last = time

        q_position = self._pid_core(
            error=position - target_position,
            error_previous_name="position_error_previous",
            error_integral_name="position_error_integral",
            time_difference=time_difference,
            kp=22.0,
            ki=1.0,
            kd=12.0,
            sensitivity_p=1.0,
            sensitivity_i=1.0,
            sensitivity_d=0.01,
            integral_clip=1.0,
        )
        q_angle = self._pid_core(
            error=angle,
            error_previous_name="angle_error_previous",
            error_integral_name="angle_error_integral",
            time_difference=time_difference,
            kp=-18.0,
            ki=-38.0,
            kd=-4.0,
            sensitivity_p=1.0,
            sensitivity_i=1.0,
            sensitivity_d=0.01,
            integral_clip=1.0 / 38.0,
        )

        return q_angle + q_position

    def _pid_core(
        self,
        *,
        error,
        error_previous_name,
        error_integral_name,
        time_difference,
        kp,
        ki,
        kd,
        sensitivity_p,
        sensitivity_i,
        sensitivity_d,
        integral_clip,
    ):
        error_previous = getattr(self, error_previous_name)
        if time_difference > 0.0001 and error_previous is not None:
            error_diff = (error - error_previous) / time_difference
        else:
            error_diff = 0.0

        setattr(self, error_previous_name, error)

        error_integral = getattr(self, error_integral_name)
        if ki != 0.0:
            error_integral += error * time_difference
            error_integral = np.clip(error_integral, -abs(integral_clip), abs(integral_clip))
        else:
            error_integral = 0.0

        setattr(self, error_integral_name, error_integral)

        return (
            kp * error * sensitivity_p
            + ki * error_integral * sensitivity_i
            + kd * error_diff * sensitivity_d
        )


def build_hardware_pid(tmp_path):
    (tmp_path / "hardware_bridge.h").write_text(
        """
#ifndef HARDWARE_BRIDGE_H
#define HARDWARE_BRIDGE_H

#include <stdbool.h>

void enable_irq(void);
void disable_irq(void);
void Message_SendToPC(const unsigned char* data, unsigned int length);

#endif
""",
        encoding="utf-8",
    )
    (tmp_path / "communication_with_PC_general.h").write_text(
        """
#ifndef COMMUNICATION_WITH_PC_GENERAL_H
#define COMMUNICATION_WITH_PC_GENERAL_H

#include <stdbool.h>

unsigned char crc(const unsigned char * message, unsigned int len);
bool crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal);
void prepare_message_to_PC_config_PID(
    unsigned char * txBuffer,
    float position_KP,
    float position_KI,
    float position_KD,
    float angle_KP,
    float angle_KI,
    float angle_KD
);

#endif
""",
        encoding="utf-8",
    )
    wrapper = tmp_path / "hardware_pid_wrapper.c"
    wrapper.write_text(
        """
#include <stdbool.h>
#include "hardware_pid.h"

void enable_irq(void) {}
void disable_irq(void) {}
void Message_SendToPC(const unsigned char* data, unsigned int length)
{
    (void)data;
    (void)length;
}
unsigned char crc(const unsigned char * message, unsigned int len)
{
    (void)message;
    (void)len;
    return 0;
}
bool crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal)
{
    (void)buff;
    (void)len;
    (void)crcVal;
    return true;
}
void prepare_message_to_PC_config_PID(
    unsigned char * txBuffer,
    float position_KP,
    float position_KI,
    float position_KD,
    float angle_KP,
    float angle_KI,
    float angle_KD
)
{
    (void)txBuffer;
    (void)position_KP;
    (void)position_KI;
    (void)position_KD;
    (void)angle_KP;
    (void)angle_KI;
    (void)angle_KD;
}

void controller_init(void)
{
    PID_Ops.init();
}

float controller_step(
    float angle,
    float angleD,
    float position,
    float positionD,
    float target_position,
    float time
)
{
    float inputs[6] = {angle, angleD, position, positionD, target_position, time};
    float outputs[1] = {0.0f};
    PID_Ops.evaluate(inputs, outputs);
    return outputs[0];
}
""",
        encoding="utf-8",
    )

    library = tmp_path / "hardware_pid.so"
    subprocess.run(
        [
            "gcc",
            "-shared",
            "-fPIC",
            "-std=c99",
            "-o",
            str(library),
            str(wrapper),
            str(FIRMWARE_GENERAL / "hardware_pid.c"),
            "-I",
            str(tmp_path),
            "-I",
            str(FIRMWARE_GENERAL),
            "-lm",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    lib = ctypes.CDLL(str(library))
    lib.controller_init.argtypes = []
    lib.controller_init.restype = None
    lib.controller_step.argtypes = [ctypes.c_float] * 6
    lib.controller_step.restype = ctypes.c_float
    return lib


def test_hardware_pid_matches_python_pid_sequence(tmp_path):
    c_pid = build_hardware_pid(tmp_path)
    c_pid.controller_init()
    py_pid = PythonPidReference()

    samples = [
        (0.020, 0.30, 0.010, 0.20, 0.000, 0.000),
        (0.018, 0.25, 0.012, 0.18, 0.000, 0.005),
        (-0.015, -0.10, 0.008, -0.05, 0.000, 0.010),
        (0.005, 0.05, -0.006, -0.10, 0.020, 0.015),
        (0.004, 0.02, -0.005, -0.08, 0.020, 0.200),
        (-0.002, -0.01, 0.001, 0.04, -0.010, 0.205),
    ]

    for sample in samples:
        c_output = c_pid.controller_step(*sample)
        py_output = py_pid.step(*sample)
        np.testing.assert_allclose(c_output, py_output, rtol=2e-6, atol=2e-6)
