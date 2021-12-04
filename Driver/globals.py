import math
import logging

MOTOR = 'ORIGINAL'  # It will be overwritten by each calibration
MOTOR_DYNAMICS_CORRECTED = True

LOGGING_LEVEL = logging.INFO
PRINT_PERIOD_MS = 100  # shows state every this many ms

LIVE_PLOT = False

CALIBRATE = False  # If True calibration will be run at start-up of the program
# important to calibrate if running standalone to avoid motor burnout
# because limits are determined during this calibration

CONTROLLER_NAME = 'PD'
CONTROL_PERIOD_MS = 2  # It was 25 originally, we changed it to 5 - marcin & asude
PATH_TO_CONTROLLERS = './Controllers/'  # Path where controllers are stored

PATH_TO_EXPERIMENT_RECORDINGS = './ExperimentRecordings/'  # Path where the experiments data is stored

JSON_PATH = 'Json/'

MOTOR_FULL_SCALE = 8192  # 7199 # with pololu motor and scaling in firmware #7199 # with original motor
MOTOR_FULL_SCALE_SAFE = int(0.95 * MOTOR_FULL_SCALE)  # Including a safety constraint

# Angle unit conversion adc to radians: (ANGLE_TARGET + ANGLE DEVIATION - ANGLE_ADC_RANGE/2)/ANGLE_ADC_RANGE*math.pi
# ANGLE_KP_SOFTWARE = ANGLE_KP_FIRMWARE/ANGLE_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
ANGLE_AVG_LENGTH = 10  # adc routine in firmware reads ADC this many times quickly in succession to reduce noise
ANGLE_ADC_RANGE = 4096  # Range of angle values #
# ANGLE_HANGING = 1019  # right cartpole # Value from sensor when pendulum is at stable equilibrium point
ANGLE_HANGING = 1199 # left cartpole # Value from sensor when pendulum is at stable equilibrium point
#ANGLE_HANGING = 1024 # right cartpole # Value from sensor when pendulum is at stable equilibrium point

if ANGLE_HANGING < ANGLE_ADC_RANGE/2:
    ANGLE_DEVIATION = - ANGLE_HANGING - ANGLE_ADC_RANGE / 2 # moves upright to 0 and hanging to -pi
else:
    ANGLE_DEVIATION = - ANGLE_HANGING + ANGLE_ADC_RANGE / 2 # moves upright to 0 and hanging to pi

ANGLE_NORMALIZATION_FACTOR = 2 * math.pi / ANGLE_ADC_RANGE
ANGLE_DEVIATION_FINETUNE = 0.11999999999999998 # adjust from key commands such that upright angle error is minimized

# Position unit conversion adc to meters: POSITION_TARGET_SOFTWARE = POSITION_TARGET_FIRMWARE*POSITION_NORMALIZATION_FACTOR
# POSITION_KP_SOFTWARE = POSITION_KP_FIRMWARE/POSITION_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
POSITION_ENCODER_RANGE = 4660  # This is an empirical approximation # seems to be 4164 now
POSITION_OFFSET = 0  # Serves to adjust starting position - position after calibration is 0
POSITION_FULL_SCALE_N = int(POSITION_ENCODER_RANGE) / 2 # Corrected position full scale - cart position should range over +- this value if calibrated for zero at center
TRACK_LENGTH = 0.396  # Total usable track length in meters
POSITION_NORMALIZATION_FACTOR = TRACK_LENGTH/POSITION_ENCODER_RANGE # 0.000084978540773

POSITION_TARGET = 0.0  # meters

JOYSTICK_DEADZONE = 0.1  # deadzone around joystick neutral position that stick is ignored
JOYSTICK_POSITION_KP = 4.0

import platform
import subprocess
SERIAL_PORT = None
try:
  SERIAL_PORT = subprocess.check_output('ls -a /dev/tty.usbserial*', shell=True).decode("utf-8").strip() if platform.system() == 'Darwin' else '/dev/ttyUSB1'
except Exception as err:
  print(err)

SERIAL_BAUD = 230400  # default 230400, in firmware. Alternatives if compiled and supported by USB serial intervace are are 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000

ratio = 1.05


def inc(param):
    if param < 2:
        param = round(param + 0.1, 1)
    else:
        old = param
        param = round(param * ratio)
        if param == old:
            param += 1
    return param


def dec(param):
    if param < 2:
        param = max(0, round(param - 0.1, 1))
    else:
        old = param
        param = round(param / ratio)
        if param == old:
            param -= 1
    return param
