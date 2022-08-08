import math
import logging
import numpy as np

##### Logging and Recordings #####
LOGGING_LEVEL = logging.ERROR
PATH_TO_EXPERIMENT_RECORDINGS = './ExperimentRecordings/'  # Path where the experiments data is stored
PRINT_PERIOD_MS = 10  # shows state in terminal every this many control updates
PRINT_AVERAGING_LENGTH = 500

##### Timing #####
performance_measurement = np.zeros((15))
performance_measurement_buffer = np.zeros((performance_measurement.size, 0))

##### Live Plot (start with 6, save plot with 7 and reset with 8) #####
LIVE_PLOT = False
LIVE_PLOT_UNITS = 'metric'  # choose 'raw' or 'metric'
LIVE_PLOT_ZERO_ANGLE_DOWN = False
LIVE_PLOT_KEEPSAMPLES = 5000
LIVE_PLOT_TIMELINES = list(range(5))  # deactivate plots for performance, for all use list(range(5))
LIVE_PLOT_HISTOGRAMMS = list(range(5))  # deactivate plots for performance, for all use list(range(5))

##### Controller Settings #####
CONTROLLER_NAME = 'lqr'  # e.g. 'pid', 'mppi', 'do-mpc', 'do-mpc-discrete'
PREDICTOR = 'RNN'  # e.g. 'predictor_ODE_tf', 'predictor_autoregressive_tf'
if CONTROLLER_NAME == 'pid':
    CONTROL_PERIOD_MS = 5
elif CONTROLLER_NAME == 'neural-imitator-tf-CPS':
    CONTROL_PERIOD_MS = 7
else:
    CONTROL_PERIOD_MS = 20  # e.g. 5 for PID or 20 for mppi
CONTROL_SYNC = True  # Delays Input until next Timeslot for more accurate measurements
AUTOSTART = False  # Autostarts Zero-Controller for Performance Measurement
JSON_PATH = 'Json/'

##### Motor Settings #####
MOTOR = 'POLOLU'  # choose 'POLOLU' or 'ORIGINAL'
MOTOR_DYNAMICS_CORRECTED = False if CONTROLLER_NAME == 'PID' else True  # Linearize and Threshold Motor Commands

MOTOR_FULL_SCALE = 8192  # 7199 # with pololu motor and scaling in firmware #7199 # with original motor
MOTOR_FULL_SCALE_SAFE = int(0.95 * MOTOR_FULL_SCALE)  # Including a safety constraint

##### Angle Conversion #####
# Angle unit conversion adc to radians: (ANGLE_TARGET + ANGLE DEVIATION - ANGLE_ADC_RANGE/2)/ANGLE_ADC_RANGE*math.pi
# ANGLE_KP_SOFTWARE = ANGLE_KP_FIRMWARE/ANGLE_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
ANGLE_AVG_LENGTH = 32  # adc routine in firmware reads ADC this many times quickly in succession to reduce noise
ANGLE_ADC_RANGE = 4096  # Range of angle values #

ANGLE_HANGING_POLOLU = 1158  # 1213     # Value from sensor when pendulum is at stable equilibrium point
ANGLE_HANGING_ORIGINAL = 1034  # Value from sensor when pendulum is at stable equilibrium point

ANGLE_HANGING_DEFAULT = True  # If True default ANGLE_HANGING is loaded for a respective cartpole when motor is detected at calibration
#  This variable changes to false after b is pressed - you can first measure angle hanging and than calibrate without overwritting
# At the beginning always default angle hanging for default motor specified in globals is loaded

ANGLE_NORMALIZATION_FACTOR = 2 * math.pi / ANGLE_ADC_RANGE
ANGLE_DEVIATION_FINETUNE = 0.134  # adjust from key commands such that upright angle error is minimized
POLYFIT_ANGLED = False
# POLYFIT_ANGLED = True if 'mppi' in CONTROLLER_NAME and PREDICTOR in ['RNN'] else False  # TODO: check if necessary for new controller, it seems it does not help for RNN trained only on simulated data

##### Position Conversion #####
# Position unit conversion adc to meters: POSITION_TARGET_SOFTWARE = POSITION_TARGET_FIRMWARE*POSITION_NORMALIZATION_FACTOR
# POSITION_KP_SOFTWARE = POSITION_KP_FIRMWARE/POSITION_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
POSITION_ENCODER_RANGE = 4660  # This is an empirical approximation # seems to be 4164 now
POSITION_OFFSET = 0  # Serves to adjust starting position - position after calibration is 0
POSITION_FULL_SCALE_N = int(POSITION_ENCODER_RANGE) / 2  # Corrected position full scale - cart position should range over +- this value if calibrated for zero at center
TRACK_LENGTH = 0.396  # Total usable track length in meters
POSITION_NORMALIZATION_FACTOR = TRACK_LENGTH / POSITION_ENCODER_RANGE  # 0.000084978540773

POSITION_TARGET = 0.0  # meters

JOYSTICK_DEADZONE = 0.1  # deadzone around joystick neutral position that stick is ignored
JOYSTICK_POSITION_KP = 4.0

##### Serial Port #####
import platform
import subprocess

SERIAL_PORT = None
try:
    SERIAL_PORT = subprocess.check_output('ls -a /dev/tty.usbserial*', shell=True).decode("utf-8").strip() if platform.system() == 'Darwin' else '/dev/ttyUSB0'
except Exception as err:
    print(err)

SERIAL_BAUD = 230400  # default 230400, in firmware. Alternatives if compiled and supported by USB serial intervace are are 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000

ratio = 1.05

##### Wrong Place ##### #TODO: remove functions and calculations from parameter file
ANGLE_HANGING = np.array(0.0)  # TODO: best would be to have theses as class variables in CartPoleDriver
ANGLE_DEVIATION = np.array(0.0)


def angle_constants_update(new_angle_hanging):
    global ANGLE_ADC_RANGE

    # update angle deviation according to ANGLE_HANGING update
    if new_angle_hanging < ANGLE_ADC_RANGE / 2:
        angle_deviation = - new_angle_hanging - ANGLE_ADC_RANGE / 2  # moves upright to 0 and hanging to -pi
    else:
        angle_deviation = - new_angle_hanging + ANGLE_ADC_RANGE / 2  # moves upright to 0 and hanging to pi

    return new_angle_hanging, angle_deviation


if MOTOR == 'ORIGINAL':
    ANGLE_HANGING[...], ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_ORIGINAL)
elif MOTOR == 'POLOLU':
    ANGLE_HANGING[...], ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_POLOLU)


def inc(param):
    if param < 0.2:
        param = round(param + 0.01, 2)
    elif param < 2:
        param = round(param + 0.1, 1)
    else:
        old = param
        param = round(param * ratio)
        if param == old:
            param += 1
    return param


def dec(param):
    if param < 0.2:
        param = max(0, round(param - 0.01, 2))
    elif param < 2:
        param = max(0, round(param - 0.1, 1))
    else:
        old = param
        param = round(param / ratio)
        if param == old:
            param -= 1
    return param
