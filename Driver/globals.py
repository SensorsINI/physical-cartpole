import math
import logging
import numpy as np

from Driver.DriverFunctions.interface import get_serial_port
from CartPole.cartpole_parameters import TrackHalfLength


CHIP = "ZYNQ"  # Can be "STM" or "ZYNQ"; remember to change chip specific values on firmware if you want to run control from there
CONTROLLER_NAME = 'mpc'  # e.g. 'pid', 'mpc', 'do-mpc', 'do-mpc-discrete'
OPTIMIZER_NAME = 'rpgd-tf'  # e.g. 'rpgd-tf', 'mppi', only taken into account if CONTROLLER_NAME = 'mpc'

# Motor type selection
# Choose 'POLOLU' or 'ORIGINAL'
# It is set automatically during calibration
# You must set it here correctly only if you want to skip calibration
# Also: In our lab we have two cartpoles - with original and pololu motors
# Setting the motor type identifies the robot instance and determines parameters which are not dependent on motor
MOTOR = 'POLOLU'

##### Controller Settings #####
if CONTROLLER_NAME == 'pid':
    CONTROL_PERIOD_MS = 3
elif CONTROLLER_NAME == 'neural-imitator':
    CONTROL_PERIOD_MS = 3
elif CONTROLLER_NAME == 'fpga':
    CONTROL_PERIOD_MS = 15
else:
    CONTROL_PERIOD_MS = 20  # e.g. 5 for PID or 20 for mppi

if CHIP == 'STM':
    PWM_PERIOD_IN_CLOCK_CYCLES = 7200
    # First number in the tuple is multiplicative factor by which control command Q (in the range[-1,1]) is multiplied.
    # The other two shift (additive) to account for friction indep. of speed (separate for pos and neg Q)
    # Only applied if CORRECT_MOTOR_DYNAMICS is True
    # Otherwise Q is multiplied by MOTOR_FULL_SCALE_SAFE
    MOTOR_CORRECTION_ORIGINAL = (4597.57, 839.026, 839.026)
    MOTOR_CORRECTION_POLOLU = (4181.47, 505.0, 334.43)
    # The 12-bit ADC has a range of 4096 units
    # However due to potentiometer dead angle these 4096 units are mapped on less than full circle
    # The full circle in adc units was determined
    # by readout difference between up and down position on the side not including dead angle
    ANGLE_360_DEG_IN_ADC_UNITS = 4271.34
    ANGLE_HANGING_POLOLU = 875.0  # Value from sensor when pendulum is at stable equilibrium point
    ANGLE_HANGING_ORIGINAL = 1046.75  # Value from sensor when pendulum is at stable equilibrium point
    POSITION_ENCODER_RANGE = 4164  # This is an empirical approximation
elif CHIP == 'ZYNQ':
    PWM_PERIOD_IN_CLOCK_CYCLES = 2500
    MOTOR_CORRECTION_POLOLU = (1451.899,  175.347,  116.122)  # Explanation - see above, same as for STM case
    ANGLE_360_DEG_IN_ADC_UNITS = 4068.67  # Explanation - see above for STM case.
    # FIXME: At first one would expect ANGLE_360_DEG_IN_ADC_UNITS to be the same for Zybo and STM
    #   It is unclear if the difference comes from measuring it on different cartpoles
    #   or is due to imprecise voltage shifting which is required on Zybo
    #   Please think it through and adjust this comment appropriately.
    ANGLE_HANGING_POLOLU = 811.6  # Value from sensor when pendulum is at stable equilibrium point # TODO: Would be better pointing downwards and recalculate later
    POSITION_ENCODER_RANGE = 4705  # For new implementation with Zybo. FIXME: Not clear why different then for STM



else:
    raise Exception("Unknown chip " + CHIP)


DEMO_PROGRAM = False

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

CONTROL_SYNC = True  # Delays Input until next Timeslot for more accurate measurements
AUTOSTART = False  # Autostarts Zero-Controller for Performance Measurement
JSON_PATH = 'Json/'

##### Motor Settings #####
CORRECT_MOTOR_DYNAMICS = False if CONTROLLER_NAME == 'pid' else True  # Linearize and Threshold Motor Commands

MOTOR_FULL_SCALE = PWM_PERIOD_IN_CLOCK_CYCLES-1  # 7199 # with pololu motor and scaling in firmware #7199 # with original motor
MOTOR_FULL_SCALE_SAFE = int(0.95 * MOTOR_FULL_SCALE + 0.5)  # Including a safety constraint

##### Angle Conversion #####
# Angle unit conversion adc to radians: (ANGLE_TARGET + ANGLE DEVIATION - ANGLE_360_DEG_IN_ADC_UNITS/2)/ANGLE_360_DEG_IN_ADC_UNITS*math.pi
ANGLE_AVG_LENGTH = 1  # adc routine in firmware reads ADC this many times quickly in succession to reduce noise

ANGLE_HANGING_DEFAULT = True  # If True default ANGLE_HANGING is loaded for a respective cartpole when motor is detected at calibration
#  This variable changes to false after b is pressed - you can first measure angle hanging and than calibrate without overwritting
# At the beginning always default angle hanging for default motor specified in globals is loaded

ANGLE_NORMALIZATION_FACTOR = (2 * math.pi) / ANGLE_360_DEG_IN_ADC_UNITS

ANGLE_DERIVATIVE_TIMESTEP_IN_SAMPLES = 1  # TODO: Python only, hardware sets it separately.

ANGLE_D_MEDIAN_LEN = 1
POSITION_D_MEDIAN_LEN = 1
##### Position Conversion #####

POSITION_NORMALIZATION_FACTOR = TrackHalfLength * 2 / POSITION_ENCODER_RANGE  # 0.000084978540773

JOYSTICK_DEADZONE = 0.1  # deadzone around joystick neutral position that stick is ignored
JOYSTICK_POSITION_KP = 4.0

##### Serial Port #####
serial_port_number = 1
SERIAL_PORT = get_serial_port(chip_type=CHIP, serial_port_number=serial_port_number)

SERIAL_BAUD = 230400  # default 230400, in firmware. Alternatives if compiled and supported by USB serial intervace are are 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000

ratio = 1.05

##### Wrong Place ##### #TODO: remove functions and calculations from parameter file
ANGLE_DEVIATION = np.array(0.0)


def angle_constants_update(new_angle_hanging):
    global ANGLE_360_DEG_IN_ADC_UNITS

    # update angle deviation according to ANGLE_HANGING update
    if new_angle_hanging < ANGLE_360_DEG_IN_ADC_UNITS / 2:
        angle_deviation = - new_angle_hanging - ANGLE_360_DEG_IN_ADC_UNITS / 2  # moves upright to 0 and hanging to -pi
    else:
        angle_deviation = - new_angle_hanging + ANGLE_360_DEG_IN_ADC_UNITS / 2  # moves upright to 0 and hanging to pi

    return angle_deviation


if MOTOR == 'ORIGINAL':
    ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_ORIGINAL)
elif MOTOR == 'POLOLU':
    ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_POLOLU)


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
