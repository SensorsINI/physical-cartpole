import math
import logging
import numpy as np

from CartPole.cartpole_parameters import TrackHalfLength


CHIP = "ZYNQ"  # Can be "STM" or "ZYNQ"; remember to change chip specific values on firmware if you want to run control from there
CONTROLLER_NAME = 'neural-imitator'  # e.g. 'pid', 'mpc', 'do-mpc', 'do-mpc-discrete'
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
    CONTROL_PERIOD_MS = 5
elif CONTROLLER_NAME == 'neural-imitator':
    CONTROL_PERIOD_MS = 5
elif CONTROLLER_NAME == 'fpga':
    CONTROL_PERIOD_MS = 15
else:
    CONTROL_PERIOD_MS = 14  # e.g. 5 for PID or 20 for mppi

if CHIP == 'STM':
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = 7200
    # First number in the tuple is multiplicative factor by which control command Q (in the range[-1,1]) is multiplied.
    # The other two shift (additive) to account for friction indep. of speed (separate for pos and neg Q)
    # Only applied if CORRECT_MOTOR_DYNAMICS is True
    MOTOR_CORRECTION_ORIGINAL = (0.63855139, 0.11653139, 0.11653139)
    MOTOR_CORRECTION_POLOLU = (0.595228, 0.0323188, 0.0385016)
    # The 12-bit ADC has a range of 4096 units
    # However due to potentiometer dead angle these 4096 units are mapped on less than full circle
    # The full circle in adc units was determined
    # by readout difference between up and down position on the side not including dead angle
    ANGLE_360_DEG_IN_ADC_UNITS = 4271.34
    ANGLE_HANGING_POLOLU = 1028.579  # Value from sensor when pendulum is at stable equilibrium point
    ANGLE_HANGING_ORIGINAL = 910.0  # Value from sensor when pendulum is at stable equilibrium point
    POSITION_ENCODER_RANGE = 4164  # This is an empirical approximation
elif CHIP == 'ZYNQ':
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = 10000  # STM value is the default, we make it match concerning Zybo PL clock
    MOTOR_CORRECTION_ORIGINAL = (0.63855139, 0.11653139, 0.11653139)
    MOTOR_CORRECTION_POLOLU = (0.5733488, 0.0257380, 0.0258429)
    ANGLE_360_DEG_IN_ADC_UNITS = 4081.9  # Explanation - see above for STM case.
    # FIXME: At first one would expect ANGLE_360_DEG_IN_ADC_UNITS to be the same for Zybo and STM
    #   It is unclear if the difference comes from measuring it on different cartpoles
    #   or is due to imprecise voltage shifting which is required on Zybo
    #   Please think it through and adjust this comment appropriately.
    ANGLE_HANGING_POLOLU = 1003.0  # Value from sensor when pendulum is at stable equilibrium point
    ANGLE_HANGING_ORIGINAL = 1008.5  # Value from sensor when pendulum is at stable equilibrium point
    POSITION_ENCODER_RANGE = 4695.0  # For new implementation with Zybo. FIXME: Not clear why different then for STM



else:
    raise Exception("Unknown chip " + CHIP)

if MOTOR == 'ORIGINAL':
    MOTOR_CORRECTION = MOTOR_CORRECTION_ORIGINAL
    ANGLE_HANGING = ANGLE_HANGING_ORIGINAL
elif MOTOR == 'POLOLU':
    MOTOR_CORRECTION = MOTOR_CORRECTION_POLOLU
    ANGLE_HANGING = ANGLE_HANGING_POLOLU
else:
    raise Exception("Unknown motor type " + MOTOR)

DEMO_PROGRAM = False

DANCE_PATH = 'square'  # 'square', 'sin'
DANCE_AMPL = 0.14  # m
DANCE_PERIOD_S = 5.0
DANCE_START_TIME = 0.0

TIME_LIMITED_RECORDING_LENGTH = 1000  # in time steps (1 step = CONTROL_PERIOD_MS)

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
JSON_PATH = 'CartPoleSimulation/Control_Toolkit_ASF/'

##### Motor Settings #####
CORRECT_MOTOR_DYNAMICS = False if CONTROLLER_NAME == 'pid' else True  # Linearize and Threshold Motor Commands

MOTOR_FULL_SCALE_SAFE = int(0.95 * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES + 0.5)  # Including a safety constraint

##### Angle Conversion #####
# Angle unit conversion adc to radians: (ANGLE_TARGET + ANGLE DEVIATION - ANGLE_360_DEG_IN_ADC_UNITS/2)/ANGLE_360_DEG_IN_ADC_UNITS*math.pi
ANGLE_AVG_LENGTH = 1  # adc routine in firmware reads ADC this many times quickly in succession to reduce noise

ANGLE_HANGING_DEFAULT = True  # If True default ANGLE_HANGING is loaded for a respective cartpole when motor is detected at calibration
#  This variable changes to false after b is pressed - you can first measure angle hanging and than calibrate without overwritting
# At the beginning always default angle hanging for default motor specified in globals is loaded

ANGLE_NORMALIZATION_FACTOR = (2 * math.pi) / ANGLE_360_DEG_IN_ADC_UNITS

ANGLE_DERIVATIVE_TIMESTEP_IN_SAMPLES = 2  # TODO: Python only, hardware sets it separately.

ANGLE_D_MEDIAN_LEN = 1
POSITION_D_MEDIAN_LEN = 1
##### Position Conversion #####

POSITION_NORMALIZATION_FACTOR = TrackHalfLength * 2 / POSITION_ENCODER_RANGE  # 0.000084978540773

JOYSTICK_DEADZONE = 0.1  # deadzone around joystick neutral position that stick is ignored
JOYSTICK_POSITION_KP = 4.0

##### Serial Port #####
SERIAL_PORT_NUMBER = 1
SERIAL_BAUD = 230400  # default 230400, in firmware. Alternatives if compiled and supported by USB serial intervace are are 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000

ratio = 1.05

##### Wrong Place ##### #TODO: remove functions and calculations from parameter file
ANGLE_DEVIATION = np.array(0.0)

SEND_CHANGE_IN_TARGET_POSITION_ALWAYS = True  # If false it sends change in target position only if firmware control is active.


def angle_deviation_update(new_angle_hanging):
    global ANGLE_360_DEG_IN_ADC_UNITS

    # update angle deviation according to ANGLE_HANGING update
    if new_angle_hanging < ANGLE_360_DEG_IN_ADC_UNITS / 2:
        angle_deviation = - new_angle_hanging - ANGLE_360_DEG_IN_ADC_UNITS / 2  # moves upright to 0 and hanging to -pi
    else:
        angle_deviation = - new_angle_hanging + ANGLE_360_DEG_IN_ADC_UNITS / 2  # moves upright to 0 and hanging to pi

    return angle_deviation

ANGLE_DEVIATION[...] = angle_deviation_update(ANGLE_HANGING)



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
