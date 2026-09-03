import os
import math
import logging
import numpy as np

from CartPole.cartpole_parameters import TrackHalfLength


CHIP = "ZYNQ"  # Can be "STM" or "ZYNQ"; remember to change chip specific values on firmware if you want to run control from there
ZYNQ_BOARD = "ZYBO_Z720"  # 'ZYBO_Z720' or 'ZEDBOARD'; must match Firmware hardware_bridge.h
# Must match Firmware/Src/CartPoleFirmware/hardware_bridge.h. When True the JB
# slider owns target_position: the driver does not send CMD_SET_TARGET_POSITION
# and shows the chip target (STATE) on the PC.
USE_EXTERNAL_INTERFACE = True
SHOW_SWITCH_MUX = True  # Zybo SW0–SW3 pick on-chip controller; chip ignores PC period/N
CONTROLLER_NAME = 'neural-imitator'  # PC k path; the show is on-chip u + switches
IROS_SHORT_POLE_PROFILE = False  # must match Firmware/Src/CartPoleFirmware/parameters.h
USE_SECLOC = False  # Wrap the selected controller with the SecLoc gate; keep False on Development Zybo
# Push config_secloc.yml to the chip (CMD_SET_SECLOC_CONFIG). Needed only when the
# on-chip SecLoc gate is in use. Leave False on Development so a home Zybo with
# older firmware is never poked with an unknown 0xD3 command.
USE_CHIP_SECLOC = False


def should_push_chip_secloc_config(use_secloc=None, use_chip_secloc=None):
    """True when the driver may send CMD_SET_SECLOC_CONFIG.

    A normal Development home run has both flags False.
    """
    if use_secloc is None:
        use_secloc = USE_SECLOC
    if use_chip_secloc is None:
        use_chip_secloc = USE_CHIP_SECLOC
    return bool(use_secloc or use_chip_secloc)


OPTIMIZER_NAME = 'rpgd-c'  # PC RPGD go-to; 'rpgd' is the TF baseline

##### Hardware (FPGA) angle filter #####
# The recovered June quant LSTM was trained and verified with median-63.
HARDWARE_ANGLE_FILTER_OVERRIDE = True
HARDWARE_ANGLE_FILTER_WINDOW = 63
HARDWARE_ANGLE_FILTER_TRIM = 0  # unused in median mode
HARDWARE_ANGLE_FILTER_MODE = 1  # median

##### Real-time CPU pinning #####
# CPU core(s) the control process is pinned to for time-predictable single-step
# execution. control.py applies this BEFORE TensorFlow is imported, so the TF/XLA
# worker threads inherit the mask (see Driver/DriverFunctions/cpu_affinity.py).
# This setting alone decides the policy:
#   non-empty -> pin to those core(s) AND run TensorFlow single-threaded. Use for
#                single-threaded optimizers, e.g. 'rpgd'.
#   ""        -> no pin, auto threading. Use for parallel optimizers, e.g. 'rpgd-c',
#                which a single-core pin would throttle.
# So set it to match OPTIMIZER_NAME above. Examples: "2" pins to core 2; "2,3" or
# "2-3" allow those cores; "" disables pinning.
# Main-thread compute inherits this process mask (io-main-split architecture).
CONTROL_CPU_AFFINITY = "2"  # "2" for TF neural-imitator / rpgd; "" for rpgd-c (OpenMP)

# Core(s) for the chip IO thread (serial polling, gate, actuation). The IO thread
# pins itself here via per-thread affinity. Full separation from compute requires
# CONTROL_CPU_AFFINITY="" so OpenMP can use other cores; with rpgd pinned to "2"
# the IO thread may remain on core 2 if the process mask allows only that core.
LOOP_CPU_AFFINITY = "3"

# Which GPUs TensorFlow may see, applied by control.py before TF is imported.
#   "-1"      -> CPU only (default; the TF control path is CPU-pinned anyway)
#   "0", "0,1"-> expose those GPU(s)
#   None      -> leave the CUDA_VISIBLE_DEVICES environment variable untouched
CONTROL_CUDA_VISIBLE_DEVICES = "-1"

# Motor type selection
# Choose 'POLOLU' or 'ORIGINAL'
# It is set automatically during calibration
# You must set it here correctly only if you want to skip calibration
# Also: In our lab we have two cartpoles - with original and pololu motors
# Setting the motor type identifies the robot instance and determines parameters which are not dependent on motor
MOTOR = 'POLOLU'

##### Controller Settings #####
# Two clocks: the chip IO thread runs every POLLING_PERIOD_MS (gate/trigger
# resolution); main thread computes and the result is applied
# CONTROLLER_APPLY_WINDOW_MS after the trigger. The classic cadence is the special case
# window == period (compute each period, apply at the next tick).
if CONTROLLER_NAME == 'pid':
    POLLING_PERIOD_MS = 5
elif CONTROLLER_NAME == 'lqr':
    POLLING_PERIOD_MS = 8
elif CONTROLLER_NAME == 'neural-imitator':
    POLLING_PERIOD_MS = 5 if IROS_SHORT_POLE_PROFILE else 10
elif CONTROLLER_NAME == 'mpc':
    POLLING_PERIOD_MS = 20  # match on-chip AMP RPGD (8 rollouts, dt == period)
elif CONTROLLER_NAME == 'fpga':
    POLLING_PERIOD_MS = 15
elif CONTROLLER_NAME.startswith('secloc'):
    POLLING_PERIOD_MS = 2  # fast polling so the Secloc gate sees fresh state every 2 ms
else:
    POLLING_PERIOD_MS = 20  # e.g. mppi

if USE_SECLOC and CONTROLLER_NAME in ('mpc', 'neural-imitator'):
    POLLING_PERIOD_MS = 5
    CONTROLLER_APPLY_WINDOW_MS = 20
else:
    CONTROLLER_APPLY_WINDOW_MS = POLLING_PERIOD_MS

if CONTROLLER_APPLY_WINDOW_MS % POLLING_PERIOD_MS != 0:
    raise ValueError(
        "CONTROLLER_APPLY_WINDOW_MS must be an integer multiple of POLLING_PERIOD_MS "
        f"(got {CONTROLLER_APPLY_WINDOW_MS} ms and {POLLING_PERIOD_MS} ms)."
    )

# Derivative span is N * POLLING_PERIOD_MS. Working TF rpgd used 20 ms (N=1 at 20 ms).
# rpgd-c at 5 ms uses N=3 → 15 ms (a bit wider than the 10 ms / N=1 run).
# Keep the env override for experiments.
if "CPP_DERIVATIVE_TIMESTEPS" in os.environ:
    _derivative_default = os.environ["CPP_DERIVATIVE_TIMESTEPS"]
elif USE_SECLOC and POLLING_PERIOD_MS == 5:
    _derivative_default = "4"
elif CONTROLLER_NAME == "mpc":
    _derivative_default = "3" if POLLING_PERIOD_MS == 5 else "1"
elif CONTROLLER_NAME == "neural-imitator":
    _derivative_default = "2"  # LSTM: 2 × 10 ms; IROS PC: 2 × 5 ms (chip N=10 at 1 ms)
else:
    _derivative_default = "2"
TIMESTEPS_FOR_DERIVATIVE = int(_derivative_default)
if not 1 <= TIMESTEPS_FOR_DERIVATIVE <= 20:
    raise ValueError("CPP_DERIVATIVE_TIMESTEPS must be between 1 and 20")

# Chip `u` (NeuralImitator / C): IROS 1 kHz with N=10 → same 10 ms derivative
# window as PC `k` (5 ms × N=2). The driver overwrites chip timing on connect;
# PhysicalCartPoleDriver restores these when firmware control is enabled.
ON_CHIP_NEURAL_POLLING_PERIOD_MS = 1
ON_CHIP_NEURAL_TIMESTEPS_FOR_DERIVATIVE = 10

if CHIP == 'STM':
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = 7200
    # First number in the tuple is multiplicative factor by which control command Q (in the range[-1,1]) is multiplied.
    # The other two shift (additive) to account for friction indep. of speed (separate for pos and neg Q)
    # Only applied if CORRECT_MOTOR_DYNAMICS is True
    MOTOR_CORRECTION_ORIGINAL = (0.63855139, 0.11653139, 0.11653139)
    MOTOR_CORRECTION_POLOLU = (0.5733488, 0.0257380, 0.0258429)
    # The 12-bit ADC has a range of 4096 units
    # However due to potentiometer dead angle these 4096 units are mapped on less than full circle
    # The full circle in adc units was determined
    # by readout difference between up and down position on the side not including dead angle
    ANGLE_HANGING_POLOLU = 1000  # Value from sensor when pendulum is at stable equilibrium point
    ANGLE_360_DEG_IN_ADC_UNITS = 4293.4
    ANGLE_HANGING_ORIGINAL = 910.0  # Value from sensor when pendulum is at stable equilibrium point
    POSITION_ENCODER_RANGE = 4672  # This is an empirical approximation
elif CHIP == 'ZYNQ' and ZYNQ_BOARD == 'ZEDBOARD':
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = 10000
    MOTOR_CORRECTION_ORIGINAL = (0.5846884, 0.0223145, 0.0224489)
    MOTOR_CORRECTION_POLOLU = (0.5846884, 0.0223145, 0.0224489)
    ANGLE_360_DEG_IN_ADC_UNITS = 4302
    ANGLE_HANGING_POLOLU = 150
    ANGLE_HANGING_ORIGINAL = 1075
    POSITION_ENCODER_RANGE = 4649
elif CHIP == 'ZYNQ':
    _iros_short_neural = IROS_SHORT_POLE_PROFILE and CONTROLLER_NAME == 'neural-imitator'
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = 2500 if _iros_short_neural else 10000
    MOTOR_CORRECTION_ORIGINAL = (0.63855139, 0.11653139, 0.11653139)
    # NOTE: MOTOR_CORRECTION is controller-dependent here on purpose.
    # The active quant LSTM must use the map from its May 2025 physical training
    # recordings: Q=-1 while moving left produced command -5991.
    MOTOR_CORRECTION_POLOLU_MLP = (0.5116974, 0.0178784, 0.0280385)   # force-fit; too weak for Dense-8 today
    LSTM_MOTOR_GAIN = float(os.environ.get("LSTM_MOTOR_GAIN", "0.5733488"))
    if not 0.0 < LSTM_MOTOR_GAIN < 0.95:
        raise ValueError("LSTM_MOTOR_GAIN must be between 0 and 0.95")
    MOTOR_CORRECTION_POLOLU_LSTM_QUANT = (0.5733488, 0.0257380, 0.0258429)
    MOTOR_CORRECTION_POLOLU_RPGD = (0.5733488, 0.0257380, 0.0258429)  # 2026-09-02 go-to: PC rpgd-c and AMP at 20 ms
    # Exact duty-cycle equivalent of IROS {1488.070, 80.797, 96.254}
    # PWM counts at period 2500; the current driver multiplies by the period.
    MOTOR_CORRECTION_POLOLU_IROS_SHORT = (0.595228, 0.0323188, 0.0385016)
    MOTOR_CORRECTION_POLOLU = (
        MOTOR_CORRECTION_POLOLU_IROS_SHORT
        if _iros_short_neural
        else (
            MOTOR_CORRECTION_POLOLU_LSTM_QUANT
            if CONTROLLER_NAME == 'neural-imitator'
            else MOTOR_CORRECTION_POLOLU_RPGD
        )
    )
    ANGLE_360_DEG_IN_ADC_UNITS = 4068.73  # 2*(upright 1238.988 - hanging 3273.353), new analog chain 2026-09-03
    # FIXME: At first one would expect ANGLE_360_DEG_IN_ADC_UNITS to be the same for Zybo and STM
    #   It is unclear if the difference comes from measuring it on different cartpoles
    #   or is due to imprecise voltage shifting which is required on Zybo
    #   Please think it through and adjust this comment appropriately.
    # Default is the measured value. For a deliberate LQR failure test, set
    # CPP_ANGLE_HANGING_POLOLU after a PS reset for a known zero shift.
    ANGLE_HANGING_POLOLU = float(os.environ.get("CPP_ANGLE_HANGING_POLOLU", "3273.353"))
    if not 0.0 <= ANGLE_HANGING_POLOLU < 4096.0:
        raise ValueError("CPP_ANGLE_HANGING_POLOLU must be in the 12-bit ADC range")
    ANGLE_HANGING_ORIGINAL = 1078.5  # Value from sensor when pendulum is at stable equilibrium point
    POSITION_ENCODER_RANGE = 4705.0 if _iros_short_neural else 4695.0



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

DANCE_PATH = 'square'  # 'square', 'sin'
DANCE_AMPL = 0.1  # m
DANCE_PERIOD_S = 10.0
DANCE_START_TIME = 0.0
DANCE_UP_AND_DOWN = False

TIME_LIMITED_RECORDING_LENGTH = 1000  # in time steps (1 step = POLLING_PERIOD_MS)

##### Logging and Recordings #####
LOGGING_LEVEL = logging.ERROR
PATH_TO_EXPERIMENT_RECORDINGS = './ExperimentRecordings/'  # Path where the experiments data is stored
# Keep terminal redraws out of the 100 Hz control path. This does not change
# control or CSV cadence; it only refreshes the status display once per second.
PRINT_PERIOD_MS = int(os.environ.get("CPP_PRINT_PERIOD_MS", "1000"))
if PRINT_PERIOD_MS < 1:
    raise ValueError("CPP_PRINT_PERIOD_MS must be at least 1 ms")
STATISTICS_IN_TERMINAL_AVERAGING_LENGTH = 500

##### Live Plot (start with 6, save plot with 7 and reset with 8) #####
LIVE_PLOTTER_USE_REMOTE_SERVER = False
LIVE_PLOTTER_REMOTE_USERNAME = 'marcinpaluch'
LIVE_PLOTTER_REMOTE_IP = '192.168.194.233'
DEFAULT_ADDRESS = ('localhost', 6000)

CONTROL_SYNC = True  # Delays Input until next Timeslot for more accurate measurements
AUTOSTART = os.environ.get("CPP_AUTOSTART", "0").lower() in ("1", "true", "yes")
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

USE_DVS_STATE_ESTIMATION = False

USE_EKF = False
EKF_CALIBRATION_RUN = not USE_DVS_STATE_ESTIMATION  # If True, EKF calibration is run at the beginning of the experiment

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
