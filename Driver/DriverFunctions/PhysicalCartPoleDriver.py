# TODO Aftrer joystick is unplugged and plugged again it interferes with the calibration, it causes the motor to get stuck at some speed after calibration. Add this to the readme file to warn the user.
# TODO: You can easily switch between controllers in runtime using this and get_available_controller_names function
# todo check if position unit conversion works for the following features: dance mode (can be checked for a nice self.controller only)
import time

import os

from tqdm import trange

from DriverFunctions.custom_logging import my_logger
from DriverFunctions.interface import Interface, set_ftdi_latency_timer
from DriverFunctions.kbhit import KBHit

from DriverFunctions.ExperimentProtocols.experiment_protocols_manager import experiment_protocols_manager_class

from DriverFunctions.joystick import Joystick

from CartPoleSimulation.CartPole.state_utilities import create_cartpole_state, ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, POSITION_IDX, POSITIOND_IDX
from CartPoleSimulation.CartPole._CartPole_mathematical_helpers import wrap_angle_rad
from CartPoleSimulation.CartPole.latency_adder import LatencyAdder

from DriverFunctions.csv_helpers import create_csv_header, create_csv_title
from CartPoleSimulation.CartPole.csv_logger import create_csv_file_name
from CartPole.data_manager import DataManager
from SI_Toolkit.Functions.FunctionalDict import FunctionalDict
from SI_Toolkit.Functions.General.TerminalContentManager import TerminalContentManager

from Driver.DriverFunctions.interface import get_serial_port
from Driver.DriverFunctions.dancer import Dancer
from globals import *

import subprocess

import sys
from numba import jit
from DriverFunctions.numba_polyfit import fit_poly, eval_polynomial

import warnings
warnings.simplefilter('ignore', np.RankWarning)

from SI_Toolkit.LivePlotter.live_plotter_sender import LivePlotter_Sender

@jit(nopython=False, cache=True, fastmath=True)
def polyfit(buffer):
    p = fit_poly(np.arange(len(buffer)), buffer, 2)
    return eval_polynomial(p, len(buffer))
    #p = np.polyfit(x=np.arange(len(buffer)), y=buffer, deg=2)
    #return np.polyval(p=p, x=len(buffer))

class PhysicalCartPoleDriver:
    def __init__(self, CartPoleInstance):

        self.CartPoleInstance = CartPoleInstance
        self.CartPoleInstance.set_optimizer(optimizer_name=OPTIMIZER_NAME)
        self.CartPoleInstance.set_controller(controller_name=CONTROLLER_NAME)
        self.controller = self.CartPoleInstance.controller

        self.InterfaceInstance = Interface()

        self.log = my_logger(__name__)

        # Console Printing
        self.printCount = 0

        self.controlEnabled = AUTOSTART
        self.firmwareControl = False
        self.terminate_experiment = False

        try:
            self.kb = KBHit()  # can only use in posix terminal; cannot use from spyder ipython console for example
            self.kbAvailable = True
        except:
            self.kbAvailable = False

        # Dance Mode
        self.dancer = Dancer()

        # Experiment Protocols
        self.experiment_protocols_manager = experiment_protocols_manager_class(self)
        self.current_experiment_protocol = self.experiment_protocols_manager.get_experiment_protocol()

        # Motor Commands
        self.Q = 0.0 # Motor command normed to be in a range -1 to 1
        self.Q_prev = None
        self.actualMotorCmd = 0
        self.actualMotorCmd_prev = None
        self.command = 0

        # State
        self.s = create_cartpole_state()
        self.angle_raw = 0
        self.angleD_raw = 0
        self.angle_raw_stable = 0.0
        self.angleD_raw_stable = 0.0
        self.last_difference = None
        self.angleD_raw_buffer = np.zeros((0))
        self.angle_raw_sensor = None
        self.angleD_raw_sensor = None
        self.invalid_steps = 0
        self.freezme = 0
        self.position_raw = 0
        self.positionD_raw = 0
        self.angleDPrev = 0.0
        self.positionDPrev = 0.0
        self.angleErr = 0.0
        self.positionErr = 0.0

        # Target
        self.position_offset = 0
        self.target_position = 0.0
        self.target_position_previous = 0.0
        self.target_equilibrium_previous = 0 # -1 or 1, 0 is not a valid value, but this ensures that at the begining the target equilibrium is always updated
        self.base_target_position = 0.0

        # Joystick variable
        self.joystick = Joystick()

        # Timing
        self.startTime = None
        self.lastTime = None
        self.lastControlTime = None
        self.timeNow = None
        self.elapsedTime = None
        self.sent = 0
        self.lastSent = None
        self.time_difference = 0

        # Performance
        self.delta_time = 0
        self.delta_time_buffer = np.zeros((0))
        self.firmware_latency = 0
        self.firmware_latency_buffer = np.zeros((0))
        self.python_latency = 0
        self.python_latency_buffer = np.zeros((0))
        self.controller_steptime = 0
        self.controller_steptime_previous = 0
        self.controller_steptime_buffer = np.zeros((0))
        self.controlled_iterations = 0
        self.total_iterations = 0
        self.latency_violation = 0
        self.latency_violations = 0

        # Artificial Latency
        self.additional_latency = 0.0
        self.LatencyAdderInstance = LatencyAdder(latency=self.additional_latency, dt_sampling=0.005)
        self.s_delayed = np.copy(self.s)

        self.time_last_switch = -np.inf

        self.demo_program = DEMO_PROGRAM

        self.safety_switch_counter = 0

        self.angle_deviation_finetune = 0.0

        self.angle_history = [-1] * (TIMESTEPS_FOR_DERIVATIVE + 1)  # Buffer to store past angles
        self.position_history = [-1] * (TIMESTEPS_FOR_DERIVATIVE + 1)  # Buffer to store past positions
        self.frozen_history = [0] * (TIMESTEPS_FOR_DERIVATIVE + 1)  # Buffer to store frozen states
        self.idx_for_derivative_calculation = 0
        self.idx_for_derivative_calculation_position = 0

        self.angleD_buffer = np.zeros(ANGLE_D_MEDIAN_LEN, dtype=np.float32)  # Buffer for angle derivatives
        self.positionD_buffer = np.zeros(POSITION_D_MEDIAN_LEN, dtype=np.float32)  # Buffer for position derivatives
        self.angleD_median_buffer_index = 0
        self.positionD_median_buffer_index = 0

        # The below dict lists variables to be logged with csv file when recording is on
        # Just add new variable here and it will be logged
        self.dict_data_to_save_basic = FunctionalDict({

            'time': lambda: self.elapsedTime,
            'deltaTimeMs': lambda: self.delta_time * 1000,

            'angle_raw': lambda: self.angle_raw,
            'angleD_raw': lambda: self.angleD_raw,
            'angle': lambda: self.s[ANGLE_IDX],
            'angleD': lambda: self.s[ANGLED_IDX],
            'angle_cos': lambda: self.s[ANGLE_COS_IDX],
            'angle_sin': lambda: self.s[ANGLE_SIN_IDX],
            'position_raw': lambda: self.position_raw,
            'position': lambda: self.s[POSITION_IDX],
            'positionD': lambda: self.s[POSITIOND_IDX],

            'target_position': lambda: self.target_position,
            'target_equilibrium': lambda: self.CartPoleInstance.target_equilibrium,

            'actualMotorSave': lambda: self.actualMotorCmd_prev,
            'Q': lambda: self.Q_prev,

            'measurement': lambda: self.current_experiment_protocol,

            'angle_squared': lambda: self.s[ANGLE_IDX] ** 2,
            'position_squared': lambda: (self.s[POSITION_IDX] - self.target_position) ** 2,
            'Q_squared': lambda: self.Q_prev ** 2,

            'sent': lambda: self.time_difference,
            'latency': lambda: self.firmware_latency,
            'latency_violations': lambda: self.latency_violations,
            'pythonLatency': lambda: self.python_latency,
            'controller_steptime': lambda: self.controller_steptime_previous,
            'additionalLatency': lambda: self.additional_latency,
            'invalid_steps': lambda: self.invalid_steps,
            'freezme': lambda: self.freezme,

            'angle_raw_sensor': lambda: self.angle_raw_sensor,
            'angleD_raw_sensor': lambda: self.angleD_raw_sensor,
        })

        self.data_to_save_measurement = {}
        self.data_to_save_controller = {}

        self.data_manager = DataManager()

        self.csv_name = None
        self.recording_length = np.inf
        self.start_recording_flag = False  # Gives signal to start recording during the current control iteration, starting recording may take more than one control iteration

        self.tcm = None  # Terminal Content Manager

        self.live_plotter_sender = LivePlotter_Sender(
            DEFAULT_ADDRESS,
            LIVE_PLOTTER_USE_REMOTE_SERVER,
            LIVE_PLOTTER_REMOTE_USERNAME,
            LIVE_PLOTTER_REMOTE_IP
        )

    @property
    def recording_running(self):
        return self.data_manager.recording_running

    @property
    def starting_recording(self):
        return self.data_manager.starting_recording

    def run(self):
        with TerminalContentManager(special_print_function=True) as tcm:
            self.tcm = tcm
            self.setup()
            self.run_experiment()
            self.quit_experiment()

    def setup(self):
        # Check that we are running from terminal, otherwise we cannot control it
        if not sys.stdin.isatty():
            print('Run from an interactive terminal to allow keyboard input.')
            quit()

        SERIAL_PORT = get_serial_port(chip_type=CHIP, serial_port_number=SERIAL_PORT_NUMBER)
        if CHIP == 'ZYNQ':
            set_ftdi_latency_timer(SERIAL_PORT)
        self.InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)
        self.InterfaceInstance.control_mode(False)
        self.InterfaceInstance.stream_output(False)

        self.log.info('\n Opened ' + str(SERIAL_PORT) + ' successfully')

        self.joystick.setup()

        try:
            self.controller.loadparams()
        except AttributeError:
            print('loadparams not defined for this self.controller')

        time.sleep(1)

        # set_firmware_parameters(self.InterfaceInstance)
        self.InterfaceInstance.set_config_control(controlLoopPeriodMs=CONTROL_PERIOD_MS, controlSync=CONTROL_SYNC, angle_hanging=ANGLE_HANGING, avgLen=ANGLE_AVG_LENGTH, correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS)

        try:
            self.controller.printparams()
        except AttributeError:
            print('printparams not implemented for this self.controller.')

        if hasattr(self.controller, 'print_help'):
            self.controller.print_help()

        self.startTime = time.time()
        self.lastTime = self.startTime
        self.lastControlTime = self.startTime

        self.InterfaceInstance.stream_output(True)  # now start streaming state

    def run_experiment(self):

        while not self.terminate_experiment:
            self.experiment_sequence()


    def experiment_sequence(self):

        self.keyboard_input()

        if self.controlEnabled:
            self.controller_steptime_previous = self.controller_steptime

        self.get_state_and_time_measurement()

        if self.demo_program and self.controlEnabled:
            self.dancer.danceEnabled = True
            if self.timeNow - self.time_last_switch > 8:
                self.time_last_switch = self.timeNow
                if self.CartPoleInstance.target_equilibrium == 1:
                    self.CartPoleInstance.target_equilibrium = -1
                else:
                    self.CartPoleInstance.target_equilibrium = 1

        self.experiment_protocol_step()

        if self.start_recording_flag:
            self.start_csv_recording()
            self.start_recording_flag = False

        self.set_target_position()

        if self.controlEnabled or self.firmwareControl:
            self.controlled_iterations += 1
        else:
            self.controlled_iterations = 0

        if self.controlEnabled:
            # Active Python Control: set values from controller
            self.lastControlTime = self.timeNow
            start = time.time()
            self.Q = float(self.controller.step(self.s, self.time_of_measurement, {"target_position": self.target_position,
                                                                       "target_equilibrium": self.CartPoleInstance.target_equilibrium}))
            performance_measurement[0] = time.time() - start
            self.controller_steptime = time.time() - start
            if AUTOSTART:
                self.Q = 0
        else:
            pass
            # Observing Firmware Control: set values from firmware for logging
            # self.actualMotorCmd = self.command
            # self.Q = self.command / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES

        self.Q = self.joystick.action(self.s[POSITION_IDX], self.Q, self.controlEnabled)

        if self.controlEnabled or self.current_experiment_protocol.is_running():
            self.control_signal_to_motor_command()

        if self.controlEnabled or self.current_experiment_protocol.is_running():
            self.motor_command_safety_check()
            self.safety_switch_off()

        if self.controlEnabled or (self.current_experiment_protocol.is_running() and self.current_experiment_protocol.Q is not None):
            self.InterfaceInstance.set_motor(self.actualMotorCmd)

        if self.firmwareControl:
            self.actualMotorCmd = self.command

        # Logging, Plotting, Terminal

        self.csv_recording_step()

        self.plot_live()

        self.write_current_data_to_terminal()

        self.actualMotorCmd_prev = self.actualMotorCmd
        self.Q_prev = self.Q

        self.update_parameters_in_cartpole_instance()

        self.end = time.time()
        self.python_latency = self.end - self.InterfaceInstance.start

    def quit_experiment(self):
        # when x hit during loop or other loop exit
        self.InterfaceInstance.set_motor(0)  # turn off motor
        self.InterfaceInstance.close()
        self.joystick.quit()
        self.live_plotter_sender.close()
        self.finish_csv_recording()

    def keyboard_input(self):

        if self.kbAvailable & self.kb.kbhit():

            c = self.kb.getch()
            try:
                self.controller.keyboard_input(c)
            except AttributeError:
                pass

            # Define a dictionary to map key presses to methods
            self.key_actions = {
    
                    ##### Help #####
                    'h': (self.print_help, "Print this help message"),
                    '?': (self.print_help, "Print this help message"),
    
                    ##### Calibration #####
                    'K': (self.calibrate, "Calibration: find track middle"),
    
                    ##### Control Mode #####
                    'k': (self.software_controller_on_off, "PC Control On/Off"),
                    'u': (self.hardware_controller_on_off, "Chip Control On/Off"),
    
                    ##### Dance #####
                    'D': (self.dancer.on_off, "Dance Mode On/Off"),
    
                    ##### Experiment Protocols #####
                    'm': (self.change_experiment_protocol,
                          "Change Experiment Protocol: running and recording predefined sequence of movements"),
                    'n': (self.experiment_protocol_on_off, "Start/Stop Experiment Protocol"),
                    'N': (self.run_hardware_experiment, "Start Experiment Protocol from Chip"),
    
                    ##### Logging #####
                    'l': (self.recording_on_off, "Start/Stop recording to a CSV file"),
                    'L': (lambda: self.recording_on_off(time_limited_recording=True),
                          "Start/Stop time limited recording to a CSV file"),
    
                    ##### Real Time Data Vizualization #####
                    '6': (self.live_plotter_sender.on_off,
                          "Start/Stop sending data to Live Plotter Server - real time visualization"),
                    '7': (self.live_plotter_sender.save_data_and_figure_if_connected,
                          "Save data and figure at Live Plotter Server"),
                    '8': (self.live_plotter_sender.reset_if_connected, "Reset Live Plotter Server"),
    
                    ##### Target #####
                    ';': (self.switch_target_equilibrium, "Switch target equilibrium"),
                    ']': (lambda: self.change_target_position(change_direction="increase"), "Increase target position"),
                    '[': (lambda: self.change_target_position(change_direction="decrease"), "Decrease target position"),
    
                    ##### Fine tune zero angle #####
                    'b': (self.precise_angle_measurement, "Start precise angle measurement - multiple samples"),
                    '=': (lambda: self.finetune_zero_angle(direction='increase'),
                          "Finetune zero angle - increase angle deviation parameter"),
                    '-': (lambda: self.finetune_zero_angle(direction='decrease'),
                          "Finetune zero angle - decrease angle deviation parameter"),
    
                    ##### Artificial Latency  #####
                    '9': (lambda: self.change_additional_latency(change_direction="increase"), "Increase additional latency"),
                    '0': (lambda: self.change_additional_latency(change_direction="decrease"), "Decrease additional latency"),
    
                    ##### Joystick  #####
                    'j': (lambda: self.joystick.toggle_mode(self.log), "Joystick On/Off"),
    
                    ##### Empty ######
                    '.': (lambda: None, "Key not assigned"),
                    ',': (lambda: None, "Key not assigned"),
                    '/': (lambda: None, "Key not assigned"),
                    '5': (lambda: None, "Key not assigned"),
    
                    ##### Exit ######
                    chr(27): (self.start_experiment_termination, "ESC: Start experiment termination")# ESC
            }

            # Execute the action if the key is in the dictionary
            if c in self.key_actions:
                self.key_actions[c][0]()

    def print_help(self):
        # Generate the help text dynamically
        help_text = "Key Bindings:\n"
        for key, (func, description) in self.key_actions.items():
            help_text += f" {key}: {description}\n"

        print(help_text)
        
        self.controller.print_help()

    def recording_on_off(self, time_limited_recording=False):
        # (Exclude situation when recording is just being initialized, it may take more than one control iteration)
        if not self.starting_recording:
            if not self.recording_running:
                if hasattr(self.controller, "controller_name"):
                    controller_name = self.controller.controller_name
                else:
                    controller_name = ''
                if hasattr(self.controller, "optimizer_name") and self.controller.has_optimizer:
                    optimizer_name = self.controller.optimizer_name
                else:
                    optimizer_name = ''
                self.csv_name = create_csv_file_name(controller_name=controller_name,
                                                     controller=self.controller,
                                                     optimizer_name=optimizer_name, prefix='CPP')
                if time_limited_recording:
                    self.recording_length = TIME_LIMITED_RECORDING_LENGTH
                else:
                    self.recording_length = np.inf

                self.start_recording_flag = True

            else:
                self.finish_csv_recording(wait_till_complete=False)

    def experiment_protocol_on_off(self):
        if self.current_experiment_protocol.is_idle():
            self.current_experiment_protocol.start()
        else:
            self.current_experiment_protocol.stop()
            self.Q = 0.0
            self.InterfaceInstance.set_motor(0)

    def change_experiment_protocol(self):
        if self.current_experiment_protocol.is_running():
            self.current_experiment_protocol.stop()
        self.current_experiment_protocol = self.experiment_protocols_manager.get_next_experiment_protocol()

    def finetune_zero_angle(self, direction='increase'):
        step_change = 0.002
        if direction == 'increase':
            self.angle_deviation_finetune += step_change
            print("\nIncreased angle deviation fine tune value to {0}".format(self.angle_deviation_finetune))
        elif direction == 'decrease':
            self.angle_deviation_finetune -= step_change
            print("\nDecreased angle deviation fine tune value to {0}".format(self.angle_deviation_finetune))

    def change_target_position(self, change_direction="increase"):

        change_step = 10 * POSITION_NORMALIZATION_FACTOR
        if change_direction == "increase":
            self.base_target_position += change_step
        elif change_direction == "decrease":
            self.base_target_position -= change_step
        else:
            raise ValueError('Unexpected command for change_direction = '.format(change_direction))

        np.clip(
            self.base_target_position, -0.8 * (POSITION_ENCODER_RANGE // 2), 0.8 * (POSITION_ENCODER_RANGE // 2)
        )

        if change_direction == "increase":
            print("\nIncreased target position to {0} cm".format(self.base_target_position * 100))
        else:
            print("\nDecreased target position to {0} cm".format(self.base_target_position * 100))

    def switch_target_equilibrium(self):
        self.CartPoleInstance.target_equilibrium *= -1.0

    def hardware_controller_on_off(self):
        self.firmwareControl = not self.firmwareControl
        print("\nFirmware Control", self.firmwareControl)
        self.InterfaceInstance.control_mode(self.firmwareControl)

    def software_controller_on_off(self):
        # Reset Performance Buffers
        if self.controlEnabled is False:
            self.switch_on_control()

        elif self.controlEnabled is True:
            self.switch_off_control()
        print("\nself.controlEnabled= {0}".format(self.controlEnabled))

    def precise_angle_measurement(self):
        global ANGLE_DEVIATION, ANGLE_HANGING_DEFAULT
        measured_angles = []
        number_of_measurements = 1000
        time_measurement_start = time.time()
        print('Started angle measurement.')
        for _ in trange(number_of_measurements):
            (angle, _, _, _, _, _, _, _, _, _,) = self.InterfaceInstance.read_state()
            measured_angles.append(float(angle))
        time_measurement = time.time() - time_measurement_start

        angle_average = np.mean(measured_angles)
        angle_std = np.std(measured_angles)

        angle_rad = wrap_angle_rad(
            (self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - self.angle_deviation_finetune)
        angle_std_rad = angle_std * ANGLE_NORMALIZATION_FACTOR
        print('\nAverage angle of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                                  angle_rad,
                                                                                  angle_average))
        print('\nAngle std of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                              angle_std_rad,
                                                                              angle_std))
        print('\nMeasurement took {} s'.format(time_measurement))

    def run_hardware_experiment(self):
        self.controlEnabled = False
        if self.current_experiment_protocol.is_running():
            self.current_experiment_protocol.stop()
        self.InterfaceInstance.run_hardware_experiment()

    def change_additional_latency(self, change_direction="increase"):

        latency_change_step = 0.001

        if change_direction == "increase":
            self.additional_latency += latency_change_step
        elif change_direction == "decrease":
            self.additional_latency -= latency_change_step
        else:
            raise ValueError('Unexpected command for change_direction = '.format(change_direction))

        print('\nAdditional latency set now to {:.1f} ms'.format(self.additional_latency * 1000))
        self.LatencyAdderInstance.set_latency(self.additional_latency)


    def start_experiment_termination(self):
        self.log.info("\nquitting....")
        self.terminate_experiment = True

    def calibrate(self):
        global MOTOR, MOTOR_CORRECTION, ANGLE_DEVIATION, ANGLE_HANGING
        self.controlEnabled = False

        print("\nCalibrating motor position.... ")
        self.InterfaceInstance.calibrate()
        print("Done calibrating")

        if self.InterfaceInstance.encoderDirection == 1:
            MOTOR = 'POLOLU'
            MOTOR_CORRECTION = MOTOR_CORRECTION_POLOLU
            ANGLE_HANGING = ANGLE_HANGING_POLOLU

        elif self.InterfaceInstance.encoderDirection == -1:
            MOTOR = 'ORIGINAL'
            MOTOR_CORRECTION = MOTOR_CORRECTION_ORIGINAL
            ANGLE_HANGING = ANGLE_HANGING_ORIGINAL
        else:
            raise ValueError('Unexpected value for self.InterfaceInstance.encoderDirection = '.format(
                self.InterfaceInstance.encoderDirection))

        if ANGLE_HANGING_DEFAULT:
            ANGLE_DEVIATION[...] = angle_deviation_update(ANGLE_HANGING)

        print('Detected motor: {}'.format(MOTOR))

        self.InterfaceInstance.set_config_control(controlLoopPeriodMs=CONTROL_PERIOD_MS,
                                                  controlSync=CONTROL_SYNC,
                                                  angle_hanging=ANGLE_HANGING, avgLen=ANGLE_AVG_LENGTH,
                                                  correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS)

    def switch_off_control(self):
        self.controlEnabled = False
        self.Q = 0
        self.InterfaceInstance.set_motor(0)
        if self.controller.controller_name == 'mppi-tf':
            self.controller.controller_report()
        self.controller.controller_reset()
        self.danceEnabled = False
        self.target_position = self.base_target_position
        self.latency_violations = 0

    def switch_on_control(self):
        self.controlEnabled = True
        self.delta_time_buffer = np.zeros((0))
        self.firmware_latency_buffer = np.zeros((0))
        self.python_latency_buffer = np.zeros((0))
        self.controller_steptime_buffer = np.zeros((0))
        global performance_measurement, performance_measurement_buffer
        performance_measurement_buffer = np.zeros((performance_measurement.size, 0))
        self.latency_violations = 0

    def update_parameters_in_cartpole_instance(self):
        """
        Just to make changes visible in GUI
        """

        self.CartPoleInstance.s[POSITION_IDX] = self.s[POSITION_IDX]
        self.CartPoleInstance.s[POSITIOND_IDX] = self.s[POSITIOND_IDX]
        self.CartPoleInstance.s[ANGLE_IDX] = self.s[ANGLE_IDX]
        self.CartPoleInstance.s[ANGLE_COS_IDX] = self.s[ANGLE_COS_IDX]
        self.CartPoleInstance.s[ANGLE_SIN_IDX] = self.s[ANGLE_SIN_IDX]
        self.CartPoleInstance.s[ANGLED_IDX] = self.s[ANGLED_IDX]
        self.CartPoleInstance.Q = self.Q
        self.CartPoleInstance.time = self.timeNow
        self.CartPoleInstance.dt = self.controller_steptime
        self.CartPoleInstance.target_position = self.target_position

    def wrap_local(self, angle):
        ADC_RANGE = ANGLE_360_DEG_IN_ADC_UNITS
        if angle >= ADC_RANGE / 2:
            return angle - ADC_RANGE
        elif angle <= -ADC_RANGE / 2:
            return angle + ADC_RANGE
        else:
            return angle

    def get_state_and_time_measurement(self):
        # This function will block at the rate of the control loop
        (self.angle_raw, self.angleD_raw, self.position_raw, self.target_position_from_chip, self.command, self.invalid_steps, self.time_difference, self.time_of_measurement, self.firmware_latency, self.latency_violation) = self.InterfaceInstance.read_state()

        # self.treat_deadangle_with_derivative()

        self.position_difference()

        self.angle_raw_sensor = self.angle_raw
        self.angleD_raw_sensor = self.angleD_raw

        self.filter_differences()

        angle, position, angle_difference, position_difference = self.convert_angle_and_position_skale()

        self.time_measurement()

        self.check_latency_violation()

        angleDerivative, positionDerivative = self.calculate_first_derivatives(angle_difference, position_difference)

        # Pack the state into interface acceptable for the self.controller
        self.pack_features_into_state_variable(position, angle, positionDerivative, angleDerivative)

        self.add_latency()

    def treat_deadangle_with_derivative(self):

        """
        This function tries to treat the dead angle of the potentiometer.
        It tries to detect an invalid measurement in dead angle region by unusually high angular acceleration.
        The threshold and its dependence on CONTROL_PERIOD_MS and CONTROL_PERIOD_MS is heuristically guessed.
        After detecting an invalid measurement
        the function freezes the derivative and dead reckon the angle for a fixed number of measurement cycles.
        It freezes for longer when the pole goes through the dead angle upwards (decelerates).
        TODO: Making the duration of the freeze dependent on the angular velocity
            could be helpful to increase performance.
        More principled approach is welcomed.

        The invalid steps are number of corrupted measurements in the buffer for angle averaging in firmware
        see "anomaly_detection" function in firmware
        This is useful in STM where averaging is done in firmware after oversampling the angle measurement.
        TODO: In Zynq, the averaging is done in hardware, and counting invalid steps should be implemented there

        Now treating deadangle is done only on firmware.
        """

        # Calculate the index for the k-th past angle
        kth_past_index = (self.idx_for_derivative_calculation + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)
        kth_past_angle = self.angle_history[kth_past_index]

        current_difference = self.wrap_local(self.angle_raw - kth_past_angle) / TIMESTEPS_FOR_DERIVATIVE if kth_past_angle != -1 else 0

        if self.last_difference is None:
            self.last_difference = current_difference

        if (
                kth_past_angle != -1
                and
                (self.angle_raw_stable > 3500 or self.angle_raw_stable < 500)
                and
                self.freezme == 0
                and
                (
                TIMESTEPS_FOR_DERIVATIVE * abs(current_difference-self.last_difference) > CONTROL_PERIOD_MS * 2.4
                or
                # This last line is for STM32, not tested nor reworked at last revision of this function
                # Just removed the abs(self.wrap_local(kth_past_angle)) < ADC_RANGE / 20
                # as this seems to me to be covered by self.angle_raw_stable > 3500 or self.angle_raw_stable < 500
                self.invalid_steps > 5
                )
        ):

            if self.angleD_raw_stable > 0:
                self.freezme = int(45/CONTROL_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE  # Accelerates through the dead angle
            else:
                self.freezme = int(90/CONTROL_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE  # Deccelerates through the dead angle

        if self.freezme > 0:
            self.freezme -= 1
            self.angleD_raw = self.angleD_raw_stable
            if self.freezme > TIMESTEPS_FOR_DERIVATIVE + 1:
                self.angle_raw_stable += self.angleD_raw_stable
                self.angle_raw = self.wrap_local(self.angle_raw_stable)
            else:
                self.angle_raw_stable = self.angle_raw
        else:
            self.angle_raw_stable = self.angle_raw
            self.angleD_raw = current_difference
            self.angleD_raw_stable = self.angleD_raw

        self.last_difference = current_difference

        # Save current angle in the history buffer and update index
        self.angle_history[self.idx_for_derivative_calculation] = self.angle_raw
        self.frozen_history[self.idx_for_derivative_calculation] = self.freezme
        self.idx_for_derivative_calculation = (self.idx_for_derivative_calculation + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)  # Move to next index, wrap around if necessary

    def position_difference(self):

        # Calculate the index for the k-th past angle
        kth_past_index = (self.idx_for_derivative_calculation_position + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)
        kth_past_position = self.position_history[kth_past_index]

        self.positionD_raw = (self.position_raw - kth_past_position) / TIMESTEPS_FOR_DERIVATIVE if kth_past_position != -1 else 0

        # Save current angle in the history buffer and update index
        self.position_history[self.idx_for_derivative_calculation_position] = self.position_raw
        self.idx_for_derivative_calculation_position = (self.idx_for_derivative_calculation_position + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)  # Move to next index, wrap around if necessary


    def filter_differences(self):

        # Update angleD buffer with current value
        self.angleD_buffer[self.angleD_median_buffer_index] = self.angleD_raw
        self.angleD_median_buffer_index = (self.angleD_median_buffer_index + 1) % ANGLE_D_MEDIAN_LEN

        # Update positionD buffer with current value
        self.positionD_buffer[self.positionD_median_buffer_index] = self.positionD_raw
        self.positionD_median_buffer_index = (self.positionD_median_buffer_index + 1) % POSITION_D_MEDIAN_LEN

        # Calculate medians using the updated buffers
        angle_d_median = np.median(self.angleD_buffer)
        position_d_median = np.median(self.positionD_buffer)

        self.angleD_raw = angle_d_median
        self.positionD_raw = position_d_median

    def convert_angle_and_position_skale(self):
        # Convert position and angle to physical units
        angle = wrap_angle_rad((self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - self.angle_deviation_finetune)
        position = self.position_raw * POSITION_NORMALIZATION_FACTOR

        angle_difference = self.angleD_raw * ANGLE_NORMALIZATION_FACTOR
        position_difference = self.positionD_raw * POSITION_NORMALIZATION_FACTOR

        return angle, position, angle_difference, position_difference


    def time_measurement(self):
        self.lastTime = self.timeNow
        self.timeNow = time.time()
        self.elapsedTime = self.timeNow - self.startTime

        if self.time_difference < 1.0e-9:
            raise ValueError(f'\ntime_difference is {self.time_difference}. '
                             f'\nThis might indicate that timer on the microcontroller has overflown. '
                             f'\nTry to restart it.')

        self.delta_time = self.time_difference

        if self.lastSent is not None:
            delta_time_test = self.sent - self.lastSent
            if abs(delta_time_test-self.delta_time)>2e-6 and delta_time_test!=0.0:
                raise ValueError(f"dt={self.delta_time}; dtt={delta_time_test}")

        else:
            delta_time_test = 1e-6
        self.lastSent = self.sent

    # FIXME: Think if these cases are right
    def check_latency_violation(self):
        # Latency Violations
        if self.latency_violation == 1:
            self.latency_violations += 1
        elif self.delta_time > 1.5*CONTROL_PERIOD_MS/1000.0:
            self.latency_violation = 1
            self.latency_violations += np.floor(self.delta_time/(CONTROL_PERIOD_MS/1000.0))
        elif self.controlEnabled and self.firmware_latency > (CONTROL_PERIOD_MS/1000.0):
            self.latency_violation = 1
            self.latency_violations += 1
        elif self.controlEnabled and self.firmware_latency < self.controller_steptime_previous:  # Heuristic, obviosuly wrong case
            self.latency_violation = 1
            self.latency_violations += 1


    def calculate_first_derivatives(self, angle_difference, position_difference):
        # Calculating derivatives (cart velocity and angular velocity of the pole)
        angleDerivative = angle_difference / self.delta_time  # rad/self.s
        positionDerivative = position_difference / self.delta_time  # m/self.s

        return angleDerivative, positionDerivative


    def pack_features_into_state_variable(self, position, angle, positionD, angleD):
        # Pack the state into interface acceptable for the self.controller
        self.s[POSITION_IDX] = position
        self.s[ANGLE_IDX] = angle
        self.s[POSITIOND_IDX] = positionD
        self.s[ANGLED_IDX] = angleD
        self.s[ANGLE_COS_IDX] = np.cos(self.s[ANGLE_IDX])
        self.s[ANGLE_SIN_IDX] = np.sin(self.s[ANGLE_IDX])

    def add_latency(self):
        self.LatencyAdderInstance.add_current_state_to_latency_buffer(self.s)
        self.s = self.LatencyAdderInstance.get_interpolated_delayed_state()

    def set_target_position(self):

        if self.current_experiment_protocol.is_idle():
            if self.dancer.danceEnabled:
                self.dancer.dance_step(self.timeNow, self.base_target_position, self.target_position)

        if SEND_CHANGE_IN_TARGET_POSITION_ALWAYS or self.firmwareControl:
            if self.target_position != self.target_position_previous:
                self.InterfaceInstance.set_target_position(self.target_position)
                self.target_position_previous = self.target_position

            if self.CartPoleInstance.target_equilibrium != self.target_equilibrium_previous:
                self.InterfaceInstance.set_target_equilibrium(self.CartPoleInstance.target_equilibrium)
                self.target_equilibrium_previous = self.CartPoleInstance.target_equilibrium
        
        self.CartPoleInstance.target_position = self.target_position

    def experiment_protocol_step(self):
        if self.current_experiment_protocol.is_running():
            try:
                self.current_experiment_protocol.update_state(self.s[ANGLE_IDX], self.s[POSITION_IDX], self.timeNow)
                if self.current_experiment_protocol.Q is not None:
                    self.Q = self.current_experiment_protocol.Q
            except TimeoutError as e:
                if self.current_experiment_protocol.Q is not None:
                    self.Q = 0.0
                self.log.warning(f'timeout in self.measurement: {e}')

            if self.current_experiment_protocol.target_position is not None:
                self.target_position = self.current_experiment_protocol.target_position

            if self.current_experiment_protocol.target_equilibrium is not None:
                self.CartPoleInstance.target_equilibrium = self.current_experiment_protocol.target_equilibrium


    # TODO: This is now in units which are chip specific. It can be rewritten, so that calibration
    #       gets the motor full scale and calculates the correction factors relative to that
    #       When you do it, make the same correction also for firmware
    def control_signal_to_motor_command(self):

        self.actualMotorCmd = self.Q
        if CORRECT_MOTOR_DYNAMICS:
            # Use Model_velocity_bidirectional.py to determine the margins and correction factor below

            # # We cut the region which is linear
            # # In fact you don't need - it it is already ensured that Q -1 to 1 corresponds to linear range
            # self.actualMotorCmd = 1.0 if self.actualMotorCmd > 1.0 else self.actualMotorCmd
            # self.actualMotorCmd = -1.0 if self.actualMotorCmd < -1.0 else self.actualMotorCmd

            # The change dependent on velocity sign is motivated theory of classical friction
            self.actualMotorCmd *= MOTOR_CORRECTION[0]
            if self.actualMotorCmd != 0:
                if np.sign(self.s[POSITIOND_IDX]) > 0:
                    self.actualMotorCmd += MOTOR_CORRECTION[1]
                elif np.sign(self.s[POSITIOND_IDX]) < 0:
                    self.actualMotorCmd -= MOTOR_CORRECTION[2]

        self.actualMotorCmd *= MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES  # Scaling to motor units

        # Convert to motor encoder units
        self.actualMotorCmd = int(self.actualMotorCmd)

    def motor_command_safety_check(self):
        # Check if motor power in safe boundaries, not to burn it in case you have an error before or not-corrected option
        # NEVER RUN IT WITHOUT IT
        self.actualMotorCmd = np.clip(self.actualMotorCmd, -MOTOR_FULL_SCALE_SAFE, MOTOR_FULL_SCALE_SAFE)

    def safety_switch_off(self):
        # Temporary safety switch off if goes to the boundary
        if abs(self.position_raw) > 0.95 * (POSITION_ENCODER_RANGE // 2):
            self.safety_switch_counter += 1
            if self.safety_switch_counter > 10:  # Allow short bumps
                self.safety_switch_counter = 0
                print('\nSafety Switch.')
                self.controlEnabled = False
                self.InterfaceInstance.set_motor(0)

                if hasattr(self.controller, 'controller_report') and self.controlled_iterations > 1:
                    self.controller.controller_report()
                if hasattr(self.controller, 'controller_reset'):
                    self.controller.controller_reset()
                self.dancer.danceEnabled = False
                self.target_position = self.base_target_position
                self.actualMotorCmd = 0
        else:
            self.safety_switch_counter = 0
            pass

    def plot_live(self):
        if self.live_plotter_sender.connection_ready:

            if not self.live_plotter_sender.headers_sent:
                headers = ['time', 'Angle', 'Position', 'Q', "ΔQ",  'Target Position', 'AngleD', 'PositionD',]
                controller_headers = list(self.controller.controller_data_for_csv.keys())
                controller_headers = [header[len('cost_component_'):] for header in controller_headers if 'cost_component_' in header]
                self.live_plotter_sender.send_headers(headers+controller_headers)
            else:
                buffer = np.array([
                                self.elapsedTime,
                                self.s[ANGLE_IDX],
                                self.s[POSITION_IDX] * 100,
                                self.Q,
                                (self.Q-self.Q_prev),
                                self.target_position*100,
                                self.s[ANGLED_IDX],
                                self.s[POSITIOND_IDX] * 100,
                            ])
                buffer_controller = np.array([self.controller.controller_data_for_csv[key] for key in self.controller.controller_data_for_csv.keys()])

                buffer = np.append(buffer, buffer_controller)

                self.live_plotter_sender.send_data(buffer)

    def start_csv_recording(self):

        combined_keys = list(self.dict_data_to_save_basic.keys()) + list(
            self.data_to_save_measurement.keys()) + list(self.data_to_save_controller.keys())

        self.data_manager.start_csv_recording(
            self.csv_name,
            combined_keys,
            create_csv_title(),
            create_csv_header(),
            PATH_TO_EXPERIMENT_RECORDINGS,
            mode='online',
            wait_till_complete=False,
            recording_length=self.recording_length
        )

    def csv_recording_step(self):
        if self.actualMotorCmd_prev is not None and self.Q_prev is not None:
            if self.recording_running:
                self.data_manager.step([
                    self.dict_data_to_save_basic,
                    self.data_to_save_measurement,
                    self.data_to_save_controller
                ])

    def finish_csv_recording(self, wait_till_complete=True):
        if self.recording_running:
            self.data_manager.finish_experiment(wait_till_complete=wait_till_complete)
        self.recording_length = np.inf

    def write_current_data_to_terminal(self):
        self.printCount += 1

        # Averaging
        self.total_iterations += 1
        if self.total_iterations > 10 and self.controlled_iterations > 10:
            self.delta_time_buffer = np.append(self.delta_time_buffer, self.delta_time)
            self.delta_time_buffer = self.delta_time_buffer[-PRINT_AVERAGING_LENGTH:]
            self.firmware_latency_buffer = np.append(self.firmware_latency_buffer, self.firmware_latency)
            self.firmware_latency_buffer = self.firmware_latency_buffer[-PRINT_AVERAGING_LENGTH:]
            self.python_latency_buffer = np.append(self.python_latency_buffer, self.python_latency)
            self.python_latency_buffer = self.python_latency_buffer[-PRINT_AVERAGING_LENGTH:]
            self.controller_steptime_buffer = np.append(self.controller_steptime_buffer, self.controller_steptime)
            self.controller_steptime_buffer = self.controller_steptime_buffer[-PRINT_AVERAGING_LENGTH:]

            global performance_measurement, performance_measurement_buffer
            performance_measurement_buffer = np.append(performance_measurement_buffer, np.expand_dims(performance_measurement, axis=1), axis=1)
            performance_measurement_buffer = performance_measurement_buffer[:, -PRINT_AVERAGING_LENGTH:]

        if True or self.printCount >= PRINT_PERIOD_MS/CONTROL_PERIOD_MS:
            self.printCount = 0

            ESC = '\033['
            BACK_TO_BEGINNING = '\r'
            CLEAR_LINE = ESC + 'K'  # Clear the entire line

            self.tcm.print_temporary(BACK_TO_BEGINNING + CLEAR_LINE)

            ############  Mode  ############
            if self.controlEnabled:
                if 'mpc' in CONTROLLER_NAME:
                    mode='CONTROLLER:   {} (Period={}ms, Synch={}, Horizon={}, Rollouts={}, Predictor={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC, self.controller.optimizer.mpc_horizon, self.controller.optimizer.num_rollouts, self.controller.predictor.predictor_name)
                else:
                    mode='CONTROLLER:   {} (Period={}ms, Synch={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC)
            else:
                mode = 'CONTROLLER:   Firmware'
            self.tcm.print_temporary(BACK_TO_BEGINNING + mode +  CLEAR_LINE)

            ############  Mode  ############
            self.tcm.print_temporary(BACK_TO_BEGINNING + f'MEASUREMENT: {self.current_experiment_protocol}' +  CLEAR_LINE)

            ############  State  ############
            self.tcm.print_temporary(BACK_TO_BEGINNING + "STATE:  angle:{:+.3f}rad, angle raw:{:04}, position:{:+.2f}cm, position raw:{:04}, target:{}, Q:{:+.2f}, command:{:+05d}, invalid_steps:{}, freezme:{}"
                .format(
                    self.s[ANGLE_IDX],
                    self.angle_raw,
                    self.s[POSITION_IDX] * 100,
                    self.position_raw,
                    f"{self.CartPoleInstance.target_position}, {self.CartPoleInstance.target_equilibrium}",
                    self.Q,
                    self.actualMotorCmd,
                    self.invalid_steps,
                    self.freezme
                ) + CLEAR_LINE
            )

            ############  Timing  ############
            if self.total_iterations > 10 and self.controlled_iterations > 10:

                timing_string = "TIMING: delta time [μ={:.1f}ms, σ={:.2f}ms], firmware latency [μ={:.1f}ms, σ={:.2f}ms], \n         python latency [μ={:.1f}ms σ={:.2f}ms], controller step [μ={:.1f}ms σ={:.2f}ms]".format(
                                float(self.delta_time_buffer.mean() * 1000),
                                float(self.delta_time_buffer.std() * 1000),

                                float(self.firmware_latency_buffer.mean() * 1000),
                                float(self.firmware_latency_buffer.std() * 1000),

                                float(self.python_latency_buffer.mean() * 1000),
                                float(self.python_latency_buffer.std() * 1000),

                                float(self.controller_steptime_buffer.mean() * 1000),
                                float(self.controller_steptime_buffer.std() * 1000)
                    )


                self.tcm.print_temporary(
                    BACK_TO_BEGINNING + timing_string + CLEAR_LINE
                    )


                ###########  Latency Violations  ############
                percentage_latency_violations = 100 * self.latency_violations / self.total_iterations if self.total_iterations > 0 else 0
                timing_latency = f"         latency violations: {self.latency_violations}/{self.total_iterations} = {percentage_latency_violations:.1f}%"
                self.tcm.print_temporary(BACK_TO_BEGINNING + timing_latency + CLEAR_LINE)


            self.tcm.print_to_terminal()

