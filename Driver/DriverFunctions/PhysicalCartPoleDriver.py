# TODO Aftrer joystick is unplugged and plugged again it interferes with the calibration, it causes the motor to get stuck at some speed after calibration. Add this to the readme file to warn the user.
# TODO: You can easily switch between controllers in runtime using this and get_available_controller_names function
# todo check if position unit conversion works for the following features: dance mode (can be checked for a nice self.controller only)
import time

import os

from tqdm import trange

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame.joystick as joystick  # https://www.pygame.org/docs/ref/joystick.html

from DriverFunctions.custom_logging import my_logger
from DriverFunctions.interface import Interface, set_ftdi_latency_timer
from DriverFunctions.kbhit import KBHit

from DriverFunctions.ExperimentProtocols.experiment_protocols_manager import experiment_protocols_manager_class

from DriverFunctions.joystick import setup_joystick, get_stick_position, motorCmd_from_joystick

from CartPoleSimulation.CartPole.state_utilities import create_cartpole_state, ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, POSITION_IDX, POSITIOND_IDX
from CartPoleSimulation.CartPole._CartPole_mathematical_helpers import wrap_angle_rad
from CartPoleSimulation.CartPole.latency_adder import LatencyAdder

from DriverFunctions.csv_helpers import csv_init

from globals import *

import subprocess
from multiprocessing.connection import Client
import sys
from numba import jit
from DriverFunctions.numba_polyfit import fit_poly, eval_polynomial

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import threading

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
        self.new_console_output = True

        self.controlEnabled = AUTOSTART
        self.firmwareControl = False
        self.manualMotorSetting = False
        self.terminate_experiment = False

        # CSV Logging
        self.loggingEnabled = False
        self.csvfile = None
        self.csvfilename = None
        self.csvwriter = None
        self.csv_init_thread = None
        self.logging_time_limited_started = False
        self.logging_time_limited_max = 1000
        self.logging_counter = 0

        # Live Plot
        self.livePlotEnabled = LIVE_PLOT

        try:
            self.kb = KBHit()  # can only use in posix terminal; cannot use from spyder ipython console for example
            self.kbAvailable = True
        except:
            self.kbAvailable = False

        # Dance Mode
        ## Parameters
        self.danceAmpl = 0.14  # m
        self.dancePeriodS = 6.0
        self.dance_start_time = 0.0
        
        ## Variables
        self.danceEnabled = False
        self.dance_finishing = False
        self.dance_current_relative_position = 0.0

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
        self.angle_raw_stable = None
        self.angleD_raw_stable = None
        self.angleD_raw_buffer = np.zeros((0))
        self.angle_raw_sensor = None
        self.angleD_raw_sensor = None
        self.angleD_fitted = None
        self.invalid_steps = 0
        self.frozen = 0
        self.fitted = 0
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
        self.stick = None
        self.joystickMode = None
        self.stickPos = None
        self.stickControl = None

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

        self.derivative_timestep_in_samples = ANGLE_DERIVATIVE_TIMESTEP_IN_SAMPLES
        self.buffer_size_for_derivative_calculation = self.derivative_timestep_in_samples + 1
        self.angle_history = [-1] * self.buffer_size_for_derivative_calculation  # Buffer to store past angles
        self.position_history = [-1] * self.buffer_size_for_derivative_calculation  # Buffer to store past positions
        self.frozen_history = [0] * self.buffer_size_for_derivative_calculation  # Buffer to store frozen states
        self.idx_for_derivative_calculation = 0

        self.angleD_buffer = np.zeros(ANGLE_D_MEDIAN_LEN, dtype=np.float32)  # Buffer for angle derivatives
        self.positionD_buffer = np.zeros(POSITION_D_MEDIAN_LEN, dtype=np.float32)  # Buffer for position derivatives
        self.angleD_median_buffer_index = 0
        self.positionD_median_buffer_index = 0

    def run(self):
        self.setup()
        self.run_experiment()
        self.quit_experiment()

    def setup(self):
        # Check that we are running from terminal, otherwise we cannot control it
        if not sys.stdin.isatty():
            print('Run from an interactive terminal to allow keyboard input.')
            quit()

        if CHIP == 'ZYNQ':
            set_ftdi_latency_timer(SERIAL_PORT)
        self.InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)
        self.InterfaceInstance.control_mode(False)
        self.InterfaceInstance.stream_output(False)

        self.log.info('\n Opened ' + str(SERIAL_PORT) + ' successfully')

        self.stick, self.joystickMode = setup_joystick()

        try:
            self.controller.loadparams()
        except AttributeError:
            print('loadparams not defined for this self.controller')

        time.sleep(1)

        # set_firmware_parameters(self.InterfaceInstance)
        self.InterfaceInstance.set_config_control(controlLoopPeriodMs=CONTROL_PERIOD_MS, controlSync=CONTROL_SYNC, angle_deviation=ANGLE_DEVIATION, avgLen=ANGLE_AVG_LENGTH, correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS)

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
            self.danceEnabled = True
            if self.timeNow - self.time_last_switch > 8:
                self.time_last_switch = self.timeNow
                if self.CartPoleInstance.target_equilibrium == 1:
                    self.CartPoleInstance.target_equilibrium = -1
                else:
                    self.CartPoleInstance.target_equilibrium = 1

        self.experiment_protocol_step()

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
            # self.Q = self.command / MOTOR_FULL_SCALE



        self.joystick_action()

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
        if self.loggingEnabled:
            self.write_csv_row()
        if self.livePlotEnabled:
            self.plot_live()
        self.write_current_data_to_terminal()

        self.update_parameters_in_cartpole_instance()

        self.end = time.time()
        self.python_latency = self.end - self.InterfaceInstance.start

    def quit_experiment(self):
        # when x hit during loop or other loop exit
        self.InterfaceInstance.set_motor(0)  # turn off motor
        self.InterfaceInstance.close()
        joystick.quit()

        if self.loggingEnabled:
            self.csvfile.close()

    def keyboard_input(self):
        global ANGLE_DEVIATION, ANGLE_HANGING_DEFAULT

        if self.kbAvailable & self.kb.kbhit():
            self.new_console_output = True

            c = self.kb.getch()
            try:
                # Keys used in self.controller: 1,2,3,4,p, =, -, w, q, s, a, x, z, r, e, f, d, v, c, S, L, b, j
                self.controller.keyboard_input(c)
            except AttributeError:
                pass

            ##### Manual Motor Movement #####
            if c == '.':  # zero motor
                self.controlEnabled = False
                self.Q = 0
                self.manualMotorSetting = False
                print('\nNormed motor command after .', self.Q)
            elif c == ',':  # left
                self.controlEnabled = False
                self.Q -= 0.01
                self.manualMotorSetting = True
                print('\nNormed motor command after ,', self.Q)
            elif c == '/':  # right
                self.controlEnabled = False
                self.Q += 0.01
                self.manualMotorSetting = True
                print('\nNormed motor command after /', self.Q)


            elif c == 'D':
                # We want the sinusoid to start at predictable (0) position
                if self.danceEnabled is True:
                    self.dance_finishing = True
                else:
                    self.dance_start_time = time.time()
                    self.danceEnabled = True
                    print(f"\nself.danceEnabled= {self.danceEnabled}")

            ##### Logging #####
            elif c == 'l' or c == 'L':
                loggingEnabled_local = not self.loggingEnabled
                print("\nself.loggingEnabled= {0}".format(loggingEnabled_local))
                if loggingEnabled_local:
                    def f():
                        self.csvfilename, self.csvfile, self.csvwriter = csv_init(controller_name = self.controller.controller_name)
                        self.loggingEnabled = loggingEnabled_local

                    self.csv_init_thread = threading.Thread(target=f)
                    self.csv_init_thread.start()
                    if c == 'L':
                        self.logging_time_limited_started = True

                else:
                    self.csvfile.close()
                    print("\n Stopped self.logging data to " + self.csvfilename)
                    self.loggingEnabled = loggingEnabled_local

                    self.logging_time_limited_started = False
                    self.logging_counter = 0

                if self.controller.controller_name == 'mppi':
                    if not self.loggingEnabled and self.controlled_iterations > 1:
                        self.controller.controller_report()

            ##### Control Mode #####
            elif c == 'u':  # toggle firmware control
                self.firmwareControl = not self.firmwareControl
                print("\nFirmware Control", self.firmwareControl)
                self.InterfaceInstance.control_mode(self.firmwareControl)
            elif c == 'k':
                # Reset Performance Buffers
                if self.controlEnabled is False:
                    self.switch_on_control()

                elif self.controlEnabled is True:
                    self.switch_off_control()
                print("\nself.controlEnabled= {0}".format(self.controlEnabled))

            elif c == ';':
                self.CartPoleInstance.target_equilibrium *= -1.0

            ##### Calibration #####
            elif c == 'K':
                global MOTOR, ANGLE_DEVIATION
                self.controlEnabled = False

                print("\nCalibrating motor position.... ")
                self.InterfaceInstance.calibrate()
                print("Done calibrating")

                if self.InterfaceInstance.encoderDirection == 1:
                    MOTOR = 'POLOLU'
                    if ANGLE_HANGING_DEFAULT:
                        ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_POLOLU)
                elif self.InterfaceInstance.encoderDirection == -1:
                    MOTOR = 'ORIGINAL'
                    if ANGLE_HANGING_DEFAULT:
                        ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_ORIGINAL)
                else:
                    raise ValueError('Unexpected value for self.InterfaceInstance.encoderDirection = '.format(self.InterfaceInstance.encoderDirection))
                print('Detected motor: {}'.format(MOTOR))

                self.InterfaceInstance.set_config_control(controlLoopPeriodMs=CONTROL_PERIOD_MS,
                                                          controlSync=CONTROL_SYNC,
                                                          angle_deviation=ANGLE_DEVIATION, avgLen=ANGLE_AVG_LENGTH, correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS)


            ##### Artificial Latency  #####
            elif c == 'b':
                measured_angles = []
                number_of_measurements = 1000
                time_measurement_start = time.time()
                print('Started angle measurement.')
                for _ in trange(number_of_measurements):
                    (angle, _, _, _, _, _, _, _, _,) = self.InterfaceInstance.read_state()
                    measured_angles.append(float(angle))
                time_measurement = time.time()-time_measurement_start

                angle_average = np.mean(measured_angles)
                angle_std = np.std(measured_angles)

                angle_rad = wrap_angle_rad((self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - self.angle_deviation_finetune)
                angle_std_rad = angle_std*ANGLE_NORMALIZATION_FACTOR
                print('\nAverage angle of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                                              angle_rad,
                                                                                              angle_average))
                print('\nAngle std of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                                              angle_std_rad,
                                                                                              angle_std))
                print('\nMeasurement took {} s'.format(time_measurement))
                # if abs(angle_rad) > 1.0:
                #     ANGLE_DEVIATION[...] = angle_constants_update(angle_average)
                #     ANGLE_HANGING_DEFAULT = False
                # self.InterfaceInstance.set_config_control(controlLoopPeriodMs=CONTROL_PERIOD_MS,
                #                                           controlSync=CONTROL_SYNC, controlLatencyUs=0,
                #                                           angle_deviation=ANGLE_DEVIATION, avgLen=ANGLE_AVG_LENGTH, correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS)

            # Fine tune angle deviation
            elif c == '=':
                self.angle_deviation_finetune += 0.002
                print("\nIncreased angle deviation fine tune value to {0}".format(self.angle_deviation_finetune))
            # Decrease Target Angle
            elif c == '-':
                self.angle_deviation_finetune -= 0.002
                print("\nDecreased angle deviation fine tune value to {0}".format(self.angle_deviation_finetune))

            ##### Target Position #####
            # Increase Target Position
            elif c == ']':
                self.base_target_position += 10 * POSITION_NORMALIZATION_FACTOR
                if self.base_target_position > 0.8 * (POSITION_ENCODER_RANGE // 2):
                    self.base_target_position = 0.8 * (POSITION_ENCODER_RANGE // 2)
                print("\nIncreased target position to {0} cm".format(self.base_target_position * 100))
            # Decrease Target Position
            elif c == '[':
                self.base_target_position -= 10 * POSITION_NORMALIZATION_FACTOR
                if self.base_target_position < -0.8 * (POSITION_ENCODER_RANGE // 2):
                    self.base_target_position = -0.8 * (POSITION_ENCODER_RANGE // 2)
                print("\nDecreased target position to {0} cm".format(self.base_target_position * 100))

            ##### Measurement Mode #####
            elif c == 'm':
                if self.current_experiment_protocol.is_running():
                    self.current_experiment_protocol.stop()
                self.current_experiment_protocol = self.experiment_protocols_manager.get_next_experiment_protocol()

            elif c == 'n':
                if self.current_experiment_protocol.is_idle():
                    self.current_experiment_protocol.start()
                else:
                    self.current_experiment_protocol.stop()
                    self.Q = 0.0
                    self.InterfaceInstance.set_motor(0)

            elif c == 'N':
                self.controlEnabled = False
                if self.current_experiment_protocol.is_running():
                    self.current_experiment_protocol.stop()
                self.InterfaceInstance.run_hardware_experiment()



            ##### Joystick  #####
            elif c == 'j':
                if self.joystickMode is None:
                    self.stick, self.joystickMode = setup_joystick()
                    self.log.warning('no joystick')
                elif self.joystickMode == 'not active':
                    self.joystickMode = 'speed'
                    self.log.info(f'set joystick to cart {self.joystickMode} control mode')
                elif self.joystickMode == 'speed':
                    self.joystickMode = 'position'
                    self.log.info(f'set joystick to cart {self.joystickMode} control mode')
                elif self.joystickMode == 'position':
                    self.joystickMode = 'not active'
                    self.log.info(f'set joystick to {self.joystickMode} mode')

            ##### Artificial Latency  #####
            elif c == '90':
                self.additional_latency += 0.002
                print('\nAdditional latency set now to {:.1f} ms'.format(self.additional_latency*1000))
                self.LatencyAdderInstance.set_latency(self.additional_latency)
            elif c == '0':
                self.additional_latency -= 0.002
                if self.additional_latency < 0.0:
                    self.additional_latency = 0.0
                print('\nAdditional latency set now to {:.1f} ms'.format(self.additional_latency * 1000))
                self.LatencyAdderInstance.set_latency(self.additional_latency)

            elif c == '5':
                subprocess.call(["python", "DataAnalysis/state_analysis.py"])


            ##### Live Plot #####
            elif c == '6':
                self.livePlotEnabled = not self.livePlotEnabled
                self.livePlotReset = True
                print(f'\nLive Plot Enabled: {self.livePlotEnabled}')
            elif c == '7':
                if hasattr(self, 'live_connection'):
                    self.live_connection.send('save')
            elif c == '8':
                if hasattr(self, 'live_connection'):
                    self.live_connection.send('reset')
            elif c == '9':
                global LIVE_PLOT_UNITS
                LIVE_PLOT_UNITS = 'raw' if LIVE_PLOT_UNITS=='metric' else 'metric'
                if hasattr(self, 'live_connection'):
                    self.live_connection.send(LIVE_PLOT_UNITS)
                    self.live_connection.send('reset')

            elif c == 'h' or c == '?':
                self.controller.print_help()

            ##### Exit ######
            elif ord(c) == 27:  # ESC
                self.log.info("\nquitting....")
                self.terminate_experiment = True

    def switch_off_control(self):
        print('off')
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
        print('on')
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
        ADC_RANGE = 4096
        if angle >= ADC_RANGE / 2:
            return angle - ADC_RANGE
        elif angle <= -ADC_RANGE / 2:
            return angle + ADC_RANGE
        else:
            return angle

    def get_state_and_time_measurement(self):
        # This function will block at the rate of the control loop
        (self.angle_raw, self.position_raw, self.target_position_from_chip, self.command, self.invalid_steps, self.time_difference, self.time_of_measurement, self.firmware_latency, self.latency_violation) = self.InterfaceInstance.read_state()

        self.treat_deadangle_with_derivative()  # Moved to hardware

        self.filter_differences()

        angle, position, angle_difference, position_difference = self.convert_angle_and_position_skale()

        self.time_measurement()

        self.check_latency_violation()

        angleDerivative, positionDerivative = self.calculate_first_derivatives(angle_difference, position_difference)

        # Pack the state into interface acceptable for the self.controller
        self.pack_features_into_state_variable(position, angle, positionDerivative, angleDerivative)

        self.add_latency()

    def treat_deadangle_with_derivative(self):

        ADC_RANGE = 4096

        # Calculate the index for the k-th past angle
        kth_past_index = (self.idx_for_derivative_calculation - self.derivative_timestep_in_samples + self.buffer_size_for_derivative_calculation) % self.buffer_size_for_derivative_calculation
        kth_past_angle = self.angle_history[kth_past_index]
        kth_past_position = self.position_history[kth_past_index]
        kth_past_frozen = self.frozen_history[kth_past_index]

        if kth_past_angle != -1 and (
            (self.invalid_steps > 5 and abs(self.wrap_local(kth_past_angle)) < ADC_RANGE / 20) or
            (abs(self.wrap_local(self.angle_raw - kth_past_angle)) > ADC_RANGE / 8 and kth_past_frozen < 3)
        ):
            self.frozen += 1
            self.angle_raw = self.angle_raw_stable if self.angle_raw_stable is not None else 0
            self.angleD_raw = self.angleD_raw_stable if self.angleD_raw_stable is not None else 0
        else:
            self.angleD_raw = self.wrap_local(self.angle_raw - kth_past_angle) / ((self.derivative_timestep_in_samples - 1) + kth_past_frozen + 1) if kth_past_angle != -1 else 0
            self.angle_raw_stable = self.angle_raw
            self.angleD_raw_stable = self.angleD_raw
            self.frozen = 0

        self.positionD_raw = (self.position_raw - kth_past_position) / self.derivative_timestep_in_samples if kth_past_position != -1 else 0

        self.angle_raw_sensor = self.angle_raw
        self.angleD_raw_sensor = self.angleD_raw

        # Save current angle in the history buffer and update index
        self.angle_history[self.idx_for_derivative_calculation] = self.angle_raw
        self.position_history[self.idx_for_derivative_calculation] = self.position_raw
        self.frozen_history[self.idx_for_derivative_calculation] = self.frozen
        self.idx_for_derivative_calculation = (self.idx_for_derivative_calculation + 1) % self.buffer_size_for_derivative_calculation  # Move to next index, wrap around if necessary


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
            if self.danceEnabled:
                if not self.dance_finishing:
                    self.dance_current_relative_position = self.danceAmpl * np.sin(
                        2 * np.pi * ((self.timeNow - self.dance_start_time) / self.dancePeriodS))
                    self.target_position = self.base_target_position + self.dance_current_relative_position
                else:
                    if abs(self.base_target_position-self.target_position) < 0.03:
                        self.danceEnabled = False
                        print(f"\nself.danceEnabled= {self.danceEnabled}")
                        self.dance_finishing = False
                        self.target_position = self.base_target_position
                    else:
                        self.target_position = 0.995 * self.target_position + 0.005 * self.base_target_position

        if SEND_CHANGE_IN_TARGET_POSITION_ALWAYS or self.firmwareControl:
            if self.target_position != self.target_position_previous:
                self.InterfaceInstance.set_target_position(self.target_position)
                self.target_position_previous = self.target_position

            if self.CartPoleInstance.target_equilibrium != self.target_equilibrium_previous:
                self.InterfaceInstance.set_target_equilibrium(self.CartPoleInstance.target_equilibrium)
                self.target_equilibrium_previous = self.CartPoleInstance.target_equilibrium
        
        self.CartPoleInstance.target_position = self.target_position

    def joystick_action(self):

        if self.joystickMode is None or self.joystickMode == 'not active':
            self.stickPos = 0.0
            if self.stickControl and not self.manualMotorSetting:
                if self.controlEnabled:
                    ...
                else:
                    self.Q = 0.0
            self.stickControl = False

        else:
            self.stickPos = get_stick_position(self.stick)
            self.stickControl = True
            self.Q = motorCmd_from_joystick(self.joystickMode, self.stickPos, self.s[POSITION_IDX])

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
            if MOTOR == 'POLOLU':
                motor_correction = MOTOR_CORRECTION_POLOLU
            else:
                motor_correction = MOTOR_CORRECTION_ORIGINAL

            self.actualMotorCmd *= motor_correction[0]
            if self.actualMotorCmd != 0:
                if np.sign(self.s[POSITIOND_IDX]) > 0:
                    self.actualMotorCmd += motor_correction[1]
                elif np.sign(self.s[POSITIOND_IDX]) < 0:
                    self.actualMotorCmd -= motor_correction[2]

        else:
            self.actualMotorCmd *= MOTOR_FULL_SCALE  # Scaling to motor units
            pass

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
                self.new_console_output = 1

                if hasattr(self.controller, 'controller_report') and self.controlled_iterations > 1:
                    self.controller.controller_report()
                if hasattr(self.controller, 'controller_reset'):
                    self.controller.controller_reset()
                self.danceEnabled = False
                self.target_position = self.base_target_position
                self.actualMotorCmd = 0
        else:
            self.safety_switch_counter = 0
            pass

    def write_csv_row(self):
        self.logging_counter += 1
        if self.actualMotorCmd_prev is not None and self.Q_prev is not None:
            if self.controller.controller_name == 'pid':
                self.csvwriter.writerow(
                    [self.elapsedTime, self.delta_time * 1000, self.angle_raw, self.angleD_raw, self.s[ANGLE_IDX], self.s[ANGLED_IDX],
                     self.s[ANGLE_COS_IDX], self.s[ANGLE_SIN_IDX], self.position_raw,
                     self.s[POSITION_IDX], self.s[POSITIOND_IDX], self.controller.ANGLE_TARGET, self.controller.angle_error,
                     self.target_position, self.controller.position_error, self.controller.Q_angle,
                     self.controller.Q_position, self.actualMotorCmd_prev, self.Q_prev,
                     self.stickControl, self.stickPos, self.current_experiment_protocol, self.s[ANGLE_IDX] ** 2, (self.s[POSITION_IDX] - self.target_position) ** 2, self.Q_prev ** 2,
                     self.time_difference, self.firmware_latency, self.latency_violation, self.python_latency, self.controller_steptime_previous, self.additional_latency, self.invalid_steps, self.frozen, self.fitted, self.angle_raw_sensor, self.angleD_raw_sensor, self.angleD_fitted])
            else:
                self.csvwriter.writerow(
                    [self.elapsedTime, self.delta_time * 1000, self.angle_raw, self.angleD_raw, self.s[ANGLE_IDX], self.s[ANGLED_IDX],
                     self.s[ANGLE_COS_IDX], self.s[ANGLE_SIN_IDX], self.position_raw,
                     self.s[POSITION_IDX], self.s[POSITIOND_IDX], 'NA', 'NA',
                     self.target_position, self.CartPoleInstance.target_equilibrium, 'NA', 'NA', 'NA', self.actualMotorCmd_prev, self.Q_prev,
                     self.stickControl, self.stickPos, self.current_experiment_protocol, self.s[ANGLE_IDX] ** 2, (self.s[POSITION_IDX] - self.target_position) ** 2, self.Q_prev ** 2,
                     self.time_difference, self.firmware_latency, self.latency_violation, self.python_latency, self.controller_steptime_previous, self.additional_latency, self.invalid_steps, self.frozen, self.fitted, self.angle_raw_sensor, self.angleD_raw_sensor, self.angleD_fitted])

        self.actualMotorCmd_prev = self.actualMotorCmd
        self.Q_prev = self.Q

        if self.logging_time_limited_started and self.logging_counter == self.logging_time_limited_max:
            self.csvfile.close()
            print("\n Stopped self.logging data to " + self.csvfilename)
            self.loggingEnabled = False

            self.logging_time_limited_started = False
            self.logging_counter = 0

    def plot_live(self):
        BUFFER_LENGTH = 5
        BUFFER_WIDTH = 7

        if not hasattr(self, 'livePlotReset') or self.livePlotReset:
            self.livePlotReset = False
            self.live_buffer_index = 0
            self.live_buffer = np.zeros((BUFFER_LENGTH, BUFFER_WIDTH))

            if not hasattr(self, 'live_connection'):
                address = ('localhost', 6000)
                self.live_connection = Client(address)
                self.live_connection.send(LIVE_PLOT_UNITS)
            else:
                self.live_connection.send(LIVE_PLOT_UNITS)

        if hasattr(self, 'live_connection'):
            if self.live_buffer_index < BUFFER_LENGTH:
                if LIVE_PLOT_UNITS == 'raw':
                    self.live_buffer[self.live_buffer_index, :] = np.array([
                        self.time_difference,
                        self.angle_raw,
                        self.angleD_raw,
                        self.position_raw,
                        self.s[POSITIOND_IDX] * 100,
                        self.actualMotorCmd,
                        self.frozen,
                    ])
                else:
                    self.live_buffer[self.live_buffer_index, :] = np.array([
                        self.time_difference,
                        self.s[ANGLE_IDX],
                        self.s[ANGLED_IDX],
                        self.s[POSITION_IDX] * 100,
                        self.s[POSITIOND_IDX] * 100,
                        self.Q,
                        self.frozen,
                    ])
                self.live_buffer_index += 1
            else:
                #print(self.live_buffer)
                self.live_connection.send(self.live_buffer)
                self.live_buffer_index = 0
                self.live_buffer = np.zeros((BUFFER_LENGTH, BUFFER_WIDTH))

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

            if True and not self.new_console_output:
                print('\033[5A\033[K', end='')
            self.new_console_output = False

            print('\r\033[K')

            ############  Mode  ############
            if self.controlEnabled:
                if 'mpc' in CONTROLLER_NAME:
                    mode='CONTROLLER:   {} (Period={}ms, Synch={}, Horizon={}, Rollouts={}, Predictor={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC, self.controller.optimizer.mpc_horizon, self.controller.optimizer.num_rollouts, self.controller.predictor.predictor_name)
                else:
                    mode='CONTROLLER:   {} (Period={}ms, Synch={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC)
            else:
                mode = 'CONTROLLER:   Firmware'
            print("\r" + mode +  '\033[K')

            ############  Mode  ############
            print("\r" + f'MEASUREMENT: {self.current_experiment_protocol}' +  '\033[K')

            ############  State  ############
            print("\rSTATE:  angle:{:+.3f}rad, angle raw:{:04}, position:{:+.2f}cm, position raw:{:04}, target:{}, Q:{:+.2f}, command:{:+05d}, invalid_steps:{}, frozen:{}\033[K"
                .format(
                    self.s[ANGLE_IDX],
                    self.angle_raw,
                    self.s[POSITION_IDX] * 100,
                    self.position_raw,
                    f"{self.CartPoleInstance.target_position}, {self.CartPoleInstance.target_equilibrium}",
                    self.Q,
                    self.actualMotorCmd,
                    self.invalid_steps,
                    self.frozen
                )
            )

            ############  Timing  ############
            if self.total_iterations > 10 and self.controlled_iterations > 10:
                print("\rTIMING: delta time [μ={:.1f}ms, σ={:.2f}ms], firmware latency [μ={:.1f}ms, σ={:.2f}ms], python latency [μ={:.1f}ms σ={:.2f}ms], controller step [μ={:.1f}ms σ={:.2f}ms], latency violations: {:}/{:} = {:.1f}%\033[K"
                    .format(
                        self.delta_time_buffer.mean() * 1000,
                        self.delta_time_buffer.std() * 1000,

                        self.firmware_latency_buffer.mean() * 1000,
                        self.firmware_latency_buffer.std() * 1000,

                        self.python_latency_buffer.mean() * 1000,
                        self.python_latency_buffer.std() * 1000,

                        self.controller_steptime_buffer.mean() * 1000,
                        self.controller_steptime_buffer.std() * 1000,

                        self.latency_violations,
                        self.total_iterations,
                        100 * self.latency_violations / self.total_iterations if self.total_iterations > 0 else 0
                    )
                )
            else:
                print('\033[K')

            ############  Performance  ############
            #if self.total_iterations > 10:
            #    np.set_printoptions(edgeitems=30, linewidth=200, formatter=dict(float=lambda x: "%.2f" % x))
            #    print("\rPERFORMANCE: μ="+str((performance_measurement_buffer.mean(axis=1)*1000) if performance_measurement_buffer.shape[1] > 1 else '')+"\033[K")
            #else:
            #    print('')

            ############  Cost  ############
            #global gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc
            #print("\rCOST: dd:{}, ep:{}, ekp:{}, ekc:{}, cc:{}, ccrc:{}\033[K".format(
            #    gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc
            #))

