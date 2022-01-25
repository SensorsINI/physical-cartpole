# TODO Aftrer joystick is unplugged and plugged again it interferes with the calibration, it causes the motor to get stuck at some speed after calibration. Add this to the readme file to warn the user.
# TODO: You can easily switch between controllers in runtime using this and get_available_controller_names function
# todo check if position unit conversion works for the following features: dance mode (can be checked for a nice self.controller only)
import math
import time
import numpy as np

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame.joystick as joystick  # https://www.pygame.org/docs/ref/joystick.html

from DriverFunctions.custom_logging import my_logger
from DriverFunctions.custom_serial_functions import setup_serial_connection
from DriverFunctions.interface import Interface
from DriverFunctions.kbhit import KBHit
from DriverFunctions.measure import StepResponseMeasurement
from DriverFunctions.utilities import terminal_check
from DriverFunctions.joystick import setup_joystick, get_stick_position, motorCmd_from_joystick

from CartPole.state_utilities import create_cartpole_state
from CartPole.state_utilities import ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, POSITION_IDX, POSITIOND_IDX
from CartPole._CartPole_mathematical_helpers import wrap_angle_rad

from DriverFunctions.controllers_management import set_controller
from DriverFunctions.firmware_parameters import set_firmware_parameters
from DriverFunctions.csv_helpers import csv_init

from globals import *

import subprocess, multiprocessing, platform
from multiprocessing.connection import Client
from CartPole.latency_adder import LatencyAdder
import sys

import tensorflow as tf

class PhysicalCartPoleDriver:
    def __init__(self):

        self.InterfaceInstance = Interface()

        self.controller, _, _ = set_controller(controller_name=CONTROLLER_NAME)

        self.log = my_logger(__name__)

        # Filters
        self.angle_smoothing = 1

        # Joystick variables
        self.stick = None
        self.joystickMode = None

        self.printCount = 0
        self.printCount2 = 0

        self.new_console_output = True

        self.controlEnabled = AUTOSTART
        self.firmwareControl = False
        self.manualMotorSetting = False

        self.danceEnabled = False
        self.danceAmpl = 0.10  # m
        self.dancePeriodS = 8.0
        self.dance_start_time = 0.0

        self.loggingEnabled = False
        self.livePlotEnabled = LIVE_PLOT

        try:
            self.kb = KBHit()  # can only use in posix terminal; cannot use from spyder ipython console for example
            self.kbAvailable = True
        except:
            self.kbAvailable = False

        self.startTime = None
        self.lastTime = None
        self.lastControlTime = None
        self.timeNow = None
        self.elapsedTime = None

        self.measurement = StepResponseMeasurement()

        self.csvfile = None
        self.csvfilename = None
        self.csvwriter = None

        self.Q = 0.0 # Motor command normed to be in a range -1 to 1
        self.Q_prev = None

        self.angle_raw = 0
        self.angle_raw_stable = None
        self.angle_raw_prev = None
        self.andleD_raw = 0
        self.angleD_raw_stable = None
        self.angleD_raw_prev = None
        self.invalid_steps = 0
        self.frozen = 0
        self.position_raw = 0

        self.anglePrev = 0.0
        self.positionPrev = None
        self.angleDPrev = 0.0
        self.positionDPrev = 0.0

        self.angleErr = 0.0
        self.positionErr = 0.0  # for printing even if not controlling

        self.position_offset = 0

        self.target_position = 0.0

        self.actualMotorCmd = 0
        self.actualMotorCmd_prev = None

        self.s = create_cartpole_state()

        self.terminate_experiment = False

        # Joystick variable
        self.stickPos = None
        self.stickControl = None

        self.command = 0
        self.sent = 0
        self.lastSent = None

        self.delta_time = 0
        self.delta_time_buffer = np.zeros((0))
        self.firmware_latency = 0
        self.firmware_latency_buffer = np.zeros((0))
        self.python_latency = 0
        self.python_latency_buffer = np.zeros((0))
        self.controller_steptime = 0
        self.controller_steptime_buffer = np.zeros((0))

        self.total_iterations = 0
        self.latency_violations = 0

        # Artificially adding latency
        self.additional_latency = 0.0
        self.LatencyAdderInstance = LatencyAdder(latency=self.additional_latency)
        self.s_delayed = np.copy(self.s)


        #options = tf.profiler.experimental.ProfilerOptions(host_tracer_level=3, python_tracer_level=1, device_tracer_level=1)
        #tf.profiler.experimental.start('tf_logs')

    def run(self):
        self.setup()
        self.run_experiment()
        self.quit_experiment()

    def setup(self):

        # check that we are running from terminal, otherwise we cannot control it
        terminal_check()

        self.InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)

        self.InterfaceInstance.control_mode(False)
        self.InterfaceInstance.stream_output(False)

        self.log.info('\n opened ' + str(SERIAL_PORT) + ' successfully')

        self.stick, self.joystickMode = setup_joystick()

        try:
            self.controller.loadparams()
        except AttributeError:
            print('loadparams not defined for this self.controller')

        time.sleep(1)

        #set_firmware_parameters(self.InterfaceInstance, ANGLE_AVG_LENGTH=ANGLE_AVG_LENGTH)
        self.InterfaceInstance.set_control_config(controlLoopPeriodMs=CONTROL_PERIOD_MS, controlSync=CONTROL_SYNC, controlLatencyUs=0)

        try:
            self.controller.printparams()
        except AttributeError:
            print('printparams not implemented for this self.controller.')

        self.controller.print_help()

        self.startTime = time.time()
        self.lastTime = self.startTime
        self.lastControlTime = self.startTime

        self.InterfaceInstance.stream_output(True)  # now start streaming state

    def run_experiment(self):

        while True:

            self.keyboard_input()

            self.get_state_and_time_measurement()

            self.set_target_position()

            if self.controlEnabled:
                self.lastControlTime = self.timeNow
                start = time.time()
                self.Q = self.controller.step(s=self.s, target_position=self.target_position, time=self.timeNow)
                performance_measurement[0] = time.time() - start
                self.controller_steptime = time.time() - start
                if AUTOSTART:
                    self.Q = 0
            else:
                # set values from firmware for logging
                self.lastControlTime = self.sent
                self.actualMotorCmd = self.command
                self.Q = self.command / MOTOR_FULL_SCALE


            self.joystick_action()

            self.measurement_action()

            if self.controlEnabled or not self.measurement.is_idle():
                self.get_motor_command()

            if self.measurement.is_idle() and self.controlEnabled:
                self.safety_switch_off()

            if self.controlEnabled or not self.measurement.is_idle():
                self.InterfaceInstance.set_motor(self.actualMotorCmd)

            # Plotting and Logging
            if self.loggingEnabled:
                self.write_csv_row()

            if self.livePlotEnabled:
                self.plot_live()

            self.write_current_data_to_terminal()

            if self.terminate_experiment:
                break

            self.end = time.time()
            self.python_latency = self.end - self.InterfaceInstance.start;

    def quit_experiment(self):
        # when x hit during loop or other loop exit
        self.InterfaceInstance.set_motor(0)  # turn off motor
        self.InterfaceInstance.close()
        joystick.quit()

        if self.loggingEnabled:
            self.csvfile.close()

    def keyboard_input(self):
        global POSITION_OFFSET, POSITION_TARGET, ANGLE_DEVIATION_FINETUNE, ANGLE_HANGING, ANGLE_DEVIATION, ANGLE_HANGING_DEFAULT
        if self.kbAvailable & self.kb.kbhit():
            self.new_console_output = True

            c = self.kb.getch()
            # Keys used in self.controller: 1,2,3,4,p, =, -, w, q, self.s, a, x, z, r, e, f, d, v, c, S, L, b, j
            try:
                self.controller.keyboard_input(c)
            except AttributeError:
                pass
            # FIXME ,./ commands not working as intended - control overwrites motor value
            if c == 'x':
                self.angle_smoothing = dec(self.angle_smoothing)
                if self.angle_smoothing > 1:
                    self.angle_smoothing = 1
                print("\nIncreased ANGLE_SMOOTHING {0}".format(self.angle_smoothing))
            elif c == 'z':
                self.angle_smoothing = inc(self.angle_smoothing)
                if self.angle_smoothing > 1:
                    self.angle_smoothing = 1
                print("\nDecreased ANGLE_SMOOTHING {0}".format(self.angle_smoothing))
            elif c == '.':  # zero motor
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
                    self.danceEnabled = False
                else:
                    self.dance_start_time = time.time()
                    self.danceEnabled = True
                print("\nself.danceEnabled= {0}".format(self.danceEnabled))
            elif c == 'l':
                self.loggingEnabled = not self.loggingEnabled
                print("\nself.loggingEnabled= {0}".format(self.loggingEnabled))
                if self.loggingEnabled:
                    self.csvfilename, self.csvfile, self.csvwriter = csv_init(controller_name = self.controller.controller_name)
                else:
                    self.csvfile.close()
                    print("\n Stopped self.logging data to " + self.csvfilename)

                if self.controller.controller_name == 'mppi':
                    if not self.loggingEnabled:
                        self.controller.controller_report()

            elif c == 'u':  # toggle firmware control
                self.firmwareControl = not self.firmwareControl
                print("\nFirmware Control", self.firmwareControl)
                self.InterfaceInstance.control_mode(self.firmwareControl)
            elif c == 'k':
                self.controlEnabled = not self.controlEnabled

                # Reset Performance Buffers
                self.delta_time_buffer = np.zeros((0))
                self.firmware_latency_buffer = np.zeros((0))
                self.python_latency_buffer = np.zeros((0))
                self.controller_steptime_buffer = np.zeros((0))
                global performance_measurement, performance_measurement_buffer
                performance_measurement_buffer = np.zeros((performance_measurement.size, 0))

                if self.controlEnabled is False:
                    self.Q = 0
                    self.InterfaceInstance.set_motor(0)
                    if self.controller.controller_name == 'mppi-tf':
                        self.controller.controller_report()
                    self.controller.controller_reset()
                self.danceEnabled = False
                print("\nself.controlEnabled= {0}".format(self.controlEnabled))
            elif c == 'K':
                global MOTOR, ANGLE_HANGING, ANGLE_DEVIATION
                self.controlEnabled = False

                print("\nCalibrating motor position.... ")
                self.InterfaceInstance.calibrate()
                (_, _, self.position_offset, _, _, _, _) = self.InterfaceInstance.read_state()
                print("Done calibrating")

                if self.InterfaceInstance.encoderDirection == 1:
                    MOTOR = 'POLOLU'
                    if ANGLE_HANGING_DEFAULT:
                        ANGLE_HANGING[...], ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_POLOLU)
                elif self.InterfaceInstance.encoderDirection == -1:
                    MOTOR = 'ORIGINAL'
                    if ANGLE_HANGING_DEFAULT:
                        ANGLE_HANGING[...], ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_ORIGINAL)
                else:
                    raise ValueError('Unexpected value for self.InterfaceInstance.encoderDirection = '.format(self.InterfaceInstance.encoderDirection))
                print('Detected motor: {}'.format(MOTOR))
            elif c == 'h' or c == '?':
                self.controller.print_help()
            # Fine tune angle deviation
            elif c == '=':
                ANGLE_DEVIATION_FINETUNE += 0.002
                print("\nIncreased angle deviation fine tune value to {0}".format(ANGLE_DEVIATION_FINETUNE))
            # Decrease Target Angle
            elif c == '-':
                ANGLE_DEVIATION_FINETUNE -= 0.002
                print("\nDecreased angle deviation fine tune value to {0}".format(ANGLE_DEVIATION_FINETUNE))

            # Increase Target Position
            elif c == ']':
                POSITION_TARGET += 10 * POSITION_NORMALIZATION_FACTOR
                if POSITION_TARGET > 0.8 * (POSITION_ENCODER_RANGE // 2):
                    POSITION_TARGET = 0.8 * (POSITION_ENCODER_RANGE // 2)
                print("\nIncreased target position to {0} cm".format(POSITION_TARGET * 100))
            # Decrease Target Position
            elif c == '[':
                POSITION_TARGET -= 10 * POSITION_NORMALIZATION_FACTOR
                if POSITION_TARGET < -0.8 * (POSITION_ENCODER_RANGE // 2):
                    POSITION_TARGET = -0.8 * (POSITION_ENCODER_RANGE // 2)
                print("\nDecreased target position to {0} cm".format(POSITION_TARGET * 100))
            elif c == 'm':  # toggle self.measurement
                if self.measurement.is_idle():
                    self.measurement.start()
                else:
                    self.measurement.stop()
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
            elif c == 'b':
                angle_average = 0
                number_of_measurements = 100
                for _ in range(number_of_measurements):
                    (angle, _, _, _, _, _, _) = self.InterfaceInstance.read_state()
                    angle_average += angle
                angle_average = angle_average / float(number_of_measurements)
                print('\nHanging angle average of {} measurements: {}     '.format(number_of_measurements, angle_average))
                ANGLE_HANGING[...], ANGLE_DEVIATION[...] = angle_constants_update(angle_average)
                ANGLE_HANGING_DEFAULT = False

            elif c == '5':
                subprocess.call(["python", "DataAnalysis/state_analysis.py"])

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

            # Exit
            elif ord(c) == 27:  # ESC
                self.log.info("\nquitting....")
                self.terminate_experiment = True

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
        (self.angle_raw, _, self.position_raw, self.command, self.invalid_steps, self.sent, self.firmware_latency) = self.InterfaceInstance.read_state()
        self.position_centered_unconverted = -(self.position_raw - self.position_offset)

        # Process Raw Angle

        if (self.invalid_steps > 5 and self.angle_raw_prev is not None and abs(self.wrap_local(self.angle_raw_prev)) < 200) or (self.angle_raw_prev is not None and abs(self.wrap_local(self.angle_raw - self.angle_raw_prev)) > 500):
            self.frozen += 1
            self.angle_raw = self.angle_raw_stable if self.angle_raw_stable is not None else 0
            self.angleD_raw = self.angleD_raw_stable if self.angleD_raw_stable is not None else 0
        else:
            self.angleD_raw = self.wrap_local(self.angle_raw - self.angle_raw_stable) / (self.frozen + 1) if self.angle_raw_stable is not None else 0
            self.angle_raw_stable = self.angle_raw
            self.angleD_raw_stable = self.angleD_raw
            self.frozen = 0

        self.angle_raw_prev = self.angle_raw
        self.angleD_raw_prev = self.angleD_raw

        # Convert position and angle to physical units
        angle = wrap_angle_rad((self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - ANGLE_DEVIATION_FINETUNE)
        position = self.position_centered_unconverted * POSITION_NORMALIZATION_FACTOR

        # Time self.measurement
        self.timeNow = time.time()
        self.lastTime = self.timeNow
        self.elapsedTime = self.timeNow - self.startTime

        if self.lastSent is not None:
            self.delta_time = self.sent - self.lastSent
        else:
            self.delta_time = 1e-6
        self.lastSent = self.sent

        # Latency Violations
        if self.firmware_latency > 1e-6:
            if self.firmware_latency > CONTROL_PERIOD_MS * 1e-3:
                self.latency_violations += 1

        # Calculating derivatives (cart velocity and angular velocity of the pole)
        angleDerivative = self.angleD_raw * ANGLE_NORMALIZATION_FACTOR / self.delta_time  # rad/self.s
        if self.positionPrev is not None:
            positionDerivative = (position - self.positionPrev) / self.delta_time  # m/self.s
        else:
            positionDerivative = 0

        # Keep values of angle and position for next timestep for derivative calculation
        self.anglePrev = angle
        self.positionPrev = position

        # Pack the state into interface acceptable for the self.controller
        self.s[POSITION_IDX] = position
        self.s[ANGLE_IDX] = angle
        self.s[POSITIOND_IDX] = positionDerivative
        self.s[ANGLED_IDX] = angleDerivative
        self.s[ANGLE_COS_IDX] = np.cos(self.s[ANGLE_IDX])
        self.s[ANGLE_SIN_IDX] = np.sin(self.s[ANGLE_IDX])

        self.LatencyAdderInstance.add_current_state_to_latency_buffer(self.s)
        self.s = self.LatencyAdderInstance.get_interpolated_delayed_state()

    def set_target_position(self):
        # Get the target position
        # if self.controlEnabled and self.danceEnabled:
        if self.danceEnabled:
            self.target_position = POSITION_TARGET + self.danceAmpl * np.sin(
                2 * np.pi * ((self.timeNow - self.dance_start_time) / self.dancePeriodS))
        else:
            self.target_position = POSITION_TARGET

    def joystick_action(self):

        if self.joystickMode is None or self.joystickMode == 'not active':
            self.stickPos = 0.0
            self.stickControl = False
            if not self.manualMotorSetting:
                if self.controlEnabled:
                    ...
                else:
                    self.Q = 0.0
        else:
            self.stickPos = get_stick_position(self.stick)
            self.stickControl = True
            self.Q = motorCmd_from_joystick(self.joystickMode, self.stickPos, self.s[POSITION_IDX])

    def measurement_action(self):
        if not self.measurement.is_idle():
            try:
                self.measurement.update_state(self.s[ANGLE_IDX], self.s[POSITION_IDX], self.timeNow)
                self.Q = self.measurement.Q
            except TimeoutError as e:
                self.log.warning(f'timeout in self.measurement: {e}')

    def get_motor_command(self):

        self.actualMotorCmd = self.Q
        if MOTOR_DYNAMICS_CORRECTED:

            self.actualMotorCmd = self.Q

            # Use Model_velocity_bidirectional.py to determine the margins and correction factor below

            # # We cut the region which is linear
            # # In fact you don't need - it it is already ensured that Q -1 to 1 corresponds to linear range
            # self.actualMotorCmd = 1.0 if self.actualMotorCmd > 1.0 else self.actualMotorCmd
            # self.actualMotorCmd = -1.0 if self.actualMotorCmd < -1.0 else self.actualMotorCmd

            # The change dependent on velocity sign is motivated theory of classical friction
            if MOTOR == 'POLOLU':
                self.actualMotorCmd *= 4307.69
                if self.actualMotorCmd != 0:
                    if np.sign(self.s[POSITIOND_IDX]) > 0:
                        self.actualMotorCmd += 398.69
                    elif np.sign(self.s[POSITIOND_IDX]) < 0:
                        self.actualMotorCmd -= 342.53
            else:
                self.actualMotorCmd *= 4916.29
                if self.actualMotorCmd != 0:
                    if np.sign(self.s[POSITIOND_IDX]) > 0:
                        self.actualMotorCmd += 266.77
                    elif np.sign(self.s[POSITIOND_IDX]) < 0:
                        self.actualMotorCmd -= 250.80


        else:
            self.actualMotorCmd *= MOTOR_FULL_SCALE  # Scaling to motor units
            pass

        # Convert to motor encoder units
        self.actualMotorCmd = int(self.actualMotorCmd)

        # Check if motor power in safe boundaries, not to burn it in case you have an error before or not-corrected option
        # NEVER RUN IT WITHOUT IT
        self.actualMotorCmd = MOTOR_FULL_SCALE_SAFE if self.actualMotorCmd > MOTOR_FULL_SCALE_SAFE else self.actualMotorCmd #Todo: use numpy clip
        self.actualMotorCmd = -MOTOR_FULL_SCALE_SAFE if self.actualMotorCmd < -MOTOR_FULL_SCALE_SAFE else self.actualMotorCmd

        self.actualMotorCmd = -self.actualMotorCmd

    def safety_switch_off(self):
        # Temporary safety switch off if goes to the boundary
        if abs(self.position_centered_unconverted) > 0.95 * (POSITION_ENCODER_RANGE // 2):
            print('\nSafety Switch.')
            self.controlEnabled = False
            self.InterfaceInstance.set_motor(0)
            self.new_console_output = 1
            if self.controller.controller_name == 'mppi-tf':
                self.controller.controller_report()
            self.controller.controller_reset()
            self.danceEnabled = False
            self.actualMotorCmd = 0
        else:
            pass

    def write_csv_row(self):
        if self.actualMotorCmd_prev is not None and self.Q_prev is not None:
            if self.controller.controller_name == 'PID':
                self.csvwriter.writerow(
                    [self.elapsedTime, self.delta_time * 1000, self.angle_raw, self.angleD_raw, self.s[ANGLE_IDX], self.s[ANGLED_IDX],
                     self.s[ANGLE_COS_IDX], self.s[ANGLE_SIN_IDX], self.position_raw,
                     self.s[POSITION_IDX], self.s[POSITIOND_IDX], self.controller.ANGLE_TARGET, self.controller.angle_error,
                     self.target_position, self.controller.position_error, self.controller.Q_angle,
                     self.controller.Q_position, self.actualMotorCmd_prev, self.Q_prev,
                     self.stickControl, self.stickPos, self.measurement, self.s[ANGLE_IDX] ** 2, (self.s[POSITION_IDX] - self.target_position) ** 2, self.Q_prev ** 2,
                     self.sent, self.firmware_latency, self.python_latency, self.additional_latency, self.invalid_steps, self.frozen])
            else:
                self.csvwriter.writerow(
                    [self.elapsedTime, self.delta_time * 1000, self.angle_raw, self.angleD_raw, self.s[ANGLE_IDX], self.s[ANGLED_IDX],
                     self.s[ANGLE_COS_IDX], self.s[ANGLE_SIN_IDX], self.position_raw,
                     self.s[POSITION_IDX], self.s[POSITIOND_IDX], 'NA', 'NA',
                     self.target_position, 'NA', 'NA', 'NA', self.actualMotorCmd_prev, self.Q_prev,
                     self.stickControl, self.stickPos, self.measurement, self.s[ANGLE_IDX] ** 2, (self.s[POSITION_IDX] - self.target_position) ** 2, self.Q_prev ** 2,
                     self.sent, self.firmware_latency, self.python_latency, self.additional_latency, self.invalid_steps, self.frozen])

        self.actualMotorCmd_prev = self.actualMotorCmd
        self.Q_prev = self.Q

    def plot_live(self):
        BUFFER_LENGTH = 2
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
                        self.sent,
                        self.angle_raw,
                        self.angleD_raw,
                        self.position_raw,
                        self.s[POSITIOND_IDX] * 100,
                        self.actualMotorCmd,
                        self.frozen,
                    ])
                else:
                    self.live_buffer[self.live_buffer_index, :] = np.array([
                        self.sent,
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
        if self.total_iterations > 2:
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

            if not self.new_console_output:
                print('\033[5A', end='')
            self.new_console_output = False

            print('\r')

            ############  Mode  ############
            if self.controlEnabled:
                if CONTROLLER_NAME=='mppi':
                    mode='MODE: mppi (Period={}ms, Synch={}, Horizon={}, Rollouts={})'.format(CONTROL_PERIOD_MS, CONTROL_SYNC, self.controller.mpc_samples, self.controller.num_rollouts)
                else:
                    mode='MODE: {} (Period={}ms, Synch={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC)
            else:
                mode = 'MODE: Firmware'
            print("\r" + mode +  '\033[K')

            ############  State  ############
            print("\rSTATE:  angle:{:+.3f}rad, angle raw:{:04}, position:{:+.2f}cm, position raw:{:04}, Q:{:+.2f}, command:{:+05d}, invalid_steps:{}\033[K"
                .format(
                    self.s[ANGLE_IDX],
                    self.angle_raw,
                    self.s[POSITION_IDX] * 100,
                    self.position_raw,
                    self.Q,
                    self.actualMotorCmd,
                    self.invalid_steps
                )
            )

            ############  Timing  ############
            if self.total_iterations > 2:
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
                print('')

            ############  Performance  ############
            if self.total_iterations > 1:
                np.set_printoptions(edgeitems=30, linewidth=200, formatter=dict(float=lambda x: "%.2f" % x))
                print("\rPERFORMANCE: μ="+str((performance_measurement_buffer.mean(axis=1)*1000) if performance_measurement_buffer.shape[1] > 1 else '')+"\033[K")
            else:
                print('')

            ############  Cost  ############
            #global gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc
            #print("\rCOST: dd:{}, ep:{}, ekp:{}, ekc:{}, cc:{}, ccrc:{}\033[K".format(
            #    gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc
            #))

