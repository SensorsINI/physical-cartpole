# TODO Aftrer joystick is unplugged and plugged again it interferes with the calibration, it causes the motor to get stuck at some speed after calibration. Add this to the readme file to warn the user.
# TODO: You can easily switch between controllers in runtime using this and get_available_controller_names function
# todo check if position unit conversion works for the following features: dance mode (can be checked for a nice self.controller only)
import time
import atexit
import sys

import os
from typing import Tuple

import numpy as np
from tqdm import trange

sys.path.extend(['Driver','Driver/CartPoleSimulation','Driver/CartPoleSimulation/CartPole','Driver/CartPoleSimulation/Control_Toolkit'])
from CartPole import CartPole
from Control_Toolkit.Controllers import template_controller, controller_mpc

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame.joystick as joystick  # https://www.pygame.org/docs/ref/joystick.html

from DriverFunctions.interface import Interface
from DriverFunctions.kbhit import KBHit
from DriverFunctions.step_response_measure import StepResponseMeasurement
from DriverFunctions.swing_up_measure import SwingUpMeasure
from DriverFunctions.random_target_measure import RandomTargetMeasure
from DriverFunctions.set_time_measure import SetTimeMeasure
from DriverFunctions.disturbance_measure import DisturbanceMeasure
from DriverFunctions.joystick import setup_joystick, get_stick_position, motorCmd_from_joystick

from CartPoleSimulation.CartPole.state_utilities import create_cartpole_state, ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, POSITION_IDX, POSITIOND_IDX
from CartPoleSimulation.CartPole._CartPole_mathematical_helpers import wrap_angle_rad
from CartPoleSimulation.CartPole.latency_adder import LatencyAdder

from DriverFunctions.csv_helpers import csv_init

from globals import *

import subprocess
from multiprocessing.connection import Client
import sys
import serial
from numba import jit
from DriverFunctions.numba_polyfit import fit_poly, eval_polynomial

from CartPoleSimulation.others.prefs import MyPreferences
prefs=MyPreferences()

import warnings
warnings.simplefilter('ignore', np.RankWarning)

from Control_Toolkit.others.get_logger import get_logger, CustomFormatter

log = get_logger(__name__) # just before loop starts in run_experiment we remove all existing handlers and add PhysicalCartpoleLoggingFormatter as a new formatter for a new handler


@jit(nopython=False, cache=True, fastmath=True)
def polyfit(buffer):
    p = fit_poly(np.arange(len(buffer)), buffer, 2)
    return eval_polynomial(p, len(buffer))
    #p = np.polyfit(x=np.arange(len(buffer)), y=buffer, deg=2)
    #return np.polyval(p=p, x=len(buffer))

class PhysicalCartPoleDriver:
    # set to singleton physical cartpole instance to detect if we are running physical cartpole; see CartPoleMainWindow for example of such static globals
    PhysicalCartPoleDriverInstance = None

    def __init__(self, CartPoleInstance:CartPole):
        PhysicalCartPoleDriver.PhysicalCartPoleDriverInstance=self # (tobi) used to access the (hopefully) singleton instance; see cartpole_dancer.py
        self.CartPoleInstance = CartPoleInstance
        self.CartPoleInstance.set_optimizer(optimizer_name=OPTIMIZER_NAME)
        self.CartPoleInstance.set_controller(controller_name=CONTROLLER_NAME)
        self.controller:controller_mpc = self.CartPoleInstance.controller
        self.print_keyboard_help()

        self.InterfaceInstance = Interface()


        # Console Printing
        self.printCount = 0
        self.new_console_output = True
        """ set new_console_output True before printing anytime you want to show some console output during runtime"""

        self.controlEnabled = AUTOSTART
        self.firmwareControl = False
        self.manualMotorSetting = False
        self.terminate_experiment = False
        self.returnToCenter = False # flag for returning cart to center after cart safety off on hitting edge of track
        self.wasControlEnabledBeforeReturnToCenter = False # flag to resume control after return to center

        self.safety_switch_counter = 0
        self.loop_counter=0

        # CSV Logging
        self.loggingEnabled = False
        self.csvfile = None
        self.csvfilename = None
        self.csvwriter = None

        # Live Plot
        self.livePlotEnabled = LIVE_PLOT

        try:
            self.kb = KBHit()  # can only use in posix terminal; cannot use from spyder ipython console for example
            self.kbAvailable = True
        except:
            self.kbAvailable = False

        # Dance Mode
        self.danceEnabled = False
        self.danceAmpl = 0.14  # m
        self.dancePeriodS = 2.0
        self.dance_start_time = 0.0

        # Measurement
        self.step_response_measure = StepResponseMeasurement()
        self.swing_up_measure = SwingUpMeasure(self)
        self.random_target_measure = RandomTargetMeasure(self)
        self.set_time_measure = SetTimeMeasure(self)
        self.disturbance_measure = DisturbanceMeasure(self)

        # self.current_measure = self.disturbance_measure
        # self.current_measure = self.set_time_measure
        # self.current_measure = self.swing_up_measure
        self.current_measure = self.random_target_measure

        # Motor Commands
        self.Q = 0.0 # Motor command normed to be in a range -1 to 1, reset to zero here
        self.Q_prev = None # previous motor command
        self.actualMotorCmd = 0 # the raw motor command as int
        self.actualMotorCmd_prev = None # previous raw motor command
        self.command = 0

        # State
        self.s = create_cartpole_state() # the cartpole state
        self.predicted_next_state=None # the predicted next state, to measure model mismatch
        self.traj_next_state=None # the target next state from cartpole_trajectory_generator, to measure model mismatch
        self.angle_raw = 0
        self.angle_raw_stable = None
        self.angle_raw_prev = None
        self.andleD_raw = 0
        self.angleD_raw_stable = None
        self.angleD_raw_prev = None
        self.angleD_raw_buffer = np.zeros((0))
        self.angle_raw_sensor = None
        self.angleD_raw_sensor = None
        self.angle_raw_sensor_prev = None
        self.angleD_fitted = None
        self.invalid_steps = 0
        self.frozen = 0
        self.fitted = 0
        self.position_raw = 0
        self.anglePrev = 0.0
        self.position_centered_unconverted = None # the cart position in motor units with center position subtracted
        self.positionPrev = None
        self.angleDPrev = 0.0
        self.positionDPrev = 0.0
        self.angleErr = 0.0
        self.positionErr = 0.0

        # Target
        self.position_offset = 0
        self.target_position = 0.0

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
        self.sent = 0 # time command sent in seconds
        self.lastSent = None # previous time command sent in seconds

        # Performance
        self.delta_time = 0
        self.delta_time_buffer = np.zeros(0)
        self.firmware_latency = 0
        self.firmware_latency_buffer = np.zeros(0)
        self.python_latency = 0
        self.python_latency_buffer = np.zeros(0)
        self.controller_steptime = 0
        self.controller_steptime_buffer = np.zeros(0)
        self.controlled_iterations = 0
        self.total_iterations = 0
        self.latency_violations = 0

        # Artificial Latency
        self.additional_latency = 0.0
        self.LatencyAdderInstance = LatencyAdder(latency=self.additional_latency, dt_sampling=0.005)
        self.s_delayed = np.copy(self.s)

        self.arduino_serial_port = serial.Serial(SERIAL_PORT, 115200, timeout=5)
        self.arduino_serial_port.write(b'1')

        self.time_last_switch = -np.inf

        self.demo_program = DEMO_PROGRAM

    def run(self):
        self.setup()
        self.run_experiment()
        self.quit_experiment()

    def setup(self):
        # Check that we are running from terminal, otherwise we cannot control it
        if not sys.stdin.isatty():
            log.error("Run from an interactive terminal to allow keyboard input (in pycharm, select 'Emulate terminal output' in run settings).")
            self.quit_experiment()
            quit(1)

        self.InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)
        self.InterfaceInstance.control_mode(False)
        self.InterfaceInstance.stream_output(False)

        log.info('Opened ' + str(SERIAL_PORT) + ' successfully')

        self.stick, self.joystickMode = setup_joystick()

        try:
            self.controller.loadparams()
        except AttributeError:
            log.info('loadparams not defined for this self.controller')

        time.sleep(1)

        #set_firmware_parameters(self.InterfaceInstance, ANGLE_AVG_LENGTH=ANGLE_AVG_LENGTH)
        self.InterfaceInstance.set_control_config(controlLoopPeriodMs=CONTROL_PERIOD_MS, controlSync=CONTROL_SYNC, controlLatencyUs=0)

        try:
            self.controller.printparams()
        except AttributeError:
            log.info('printparams not implemented for this self.controller.')


        self.startTime = time.time()
        self.lastTime = 0 # set to zero at start of execution
        self.lastControlTime = self.startTime

        last_calibration_time=prefs.get('last_calibration_time',0)
        if last_calibration_time==0 or self.startTime-last_calibration_time>6*3600:

            if last_calibration_time==0:
                    log.warning(f'the cart position has never been calibrated, use \'K\' to perform cart calibration to find center position')
                    time.sleep(2.)
            else:
                dthours=(self.startTime - last_calibration_time)/3600.
                if dthours>3:
                    log.warning(f'The cart position has not been calibrated in {dthours:.1f} hours; use \'K\' to perform cart calibration to find center position')
                    time.sleep(2.)

        self.InterfaceInstance.stream_output(True)  # now start streaming state
        atexit.register(self.switch_off_motor)

    def run_experiment(self):

        # remove all handlers and then add our special handler to log to console
        log.handlers.clear()
        ch = logging.StreamHandler()
        ch.setFormatter(PhysicalCartpoleLoggingFormatter())
        log.addHandler(ch)

        print('starting main loop now...')
        # see ansi escape codes https://en.wikipedia.org/wiki/ANSI_escape_code
        print('\033[2J') # clear whole screen and move to top left

        while not self.terminate_experiment:
            self.experiment_sequence()
        print("experiment terminated, wait ....")

    def experiment_sequence(self):

        self.loop_counter+=1

        # debug logger
        # if self.loop_counter%300==0:
        #     log.debug(f'loop counter = {self.loop_counter}') # TODO remove debug

        self.keyboard_input()
        # self.controlEnabled=True # DEBUG hack since debugger hangs on any keyboard input

        self.get_state_and_time_measurement()

        if self.demo_program and self.controlEnabled:
            self.danceEnabled = True
            if self.timeNow - self.time_last_switch > 8:
                self.time_last_switch = self.timeNow
                if self.CartPoleInstance.target_equilibrium == 1:
                    self.CartPoleInstance.target_equilibrium = -1
                else:
                    self.CartPoleInstance.target_equilibrium = 1

        self.set_target_position()

        if not self.manualMotorSetting:
            self.Q=0.0 # set motor to zero unless something below sets it

        if self.controlEnabled:
            # Active Python Control: set values from controller
            self.lastControlTime = self.timeNow
            start = time.time()
            self.Q = float(self.controller.step(self.s, self.timeNow, {"target_position": self.target_position,
                                                                       "target_equilibrium": self.CartPoleInstance.target_equilibrium}))
            # tobi: predict next state so that we can measure model mismatch
            next_states=self.controller.predictor_wrapper.predictor.predict(self.s, [self.Q], horizon=1) # Q must be a vector
            self.predicted_next_state = next_states[0,1,:]  # take 2nd step of rollout since first step is current state

            self.traj_next_state= self.controller.cartpole_trajectory_generator.traj[:, 0] # what the cartpole_trajectory_generator tries to get to

            performance_measurement[0] = time.time() - start
            self.controller_steptime = time.time() - start
            if AUTOSTART:
                self.Q = 0
            self.controlled_iterations += 1
        elif self.returnToCenter:
            (done,self.Q)=self.compute_Q_to_center_cart()
            if done: # returns True when done centering
                self.new_console_output=True
                log.info(f'done returning to center, turning off motor and starting control; pos={self.s[POSITION_IDX] * 100:.3f}cm')
                self.returnToCenter=False
                self.InterfaceInstance.set_motor(0)
                self.switch_on_control()
        else:
            # Observing Firmware Control: set values from firmware for logging
            # self.lastControlTime = self.sent
            # self.actualMotorCmd = self.command
            # self.Q = self.command / MOTOR_FULL_SCALE
            self.controlled_iterations = 0

        self.joystick_action()

        self.measurement_action()
        # log.debug(f'before actual motor command self.Q={self.Q:.3f}')
        if self.controlEnabled or self.current_measure.is_running() or self.manualMotorSetting or self.returnToCenter:
            self.actualMotorCmd=self.compute_raw_motor_command(self.Q)
        # log.debug(f'after actual motor command self.Q={self.Q:.3f}')

        if self.controlEnabled and self.current_measure.is_idle() :
            self.check_for_safety_switch_off()

        if self.controlEnabled or self.current_measure.is_running() or self.manualMotorSetting  or self.returnToCenter:
            self.InterfaceInstance.set_motor(self.actualMotorCmd)

        # if self.returnToCenter: #debug
        #     print(f'\ncentering: self.Q={self.Q:.2f}, self.actualMotorCmd={self.actualMotorCmd}')

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
        print('turning off motor and shutting down interfaces to cartpole and other peripherals')
        # when q/ESC hit during loop or other loop exit
        self.switch_off_motor()
        self.InterfaceInstance.close()
        joystick.quit()

        if self.loggingEnabled:
            self.csvfile.close()

    def keyboard_input(self):
        """ Checks for keyboard input keystroke and takes action on it."""
        global POSITION_OFFSET, POSITION_TARGET, ANGLE_DEVIATION_FINETUNE, ANGLE_HANGING, ANGLE_DEVIATION, ANGLE_HANGING_DEFAULT
        pos_target_step = 0.05  # meters
        pos_target_limit=0.8*TRACK_LENGTH/2

        if self.kbAvailable & self.kb.kbhit():
            self.new_console_output = True # to allow showing the command response

            c = self.kb.getch()
            try:
                # Keys used in PID controller: 1,2,3,4,p, =, -, w, q, s, a, x, z, r, e, f, d, v, c, S, L, b, j
                self.controller.keyboard_input(c)
            except AttributeError:
                pass

            ##### Manual Motor Movement #####
            if c == '.':  # zero motor
                self.controlEnabled = False
                self.Q = 0
                self.manualMotorSetting = True
                print('\nZeroed motor with "."')
            elif c == ',':  # left
                self.controlEnabled = False
                self.Q -= 0.05
                self.manualMotorSetting = True
                print('\nDecreased normed motor command with "," to', self.Q)
            elif c == '/':  # right
                self.controlEnabled = False
                self.Q += 0.05
                self.manualMotorSetting = True
                print('\nIncreased normed motor command with "/" to', self.Q)

            elif c=='C': # switch off control and return to center position mode with constant cart speed
                if not self.returnToCenter:
                    self.new_console_output=True
                    log.info('started returning to center...')
                    self.wasControlEnabledBeforeReturnToCenter=self.controlEnabled
                    self.controlEnabled=False
                    self.returnToCenter=True # will call self.center_cart() in control loop experiment_sequence()
                    self.manualMotorSetting=False
                else:
                    self.new_console_output=True
                    log.info('stopped returning to center')
                    self.controlEnabled = False
                    self.returnToCenter = False
                    self.manualMotorSetting = False
                    self.switch_off_control()


            elif c == 'D': # old 'dance' mode to move target position sinusoidally
                # We want the sinusoid to start at predictable (0) position
                if self.danceEnabled is True:
                    self.danceEnabled = False
                else:
                    self.dance_start_time = time.time()
                    self.danceEnabled = True
                print("\nself.danceEnabled= {0}".format(self.danceEnabled))

            ##### Logging #####
            elif c == 'l':
                self.loggingEnabled = not self.loggingEnabled
                print("\nself.loggingEnabled= {0}".format(self.loggingEnabled))
                if self.loggingEnabled:
                    self.csvfilename, self.csvfile, self.csvwriter = csv_init(controller_name = self.controller.controller_name)
                else:
                    self.csvfile.close()
                    print("\n Stopped self.logging data to " + self.csvfilename)

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
                log.info(f'inverting pole equilibrium setting')
                self.CartPoleInstance.target_equilibrium *= -1.0

            ##### Calibration #####
            elif c == 'K':
                global MOTOR, ANGLE_HANGING, ANGLE_DEVIATION
                self.controlEnabled = False

                print("\nCalibrating motor position.... ")
                self.InterfaceInstance.calibrate()
                (_, _, self.position_offset, _, _, _, _) = self.InterfaceInstance.read_state()
                print("Done calibrating")
                prefs.put('last_calibration_time', time.time())

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

            ##### Measure angle precisely  #####
            elif c == 'b':
                measured_angles = []
                number_of_measurements = 1000
                time_measurement_start = time.time()
                print('Started angle measurement.')
                for _ in trange(number_of_measurements):
                    (angle, _, _, _, _, _, _) = self.InterfaceInstance.read_state()
                    measured_angles.append(float(angle))
                time_measurement = time.time()-time_measurement_start

                angle_average = np.mean(measured_angles)
                angle_std = np.std(measured_angles)

                angle_rad = wrap_angle_rad((self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - ANGLE_DEVIATION_FINETUNE)
                angle_std_rad = angle_std*ANGLE_NORMALIZATION_FACTOR
                print('\nAverage angle of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                                              angle_rad,
                                                                                              angle_average))
                print('\nAngle std of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                                              angle_std_rad,
                                                                                              angle_std))
                print('\nMeasurement took {} s'.format(time_measurement))
                # if abs(angle_rad) > 1.0:
                #     ANGLE_HANGING[...], ANGLE_DEVIATION[...] = angle_constants_update(angle_average)
                #     ANGLE_HANGING_DEFAULT = False
            # Fine tune angle deviation
            elif c == '=':
                ANGLE_DEVIATION_FINETUNE += 0.002
                print(f'\nIncreased angle deviation fine tune value to {ANGLE_DEVIATION_FINETUNE:.3f}')
                prefs.put('ANGLE_DEVIATION_FINETUNE',ANGLE_DEVIATION_FINETUNE)
            # Decrease Target Angle
            elif c == '-':
                ANGLE_DEVIATION_FINETUNE -= 0.002
                print(f'\nDecreased angle deviation fine tune value to {ANGLE_DEVIATION_FINETUNE:.3f}')
                prefs.put('ANGLE_DEVIATION_FINETUNE',ANGLE_DEVIATION_FINETUNE)

            ##### Target Position #####
            # Increase Target Position of pole
            elif c == ']':
                POSITION_TARGET += pos_target_step
                if POSITION_TARGET>pos_target_limit: POSITION_TARGET=pos_target_limit
                print(f"\nIncreased target position to {POSITION_TARGET*100:.1f} cm")
            # Decrease Target Position
            elif c == '[':
                POSITION_TARGET -= pos_target_step
                if POSITION_TARGET < -pos_target_limit: POSITION_TARGET = -pos_target_limit
                print(f"\nDecreased target position to {POSITION_TARGET*100:.1f} cm")

            ##### Measurement Mode #####
            elif c == 'm':
                if self.current_measure is self.step_response_measure:
                    if self.current_measure.is_running():
                        print('Closing step_response_measure')
                        self.current_measure.stop()
                    print('Setting swing_up_measure')
                    self.current_measure = self.swing_up_measure

                elif self.current_measure is self.swing_up_measure:
                    if self.current_measure.is_running():
                        print('Closing swing_up_measure')
                        self.current_measure.stop()
                    print('Setting random_target_measure')
                    self.current_measure = self.random_target_measure

                elif self.current_measure is self.random_target_measure:
                    if self.current_measure.is_running():
                        print('Closing random_target_measure')
                        self.current_measure.stop()
                    print('Setting step_response_measure')
                    self.current_measure = self.step_response_measure
                else:
                    print('No recognized measure loaded. Setting step_response_measure')
                    self.current_measure = self.step_response_measure

            elif c == 'n':

                if self.current_measure.is_idle():
                     print('\nMeasurement started!\n')
                     self.current_measure.start()
                else:
                     self.current_measure.stop()
                     print('\nMeasurement stopped\n')

            ##### Joystick  #####
            elif c == 'j':
                if self.joystickMode is None:
                    self.stick, self.joystickMode = setup_joystick()
                    log.warning('no joystick')
                elif self.joystickMode == 'not active':
                    self.joystickMode = 'speed'
                    log.info(f'set joystick to cart {self.joystickMode} control mode')
                elif self.joystickMode == 'speed':
                    self.joystickMode = 'position'
                    log.info(f'set joystick to cart {self.joystickMode} control mode')
                elif self.joystickMode == 'position':
                    self.joystickMode = 'not active'
                    log.info(f'set joystick to {self.joystickMode} mode')

            ##### Artificial Latency  #####
            elif c == '9':
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
                print('\nstarting state analysis.py')
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
                self.print_keyboard_help()

            ##### Exit ######
            elif c=='q' or c=='x' or ord(c) == 27:  # ESC/x/q all quit
                print("\nquitting....")
                self.terminate_experiment = True
    def print_keyboard_help(self):
        """ Prints help message for keyboard commands to console"""
        print("\n\n\n\n***********************************")
        print("keystroke commands")
        print("q/x/ESC quit")
        print("k toggle control on/off (initially off)")
        print("K trigger motor position calibration")
        print("=/- increase/decrease (fine tune) angle offset value to obtain 0 when vertically")
        print("[] increase/decrease position target")
        print("; flip target pole equilibrium fron balance to hanging or vice versa; also affects other behaviors like spin")
        print("2/1 angle proportional gain")
        print("w/q angle integral gain")
        print("s/a angle derivative gain")
        print("z/x angle smoothing")
        print("4/3 position proportional gain")
        print("r/e position integral gain")
        print("f/d position derivative gain")
        print("p print PID parameters")
        print("l toggle logging data")
        print("S/L Save/Load param values from disk")
        print("D Toggle dance mode")
        print("C switch off control and center cart on track")
        print(",./ Turn on motor left zero right")
        print("m Toggle measurement")
        print("n Toggle motor current measurement") # TODO not sure
        print("j Switch joystick control mode")
        print("b Print averaged angle measurement from sensor")
        print("6 Enable/Disable live plot")
        print("5 Interrupts for histogram plot")
        print("6789 toggle/save/reset/units of live state plotting")
        print(" **** commands for specific controller")
        try:
            self.controller.print_keyboard_help() #e.g. cartpole_trajectory_generator
        except AttributeError:
            print(f'no print_keyboard_help() for controller "{self.controller}"')
            pass
        print("***********************************\n\n\n\n")

    def switch_off_motor(self):
        if self.InterfaceInstance:
            try:
                self.InterfaceInstance.set_motor(0)
                self.new_console_output=True
                log.info('switched off motor')
            except AttributeError:
                pass

    def switch_off_control(self):
        print('control turned off')
        self.controlEnabled = False
        self.Q = 0
        self.switch_off_motor()
        if self.controller.controller_name == 'mppi-tf':
            self.controller.controller_report()
        self.controller.controller_reset()
        self.danceEnabled = False

    def switch_on_control(self):
        print('control turned on')
        self.controlEnabled = True
        self.manualMotorSetting = False
        self.returnToCenter=False
        self.delta_time_buffer = np.zeros((0))
        self.firmware_latency_buffer = np.zeros((0))
        self.python_latency_buffer = np.zeros((0))
        self.controller_steptime_buffer = np.zeros((0))
        global performance_measurement, performance_measurement_buffer
        performance_measurement_buffer = np.zeros((performance_measurement.size, 0))

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
        (self.angle_raw, _, self.position_raw, self.command, self.invalid_steps, self.sent, self.firmware_latency) = self.InterfaceInstance.read_state()
        self.position_centered_unconverted = -(self.position_raw - self.position_offset)


        # Anomaly Detection: unstable buffer (invalid steps) or unstable angle_raw (jump in angle_raw), only inside region close to 0
        if (self.invalid_steps > 5 and self.angle_raw_prev is not None and abs(self.wrap_local(self.angle_raw_prev)) < 200) or (self.angle_raw_prev is not None and abs(self.wrap_local(self.angle_raw - self.angle_raw_prev)) > 500 and self.frozen < 3):
            self.frozen += 1
            self.angle_raw = self.angle_raw_stable if self.angle_raw_stable is not None else 0
            self.angleD_raw = self.angleD_raw_stable if self.angleD_raw_stable is not None else 0
        else:
            self.angleD_raw = self.wrap_local(self.angle_raw - self.angle_raw_stable) / (self.frozen + 1) if self.angle_raw_stable is not None else 0
            self.angle_raw_stable = self.angle_raw
            self.angleD_raw_stable = self.angleD_raw
            self.frozen = 0

        self.angle_raw_sensor = self.angle_raw
        self.angleD_raw_sensor = self.angleD_raw

        # Polyfit AngleD if filled buffer and inside Anomaly (close to 0 and small angleD_raw)
        if POLYFIT_ANGLED:
            MAX_BUFFER_LENGTH = 10 # 10 * 20ms = 200ms
            if len(self.angleD_raw_buffer) >= 2:
                self.angleD_fitted = polyfit(self.angleD_raw_buffer)
                if abs(self.wrap_local(self.angle_raw_prev)) < 200 and self.fitted < 3 and len(self.angleD_raw_buffer) >= MAX_BUFFER_LENGTH:
                    if abs(self.angleD_fitted) < 500 and (abs(self.wrap_local(self.angleD_fitted - self.angleD_raw)) > (20 + 12 * self.fitted) or abs(self.wrap_local(self.angle_raw_prev-self.angle_raw)) < 25 or abs(self.wrap_local(self.angleD_raw_prev-self.angleD_raw)) > 25):
                        self.fitted += 1
                        self.angleD_raw = self.angleD_fitted
                    else:
                        if self.fitted:
                            self.angleD_raw_buffer = np.zeros((0))
                        self.fitted = 0
                else:
                    if self.fitted:
                        self.angleD_raw_buffer = np.zeros((0))
                    self.fitted = 0
            else:
                self.fitted = 0

            self.angleD_raw_buffer = np.append(self.angleD_raw_buffer, self.angleD_raw)
            self.angleD_raw_buffer = self.angleD_raw_buffer[-(MAX_BUFFER_LENGTH+self.fitted):]

        # Save previous Values
        self.angle_raw_prev = self.angle_raw
        self.angleD_raw_prev = self.angleD_raw

        # Convert position and angle to physical units of meters and radians
        angle = wrap_angle_rad((self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - ANGLE_DEVIATION_FINETUNE)
        position = self.position_centered_unconverted * POSITION_NORMALIZATION_FACTOR

        # Time self.measurement
        self.timeNow = time.time()-self.startTime # make times relative to start to avoid large time numbers in controller
        self.lastTime = self.timeNow
        self.elapsedTime = self.timeNow - self.startTime

        if self.lastSent is not None:
            self.delta_time = self.sent - self.lastSent
        else:
            self.delta_time = 1e-6
        self.lastSent = self.sent

        # Latency Violations
        if self.delta_time > 1.1 * CONTROL_PERIOD_MS * 1e-3:
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
        if self.current_measure.is_running():
            try:
                self.target_position = self.current_measure.target_position
                # print(self.target_position)
            except AttributeError:
                pass
        else:
            if self.danceEnabled:
                self.target_position = POSITION_TARGET + self.danceAmpl * np.sin(
                    2 * np.pi * ((self.timeNow - self.dance_start_time) / self.dancePeriodS))
            else:
                self.target_position = POSITION_TARGET # tobi it was lowpass filtered for some reason, not clear why 0.995 * self.target_position + 0.005 * POSITION_TARGET
        
        self.CartPoleInstance.target_position = self.target_position

    def joystick_action(self):

        if self.joystickMode is None or self.joystickMode == 'not active':
            self.stickPos = 0.0
            self.stickControl = False
            # tobi commented out since it is not clear why the motor command should be set to zero if joystick is disalbed
            # if not self.manualMotorSetting:
            #     if self.controlEnabled:
            #         ...
            #     else:
            #         self.Q = 0.0
        else:
            self.stickPos = get_stick_position(self.stick)
            self.stickControl = True
            self.Q = motorCmd_from_joystick(self.joystickMode, self.stickPos, self.s[POSITION_IDX])

    def measurement_action(self):
        if self.current_measure.is_running():
            try:
                self.current_measure.update_state(self.s[ANGLE_IDX], self.s[POSITION_IDX], self.timeNow)
                self.Q = self.current_measure.Q
            except TimeoutError as e:
                log.warning(f'timeout in self.measurement: {e}')

    def compute_raw_motor_command(self, q:float)->int:
        """ Computes the actual raw motor command self.actualMotorCmd from the controller self.Q

        :param q: the motor command -1.0 to +1.0
        :returns: the raw motor command as int value
        """

        rawMotorCommand = q
        if MOTOR_DYNAMICS_CORRECTED:

            rawMotorCommand = q

            # Use Model_velocity_bidirectional.py to determine the margins and correction factor below

            # # We cut the region which is linear
            # # In fact you don't need - it is already ensured that Q -1 to 1 corresponds to linear range
            # rawMotorCommand = 1.0 if rawMotorCommand > 1.0 else rawMotorCommand
            # rawMotorCommand = -1.0 if rawMotorCommand < -1.0 else rawMotorCommand

            # The change dependent on velocity sign is motivated theory of classical friction
            if MOTOR == 'POLOLU':
                rawMotorCommand *= 3617.43
                if rawMotorCommand != 0:
                    if np.sign(self.s[POSITIOND_IDX]) > 0:
                        rawMotorCommand += 181.66
                    elif np.sign(self.s[POSITIOND_IDX]) < 0:
                        rawMotorCommand -= 312.98
            else:
                rawMotorCommand *= 4729.99
                if rawMotorCommand != 0:
                    if np.sign(self.s[POSITIOND_IDX]) > 0:
                        rawMotorCommand += 135.75
                    elif np.sign(self.s[POSITIOND_IDX]) < 0:
                        rawMotorCommand -= 170.90

        else:
            rawMotorCommand *= MOTOR_FULL_SCALE_SAFE  # Scaling to motor units
            pass

        # Convert to motor encoder units
        rawMotorCommand = int(rawMotorCommand)

        # Check if motor power in safe boundaries, not to burn it in case you have an error before or not-corrected option
        # NEVER RUN IT WITHOUT IT
        rawMotorCommand = MOTOR_FULL_SCALE_SAFE if rawMotorCommand > MOTOR_FULL_SCALE_SAFE else rawMotorCommand
        rawMotorCommand = -MOTOR_FULL_SCALE_SAFE if rawMotorCommand < -MOTOR_FULL_SCALE_SAFE else rawMotorCommand

        rawMotorCommand = -rawMotorCommand # todo why minus sign here?
        return rawMotorCommand

    def check_for_safety_switch_off(self):
        """Checks if cart is too close to boundary and switches off motor if so
        """
        if abs(self.position_centered_unconverted) >  ((0.95 *POSITION_ENCODER_RANGE) // 2): # // is floor division, results in int value closest to zero
            self.safety_switch_counter += 1
            if self.safety_switch_counter > 10:  # Allow short bumps
                self.safety_switch_counter = 0
                log.error(f'\nMotor disabled because more than 10 times abs(self.position_centered_unconverted)={abs(self.position_centered_unconverted)} >  ((.95* POSITION_ENCODER_RANGE) // 2)={((.95* POSITION_ENCODER_RANGE) // 2)}')
                self.controlEnabled = False
                self.manualMotorSetting=False
                self.InterfaceInstance.set_motor(0)
                self.new_console_output = 1

                if hasattr(self.controller, 'controller_report') and self.controlled_iterations > 1:
                    self.controller.controller_report()
                if hasattr(self.controller, 'controller_reset'):
                    log.debug('resetting controller')
                    self.controller.controller_reset()
                self.danceEnabled = False
                self.actualMotorCmd = 0
        else:
            self.safety_switch_counter = 0
            pass

    def compute_Q_to_center_cart(self)->Tuple[bool,float]:
        """
        Sets self.Q depending on condition of cart position to drive cart with constant motor speed toward center,
        until the position is close to zero. Returns True when cart is centered.

        :returns: True when centered, False if still moving
        """
        pos=self.s[POSITION_IDX]
        Q=0.0
        if(abs(pos)<.02):
            return (True,Q)
        motorPower=.3 # power to return to center
        Q=(-np.sign(pos)*motorPower) # minus to drive cart left when pos>0
        # log.debug(f'self.Q={self.Q:.3f}')
        return (False,Q)

    def write_csv_row(self):
        """ Writes one row of CSV data file. If new fields are added here, update csv_helper.py csv_init() method to add to header
        """

        def comsep(*values):
            s = ''
            for v in values:
                s = s + f'{v:.5f},'
            return s

        if self.actualMotorCmd_prev is not None and self.Q_prev is not None:
            if self.controller.controller_name == 'pid':
                self.csvwriter.writerow(
                    [self.elapsedTime, self.delta_time * 1000, self.angle_raw, self.angleD_raw, self.s[ANGLE_IDX], self.s[ANGLED_IDX],
                     self.s[ANGLE_COS_IDX], self.s[ANGLE_SIN_IDX], self.position_raw,
                     self.s[POSITION_IDX], self.s[POSITIOND_IDX], self.controller.ANGLE_TARGET, self.controller.angle_error,
                     self.target_position, self.controller.position_error, self.controller.Q_angle,
                     self.controller.Q_position, self.actualMotorCmd_prev, self.Q_prev,
                     self.stickControl, self.stickPos, self.step_response_measure, self.s[ANGLE_IDX] ** 2, (self.s[POSITION_IDX] - self.target_position) ** 2, self.Q_prev ** 2,
                     self.sent, self.firmware_latency, self.python_latency, self.controller_steptime, self.additional_latency, self.invalid_steps, self.frozen, self.fitted, self.angle_raw_sensor, self.angleD_raw_sensor, self.angleD_fitted])
            else:
                if not self.predicted_next_state is None and not self.traj_next_state is None:
                    self.csvwriter.writerow(
                        [self.elapsedTime, self.delta_time * 1000, self.angle_raw, self.angleD_raw,
                         self.s[ANGLE_IDX], self.s[ANGLED_IDX],
                         self.s[ANGLE_COS_IDX], self.s[ANGLE_SIN_IDX], self.position_raw,
                         self.s[POSITION_IDX], self.s[POSITIOND_IDX],
                         'NA', 'NA',
                         self.target_position, self.CartPoleInstance.target_equilibrium, 'NA', 'NA', 'NA', self.actualMotorCmd_prev, self.Q_prev,
                         self.stickControl, self.stickPos, self.step_response_measure, self.s[ANGLE_IDX] ** 2, (self.s[POSITION_IDX] - self.target_position) ** 2, self.Q_prev ** 2,
                         self.sent, self.firmware_latency, self.python_latency, self.controller_steptime, self.additional_latency, self.invalid_steps, self.frozen, self.fitted, self.angle_raw_sensor, self.angleD_raw_sensor, self.angleD_fitted,
                         self.predicted_next_state[ANGLE_IDX], self.predicted_next_state[ANGLED_IDX],
                         self.predicted_next_state[ANGLE_COS_IDX], self.predicted_next_state[ANGLE_SIN_IDX],
                         self.predicted_next_state[POSITION_IDX], self.predicted_next_state[POSITIOND_IDX],
                         self.traj_next_state[ANGLE_IDX], self.traj_next_state[ANGLED_IDX],
                         self.traj_next_state[ANGLE_COS_IDX], self.traj_next_state[ANGLE_SIN_IDX],
                         self.traj_next_state[POSITION_IDX], self.traj_next_state[POSITIOND_IDX],
                         ]
                )

        self.actualMotorCmd_prev = self.actualMotorCmd
        self.Q_prev = self.Q

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

        if self.total_iterations<2 or self.printCount >= PRINT_PERIOD_MS/CONTROL_PERIOD_MS:
            self.printCount = 0

            if (not self.new_console_output): # if no new console output from above, then move to top left, but don't clear screen because there might be logging output between these status reports
                # print('\033[2J', end='') # clear whole screen and move to top left
                print('\033[1;1H', end='') # move to top left
                # print('\033[6A\033[K', end='') # 5A is cursor up 5 lines, then -033[K clears to end of this line. The number of lines should match the number of lprint( below
            self.new_console_output = False

            lprint()

            # cartpole trajectory controller and dancer
            if self.controlEnabled:
                lprint(self.controller.cartpole_trajectory_generator.last_status_text) # TODO add status of dancer here

            ############  Mode  ############
            if self.controlEnabled:
                if 'mpc' in CONTROLLER_NAME:
                    mode='CONTROLLER:   {} (Period={}ms, Synch={}, Horizon={}, Rollouts={}, Predictor={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC, self.controller.optimizer.mpc_horizon, self.controller.optimizer.num_rollouts, self.controller.predictor_wrapper.predictor_name)
                else:
                    mode='CONTROLLER:   {} (Period={}ms, Synch={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC)
            else:
                mode = 'CONTROLLER:   Firmware'
            lprint(mode)

            ############  Mode  ############
            if not self.current_measure.is_idle():
                lprint(f'MEASUREMENT: {self.current_measure}')

            ############  State  ############
            lprint("STATE:  angle:{:+.3f}rad, angle raw:{:04}, position:{:+.2f}cm, position raw:{:04}, target:{}, Q:{:+.2f}, command:{:+05d}, invalid_steps:{}, frozen:{}"
                .format(
                    self.s[ANGLE_IDX],
                    self.angle_raw,
                    self.s[POSITION_IDX] * 100,
                    self.position_raw,
                    f"pos: {self.CartPoleInstance.target_position*100:+.2f}cm, eq: {self.CartPoleInstance.target_equilibrium:+.0f}",
                    self.Q,
                    self.actualMotorCmd,
                    self.invalid_steps,
                    self.frozen
                )
            )

            ############  Timing  ############
            if self.total_iterations > 10 and self.controlled_iterations > 10:
                lprint("TIMING: delta time [μ={:.1f}ms, σ={:.2f}ms], firmware latency [μ={:.1f}ms, σ={:.2f}ms], python latency [μ={:.1f}ms σ={:.2f}ms], controller step [μ={:.1f}ms σ={:.2f}ms], latency violations: {:}/{:} = {:.1f}%"
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
                lprint()

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

def lprint(str=''):
    """ Prints the str with carriage return before rest of line erased after printing out str
    :param str: the string to print
    """
    if str is None:
        str=''
    print("\r" + str + '\033[K') #-  \033[K - Erase to end of line; \r - return to start of line



class PhysicalCartpoleLoggingFormatter(CustomFormatter):
    """Logging Formatter to add colors and customized for physical-cartpole to append to the existing status output"""
    # see https://stackoverflow.com/questions/384076/how-can-i-color-python-logging-output/7995762#7995762

    cr='\r' # carriage return to start of line
    clear_to_eol= '\033[K' # clear rest of line
    clear_to_eos= '\033[0J' # clears to end of screen
    # File "{file}", line {max(line, 1)}'.replace("\\", "/")
    physical_cartpole_formatter = '[%(levelname)s]: %(name)s - %(message)s at line %(lineno)d in %(filename)s %(funcName)s'

    FORMATS = {
        logging.DEBUG: cr + CustomFormatter.grey + physical_cartpole_formatter + CustomFormatter.reset + clear_to_eol+ clear_to_eos,
        logging.INFO: cr + CustomFormatter.cyan + physical_cartpole_formatter + CustomFormatter.reset + clear_to_eol+ clear_to_eos,
        logging.WARNING: cr + CustomFormatter.red + physical_cartpole_formatter + CustomFormatter.reset + clear_to_eol+ clear_to_eos,
        logging.ERROR: cr + CustomFormatter.bold_red + physical_cartpole_formatter + CustomFormatter.reset + clear_to_eol+ clear_to_eos,
        logging.CRITICAL: cr + CustomFormatter.bold_red + physical_cartpole_formatter + CustomFormatter.reset + clear_to_eol+ clear_to_eos
    }

    def __init__(self, fmt=None, datefmt=None, style='%', validate=True):
        super().__init__(fmt, datefmt, style, validate)

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

