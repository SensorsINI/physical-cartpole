# TODO Aftrer joystick is unplugged and plugged again it interferes with the calibration, it causes the motor to get stuck at some speed after calibration. Add this to the readme file to warn the user.
# TODO: You can easily switch between controllers in runtime using this and get_available_controller_names function
# todo check if position unit conversion works for the following features: dance mode (can be checked for a nice self.controller only)
import time

from SI_Toolkit.Functions.FunctionalDict import FunctionalDict
from SI_Toolkit.LivePlotter.live_plotter_sender import LivePlotter_Sender
from SI_Toolkit.Functions.General.TerminalContentManager import TerminalContentManager

from CartPoleSimulation.CartPole.state_utilities import create_cartpole_state, ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, POSITION_IDX, POSITIOND_IDX

from CartPoleSimulation.CartPole.data_manager import DataManager
from CartPoleSimulation.CartPole.csv_logger import create_csv_file_name
from DriverFunctions.csv_helpers import create_csv_header, create_csv_title

from DriverFunctions.joystick import Joystick
from DriverFunctions.custom_logging import my_logger
from DriverFunctions.interface import Interface, set_ftdi_latency_timer
from DriverFunctions.ExperimentProtocols.experiment_protocols_manager import experiment_protocols_manager_class
from DriverFunctions.incoming_data_processor import IncomingDataProcessor

from DriverFunctions.timing_helper import TimingHelper
from Driver.DriverFunctions.dancer import Dancer
from Driver.DriverFunctions.interface import get_serial_port
from Driver.DriverFunctions.keyboard_controller import KeyboardController

from globals import *

import warnings
warnings.simplefilter('ignore', np.RankWarning)


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
        self.th = TimingHelper()
        self.idp = IncomingDataProcessor()  # Takes care of receiving data from the chip and serves as container for raw values

        # Target
        self.position_offset = 0
        self.target_position = 0.0
        self.target_position_previous = 0.0
        self.target_equilibrium_previous = 0 # -1 or 1, 0 is not a valid value, but this ensures that at the begining the target equilibrium is always updated
        self.base_target_position = 0.0

        # Joystick variable
        self.joystick = Joystick()

        self.safety_switch_counter = 0

        # The below dict lists variables to be logged with csv file when recording is on
        # Just add new variable here and it will be logged
        self.dict_data_to_save_basic = FunctionalDict({

            'time': lambda: self.th.elapsedTime,
            'deltaTimeMs': lambda: self.th.time_between_measurements_chip * 1000,

            'angle_raw': lambda: self.idp.angle_raw,
            'angleD_raw': lambda: self.idp.angleD_raw,
            'angle': lambda: self.s[ANGLE_IDX],
            'angleD': lambda: self.s[ANGLED_IDX],
            'angle_cos': lambda: self.s[ANGLE_COS_IDX],
            'angle_sin': lambda: self.s[ANGLE_SIN_IDX],
            'position_raw': lambda: self.idp.position_raw,
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

            'latency': lambda: self.th.firmware_latency,
            'latency_violations': lambda: self.th.latency_violations,
            'pythonLatency': lambda: self.python_latency,
            'controller_steptime': lambda: self.th.controller_steptime_previous,
            'additionalLatency': lambda: self.th.additional_latency,
            'invalid_steps': lambda: self.idp.invalid_steps,
            'freezme': lambda: self.idp.freezme,

            'angle_raw_sensor': lambda: self.idp.angle_raw_sensor,
            'angleD_raw_sensor': lambda: self.idp.angleD_raw_sensor,
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

        self.keyboard_controller = KeyboardController(self)

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
        self.keyboard_controller.setup()

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

        self.th.setup()

        self.InterfaceInstance.stream_output(True)  # now start streaming state

    def run_experiment(self):

        while not self.terminate_experiment:
            self.experiment_sequence()


    def experiment_sequence(self):

        self.keyboard_controller.keyboard_input()

        self.load_data_from_chip()

        self.th.time_measurement()

        self.th.check_latency_violation(self.controlEnabled)

        self.idp.process_state_information(self.s, self.th.time_between_measurements_chip)

        self.s = self.th.add_latency(self.s)

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
            with self.th.timer('controller_steptime', 'controller_steptime_previous'):
                self.Q = float(self.controller.step(
                    self.s,
                    self.th.time_current_measurement_chip,
                    {"target_position": self.target_position,
                     "target_equilibrium": self.CartPoleInstance.target_equilibrium
                     }
                ))

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

    def load_data_from_chip(self):
        # This function will block at the rate of the control loop
        (angle_raw, angleD_raw, position_raw, self.target_position_from_chip, self.command,
         invalid_steps, time_between_measurements_chip, time_current_measurement_chip,
         firmware_latency, latency_violation_chip) = self.InterfaceInstance.read_state()

        self.th.load_timing_data_from_chip(
            time_current_measurement_chip, time_between_measurements_chip, latency_violation_chip, firmware_latency
        )
        self.idp.load_state_data_from_chip(angle_raw, angleD_raw, invalid_steps, position_raw)

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

    def run_hardware_experiment(self):
        self.controlEnabled = False
        if self.current_experiment_protocol.is_running():
            self.current_experiment_protocol.stop()
        self.InterfaceInstance.run_hardware_experiment()


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
        self.dancer.danceEnabled = False
        self.target_position = self.base_target_position
        self.th.latency_violations = 0

    def switch_on_control(self):
        self.controlEnabled = True
        self.th.reset_timing_helper_memory()

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
        self.CartPoleInstance.time = self.th.time_current_measurement
        self.CartPoleInstance.dt = self.th.controller_steptime
        self.CartPoleInstance.target_position = self.target_position

    def set_target_position(self):

        if self.current_experiment_protocol.is_idle():
            self.target_position, self.CartPoleInstance.target_equilibrium = self.dancer.dance_step(
                self.th.time_current_measurement,
                self.base_target_position,
                self.target_position,
                self.CartPoleInstance.target_equilibrium,
            )

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
                self.current_experiment_protocol.update_state(self.s[ANGLE_IDX], self.s[POSITION_IDX], self.th.time_current_measurement)
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
        if abs(self.idp.position_raw) > 0.95 * (POSITION_ENCODER_RANGE // 2):
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
                                self.th.elapsedTime,
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

        self.th.latency_data_for_statistics_in_terminal()

        if True or self.printCount >= PRINT_PERIOD_MS/CONTROL_PERIOD_MS:
            self.printCount = 0

            ESC = '\033['
            BACK_TO_BEGINNING = '\r'
            CLEAR_LINE = ESC + 'K'  # Clear the entire line

            self.tcm.print_temporary(BACK_TO_BEGINNING + CLEAR_LINE)

            ############  Mode  ############
            if self.controlEnabled:
                if 'mpc' in CONTROLLER_NAME:
                    mode = 'CONTROLLER:   {} (Period={}ms, Synch={}, Horizon={}, Rollouts={}, Predictor={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC, self.controller.optimizer.mpc_horizon, self.controller.optimizer.num_rollouts, self.controller.predictor.predictor_name)
                else:
                    mode = 'CONTROLLER:   {} (Period={}ms, Synch={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC)
            else:
                mode = 'CONTROLLER:   Firmware'
            self.tcm.print_temporary(BACK_TO_BEGINNING + mode +  CLEAR_LINE)

            ############  Mode  ############
            self.tcm.print_temporary(BACK_TO_BEGINNING + f'MEASUREMENT: {self.current_experiment_protocol}' +  CLEAR_LINE)

            ############  State  ############
            self.tcm.print_temporary(BACK_TO_BEGINNING + "STATE:  angle:{:+.3f}rad, angle raw:{:04}, position:{:+.2f}cm, position raw:{:04}, target:{}, Q:{:+.2f}, command:{:+05d}, invalid_steps:{}, freezme:{}"
                .format(
                    self.s[ANGLE_IDX],
                    self.idp.angle_raw,
                    self.s[POSITION_IDX] * 100,
                    self.idp.position_raw,
                    f"{self.CartPoleInstance.target_position}, {self.CartPoleInstance.target_equilibrium}",
                    self.Q,
                    self.actualMotorCmd,
                    self.idp.invalid_steps,
                    self.idp.freezme
                ) + CLEAR_LINE
            )

            ############  Timing  ############
            timing_string, timing_latency_string = self.th.strings_for_statistics_in_terminal()
            if timing_string:
                self.tcm.print_temporary(BACK_TO_BEGINNING + timing_string + CLEAR_LINE)

            if timing_latency_string:
                self.tcm.print_temporary(BACK_TO_BEGINNING + timing_latency_string + CLEAR_LINE)

            self.tcm.print_to_terminal()
