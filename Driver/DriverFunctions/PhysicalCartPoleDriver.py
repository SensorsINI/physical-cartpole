# TODO: You can easily switch between controllers in runtime using get_available_controller_names function
import numpy as np

from CartPoleSimulation.CartPole.state_utilities import (create_cartpole_state,
                                                         ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX,
                                                         POSITION_IDX, POSITIOND_IDX)

from DriverFunctions.joystick import Joystick
from DriverFunctions.custom_logging import my_logger
from DriverFunctions.interface import Interface, set_ftdi_latency_timer
from DriverFunctions.incoming_data_processor import IncomingDataProcessor
from DriverFunctions.ExperimentProtocols.experiment_protocols_manager import ExperimentProtocolsManager

from Driver.DriverFunctions.dancer import Dancer
from DriverFunctions.timing_helper import TimingHelper
from Driver.DriverFunctions.interface import get_serial_port
from Driver.DriverFunctions.main_logging_manager import MainLoggingManager
from Driver.DriverFunctions.keyboard_controller import KeyboardController

from globals import (
    CHIP,
    OPTIMIZER_NAME, CONTROLLER_NAME,
    CONTROL_PERIOD_MS, CONTROL_SYNC,
    ANGLE_DEVIATION, ANGLE_AVG_LENGTH,
    ANGLE_HANGING, ANGLE_HANGING_DEFAULT, ANGLE_HANGING_POLOLU, ANGLE_HANGING_ORIGINAL,
    angle_deviation_update,
    POSITION_ENCODER_RANGE, POSITION_NORMALIZATION_FACTOR,
    MOTOR, MOTOR_CORRECTION, CORRECT_MOTOR_DYNAMICS,
    MOTOR_CORRECTION_POLOLU, MOTOR_CORRECTION_ORIGINAL,
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES, MOTOR_FULL_SCALE_SAFE,
    SERIAL_PORT_NUMBER, SERIAL_BAUD,
    SEND_CHANGE_IN_TARGET_POSITION_ALWAYS,
    AUTOSTART,
)

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

        self.controlEnabled = AUTOSTART
        self.firmwareControl = False
        self.terminate_experiment = False

        # Dance Mode
        self.dancer = Dancer()

        # Experiment Protocols
        self.epm = ExperimentProtocolsManager(self)

        # Motor Commands
        self.Q = 0.0  # Motor command normed to be in a range -1 to 1
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
        self.target_equilibrium_previous = 0  # -1 or 1, 0 is not a valid value, but this ensures that at the begining the target equilibrium is always updated
        self.base_target_position = 0.0

        # Joystick variable
        self.joystick = Joystick()

        self.safety_switch_counter = 0

        self.mlm = MainLoggingManager(self)

        self.keyboard_controller = KeyboardController(self)

    def run(self):
        with self.mlm.terminal_manager():
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

        self.th.sleep(1)

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

    def quit_experiment(self):
        # when x hit during loop or other loop exit
        self.InterfaceInstance.set_motor(0)  # turn off motor
        self.InterfaceInstance.close()
        self.joystick.quit()
        self.mlm.live_plotter_sender.close()
        self.mlm.finish_csv_recording()

    def experiment_sequence(self):

        self.keyboard_controller.keyboard_input()

        self.load_data_from_chip()

        self.th.time_measurement()

        self.th.check_latency_violation(self.controlEnabled)

        self.idp.process_state_information(self.s, self.th.time_between_measurements_chip)

        self.s = self.th.add_latency(self.s)

        self.epm.experiment_protocol_step()

        self.mlm.start_csv_recording_if_requested()

        self.set_target_position()

        if self.controlEnabled or self.firmwareControl:
            self.th.controlled_iterations += 1
        else:
            self.th.controlled_iterations = 0

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

        if self.controlEnabled or self.epm.current_experiment_protocol.is_running():
            self.control_signal_to_motor_command()

        if self.controlEnabled or self.epm.current_experiment_protocol.is_running():
            self.motor_command_safety_check()
            self.safety_switch_off()

        if self.controlEnabled or (self.epm.current_experiment_protocol.is_running() and self.epm.current_experiment_protocol.Q is not None):
            self.InterfaceInstance.set_motor(self.actualMotorCmd)

        if self.firmwareControl:
            self.actualMotorCmd = self.command

        # Logging, Plotting, Terminal
        self.mlm.step()

        self.actualMotorCmd_prev = self.actualMotorCmd
        self.Q_prev = self.Q

        self.update_parameters_in_cartpole_instance()

        self.th.python_latency = self.th.time_since(self.InterfaceInstance.start)

    def load_data_from_chip(self):
        # This function will block at the rate of the control loop
        (angle_raw, angleD_raw, position_raw, self.target_position_from_chip, self.command,
         invalid_steps, time_between_measurements_chip, time_current_measurement_chip,
         firmware_latency, latency_violation_chip) = self.InterfaceInstance.read_state()

        self.th.load_timing_data_from_chip(
            time_current_measurement_chip, time_between_measurements_chip, latency_violation_chip, firmware_latency
        )
        self.idp.load_state_data_from_chip(angle_raw, angleD_raw, invalid_steps, position_raw)

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

        if self.epm.current_experiment_protocol.is_idle():
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

    def change_target_position(self, change_direction="increase"):
        """
        This is used just to manually increment, decrement target position with keyboard commands
        """

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

    def software_controller_on_off(self):
        # Reset Performance Buffers
        if self.controlEnabled is False:
            self.switch_on_control()

        elif self.controlEnabled is True:
            self.switch_off_control()
        print("\nself.controlEnabled= {0}".format(self.controlEnabled))

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

    def hardware_controller_on_off(self):
        self.firmwareControl = not self.firmwareControl
        print("\nFirmware Control", self.firmwareControl)
        self.InterfaceInstance.control_mode(self.firmwareControl)

    def run_hardware_experiment(self):
        self.controlEnabled = False
        if self.epm.current_experiment_protocol.is_running():
            self.epm.current_experiment_protocol.stop()
        self.InterfaceInstance.run_hardware_experiment()


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

                if hasattr(self.controller, 'controller_report') and self.th.controlled_iterations > 1:
                    self.controller.controller_report()
                if hasattr(self.controller, 'controller_reset'):
                    self.controller.controller_reset()
                self.dancer.danceEnabled = False
                self.target_position = self.base_target_position
                self.actualMotorCmd = 0
        else:
            self.safety_switch_counter = 0
            pass

    def start_experiment_termination(self):
        self.log.info("\nquitting....")
        self.terminate_experiment = True
