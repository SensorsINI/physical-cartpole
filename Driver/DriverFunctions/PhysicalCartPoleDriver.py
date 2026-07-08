# TODO: You can easily switch between controllers in runtime using get_available_controller_names function
import threading

import numpy as np

from CartPoleSimulation.CartPole.state_utilities import (create_cartpole_state,
                                                         ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX,
                                                         POSITION_IDX, POSITIOND_IDX)
from CartPoleSimulation.CartPole.cartpole_ekf import EKFCartPole, EKFAdaptiveTuner

from DriverFunctions.joystick import Joystick
from DriverFunctions.custom_logging import my_logger
from DriverFunctions.interface import Interface
from DriverFunctions.incoming_data_processor import IncomingDataProcessor
from DriverFunctions.ExperimentProtocols.experiment_protocols_manager import ExperimentProtocolsManager

from Driver.DriverFunctions.dancer import Dancer
from DriverFunctions.timing_helper import TimingHelper
from DriverFunctions.split_control_loop import SplitControlLoop
from DriverFunctions.cpu_affinity import set_thread_cpu_affinity
from Control_Toolkit.serial_interface_helper import get_serial_port, set_ftdi_latency_timer
from Driver.DriverFunctions.main_logging_manager import MainLoggingManager
from Driver.DriverFunctions.keyboard_controller import KeyboardController
from Driver.DriverFunctions.DVS.angle_pos_client import AnglePositionClient

from Control_Toolkit.Cost_Functions.CostFunctionUpdater import CostFunctionUpdater

from globals import (
    CHIP,
    HARDWARE_ANGLE_FILTER_WINDOW, HARDWARE_ANGLE_FILTER_TRIM, HARDWARE_ANGLE_FILTER_MODE,
    OPTIMIZER_NAME, CONTROLLER_NAME, USE_SECLOC,
    POLLING_PERIOD_MS, CONTROL_SYNC,
    CONTROLLER_APPLY_WINDOW_MS,
    TIMESTEPS_FOR_DERIVATIVE,
    CONTROL_CPU_AFFINITY, LOOP_CPU_AFFINITY,
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
    USE_DVS_STATE_ESTIMATION,
    USE_EKF, EKF_CALIBRATION_RUN,
)

import warnings
warnings.simplefilter('ignore', np.RankWarning)


class PhysicalCartPoleDriver:
    def __init__(self, CartPoleInstance):

        self.CartPoleInstance = CartPoleInstance
        if CONTROLLER_NAME == 'mpc':
            self.CartPoleInstance.set_optimizer(optimizer_name=OPTIMIZER_NAME)
        self.CartPoleInstance.set_controller(
            controller_name=CONTROLLER_NAME, use_secloc=USE_SECLOC
        )
        self.controller = self.CartPoleInstance.controller

        control_label = (
            f"{CONTROLLER_NAME}+secloc" if USE_SECLOC else CONTROLLER_NAME
        )
        self.control_label = control_label
        # IO thread: chip polling, gate, actuation. Main thread: compute_step/step.
        # tick-based apply window (see split_control_loop.py).
        self.split_control = SplitControlLoop(
            self.controller,
            apply_window_polling_loops=CONTROLLER_APPLY_WINDOW_MS // POLLING_PERIOD_MS,
            polling_period_s=POLLING_PERIOD_MS / 1000.0,
            name=control_label,
        )
        print(
            f"[{control_label}] IO -> LOOP_CPU_AFFINITY={LOOP_CPU_AFFINITY!r} | "
            f"compute -> main (CONTROL_CPU_AFFINITY={CONTROL_CPU_AFFINITY!r})",
            flush=True,
        )

        self._io_stop = threading.Event()
        self._io_thread = None
        self.on_io_step = None

        self.InterfaceInstance = Interface()

        self.log = my_logger(__name__)

        self.controlEnabled = AUTOSTART
        self.firmwareControl = False
        self.terminate_experiment = False
        self.controller_status_print_period = 1.0
        self._last_controller_status_print_time = -np.inf
        self._cached_controller_status = None

        # Dance Mode
        self.dancer = Dancer()

        # Experiment Protocols
        self.epm = ExperimentProtocolsManager(self)

        # Motor Commands
        self.Q = 0.0  # Motor command normed to be in a range -1 to 1
        self.Q_prev = 0.0
        self.Q_prev_prev = 0.0
        self.Q_ccrc_prev = None
        self.actualMotorCmd = 0
        self.actualMotorCmd_prev = None
        self.command = 0

        # State
        self.s = create_cartpole_state()
        self.th = TimingHelper()
        self.idp = IncomingDataProcessor()  # Takes care of receiving data from the chip and serves as container for raw values

        self.s_dvs = create_cartpole_state()
        self.s_ekf_dvs = create_cartpole_state()
        self.s_original = create_cartpole_state()
        self.s_ekf = create_cartpole_state()

        # Target
        self.position_offset = 0
        self.target_position = 0.0
        self.target_position_previous = 0.0
        self.target_position_from_chip = 0.0
        self.target_equilibrium_previous = 0  # -1 or 1, 0 is not a valid value, but this ensures that at the begining the target equilibrium is always updated
        self.base_target_position = 0.0

        # Joystick variable
        self.joystick = Joystick()

        self.safety_switch_counter = 0

        self.mlm = MainLoggingManager(self)

        self.keyboard_controller = KeyboardController(self)

        self.angle_position_client = None

        if USE_EKF:
            self.ekf = EKFCartPole(
                POLLING_PERIOD_MS / 1000.0,
                self.CartPoleInstance.cpe.params,
            )
            self.ekf_tuner = EKFAdaptiveTuner(self.ekf)
            self._ekf_initialized = True
        else:
            self._ekf_initialized = False

    def run(self):
        self.setup()
        with self.mlm.terminal_manager():
            self.run_experiment()
            self.quit_experiment()

    def setup(self):
        self.keyboard_controller.setup()

        SERIAL_PORT = get_serial_port(chip_type=CHIP, serial_port_number=SERIAL_PORT_NUMBER)
        if CHIP == 'ZYNQ':
            set_ftdi_latency_timer(SERIAL_PORT)
        self.InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)
        self.InterfaceInstance.pc_control_mode(False)
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
        self.InterfaceInstance.set_config_control(controlLoopPeriodMs=POLLING_PERIOD_MS, controlSync=CONTROL_SYNC, angle_hanging=ANGLE_HANGING, avgLen=ANGLE_AVG_LENGTH, correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS, timesteps_for_derivative=TIMESTEPS_FOR_DERIVATIVE)

        if CHIP == 'ZYNQ':
            self.InterfaceInstance.set_angle_filter(
                HARDWARE_ANGLE_FILTER_WINDOW,
                HARDWARE_ANGLE_FILTER_TRIM,
                HARDWARE_ANGLE_FILTER_MODE,
            )
            filter_mode_names = {0: 'raw', 1: 'median', 2: 'trimmed_mean'}
            print(
                f"Hardware angle filter set: window={HARDWARE_ANGLE_FILTER_WINDOW}, "
                f"trim={HARDWARE_ANGLE_FILTER_TRIM}, "
                f"mode={filter_mode_names.get(HARDWARE_ANGLE_FILTER_MODE, HARDWARE_ANGLE_FILTER_MODE)}",
                flush=True,
            )

        try:
            self.controller.printparams()
        except AttributeError:
            print('printparams not implemented for this self.controller.')

        self.th.setup()

        self.InterfaceInstance.stream_output(True)  # now start streaming state

        self.angle_position_client = AnglePositionClient()

        if USE_EKF and not self._ekf_initialized:
            x0 = np.array([self.s[POSITION_IDX],  # cart position
                           0.0,  # start with v = 0
                           self.s[ANGLE_IDX],  # pole angle
                           0.0])  # start with ω = 0
            self.ekf.reset(x0)
            self._ekf_initialized = True


    def run_experiment(self):
        self.split_control.prepare_for_run()
        self._io_stop.clear()
        self._io_thread = threading.Thread(
            target=self._io_loop, name="chip_io", daemon=True
        )
        self._io_thread.start()
        try:
            self.split_control.run_compute_loop()
        finally:
            self._io_stop.set()
            if self._io_thread is not None:
                self._io_thread.join(timeout=2.0)
            self._io_thread = None

    def _io_loop(self):
        set_thread_cpu_affinity(LOOP_CPU_AFFINITY, thread_label="chip io")
        try:
            while not self.terminate_experiment and not self._io_stop.is_set():
                self.io_step()
        finally:
            # Main is parked in run_compute_loop(); unblock it on IO exit (e.g. ESC).
            self.split_control.stop()

    def quit_experiment(self):
        CostFunctionUpdater.stop_all_watchers()
        self.split_control.stop()
        self._io_stop.set()
        if self._io_thread is not None and self._io_thread.is_alive():
            self._io_thread.join(timeout=2.0)
        self._io_thread = None
        self.angle_position_client.close()
        self.InterfaceInstance.set_motor(0)
        self.InterfaceInstance.close()
        self.joystick.quit()
        self.mlm.live_plotter_sender.close()
        self.mlm.finish_csv_recording()

    def io_step(self):

        self.keyboard_controller.keyboard_input()

        self.load_data_from_chip()
        self.check_calibration_status()

        self.th.time_measurement()

        self.th.check_latency_violation(self.controlEnabled or self.firmwareControl)

        self.idp.process_state_information(self.s, self.th.time_between_measurements_chip)

        self.s_original[:] = self.s

        if USE_DVS_STATE_ESTIMATION:
            self.overwrite_with_state_from_DVS(self.s)

        if self._ekf_initialized:

            # Feed the adaptive tuner *before* the EKF step
            self.ekf_tuner.feed_measurement(
                pos=self.s[POSITION_IDX],
                ang=self.s[ANGLE_IDX],
                u=self.Q_prev,  # motor effort of previous cycle
                hi_grade=EKF_CALIBRATION_RUN,
                vel_gt=self.s[POSITIOND_IDX],  # only valid while hi‑grade == True
                angvel_gt=self.s[ANGLED_IDX],
            )


            # Use the motor effort applied *during the previous interval*.
            # self.Q_prev is your control signal in the range [-1,1]; scale if needed.
            v_est, omega_est = self.ekf.step(
                position_meas=self.s[POSITION_IDX],
                angle_meas=self.s[ANGLE_IDX],
                u=self.Q_prev,
            )

            x_hat = self.ekf.get_state()

            # splice the estimates back into the pipeline
            self.s_ekf[POSITIOND_IDX] = v_est
            self.s_ekf[ANGLED_IDX] = omega_est
            self.s_ekf[POSITION_IDX] = x_hat[0]
            self.s_ekf[ANGLE_IDX] = x_hat[2]
            self.s_ekf[ANGLE_COS_IDX] = np.cos(x_hat[2])
            self.s_ekf[ANGLE_SIN_IDX] = np.sin(x_hat[2])

            self.s[:] = self.s_ekf[:]

        self.s = self.th.add_latency(self.s)

        self.epm.experiment_protocol_step()

        self.mlm.start_csv_recording_if_requested()

        self.set_target_position()

        if self.controlEnabled or self.firmwareControl:
            self.th.controlled_iterations += 1
        else:
            self.th.controlled_iterations = 0

        self.CartPoleInstance.Q_ccrc = self.Q

        if self.controlEnabled:
            self.Q = float(self.split_control.tick(
                self.s,
                self.th.time_current_measurement_chip,
                {"target_position": self.target_position,
                 "target_equilibrium": self.CartPoleInstance.target_equilibrium,
                 "Q_ccrc": self.Q_prev_prev,
                 }
            ))
            self.th.load_controller_timing(
                self.split_control.calc_time_last,
                self.split_control.calc_count,
                self.split_control.overrun_count,
            )
            self.print_controller_status_if_available()

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

        if (
                not self.InterfaceInstance.calibration_in_progress
                and (self.controlEnabled or (
                    self.epm.current_experiment_protocol.is_running()
                    and self.epm.current_experiment_protocol.Q is not None
                ))
        ):
            self.InterfaceInstance.set_motor(self.actualMotorCmd)

        if self.firmwareControl:
            self.actualMotorCmd = self.command

        # Logging, Plotting, Terminal
        self.mlm.step()

        self.actualMotorCmd_prev = self.actualMotorCmd
        self.Q_prev_prev = self.Q_prev
        self.Q_prev = self.Q
        self.Q_ccrc_prev = self.CartPoleInstance.Q_ccrc

        self.update_parameters_in_cartpole_instance()

        if self.on_io_step is not None:
            self.on_io_step()

    experiment_sequence = io_step  # legacy alias

    def print_controller_status_if_available(self):
        if self.controller is None:
            return
        if (
            self.th.time_current_measurement_chip - self._last_controller_status_print_time
            < self.controller_status_print_period
        ):
            return

        get_controller_status = getattr(self.controller, "get_controller_status", None)
        controller_status = get_controller_status() if get_controller_status is not None else None

        status_parts = []
        if controller_status:
            status_parts.append(controller_status)
        status_parts.append(self.split_control.get_status())

        if status_parts:
            self._cached_controller_status = f"[{self.control_label}] " + " | ".join(status_parts)
            self._last_controller_status_print_time = self.th.time_current_measurement_chip

        self.th.python_latency = self.th.time_since(self.InterfaceInstance.start)


    def overwrite_with_state_from_DVS(self, s):
        angle, position, positionD, angleD, ts = self.angle_position_client.get_estimate()

        self.s_dvs[ANGLE_IDX] = angle if angle is not None else self.s_dvs[ANGLE_IDX]
        self.s_dvs[POSITION_IDX] = position if position is not None else self.s_dvs[POSITION_IDX]
        self.s_dvs[POSITIOND_IDX] = positionD if positionD is not None else self.s_dvs[POSITIOND_IDX]
        self.s_dvs[ANGLED_IDX] = angleD if angleD is not None else self.s_dvs[ANGLED_IDX]
        self.s_dvs[ANGLE_COS_IDX] = np.cos(angle) if angle is not None else self.s_dvs[ANGLE_COS_IDX]
        self.s_dvs[ANGLE_SIN_IDX] = np.sin(angle) if angle is not None else self.s_dvs[ANGLE_SIN_IDX]

        s[:] = self.s_dvs[:]


    def load_data_from_chip(self):
        # This function will block at the rate of the control loop
        (angle_raw, angleD_raw, position_raw, self.target_position_from_chip, self.command,
         invalid_steps, time_between_measurements_chip, time_current_measurement_chip,
         firmware_latency, latency_violation_chip) = self.InterfaceInstance.read_state()

        self.th.load_timing_data_from_chip(
            time_current_measurement_chip, time_between_measurements_chip, latency_violation_chip, firmware_latency
        )
        self.idp.load_state_data_from_chip(angle_raw, angleD_raw, invalid_steps, position_raw)

    def apply_target_position_from_chip(self):
        self.target_position = self.target_position_from_chip

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

        send_target_to_chip = (
            self.controlEnabled
            or self.firmwareControl
            or self.epm.current_experiment_protocol.is_running()
        )
        if send_target_to_chip and (SEND_CHANGE_IN_TARGET_POSITION_ALWAYS or self.firmwareControl):
            if self.target_position != self.target_position_previous:
                self.InterfaceInstance.set_target_position(self.target_position)
                self.target_position_previous = self.target_position

            if self.CartPoleInstance.target_equilibrium != self.target_equilibrium_previous:
                self.InterfaceInstance.set_target_equilibrium(self.CartPoleInstance.target_equilibrium)
                self.target_equilibrium_previous = self.CartPoleInstance.target_equilibrium

        if not self.controlEnabled:
            self.apply_target_position_from_chip()

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
        self._cached_controller_status = None
        self.controlEnabled = False
        self.Q = 0
        self.InterfaceInstance.set_motor(0)
        self.InterfaceInstance.pc_control_mode(False)
        if self.controller.controller_name == 'mppi-tf':
            self.controller.controller_report()
        try:
            self.controller.controller_reset()
        except NotImplementedError:
            pass
        self.split_control.reset()
        self.dancer.danceEnabled = False
        self.target_position = self.base_target_position
        self.th.reset_timing_helper_memory()

    def switch_on_control(self):
        self.controlEnabled = True
        self.split_control.reset()
        self.th.reset_timing_helper_memory()
        self.InterfaceInstance.control_mode(False)
        self.InterfaceInstance.pc_control_mode(True)

    def hardware_controller_on_off(self):
        self.firmwareControl = not self.firmwareControl
        if self.firmwareControl and self.controlEnabled:
            self.switch_off_control()
        print("\nFirmware Control", self.firmwareControl)
        self.InterfaceInstance.control_mode(self.firmwareControl)

    def run_hardware_experiment(self):
        self.controlEnabled = False
        self.InterfaceInstance.pc_control_mode(False)
        if self.epm.current_experiment_protocol.is_running():
            self.epm.current_experiment_protocol.stop()
        self.InterfaceInstance.run_hardware_experiment()

    def measure_and_apply_hanging_angle(self):
        global ANGLE_HANGING, ANGLE_HANGING_DEFAULT, ANGLE_DEVIATION

        if self.controlEnabled:
            self.switch_off_control()
        if self.firmwareControl:
            self.firmwareControl = False
            self.InterfaceInstance.control_mode(False)
        if self.epm.current_experiment_protocol.is_running():
            self.epm.current_experiment_protocol.stop()

        angle_hanging, angle_hanging_std = self.idp.precise_angle_measurement(self.InterfaceInstance)
        ANGLE_HANGING = float(angle_hanging)
        ANGLE_HANGING_DEFAULT = False
        ANGLE_DEVIATION[...] = angle_deviation_update(ANGLE_HANGING)
        self.idp.angle_deviation_finetune = 0.0

        self.InterfaceInstance.set_config_control(controlLoopPeriodMs=POLLING_PERIOD_MS,
                                                  controlSync=CONTROL_SYNC,
                                                  angle_hanging=ANGLE_HANGING, avgLen=ANGLE_AVG_LENGTH,
                                                  correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS,
                                                  timesteps_for_derivative=TIMESTEPS_FOR_DERIVATIVE)

        print('\nApplied measured hanging angle for this run.')
        print('ANGLE_HANGING: {:.3f} ADC reading (std {:.3f})'.format(ANGLE_HANGING, angle_hanging_std))
        print('ANGLE_DEVIATION: {:.3f} ADC reading'.format(float(ANGLE_DEVIATION.item())))

    def calibrate(self):
        self.controlEnabled = False
        self.InterfaceInstance.pc_control_mode(False)
        if self.epm.current_experiment_protocol.is_running():
            self.epm.current_experiment_protocol.stop()

        if self.InterfaceInstance.start_calibration():
            print("\nCalibrating motor position.... ")
        else:
            print("\nCalibration already in progress.")

    def check_calibration_status(self):
        if not self.InterfaceInstance.calibration_completed:
            return

        self.InterfaceInstance.calibration_completed = False
        self.apply_calibration_result()

    def apply_calibration_result(self):
        global MOTOR, MOTOR_CORRECTION, ANGLE_DEVIATION, ANGLE_HANGING

        if self.InterfaceInstance.encoderDirection == 1:
            MOTOR = 'POLOLU'
            MOTOR_CORRECTION = MOTOR_CORRECTION_POLOLU
            detected_motor_angle_hanging = ANGLE_HANGING_POLOLU

        elif self.InterfaceInstance.encoderDirection == -1:
            MOTOR = 'ORIGINAL'
            MOTOR_CORRECTION = MOTOR_CORRECTION_ORIGINAL
            detected_motor_angle_hanging = ANGLE_HANGING_ORIGINAL
        elif self.InterfaceInstance.encoderDirection == 0:
            raise RuntimeError(
                'Firmware calibration failed: reverse movement was not detected or the cart did not reach center. '
                'Check motor direction, encoder counts, and mechanical friction/stall.'
            )
        else:
            raise ValueError(
                f'Unexpected value for self.InterfaceInstance.encoderDirection = {self.InterfaceInstance.encoderDirection}'
            )

        if ANGLE_HANGING_DEFAULT:
            ANGLE_HANGING = detected_motor_angle_hanging
            ANGLE_DEVIATION[...] = angle_deviation_update(ANGLE_HANGING)

        print("Done calibrating")
        print('Detected motor: {}'.format(MOTOR))

        self.InterfaceInstance.set_config_control(controlLoopPeriodMs=POLLING_PERIOD_MS,
                                                  controlSync=CONTROL_SYNC,
                                                  angle_hanging=ANGLE_HANGING, avgLen=ANGLE_AVG_LENGTH,
                                                  correct_motor_dynamics=CORRECT_MOTOR_DYNAMICS,
                                                  timesteps_for_derivative=TIMESTEPS_FOR_DERIVATIVE)

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
                self.InterfaceInstance.pc_control_mode(False)

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
