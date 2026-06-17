import numpy as np
from SI_Toolkit.Functions.FunctionalDict import FunctionalDict
from SI_Toolkit.LivePlotter.live_plotter_sender import LivePlotter_Sender
from SI_Toolkit.Functions.General.TerminalContentManager import TerminalContentManager

from CartPoleSimulation.CartPole.state_utilities import ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, \
    POSITION_IDX, POSITIOND_IDX

from SI_Toolkit.General.data_manager import DataManager
from CartPoleSimulation.CartPole.csv_logger import create_csv_file_name, create_csv_file, create_csv_header, create_csv_title

from globals import (
    CONTROLLER_NAME, CONTROL_PERIOD_MS, PRINT_PERIOD_MS, CONTROL_SYNC,
    PATH_TO_EXPERIMENT_RECORDINGS, TIME_LIMITED_RECORDING_LENGTH,
    DEFAULT_ADDRESS, LIVE_PLOTTER_USE_REMOTE_SERVER, LIVE_PLOTTER_REMOTE_USERNAME, LIVE_PLOTTER_REMOTE_IP
)


class MainLoggingManager:
    def __init__(self, driver):

        self.driver = driver

        # The below dict lists variables to be logged with csv file when recording is on
        # Just add new variable here and it will be logged
        self.dict_data_to_save_basic = FunctionalDict({

            'time': lambda: driver.th.elapsedTime,
            'deltaTimeMs': lambda: driver.th.time_between_measurements_chip * 1000,

            'angle_raw': lambda: driver.idp.angle_raw,
            'angleD_raw': lambda: driver.idp.angleD_raw,
            'angle': lambda: driver.s[ANGLE_IDX],
            'angleD': lambda: driver.s[ANGLED_IDX],
            'angle_cos': lambda: driver.s[ANGLE_COS_IDX],
            'angle_sin': lambda: driver.s[ANGLE_SIN_IDX],
            'position_raw': lambda: driver.idp.position_raw,
            'position': lambda: driver.s[POSITION_IDX],
            'positionD': lambda: driver.s[POSITIOND_IDX],

            'target_position': lambda: driver.target_position,
            'target_equilibrium': lambda: driver.CartPoleInstance.target_equilibrium,

            'actualMotorSave': lambda: driver.actualMotorCmd_prev,
            'Q': lambda: driver.Q_prev,
            'Q_ccrc': lambda: driver.Q_ccrc_prev,

            'measurement': lambda: driver.epm.current_experiment_protocol,

            'angle_squared': lambda: driver.s[ANGLE_IDX] ** 2,
            'position_squared': lambda: (driver.s[POSITION_IDX] - driver.target_position) ** 2,
            'Q_squared': lambda: driver.Q_prev ** 2,

            'latency': lambda: driver.th.firmware_latency,
            'latency_violations': lambda: driver.th.latency_violations,
            'pythonLatency': lambda: driver.th.python_latency,
            'controller_steptime': lambda: driver.th.controller_steptime_previous,
            'additionalLatency': lambda: driver.th.additional_latency,
            'invalid_steps': lambda: driver.idp.invalid_steps,
            'freezme': lambda: driver.idp.freezme,

            'angle_raw_sensor': lambda: driver.idp.angle_raw_sensor,
            'angleD_raw_sensor': lambda: driver.idp.angleD_raw_sensor,
        })

        self.data_to_save_measurement = {}
        self.data_to_save_controller = {}

        self.data_manager = DataManager(create_csv_file)

        self.csv_name = None
        self.recording_length = np.inf
        self.start_recording_flag = False  # Gives signal to start recording during the current control iteration, starting recording may take more than one control iteration

        # Console Printing
        self.printCount = 0
        self.tcm = None  # Terminal Content Manager

        self.live_plotter_sender = LivePlotter_Sender(
            DEFAULT_ADDRESS,
            LIVE_PLOTTER_USE_REMOTE_SERVER,
            LIVE_PLOTTER_REMOTE_USERNAME,
            LIVE_PLOTTER_REMOTE_IP
        )

    def step(self):
        self.csv_recording_step()

        self.plot_live()

        self.write_current_data_to_terminal()

    @property
    def recording_running(self):
        return self.data_manager.recording_running

    @property
    def starting_recording(self):
        return self.data_manager.starting_recording

    def recording_on_off(self, time_limited_recording=False):
        # (Exclude situation when recording is just being initialized, it may take more than one control iteration)
        if not self.starting_recording:
            if not self.recording_running:
                if hasattr(self.driver.controller, "controller_name"):
                    controller_name = self.driver.controller.controller_name
                else:
                    controller_name = ''
                if hasattr(self.driver.controller, "optimizer_name") and self.driver.controller.has_optimizer:
                    optimizer_name = self.driver.controller.optimizer_name
                else:
                    optimizer_name = ''
                self.csv_name = create_csv_file_name(controller_name=controller_name,
                                                     controller=self.driver.controller,
                                                     optimizer_name=optimizer_name, prefix='CPP')
                if time_limited_recording:
                    self.recording_length = TIME_LIMITED_RECORDING_LENGTH
                else:
                    self.recording_length = np.inf

                self.start_recording_flag = True

            else:
                self.finish_csv_recording(wait_till_complete=False)

    def plot_live(self):
        if self.live_plotter_sender.connection_ready:

            # only include controller columns when cost_component_* entries exist
            controller_keys = [
                k for k in self.driver.controller.controller_data_for_csv
                if k.startswith('cost_component_')
            ]

            if not self.live_plotter_sender.headers_sent:
                headers = [
                    'time', 'Angle', 'Position', 'Q', 'ΔQ',
                    'Target Position', 'AngleD', 'PositionD',
                    'Angle_DVS', 'Position_DVS',
                    'AngleD_DVS', 'PositionD_DVS'
                ]
                # ► controller headers only in the special case
                if controller_keys:
                    # strip the 'cost_component_' prefix
                    controller_headers = [
                        k[len('cost_component_'):] for k in controller_keys
                    ]
                    headers += controller_headers

                self.live_plotter_sender.send_headers(headers)

            else:
                buffer = np.array([
                    self.driver.th.elapsedTime,
                    self.driver.s_original[ANGLE_IDX],
                    self.driver.s_original[POSITION_IDX] * 100,
                    self.driver.Q,
                    (self.driver.Q - self.driver.Q_prev),
                    self.driver.target_position * 100,
                    self.driver.s_original[ANGLED_IDX],
                    self.driver.s_original[POSITIOND_IDX] * 100,

                    self.driver.s_dvs[ANGLE_IDX],
                    self.driver.s_dvs[POSITION_IDX] * 100,
                    self.driver.s_dvs[ANGLED_IDX],
                    self.driver.s_dvs[POSITIOND_IDX] * 100
                ])
                # ► append controller data only when the same special case holds
                if controller_keys:
                    buffer_controller = np.array([
                        self.driver.controller.controller_data_for_csv[k]
                        for k in controller_keys
                    ])
                    buffer = np.append(buffer, buffer_controller)

                self.live_plotter_sender.send_data(buffer)

    def start_csv_recording_if_requested(self):

        if self.start_recording_flag:
            self.start_recording_flag = False
            combined_keys = list(self.dict_data_to_save_basic.keys()) + list(
                self.data_to_save_measurement.keys()) + list(self.data_to_save_controller.keys())

            self.driver.CartPoleInstance.dt_controller = CONTROL_PERIOD_MS / 1000
            self.driver.CartPoleInstance.dt_save = CONTROL_PERIOD_MS / 1000

            self.data_manager.start_csv_recording(
                self.csv_name,
                combined_keys,
                create_csv_title(mode='CPP'),
                create_csv_header(self.driver.CartPoleInstance, mode='CPP'),
                PATH_TO_EXPERIMENT_RECORDINGS,
                mode='online',
                wait_till_complete=False,
                recording_length=self.recording_length
            )

    def csv_recording_step(self):
        if self.driver.actualMotorCmd_prev is not None and self.driver.Q_prev is not None:
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

    def terminal_manager(self):
        self.tcm = TerminalContentManager(special_print_function=True)
        return self.tcm

    def write_current_data_to_terminal(self):
        self.printCount += 1

        self.driver.th.latency_data_for_statistics_in_terminal()

        if True or self.printCount >= PRINT_PERIOD_MS / CONTROL_PERIOD_MS:
            self.printCount = 0

            ESC = '\033['
            BACK_TO_BEGINNING = '\r'
            CLEAR_LINE = ESC + 'K'  # Clear after the status text

            # Controller
            if self.driver.controlEnabled:
                if 'mpc' in CONTROLLER_NAME:
                    mode = 'CONTROLLER:   {} (Period={}ms, Synch={}, Horizon={}, Rollouts={}, Predictor={})'.format(
                        CONTROLLER_NAME, CONTROL_PERIOD_MS, CONTROL_SYNC, self.driver.controller.optimizer.mpc_horizon,
                        self.driver.controller.optimizer.num_rollouts, self.driver.controller.predictor.predictor_name)
                else:
                    mode = 'CONTROLLER:   {} (Period={}ms, Synch={})'.format(CONTROLLER_NAME, CONTROL_PERIOD_MS,
                                                                             CONTROL_SYNC)
            elif self.driver.firmwareControl:
                mode = 'CONTROLLER:   Firmware'
            else:
                mode = 'CONTROLLER:   Idle'
            self.tcm.print_temporary(BACK_TO_BEGINNING + mode + CLEAR_LINE)

            # Experiment Protocol
            self.tcm.print_temporary(
                BACK_TO_BEGINNING + f'MEASUREMENT: {self.driver.epm.current_experiment_protocol}' + CLEAR_LINE)

            # State
            self.tcm.print_temporary(
                BACK_TO_BEGINNING + "STATE:  angle:{:+.3f}rad, angle raw:{:04}, position:{:+.2f}cm, position raw:{:04}, target:{}, Q:{:+.2f}, command:{:+05d}, invalid_steps:{}, freezme:{}"
                .format(
                    self.driver.s[ANGLE_IDX],
                    self.driver.idp.angle_raw,
                    self.driver.s[POSITION_IDX] * 100,
                    self.driver.idp.position_raw,
                    f"{self.driver.CartPoleInstance.target_position}, {self.driver.CartPoleInstance.target_equilibrium}",
                    self.driver.Q,
                    self.driver.actualMotorCmd,
                    self.driver.idp.invalid_steps,
                    self.driver.idp.freezme
                ) + CLEAR_LINE
                )

            # Timing
            timing_string, timing_latency_string = self.driver.th.strings_for_statistics_in_terminal()
            if timing_string:
                self.tcm.print_temporary(BACK_TO_BEGINNING + timing_string + CLEAR_LINE)

            if timing_latency_string:
                self.tcm.print_temporary(BACK_TO_BEGINNING + timing_latency_string + CLEAR_LINE)

            self.tcm.print_to_terminal()
