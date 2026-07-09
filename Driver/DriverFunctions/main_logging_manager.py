import numpy as np
from SI_Toolkit.Functions.FunctionalDict import FunctionalDict
from SI_Toolkit.LivePlotter.live_plotter_sender import LivePlotter_Sender
from SI_Toolkit.Functions.General.TerminalContentManager import TerminalContentManager

from CartPoleSimulation.CartPole.state_utilities import ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX, \
    POSITION_IDX, POSITIOND_IDX

from SI_Toolkit.General.data_manager import DataManager
from CartPoleSimulation.CartPole.csv_logger import create_csv_file_name, create_csv_file, create_csv_header, create_csv_title

from globals import (
    CHIP,
    CONTROLLER_NAME, POLLING_PERIOD_MS, PRINT_PERIOD_MS, CONTROL_SYNC,
    CONTROLLER_APPLY_WINDOW_MS, LOOP_CPU_AFFINITY, CONTROL_CPU_AFFINITY,
    USE_SECLOC,
    HARDWARE_ANGLE_FILTER_OVERRIDE,
    HARDWARE_ANGLE_FILTER_WINDOW, HARDWARE_ANGLE_FILTER_TRIM, HARDWARE_ANGLE_FILTER_MODE,
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
            # Absolute chip timestamp (s). The Secloc gate times its ref_period on
            # this clock; deltaTimeMs alone cannot reconstruct it because dropped
            # serial packets are invisible in chip-side deltas.
            'time_chip': lambda: driver.th.time_current_measurement_chip,

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

            # TEMP DEBUG: raw 16-bit channel-0 ADC word from the chip. The
            # firmware temporarily sends (raw_count / 100) as target_position,
            # so multiply by 100 to recover the integer ADC count for analysis.
            'slider_adc_raw': lambda: driver.target_position_from_chip * 100,

            'actualMotorSave': lambda: driver.actualMotorCmd_prev,
            'Q': lambda: driver.Q_prev,
            'Q_ccrc': lambda: driver.Q_ccrc_prev,

            'measurement': lambda: driver.epm.current_experiment_protocol,

            'angle_squared': lambda: driver.s[ANGLE_IDX] ** 2,
            'position_squared': lambda: (driver.s[POSITION_IDX] - driver.target_position) ** 2,
            'Q_squared': lambda: driver.Q_prev ** 2,

            'latency': lambda: driver.th.firmware_latency,
            'firmware_latency_violations': lambda: driver.th.firmware_latency_violations,
            'controller_latency_violations': lambda: driver.th.controller_latency_violations,
            'pythonLatency': lambda: driver.th.python_latency,
            'controller_steptime': lambda: driver.th.controller_steptime,
            'additionalLatency': lambda: driver.th.additional_latency,
            'invalid_steps': lambda: driver.idp.invalid_steps,
            'freezme': lambda: driver.idp.freezme,

            'angle_raw_sensor': lambda: driver.idp.angle_raw_sensor,
            'angleD_raw_sensor': lambda: driver.idp.angleD_raw_sensor,
        })

        self.data_to_save_measurement = {}
        # 1 if a freshly computed control was applied this iteration, 0 if held
        self.data_to_save_controller = FunctionalDict({
            'controller_update_applied': lambda: int(driver.split_control.last_applied_now),
            'split_control_busy': lambda: int(driver.split_control.is_busy),
        })

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

    def _controller_csv_value(self, key, default=0):
        """Read a value from the controller's controller_data_for_csv (values may be callables)."""
        controller = getattr(self.driver, "controller", None)
        data = getattr(controller, "controller_data_for_csv", None)
        if data is None:
            return default
        try:
            if key not in data:
                return default
            value = data[key]
            return value() if callable(value) else value
        except (KeyError, TypeError):
            return default

    def _controller_data_to_save(self):
        data = FunctionalDict()
        data.update(self.data_to_save_controller)
        if self.driver.firmwareControl:
            # On-chip SecLoc telemetry parsed from the state packet; the PC
            # controller is idle so its csv dict is not consulted.
            data['secloc_skipped_update'] = lambda: int(self.driver.secloc_skipped_update_chip)
            data['secloc_gate_skipped'] = lambda: int(self.driver.secloc_gate_skipped_chip)
            return data
        controller = getattr(self.driver, "controller", None)
        controller_data = getattr(controller, "controller_data_for_csv", {})
        for key in controller_data:
            if key.startswith("cost_component_"):
                continue
            data[key] = lambda key=key: self._controller_csv_value(key, 0)
        return data

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
                controller_name = self.driver.CartPoleInstance.controller_name
                if hasattr(self.driver.controller, "optimizer_name") and self.driver.controller.has_optimizer:
                    optimizer_name = self.driver.controller.optimizer_name
                else:
                    optimizer_name = ''
                self.csv_name = create_csv_file_name(
                    controller_name=controller_name,
                    controller=self.driver.controller,
                    optimizer_name=optimizer_name,
                    prefix='CPP',
                )
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
                self.data_to_save_measurement.keys()) + list(self._controller_data_to_save().keys())

            self.driver.CartPoleInstance.dt_controller = (
                POLLING_PERIOD_MS / 1000 if self.driver.firmwareControl
                else CONTROLLER_APPLY_WINDOW_MS / 1000
            )
            self.driver.CartPoleInstance.dt_save = POLLING_PERIOD_MS / 1000

            header = create_csv_header(self.driver.CartPoleInstance, mode='CPP')
            mode_lines = [
                f"Secloc gate: {USE_SECLOC}",
                f"IO CPU affinity: {LOOP_CPU_AFFINITY}",
                f"Control CPU affinity: {CONTROL_CPU_AFFINITY}",
            ]
            if CHIP == 'ZYNQ':
                if HARDWARE_ANGLE_FILTER_OVERRIDE:
                    filter_mode_names = {0: 'raw', 1: 'median', 2: 'trimmed_mean'}
                    mode_lines.append(
                        "Hardware angle filter: "
                        f"{filter_mode_names.get(HARDWARE_ANGLE_FILTER_MODE, HARDWARE_ANGLE_FILTER_MODE)} "
                        f"window={HARDWARE_ANGLE_FILTER_WINDOW} trim={HARDWARE_ANGLE_FILTER_TRIM}"
                    )
                else:
                    mode_lines.append(
                        "Hardware angle filter: firmware boot default (trimmed mean 63/7)"
                    )
            if self.driver.firmwareControl:
                # The driver mirrors config_secloc.yml onto the chip
                # (CMD_SET_SECLOC_CONFIG), so the PC-side view is authoritative
                # for what the chip gate is running.
                chip_gate = self.driver.chip_secloc_config
                mode_lines[0] = "Secloc on chip: True"
                mode_lines[1:1] = [
                    "On-chip controller: secloc+neural_controller_C",
                    f"Secloc log_base: {chip_gate.log_base:g}",
                    f"Secloc ref_period_ticks: {chip_gate.ref_period_ticks}",
                    f"Secloc dead_ang: {chip_gate.dead_ang:g}",
                    f"Secloc dead_pos: {chip_gate.dead_pos:g}",
                ]
            elif USE_SECLOC:
                secloc = self.driver.controller.secloc
                mode_lines[1:1] = [
                    f"Secloc log_base: {secloc.log_base}",
                    f"Secloc ref_period_ticks: {secloc.ref_period_ticks}",
                    f"Secloc dead_ang: {secloc.dead_ang}",
                    f"Secloc dead_pos: {secloc.dead_pos}",
                ]
            if "Parameters:" in header:
                insert_at = header.index("Parameters:")
                header[insert_at:insert_at] = mode_lines + [""]
            else:
                header.extend(mode_lines)

            self.data_manager.start_csv_recording(
                self.csv_name,
                combined_keys,
                create_csv_title(mode='CPP'),
                header,
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
                    self._controller_data_to_save()
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

        if True or self.printCount >= PRINT_PERIOD_MS / POLLING_PERIOD_MS:
            self.printCount = 0

            ESC = '\033['
            BACK_TO_BEGINNING = '\r'
            CLEAR_LINE = ESC + 'K'  # Clear after the status text

            # Controller
            if self.driver.controlEnabled:
                ctrl = CONTROLLER_NAME
                if USE_SECLOC:
                    ctrl = f"{CONTROLLER_NAME}+secloc"
                if CONTROLLER_NAME == 'mpc':
                    mode = 'CONTROLLER:   {} (Period={}ms, Synch={}, Horizon={}, Rollouts={}, Predictor={})'.format(
                        ctrl, CONTROLLER_APPLY_WINDOW_MS, CONTROL_SYNC, self.driver.controller.optimizer.mpc_horizon,
                        self.driver.controller.optimizer.num_rollouts, self.driver.controller.predictor.predictor_name)
                else:
                    mode = 'CONTROLLER:   {} (Period={}ms, Synch={})'.format(
                        ctrl, CONTROLLER_APPLY_WINDOW_MS, CONTROL_SYNC)
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
                    f"{self.driver.CartPoleInstance.target_position * 100:+.1f}cm, {self.driver.CartPoleInstance.target_equilibrium:+.0f}",
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

            cached_controller_status = getattr(self.driver, "_cached_controller_status", None)
            if cached_controller_status and self.driver.controlEnabled:
                self.tcm.print_temporary(
                    BACK_TO_BEGINNING + cached_controller_status + CLEAR_LINE
                )

            self.tcm.print_to_terminal()
