# measurements from cartpole, controlled by state machine.
# control.py calls update_state() if state is not 'idle'
import numpy as np
from DriverFunctions.ExperimentProtocols import template_experiment_protocol
from CartPole.state_utilities import (
    ANGLE_IDX,
    ANGLED_IDX,
    POSITION_IDX,
    POSITIOND_IDX,
)

NUMBER_OF_ITERATIONS = 10

TARGET_POSITION_0 = 0.0
TARGET_POSITION_SWING_UP = 0.0
TARGET_POSITION_1 = 0.09
TARGET_POSITION_2 = -0.09

TIME_FOR_SWINGUP = 10.0
TIME_FOR_TARGET_1 = 15.0
TIME_OF_EXPERIMENT = 20.0

RECALIBRATE_EVERY_N_SWING_UPS = None

FIRMWARE_CONTROL = False

SKIP_RESET = False  # Use for PID to avoid


class iros24_ex1_experiment(template_experiment_protocol):
    def __init__(self, driver):
        super().__init__(
            driver=driver,
            experiment_protocol_name=self.__class__.__name__[:-len('_experiment')], )

        self.counter_iterations = 0

    def set_up_experiment(self, first_iteration=True):
        if first_iteration:
            self.counter_iterations = 0

        if RECALIBRATE_EVERY_N_SWING_UPS is not None and self.counter_iterations % RECALIBRATE_EVERY_N_SWING_UPS == 0:
            print("\nCalibrating motor position.... ")
            self.driver.InterfaceInstance.calibrate()
            print("Done calibrating")

        self.target_position = TARGET_POSITION_0
        self.target_equilibrium = -1

        if SKIP_RESET:
            self.start_new_recording(index=self.counter_iterations)
            self.target_position = TARGET_POSITION_SWING_UP
            self.target_equilibrium = 1.0
            self.time_start_stable_down = None
            self.current_experiment_phase = 'swingup'
        else:
            self.current_experiment_phase = 'reset'

        if FIRMWARE_CONTROL:
            self.driver.firmwareControl = True
            self.driver.InterfaceInstance.set_target_position(self.target_position)
            self.driver.InterfaceInstance.set_target_equilibrium(self.target_equilibrium)
            self.driver.InterfaceInstance.control_mode(self.driver.firmwareControl)
        else:
            self.driver.controlEnabled = True  # We are not enabling control automatically in case we want to use hardware controller and only provide target position and equilibrium from PC



    def stop(self):
        super().stop()
        self.driver.firmwareControl = False
        self.driver.InterfaceInstance.control_mode(self.driver.firmwareControl)
    def update_state(self, angle, position, time):
        self.angle = angle
        self.angleD = self.driver.s[ANGLED_IDX]
        self.position = position
        self.positionD = self.driver.s[POSITIOND_IDX]
        self.time = time

        if self.current_experiment_phase == 'idle':
            pass
        elif self.current_experiment_phase == 'reset':
            self.action_reset()
        elif self.current_experiment_phase == 'swingup':
            self.action_swing_up()
        elif self.current_experiment_phase == 'go-to-target_1':
            self.action_go_to_target_1()
        elif self.current_experiment_phase == 'go-to-target_2':
            self.action_go_to_target_2()
        else:
            raise Exception(f'unknown experiment phase: {self.current_experiment_phase}')


    def action_reset(self):

        if (abs(self.position - self.target_position) < 0.01
                and
                abs(self.angle) > np.pi-0.1
                # and abs(self.angleD) < 0.1
        ):
            self.start_new_recording(index=self.counter_iterations)
            self.target_position = TARGET_POSITION_SWING_UP
            self.target_equilibrium = 1.0
            self.time_start_stable_down = self.time
            self.current_experiment_phase = 'swingup'


    def action_swing_up(self):
        if self.time_start_stable_down is None:
            self.time_start_stable_down = self.time
        if self.time - self.time_start_stable_down >= TIME_FOR_SWINGUP:
            self.target_position = TARGET_POSITION_1
            self.current_experiment_phase = 'go-to-target_1'

    def action_go_to_target_1(self):
        if self.time - self.time_start_stable_down > TIME_FOR_TARGET_1:
            self.target_position = TARGET_POSITION_2
            self.current_experiment_phase = 'go-to-target_2'

    def action_go_to_target_2(self):
        if self.time - self.time_start_stable_down > TIME_OF_EXPERIMENT:
            self.counter_iterations += 1
            self.finish_recording()
            if self.counter_iterations >= NUMBER_OF_ITERATIONS:
                self.stop()
                self.driver.controller.controller_report()
            else:
                self.set_up_experiment(first_iteration=False)

    def __str__(self):
        return f'IROS Exp1 (Experiment Phase: {self.current_experiment_phase})'
