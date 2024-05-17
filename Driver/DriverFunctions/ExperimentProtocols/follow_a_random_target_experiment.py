# measurements from cartpole, controlled by state machine.
# control.py calls update_state() if state is not 'idle'
from DriverFunctions.ExperimentProtocols import template_experiment_protocol
import time
from globals import *
from CartPole.state_utilities import (
    ANGLE_IDX,
    ANGLED_IDX,
    POSITION_IDX,
    POSITIOND_IDX,
)
import numpy as np

NUMBER_OF_RANDOM_TARGETS = 20
STABLE_POSITION_TOLERANCE = 0.015
TIME_STABLE = 1.0
TARGET_SMOOTHING = 0.995


class follow_a_random_target_experiment(template_experiment_protocol):
    def __init__(self, driver):
        super().__init__(
            driver=driver,
            experiment_protocol_name=self.__class__.__name__[:-len('_experiment')],)

        self.counter_target_position = 0
        self.cycle_counter = 0

        self.current_target = 0.0
        self.current_target_filtered = 0.0
        self.target_position = 0.0

        self.angle = 0.0
        self.angleD = 0.0
        self.position = 0.0
        self.positionD = 0.0
        self.time = 0.0

        self.start_angle = 0.0
        self.start_angleD = 0.0

    def set_up_experiment(self, first_iteration=True):
        self.current_target = 0.0
        self.current_target_filtered = 0.0
        self.counter_target_position = 0
        self.target_position = self.driver.target_position
        self.start_new_recording()
        self.driver.controlEnabled = True
        self.current_experiment_phase = 'reset'

    def update_state(self, angle, position, time):
        self.angle = angle
        self.angleD = self.driver.s[ANGLED_IDX]
        self.position = position
        self.positionD = self.driver.s[POSITIOND_IDX]
        self.time = time

        if self.current_experiment_phase == 'idle':
            pass

        # Move Back to Starting Position
        elif self.current_experiment_phase == 'reset':
            self.current_target = 0.15 * np.random.uniform(low=-1, high=1)
            self.start_angle = self.driver.s[ANGLE_IDX]
            self.start_angleD = self.driver.s[ANGLED_IDX]
            self.current_target_filtered = TARGET_SMOOTHING*self.current_target_filtered + (1-TARGET_SMOOTHING)*self.current_target
            self.target_position = self.current_target_filtered
            self.current_experiment_phase = 'go-to-target'


        # Swingup
        elif self.current_experiment_phase == 'go-to-target':

            self.current_target_filtered = TARGET_SMOOTHING*self.current_target_filtered + (1-TARGET_SMOOTHING)*self.current_target
            self.target_position = self.current_target_filtered

            if abs(self.position-self.current_target) < STABLE_POSITION_TOLERANCE:
                self.time_stabilized = self.time
                self.current_experiment_phase = 'stable'

        # Wait until 2s stable
        elif self.current_experiment_phase == 'stable':

            self.current_target_filtered = TARGET_SMOOTHING*self.current_target_filtered + (1-TARGET_SMOOTHING)*self.current_target
            self.target_position = self.current_target_filtered

            if self.time - self.time_stabilized > TIME_STABLE:
                self.counter_target_position += 1
                self.driver.controlEnabled = True
                # self.driver.controller.controller_report()

                if self.counter_target_position >= NUMBER_OF_RANDOM_TARGETS:
                    self.cycle_counter += 1
                    self.finish_recording()
                    self.set_up_experiment(first_iteration=False)
                else:
                    self.current_experiment_phase = 'reset'

            else:
                if abs(self.position-self.current_target) > STABLE_POSITION_TOLERANCE:
                    self.current_experiment_phase = 'go-to-target'

        else:
            raise Exception(f'unknown state {self.current_experiment_phase}')

    def __str__(self):
        if self.current_experiment_phase == 'reset':
            return f'Go-to-target (State: {self.current_experiment_phase}, Total targets:{self.counter_target_position}, Q:{self.driver.Q:.2f}, Start Position={self.current_target*100:.1f}, Start Angle={self.start_angle:.1f}, Start AngleD={self.start_angleD:.1f})'
        elif self.current_experiment_phase == 'stable':
            return f'Go-to-target (State: {self.current_experiment_phase}, Total targets:{self.counter_target_position}, Q:{self.driver.Q:.2f}, Time Stable={(self.time - self.time_stabilized):.2f})'
        else:
            return f'Go-to-target (State: {self.current_experiment_phase}, Total targets:{self.counter_target_position}, Q:{self.driver.Q:.2f})'
