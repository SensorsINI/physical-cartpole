# measurements from cartpole, controlled by state machine.
# control.py calls update_state() if state is not 'idle'
import time
from globals import *
from CartPole.state_utilities import (
    ANGLE_IDX,
    ANGLED_IDX,
    POSITION_IDX,
    POSITIOND_IDX,
)
import numpy as np

from DriverFunctions.csv_helpers import csv_init


NUMBER_OF_RANDOM_TARGETS = 20
STABLE_POSITION_TOLERANCE = 0.015
TIME_STABLE = 1.0
TARGET_SMOOTHING = 0.995

class RandomTargetMeasure:
    def __init__(self, driver):
        self.driver = driver
        self.state = 'idle'
        self.Q = 0
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

    def start(self):
        print('Start follow-the-target measurements!')
        self.state = 'start'

    def stop(self):
        print('Stop follow-the-target measurements!')
        self.state = 'idle'
        self.driver.loggingEnabled = False

    def is_idle(self):
        return self.state == 'idle'

    def is_running(self):
        return not self.is_idle()

    def update_state(self, angle, position, time):
        self.angle = angle
        self.angleD = self.driver.s[ANGLED_IDX]
        self.position = position
        self.positionD = self.driver.s[POSITIOND_IDX]
        self.time = time

        if self.state == 'idle':
            self.Q = self.driver.Q
            self.current_target = 0.0
            self.current_target_filtered = 0.0
            self.target_position = self.driver.target_position

        # Generate Starting Position
        elif self.state == 'start':
            self.state = 'reset'
            self.Q = self.driver.Q
            self.counter_target_position = 0
            self.target_position = self.driver.target_position
            self.driver.controlEnabled = True
            self.driver.csvfilename, self.driver.csvfile, self.driver.csvwriter = csv_init(csv_name='Experiment-'+str(self.cycle_counter), controller_name=self.driver.controller.controller_name)

        # Move Back to Starting Position
        elif self.state == 'reset':
            self.Q = self.driver.Q
            self.current_target = 0.15 * np.random.uniform(low=-1, high=1)
            self.start_angle = self.driver.s[ANGLE_IDX]
            self.start_angleD = self.driver.s[ANGLED_IDX]
            self.current_target_filtered = TARGET_SMOOTHING*self.current_target_filtered + (1-TARGET_SMOOTHING)*self.current_target
            self.target_position = self.current_target_filtered
            self.state = 'go-to-target'


        # Swingup
        elif self.state == 'go-to-target':

            self.driver.loggingEnabled = True

            self.current_target_filtered = TARGET_SMOOTHING*self.current_target_filtered + (1-TARGET_SMOOTHING)*self.current_target
            self.target_position = self.current_target_filtered

            if abs(self.position-self.current_target) < STABLE_POSITION_TOLERANCE:
                self.time_stabilized = self.time
                self.state = 'stable'
                self.Q = self.driver.Q
            else:
                self.Q = self.driver.Q

        # Wait until 2s stable
        elif self.state == 'stable':
            self.driver.loggingEnabled = True

            self.current_target_filtered = TARGET_SMOOTHING*self.current_target_filtered + (1-TARGET_SMOOTHING)*self.current_target
            self.target_position = self.current_target_filtered

            if self.time - self.time_stabilized > TIME_STABLE:
                self.counter_target_position += 1
                self.driver.controlEnabled = True
                self.Q = self.driver.Q
                # self.driver.controller.controller_report()

                if self.counter_target_position >= NUMBER_OF_RANDOM_TARGETS:
                    self.state = 'start'
                    self.driver.loggingEnabled = False
                    self.driver.csvfile.close()
                    self.cycle_counter += 1
                else:
                    self.state = 'reset'

            else:
                if abs(self.position-self.current_target) > STABLE_POSITION_TOLERANCE:
                    self.state = 'go-to-target'
                self.Q = self.driver.Q

        else:
            raise Exception(f'unknown state {self.state}')

    def __str__(self):
        if self.state == 'reset':
            return f'Go-to-target (State: {self.state}, Total targets:{self.counter_target_position}, Q:{self.Q:.2f}, Start Position={self.current_target*100:.1f}, Start Angle={self.start_angle:.1f}, Start AngleD={self.start_angleD:.1f})'
        elif self.state == 'stable':
            return f'Go-to-target (State: {self.state}, Total targets:{self.counter_target_position}, Q:{self.Q:.2f}, Time Stable={(self.time - self.time_stabilized):.2f})'
        else:
            return f'Go-to-target (State: {self.state}, Total targets:{self.counter_target_position}, Q:{self.Q:.2f})'
