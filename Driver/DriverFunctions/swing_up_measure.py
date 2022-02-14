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
from others.p_globals import (
    k, M, m, g, J_fric, M_fric, L, v_max, u_max, controlDisturbance, controlBias, TrackHalfLength
)
from DriverFunctions.csv_helpers import csv_init


NUMBER_OF_SWINGUPS = 10000
STABLE_ANGLE_RAD = 0.2
TIME_STABLE = 2
RESET_Q = 0.5

class SwingUpMeasure:
    def __init__(self, driver):
        self.driver = driver
        self.state = 'idle'
        self.Q = 0
        self.counter_swingup = 0

    def start(self):
        print('Start swing-up measurements!')
        self.state = 'start'
        self.counter_swingup = 0

    def stop(self):
        print('Stop swing-up measurements!')
        self.state = 'idle'

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
            self.Q = 0

        # Generate Starting Position
        elif self.state == 'start':
            self.Q = 0
            self.start_position = 0.95 * np.random.uniform(low=-1,high=1) * TrackHalfLength
            self.start_angle = np.random.uniform(low=0.5,high=0.8) * np.pi
            self.start_angleD = np.random.uniform(low=0.5,high=0.8) * np.pi
            self.state = 'reset'

        # Move Back to Starting Position
        elif self.state == 'reset':
            if abs(self.position - self.start_position) > 0.05:
                self.Q = -RESET_Q if position > self.start_position else RESET_Q
            if abs(self.position - self.start_position) > 0.01:
                self.Q = -RESET_Q/4 if position > self.start_position else RESET_Q/4
            else:
                self.Q = 0
                if abs(self.angle) > self.start_angle and abs(self.angleD) < self.start_angleD:
                    self.state = 'swingup'
                    self.driver.controlEnabled = True
                    self.driver.csvfilename, self.driver.csvfile, self.driver.csvwriter = csv_init(csv_name='Experiment-'+str(self.counter_swingup), controller_name=self.driver.controller.controller_name)

        # Swingup
        elif self.state == 'swingup':
            self.driver.loggingEnabled = True

            if abs(self.angle) < 0.2:
                self.time_stabilized = self.time
                self.state = 'stable'
                self.Q = self.driver.Q
            else:
                self.Q = self.driver.Q

        # Wait until 2s stable
        elif self.state == 'stable':
            self.driver.loggingEnabled = True

            if self.time - self.time_stabilized > TIME_STABLE:
                self.counter_swingup += 1
                self.driver.controlEnabled = False
                self.Q = 0

                if self.counter_swingup >= NUMBER_OF_SWINGUPS:
                    self.state = 'idle'
                else:
                    self.state = 'start'

                self.driver.csvfile.close()
                self.driver.loggingEnabled = False
            else:
                self.Q = self.driver.Q

        else:
            raise Exception(f'unknown state {self.state}')

    def __str__(self):
        if self.state == 'reset':
            return f'Swingup (State: {self.state}, Total Swingups:{self.counter_swingup}, Q:{self.Q:.2f}, Start Position={self.start_position*100:.1f}, Start Angle={self.start_angle:.1f}, Start AngleD={self.start_angleD:.1f})'
        elif self.state == 'stable':
            return f'Swingup (State: {self.state}, Total Swingups:{self.counter_swingup}, Q:{self.Q:.2f}, Time Stable={(self.time - self.time_stabilized):.2f})'
        else:
            return f'Swingup (State: {self.state}, Total Swingups:{self.counter_swingup}, Q:{self.Q:.2f})'
