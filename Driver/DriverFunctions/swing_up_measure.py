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
    k, m_cart, m_pole, g, J_fric, M_fric, L, v_max, u_max, controlDisturbance, controlBias, TrackHalfLength
)
from DriverFunctions.csv_helpers import csv_init
from DriverFunctions.TargetPositionGenerator import TargetPositionGenerator
from DriverFunctions.PID_Position import controller_PID_position

NUMBER_OF_SWINGUPS = 10
STABLE_ANGLE_RAD = 0.2
TIME_STABLE = 15
RESET_Q = 0.5
TIME_STABLE_DOWN = 10.0

class SwingUpMeasure:
    def __init__(self, driver):
        self.driver = driver
        self.state = 'idle'
        self.Q = 0
        self.counter_swingup = 0
        self.TPG = TargetPositionGenerator()
        self.target_position = 0.0

        self.first_start = True

        self.PID = controller_PID_position()

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
            self.first_start = True

        # Generate Starting Position
        elif self.state == 'start':
            self.Q = self.driver.Q

            if self.first_start:
                self.driver.csvfilename, self.driver.csvfile, self.driver.csvwriter = csv_init(
                    csv_name='Experiment-' + str(self.counter_swingup),
                    controller_name=self.driver.controller.controller_name)
                self.TPG.set_experiment()
                self.first_start = False
            self.target_position = np.random.uniform(low=-0.18, high=0.18)
            self.start_angle = np.random.uniform(low=0.5,high=0.8) * np.pi
            self.start_angleD = np.random.uniform(low=0.5,high=0.8) * np.pi
            self.state = 'reset'

            self.driver.target_equilibrium = -1

        # Move Back to Starting Position
        elif self.state == 'reset':

            self.driver.target_equilibrium = -1
            self.driver.controlEnabled = True
            self.Q = self.driver.Q

            if abs(self.position - self.target_position) > 0.01\
                    and not (
                    abs(self.angle) > self.start_angle and abs(self.angleD) < self.start_angleD
            ):
                pass
            else:
                self.time_start_stable_down = self.time
                self.state = 'wait inbetween'

        elif self.state == 'wait inbetween':
            self.driver.target_equilibrium = -1
            self.driver.controlEnabled = True
            self.Q = self.driver.Q
            self.target_position = self.TPG.get_target_position()

            if self.time - self.time_start_stable_down > TIME_STABLE_DOWN:
                self.target_position = self.TPG.get_target_position()
                self.time_start = self.time
                self.state = 'swingup'
                self.driver.controlEnabled = True


        # Wait until 2s stable
        elif self.state == 'swingup':

            self.driver.target_equilibrium = 1

            self.driver.loggingEnabled = True
            self.target_position = self.TPG.get_target_position()

            if self.time - self.time_start > TIME_STABLE:
                self.counter_swingup += 1
                self.Q = self.driver.Q

                if self.counter_swingup >= NUMBER_OF_SWINGUPS:
                    self.state = 'idle'
                    self.driver.loggingEnabled = False
                    self.driver.csvfile.close()
                    self.driver.controlEnabled = False
                    self.driver.controller.controller_report()
                    self.first_start = True
                else:
                    self.state = 'start'
            else:
                self.Q = self.driver.Q

        else:
            raise Exception(f'unknown state {self.state}')

        self.driver.target_position = self.target_position

    def __str__(self):
        if self.state == 'reset':
            return f'Swingup (State: {self.state}, Total Swingups:{self.counter_swingup}, Q:{self.Q:.2f}, Start Position={self.target_position*100:.1f}, Start Angle={self.start_angle:.1f}, Start AngleD={self.start_angleD:.1f})'
        elif self.state == 'swingup':
            return f'Swingup (State: {self.state}, Total Swingups:{self.counter_swingup}, Q:{self.Q:.2f}, Time swingup={(self.time - self.time_start):.2f}, Target Position={self.target_position*100:.1f})'
        else:
            return f'Swingup (State: {self.state}, Total Swingups:{self.counter_swingup}, Q:{self.Q:.2f})'
