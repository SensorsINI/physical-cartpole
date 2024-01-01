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

RUN_TIME = 15.0  # Test time in seconds
PAUSE_TIME = 60.0
TOTAL_RUNS = 100

class SetTimeMeasure:
    def __init__(self, driver):
        self.driver = driver
        self.state = 'idle'
        self.Q = 0
        self.run_num = 0
        self.initial_run = True

    def start(self):
        print('Start set-time measurements!')
        self.state = 'reset'

    def stop(self):
        print('Stop set-time measurements!')
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

        if self.state == 'idle' or self.state == 'stop':
            self.Q = 0

        # Move Back to Starting Position
        elif self.state == 'reset':
            self.controlEnabled = False

            self.Q = self.driver.Q
            self.start_angle = self.driver.s[ANGLE_IDX]
            self.start_angleD = self.driver.s[ANGLED_IDX]
            self.target_position = self.driver.target_position

            global MOTOR, ANGLE_DEVIATION

            print("\nCalibrating motor position.... ")
            self.driver.InterfaceInstance.calibrate()
            (_, _, self.position_offset, _, _, _, _, _) = self.driver.InterfaceInstance.read_state()
            print("Done calibrating")

            if self.driver.InterfaceInstance.encoderDirection == 1:
                MOTOR = 'POLOLU'
                if ANGLE_HANGING_DEFAULT:
                    ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_POLOLU)
            elif self.driver.InterfaceInstance.encoderDirection == -1:
                MOTOR = 'ORIGINAL'
                if ANGLE_HANGING_DEFAULT:
                    ANGLE_DEVIATION[...] = angle_constants_update(ANGLE_HANGING_ORIGINAL)
            else:
                raise ValueError('Unexpected value for self.InterfaceInstance.encoderDirection = '.format(
                    self.driver.InterfaceInstance.encoderDirection))
            print('Detected motor: {}'.format(MOTOR))

            self.start_time = self.time
            self.state = 'pause'

        elif self.state == 'pause':
            self.Q = self.driver.Q

            if self.run_num > TOTAL_RUNS:
                self.state = 'stop'

            if (self.time - self.start_time) > PAUSE_TIME:
                if not self.initial_run:
                    self.driver.loggingEnabled = True
                    self.driver.csvfilename, self.driver.csvfile, self.driver.csvwriter = csv_init(
                        csv_name='Experiment-' + self.driver.controller.controller_name,
                        controller_name=self.driver.controller.controller_name)

                self.driver.controlEnabled = True
                self.start_time = self.time
                self.state = 'run'

        elif self.state == 'run':
            self.Q = self.driver.Q

            if (self.time - self.start_time) > RUN_TIME:
                self.driver.controlEnabled = False
                self.run_num += 1
                self.Q = self.driver.Q

                if not self.initial_run:
                    self.driver.loggingEnabled = False
                    self.driver.csvfile.close()
                else:
                    self.initial_run = False

                self.state = 'reset'

        else:
            raise Exception(f'unknown state {self.state}')

    def __str__(self):
        if self.state == 'reset':
            return f'Set-time (State: {self.state}, Q:{self.Q:.2f}, Start Angle={self.start_angle:.1f}, Start AngleD={self.start_angleD:.1f})\nRUN: {self.run_num}'
        elif self.state == 'run':
            return f'Set-time (State: {self.state}, Q:{self.Q:.2f}, Time={self.time:.2f})\nRUN: {self.run_num}'
        elif self.state == 'stop':
            return f'Set-time (State: {self.state}, Q:{self.Q:.2f}, Time={self.time:.2f})\nDONE!'
        else:
            return f'Set-time (State: {self.state}, Q:{self.Q:.2f})\nRUN: {self.run_num}'

