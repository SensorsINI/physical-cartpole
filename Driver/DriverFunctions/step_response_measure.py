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

ACCELERATE_FIRST_TO_LEFT = True
BIDIRECTIONAL = True
FRICTION_SLOWDOWN = False

# Direction for measurement.py with the cart accelerating to right:
STARTING_POSITION = -0.15  # cart starting position (in m)
ENDING_POSITION = 0.05  # position to turn off motor
RESET_Q = 0.5
SPEED_STEP = 0.05
STARTING_SPEED = 0.1  # doesn't work for low values for some reason
ENDING_SPEED = 1.5
PAUSE_BEFORE_STEP_S = .5  # pause after reset to start position before starting step
FRICTION_SLOWDOWN_TIME_S = 1  # time at end to just turn off motor and glide to stop
STEP_TIMEOUT_S = 10


# Call this function if you wish cart to accelerate to left instead
def get_parameters_opposite_direction():
    global STARTING_POSITION, ENDING_POSITION, RESET_Q, SPEED_STEP, STARTING_SPEED, ENDING_SPEED

    STARTING_POSITION = -STARTING_POSITION
    ENDING_POSITION = -ENDING_POSITION
    RESET_Q = -RESET_Q
    SPEED_STEP = -SPEED_STEP
    STARTING_SPEED = -STARTING_SPEED
    ENDING_SPEED = -ENDING_SPEED


class StepResponseMeasurement:
    def __init__(self):

        if ACCELERATE_FIRST_TO_LEFT:
            get_parameters_opposite_direction()

        self.second_round = False  # In case of bidirectional measurement, detect second "round", the opposite direction

        self.state = 'idle'
        self.speed = RESET_Q
        self.Q = 0.0
        self.time_state_changed = time.time()

    def start(self):
        print('Start measurement!')
        self.state = 'start'
        self.Q = 0.0

    def stop(self):
        print('Stop measurement!')
        self.Q = 0.0
        self.state = 'idle'

    def get_state(self):
        return self.state

    def is_idle(self):
        return self.state == 'idle'

    def is_running(self):
        return not self.is_idle()

    def update_state(self, angle: int, position: int, time: float):
        if self.state == 'idle':
            self.Q = 0.0
            return
        elif self.state == 'start':  # init measurement
            self.speed = STARTING_SPEED
            self.state = 'resetting'
            self.time_state_changed = time
        elif self.state == 'resetting':  # moving back to start
            if (STARTING_POSITION < 0 and position > STARTING_POSITION) or (
                    STARTING_POSITION > 0 and position < STARTING_POSITION):
                self.Q = -RESET_Q
            else:
                self.Q = 0.0
                self.state = 'pausing_before_step'
                self.time_state_changed = time
            self._check_timeout(time, raise_error=True)
        elif self.state == 'pausing_before_step':
            if time - self.time_state_changed > PAUSE_BEFORE_STEP_S:
                self.state = 'starting_step'
                self.time_state_changed = time
        elif self.state == 'starting_step':
            if abs(self.speed) < abs(ENDING_SPEED):
                self.speed += SPEED_STEP
                self.Q = self.speed
                self.state = 'moving'
                self.time_state_changed = time
            else:
                if BIDIRECTIONAL and (self.second_round is False):
                    get_parameters_opposite_direction()
                    self.second_round = True
                    self.state = 'start'
                else:
                    if BIDIRECTIONAL:
                        get_parameters_opposite_direction()
                        self.second_round = False
                    self.state = 'idle'

                self.speed = 0
                self.Q = 0.0
                self.time_state_changed = time
        elif self.state == 'moving':
            if (0 < ENDING_POSITION < position) or (0 > ENDING_POSITION > position):
                self.Q = 0.0
                if FRICTION_SLOWDOWN:
                    self.state = 'friction_slowdown'
                else:
                    self.state = 'resetting'
                self.time_state_changed = time
            if self._check_timeout(time, raise_error=False):
                self.Q = 0.0
                if FRICTION_SLOWDOWN:
                    self.state = 'friction_slowdown'
                else:
                    self.state = 'resetting'
        elif self.state == 'friction_slowdown':
            self.Q = 0.0
            if time - self.time_state_changed > FRICTION_SLOWDOWN_TIME_S:
                self.state = 'resetting'
                self.time_state_changed = time
        else:
            raise Exception(f'unknown state {self.state}')

    def _check_timeout(self, time, raise_error=False):
        if time - self.time_state_changed > STEP_TIMEOUT_S:
            if raise_error:
                self.stop()
                raise TimeoutError(f'step took more than {STEP_TIMEOUT_S} to complete')
            else:
                return True
        else:
            return False

    def __str__(self):
        return f' Step Response (State:{self.state}, Speed:{self.speed}, Q:{self.Q})'
