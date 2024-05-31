# measurements from cartpole, controlled by state machine.
# control.py calls update_state() if state is not 'idle'
import numpy as np
from DriverFunctions.ExperimentProtocols import template_experiment_protocol
import time
from globals import MOTOR, MOTOR_CORRECTION, CORRECT_MOTOR_DYNAMICS
from CartPoleSimulation.CartPole.state_utilities import (
    ANGLE_IDX,
    ANGLED_IDX,
    POSITION_IDX,
    POSITIOND_IDX,
)

ACCOUNT_FOR_MOTOR_CORRECTION = CORRECT_MOTOR_DYNAMICS
ACCELERATE_FIRST_TO_LEFT = True
BIDIRECTIONAL = True
FRICTION_SLOWDOWN = False

# Direction for measurement.py with the cart accelerating to right:
STARTING_POSITION = -0.15  # cart starting position (in m)
ENDING_POSITION = 0.1  # position to turn off motor
RESET_Q = 0.3
SPEED_STEP = 0.1
STARTING_SPEED = 0.25  # doesn't work for low values for some reason
ENDING_SPEED = 0.95
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


class step_response_experiment(template_experiment_protocol):
    def __init__(self, driver):
        super().__init__(
            driver=driver,
            experiment_protocol_name=self.__class__.__name__[:-len('_experiment')],)

        self.motor = None
        self.motor_correction = None

        if ACCELERATE_FIRST_TO_LEFT:
            get_parameters_opposite_direction()

        self.reset_Q = None
        self.speed_step = None
        self.starting_speed = None
        self.ending_speed = None

        self.second_round = False  # In case of bidirectional measurement, detect second "round", the opposite direction

        self.speed = self.reset_Q

        self.time_state_changed = None

    def set_up_experiment(self, first_iteration=True):

        self.motor = MOTOR
        self.motor_correction = MOTOR_CORRECTION

        self.assign_parameters()

        self.second_round = False  # In case of bidirectional measurement, detect second "round", the opposite direction


        self.time_state_changed = None
        self.speed = self.starting_speed

        if ACCOUNT_FOR_MOTOR_CORRECTION:
            minimal_starting_speed = np.max((abs(self.motor_correction[1]), abs(self.motor_correction[2])))
            if abs(STARTING_SPEED - minimal_starting_speed) < 0:
                raise Exception(
                    'To small starting speed ({}). When ACCOUNT_FOR_MOTOR_CORRECTION is True minimal starting speed is {}'.format(
                        STARTING_SPEED, minimal_starting_speed))

        self.current_experiment_phase = 'resetting'
        self.start_new_recording()

    def update_state(self, angle: int, position: int, time_now: float):
        if self.current_experiment_phase == 'idle':
            pass
        elif self.current_experiment_phase == 'resetting':  # moving back to start
            self.action_resetting(position, time_now)
        elif self.current_experiment_phase == 'pausing_before_step':
            self.action_pausing_before_step(time_now)
        elif self.current_experiment_phase == 'starting_step':
            self.action_starting_step(time_now)
        elif self.current_experiment_phase == 'moving':
            self.action_moving(position, time_now)
        elif self.current_experiment_phase == 'friction_slowdown':
            self.action_friction_slowdown(time_now)
        else:
            raise Exception(f'unknown state {self.current_experiment_phase}')

        self.data_to_save_measurement = {'measurement': self}

    def action_resetting(self, position, time_now):
        if self.time_state_changed is None:
            self.time_state_changed = time_now
        if (STARTING_POSITION < 0 and position > STARTING_POSITION) or (
                STARTING_POSITION > 0 and position < STARTING_POSITION):
            self.Q = -self.reset_Q
        else:
            self.Q = 0.0
            self.current_experiment_phase = 'pausing_before_step'
            self.time_state_changed = time_now
        self._check_timeout(time_now, raise_error=True)

    def action_pausing_before_step(self, time_now):
        if time_now - self.time_state_changed > PAUSE_BEFORE_STEP_S:
            self.current_experiment_phase = 'starting_step'
            self.time_state_changed = time_now

    def action_starting_step(self, time_now):
        if abs(self.speed) < abs(self.ending_speed):
            self.speed += self.speed_step
            print(self.speed)
            self.Q = self.speed
            self.current_experiment_phase = 'moving'
            self.time_state_changed = time_now
        else:
            if BIDIRECTIONAL and (self.second_round is False):
                get_parameters_opposite_direction()
                self.assign_parameters()
                self.second_round = True
                self.set_up_experiment(first_iteration=False)
            else:
                if BIDIRECTIONAL:
                    get_parameters_opposite_direction()
                    self.assign_parameters()
                    self.second_round = False
                self.current_experiment_phase = 'idle'

            self.speed = 0
            self.Q = 0.0
            self.time_state_changed = time_now

    def action_moving(self, position, time_now):
        if (0 < ENDING_POSITION < position) or (0 > ENDING_POSITION > position):
            self.Q = 0.0
            if FRICTION_SLOWDOWN:
                self.current_experiment_phase = 'friction_slowdown'
            else:
                self.current_experiment_phase = 'resetting'
            self.time_state_changed = time_now
        if self._check_timeout(time_now, raise_error=False):
            self.Q = 0.0
            if FRICTION_SLOWDOWN:
                self.current_experiment_phase = 'friction_slowdown'
            else:
                self.current_experiment_phase = 'resetting'

    def action_friction_slowdown(self, time_now):
        self.Q = 0.0
        if time_now - self.time_state_changed > FRICTION_SLOWDOWN_TIME_S:
            self.current_experiment_phase = 'resetting'
            self.time_state_changed = time_now




    def rescale_motor_command(self, Q):
        if Q > 0:
            return (Q - self.motor_correction[1]) / self.motor_correction[0]
        else:
            return (Q + self.motor_correction[2]) / self.motor_correction[0]


    def assign_parameters(self):

        if ACCOUNT_FOR_MOTOR_CORRECTION:
            self.reset_Q = self.rescale_motor_command(RESET_Q)
            self.speed_step = SPEED_STEP / self.motor_correction[0]
            self.starting_speed = self.rescale_motor_command(STARTING_SPEED)
            self.ending_speed = self.rescale_motor_command(ENDING_SPEED)
        else:
            self.reset_Q = RESET_Q
            self.speed_step = SPEED_STEP
            self.starting_speed = STARTING_SPEED
            self.ending_speed = ENDING_SPEED

    def _check_timeout(self, time_now, raise_error=False):
        if time_now - self.time_state_changed > STEP_TIMEOUT_S:
            if raise_error:
                self.stop()
                raise TimeoutError(f'step took more than {STEP_TIMEOUT_S} to complete')
            else:
                return True
        else:
            return False

    def __str__(self):
        return f' Step Response (State:{self.current_experiment_phase}, Speed:{self.speed}, Q:{self.Q})'
