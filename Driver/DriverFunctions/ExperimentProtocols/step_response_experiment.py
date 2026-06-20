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

# Number of times the whole (bidirectional) Q sweep is repeated within ONE recording.
# Set >1 to accumulate many transients for the effective-mass identification
# (matched-(u,v) acceleration-ratio method needs lots of moving samples for a tight CI).
# Set back to 1 for the ordinary single-sweep motor calibration.
NUM_REPEATS = 8

# Re-run the firmware position calibration (re-zero the encoder against the
# physical track centre) after every directional sweep. The position encoder
# drifts over many repeats; without this the reset target slowly walks towards a
# track edge and the cart eventually hits it. Set False for the ordinary
# single-sweep motor calibration where it is not needed.
RECALIBRATE_BETWEEN_SERIES = True
CALIBRATION_SAFETY_TIMEOUT_S = 30  # abort the experiment if firmware calibration never reports back

# Direction for measurement.py with the cart accelerating to right:
STARTING_POSITION = -0.15  # cart starting position (in m)
ENDING_POSITION = 0.1  # position to turn off motor
RESET_Q = 0.3
SPEED_STEP = 0.05
STARTING_SPEED = 0.30  # doesn't work for low values for some reason; bumped a bit so a loaded (heavier) cart still breaks stiction
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

        self.forward_speed = None

        self.repeats_done = 0  # how many full (bidirectional) sweeps completed in this recording

        self.time_state_changed = None

    def set_up_experiment(self, first_iteration=True):

        self.motor = MOTOR
        self.motor_correction = MOTOR_CORRECTION

        self.assign_parameters()

        self.second_round = False  # In case of bidirectional measurement, detect second "round", the opposite direction
        self.repeats_done = 0

        self.time_state_changed = None
        self.forward_speed = self.starting_speed

        if ACCOUNT_FOR_MOTOR_CORRECTION:
            minimal_starting_speed = np.max((abs(self.motor_correction[1]), abs(self.motor_correction[2])))
            if abs(STARTING_SPEED - minimal_starting_speed) < 0:
                raise Exception(
                    'To small starting forward_speed ({}). When ACCOUNT_FOR_MOTOR_CORRECTION is True minimal starting forward_speed is {}'.format(
                        STARTING_SPEED, minimal_starting_speed))

        self.current_experiment_phase = 'resetting'
        self.Q = -self.reset_Q
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
        elif self.current_experiment_phase == 'calibrating':
            self.action_calibrating(time_now)
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

            if abs(self.forward_speed) < abs(self.ending_speed):
                self.forward_speed += self.speed_step
                self.forward_speed = np.clip(self.forward_speed, -abs(self.ending_speed), abs(self.ending_speed))
                self.Q = self.forward_speed
                self.current_experiment_phase = 'moving'
            else:
                # A full Q sweep (one series in the current direction) just finished.
                # Re-zero the encoder before the next series to counter drift,
                # otherwise advance straight to the next series / repeat / stop.
                if RECALIBRATE_BETWEEN_SERIES and self._another_series_follows():
                    self._begin_recalibration()
                else:
                    self._advance_after_series()

            self.time_state_changed = time_now

    def _another_series_follows(self):
        """True if at least one more directional sweep will run after the current one."""
        if BIDIRECTIONAL and not self.second_round:
            return True  # the opposite-direction sweep of this repeat is still to come
        return (self.repeats_done + 1) < NUM_REPEATS

    def _begin_recalibration(self):
        """Hand the motor to the firmware and trigger its position-calibration routine.

        Unlike driver.calibrate() this does NOT stop the running protocol/recording;
        it only re-zeros the encoder. The driver main loop gates set_motor() off while
        calibration_in_progress is True, so the firmware is free to move the cart.
        """
        self.Q = 0.0
        self.driver.InterfaceInstance.set_motor(0)
        self.driver.InterfaceInstance.pc_control_mode(False)
        self.driver.InterfaceInstance.start_calibration()
        self.current_experiment_phase = 'calibrating'
        print('\n[step-response] Re-zeroing encoder between series (firmware calibration)...')

    def action_calibrating(self, time_now):
        self.Q = 0.0
        if not self.driver.InterfaceInstance.calibration_in_progress:
            # Firmware finished; check_calibration_status() in the driver loop has
            # already applied the new zero. Re-enable PC motor commands and resume.
            self.driver.InterfaceInstance.pc_control_mode(True)
            print('[step-response] Recalibration done, resuming sweep.')
            self._advance_after_series()
            self.time_state_changed = time_now
        elif time_now - self.time_state_changed > CALIBRATION_SAFETY_TIMEOUT_S:
            print('[step-response] Calibration did not report back in time; stopping experiment.')
            self.stop()

    def _advance_after_series(self):
        """Switch direction / start the next repeat / stop after a completed sweep."""
        if BIDIRECTIONAL and (self.second_round is False):
            get_parameters_opposite_direction()
            self.assign_parameters()
            self.second_round = True
            self.forward_speed = self.starting_speed
            self.current_experiment_phase = 'resetting'
            self.Q = -self.reset_Q
        else:
            if BIDIRECTIONAL:
                get_parameters_opposite_direction()
                self.assign_parameters()
                self.second_round = False
            self.repeats_done += 1
            if self.repeats_done < NUM_REPEATS:
                # Restart a fresh full sweep (direction params are back to the
                # original orientation here) to gather more data in one recording.
                self.forward_speed = self.starting_speed
                self.current_experiment_phase = 'resetting'
                self.Q = -self.reset_Q
            else:
                self.stop()
                return

    def action_moving(self, position, time_now):
        if ((0 < ENDING_POSITION < position) or (0 > ENDING_POSITION > position)
                or self._check_timeout(time_now, raise_error=False)):
            if FRICTION_SLOWDOWN:
                self.current_experiment_phase = 'friction_slowdown'
                self.Q = 0.0
            else:
                self.current_experiment_phase = 'resetting'
                self.Q = -self.reset_Q
            self.time_state_changed = time_now

    def action_friction_slowdown(self, time_now):

        if time_now - self.time_state_changed > FRICTION_SLOWDOWN_TIME_S:
            self.current_experiment_phase = 'resetting'
            self.Q = -self.reset_Q
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
        return f' Step Response (State:{self.current_experiment_phase}, Forward Speed:{self.forward_speed}, Q:{self.Q})'
