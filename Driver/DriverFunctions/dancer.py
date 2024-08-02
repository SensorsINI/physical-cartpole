import time

import numpy as np

from globals import DANCE_PATH, DANCE_AMPL, DANCE_PERIOD_S, DANCE_START_TIME, DANCE_UP_AND_DOWN


class Dancer:
    def __init__(self):

        # Parameters
        self.dance_path = DANCE_PATH
        self.danceAmpl = DANCE_AMPL  # m
        self.dancePeriodS = DANCE_PERIOD_S
        self.dance_start_time = DANCE_START_TIME
        self.dance_up_and_down = DANCE_UP_AND_DOWN

        # Variables
        self.danceEnabled = False
        self.dance_finishing = False
        self.dance_current_relative_position = 0.0

        self.time_last_switch = -np.inf

    def dance_step(self, time_now, base_target_position, last_target_position, base_target_equilibrium):
        if self.danceEnabled:
            # target equilibrium part
            if not self.dance_finishing and self.dance_up_and_down:
                time_since_last_target_equilibrium_switch = time_now - self.time_last_switch
                if time_since_last_target_equilibrium_switch > 8:
                    self.time_last_switch = time_now
                    if base_target_equilibrium == 1:
                        target_equilibrium = -1
                    else:
                        target_equilibrium = 1
                else:
                    target_equilibrium = base_target_equilibrium
            else:
                target_equilibrium = base_target_equilibrium

            # target position part
            if not self.dance_finishing:
                dance_phase = np.sin(
                    2 * np.pi * ((time_now - self.dance_start_time) / self.dancePeriodS))
                if self.dance_path == 'sin':
                    self.dance_current_relative_position = self.danceAmpl * dance_phase
                elif self.dance_path == 'square':
                    self.dance_current_relative_position = self.danceAmpl * np.sign(dance_phase)
                else:
                    raise ValueError(f'Unknown dance path: {self.dance_path}')

                target_position = base_target_position + self.dance_current_relative_position
            else:  # TODO: Maybe can be removed, but it is useful for smooth dance paths with PID not to end abruptly
                if abs(base_target_position - last_target_position) < 0.03:
                    self.danceEnabled = False
                    print(f"\nself.danceEnabled= {self.danceEnabled}")
                    self.dance_finishing = False
                    target_position = base_target_position
                else:
                    target_position = 0.995 * last_target_position + 0.005 * base_target_position

            return target_position, target_equilibrium
        else:
            return base_target_position, base_target_equilibrium

    def on_off(self):

        if self.danceEnabled is True:
            self.dance_finishing = True
        else:
            self.dance_start_time = time.time()  # We want the sinusoid to start at predictable (0) position
            self.danceEnabled = True
            self.time_last_switch = -np.inf
            print(f"\nself.danceEnabled= {self.danceEnabled}")
