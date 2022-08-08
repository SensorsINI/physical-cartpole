import numpy as np


sensitivity_pP_gain = -1.0
sensitivity_pI_gain = -1.0
sensitivity_pD_gain = 0.01


class controller_PID_position:
    def __init__(self):

        self.time_last = None
        ########################################################################################################

        # Position PID

        self.POSITION_TARGET = 0

        # Errors
        self.position_error = 0.0
        self.position_error_integral = 0.0
        self.position_error_diff = 0.0

        self.position_error_previous = None

        # Gains
        self.POSITION_KP = 10.0
        self.POSITION_KD = 0.0
        self.POSITION_KI = 1.0

        # "Cost" components:
        # gain * error(or error integral or error difference) * sensitivity factor (see at the top of the file)
        self.pP = 0.0
        self.pI = 0.0
        self.pD = 0.0

        # Motor command - position-PID contribution
        self.Q_position = 0.0

        # Final motor command - sum of angle-PID and position-PID motor commands
        self.Q = 0

    def step(self, position, target_position, time=None):

        self.POSITION_TARGET = target_position

        ########################################################################################################

        # Time

        if self.time_last is None:
            time_difference = 0.0
        else:
            time_difference = time - self.time_last

        # Ignore time difference if the difference very big
        # (it would harm integral gain if appears as error and otherwise the system is anyway not stable)
        if time_difference > 0.1:
            time_difference = 0.0

        self.time_last = time

        ########################################################################################################

        # Position PID

        # Error
        self.position_error = (position - target_position)

        # Error difference
        if time_difference > 0.0001 and (self.position_error_previous is not None):
            self.position_error_diff = (self.position_error - self.position_error_previous) / time_difference
        else:
            self.position_error_diff = 0.0

        self.position_error_previous = self.position_error

        # Error integral
        if self.POSITION_KI > 0.0:
            self.position_error_integral += self.position_error * time_difference
            self.position_error_integral = np.clip(self.position_error_integral, -1.0 / self.POSITION_KI,
                                                   1.0 / self.POSITION_KI)  # Makes sure pI is not bigger than 1.0. KI regulates rather the rate of then max value
        else:
            self.position_error_integral = 0.0

        # "Cost" components
        # We split the "cost" components to allow separate printing helping to understand which components are relevant
        self.pP = self.POSITION_KP * self.position_error * sensitivity_pP_gain
        self.pI = self.POSITION_KI * self.position_error_integral * sensitivity_pI_gain
        self.pD = self.POSITION_KD * self.position_error_diff * sensitivity_pD_gain

        # Motor command - position-PID contribution
        self.Q_position = self.pP + self.pI + self.pD

        self.Q = np.clip(self.Q_position, -0.8, 0.8)

        return self.Q