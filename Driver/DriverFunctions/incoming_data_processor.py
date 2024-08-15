import time

import numpy as np

from tqdm import trange

from CartPoleSimulation.CartPole._CartPole_mathematical_helpers import wrap_angle_rad
from CartPoleSimulation.CartPole.state_utilities import (ANGLE_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX, ANGLED_IDX,
                                                         POSITION_IDX, POSITIOND_IDX)

from globals import (ANGLE_DEVIATION, ANGLE_360_DEG_IN_ADC_UNITS,
                     ANGLE_NORMALIZATION_FACTOR, POSITION_NORMALIZATION_FACTOR,
                     CONTROL_PERIOD_MS,
                     TIMESTEPS_FOR_DERIVATIVE,
                     ANGLE_D_MEDIAN_LEN, POSITION_D_MEDIAN_LEN)


class IncomingDataProcessor:
    def __init__(self):

        self.angle_raw = 0
        self.angleD_raw = 0
        self.angle_raw_stable = 0.0
        self.angleD_raw_stable = 0.0
        self.last_difference = None

        self.angle_raw_sensor = None
        self.angleD_raw_sensor = None
        self.invalid_steps = 0
        self.freezme = 0
        self.position_raw = 0
        self.positionD_raw = 0

        self.angle_deviation_finetune = 0.0

        # Derivative calculation
        self.angle_history = [-1] * (TIMESTEPS_FOR_DERIVATIVE + 1)  # Buffer to store past angles
        self.position_history = [-1] * (TIMESTEPS_FOR_DERIVATIVE + 1)  # Buffer to store past positions
        self.frozen_history = [0] * (TIMESTEPS_FOR_DERIVATIVE + 1)  # Buffer to store frozen states
        self.idx_for_derivative_calculation = 0
        self.idx_for_derivative_calculation_position = 0

        self.angleD_buffer = np.zeros(ANGLE_D_MEDIAN_LEN, dtype=np.float32)  # Buffer for angle derivatives
        self.positionD_buffer = np.zeros(POSITION_D_MEDIAN_LEN, dtype=np.float32)  # Buffer for position derivatives
        self.angleD_median_buffer_index = 0
        self.positionD_median_buffer_index = 0

    def load_state_data_from_chip(self, angle_raw, angleD_raw, invalid_steps, position_raw):
        self.angle_raw = angle_raw
        self.angleD_raw = angleD_raw
        self.invalid_steps = invalid_steps
        self.position_raw = position_raw

    def process_state_information(self, s, time_between_measurements_chip):

        # self.treat_deadangle_with_derivative()

        self.position_difference()

        self.angle_raw_sensor = self.angle_raw
        self.angleD_raw_sensor = self.angleD_raw

        self.filter_differences()

        angle, position, angle_difference, position_difference = self.convert_angle_and_position_skale()

        angleDerivative, positionDerivative = self.calculate_first_derivatives(
            angle_difference,
            position_difference,
            time_between_measurements_chip
        )

        # Pack the state into interface acceptable for the self.controller
        self.pack_features_into_state_variable(s, position, angle, positionDerivative, angleDerivative)

    def treat_deadangle_with_derivative(self):

        """
        This function tries to treat the dead angle of the potentiometer.
        It tries to detect an invalid measurement in dead angle region by unusually high angular acceleration.
        The threshold and its dependence on CONTROL_PERIOD_MS and CONTROL_PERIOD_MS is heuristically guessed.
        After detecting an invalid measurement
        the function freezes the derivative and dead reckon the angle for a fixed number of measurement cycles.
        It freezes for longer when the pole goes through the dead angle upwards (decelerates).
        TODO: Making the duration of the freeze dependent on the angular velocity
            could be helpful to increase performance.
        More principled approach is welcomed.

        The invalid steps are number of corrupted measurements in the buffer for angle averaging in firmware
        see "anomaly_detection" function in firmware
        This is useful in STM where averaging is done in firmware after oversampling the angle measurement.
        TODO: In Zynq, the averaging is done in hardware, and counting invalid steps should be implemented there

        Now treating deadangle is done only on firmware.
        """

        # Calculate the index for the k-th past angle
        kth_past_index = (self.idx_for_derivative_calculation + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)
        kth_past_angle = self.angle_history[kth_past_index]

        current_difference = self.wrap_local(
            self.angle_raw - kth_past_angle) / TIMESTEPS_FOR_DERIVATIVE if kth_past_angle != -1 else 0

        if self.last_difference is None:
            self.last_difference = current_difference

        if (
                kth_past_angle != -1
                and
                (self.angle_raw_stable > 3500 or self.angle_raw_stable < 500)
                and
                self.freezme == 0
                and
                (
                        TIMESTEPS_FOR_DERIVATIVE * abs(
                            current_difference - self.last_difference) > CONTROL_PERIOD_MS * 2.4
                        or
                        # This last line is for STM32, not tested nor reworked at last revision of this function
                        # Just removed the abs(self.wrap_local(kth_past_angle)) < ADC_RANGE / 20
                        # as this seems to me to be covered by self.angle_raw_stable > 3500 or self.angle_raw_stable < 500
                        self.invalid_steps > 5
                )
        ):

            if self.angleD_raw_stable > 0:
                self.freezme = int(
                    45 / CONTROL_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE  # Accelerates through the dead angle
            else:
                self.freezme = int(
                    90 / CONTROL_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE  # Deccelerates through the dead angle

        if self.freezme > 0:
            self.freezme -= 1
            self.angleD_raw = self.angleD_raw_stable
            if self.freezme > TIMESTEPS_FOR_DERIVATIVE + 1:
                self.angle_raw_stable += self.angleD_raw_stable
                self.angle_raw = self.wrap_local(self.angle_raw_stable)
            else:
                self.angle_raw_stable = self.angle_raw
        else:
            self.angle_raw_stable = self.angle_raw
            self.angleD_raw = current_difference
            self.angleD_raw_stable = self.angleD_raw

        self.last_difference = current_difference

        # Save current angle in the history buffer and update index
        self.angle_history[self.idx_for_derivative_calculation] = self.angle_raw
        self.frozen_history[self.idx_for_derivative_calculation] = self.freezme
        self.idx_for_derivative_calculation = (self.idx_for_derivative_calculation + 1) % (
                TIMESTEPS_FOR_DERIVATIVE + 1)  # Move to next index, wrap around if necessary

    def position_difference(self):

        # Calculate the index for the k-th past angle
        kth_past_index = (self.idx_for_derivative_calculation_position + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)
        kth_past_position = self.position_history[kth_past_index]

        self.positionD_raw = (
                                     self.position_raw - kth_past_position) / TIMESTEPS_FOR_DERIVATIVE if kth_past_position != -1 else 0

        # Save current angle in the history buffer and update index
        self.position_history[self.idx_for_derivative_calculation_position] = self.position_raw
        self.idx_for_derivative_calculation_position = (self.idx_for_derivative_calculation_position + 1) % (
                TIMESTEPS_FOR_DERIVATIVE + 1)  # Move to next index, wrap around if necessary

    def filter_differences(self):

        # Update angleD buffer with current value
        self.angleD_buffer[self.angleD_median_buffer_index] = self.angleD_raw
        self.angleD_median_buffer_index = (self.angleD_median_buffer_index + 1) % ANGLE_D_MEDIAN_LEN

        # Update positionD buffer with current value
        self.positionD_buffer[self.positionD_median_buffer_index] = self.positionD_raw
        self.positionD_median_buffer_index = (self.positionD_median_buffer_index + 1) % POSITION_D_MEDIAN_LEN

        # Calculate medians using the updated buffers
        angle_d_median = np.median(self.angleD_buffer)
        position_d_median = np.median(self.positionD_buffer)

        self.angleD_raw = angle_d_median
        self.positionD_raw = position_d_median

    def convert_angle_and_position_skale(self):
        # Convert position and angle to physical units
        angle = wrap_angle_rad(
            (self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - self.angle_deviation_finetune)
        position = self.position_raw * POSITION_NORMALIZATION_FACTOR

        angle_difference = self.angleD_raw * ANGLE_NORMALIZATION_FACTOR
        position_difference = self.positionD_raw * POSITION_NORMALIZATION_FACTOR

        return angle, position, angle_difference, position_difference

    @staticmethod
    def calculate_first_derivatives(angle_difference, position_difference, time_between_measurements_chip):
        # Calculating derivatives (cart velocity and angular velocity of the pole)
        angleDerivative = angle_difference / time_between_measurements_chip  # rad/self.s
        positionDerivative = position_difference / time_between_measurements_chip  # m/self.s

        return angleDerivative, positionDerivative

    @staticmethod
    def pack_features_into_state_variable(s, position, angle, positionD, angleD):
        # Pack the state into interface acceptable for the self.controller
        s[POSITION_IDX] = position
        s[ANGLE_IDX] = angle
        s[POSITIOND_IDX] = positionD
        s[ANGLED_IDX] = angleD
        s[ANGLE_COS_IDX] = np.cos(angle)
        s[ANGLE_SIN_IDX] = np.sin(angle)

    def precise_angle_measurement(self, InterfaceInstance):
        global ANGLE_DEVIATION, ANGLE_HANGING_DEFAULT
        measured_angles = []
        number_of_measurements = 1000
        time_measurement_start = time.time()
        print('Started angle measurement.')
        for _ in trange(number_of_measurements):
            (angle, _, _, _, _, _, _, _, _, _,) = InterfaceInstance.read_state()
            measured_angles.append(float(angle))
        time_measurement = time.time() - time_measurement_start

        angle_average = np.mean(measured_angles)
        angle_std = np.std(measured_angles)

        angle_rad = wrap_angle_rad(
            (self.angle_raw + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR - self.angle_deviation_finetune)
        angle_std_rad = angle_std * ANGLE_NORMALIZATION_FACTOR
        print('\nAverage angle of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                                  angle_rad,
                                                                                  angle_average))
        print('\nAngle std of {} measurements: {} rad, {} ADC reading'.format(number_of_measurements,
                                                                              angle_std_rad,
                                                                              angle_std))
        print('\nMeasurement took {} s'.format(time_measurement))

    def finetune_zero_angle(self, direction='increase'):
        step_change = 0.002
        if direction == 'increase':
            self.angle_deviation_finetune += step_change
            print("\nIncreased angle deviation fine tune value to {0}".format(self.angle_deviation_finetune))
        elif direction == 'decrease':
            self.angle_deviation_finetune -= step_change
            print("\nDecreased angle deviation fine tune value to {0}".format(self.angle_deviation_finetune))

    @staticmethod
    def wrap_local(angle):  # FIXME: This should be a general function, shared by different classes
        ADC_RANGE = ANGLE_360_DEG_IN_ADC_UNITS
        if angle >= ADC_RANGE / 2:
            return angle - ADC_RANGE
        elif angle <= -ADC_RANGE / 2:
            return angle + ADC_RANGE
        else:
            return angle
