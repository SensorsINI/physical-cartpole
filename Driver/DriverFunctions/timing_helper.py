import time

import numpy as np

from CartPoleSimulation.CartPole.latency_adder import LatencyAdder

from globals import CONTROL_PERIOD_MS, STATISTICS_IN_TERMINAL_AVERAGING_LENGTH


class TimingHelper:
    def __init__(self):
        self.time_current_measurement_chip = 0
        self.time_last_measurement_chip = None
        self.time_between_measurements_chip = 0

        # Timing
        self.time_experiment_started = None
        self.time_current_measurement = None
        self.elapsedTime = None

        self.delta_time_buffer = np.zeros((0,))
        self.firmware_latency_buffer = np.zeros((0,))
        self.python_latency = 0
        self.python_latency_buffer = np.zeros((0,))
        self.controller_steptime = 0
        self.controller_steptime_previous = 0
        self.controller_steptime_buffer = np.zeros((0,))
        self.controlled_iterations = 0
        self.total_iterations = 0

        self.firmware_latency = 0
        self.latency_violation = 0
        self.latency_violations = 0

        # Artificial Latency
        self.additional_latency = 0.0
        self.LatencyAdderInstance = LatencyAdder(latency=self.additional_latency, dt_sampling=0.005)


    def timer(self, attr_name, prev_attr_name=None):
        """Create a NamedTimer for a given attribute name, with an optional previous attribute name."""
        return NamedTimer(self, attr_name, prev_attr_name)

    def setup(self):
        self.time_experiment_started = time.time()

    def load_timing_data_from_chip(
            self,
            time_current_measurement_chip,
            time_between_measurements_chip,
            latency_violation_chip,
            firmware_latency,
    ):
        self.time_current_measurement_chip = time_current_measurement_chip
        self.time_between_measurements_chip = time_between_measurements_chip
        self.latency_violation = latency_violation_chip
        self.firmware_latency = firmware_latency

    def time_measurement(self):
        self.time_current_measurement = time.time()
        self.elapsedTime = self.time_current_measurement - self.time_experiment_started

        if self.time_between_measurements_chip < 1.0e-9:
            raise ValueError(f'\nTime between measurements measured on chip is {self.time_between_measurements_chip}. '
                             f'\nThis might indicate that timer on the microcontroller has overflown. '
                             f'\nTry to restart it.')

        if self.time_last_measurement_chip is not None:  # TODO: Maybe can be removed
            delta_time_test = self.time_current_measurement_chip - self.time_last_measurement_chip
            if abs(delta_time_test - self.time_between_measurements_chip) > 2e-6 and delta_time_test != 0.0:
                raise ValueError(f"dt={self.time_between_measurements_chip}; dtt={delta_time_test}")

        self.time_last_measurement_chip = self.time_current_measurement_chip

    # FIXME: Think if these cases are right
    def check_latency_violation(self, controlEnabled):
        # Latency Violations
        if self.latency_violation == 1:
            self.latency_violations += 1
        elif self.time_between_measurements_chip > 1.5 * CONTROL_PERIOD_MS / 1000.0:
            self.latency_violation = 1
            self.latency_violations += np.floor(self.time_between_measurements_chip / (CONTROL_PERIOD_MS / 1000.0))
        elif controlEnabled and self.firmware_latency > (CONTROL_PERIOD_MS / 1000.0):
            self.latency_violation = 1
            self.latency_violations += 1
        elif controlEnabled and self.firmware_latency < self.controller_steptime_previous:  # Heuristic, obviosuly wrong case
            self.latency_violation = 1
            self.latency_violations += 1


    def add_latency(self, s):
        self.LatencyAdderInstance.add_current_state_to_latency_buffer(s)
        s_with_latency = self.LatencyAdderInstance.get_interpolated_delayed_state()
        return s_with_latency


    def change_additional_latency(self, change_direction="increase"):

        latency_change_step = 0.001

        if change_direction == "increase":
            self.additional_latency += latency_change_step
        elif change_direction == "decrease":
            self.additional_latency -= latency_change_step
        else:
            raise ValueError('Unexpected command for change_direction = '.format(change_direction))

        print('\nAdditional latency set now to {:.1f} ms'.format(self.additional_latency * 1000))
        self.LatencyAdderInstance.set_latency(self.additional_latency)

    def latency_data_for_statistics_in_terminal(self):
        # Averaging
        self.total_iterations += 1
        if self.total_iterations > 10 and self.controlled_iterations > 10:
            self.delta_time_buffer = np.append(self.delta_time_buffer, self.time_between_measurements_chip)
            self.delta_time_buffer = self.delta_time_buffer[-STATISTICS_IN_TERMINAL_AVERAGING_LENGTH:]
            self.firmware_latency_buffer = np.append(self.firmware_latency_buffer, self.firmware_latency)
            self.firmware_latency_buffer = self.firmware_latency_buffer[-STATISTICS_IN_TERMINAL_AVERAGING_LENGTH:]
            self.python_latency_buffer = np.append(self.python_latency_buffer, self.python_latency)
            self.python_latency_buffer = self.python_latency_buffer[-STATISTICS_IN_TERMINAL_AVERAGING_LENGTH:]
            self.controller_steptime_buffer = np.append(self.controller_steptime_buffer, self.controller_steptime)
            self.controller_steptime_buffer = self.controller_steptime_buffer[-STATISTICS_IN_TERMINAL_AVERAGING_LENGTH:]

    def strings_for_statistics_in_terminal(self):
        if self.total_iterations > 10 and self.controlled_iterations > 10:
            timing_string = "TIMING: delta time [μ={:.1f}ms, σ={:.2f}ms], firmware latency [μ={:.1f}ms, σ={:.2f}ms], \n         python latency [μ={:.1f}ms σ={:.2f}ms], controller step [μ={:.1f}ms σ={:.2f}ms]".format(
                float(self.delta_time_buffer.mean() * 1000),
                float(self.delta_time_buffer.std() * 1000),

                float(self.firmware_latency_buffer.mean() * 1000),
                float(self.firmware_latency_buffer.std() * 1000),

                float(self.python_latency_buffer.mean() * 1000),
                float(self.python_latency_buffer.std() * 1000),

                float(self.controller_steptime_buffer.mean() * 1000),
                float(self.controller_steptime_buffer.std() * 1000)
            )



            ###########  Latency Violations  ############
            percentage_latency_violations = 100 * self.latency_violations / self.total_iterations if self.total_iterations > 0 else 0
            timing_latency_string = f"         latency violations: {self.latency_violations}/{self.total_iterations} = {percentage_latency_violations:.1f}%"

            return timing_string, timing_latency_string
        else:
            return None, None

    def reset_timing_helper_memory(self):
        self.delta_time_buffer = np.zeros((0,))
        self.firmware_latency_buffer = np.zeros((0,))
        self.python_latency_buffer = np.zeros((0,))
        self.controller_steptime_buffer = np.zeros((0,))

        self.latency_violations = 0

    @staticmethod
    def time_since(starting_time):
        return time.time() - starting_time

    @staticmethod
    def sleep(time_to_sleep):
        time.sleep(time_to_sleep)


# The Named Timer class allows to time code snippets with "with" statement.
# After exiting the "with" statement, the elapsed time is stored in the attr_name attribute of helper instrance.
class NamedTimer:
    def __init__(self, helper, attr_name, prev_attr_name=None):
        self.helper = helper  # TimingHelper instance
        self.attr_name = attr_name  # Attribute name to store the elapsed time
        self.prev_attr_name = prev_attr_name  # Attribute name to store the previous elapsed time

    def __enter__(self):
        # Record the start time when entering the context
        self.start_time = time.time()
        # Save the previous elapsed time if prev_attr_name is provided
        if self.prev_attr_name and hasattr(self.helper, self.attr_name):
            setattr(self.helper, self.prev_attr_name, getattr(self.helper, self.attr_name))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Record the end time and calculate the elapsed time when exiting the context
        self.end_time = time.time()
        self.elapsed_time = self.end_time - self.start_time
        # Dynamically set the attribute on the TimingHelper instance
        setattr(self.helper, self.attr_name, self.elapsed_time)
