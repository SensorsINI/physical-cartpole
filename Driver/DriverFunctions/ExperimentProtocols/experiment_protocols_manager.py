from DriverFunctions.ExperimentProtocols.step_response_experiment import step_response_experiment
from DriverFunctions.ExperimentProtocols.swing_up_experiment import swing_up_experiment
from DriverFunctions.ExperimentProtocols.follow_a_random_target_experiment import follow_a_random_target_experiment


class experiment_protocols_manager_class:
    def __init__(self, driver):

        self.driver = driver

        self.experiment_protocols = [
            step_response_experiment,
            swing_up_experiment,
            follow_a_random_target_experiment,
        ]
        self.current_experiment_index = 0

    def get_next_experiment_protocol(self):
        self.current_experiment_index += 1
        if self.current_experiment_index >= len(self.experiment_protocols):
            self.current_experiment_index = 0
        return self.experiment_protocols[self.current_experiment_index](self.driver)

    def get_experiment_protocol(self):
        return self.experiment_protocols[self.current_experiment_index](self.driver)
