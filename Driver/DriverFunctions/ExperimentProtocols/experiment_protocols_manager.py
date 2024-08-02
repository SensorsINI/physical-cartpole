from DriverFunctions.ExperimentProtocols.experiment_protocols_selector import experiment_protocols_selector_class


class ExperimentProtocolsManager:
    def __init__(self, driver):

        self.driver = driver
        self.experiment_protocols_selector = experiment_protocols_selector_class(driver)
        self.current_experiment_protocol = self.experiment_protocols_selector.get_experiment_protocol()

    def experiment_protocol_on_off(self):
        if self.current_experiment_protocol.is_idle():
            self.current_experiment_protocol.start()
        else:
            self.current_experiment_protocol.stop()
            self.driver.Q = 0.0
            self.driver.InterfaceInstance.set_motor(0)

    def change_experiment_protocol(self):
        if self.current_experiment_protocol.is_running():
            self.current_experiment_protocol.stop()
        self.current_experiment_protocol = self.experiment_protocols_selector.get_next_experiment_protocol()

    def experiment_protocol_step(self):
        if self.current_experiment_protocol.is_running():
            try:
                self.current_experiment_protocol.update_state(self.driver.s[ANGLE_IDX], self.driver.s[POSITION_IDX], self.driver.th.time_current_measurement)
                if self.current_experiment_protocol.Q is not None:
                    self.driver.Q = self.current_experiment_protocol.Q
            except TimeoutError as e:
                if self.current_experiment_protocol.Q is not None:
                    self.driver.Q = 0.0
                self.driver.log.warning(f'timeout in self.measurement: {e}')

            if self.current_experiment_protocol.target_position is not None:
                self.driver.target_position = self.current_experiment_protocol.target_position

            if self.current_experiment_protocol.target_equilibrium is not None:
                self.driver.CartPoleInstance.target_equilibrium = self.current_experiment_protocol.target_equilibrium
