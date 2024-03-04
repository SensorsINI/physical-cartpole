import os
from abc import ABC, abstractmethod

import threading
from DriverFunctions.csv_helpers import csv_init

class template_experiment_protocol(ABC):

    def __init__(
            self,
            driver,
            experiment_protocol_name: str,
    ):
        self.driver = driver  # Use always as read-only inside a measurement, possibly delete later on...
        self.current_experiment_phase = 'idle'
        self.experiment_protocol_name = experiment_protocol_name.replace('_', '-')
        print(f'\nLoading {self.experiment_protocol_name} experiment protocol!\n')

        self.Q = None
        self.target_position = None
        self.target_equilibrium = None

    def is_idle(self):
        return self.current_experiment_phase == 'idle'

    def is_running(self):
        return not self.is_idle()

    def start(self):
        print(f'\nStarting {self.experiment_protocol_name} experiments!\n')
        self.set_up_experiment()

    @abstractmethod
    def set_up_experiment(self, first_iteration=True):
        pass

    def stop(self):
        if self.driver.loggingEnabled:
            self.finish_recording()
        self.current_experiment_phase = 'idle'
        self.driver.controlEnabled = False
        self.driver.Q = 0.0
        self.driver.InterfaceInstance.set_motor(0.0)
        print(f'\nStopped {self.experiment_protocol_name} experiments!\n')


    def start_new_recording(self, index=None):
        if index is None:
            index_string = ''
        else:
            index_string = '-' + str(index)

        def f():
            self.driver.csvfilename, self.driver.csvfile, self.driver.csvwriter = csv_init(
                csv_name=self.experiment_protocol_name + '-experiment' + index_string,
                controller_name=self.driver.controller.controller_name)

        self.driver.csv_init_thread = threading.Thread(target=f)
        self.driver.csv_init_thread.start()
        self.driver.csv_init_thread.join()
        self.driver.loggingEnabled = True

    def finish_recording(self):
        self.driver.loggingEnabled = False
        self.driver.csvfile.close()

    def __str__(self):
        return 'Experiment Protocol: ' + self.experiment_protocol_name

