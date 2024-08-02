import numpy as np

from time import sleep
from abc import ABC, abstractmethod

from CartPoleSimulation.CartPole.csv_logger import create_csv_file_name

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
        self.data_to_save_measurement = {}

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
        if self.driver.mlm.recording_running:
            self.finish_recording()
        self.current_experiment_phase = 'idle'
        self.driver.controlEnabled = False
        self.driver.Q = 0.0
        self.driver.InterfaceInstance.set_motor(0)
        self.data_to_save_measurement = {}
        print(f'\nStopped {self.experiment_protocol_name} experiments!\n')


    def start_new_recording(self, index=None, recording_length=np.inf):

        if self.driver.mlm.starting_recording:
            for _ in range(10):
                if not self.driver.mlm.starting_recording:
                    break
                sleep(0.1)
            if self.driver.mlm.starting_recording:
                raise Exception('The new recording for this measurement could not be started,'
                                'because the starting recording flag of data manager is high all the time!')


        if self.driver.mlm.recording_running:
            print('Finishing currently running recording, before starting a new one for this measurement!')
            self.finish_recording()

        if index is None:
            index_string = ''
        else:
            index_string = '-' + str(index)

        title = self.experiment_protocol_name.replace('-', '_') + index_string
        self.driver.csv_name = create_csv_file_name(prefix='CPP', with_date=False, title=title)
        self.driver.recording_length = recording_length
        self.driver.start_recording_flag = True

    def finish_recording(self):
        self.driver.finish_csv_recording()

    def __str__(self):
        return 'Experiment Protocol: ' + self.experiment_protocol_name

