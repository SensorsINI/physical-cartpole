# measurements from cartpole, controlled by state machine.
# control.py calls update_state() if state is not 'idle'
from DriverFunctions.ExperimentProtocols import template_experiment_protocol
import time
from globals import *
from CartPole.state_utilities import (
    ANGLE_IDX,
    ANGLED_IDX,
    POSITION_IDX,
    POSITIOND_IDX,
)
import numpy as np

from DriverFunctions.TargetPositionGenerator import TargetPositionGenerator
from DriverFunctions.PID_Position import controller_PID_position

NUMBER_OF_SWINGUPS = 100
STABLE_ANGLE_RAD = 0.2
TIME_STABLE = 16
RESET_Q = 0.5
TIME_STABLE_DOWN = 8.0
RECALIBRATE_EVERY_N_SWING_UPS = 2

class swing_up_experiment(template_experiment_protocol):
    def __init__(self, driver):
        super().__init__(
            driver=driver,
            experiment_protocol_name=self.__class__.__name__[:-len('_experiment')],)

        self.counter_swingup = 0
        self.TPG = TargetPositionGenerator()

        self.PID = controller_PID_position()
        
        
    def set_up_experiment(self, first_iteration=True):
        if first_iteration:
            self.counter_swingup = 0

        if RECALIBRATE_EVERY_N_SWING_UPS is not None and self.counter_swingup % RECALIBRATE_EVERY_N_SWING_UPS == 0:
            print("\nCalibrating motor position.... ")
            self.driver.InterfaceInstance.calibrate()
            print("Done calibrating")

        self.start_new_recording(index=self.counter_swingup)

        self.TPG.set_experiment()

        self.start_angle = np.random.uniform(low=0.5, high=0.8) * np.pi
        self.start_angleD = np.random.uniform(low=0.5, high=0.8) * np.pi
        self.target_position = np.random.uniform(low=-0.18, high=0.18)
        self.target_equilibrium = -1

        self.current_experiment_phase = 'reset'
        
        

    def update_state(self, angle, position, time):
        self.angle = angle
        self.angleD = self.driver.s[ANGLED_IDX]
        self.position = position
        self.positionD = self.driver.s[POSITIOND_IDX]
        self.time = time

        if self.current_experiment_phase == 'idle':
            pass
        elif self.current_experiment_phase == 'reset':
            self.action_reset()
        elif self.current_experiment_phase == 'wait inbetween':
            self.action_wait_inbetween()
        elif self.current_experiment_phase == 'swingup':
            self.action_swing_up()
        else:
            raise Exception(f'unknown experiment phase: {self.current_experiment_phase}')

    def action_reset(self):
        self.target_equilibrium = -1
        self.driver.controlEnabled = True

        if abs(self.position - self.target_position) > 0.01 \
                and not (
                abs(self.angle) > self.start_angle and abs(self.angleD) < self.start_angleD
        ):
            pass
        else:
            self.time_start_stable_down = self.time
            self.current_experiment_phase = 'wait inbetween'

    def action_wait_inbetween(self):
        self.target_equilibrium = -1
        self.driver.controlEnabled = True
        self.target_position = self.TPG.get_target_position()

        if self.time - self.time_start_stable_down > TIME_STABLE_DOWN:
            self.target_position = self.TPG.get_target_position()
            self.time_start = self.time
            self.current_experiment_phase = 'swingup'
            self.driver.controlEnabled = True


    def action_swing_up(self):
        self.target_equilibrium = 1

        self.target_position = self.TPG.get_target_position()

        if self.time - self.time_start > TIME_STABLE:
            self.counter_swingup += 1
            self.finish_recording()
            if self.counter_swingup >= NUMBER_OF_SWINGUPS:
                self.stop()
                self.driver.controller.controller_report()
            else:
                self.set_up_experiment(first_iteration=False)


    def __str__(self):
        if self.current_experiment_phase == 'reset':
            return f'Swingup (Experiment Phase: {self.current_experiment_phase}, Total Swingups:{self.counter_swingup}, Q:{self.driver.Q:.2f}, Start Position={self.target_position*100:.1f}, Start Angle={self.start_angle:.1f}, Start AngleD={self.start_angleD:.1f})'
        elif self.current_experiment_phase == 'swingup':
            return f'Swingup (Experiment Phase: {self.current_experiment_phase}, Total Swingups:{self.counter_swingup}, Q:{self.driver.Q:.2f}, Time swingup={(self.time - self.time_start):.2f}, Target Position={self.target_position*100:.1f})'
        else:
            return f'Swingup (Experiment Phase: {self.current_experiment_phase}, Total Swingups:{self.counter_swingup}, Q:{self.driver.Q:.2f})'
