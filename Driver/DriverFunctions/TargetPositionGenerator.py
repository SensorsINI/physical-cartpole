import timeit

from CartPoleSimulation.CartPole import CartPole
from CartPoleSimulation.CartPole.data_generator import random_experiment_setter

class TargetPositionGenerator:
    def __init__(self, cartpole_instance=None):
        self.RES = random_experiment_setter()
        self.CartPoleInstance = cartpole_instance
        self.time_start = 0.0

    def set_experiment(self):
        if self.CartPoleInstance is None:
            self.CartPoleInstance = CartPole()
        self.CartPoleInstance = self.RES.set(self.CartPoleInstance)
        self.time_start = timeit.default_timer()

    def get_target_position(self):
        self.CartPoleInstance.time = timeit.default_timer() - self.time_start
        self.CartPoleInstance.update_target_position()
        return self.CartPoleInstance.target_position
