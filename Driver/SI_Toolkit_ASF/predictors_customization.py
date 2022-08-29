import os
from importlib import import_module

import numpy as np
from CartPoleSimulation.Control_Toolkit.others.environment import NumpyLibrary
from globals import CONTROLLER_NAME
from yaml import FullLoader, load

environment_module = "cartpole_simulator_batched"
Environment = getattr(
    import_module(f"Driver.DriverFunctions.{environment_module}"), environment_module
)


STATE_VARIABLES = np.array([f"x_{i}" for i in range(1, Environment.num_states)])
STATE_INDICES = {x: np.where(STATE_VARIABLES == x)[0][0] for x in STATE_VARIABLES}
CONTROL_INPUTS = np.array([f"u_{i}" for i in range(Environment.num_actions)])
CONTROL_INDICES = {x: np.where(CONTROL_INPUTS == x)[0][0] for x in CONTROL_INPUTS}

config = load(open(os.path.join("CartPoleSimulation", "config.yml"), "r"), FullLoader)


class next_state_predictor_ODE:
    def __init__(self, dt, intermediate_steps, batch_size, **kwargs):
        self.s = None
        
        planning_env_config = {
            **config["controller"][CONTROLLER_NAME],
            **config["cartpole"],
            **{"seed": config["data_generator"]["seed"]},
            **{"computation_lib": NumpyLibrary},
        }
        self.env = getattr(import_module("Driver.DriverFunctions.cartpole_simulator_batched"), "cartpole_simulator_batched")(
            batch_size=batch_size, **planning_env_config
        )

        self.intermediate_steps = intermediate_steps
        self.t_step = np.float32(dt / float(self.intermediate_steps))
        self.env.dt = self.t_step

    def step(self, s, Q, params):
        self.env.state = s.copy()
        for _ in range(self.intermediate_steps):
            next_state, _, _, _ = self.env.step(Q)
            s = next_state
        return next_state


def augment_predictor_output(output_array, net_info):
    pass
    return output_array
