import os
from importlib import import_module

import tensorflow as tf
from Control_Toolkit.others.environment import TensorFlowLibrary
from globals import CONTROLLER_NAME
from SI_Toolkit.TF.TF_Functions.Compile import Compile
from yaml import FullLoader, load

from SI_Toolkit_ASF.predictors_customization import STATE_INDICES

STATE_INDICES_TF = tf.lookup.StaticHashTable(  # TF style dictionary
    initializer=tf.lookup.KeyValueTensorInitializer(
        keys=tf.constant(list(STATE_INDICES.keys())),
        values=tf.constant(list(STATE_INDICES.values())),
    ),
    default_value=-100,
    name=None,
)

config = load(open(os.path.join("Driver", "CartPoleSimulation", "config.yml"), "r"), FullLoader)


class next_state_predictor_ODE_tf:
    def __init__(self, dt, intermediate_steps, batch_size, **kwargs):
        self.s = None

        planning_env_config = {
            **config["controller"][CONTROLLER_NAME],
            **config["cartpole"],
            **{"seed": config["data_generator"]["seed"]},
            **{"computation_lib": TensorFlowLibrary},
        }
        self.env = getattr(import_module("Driver.DriverFunctions.cartpole_simulator_batched"), "cartpole_simulator_batched")(
            batch_size=batch_size, **planning_env_config
        )

        self.intermediate_steps = tf.convert_to_tensor(
            intermediate_steps, dtype=tf.int32
        )
        self.t_step = tf.convert_to_tensor(
            dt / float(self.intermediate_steps), dtype=tf.float32
        )

    def step(self, s, Q, params):
        self.env.reset(s)
        next_state = self.env.step_tf(s, Q)
        return next_state


class predictor_output_augmentation_tf:
    def __init__(self, net_info, differential_network=False):
        self.net_output_indices = {
            key: value for value, key in enumerate(net_info.outputs)
        }
        indices_augmentation = []
        features_augmentation = []

        self.indices_augmentation = indices_augmentation
        self.features_augmentation = features_augmentation
        self.augmentation_len = len(self.indices_augmentation)

    def get_indices_augmentation(self):
        return self.indices_augmentation

    def get_features_augmentation(self):
        return self.features_augmentation

    @Compile
    def augment(self, net_output):

        output = net_output
        # if 'sin(x)' in self.features_augmentation:
        #     sin_x = tf.math.sin(net_output[..., self.index_x])[:, :, tf.newaxis]
        #     output = tf.concat([output, sin_x], axis=-1)

        return output
