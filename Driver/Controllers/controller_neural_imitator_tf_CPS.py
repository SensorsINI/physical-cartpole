from CartPoleSimulation.Controllers.controller_neural_imitator_tf import controller_neural_imitator_tf
from Controllers.template_controller import template_controller

"""
Please modify this template to include a controller from cartpole simulator in physical cartpole
Please remember the general rules:

For a controller to be found and imported it must:
1. Be in Controllers folder
2. Have a name starting with "controller_"
3. The name of the controller class must be the same as the name of the file.
4. It must have __init__ and step methods
"""


class controller_neural_imitator_tf_CPS(controller_neural_imitator_tf):
    def __init__(self):
        super().__init__()
        self.controller_name = "neural-imitator-tf-CPS"

    def functions_specific_to_physical_cartpole(self):
        # e.g. inout and output from the terminal
        pass

    def controller_reset(self):
        pass