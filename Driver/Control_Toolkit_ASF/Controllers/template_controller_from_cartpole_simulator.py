from Driver.Control_Toolkit.Controllers import template_controller

"""
Please modify this template to include a controller from cartpole simulator in physical cartpole
Please remember the general rules:

For a controller to be found and imported it must:
1. Be in Controllers folder
2. Have a name starting with "controller_"
3. The name of the controller class must be the same as the name of the file.
4. It must have __init__ and step methods
"""

class controller_from_cartpole_simulator_PCP(template_controller):
    def __init__(self, SomeController, *args, **kwargs):
        self.controller_kern: template_controller = SomeController(*args, **kwargs)

    def step(self, s, time=None):
        Q = self.controller_kern.step(s, time)
        return Q

    def controller_reset(self):
        self.controller_kern.controller_reset()

    def functions_specific_to_physical_cartpole(self):
        # e.g. inout and output from the terminal
        pass