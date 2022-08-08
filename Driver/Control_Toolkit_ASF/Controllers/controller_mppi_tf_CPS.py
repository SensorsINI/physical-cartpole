from Control_Toolkit.Controllers.controller_mppi_tf import controller_mppi_tf

"""
Please modify this template to include a controller from cartpole simulator in physical cartpole
Please remember the general rules:

For a controller to be found and imported it must:
1. Be in Controllers folder
2. Have a name starting with "controller_"
3. The name of the controller class must be the same as the name of the file.
4. It must have __init__ and step methods
"""


class controller_mppi_tf_CPS(controller_mppi_tf):
    def functions_specific_to_physical_cartpole(self):
        # e.g. inout and output from the terminal
        pass
