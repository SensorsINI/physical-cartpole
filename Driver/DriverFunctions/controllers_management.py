"""

These are the functions used for loading the appropriate controllers.
# TODO: It requires only minor changes to eliminate self from them.
        Maybe it is worth effort changing them in the same way in CartPole simulator to ensure compatibility?

"""
from Driver.Control_Toolkit.others.globals_and_utils import get_available_controller_names

# Set the controller of CartPole
def set_controller(controller_names=None, controller_name=None, controller_idx=None):
    """
    The method sets a new controller as the current controller of the CartPole instance.
    The controller may be indicated either by its name
    or by the index on the controller list (see get_available_controller_names method).
    """

    # Check if the proper information was provided: either controller_name or controller_idx
    if (controller_name is None) and (controller_idx is None):
        raise ValueError('You have to specify either controller_name or controller_idx to set a new controller.'
                         'You have specified none of the two.')
    elif (controller_name is not None) and (controller_idx is not None):
        raise ValueError('You have to specify either controller_name or controller_idx to set a new controller.'
                         'You have specified both.')
    else:
        pass

    if controller_names is None:
        controller_names = get_available_controller_names()

    # If controller name provided get controller index and vice versa
    if (controller_name is not None):
        try:
            controller_idx = controller_names.index(controller_name)
        except ValueError:
            raise ValueError('{} is not in list. \n In list are: {}'.format(controller_name, controller_names))
    else:
        controller_name = controller_names[controller_idx]

    # Load controller
    if controller_name == 'manual-stabilization':
        controller = None
    else:
        controller_full_name = 'controller_' + controller_name.replace('-', '_')
        path_import = PATH_TO_CONTROLLERS[2:].replace('/', '.').replace(r'\\', '.')
        import_str = 'from ' + path_import + controller_full_name + ' import ' + controller_full_name
        exec(import_str)
        controller = eval(controller_full_name + '()')

    return controller, controller_name, controller_idx
