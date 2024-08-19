import sys

from DriverFunctions.kbhit import KBHit


class KeyboardController:
    def __init__(self, driver):

        try:
            self.kb = KBHit()  # can only use in posix terminal; cannot use from spyder ipython console for example
            self.kbAvailable = True
        except:
            self.kbAvailable = False

        self.driver = driver
        self.key_actions = {}

        self.assign_actions_to_keys()

    def assign_actions_to_keys(self):
        # Define a dictionary to map key presses to methods
        self.key_actions = {

            ##### Help #####
            'h': (self.print_help, "Print this help message"),
            '?': (self.print_help, "Print this help message"),

            ##### Calibration #####
            'K': (self.driver.calibrate, "Calibration: find track middle"),

            ##### Control Mode #####
            'k': (self.driver.software_controller_on_off, "PC Control On/Off"),
            'u': (self.driver.hardware_controller_on_off, "Chip Control On/Off"),

            ##### Dance #####
            'D': (self.driver.dancer.on_off, "Dance Mode On/Off"),

            ##### Experiment Protocols #####
            'm': (self.driver.epm.change_experiment_protocol,
                  "Change Experiment Protocol: running and recording predefined sequence of movements"),
            'n': (self.driver.epm.experiment_protocol_on_off, "Start/Stop Experiment Protocol"),
            'N': (self.driver.run_hardware_experiment, "Start Experiment Protocol from Chip"),

            ##### Logging #####
            'l': (self.driver.mlm.recording_on_off, "Start/Stop recording to a CSV file"),
            'L': (lambda: self.driver.mlm.recording_on_off(time_limited_recording=True),
                  "Start/Stop time limited recording to a CSV file"),

            ##### Real Time Data Vizualization #####
            '6': (self.driver.mlm.live_plotter_sender.on_off,
                  "Start/Stop sending data to Live Plotter Server - real time visualization"),
            '7': (self.driver.mlm.live_plotter_sender.save_data_and_figure_if_connected,
                  "Save data and figure at Live Plotter Server"),
            '8': (self.driver.mlm.live_plotter_sender.reset_if_connected, "Reset Live Plotter Server"),

            ##### Target #####
            ';': (self.driver.switch_target_equilibrium, "Switch target equilibrium"),
            ']': (lambda: self.driver.change_target_position(change_direction="increase"), "Increase target position"),
            '[': (lambda: self.driver.change_target_position(change_direction="decrease"), "Decrease target position"),

            ##### Fine tune zero angle #####
            'b': (lambda: self.driver.idp.precise_angle_measurement(self.driver.InterfaceInstance),
                  "Start precise angle measurement - multiple samples"),
            '=': (lambda: self.driver.idp.finetune_zero_angle(direction='increase'),
                  "Finetune zero angle - increase angle deviation parameter"),
            '-': (lambda: self.driver.idp.finetune_zero_angle(direction='decrease'),
                  "Finetune zero angle - decrease angle deviation parameter"),

            ##### Artificial Latency  #####
            '9': (
                lambda: self.driver.idp.change_additional_latency(change_direction="increase"),
                "Increase additional latency"),
            '0': (
                lambda: self.driver.idp.change_additional_latency(change_direction="decrease"),
                "Decrease additional latency"),

            ##### Joystick  #####
            'j': (lambda: self.driver.joystick.toggle_mode(self.driver.log), "Joystick On/Off"),

            ##### Empty ######
            '.': (lambda: None, "Key not assigned"),
            ',': (lambda: None, "Key not assigned"),
            '/': (lambda: None, "Key not assigned"),
            '5': (lambda: None, "Key not assigned"),

            ##### Exit ######
            chr(27): (self.driver.start_experiment_termination, "ESC: Start experiment termination")  # ESC
        }

    def setup(self):
        # Check that we are running from terminal, otherwise we cannot control it
        if not sys.stdin.isatty():
            print('Run from an interactive terminal to allow keyboard input.')
            quit()
            
        self.print_help()

    def keyboard_input(self):

        if self.kbAvailable & self.kb.kbhit():

            c = self.kb.getch()
            try:
                self.driver.controller.keyboard_input(c)
            except AttributeError:
                pass

            # Execute the action if the key is in the dictionary
            if c in self.key_actions:
                self.key_actions[c][0]()

    def print_help(self):
        # Generate the help text dynamically
        help_text = "Key Bindings:\n"
        for key, (func, description) in self.key_actions.items():
            help_text += f" {key}: {description}\n"

        print(help_text)
        print()
        if hasattr(self.driver.controller, 'print_help'):
            self.driver.controller.print_help()