"""
A PD controller for the Cartpole using CartpoleSimulator conventions
"""

import json
from datetime import datetime

from Controllers.template_controller import template_controller
from CartPole.state_utilities import cartpole_state_varname_to_index
from DriverFunctions.json_helpers import get_new_json_filename

from globals import *

# PID params from json
PARAMS_JSON_FILE = JSON_PATH + 'control_PD-9.json'
 #PARAMS_JSON_FILE = JSON_PATH + 'control-factory.json'

class controller_PD(template_controller):
    def __init__(self):

        self.controller_name = 'PD'
        self.time_last = None

        self.positionErrPrev = 0.0
        self.angleErrPrev = 0.0

        self.angleErr_integral = 0.0
        self.positionErr_integral = 0.0

        self.motorCmd = 0

        self.angleErr = 0.0
        self.positionErr = 0.0

        self.angleCmd = 0.0
        self.positionCmd = 0.0

        self.ANGLE_TARGET = 0.0

        self.ANGLE_KP = 0.0
        self.ANGLE_KD = 0.0
        self.ANGLE_KI = 0.0

        self.POSITION_TARGET = 0

        self.POSITION_KP = 0.0
        self.POSITION_KD = 0.0
        self.POSITION_KI = 0.0

        self.PARAMS_JSON_FILE = PARAMS_JSON_FILE

        self.time_last = None


    def step(self, s, target_position, time=None):
        # This diffFactor was before strangely - dt was a sampling time

        if self.time_last is None:
            diffFactor = 1.0
        else:
            diffFactor = CONTROL_PERIOD_MS / (time - self.time_last) / 1000

        self.time_last = time

        self.POSITION_TARGET = target_position

        self.positionErr = (s[cartpole_state_varname_to_index('position')] - target_position)
        positionErrDiff = (self.positionErr - self.positionErrPrev) * diffFactor
        self.positionErrPrev = self.positionErr
        self.positionErr_integral += self.positionErr
        if self.POSITION_KI > 0.0:
            if self.positionErr_integral > 1.0/(CONTROL_PERIOD_MS/1.0e3)/self.POSITION_KI:
                self.positionErr_integral = 1.0/(CONTROL_PERIOD_MS/1.0e3)/self.POSITION_KI
            elif self.positionErr_integral < -1.0/(CONTROL_PERIOD_MS/1.0e3)/self.POSITION_KI:
                self.positionErr_integral = -1.0/(CONTROL_PERIOD_MS/1.0e3)/self.POSITION_KI

        # Naive solution: if too positive (too right), move left (minus on positionCmd),
        # but this does not produce correct control.
        # The correct strategy is that if cart is too positive (too right),
        # produce lean to the left by introducing a positive set point angle leaning slightly to left,
        # i.e. more positve positionErr makes more positive effective ANGLE_TARGET
        # End result is that sign of positionCmd is flipped
        # KD term with "-" resists the motion
        # KP and KI with "-" acts attractive towards the target position
        self.positionCmd = self.POSITION_KP * self.positionErr + self.POSITION_KD * positionErrDiff + self.POSITION_KI * self.positionErr_integral*(CONTROL_PERIOD_MS/1.0e3)

        self.angleErr = (s[cartpole_state_varname_to_index('angle')] - self.ANGLE_TARGET)
        angleErrDiff = (self.angleErr - self.angleErrPrev) * diffFactor  # correct for actual sample interval; if interval is too long, reduce diff error
        self.angleErrPrev = self.angleErr
        self.angleErr_integral += self.angleErr
        if self.ANGLE_KI > 0.0:
            if self.angleErr_integral > 1.0/(CONTROL_PERIOD_MS/1.0e3)/self.ANGLE_KI:
                self.angleErr_integral = 1.0/(CONTROL_PERIOD_MS/1.0e3)/self.ANGLE_KI
            elif self.angleErr_integral < -1.0/(CONTROL_PERIOD_MS/1.0e3)/self.ANGLE_KI:
                self.angleErr_integral = -1.0/(CONTROL_PERIOD_MS/1.0e3)/self.ANGLE_KI
        # Assuming gains are positive, error growing to the "right" (around zero in upright position , this means in fact angle gets negative), causes motor to move to the right
        # iff a term below has - sign
        self.angleCmd = -self.ANGLE_KP * self.angleErr - self.ANGLE_KD * angleErrDiff - self.ANGLE_KI * self.angleErr_integral*(CONTROL_PERIOD_MS/1.0e3)  # if too CCW (pos error), move cart left

        motorCmd = self.angleCmd + self.positionCmd  # change to plus for original, check that when cart is displayed, the KP term for cart position leans cart the correct direction
        return motorCmd

    def printparams(self):
        print("\nAngle PID Control Parameters")
        print("    Set point       {0}".format(self.ANGLE_TARGET))
        print("    P Gain          {0:.2f}".format(self.ANGLE_KP))
        print("    I Gain          {0:.2f}".format(self.ANGLE_KI))
        print("    D Gain          {0:.2f}".format(self.ANGLE_KD))

        print("Position PD Control Parameters")
        print("    Set point       {0}".format(self.POSITION_TARGET))
        print("    P Gain          {0:.2f}".format(self.POSITION_KP))
        print("    I Gain          {0:.2f}".format(self.POSITION_KI))
        print("    D Gain          {0:.2f}".format(self.POSITION_KD))

    def loadparams(self):
        print(f"\nLoading parameters from {self.PARAMS_JSON_FILE}....")
        f = open(self.PARAMS_JSON_FILE)
        try:
            p = json.load(f)
            self.ANGLE_TARGET = p['ANGLE_TARGET']
            self.ANGLE_KP = p['ANGLE_KP']
            self.ANGLE_KI = p['ANGLE_KI']
            self.ANGLE_KD = p['ANGLE_KD']
            self.POSITION_KP = p['POSITION_KP']
            self.POSITION_KI = p['POSITION_KI']
            self.POSITION_KD = p['POSITION_KD']
        except Exception as e:
            print(f"\nsomething went wrong loading parameters: {e}")
            return
        print("success, parameters are")
        self.printparams()

    def saveparams(self):
        json_filepath = get_new_json_filename(self.controller_name)
        print(f"\nSaving parameters to {json_filepath}")

        p = {}
        p['ANGLE_TARGET'] = self.ANGLE_TARGET
        p['ANGLE_KP'] = self.ANGLE_KP
        p['ANGLE_KI'] = self.ANGLE_KI
        p['ANGLE_KD'] = self.ANGLE_KD
        p['POSITION_KP'] = self.POSITION_KP
        p['POSITION_KI'] = self.POSITION_KI
        p['POSITION_KD'] = self.POSITION_KD
        with open(json_filepath, 'w') as f:
            json.dump(p, f)

    def keyboard_input(self, c):
        if c == 'p':
            self.printparams()
        # Angle Gains
        elif c == '2':
            self.ANGLE_KP = inc(self.ANGLE_KP)
            print("\nIncreased angle KP {0}".format(self.ANGLE_KP))
        elif c == '1':
            self.ANGLE_KP = dec(self.ANGLE_KP)
            print("\nDecreased angle KP {0}".format(self.ANGLE_KP))
        elif c == 'w':
            self.ANGLE_KI = inc(self.ANGLE_KI)
            print("\nIncreased angle KI {0}".format(self.ANGLE_KI))
        elif c == 'q':
            self.ANGLE_KI = dec(self.ANGLE_KI)
            print("\nDecreased angle KI {0}".format(self.ANGLE_KI))
        elif c == 's':
            self.ANGLE_KD = inc(self.ANGLE_KD)
            print("\nIncreased angle KD {0}".format(self.ANGLE_KD))
        elif c == 'a':
            self.ANGLE_KD = dec(self.ANGLE_KD)
            print("\nDecreased angle KD {0}".format(self.ANGLE_KD))
        # Position Gains
        elif c == '4':
            self.POSITION_KP = inc(self.POSITION_KP)
            print("\nIncreased position KP {0}".format(self.POSITION_KP))
        elif c == '3':
            self.POSITION_KP = dec(self.POSITION_KP)
            print("\nDecreased position KP {0}".format(self.POSITION_KP))
        elif c == 'r':
            self.POSITION_KI = inc(self.POSITION_KI)
            print("\nIncreased position KI {0}".format(self.POSITION_KI))
        elif c == 'e':
            self.POSITION_KI = dec(self.POSITION_KI)
            print("\nDecreased position KI {0}".format(self.POSITION_KI))
        elif c == 'f':
            self.POSITION_KD = inc(self.POSITION_KD)
            print("\nIncreased position KD {0}".format(self.POSITION_KD))
        elif c == 'd':
            self.POSITION_KD = dec(self.POSITION_KD)
            print("\nDecreased position KD {0}".format(self.POSITION_KD))
        elif c == 'S':
            self.saveparams()
        elif c == 'L':
            self.loadparams()

    def print_help(self):
        print("\n***********************************")
        print("keystroke commands")
        print("ESC quit")
        print("k toggle control on/off (initially off)")
        print("K trigger motor position calibration")
        print("=/- increase/decrease (fine tune) angle deviation value")
        print("[/] increase/decrease position target")
        print("2/1 angle proportional gain")
        print("w/q angle integral gain")
        print("s/a angle derivative gain")
        print("z/x angle smoothing")
        print("4/3 position proportional gain")
        print("r/e position integral gain")
        print("f/d position derivative gain")
        print("p print PID parameters")
        print("l toggle logging data")
        print("S/L Save/Load param values from disk")
        print("D Toggle dance mode")
        print(",./ Turn on motor left zero right")
        print("m Toggle measurement")
        print("j Switch joystick control mode")
        print("b Print angle measurement from sensor")
        print("***********************************")

    def controller_reset(self):
        self.time_last = None

        self.positionErrPrev = 0.0
        self.angleErrPrev = 0.0

        self.positionErr_integral = 0.0
        self.angleErr_integral = 0.0

        self.motorCmd = 0

        self.angleErr = 0.0
        self.positionErr = 0.0

        self.angleCmd = 0.0
        self.positionCmd = 0.0
