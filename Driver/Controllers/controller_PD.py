"""
A PD controller for the Cartpole using CartpoleSimulator conventions
"""

import json

from Controllers.template_controller import template_controller
from Driver.CartPole.state_utilities import cartpole_state_varname_to_index

from globals import *

# TODO: Remove angle_target from json. You neither should set it externally.
#  The only possible scenario when angle target is not 0 would be if it follows some trajectory. This trajectory would be calculated inside of the controller.
JSON_PATH = 'json/'
PARAMS_JSON_FILE = JSON_PATH + 'control-7.json'

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

        self.ANGLE_SMOOTHING = 0.0
        self.ANGLE_KP = 0.0
        self.ANGLE_KD = 0.0
        self.ANGLE_KI = 0.0

        self.POSITION_TARGET = 0

        self.POSITION_SMOOTHING = 0.0
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
            # print(diffFactor)
            #diffFactor = 0.005
        self.time_last = time
        #print(diffFactor)

        self.POSITION_TARGET = target_position

        self.positionErr = self.POSITION_SMOOTHING * (s[cartpole_state_varname_to_index('position')] - target_position) + (1.0 - self.POSITION_SMOOTHING) * self.positionErrPrev  # First order low-P=pass filter
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

        self.angleErr = self.ANGLE_SMOOTHING * (s[cartpole_state_varname_to_index('angle')] - self.ANGLE_TARGET) + (1.0 - self.ANGLE_SMOOTHING) * self.angleErrPrev  # First order low-pass filter
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
        print("    Smoothing       {0:.2f}".format(self.ANGLE_SMOOTHING))
        print("    P Gain          {0:.2f}".format(self.ANGLE_KP))
        print("    I Gain          {0:.2f}".format(self.ANGLE_KI))
        print("    D Gain          {0:.2f}".format(self.ANGLE_KD))

        print("Position PD Control Parameters")
        print("    Set point       {0}".format(self.POSITION_TARGET))
        print("    Smoothing       {0:.2f}".format(self.POSITION_SMOOTHING))
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
            self.ANGLE_SMOOTHING = p['ANGLE_SMOOTHING']
            self.POSITION_SMOOTHING = p['POSITION_SMOOTHING']
        except Exception as e:
            print(f"\nsomething went wrong loading parameters: {e}")
            return
        print("success, parameters are")
        self.printparams()

    def saveparams(self):
        print(f"\nSaving parameters to {PARAMS_JSON_FILE}")
        p = {}
        p['ANGLE_TARGET'] = self.ANGLE_TARGET
        p['ANGLE_KP'] = self.ANGLE_KP
        p['ANGLE_KI'] = self.ANGLE_KI
        p['ANGLE_KD'] = self.ANGLE_KD
        p['POSITION_KP'] = self.POSITION_KP
        p['POSITION_KI'] = self.POSITION_KI
        p['POSITION_KD'] = self.POSITION_KD
        p['ANGLE_SMOOTHING'] = self.ANGLE_SMOOTHING
        p['POSITION_SMOOTHING'] = self.POSITION_SMOOTHING
        with open('control.json', 'w') as f:
            json.dump(p, f)

    def keyboard_input(self, c):
        if c == 'p':
            self.printparams()
        # Increase Target Angle
        elif c == '=':
            self.ANGLE_TARGET += 1
            print("\nIncreased target angle to {0}".format(self.ANGLE_TARGET))
        # Decrease Target Angle
        elif c == '-':
            self.ANGLE_TARGET -= 1
            print("\nDecreased target angle to {0}".format(self.ANGLE_TARGET))
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
        elif c == 'x':
            self.ANGLE_SMOOTHING = dec(self.ANGLE_SMOOTHING)
            if self.ANGLE_SMOOTHING > 1:
                self.ANGLE_SMOOTHING = 1
            print("\nIncreased ANGLE_SMOOTHING {0}".format(self.ANGLE_SMOOTHING))
        elif c == 'z':
            self.ANGLE_SMOOTHING = inc(self.ANGLE_SMOOTHING)
            if self.ANGLE_SMOOTHING > 1:
                self.ANGLE_SMOOTHING = 1
            print("\nDecreased ANGLE_SMOOTHING {0}".format(self.ANGLE_SMOOTHING))

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
        elif c == 'v':
            self.POSITION_SMOOTHING = dec(self.POSITION_SMOOTHING)
            if self.POSITION_SMOOTHING > 1:
                self.POSITION_SMOOTHING = 1
            print("\nIncreased POSITION_SMOOTHING {0}".format(self.POSITION_SMOOTHING))
        elif c == 'c':
            self.POSITION_SMOOTHING = inc(self.POSITION_SMOOTHING)
            if self.POSITION_SMOOTHING > 1:
                self.POSITION_SMOOTHING = 1
            print("\nDecreased POSITION_SMOOTHING {0}".format(self.POSITION_SMOOTHING))
        elif c == 'S':
            self.saveparams()
        elif c == 'L':
            self.loadparams()

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
