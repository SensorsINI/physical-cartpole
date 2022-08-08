"""
This is a linear-quadratic regulator
It assumes that the input relation is u = Q*u_max (no fancy motor model) !
"""

from scipy.linalg import solve_continuous_are
import numpy as np

from Control_Toolkit.Controllers import template_controller
from CartPoleSimulation.CartPole.state_utilities import ANGLE_IDX, ANGLED_IDX, POSITION_IDX, POSITIOND_IDX
from CartPoleSimulation.CartPole.cartpole_model import u_max, s0
from CartPoleSimulation.CartPole.cartpole_jacobian import cartpole_jacobian

import json
from DriverFunctions.json_helpers import get_new_json_filename

import yaml

from globals import dec, inc

config = yaml.load(open("config.yml", "r"), Loader=yaml.FullLoader)
Q = np.diag(config["controller"]["lqr"]["Q"])
R = config["controller"]["lqr"]["R"]

class controller_lqr(template_controller):
    def __init__(self):
        # From https://github.com/markwmuller/controlpy/blob/master/controlpy/synthesis.py#L8
        """Solve the continuous time LQR controller for a continuous time system.

        A and B are system matrices, describing the systems dynamics:
         dx/dt = A x + B u

        The controller minimizes the infinite horizon quadratic cost function:
         cost = integral (x.T*Q*x + u.T*R*u) dt

        where Q is a positive semidefinite matrix, and R is positive definite matrix.

        Returns K, X, eigVals:
        Returns gain the optimal gain K, the solution matrix X, and the closed loop system eigenvalues.
        The optimal input is then computed as:
         input: u = -K*x
        """
        # ref Bertsekas, p.151

        # Calculate Jacobian around equilibrium
        # Set point around which the Jacobian should be linearized
        # It can be here either pole up (all zeros) or pole down

        self.controller_name = 'lqr'

        s = s0
        s[POSITION_IDX] = 0.0
        s[POSITIOND_IDX] = 0.0
        s[ANGLE_IDX] = 0.0
        s[ANGLED_IDX] = 0.0
        u = 0.0

        jacobian = cartpole_jacobian(s, u)
        self.A = jacobian[:, :-1]
        self.B = np.reshape(jacobian[:, -1], newshape=(4, 1)) * u_max

        # Cost matrices for LQR controller
        self.Q = Q  # How much to punish x, v, theta, omega
        self.R = R  # How much to punish Q

        self.update()

    def step(self, s: np.ndarray, target_position: np.ndarray, time=None):

        state = np.array(
            [[s[POSITION_IDX] - target_position], [s[POSITIOND_IDX]], [s[ANGLE_IDX]], [s[ANGLED_IDX]]])

        Q = np.asscalar(np.dot(-self.K, state))

        # Clip Q
        if Q > 1.0:
            Q = 1.0
        elif Q < -1.0:
            Q = -1.0
        else:
            pass

        return Q

    def update(self):
        # first, try to solve the ricatti equation
        X = solve_continuous_are(self.A, self.B, self.Q, self.R)

        # compute the LQR gain
        if np.array(self.R).ndim == 0:
            Ri = 1.0 / self.R
        else:
            Ri = np.linalg.inv(self.R)

        K = np.dot(Ri, (np.dot(self.B.T, X)))

        eigVals = np.linalg.eigvals(self.A - np.dot(self.B, K))

        self.K = K
        self.X = X
        self.eigVals = eigVals

    def printparams(self):
        print("Q  - STATE COST MATRIX: ".format(self.Q))
        print(self.Q)
        print("R - INPUT COST: ")
        print(self.R)
        print("EIGEN VALUES:")
        print(self.eigVals)
        print("GAIN VECTOR K:")
        print(self.K)
        # print(self.motorcmd_save)
        # if self.motorcmd_save:
        #     print(np.max(self.motorcmd_save))
        #     print(np.min(self.motorcmd_save))

    def print_help(self):
        print("\n***********************************")
        print("keystroke commands")
        print("ESC quit")
        print("k toggle control on/off (initially off)")
        print("K trigger motor position calibration")
        print("=/- increase/decrease (fine tune) angle deviation value")
        print("[/] increase/decrease position target")
        print("2/1 increase/decrease input penalization")
        print("w/q increase/decrease position penalization")
        print("s/a increase/decrease velocity penalization")
        print("r/e increase/decrease angle penalization")
        print("f/d increase/decrease angular velocity penalization")
        print("z/x angle smoothing")
        print("p print LQR parameters")
        print("l toggle logging data")
        print("S Save param values to disk")
        print("D Toggle dance mode")
        print(",./ Turn on motor left zero right")
        print("m Toggle measurement")
        print("j Switch joystick control mode")
        print("b Print angle measurement from sensor")
        print("6 Enable/Disable live plot")
        print("5 Interrupts for histogram plot")
        print("***********************************")

    def loadparams(self):

        self.Q = np.diag(config["controller"]["lqr"]["Q"])
        self.R = config["controller"]["lqr"]["R"]
        self.update()

    def saveparams(self):
        json_filepath = get_new_json_filename(self.controller_name)
        print(f"\nSaving parameters to {json_filepath}")

        p = {}
        p['R'] = float(self.R)
        Q_list = [float(self.Q[0, 0]), float(self.Q[1, 1]), float(self.Q[2, 2]), float(self.Q[3, 3])]
        p['Q'] = Q_list
        with open(json_filepath, 'w') as f:
            json.dump(p, f)

    def keyboard_input(self, c):
        if c == 'p':
            self.printparams()

        elif c == '2': # Increase input penalization
            self.R = inc(self.R)
            self.update()
            print("\nIncreased input penalization to {:}".format(self.R))
        elif c == '1': # Decrease input penalization
            self.R = dec(self.R)
            self.update()
            print("\nDecreased input penalization to {:}".format(self.R))
        elif c == 'w': # Increase position penalization
            self.Q[0][0] = inc(self.Q[0][0])
            self.update()
            print("\nIncreased position penalization {:}".format(self.Q[0][0]))
        elif c == 'q': # Decrease position penalization
            self.Q[0][0] = dec(self.Q[0][0])
            self.update()
            print("\nDecreased position penalization {:}".format(self.Q[0][0]))
        elif c == 's': # Increase velocity penalization
            self.Q[1][1] = inc(self.Q[1][1])
            self.update()
            print("\nIncreased velocity penalization {:}".format(self.Q[1][1]))
        elif c == 'a': # Decrease velocity penalization
            self.Q[1][1] = dec(self.Q[1][1])
            self.update()
            print("\nDecreased velocity penalization {:}".format(self.Q[1][1]))
        elif c == 'r': # Increase angle penalization
            self.Q[2][2] = inc(self.Q[2][2])
            self.update()
            print("\nIncreased angle penalization {:}".format(self.Q[2][2]))
        elif c == 'e': # Decrease angle penalization
            self.Q[2][2] = dec(self.Q[2][2])
            self.update()
            print("\nDecreased angle penalization {:}".format(self.Q[2][2]))
        elif c == 'f': # Increase angular velocity penalization
            self.Q[3][3] = inc(self.Q[3][3])
            self.update()
            print("\nIncreased angular velocity penalization {:}".format(self.Q[3][3]))
        elif c == 'd': # Decrease angular velocity penalization
            self.Q[3][3] = dec(self.Q[3][3])
            self.update()
            print("\nDecreased angular velocity penalization {:}".format(self.Q[3][3]))
        elif c == 'S':
            self.saveparams()

    def controller_reset(self):
        pass
