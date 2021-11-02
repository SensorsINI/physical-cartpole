"""
This is a linear-quadratic regulator
It assumes that the input relation is u = Q*u_max (no fancy motor model) !
"""

from scipy.linalg import solve_continuous_are, solve_discrete_are
import numpy as np

from Controllers.template_controller import template_controller
from CartPole.state_utilities import create_cartpole_state, cartpole_state_varname_to_index
from CartPole.cartpole_model import cartpole_jacobian, u_max, s0

from globals import *

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
        s = s0
        s[cartpole_state_varname_to_index('position')] = 0.0
        s[cartpole_state_varname_to_index('positionD')] = 0.0
        s[cartpole_state_varname_to_index('angle')] = 0.0
        s[cartpole_state_varname_to_index('angleD')] = 0.0
        u = 0.0

        jacobian = cartpole_jacobian(s, u)

        self.A = jacobian[:, :-1]
        self.B = np.reshape(jacobian[:, -1], newshape=(4, 1)) * u_max

        # Cost matrices for LQR controller
        self.Q = np.diag([0.0, 0.0, 0.0, 0.0])  # How much to punish x, v, theta, omega
        self.R = 100.0 # How much to punish the input

        # first, try to solve the ricatti equation
        # FIXME: Import needs to be different for some reason than in simulator.
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

        # self.motorcmd_save = []

    def update(self):
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

    def step(self, s: np.ndarray, target_position: np.ndarray, time=None):

        state = np.array(
            [[s[cartpole_state_varname_to_index('position')] - target_position], [s[cartpole_state_varname_to_index('positionD')]], [s[cartpole_state_varname_to_index('angle')]], [s[cartpole_state_varname_to_index('angleD')]]])

        motorCmd = np.asscalar(np.dot(-self.K, state))
        # self.motorcmd_save.append(motorCmd)

        # Clip Q
        if motorCmd > 1.0:
            motorCmd = 1.0
        elif motorCmd < -1.0:
            motorCmd = -1.0
        else:
            pass

        return motorCmd

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
        print("S/L Save/Load param values from disk")
        print("D Toggle dance mode")
        print(",./ Turn on motor left zero right")
        print("m Toggle measurement")
        print("j Switch joystick control mode")
        print("b Print angle measurement from sensor")
        print("***********************************")

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

    def controller_reset(self):
        Q = 0
        return Q
