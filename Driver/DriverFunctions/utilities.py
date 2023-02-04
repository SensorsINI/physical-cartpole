"""
Here are all the function necessary for running the CartPole which does not fit well in any category
But the ultimate aim is to have this file empty and all functions assigned to some category
"""

import sys

def calibrate(CartPoleInstance):
    print("Calibrating motor position.... ")
    if not CartPoleInstance.calibrate():
        print("Failed to connect to device. Terminate program.")
        CartPoleInstance.close()
        exit()
    (_, _, POSITION_OFFSET, _, _, _) = CartPoleInstance.read_state()
    print("Done calibrating")

    return POSITION_OFFSET


def terminal_check():
    if sys.stdin.isatty():
        # running interactively
        print('Running interactively from an interactive terminal, ok')
    else:
        print('Run from an interactive terminal to allow keyboard input')
        quit()
