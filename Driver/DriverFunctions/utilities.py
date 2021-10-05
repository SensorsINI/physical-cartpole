"""
Here are all the function necessary for running the CartPole which does not fit well in any category
But the ultimate aim is to have this file empty and all functions assigned to some category
"""

import sys

def help():
    print("\n***********************************")
    print("keystroke commands")
    print("ESC quit")
    print("k toggle control on/off (initially off)")
    print("K trigger motor position calibration")
    print("=/- increase/decrease (fine tune) angle deviation value")
    print("[/] increase/decrease position target")
    print("w/q angle proportional gain")
    print("s/a angle derivative gain")
    print("z/x angle smoothing")
    print("r/e position proportional gain")
    print("f/d position derivative gain")
    print("c/v position smoothing")
    print("l toggle logging data")
    print("S/L Save/Load param values from disk")
    print("D Toggle dance mode")
    print(",./ Turn on motor left zero right")
    print("m Toggle measurement")
    print("j Switch joystick control mode")
    print("b Print angle measurement from sensor")
    print("***********************************")


def calibrate(CartPoleInstance):
    print("Calibrating motor position.... ")
    if not CartPoleInstance.calibrate():
        print("Failed to connect to device. Terminate program.")
        CartPoleInstance.close()
        exit()
    (_, POSITION_OFFSET, _, _, _) = CartPoleInstance.read_state()
    print("Done calibrating")

    return POSITION_OFFSET


def terminal_check():
    if sys.stdin.isatty():
        # running interactively
        print('running interactively from an interactive terminal, ok')
    else:
        print('run from an interactive terminal to allow keyboard input')
        quit()
