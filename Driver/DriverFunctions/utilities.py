"""
Here are all the function necessary for running the CartPole which does not fit well in any category
But the ultimate aim is to have this file empty and all functions assigned to some category
"""

import sys


def terminal_check():
    if not sys.stdin.isatty():
        print('Run from an interactive terminal to allow keyboard input')
        quit()