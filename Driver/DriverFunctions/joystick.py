# pygame needs python 3.6, not available for 3.7
import pygame  # pip install -U pygame
# older:  conda install -c cogsci pygame; maybe because it only is supplied for earlier python, might need conda install -c evindunn pygame ; sudo apt-get install libsdl-ttf2.0-0
import pygame.joystick as joystick  # https://www.pygame.org/docs/ref/joystick.html
from globals import JOYSTICK_POSITION_KP, TRACK_LENGTH


def setup_joystick():
    pygame.init()
    joystick.init()
    if joystick.get_count() == 1:
        stick = joystick.Joystick(0)
        stick.init()
        axisNum = stick.get_numaxes()
        buttonNum = stick.get_numbuttons()
        joystickMode = 'not active'  # toggles to 'position' with 'j' key
        print('joystick found with ' + str(axisNum) + ' axes and ' + str(buttonNum) + ' buttons')
    else:
        stick = None
        joystickMode = None
        print('no joystick found, only PD control or no control possible')

    return stick, joystickMode


def get_stick_position(stick):
    pygame.event.get()  # must call get() to handle internal queue
    return stick.get_axis(0)  # 0 left right, 1 front back 2 rotate

# Other joystick actions (present as comment in original file):
# for event in pygame.event.get(): # User did something.
#     if event.type == pygame.QUIT: # If user clicked close.
#         done = True # Flag that we are done so we exit this loop.
#     elif event.type == pygame.JOYBUTTONDOWN:
#         print("Joystick button pressed.")
#     elif event.type == pygame.JOYBUTTONUP:
#         print("Joystick button released.")

def motorCmd_from_joystick(joystickMode, stickPos, position):
    # todo handle joystick control of cart to position, not speed - is this resolved?
    if joystickMode == 'speed':
        Q = stickPos
    elif joystickMode == 'position':
        position_normed = position/TRACK_LENGTH
        normed_distance_difference = 0.5*(stickPos - position_normed)
        Q = normed_distance_difference * JOYSTICK_POSITION_KP
    else:
        error_string = 'Unknown joystick mode: {}'.format(joystickMode)
        raise NameError(error_string)
    return Q

# The rest of this file is what was previously in joystick_tester.py
# Run this script as main to test the joystick
if __name__ == '__main__':
    pygame.init()
    pygame.joystick.init()
    if joystick.get_count() == 1:
        stick = joystick.Joystick(0)
        stick.init()
        axisNum = stick.get_numaxes()
        buttonNum = stick.get_numbuttons()
        joystickExists = True
        print('joystick found with ' + str(axisNum) + ' axes and ' + str(buttonNum) + ' buttons')
    else:
        print('no joystick found')

    axis = 2
    while True:
        pygame.event.get()
        vals = []
        for i in range(axisNum):
            vals.append(stick.get_axis(i))
        buts = []
        for i in range(buttonNum):
            buts.append(stick.get_button(i))
        print(f'axis={vals} but={buts}')
        pygame.time.delay(100)

