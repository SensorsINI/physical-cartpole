import pygame
from pygame import joystick

pygame.init()
pygame.joystick.init()
if joystick.get_count()==1:
    stick = joystick.Joystick(0)
    stick.init()
    axisNum = stick.get_numaxes()
    buttonNum = stick.get_numbuttons()
    joystickExists=True
    print('joystick found with '+str(axisNum)+' axes and '+str(buttonNum)+' buttons')
else:
    print('no joystick found')

axis=2
while True:
    pygame.event.get()
    vals=[]
    for i in range(axisNum):
        vals.append(stick.get_axis(i))
    buts=[]
    for i in range(buttonNum):
        buts.append(stick.get_button(i))
    print(f'axis={vals} but={buts}')
    pygame.time.delay(100)
