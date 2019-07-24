# inverted-pendulum
Work related to the Inverted Pendulum Hardware

The Inverted Pendulum is powered by a ST32F103C8T6 microcontroller. `FactoryFirmwareImage.bin` contains the fimrware image as shipped by the factory. It was pulled directly from the micro using a debugger.

The new firmware built during the workshop enables the following features:
* PD control of balance and position can be run on either the micro or on a PC.
* USB interface to a PC (send angle and position, receive motor velocity).
* Real-time tunable gains on the micro.
* Motor safety cut off - prevents the motor from melting if the cart hits a limit.

# Requirements to Build/Program the Micro
* KEIL MDK development environment.
* STLink debugger.
