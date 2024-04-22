# Physical Cartpole Robot (physical-cartpole)

![NC_short_pole_1kHz.gif](Docs%2FNC_short_pole_1kHz.gif)

## Overview
This repository contains software, firmware and hardware to operate a 
[commercial cartpole robot](https://de.aliexpress.com/item/1005004322352088.html).
It provides integration with [Neural Control Tools](https://github.com/SensorsINI/Neural-Control-Tools).
The cartpole robot producer and developers of this repository are independent parties
hence we cannot guarantee the compatibility with future versions of the robot.

## Features

* USB interface to a PC
* Support for original STM but also for Zybo Z7-20
* Integration with tailored [Cartpole Simulator](https://github.com/SensorsINI/CartPoleSimulation)
and tools for [system identification with neural networks](https://github.com/SensorsINI/SI_Toolkit) and [control](https://github.com/SensorsINI/Control_Toolkit).
* Various control modes 
  * Control from PC: PID, MPC and neural controller
  * Control from STM: PID
  * Control from Zybo Z7-20: PID and FPGA-accelerated neural controller
    
  All controllers can stabilize the pole in the upright position and follow target position.
MPC and neural controller can also swing-up the pole.
* Template with examples for programing experiment sequence for automated data collection.
* Auxiliary functions allowing easier calibration of the robot, motor safety cut off, and more.


## Hardware

### BOM in short
To operate cartpole, beyond the cartpole robot itself (see "Original setup" below), you need:
* If you want to use it with STM32:
  * Make sure to buy cartpole **with STM32 board**.
  * STM32 programmer (e.g. STLink or J-Link)
* If you want to use it with Zybo-Z7-20:
  * [Zybo-Z7-20 board](https://digilent.com/shop/zybo-z7-zynq-7000-arm-fpga-soc-development-board/)
  
     We are using Zybo-Z7 **-20** board, and our current designs consumes over the half of the FPGA resources.
With some additional optimization it should be possible to fit well performing neural controllers on even smaller Zynq chips.
No matter if you switch to smaller or bigger board, you will probably need to adjust the design
regarding input and output pins and buttons, LEDs and switches assignments.
  * You need to prepare the analog filter with voltage divider for angle (ADC) input and H-bridge for motor.
  See below for the details, here just list of items:
      * 12V, 5A power supply for the motor (usually comes with the cartpole robot)
      * 2 x [Pmod TPH2](https://digilent.com/shop/pmod-tph2-12-pin-test-point-header/) (12-pin Test Point Header),
        we use it to solder H-bridge and analog filter on it
      * H-Bridge: [Pololu TB6612FNG Dualer Motortreiber](https://www.berrybase.ch/pololu-tb6612fng-dualer-motortreiber)
      * 1 x Ferrite bead, we took just what we had in house to clean power supply for potentiometer
      * Capacitors: 
        * 1 x 1uF for filtering angle signal from potentiometer (RC time constant  =  0.1 ms; you can adjust this one)
        * 1 x 20uF and 1 x 4.7 uF for filtering power supply for potentiometer
        * 1 x 470uF for filtering power supply for H-Bridge - probably facultative, just any big capacitor will do
      * Resistors:
        * 12 kOhm for voltage divider for potentiometer
        * 100 Ohm for filtering angle signal from potentiometer (RC time constant  =  0.1 ms)
      * Some wires to solder connections
      * Barrel connector for power supply to be soldered to H-Bridge
* Additionally, you might want to buy:
    * Possibly a cable to connect the STM32 or Zybo board (micro USB) to PC, not sure if included with cartpole robot or Zybo.
      You probably want a rather long one for convenience (1-2m).
    * Spare motor **with Encoder** - we use Pololu 19:1 Metal Gearmotor, 37Dx68Lmm, 12V with 64 CPR Encoder [#4751](https://www.pololu.com/product/4751) 
    * Power supply for Zybo-Z7 board - 5V, 2.5A, barrel connector; the board can be powered also from USB.
    * SD card and SD card reader - to flash Zybo board from it without need to connect to computer after start up
      (should be also possible to program the chip with flash memory on Zybo board, but we did not test it)
    * Spare STM boards if you want to work mostly with them - they seem to be fragile
    * metal bars for experiments with poles of different mass and length -
we had these in-house so you have to find on your own where to buy them.
With a rough measurement the pole mounting hole is 6mm in diameter, the pole is 5.7mm in diameter.
  * Some material to decorate the cartpole - don't forget the artistic side of the project! ;-)

### Original setup

We bought our cartpole robot on AliExpress.
As of today there are a few links to the product e.g. [this](https://de.aliexpress.com/item/1005004322352088.html).
It is worth to search for "inverted pendulum" on AliExpress to find the cheapest option and compare with the picture below.
Out of different versions we have the one using STM32 (ST32F103C8T6) board. **Not** the one with Arduino.
If you plan to use Zybo-Z7-20 instead, you might be fine buying just the mechanical part of the robot
(but probably need to buy power supply for the motor separately in such a case).
![cartpole_official_picture.jpg](Docs%2Fcartpole_official_picture.jpg).

### Motors

As the replacement of the original motor
we use Pololu 19:1 Metal Gearmotor, 37Dx68Lmm, 12V with 64 CPR Encoder [#4751](https://www.pololu.com/product/4751).
This motor has a similar characteristics to the original one, but its connector has wires in different order.
For STM setup the original motor is the default, the Pololu needs additional "adapter".
This adapter is shown below:
![motor_adapter.png](Docs%2Fmotor_adapter.png)

TODO:
As for my understanding same adapter (up to the male-female reverse endings) would do to port original motor to Zybo-Z7-20.
I have not tried this yet.
In this case the end with the white tape should be on the Zybo side - all the cables keep their color-indicated meaning as on the picture -
and the other end connected to motor taking care that the wiring is as if it were connected directly to the stm board.

You can notice that the polulu motor with the adapter has still the reversed sign of the encoder signal.
We use this feature in the calibration to distinguish the motors:
while applying the same voltage to the motor, the encoder signal should have the opposite sign for the Pololu motor.


### Custom PMOD connectors for Zybo-Z7-20
#### H-bridge
Unfortunately the Digilent h-bridge PMODs does not meet specification of the cartpole motor.
The original setup with STM32 uses TB6612FNG H-brdige.
We bought it as [Pololu TB6612FNG Dualer Motortreiber](https://www.berrybase.ch/pololu-tb6612fng-dualer-motortreiber)
and build our own PMOD H-Bridge by soldering it to [Pmod TPH2](https://digilent.com/shop/pmod-tph2-12-pin-test-point-header/).
Below hopefully self-explanatory picture on how to prepare the h-bridge.
The PMOD should be connected to the Zybo's JE PMOD connector, the other end should be connected to the motor.
The colors of the wires on the picture match the colors of the wires at the Pololu motor connector.
The barrel connector is for motor supply (12V, 5A).
![HBridgePMOD.png](Docs%2FHBridgePMOD.png)

#### Analog Filter
To get a clean angle measurement from the potentiometer, we designed an analog filter.
We also need a voltage divider as the board delivers 3.3V to the potentiometer
and ADC (XADC) on Zynq operate in the range 0-1V.
The potentiometer output is read from PIN 3 of PMOD connector JA on Zybo Z7-20.
The schematic is provided below:
![Analog-Filter-Zynq-Angle.png](Docs%2FAnalog-Filter-Zynq-Angle.png)

Below some picture to facilitate the assembly.
![pot_pmod_front.png](Docs%2Fpot_pmod_front.png)

Visible capacitor is 1uF for filtering angle signal from potentiometer.
It is place so that it can be easily replaced with different value to adjust the filter's time constant.
Also visible a lot of glue to keep the connection stable when the cartpole is moving.

![pot_pmod_back.png](Docs%2Fpot_pmod_back.png)
From top to bottom visible is the ferrite bead,
the 20 uF capacitor filtering potentiometer input
 and connection shorting pin 9 to ground (the last one just because haw we placed the circuits).
The remaining resistors and capacitor are hidden under heat shrink tube.



## Set up and installation

### PC (Python project)

0. Create python 3.11 environment, if you haven't done it yet. e.g.

    `conda create -n CPP python=3.11`

    `conda activate CPP`

    `conda install pip`

    Where CPP is the name of the environment (stands for CartPole Physical).

1. Clone the repository

    `git clone https://github.com/SensorsINI/physical-cartpole`

2. Install the dependencies

    `pip install -r requirements.txt`

    This installs all the dependencies - it is quite a lot of packages, so it may take a while.


### STM32
* Connect the STLink or the J-link to the STM32 board and to the PC. For j-link we attach the picture, for st-link, you have to figure it out on your own.
![jtag_programming.png](Docs%2Fjtag_programming.png)
* Open STM32CubeIDE and go to `File -> Import... -> Existing Projects into Workspace`
and import project from [Firmware/FactoryFirmware CubeIDE](Firmware%2FCartPole%20CubeIDE).
* Go to hardware_bridge.h and comment out `#define ZYNQ` and uncomment `#define STM`.
* Build the project (hammer icon).
* Right click on the project and select `Run As -> STM32 C/C++ Application`. Go to `Debugger` tab and set up your J-link or ST-link.
For us it looks as follows:
![RunConfigurationSTM.png](Docs%2FRunConfigurationSTM.png)

* Run the project. The firmware should be loaded to the STM32 board.
For STM board, the firmware is saved in the flash memory,
so you do not need to load it every time you start the board.
This also means that the original firmware will be overwritten,
so if you intend to use it later, make sure to fetch it from the chip and save.

* Open `Driver/globals.py` and set `CHIP` variable to `STM`

## BELOW NOT FULLY UPDATED YET

### ZYNQ
* The project is configured to work with ZYNQ as default after download.
In case you used it first with STM, you need to undo the relevant changes:
  * Go to hardware_bridge.h and comment out `#define STM` and uncomment `#define ZYNQ`.
  * Open globals.py and set `CHIP` variable to `ZYNQ`

  Otherwise you can ignore this first point.
* 

## Running
Do the steps as described in `Set up and installation` section.

The main module to control the cartpole from PC is [control.py](Driver/control.py).

Parameters are in [globals.py](Driver/globals.py).

To get working controllers you first need to calibrate the cartpole.

## Calibration

There are mainly three things which need to be calibrated:

* middle of the cartpole track
* motor power
* vertical angle

There is also a friction which might play an important role in cartpole modeling and control,
but you should be able to get a working controller without adjusting it.
Hence for friction, see the wiki page.

### Middle of the cartpole track
This is the only calibration value that is not hardcoded and needs to be recalibrated each time cartpole is powered on.
After starting the python program to drive cartpole from PC,
press `Shift+K`.
The cartpole will move to the left and right boundary of the track.
and stop in the middle. 

If the cartpole get stuck it might mean that the motor power is too low to overcome friction.
This usually happens just before stopping in the middle (as it slows down not to overshoot),
hence it might be confusing - the cartpole seems in the middle but PC is still waiting for the calibration to finish.
In this case you need to increase it in calibration function in firmware.

If you are running multiple dozens of experiments,
you might want to recalibrate the cartpole from time to time.

The calibration also allows cartpole to distinguish 

## Notes

### Firmware
The [firmware](Firmware) folder has the ST firmware.

### Modes of Operation

The pendulum microcontroller firmware has two modes of operation:

_(1) Self-Contained Mode_  
In this mode, the pendulum is controlled directly by the firmware using an onboard basic PD control scheme. A PC is not required at all for this mode. However you can use the PC to adjust the control parameters on-the-fly, as well as read out set points and other useful values generated by the on-board control algorithm in real time. If you want 'factory mode', just boot up the controller and set it running.

_(2) PC Control Mode_  
In this mode, the pendulum firmware runs as an interface to the physical hardware. Control is performed over USB by an algorithm running on another connected device (PC, FPGA, etc). The firmware outputs the current pendulum angle and cart position at regular intervals, and accepts motor speed and direction as input. The frequency of the control loop running on the PC is governed by the period set for outputting angle/position from the pendulum micro (currently hardcoded to 5 ms).

In either mode, when enabling control of the pendulum for the first time, the firmware will automatically run a calibration routine. During this routine, the cart will slowly move from left to right in order to determine the maximum limits of movement.

### Buttons

Two buttons are used by the firmware

_S1_  
This is the CPU's hard reset.

_S2_  
Used to switch between the 2 modes of operation in the pendulum firmware.

### LEDs

There is a blue LED (L2) that flashes periodically, fast or slow depending on which mode the firmware is operating in. Fast (200 ms period) -> Self-contained mode. Slow (1 s period) -> PC control mode.

### PC Interface

The interface to the pendulum firmware is through the `pendulum.py` module, which provides a series of high-level functions to configure and command various things on the microcontroller. `control.py` contains an example implementation of a PD controller running the PC and interfacing directly with the pendulum firmware.

### Parameters

Parameters that can be adjusted via the PC are listed below. Note that raw values are used for set points and control gains.
* Angle set point (~3110 is vertical).
* Angle average length (number of samples to average over when determining current angle).
* Angle smoothing factor (0 to 1.0 - used in 1st order low-pass filter).
* Angle control gains (P and D).
* Position set point (0: centre, <0: left, >0: right).
* Position control period (as a multiple of Angle control period - 5 ms).
* Position smoothing factor (0 to 1.0 - used in 1st order low-pass filter).
* Position control gains (P and D).

### Output

If enabled, the following data is streamed to the PC at regular 5 ms intervals:
* Current pendulum angle.
* Current cart position.
* Motor speed command calculated by onboard controller (0 when running in PC-control mode).

<!---## Zynq
The [zynq](zynq) folder has the Vivado and Petalinux project for the Minized zynq board. The Xilinx tools version is 2021.2. To replicate the projects you need to install both tools following the installation guides [Vivado](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiTo8K28Lf-AhUxXaQEHfyKDOMQFnoECAMQAQ&url=https%3A%2F%2Fdocs.xilinx.com%2Fr%2F2021.2-English%2Fug973-vivado-release-notes-install-license%2FDownload-and-Installation&usg=AOvVaw3DnvsfdstplLh6SIkN30Gq) and [Petalinux](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwjq55rD8Lf-AhXmUqQEHbBYCGYQFnoECAgQAQ&url=https%3A%2F%2Fdocs.xilinx.com%2Fr%2F2021.2-English%2Fug1144-petalinux-tools-reference-guide%2FInstalling-the-PetaLinux-Tool&usg=AOvVaw2rKoUMzp-5K6sm-C3m291-)

### Deploying the bsp file
To decompress the bsp file and get the Petalinux and Vivado project, run the next commands under [zynq](zynq) folder:
```console
source /path/to/petalinux/install/folder/settings.sh
petalinux-create -t project -s minized_sbc_base_2021_1.bsp
```
A new folder will be create with the petalinux project where you can find the Vivado project under the hardware folder.

### Apps cross-compilation
Minized zynq PS (Processing System) is a ARM based architecture, so we need to cross-compile the application to be deployed in the ARM cores os the Zynq. A [makefile](zynq/apps/makefile) template is ready in the [apps](zynq/apps/) folder to be modified according our app requierements.
* <u>Cross-compiler installation</u>
```console
sudo apt-get install g++-8-arm-linux-gnueabihf
```
* <u>Compile the user app</u>
    - Create a folder for the new app and copy the makefile template and all app source file.
    - Modify the makefile template according to the app requirements.
    - Run the following command in the new app folder.
    ```console
    make
    ```-->
## Zynq
The [zynq](zynq) folder has the Vivado and Vitis projects for both Zybo and Zedboard boards. The Xilinx tools version is 2021.2. To replicate the projects you need to install Vivado (including Vitis) following the installation guides [Vivado](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiTo8K28Lf-AhUxXaQEHfyKDOMQFnoECAMQAQ&url=https%3A%2F%2Fdocs.xilinx.com%2Fr%2F2021.2-English%2Fug973-vivado-release-notes-install-license%2FDownload-and-Installation&usg=AOvVaw3DnvsfdstplLh6SIkN30Gq)

### Creating Vivado project
Open Vivado tool and click on the TCL console tab. This tab is place at window bottom. Navigate to [zynq](zynq) folder:
```console
cd path/to/zynq/folder
```

Then run the tcl script using the next command:
```console
source ./zybo.tcl
```
The vivado project will be create and it will be ready for synthesis. Yo can modify this project according your needs.

### Creating Vitis project
Vitis projects are compressed in [zybo_vitis_pot_motor_test.zip](zynq/zybo_vitis_pot_motor_test.zip) file. So to import then into Vitis follow the next steps:
- Open Vitis using a new workspace
- Click on File -> Import...
- Select "Vitis project exported zip file" option
- Browse [zybo_vitis_pot_motor_test.zip](zynq/zybo_vitis_pot_motor_test.zip) file
- Select all the projects
- Click Finish

The projects are ready to be run in the Zybo board.


## Known issues

sys.stdin.read(1) which is used for keyboard input causes the debugger of Pycharm to hang.

For USB-UART bridge if you use FTDI chip - not the case for original stm board, but a default bridge for Zybo board or PMODs
you will get 16ms latency due to FTDI chip waiting for minimal number of bytes to transfer to PC. You can disable this feature programmatically.