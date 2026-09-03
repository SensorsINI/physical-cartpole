# Physical Cartpole Robot (physical-cartpole)

![NC_short_pole_1kHz.gif](Docs%2FNC_short_pole_1kHz.gif)

## Overview

This repository contains software, firmware, and hardware to operate a
[commercial cartpole robot](https://de.aliexpress.com/item/1005004322352088.html)
from a PC and from a Zybo Z7-20 (Zynq-7000). The default lab/show path is
**Zybo + AMP firmware**: CPU0 runs the show mux and motor loop; CPU1 runs the
RPGD worker. STM32 on the original robot is still supported.

Once the hardware is up you can swing up and stabilize from the board alone
(QSPI + switches), log or retarget from a PC while the chip controls, run
PID / MPC / neural imitators on the host, record scripted experiments, or use
the CartPoleSimulation GUI with no robot attached. How those modes fit together
is in [Docs/operating.md](Docs/operating.md).

The cartpole producer and the authors of this repository are independent parties,
so compatibility with future robot revisions is not guaranteed.

Integration with [Neural Control Tools](https://github.com/SensorsINI/Neural-Control-Tools),
[CartPoleSimulation](https://github.com/SensorsINI/CartPoleSimulation),
[SI_Toolkit](https://github.com/SensorsINI/SI_Toolkit), and
[Control_Toolkit](https://github.com/SensorsINI/Control_Toolkit).

## Publications

* [A-NC: Adaptive Neural Control with implicit online inference of privileged parameters](https://proceedings.mlr.press/v283/paluch25a.html)
  (L4DC 2025, PMLR 283:987–998)
* [Hardware Neural Control of CartPole and F1TENTH Race Car](https://arxiv.org/abs/2407.08681)
* [RPGD: A Small-Batch Parallel Gradient Descent Optimizer with Explorative Resampling
for Nonlinear Model Predictive Control](https://www.zora.uzh.ch/id/eprint/254218/1/RPGD_ICRA_2023.pdf)

## Features

* USB-UART to a PC (230400 baud on Zybo)
* Zybo Z7-20 on-chip controllers selected by SW0–SW3 (one-hot show mux):
  AMP RPGD, Dense-8 C, LSTM C, short-pole PL neural imitator
* Dual-core AMP: CPU0 starts CPU1; RPGD runs on the worker
* QSPI standalone boot (no PC after programming)
* JB Pmod slider owns `target_position` when `USE_EXTERNAL_INTERFACE` is on
* PC driver: PID, MPC (rpgd-c), neural imitator, CSV, live plotter, dance, joystick
* STM32: onboard PID (board button + PC `u`)
* Experiment-protocol templates (IROS script, motor ID, swing-up batches)
* Motor safety: output disabled until armed; stall cut; brake before track ends

## Documentation

| Document | What it covers |
|---|---|
| This README | Hardware, install, calibration, flashing, known issues |
| [Docs/operating.md](Docs/operating.md) | Day-to-day use: modes, buttons, switches, LEDs, slider, safety, typical sessions |
| [Docs/pc-driver.md](Docs/pc-driver.md) | `control.py`: keys, controllers, logging, plotter, protocols, simulation |
| [tools/slider_pmod/README.md](tools/slider_pmod/README.md) | JB slider map and UART check |
| [Driver/CartPoleSimulation/README.md](Driver/CartPoleSimulation/README.md) | Simulator GUI, data generation, training pipeline |
| [examples/models](examples/models/README.md) | Preserved on-chip / PC network bundles |
| [Docs/SecLoc_Experiment_Platform.md](Docs/SecLoc_Experiment_Platform.md) | SecLoc / Zedboard research path (not the Development show default) |

## Contents

* [Quick start (Zybo)](#quick-start-zybo)
* [Hardware](#hardware)
* [Set up and installation](#set-up-and-installation)
* [Calibration](#calibration)
* [Using the robot](#using-the-robot)
* [Standalone / QSPI (no PC at power-up)](#standalone--qspi-no-pc-at-power-up)
* [STM32](#stm32)
* [FPGA and Vitis rebuild](#fpga-and-vitis-rebuild)
* [Known issues](#known-issues)

## Quick start (Zybo)

Keep the track clear. For a first boot after flashing, leave the motor disconnected
or be ready to cut 12 V.

**JP5 = JTAG** while programming flash or loading over JTAG. **JP5 = QSPI**
(two center pins) only for a cold boot from flash. Close Vitis; it steals JTAG.

1. Program (pick one):
   * QSPI: `Firmware/Scripts/program_show_qspi.sh`
   * JTAG RAM (does not write flash): `Firmware/Scripts/program_rpgd_amp_production.sh`
2. Power on with **SW0–SW3 all off**. The motor command stays 0 until you arm.
3. Hang the pole straight down. Press **BTN0**. That disarms control, zeros PWM,
   captures `ANGLE_HANGING`, and leaves control off. RGB flashes white on success.
4. Arm with **BTN4**, or `python Driver/control.py` then **`u`**.
5. Turn on **exactly one** switch:

   | Switch | On-chip controller |
   |---|---|
   | SW0 | AMP RPGD (CPU1) |
   | SW1 | Dense-8 C (PS) |
   | SW2 | LSTM C (PS) |
   | SW3 | Short-pole PL neural imitator |

   All off, or more than one on: Q = 0, motor stopped.
6. **BTN0** again: disarm + new hanging capture. **BTN5** or PC **`K`**: track
   center only (does not change hanging).

What the switches, RGB LEDs, and slider do after this first boot, and how to
log or take over from the PC:
[Docs/operating.md](Docs/operating.md), [Docs/pc-driver.md](Docs/pc-driver.md).

The show image is AMP CPU0 (`CartPoleFirmware_rpgd_amp_cpu0.elf`), not the Vitis
Debug ELF. FSBL loads that one app; CPU0 copies the CPU1 blob into DDR and
releases it. Do not pack a second ELF or `destination_cpu`.

---

## Hardware

### BOM in short

To operate the cartpole, beyond the robot itself (see [Original setup](#original-setup)):

* If you want to use it with STM32:
  * Buy the cartpole **with STM32 board**.
  * STM32 programmer (ST-Link or J-Link)
* If you want to use it with Zybo-Z7-20:
  * [Zybo-Z7-20 board](https://digilent.com/shop/zybo-z7-zynq-7000-arm-fpga-soc-development-board/)

     We use Zybo-Z7 **-20**. The current design consumes over half of the FPGA.
     A smaller or larger Zynq needs pin, button, LED, and switch remapping.
  * Analog filter with voltage divider for the angle ADC, and an H-bridge for the motor:
      * 12 V, 5 A power supply for the motor (usually ships with the robot)
      * 2 × [Pmod TPH2](https://digilent.com/shop/pmod-tph2-12-pin-test-point-header/)
        (H-bridge and analog filter)
      * H-bridge: [Pololu TB6612FNG](https://www.berrybase.ch/pololu-tb6612fng-dualer-motortreiber)
      * 1 × ferrite bead (potentiometer supply)
      * Capacitors: 1 µF (angle RC, τ ≈ 0.1 ms), 20 µF and 4.7 µF (potentiometer
        supply), 470 µF (H-bridge supply, optional)
      * Resistors: 12 kΩ divider, 100 Ω in the angle RC
      * Wires; barrel connector on the H-bridge
* Also useful:
    * Long micro-USB cable (1–2 m)
    * Spare motor **with encoder**: Pololu 19:1 Metal Gearmotor 37Dx68Lmm 12 V,
      64 CPR [#4751](https://www.pololu.com/product/4751)
    * Zybo 5 V, 2.5 A barrel supply (USB power also works)
    * Zybo on-board QSPI — supported standalone boot. JP5 on the two center pins
      labeled QSPI. Image: [Firmware/Scripts/cartpole_qspi.bif](Firmware/Scripts/cartpole_qspi.bif)
    * SD card — optional fallback (`BOOT.BIN` on the card, JP5 = SD)
    * Spare STM boards if you work mostly with them
    * Metal bars for other pole masses/lengths (mounting hole ~6 mm, pole ~5.7 mm)

### Original setup

We bought the robot on AliExpress
([example listing](https://de.aliexpress.com/item/1005004322352088.html)).
Search for “inverted pendulum” and match the picture. We have the STM32
(ST32F103C8T6) version, **not** Arduino. For Zybo-only use, the mechanical
assembly plus a motor supply is enough.

![cartpole_official_picture.jpg](Docs%2Fcartpole_official_picture.jpg)

#### Belt tension

Tight enough that the belt does not slip on the motor pulley, not so tight that
it side-loads the motor. A practical check: pushing the upper belt down should
just meet the lower belt and then go taut.

### Motors

Replacement motor: Pololu 19:1 37Dx68Lmm 12 V with 64 CPR encoder
[#4751](https://www.pololu.com/product/4751). Similar dynamics to the original;
the connector wire order differs. On STM the original motor is the firmware
default; Pololu needs the adapter below.

![motor_adapter.png](Docs%2Fmotor_adapter.png)

The same adapter (male/female reversed) should port the original motor to Zybo,
with the taped end on the Zybo side and STM-equivalent color mapping on the
motor end. That Zybo wiring has **not** been tried on the lab robots.

The Pololu-plus-adapter encoder sign is reversed versus the original motor.
Track calibration uses that to distinguish motors.

### Custom PMODs for Zybo-Z7-20

#### H-bridge

Digilent H-bridge PMODs do not meet this motor. The STM robot uses TB6612FNG;
we solder the Pololu module to a Pmod TPH2. Plug it into **JE**. Wire colors
match the Pololu motor connector. Barrel is 12 V, 5 A.

![HBridgePMOD.png](Docs%2FHBridgePMOD.png)

#### Analog filter

The potentiometer is 3.3 V; Zynq XADC is 0–1 V, so a divider plus RC is required.
Angle is PIN 3 of **JA**.

![Analog-Filter-Zynq-Angle.png](Docs%2FAnalog-Filter-Zynq-Angle.png)

![pot_pmod_front.png](Docs%2Fpot_pmod_front.png)

The visible 1 µF is the angle filter; it is easy to swap to change τ.
Glue helps the wiring survive cart motion.

![pot_pmod_back.png](Docs%2Fpot_pmod_back.png)

From top to bottom: ferrite bead, 20 µF on the potentiometer supply, pin 9
shorted to ground. Other parts are under heat-shrink.

#### Target-position slider (JB)

A Pmod slider on **JB** (PmodAD1) sets `target_position` when
`USE_EXTERNAL_INTERFACE` is defined in
[hardware_bridge.h](Firmware/Src/CartPoleFirmware/hardware_bridge.h)
(default on Development). Firmware maps ADC affinely between the parked rails
(electrical mid = 0). The ends are ±`SliderTargetHalfLength` (**0.12 m**, inside
the 0.198 m track half-length). The driver displays the chip target and does
not send `CMD_SET_TARGET_POSITION`.

PmodAD1 must be built with SPI counts **40/120/1000/800** at 100 MHz.
Calibration and check scripts: [tools/slider_pmod/README.md](tools/slider_pmod/README.md).
Close the GUI before a UART slider check (230400, Digilent interface 1).

---

## Set up and installation

### PC (Python)

0. Python 3.11 environment, e.g.

    `conda create -n cpp python=3.11`

    `conda activate cpp`

    `conda install pip`

1. Clone with submodules:

    `git clone --recurse-submodules https://github.com/SensorsINI/physical-cartpole`

If `CartPoleSimulation`, SI_Toolkit, or Control Toolkit are empty:

```bash
git submodule update --init --recursive
```

Prefer a pinned submodule checkout over `--remote` unless you intend to move them.
`SI_Toolkit` and `Control_Toolkit` are pinned to `cartpole_master` so
[f1tenth_development_gym](https://github.com/F1Tenth-INI/f1tenth_development_gym)
can keep using toolkit `main`/`master`.

2. `pip install -r requirements.txt`

3. Path helper (adjust the conda hook path):

```bash
alias xilinx='source /tools/Xilinx/Vivado/2020.1/settings64.sh'

cpp() {
  eval "$(/(path to miniconda3)/bin/conda shell.bash hook)"
  conda activate cpp
  export PYTHONPATH=$HOME/physical-cartpole:$PYTHONPATH
  export PYTHONPATH=$HOME/physical-cartpole/Driver:$PYTHONPATH
  export PYTHONPATH=$HOME/physical-cartpole/Driver/CartPoleSimulation:$PYTHONPATH
}
```

In PyCharm, mark `physical-cartpole`, `Driver`, and `Driver/CartPoleSimulation`
as Sources Root. `conda env config vars set` is not reliable for this.

### Zybo firmware and driver defaults

Xilinx tools are **Vivado / Vitis 2020.1**. Development defaults
([globals.py](Driver/globals.py) and
[hardware_bridge.h](Firmware/Src/CartPoleFirmware/hardware_bridge.h)):

* `CHIP = "ZYNQ"`, `ZYNQ_BOARD = "ZYBO_Z720"`
* `#define ZYNQ`, `#define ZYBO_Z720`, `#define USE_EXTERNAL_INTERFACE`
* `SHOW_SWITCH_MUX = True` — chip ignores PC period / derivative N
* `MOTOR = 'POLOLU'`

UART is **230400** on the Digilent FTDI **interface 1** (usually `/dev/ttyUSB1`).
The FTDI default latency is 16 ms; the driver sets 1 ms on Linux. On Windows,
set the latency timer in the FTDI driver (see
[Docs/SettingLatencyTimerOnWindows.png](Docs/SettingLatencyTimerOnWindows.png)).
No MacOS fix is known.

If you previously built for STM, restore `#define ZYNQ` and `CHIP = "ZYNQ"`.

Daily JTAG load (RAM only):

```console
Firmware/Scripts/program_rpgd_amp_production.sh
```

QSPI flash (JP5 must be JTAG; boot-mode register `0xF800025C` = 0):

```console
Firmware/Scripts/program_show_qspi.sh
```

Then power off, JP5 = QSPI, power on.

### STM32 firmware

* Connect ST-Link or J-Link. The J-Link picture applies to ST-Link; the
  connector is the same.
![jtag_programming.png](Docs%2Fjtag_programming.png)
* STM32CubeIDE → `File -> Import... -> Existing Projects into Workspace` →
  [Firmware/CubeIDE/CartPoleFirmware](Firmware/CubeIDE/CartPoleFirmware).
![CubeImportProject.png](Docs/CubeImportProject.png)
* In `hardware_bridge.h`: comment `#define ZYNQ`, uncomment `#define STM`.
* Build, then `Run As -> STM32 C/C++ Application`. Debugger tab: your probe.
![RunConfigurationSTM.png](Docs%2FRunConfigurationSTM.png)
* STM firmware lives in flash and overwrites the factory image. Save the
  original first if you still need it.
* `Driver/globals.py`: `CHIP = "STM"`

---

## Calibration

Three things matter for control: track center, hanging / zero angle, and the
motor map. Friction identification is optional for a first working controller;
see [Driver/DataAnalysis/MotorAndCartFriction/README.md](Driver/DataAnalysis/MotorAndCartFriction/README.md).

### Track center

Not stored. Recalibrate after every power cycle (`K` on the PC, **BTN5** on Zybo).
The cart drives to both ends and stops in the middle. If it stalls near center,
calibration speed in firmware may be too low for friction.

`K` / BTN5 never change `ANGLE_HANGING`.

### Motor type (ORIGINAL vs POLOLU)

Calibration also detects encoder sign and sets `MOTOR`. That value is hardcoded
**independently** in firmware (`parameters.c`) and in `globals.py`.

* Starting the Python driver sends the PC motor type to the chip and overwrites
  firmware RAM.
* Calibration from the PC updates both sides for that session.
* Calibration from a board button updates firmware RAM only.
* After reset, each side reloads its compile-time / file default.

In the lab, motor type also selects which robot’s hanging constants apply
(`ANGLE_HANGING_POLOLU` vs `ANGLE_HANGING_ORIGINAL`).

### Motor power

Each motor (and wear) changes the Q → acceleration map. Record a bidirectional
step response (`m` until the step-response protocol is selected, then `n`),
then follow
[Driver/DataAnalysis/MotorAndCartFriction/README.md](Driver/DataAnalysis/MotorAndCartFriction/README.md).
Paste `MOTOR_CORRECTION` into `globals.py` **and**
[parameters.c](Firmware/Src/CartPoleFirmware/parameters.c) (reflash for on-chip).
Show-mux profiles currently share the LSTM/RPGD map
`{0.5733488, 0.0257380, 0.0258429}`.

### Angle: dead zone

Put the potentiometer dead zone at **horizontal** (usually left). With the PC
running, hold the pole horizontal and turn the pot joint until the raw ADC
wraps. Tighten the clamp afterward or the reading drifts.

On Zybo, hang the pole and press **BTN0**. That immediately disarms on-chip and
PC control, zeros PWM, and overwrites `ANGLE_HANGING` from a wrap-aware mean of
50 hardware-filtered 12-bit samples (aborted if the pole is moving or track
calibration is running). Control stays off. PC **`b`** is the same role but
averages 1000 streamed samples and does not write QSPI.

RGB **alternating red**: stored hanging places the dead zone within 20° of
vertical. Turn the screw toward horizontal and press BTN0 again. **Cyan** is a
centered cart target, not a fault.

After reset, hanging is the compile-time value in `parameters.c` (QSPI is not
loaded at boot). The first PC connect applies `globals.py` once, unless BTN0
already ran this boot. Later `CMD_SET_CONTROL_CONFIG` (including after `K`) does
not change hanging. `b` forces RAM on the chip; BTN0 also writes QSPI at
`0xFD0000` / `0xFFF000`. To persist across reboot without BTN0, copy the value
into `ANGLE_HANGING_*` in `globals.py` and `parameters.c`.

BTN1–BTN3 are in the bitstream and can take `Button_SetAction(PL_BTN_n, ...)`
without another FPGA build.

### Full circle (`ANGLE_360_DEG_IN_ADC_UNITS`)

The ADC is 12-bit, but the dead zone means a physical turn is more than 4096
counts. Measure hanging vs upright on the side that does not include the wrap;
the circle is twice that difference. Script:
[Driver/DataAnalysis/AngleUpDown](Driver/DataAnalysis/AngleUpDown).

Firmware **does** define `ANGLE_360_DEG_IN_ADC_UNITS` in `parameters.c`. It
must match `globals.py`. Current Zybo Development values (new analog chain,
2026-09-03): hanging **3273.353**, circle **4068.73**. STM and Zybo numbers
differ (different robots and/or the 3.3 V → 1 V divider).

### Zero angle

Hang the pole, then **BTN0** or **`b`**. Gently balance upright: the reported
angle should oscillate around 0. Fine-tune with `=` / `-`, or edit
`ANGLE_HANGING_*` and restart.

---

## Using the robot

Hardware and flashing are above. Daily operation — show vs PC, what SW0–SW3
and the RGB LEDs mean, targets, safety — is
[Docs/operating.md](Docs/operating.md). The host program
(`python Driver/control.py`), every key, CSV / plotter, and experiment
protocols are [Docs/pc-driver.md](Docs/pc-driver.md).

Short map:

* **Show, no PC:** QSPI boot, BTN0, BTN4, exactly one of SW0–SW3, JB slider.
  Full first-boot list: [Quick start](#quick-start-zybo).
* **PC as logger:** same on-chip path, then `control.py` and **`u`** / **`l`**.
* **PC as controller:** set `CONTROLLER_NAME` in `Driver/globals.py`, **`k`**.
  That name is only the `k` path; switches still select the chip controller.
* **`k` and `u` are exclusive.** The slider, when `USE_EXTERNAL_INTERFACE` is
  on, owns `target_position` on both paths (`[` / `]` / `;` are ignored).

---

## Standalone / QSPI (no PC at power-up)

Hardcode hanging and motor map in `parameters.c` first, or press BTN0 after each
boot. Prove the same image on JTAG before flashing.

The show `BOOT.BIN` is secloc2026 FSBL +
`FPGA/bitstreams/cartpole_short_pole_secloc.bit` + AMP CPU0
(`CartPoleFirmware_rpgd_amp_cpu0.elf`). CPU0 starts CPU1. Image-range erase
only — **do not erase the entire 16 MiB** or you wipe hanging at `0xFD0000` /
`0xFFF000`.

1. JP5 = **JTAG**. Close Vitis. Build if needed:
   `RPGD_AMP_PRODUCTION=1 Firmware/Scripts/build_rpgd_amp_elfs.sh`
2. `Firmware/Scripts/program_show_qspi.sh`
   (uses [program_qspi_boot.tcl](Firmware/Scripts/program_qspi_boot.tcl);
   template BIF: [cartpole_qspi.bif](Firmware/Scripts/cartpole_qspi.bif)).
3. Power off. JP5 = **QSPI**. Power on. Hang, BTN0, BTN4, exactly one of SW0–SW3.

SD boot (same `BOOT.BIN` on a card, JP5 = SD) is a fallback.

Safety on this image:

* Motor output is latched off until BTN4 / `u` / `k`
* Stall: significant command with almost no encoder travel for 100 ms → command 0
* Position limits brake with a margin before the encoder range ends
* Invalid / bouncing DIP: Q = 0 for that tick

PL buttons need a platform whose `xparameters.h` lists `PL_BUTTONS_GPIO`.
Source of truth:
[FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_12_09_2025.tcl](FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_12_09_2025.tcl).
To patch an existing secloc project:
[add_pl_buttons_and_build.tcl](FPGA/VivadoProjects/add_pl_buttons_and_build.tcl),
then import `FPGA/VivadoProjects/cartpole_zybo_pl_buttons.xsa` and refresh
`cartpole_zybo_secloc2026`. Old bit + new firmware still boots; BTN0 does
nothing until that XSA is imported.

---

## STM32

STM remains a supported chip (`#define STM`, `CHIP = "STM"`). On-chip control is
PID. The USER button (`KEY_5` / `BUTTON_1`) toggles it the same way as PC `u`.
Track calibration and hanging capture are **not** wired to STM extra buttons
(`BUTTON_2` / `BUTTON_3` are unused); use the PC (`K`, `b`).

PID does not swing up. Do not arm with the pole far from vertical or the cart
will hit the ends. The reset button reloads flash.

The old factory-firmware PC module, board mode buttons, and
“calibrate automatically on first enable” behavior do **not** apply to this
firmware.

---

## FPGA and Vitis rebuild

You do **not** need this for daily show use. Prebuilt bitstream:
[FPGA/bitstreams/cartpole_short_pole_secloc.bit](FPGA/bitstreams/cartpole_short_pole_secloc.bit).
Platform: `Firmware/VitisProjects/cartpole_zybo_secloc2026`.

Tools: **Vivado / Vitis 2020.1** (`source /tools/Xilinx/Vivado/2020.1/settings64.sh`).

Recreate the Zybo block design from
[FPGA/VivadoProjects](FPGA/VivadoProjects):

```tcl
cd FPGA/VivadoProjects
source ./CartpoleDriverZynq_AXIS_12_09_2025.tcl
```

Generate bitstream, export hardware (**Fixed**, include bitstream), import the
XSA into the `cartpole_zybo_secloc2026` Vitis platform and refresh both
standalone domains. AMP CPU1 domain:

```console
xsct Firmware/Scripts/setup_rpgd_amp_platform.tcl
```

Production ELFs (CPU1 blob linked into CPU0):

```console
RPGD_AMP_PRODUCTION=1 Firmware/Scripts/build_rpgd_amp_elfs.sh
```

Related TCL: `add_pl_buttons_and_build.tcl`, `set_pmodad1_timing_and_build.tcl`,
`swap_nn_and_build.tcl`, Zedboard `CartpoleDriverZynq_AXIS_Zedboard.tcl`.

To snapshot a live Vivado project:

```tcl
write_project_tcl -force ./zybo_new.tcl
```

SecLoc (Zedboard / research, not the Development show default):
[Docs/SecLoc_Experiment_Platform.md](Docs/SecLoc_Experiment_Platform.md),
[FPGA/CustomIPs/README_secloc_chain.md](FPGA/CustomIPs/README_secloc_chain.md).
`USE_SECLOC` / `USE_CHIP_SECLOC` stay **False** on Development Zybo.

There is no legacy top-level FPGA folder and no exported Vitis zip in this tree.
Recreate from `FPGA/VivadoProjects` as above.

---

## Known issues

* `sys.stdin` / `KBHit` can hang the **PyCharm debugger**. Run `control.py` from
  a real terminal.
* FTDI UART is 16 ms latency until the driver (Linux) or the Windows driver
  setting forces 1 ms. MacOS: no known fix.
* Vitis and `program_flash` cannot share JTAG. Close Vitis first.
* Flashing QSPI with JP5 on QSPI stalls (`BOOT_MODE` is then 1). Power off, JP5
  = JTAG, power on, flash, then move JP5 back.
* `git submodule update --remote` can move CartPoleSimulation / toolkits off the
  pinned commits this repo expects. Toolkit `--remote` follows `cartpole_master`,
  not toolkit `main`/`master`.
