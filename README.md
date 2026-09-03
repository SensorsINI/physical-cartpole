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
| This README | Overview, install, quick start, known issues |
| [Docs/architecture.md](Docs/architecture.md) | Repo layout, runtime paths, config sync, sim-to-real pointers |
| [Docs/hardware.md](Docs/hardware.md) | BOM, PMODs, motors, belt tension, slider hardware |
| [Docs/calibration.md](Docs/calibration.md) | Track center, hanging, motor map, angle circle |
| [Docs/firmware-and-flash.md](Docs/firmware-and-flash.md) | QSPI, STM32, FPGA/Vitis rebuild |
| [Docs/operating.md](Docs/operating.md) | Day-to-day use: modes, buttons, switches, LEDs, safety |
| [Docs/pc-driver.md](Docs/pc-driver.md) | `control.py`: keys, controllers, logging, protocols |
| [Firmware/README.md](Firmware/README.md) | Build scripts and AMP production flow |
| [tools/slider_pmod/README.md](tools/slider_pmod/README.md) | JB slider map and UART check |
| [Driver/CartPoleSimulation/README.md](Driver/CartPoleSimulation/README.md) | Simulator GUI, data generation, training pipeline |
| [examples/models](examples/models/README.md) | On-chip / PC model deployment index |
| [Docs/SecLoc_Experiment_Platform.md](Docs/SecLoc_Experiment_Platform.md) | SecLoc / Zedboard research (not Development Zybo) |

## Contents

* [Quick start (Zybo)](#quick-start-zybo)
* [Set up and installation](#set-up-and-installation)
* [Using the robot](#using-the-robot)
* [Known issues](#known-issues)

## Quick start (Zybo)

Boot from QSPI (JP5 on the two center pins labeled QSPI). The show image must
already be in flash — how to program it is in
[Docs/firmware-and-flash.md](Docs/firmware-and-flash.md).

1. Power on with **SW0–SW3 all off**.
2. Hang the pole straight down. Press **BTN0**. That disarms control, zeros PWM,
   captures `ANGLE_HANGING`, and leaves control off. RGB flashes white on success.
3. Arm with **BTN4**, or `python Driver/control.py` then **`u`**.
4. Turn on **exactly one** switch:

   | Switch | On-chip controller |
   |---|---|
   | SW0 | AMP RPGD (CPU1) |
   | SW1 | Dense-8 C (PS) — `Dense-7IN-32H1-32H2-1OUT-8` in `Firmware/Src/General/NC_C/` |
   | SW2 | LSTM C (PS) |
   | SW3 | Short-pole PL neural imitator |

   All off, or more than one on: Q = 0, motor stopped.
5. **BTN0** again: disarm + new hanging capture. **BTN5** or PC **`K`**: track
   center only (does not change hanging).

What the switches, RGB LEDs, and slider do after this first boot, and how to
log or take over from the PC:
[Docs/operating.md](Docs/operating.md), [Docs/pc-driver.md](Docs/pc-driver.md).

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

Then power off, JP5 = QSPI, power on. The show image is AMP CPU0
(`CartPoleFirmware_rpgd_amp_cpu0.elf`); FSBL loads that one app. See
[Docs/firmware-and-flash.md](Docs/firmware-and-flash.md).
The script uses reliable 1-bit FSBL reads on the Zybo's 16 MiB S25FL128S;
otherwise a cleared QUAD-enable bit makes linear reads return `0x888888xx`
and the FSBL stops before configuring the FPGA. It erases only the boot-image
range, preserving the hanging-calibration slot at `0xFD0000`.

microSD backup (FAT card labeled `SD_Zybo` mounted on the PC):

```console
Firmware/Scripts/install_show_sd.sh
```

Safely eject and insert the card, set JP5 = SD, then power-cycle. The card
contains the same standalone `BOOT.BIN` as QSPI.

### Hardware and calibration

* Mechanical setup, PMODs, motors: [Docs/hardware.md](Docs/hardware.md)
* Track center, hanging, motor map: [Docs/calibration.md](Docs/calibration.md)
* STM32 firmware path: [Docs/firmware-and-flash.md](Docs/firmware-and-flash.md#stm32)

---

## Using the robot

Hardware and flashing are linked above. Daily operation — show vs PC, what SW0–SW3
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

Simulation and training (no robot): [Driver/CartPoleSimulation/README.md](Driver/CartPoleSimulation/README.md).

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
