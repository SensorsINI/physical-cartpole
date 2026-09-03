# Operating the physical cartpole

This page is the day-to-day guide: what the robot can do, which path to use
(standalone show, PC as logger, PC as controller, STM32), and how the board
controls, targets, and safety interact.

Hardware assembly, flashing, and first-time calibration stay in the
[top-level README](../README.md). Keyboard bindings, logging, and experiment
protocols are in [pc-driver.md](pc-driver.md).

Keep the track clear. Have a hand on the 12 V motor supply.

## What you can do

| Goal | Path |
|---|---|
| Demo / show, no laptop after boot | QSPI image, **BTN0 → BTN4**, exactly one of **SW0–SW3**, JB slider for the cart target |
| Watch an on-chip controller and log CSV | Same as the show, plus `python Driver/control.py` and **`u`** / **`l`** |
| Run a controller on the PC | Set `CONTROLLER_NAME` in `Driver/globals.py`, then **`k`** |
| Identify the motor / cart | Experiment protocols **`m`** / **`n`** (pole off for force ID) |
| Repeat the IROS-style swing-up + two targets | PC protocol `iros24-ex1`, or on-chip **`N`** |
| Tune a PID live | `CONTROLLER_NAME = 'pid'`, **`k`**, then the PID gain keys |
| Drive the cart by hand | Joystick **`j`**, or an experiment / dance that sets `Q` |
| Simulate without the robot | `python Driver/CartPoleSimulation/run_cartpole_gui.py` |

On Zybo the default lab path is **on-chip control** (switches + **BTN4** / **`u`**).
The PC `k` path is for development, comparison, and protocols that need a Python
controller.

## Pre-flight checklist

Before arming with the pole on the track:

1. Track clear; 12 V motor supply ready to cut.
2. **JP5** = QSPI for standalone boot, or JTAG if developing from RAM.
3. **SW0–SW3** all off until after arm.
4. Pole hanging; **BTN0** (or **`b`**) captured hanging — RGB white flash, not alternating red.
5. **BTN5** or **`K`** once per power cycle if the encoder zero may be stale.
6. Exactly **one** switch on before expecting motion.
7. JB slider near center if you want target ≈ 0.

## Safety

Firmware keeps PWM at 0 until you arm (**BTN4**, PC **`u`**, or PC **`k`**).
Disarming calls `Motor_DisableOutput()`.

While armed:

* **Stall cut** — a large command with almost no encoder travel for 100 ms
  latches the command to 0. Re-arm to clear it.
* **Track ends** — the chip zeros a command that would drive into the last
  ~300 encoder counts before either stop.
* **Invalid / bouncing DIP** — more than one of SW0–SW3, or a bounce:
  `Q = 0` for that tick.
* **PWM clip** — command is limited to 95% of the PWM period.
* **PC safety switch** — if the cart sits past ~95% of the encoder half-range
  for more than 10 IO cycles, the driver drops PC control and writes motor 0.

None of this replaces watching the cart. Cut 12 V if it runs away.

**Do not arm PID** (STM onboard, or PC `pid`) with the pole far from vertical.
PID does not swing up; the cart will hit the ends.

## Operating modes

```
                    ┌─ SW0  AMP RPGD (CPU1)     ~20 ms
  arm (BTN4 / u) ───┼─ SW1  Dense-8 C (PS)      10 ms
                    ├─ SW2  LSTM C (PS)         10 ms
                    └─ SW3  short-pole PL NN    1 ms
                              │
 JB slider ──► target_position ┤  (USE_EXTERNAL_INTERFACE)
                              ▼
                            motor

  PC k ──► Python controller ──► UART Q ──► motor
           (disarms on-chip u; slider still owns the target)
```

PC **`k`** and on-chip **`u`** are mutually exclusive: enabling one disables the
other. The slider, when compiled in, overwrites `target_position` every cycle
on both paths.

| Mode | Motor command comes from | Typical use |
|---|---|---|
| Idle (all SW off, not armed) | 0 | Boot, hanging capture, talking through the demo |
| On-chip show | SW0–SW3 controller | Lab default |
| PC controller | `CONTROLLER_NAME` in `globals.py` | Research, PID, rpgd-c, neural-imitator on the host |
| Experiment protocol (`n`) | Protocol state machine (open-loop `Q` and/or targets) | Motor ID, swing-up batches, IROS script |
| Joystick (`j`) | Stick (overrides `Q` while active) | Manual cart motion |
| STM onboard PID | Chip PID | STM robot without Zybo |

## Typical sessions

### Standalone show (no PC)

1. JP5 = **QSPI**, 12 V connected, track empty, pole hanging.
2. Power on with **SW0–SW3 all off**. RGB may be cyan (target at center).
3. **BTN0** — disarms, zeros PWM, captures `ANGLE_HANGING`. RGB flashes **white**
   on success. Alternating **red** means the pot dead zone is too close to
   vertical: rotate the clamp toward horizontal and press BTN0 again.
4. Optional: **BTN5** once after power-up to re-center the track (does not
   change hanging).
5. **BTN4** to arm. Motor is still 0 until a valid one-hot switch is on.
6. Turn on **exactly one** of SW0–SW3. Move the JB slider to set the cart
   target (±12 cm).
7. **BTN0** again to stop and recapture hanging. All switches off also stops
   the controller (`Q = 0`).

Hardcode hanging and the motor map in `parameters.c` if you do not want to
press BTN0 after every boot. See the README section
[Standalone / QSPI](../README.md#standalone--qspi-no-pc-at-power-up).

### PC as monitor while the chip controls

Use this to see state, record CSV, or run the live plotter without moving
compute off the Zynq.

```bash
python Driver/control.py
```

Leave `SHOW_SWITCH_MUX = True` (Development default). The chip ignores the
period / derivative-N the driver would otherwise push.

1. Same hanging / center / arm sequence as the show, or **`b`** / **`K`** / **`u`**
   from the keyboard (same roles as BTN0 / BTN5 / BTN4).
2. Terminal **CONTROLLER** line should read `Firmware` after **`u`**.
3. **`l`** starts a CSV in `Driver/ExperimentRecordings/`. **`L`** is the same
   with a fixed length (`TIME_LIMITED_RECORDING_LENGTH` steps).
4. Start the live-plotter server, then **`6`**. Details:
   [pc-driver.md](pc-driver.md#live-plotter).

### PC as controller

1. In `Driver/globals.py` set `CONTROLLER_NAME` (`neural-imitator`, `mpc`,
   `pid`, `lqr`, …). For `mpc`, `OPTIMIZER_NAME` selects the optimizer
   (`rpgd-c` is the usual C implementation).
2. `python Driver/control.py` from an interactive terminal.
3. Hang, **`b`** or **BTN0**, **`K`** or **BTN5** if the encoder zero is stale.
4. **`k`** arms the Python controller. On-chip control turns off.
5. With `USE_EXTERNAL_INTERFACE = True`, **`[` / `]` / `;`** do not move the
   target — the slider does. Set the flag `False` (and rebuild firmware
   without `USE_EXTERNAL_INTERFACE`) if you need keyboard targets.

`CONTROLLER_NAME` is **only** the `k` path. Switches still select the on-chip
controller for `u`.

### STM32

`CHIP = "STM"` in `globals.py`, `#define STM` in `hardware_bridge.h`.
On-chip control is PID only. The USER button toggles it the same way as **`u`**.
Track center and hanging capture are not on extra STM buttons: use **`K`** and
**`b`**. Do not arm until the pole is near upright.

## Board controls (Zybo)

### Switches SW0–SW3 (show mux)

One-hot. The reading must repeat for two consecutive polls before the firmware
commits it (debounce). All off, more than one on, or an unstable bounce:
idle, `Q = 0`.

| Switch | On-chip controller | Period (typical) | Notes |
|---|---|---|---|
| SW0 | AMP RPGD on CPU1 | 20 ms | Small-batch gradient NMPC. CPU0 runs the mux and motor loop. |
| SW1 | Dense-8 C on the PS | 10 ms | Feedforward neural imitator (`OnChipController_neural_controller_C`; weights in `Firmware/Src/General/NC_C/`, net name `Dense-7IN-32H1-32H2-1OUT-8`). |
| SW2 | LSTM C on the PS | 10 ms | Recurrent imitator; adapts from recent trajectory ([A-NC](https://proceedings.mlr.press/v283/paluch25a.html)). |
| SW3 | Short-pole neural imitator in the PL | 1 ms | FPGA network; PWM period 2500. |

Each selection also loads that controller’s plant profile (PWM period, motor
map, derivative span, encoder range). Show-mux profiles currently share the
LSTM/RPGD motor map `{0.5733488, 0.0257380, 0.0258429}`.

### On-chip controller implementation map

| Switch | Firmware id | Period | PWM period | Primary source |
|---|---|---|---|---|
| SW0 | `OnChipController_RPGD` | 20 ms | 10000 | CPU1 RPGD worker; [controller_profiles.c](../Firmware/Src/CartPoleFirmware/controller_profiles.c) |
| SW1 | `OnChipController_neural_controller_C` | 10 ms | 10000 | [neural_controller_C.c](../Firmware/Src/General/neural_controller_C.c), weights [NC_C/](../Firmware/Src/General/NC_C/) |
| SW2 | `OnChipController_neural_controller_LSTM_C` | 10 ms | 10000 | [neural_controller_LSTM_C.c](../Firmware/Src/General/neural_controller_LSTM_C.c), weights [NC_LSTM/](../Firmware/Src/General/NC_LSTM/) |
| SW3 | `OnChipController_NeuralImitator` | 1 ms | 2500 | PL + [neural_imitator.c](../Firmware/Src/Zynq/neural_imitator.c); bitstream [cartpole_short_pole_secloc.bit](../FPGA/bitstreams/cartpole_short_pole_secloc.bit) |

Model bundles and PC paths: [examples/models/README.md](../examples/models/README.md).

The LEDs above the switches mirror the DIP state.

### Buttons

| Button | Firmware name | Action |
|---|---|---|
| **BTN0** (PL) | `BUTTON_3` | Disarm on-chip and PC control, zero PWM, capture hanging (50 wrap-aware filtered ADC samples). Aborts if the pole is moving, the dead zone is in the sample, or calibration is running. Writes QSPI at `0xFD0000` / `0xFFF000` when control stays off. |
| **BTN4** (PS) | `BUTTON_1` | Arm / disarm on-chip control. Same as PC **`u`**. |
| **BTN5** (PS) | `BUTTON_2` | Track-center calibration. Same as PC **`K`**. Never changes hanging. |
| **BTN1–BTN3** | PL, unassigned by default | Present in the bitstream; `Button_SetAction(PL_BTN_n, …)` can bind them without a new FPGA build. |

After reset, hanging is the compile-time value in `parameters.c` (QSPI is not
loaded at boot). First PC connect applies `globals.py` once, unless BTN0 already
ran this boot.

### RGB LEDs

| Pattern | Meaning |
|---|---|
| Both white, ~300 ms | BTN0 hanging capture succeeded |
| Alternating red | Stored hanging puts the pot dead zone within ~20° of vertical |
| Both cyan | Cart target is 0 (center) |
| Green (one diode) | Target > 0 |
| Blue (one diode) | Target < 0 |

Cyan is a centered target, not a fault. The PS LED blinks faster (~100 ms)
while on-chip control is armed and slower (~500 ms) when it is not.

### JB slider and optional equilibrium switch

With `USE_EXTERNAL_INTERFACE` (Development default):

* Pmod slider on **JB** sets `target_position` every cycle to
  ±`SliderTargetHalfLength` (**0.12 m**), electrical mid = 0.
* The driver displays the chip target and does not send
  `CMD_SET_TARGET_POSITION`.
* If the bitstream has the equilibrium GPIO, an external 2-position switch
  sets `target_equilibrium` to ±1. Otherwise equilibrium stays at the last
  value (upright `+1` after a normal arm).

Slider calibration: [tools/slider_pmod/README.md](../tools/slider_pmod/README.md).
Close `control.py` before a UART slider check — it holds the port.

## Targets and equilibria

Controllers (on-chip and PC) track two setpoints:

* **`target_position`** — where the cart should sit, in metres, 0 = track
  center. Slider range is ±0.12 m; the mechanical half-track is ~0.20 m.
* **`target_equilibrium`** — `+1` upright, `−1` hanging. Swing-up is the
  transition from `−1` to `+1`.

Dance mode (`D` in the PC driver) adds a square or sine offset to the position
target. Keyboard **`;`** flips equilibrium only when the slider does not own
the target.

## After every power cycle

Not stored across reset (unless you copy numbers into `parameters.c` /
`globals.py`):

* Track center — **BTN5** / **`K`**
* Session hanging — **BTN0** / **`b`**, or the compile-time default
* Motor type in RAM — calibration detects encoder sign; each side reloads its
  file default after reset. Starting the Python driver overwrites firmware RAM
  with `MOTOR` from `globals.py`

Angle circle (`ANGLE_360_DEG_IN_ADC_UNITS`) and the motor map
(`MOTOR_CORRECTION`) are compile-time / file constants. Recalibrate those only
when the analog chain or the motor changes. See
[calibration.md](calibration.md).

## Glossary

| Term | Meaning |
|---|---|
| **Q** | Normalized motor command in \[-1, 1\] before PWM scaling |
| **ANGLE_HANGING** | Raw ADC value when the pole hangs straight down; defines zero angle |
| **target_equilibrium** | `+1` upright, `−1` hanging setpoint for swing-up controllers |
| **target_position** | Cart position setpoint (m); JB slider owns it when `USE_EXTERNAL_INTERFACE` is on |
| **show mux** | SW0–SW3 one-hot selector for on-chip controllers |
| **MOTOR_CORRECTION** | `(gain, friction+, friction−)` map from Q to PWM counts |
| **Arm** | BTN4, **`u`**, or **`k`** — enables control path; motor still needs valid switch for on-chip |

## If something goes wrong

| Symptom | What to try |
|---|---|
| Cart does not move after arm | Switches not one-hot; motor output still latched off; 12 V unplugged; stall latched (disarm and re-arm) |
| RGB alternating red | Dead zone near vertical — rotate the pot clamp, BTN0 again |
| Angle jumps or wraps near upright | Dead zone is not at horizontal; see Calibration |
| BTN0 does nothing | Old bitstream without `PL_BUTTONS_GPIO`; JP5 / image mismatch |
| PC `k` does nothing | Run from a real terminal, not the PyCharm debugger; check UART (`/dev/ttyUSB1`, 230400) |
| Slider ignored / keyboard target ignored | `USE_EXTERNAL_INTERFACE` on both sides, or off on both |
| Vitis / `program_flash` cannot see the board | Close the other JTAG user; JP5 must be JTAG to program |

More in [Known issues](../README.md#known-issues).
