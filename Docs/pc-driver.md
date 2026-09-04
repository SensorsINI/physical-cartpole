# PC driver

`Driver/control.py` is the host program for the physical robot: UART I/O,
optional Python controllers, CSV logging, live plots, and experiment
protocols. Board-level operation (switches, buttons, safety) is in
[operating.md](operating.md).

Run it from an **interactive terminal**. PyCharm’s debugger can hang on
`sys.stdin` (see [Known issues](../README.md#known-issues)).

```bash
# after the cpp() alias from the README
python Driver/control.py
```

On connect the driver opens the Digilent FTDI **interface 1** (usually
`/dev/ttyUSB1`) at **230400** baud, sets the FTDI latency timer to 1 ms on
Linux, pushes control config from `Driver/globals.py`, and starts streaming
state. It does **not** arm the motor.

Press **`h`** or **`?`** at any time for the key list. Esc starts a clean
shutdown (motor 0, port closed).

## Terminal status

The display refreshes once per second by default (`PRINT_PERIOD_MS`, env
`CPP_PRINT_PERIOD_MS`). It is not in the control path.

| Line | Meaning |
|---|---|
| `CONTROLLER` | `Idle`, `Firmware` (on-chip `u`), or the PC `CONTROLLER_NAME` and period |
| `MEASUREMENT` | Selected experiment protocol (even when idle) |
| `STATE` | Angle (rad and raw ADC), position (cm and encoder), target, `Q`, PWM command, invalid-step / freeze flags |
| Timing / UART | Loop times, latency violations, serial wait; `skipped` means a STATE wait > 1.5 periods |

Angle near 0 with the pole upright, and near ±π hanging, means zero-angle
calibration is sane. `command` stays 0 until you arm.

## Keyboard

| Key | Action |
|---|---|
| `h` / `?` | Print this help (plus controller-specific keys, e.g. PID) |
| `u` | On-chip control on/off (same as **BTN4**) |
| `k` | PC control on/off (`CONTROLLER_NAME` in `globals.py`) |
| `K` | Track-center calibration (same as **BTN5**) |
| `b` | Capture hanging from ~1000 streamed samples and store it with the current angle circle in QSPI |
| `=` / `-` | Fine-tune angle deviation |
| `[` / `]` | Decrease / increase target position — **ignored** while the JB slider owns the target |
| `;` | Flip target equilibrium (±1) — **ignored** with the slider interface |
| `D` | Dance mode on/off (square or sine target; see below) |
| `m` | Cycle experiment protocol |
| `n` | Start / stop the selected PC protocol |
| `N` | Run the on-chip hardware experiment (IROS-style script) |
| `l` | Start / stop CSV recording |
| `L` | Time-limited CSV (`TIME_LIMITED_RECORDING_LENGTH` steps) |
| `6` / `7` / `8` | Live plotter: send / save / reset |
| `j` | Cycle joystick: off → speed → position → off |
| `9` / `0` | Increase / decrease artificial extra latency (debug) |
| Esc | Clean shutdown |

`k` and `u` are exclusive. Starting `k` sends `pc_control_mode(True)` and
turns firmware control off. Starting `u` does the reverse and, if
`CONTROLLER_NAME == 'neural-imitator'` with the IROS short-pole profile and
the show mux off, restores the chip 1 kHz / N=10 timing.

## Choosing a PC controller

`CONTROLLER_NAME` in `Driver/globals.py` is **only** used when you press
**`k`**. On-chip controllers stay on SW0–SW3.

| `CONTROLLER_NAME` | What it is | Typical period |
|---|---|---|
| `neural-imitator` | Supervised network (TF or generated C). Network path in `config_controllers.yml` | 10 ms (5 ms if `IROS_SHORT_POLE_PROFILE`) |
| `mpc` | MPC; optimizer from `OPTIMIZER_NAME` (`rpgd-c` is the usual C RPGD) | 20 ms |
| `pid` | Nested angle + position PID; live gain keys | 5 ms |
| `lqr` | Linear quadratic regulator | 8 ms |
| `mppi-cartpole`, `do-mpc`, … | Other Control Toolkit controllers | see `config_controllers.yml` |
| `secloc*` | SecLoc gate around an inner controller — keep `USE_SECLOC = False` on Development Zybo | — |

Related flags in the same file:

| Flag | Role |
|---|---|
| `CHIP` / `ZYNQ_BOARD` | Must match firmware (`ZYNQ` + `ZYBO_Z720` on Development) |
| `SHOW_SWITCH_MUX` | `True`: chip ignores PC period and derivative N |
| `USE_EXTERNAL_INTERFACE` | `True`: slider owns `target_position` |
| `OPTIMIZER_NAME` | Optimizer for `CONTROLLER_NAME == 'mpc'` |
| `MOTOR` / `MOTOR_CORRECTION` | Robot instance and Q → PWM map |
| `CORRECT_MOTOR_DYNAMICS` | Apply `MOTOR_CORRECTION` (off for `pid`) |
| `CONTROL_CPU_AFFINITY` | Pin compute (`"2"` for TF; `""` for parallel `rpgd-c`) |
| `IROS_SHORT_POLE_PROFILE` | Must match firmware; short-pole 1 kHz neural timing |
| `USE_SECLOC` / `USE_CHIP_SECLOC` | Leave `False` on Development |

Controller hyperparameters live in
[`Driver/CartPoleSimulation/Control_Toolkit_ASF/config_controllers.yml`](../Driver/CartPoleSimulation/Control_Toolkit_ASF/config_controllers.yml).
A preserved swing-up LSTM bundle (PC TF + on-chip C) is documented under
[examples/models/adaptive-quant-lstm-2025-06-01](../examples/models/adaptive-quant-lstm-2025-06-01/README.md).

### PID live keys

With `CONTROLLER_NAME = 'pid'`, `h` also prints gain keys:

| Keys | Gain |
|---|---|
| `1` / `2` | Angle Kp down / up |
| `q` / `w` | Angle Ki |
| `a` / `s` | Angle Kd |
| `3` / `4` | Position Kp |
| `e` / `r` | Position Ki |
| `d` / `f` | Position Kd |
| `p` | Print gains |
| `S` | Save gains to JSON |

Do not arm PID with the pole hanging.

## Split control loop

`control.py` runs two threads:

* **IO thread** (`LOOP_CPU_AFFINITY`, default core `"3"`) — UART polling, keyboard,
  gate, actuation, CSV trigger. Period: `POLLING_PERIOD_MS`.
* **Main thread** (`CONTROL_CPU_AFFINITY`) — controller `compute_step`. Result
  applied after `CONTROLLER_APPLY_WINDOW_MS` (often equal to the period).

Pin the main thread for single-threaded TensorFlow optimizers (`rpgd`, neural
imitator): `CONTROL_CPU_AFFINITY = "2"`. Clear it (`""`) for parallel `rpgd-c`
so OpenMP can use other cores.

## CSV logging

**`l`** writes to `Driver/ExperimentRecordings/` (override with
`PATH_TO_EXPERIMENT_RECORDINGS`). File names start with `CPP`.

Core columns (see [main_logging_manager.py](../Driver/DriverFunctions/main_logging_manager.py)):

| Column | Content |
|---|---|
| `time`, `time_chip`, `deltaTimeMs` | Host and chip clocks |
| `angle_raw`, `angle`, `angleD` | Pole state (rad / ADC) |
| `position_raw`, `position`, `positionD` | Cart state |
| `target_position`, `target_equilibrium` | Setpoints |
| `Q`, `actualMotorSave` | Normalized command and PWM counts |
| `latency`, `controller_steptime` | Timing diagnostics |
| `measurement` | Active experiment protocol name |

Controller-specific columns are appended when the active PC controller exports
`controller_data_for_csv`. Recording while on-chip control is active (`u`) is
supported — the usual way to log a show-mux run.

**`L`** stops after `TIME_LIMITED_RECORDING_LENGTH` steps (default 1000).

Experiment protocols often start and stop recordings themselves.

## Live plotter

The driver is the sender. Start the receiver first, in another terminal:

```bash
python "Driver/CartPoleSimulation/SI_Toolkit_ASF/Run/Run LivePlotter.py"
```

Default listen address is `0.0.0.0:6000`. The driver sends to
`DEFAULT_ADDRESS` (`localhost`, 6000) unless
`LIVE_PLOTTER_USE_REMOTE_SERVER` is set.

Then in `control.py`: **`6`** starts streaming, **`7`** asks the server to
save, **`8`** resets the traces. Series include angle, position, `Q`, targets,
and derivatives.

## Experiment protocols

**`m`** cycles the list; **`n`** starts or stops the one currently named on the
`MEASUREMENT` line. Protocols are state machines in
[`Driver/DriverFunctions/ExperimentProtocols`](../Driver/DriverFunctions/ExperimentProtocols).
Several drive the motor **open-loop** — take the pole off for identification
runs.

| Protocol (as printed) | What it does |
|---|---|
| `iros24-ex1` | Scripted swing-up, then cart targets +9 cm and −9 cm (PC controller or, if configured, firmware). Default selection at startup. |
| `step-response` | Bidirectional Q steps for the motor map. After a run, follow [MotorAndCartFriction](../Driver/DataAnalysis/MotorAndCartFriction/README.md) and paste `MOTOR_CORRECTION` into `globals.py` **and** `parameters.c`. |
| `emf-identification` | Open-loop shuttle to identify force vs velocity. **Pole off, no added mass.** |
| `pulse-identification` | Short known PWM pulses at target speeds. **Pole off.** |
| `swing-up` | Many swing-ups with random starts; records each trial. |
| `follow-a-random-target` | Enables PC control and steps a smoothed random cart target. |

**`N`** runs the **on-chip** twin of the IROS script (hanging reset, swing-up,
two targets, ~16 s, then motor 0). It turns PC control off first.

### Which protocol to use

| Goal | Protocol | Pole on cart? | Notes |
|---|---|---|---|
| Motor map for control | `step-response` | Yes (or off for bare cart mass ID) | Paste result into `globals.py` + `parameters.c` |
| Force vs velocity (EMF) | `emf-identification` | **Off**, no added mass | Long shuttle; decorrelates power and speed |
| Absolute force gain | `pulse-identification` | **Off** | Short pulses at known speeds |
| Batch swing-up data | `swing-up` | Yes | Needs a controller that can swing up |
| IROS-style demo script | `iros24-ex1` | Yes | PC **`k`** or configure firmware path |
| Random cart targets | `follow-a-random-target` | Yes | Enables PC **`k`** automatically |

## UART troubleshooting

| Issue | Check |
|---|---|
| No connection | Port: Digilent **interface 1** (`SERIAL_PORT_NUMBER = 1`, often `/dev/ttyUSB1`). Close Vitis and other serial monitors. |
| Laggy PC control | FTDI latency timer: 1 ms on Linux (driver sets it); Windows: [SettingLatencyTimerOnWindows.png](../Docs/SettingLatencyTimerOnWindows.png). |
| `skipped` in status line | IO thread waited > 1.5 chip STATE periods — heavy PC load or wrong `POLLING_PERIOD_MS` vs chip period. |
| Wrong hanging after connect | A valid stored QSPI calibration takes priority and the driver adopts it. Use **BTN0** or **`b`** to replace hanging; use BTN1 to replace the circle. |
| MacOS latency | No known FTDI fix (see [Known issues](../README.md#known-issues)). |

## Dance

**`D`** adds a periodic offset to `target_position` (and optionally flips
equilibrium if `DANCE_UP_AND_DOWN` is set). Parameters in `globals.py`:

* `DANCE_PATH` — `'square'` or `'sin'`
* `DANCE_AMPL` — metres (default 0.1)
* `DANCE_PERIOD_S` — default 10

Turning dance off eases the target back to the base position. Disarming
control also clears dance.

## Joystick

If pygame sees exactly one stick at startup, **`j`** cycles
`not active` → `speed` (axis 0 is `Q`) → `position` (P on stick vs cart)
→ off. Deadzone and position gain: `JOYSTICK_DEADZONE`,
`JOYSTICK_POSITION_KP`.

Unplugging and replugging a stick during a session can disturb the next
track calibration (motor left running). Unplug before **`K`** / **BTN5**, or
restart `control.py`.

## Simulation and analysis (no robot required)

The [CartPoleSimulation](https://github.com/SensorsINI/CartPoleSimulation)
submodule is the same plant model used to train imitators and to develop
MPC.

```bash
python Driver/CartPoleSimulation/run_cartpole_gui.py
```

Data generation, network training, and Brunton tests are described in
[`Driver/CartPoleSimulation/README.md`](../Driver/CartPoleSimulation/README.md).

Offline helpers in this repo:

* Motor / friction fits:
  [Driver/DataAnalysis/MotorAndCartFriction](../Driver/DataAnalysis/MotorAndCartFriction/README.md)
* Angle circle:
  [Driver/DataAnalysis/AngleUpDown](../Driver/DataAnalysis/AngleUpDown)
* Recording GUI:
  [Driver/CartPoleSimulation/others/DataViz](../Driver/CartPoleSimulation/others/DataViz/README.md)

## Optional paths

These stay off on a normal Development Zybo (`False` in `globals.py`):

* **`USE_DVS_STATE_ESTIMATION`** — overwrite angle/position from a ZMQ DVS
  helper (`Driver/DriverFunctions/DVS`).
* **`USE_EKF`** — fuse cart/pole rates with an EKF; optional calibration run.
* **`USE_SECLOC` / `USE_CHIP_SECLOC`** — event gate around a controller.
  Research / Zedboard; see
  [Docs/SecLoc_Experiment_Platform.md](SecLoc_Experiment_Platform.md).
* On-chip timing harness (motor off):
  [Firmware/Src/Embedded_Controller/TIMING_TEST_README.md](../Firmware/Src/Embedded_Controller/TIMING_TEST_README.md).
