# SecLoc on Zedboard — experiment platform guide

For the **SecLoc2026** branch. Assumes a flashed Zedboard.
```bash
git checkout Secloc2026
git submodule update --init --recursive
```

---

## Quick start

```bash
conda activate cpp
pip install -r requirements.txt   # once
export PYTHONPATH=$PWD:$PWD/Driver/CartPoleSimulation
python Driver/control.py
```

1. `Shift+K` — calibrate track centre.
2. **`u`** — chip SecLoc (gate + NN in PL; default experiment path).
3. **`k`** — PC SecLoc (gate in Python, inner controller on CPU).
4. **`l`** — CSV recording → `Driver/ExperimentRecordings/`.
5. **`5`** — `CMD_GET_SECLOC_INFO` (backend, PL faults, shadow mismatches).
6. **`h`** — key list.

Status tag: `secloc@chip:PL` = last step from PL backend; `PL-FAULT` = PL backend selected but transaction failed → **zero force** (no SW fallback).

### Execution domains (used throughout this doc)

| Tag | Domain | Runs |
|---|---|---|
| **PC** | Linux host, `Driver/control.py` | Python gate + inner controller (`k` mode), logging, config push |
| **PS** | Zynq ARM core, CartPoleFirmware | Sensor loop, derivatives, motor command, SW gate + C inner (`u` mode, `SECLOC_BACKEND_SW`) |
| **PL** | Zynq fabric (bitstream) | Angle filter, `secloc_shell`→`secloc_gate`→`nn_marshal`→NN chain (`u` mode, `SECLOC_BACKEND_PL*`) |

The PS is always in the loop (sensing/actuation) regardless of who computes Q. A parameter can affect several domains at once — every table below has an **Affects** column.

### First files to edit

| Want to change | File | Affects | Takes effect |
|---|---|---|---|
| Gate params (`log_base`, `ref_period_ticks`, dead bands) | `Driver/CartPoleSimulation/Control_Toolkit_ASF/config_secloc.yml` | **PC + PS + PL** (one profile, pushed everywhere) | On save |
| PC controller / SecLoc on-off / loop timing | `Driver/globals.py` (`CONTROLLER_NAME`, `USE_SECLOC`, `POLLING_PERIOD_MS`) | PC (timing values also pushed to PS) | Restart `control.py` |
| Controller hyper-parameters (NN name, LQR Q/R, MPC horizon…) | `Driver/CartPoleSimulation/Control_Toolkit_ASF/config_controllers.yml` | PC only | Restart (LQR `Q`/`R`: on save) |
| Chip inner controller / backend / standalone gate defaults | `Firmware/Src/General/secloc_defaults.h` | PS (backend choice decides PS-vs-PL execution) | Firmware rebuild |
| On-chip boot controller (id 0–6) | `Firmware/Src/CartPoleFirmware/control.c` (`ON_CHIP_BOOT_CONTROLLER`) | PS | Firmware rebuild |

---

## Tutorial — a full experiment session

The canonical loop for **every** experiment below:

```text
edit knob → python Driver/control.py → Shift+K (calibrate, once per power-up)
→ u or k (start control) → l (start recording) → run 30–60 s, optionally disturb
→ l (stop recording) → ESC → analyze + plot the CSV
```

### Step 1 — baseline run (all defaults)

Defaults are a working profile: chip SecLoc, PL_SHADOW backend, `log_base=1.05`, NN inner. Just:

```bash
python Driver/control.py     # Shift+K, u, l … l, ESC
```

While running, watch the terminal: rolling skip %, `secloc@chip:PL` tag, and press **`5`** — expect `pl_available=1`, `pl_faults=0`, `shadow_mismatches=0`.

### Step 2 — plot the recording

```bash
python tests/plot_secloc_experiment.py --latest      # four-panel figure → tests/output/
```

Details in [Analysis and plotting tools](#analysis-and-plotting-tools). Keep the terminal skip % from the run and check it against the replay-vs-hardware skip rates printed in the figure caption.

### Step 3 — disturb the system

A gated controller is only interesting when the state changes. During a run:

- **`]` / `[`** — step the target position (canonical disturbance; the gate's position criterion is relative to target, so each step forces events).
- **`;`** — flip target equilibrium (upright ↔ hanging → swing-up transient, heavy event burst).
- **`Shift+D`** — dance mode: target follows `DANCE_PATH` in `globals.py` (`square` or `sin`, `DANCE_AMPL`, `DANCE_PERIOD_S`) — repeatable periodic disturbance, good for comparing runs.
- **`m` then `n`** — scripted experiment protocols (`Driver/DriverFunctions/ExperimentProtocols/`) — fully reproducible target sequences.
- A gentle finger tap on the pole works too; it just isn't reproducible.

### Step 4 — change one knob and repeat

The scenario catalog below tells you which file to touch per knob and whether the rig must be restarted or the firmware rebuilt. Record one CSV per setting; the CSV header stores the gate params, controller and git revision, so runs stay attributable.

### Beyond knobs — two deeper change workflows

**Changing the gate logic** (e.g., evaluate the position condition only if the angle condition fired). The algorithm lives in one C file used by all three sites — the PS compiles it, the PL HLS gate `#include`s it, and the PC `secloc-c` controller compiles it via ctypes — plus a hand-maintained Python mirror:

1. Edit `Firmware/Src/General/secloc_logic.c` and make the identical change in `Driver/CartPoleSimulation/Control_Toolkit_ASF/Controllers/secloc_logic.py`.
2. Verify decision-for-decision C↔Python equality: `pytest tests/test_secloc_c_python_parity.py`. Optionally explore the new behavior offline with the simulated `log_base` sweep or in the simulation GUI (`secloc-c` recompiles automatically from the edited sources).
3. Rebuild the firmware (PS gate) and the PL gate IP + bitstream (`FPGA/CustomIPs/build_secloc_chain.sh`, then the Vivado build) — the PL copy is baked into the bitstream.
4. On the rig, run in `PL_SHADOW` and press `5`: `shadow_mismatches=0` confirms PS and PL run the same new logic.

**Swapping the PL neural network for another controller, e.g. an LQR in the PL.** The gate is controller-agnostic: it streams 7 raw float32 inputs (`position, positionD, angle, angleD, target_position, target_equilibrium, time`) and expects 1 float32 Q back (`secloc_stream_protocol.h`). Everything downstream of the gate is replaceable:

1. Write a new HLS IP implementing that stream contract — an LQR is a dot product over 5 of the inputs, so one small block replaces the whole `nn_marshal → controller_axis → myproject` NN tail (no normalization/quantization marshal needed).
2. Edit the block design (`FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_Zedboard.tcl`) to instantiate it after `SECLOC_GATE_0` in place of `NN_MARSHAL_0` + NN, rebuild the bitstream. `secloc_shell` and `secloc_gate` are untouched, so the PS register map and firmware keep working as-is.
3. Keep the gains consistent with the PS/PC twins (`lqr.c` `LQR_DefaultGains`, yaml `lqr:` Q/R) and set `SECLOC_DEFAULT_INNER_CONTROLLER SECLOC_INNER_LQR` (firmware rebuild) if you also want the SW backend / shadow comparisons to run the same controller.

Rule of thumb: gate logic = one shared C file, but three deployments (Python mirror, firmware, bitstream); the PL inner controller = anything honoring the 7-in/1-out stream, swapped purely on the FPGA side.

---

## Experiment scenarios

### A. Gate sensitivity sweep (`log_base`) — no restart

The core SecLoc experiment: how much can the update rate be cut before control quality degrades?

1. Keep the rig running (`u` or `k`).
2. Edit `config_secloc.yml` → `log_base: 1.00` → save. PC watcher reloads it and pushes it to PS → PL sticky registers within one poll window.
3. `l` … record ~30 s with a dance or target steps … `l`.
4. Repeat for `log_base` ∈ {1.02, 1.05, 1.10, 1.20, …}.
5. Compare: `python tests/plot_secloc_experiment.py` per recording — the figure title carries the `log_base`, panel 4 the skip level.

Same procedure works for `ref_period_ticks` (throttle) and `dead_ang`/`dead_pos` (noise floor). Affects **PC+PS+PL** simultaneously — the three gates always share one profile when the driver is connected.

**Offline first:** you can sweep `log_base` in pure simulation (PID + measurement noise, no rig or recording needed) and pick the threshold for a target skip rate before occupying the hardware:

```bash
python tests/test_secloc_log_base_sweep.py
```

### B. Always-fire baseline vs gated — no restart

Degenerate profile `log_base: 1.0`, `dead_ang: 0`, `dead_pos: 0`, `ref_period_ticks: 0` = plain controller through the identical code path (every step fires). Record baseline, then restore the gate and record again — the pair isolates the effect of event-triggering from everything else.

### C. Control placement: PC vs PS vs PL — keys + one rebuild

| Variant | How | Q computed on |
|---|---|---|
| PC gate + Python inner | `k` (with `USE_SECLOC=True`) | PC |
| Chip gate + C inner on CPU | `u` + `SECLOC_DEFAULT_BACKEND SECLOC_BACKEND_SW` (rebuild) | PS |
| Chip, full PL chain | `u` + `SECLOC_BACKEND_PL` (rebuild) | PL |
| Chip, PL with PS shadow check | `u` + `SECLOC_BACKEND_PL_SHADOW` (current default) | PL (+PS compare) |

Switching `k`↔`u` is live; switching backend is a firmware rebuild. Compare `latency` and `deltaTimeMs` CSV columns across variants — this is the platform's headline comparison (host-loop jitter vs deterministic on-chip loop vs fabric).

With PL_SHADOW, `5` should always report `shadow_mismatches=0`; a nonzero count is itself a finding (param desync or float divergence).

### D. Inner controller swap

**PC (`k` mode)** — edit `Driver/globals.py`, restart:

- `CONTROLLER_NAME = 'neural-imitator'` (default), `'mpc'` (RPGD optimizer; heavier compute → exercises the apply window), `'lqr'`, `'pid'`.
- `USE_SECLOC=True` wraps any of them with the modular gate; the gate cost is identical, only the inner changes.

**Chip (`u` mode)** — edit `Firmware/Src/General/secloc_defaults.h`, rebuild:

- `SECLOC_DEFAULT_INNER_CONTROLLER` = `SECLOC_INNER_NNC` (default) / `SECLOC_INNER_LQR` / `SECLOC_INNER_PID`. Only relevant for the SW backend; the PL backend always runs the NN baked into the bitstream.

### E. Modular vs legacy monolithic SecLoc

The branch carries the pre-modularization controllers with the gate fused into the control law (slightly different semantics: shared last-shift refresh, no target-relative angle shift):

- **PC:** `CONTROLLER_NAME='secloc-lqr'` or `'secloc-do-mpc-discrete'` with **`USE_SECLOC=False`** (gate is built in; the wrapper would double-gate). Gate params live in their `config_controllers.yml` entries (restart to apply), not in `config_secloc.yml`.
- **Chip:** `ON_CHIP_BOOT_CONTROLLER = OnChipController_SECLOC_LQR` (id 6) in `control.c`, rebuild. Fixed float gains in `secloc_lqr.c`.

Useful as a historical baseline against the modular gate (id 5 / `secloc` wrapper).

### F. Timing and sensing sensitivity

All in `Driver/globals.py`, restart to apply (pushed to PS at startup):

- `POLLING_PERIOD_MS` — loop/gate tick everywhere (PC IO thread, PS loop, gate tick size). Changing it rescales `ref_period_ticks`.
- `CONTROLLER_APPLY_WINDOW_MS` — PC split loop only: trigger→apply latency. Shrink until the inner controller misses deadlines (`controller_latency_violations` column).
- `TIMESTEPS_FOR_DERIVATIVE` — angleD/positionD smoothing on the PS; interacts with gate noise sensitivity.
- `HARDWARE_ANGLE_FILTER_*` (`OVERRIDE=True`) — PL angle filter: mode raw/median/trimmed-mean, window 1–64, trim. Determines the noise floor the gate and the dead bands see.
- `9` / `0` keys — inject artificial latency ±1 ms live, to probe latency tolerance of a gated controller.

### G. NN swap (advanced, three artifacts)

Replace the policy network everywhere it exists — the three copies must match:

| Domain | Artifact | Effort |
|---|---|---|
| PC | `config_controllers.yml` → `neural-imitator.net_name` | Restart |
| PS | regenerate `Firmware/Src/General/NC_C/` | Firmware rebuild |
| PL | `FPGA/VivadoProjects/swap_nn_and_build.sh [network_dir]` (re-synths marshal only) + new bitstream | FPGA rebuild |

### H. Simulated log_base sweep (no rig needed)

```bash
python tests/test_secloc_log_base_sweep.py            # skip % vs log_base, one curve per noise level
python tests/test_secloc_log_base_sweep.py --quant    # same with ADC/encoder-quantized sensors
```

### Knob summary

| # | Knob | Affects | Where | Effort |
|---|---|---|---|---|
| 1 | `log_base`, `ref_period_ticks`, dead bands | PC + PS + PL | `config_secloc.yml` (`secloc_defaults.h` for standalone chip) | Save file (rebuild for standalone) |
| 2 | Always-fire baseline | wherever gate runs | Degenerate gate params (scenario B) | Save file |
| 3 | Control placement `k`/`u` | PC vs PS/PL | Keyboard | Live |
| 4 | Backend (SW / PL / PL_SHADOW) | PS↔PL split | `SECLOC_DEFAULT_BACKEND` | Firmware rebuild |
| 5 | Inner controller | PC / PS | `CONTROLLER_NAME` / `SECLOC_DEFAULT_INNER_CONTROLLER` | Restart / rebuild |
| 6 | Modular vs legacy monolithic gate | PC / PS | `secloc-*` controllers / boot id 5 vs 6 | Restart / rebuild |
| 7 | Timing (polling, apply window, latency) | PC + PS | `globals.py`, keys `9`/`0` | Restart / live |
| 8 | Sensing (derivative window, PL angle filter) | PS / PL | `globals.py` | Restart |
| 9 | NN | PC / PS / PL | yaml / `NC_C/` / bitstream swap | Restart / rebuilds |
| 10 | Disturbance patterns | PC + PS (targets forwarded) | Keys `[`/`]`/`;`, dance, protocols, board switches | Live |

---

## SecLoc on this branch — what exists

Three implementations of **one gate** (`Firmware/Src/General/secloc_logic.c`, parity-checked against Python in `tests/test_secloc_c_python_parity.py`):

| Site | File / block | Role |
|---|---|---|
| PC driver | `Control_Toolkit_ASF/Controllers/secloc_gate.py` → `secloc_logic.py` | Gate + ZOH when `USE_SECLOC=True` |
| Zynq PS (SW backend) | `secloc_controller.c` | Gate + inner on CPU |
| Zynq PL | `FPGA/CustomIPs/secloc_gate_hls/` (`#include "secloc_logic.c"`) | Same gate in HLS |

**On-chip default path:** `SECLOC_Ops` → PL backend (`secloc_shell` → `secloc_gate` → `nn_marshal` → hls4ml VHDL). PS writes state via AXI-Lite to `secloc_shell`; gate params are sticky registers forwarded on config change.

```
PS  secloc_controller.c
 │  AXI-Lite (0x40410000, secloc_shell)
 ▼
secloc_shell  ──14-word req / 4-word resp──►  secloc_gate  ──7 float32 in / 1 float32 Q──►  nn_marshal  ──►  controller_axis  ──►  myproject (VHDL)
     ▲                                              │
     └──────────────── Q, status, counters ─────────┘
```

Packet layout: `FPGA/CustomIPs/secloc_stream_protocol.h` (`SECLOC_STREAM_VERSION = 2`).

Additionally the branch carries the **legacy monolithic SecLoc controllers** (gate fused into the controller, no shared gate object): `secloc-lqr` / `secloc-do-mpc-discrete` on the PC and `SECLOC_LQR_Ops` on chip — see scenario E and the [Controllers catalog](#controllers-catalog).

---

## Gate decision (precise)

Each control-loop iteration the gate may:

1. **Throttle** — if `ref_period_ticks > 1`, skip consultation until `ref_period_ticks` iterations have passed since the last **accepted update** (tick = `round(time / time_quantum_s)`, `time_quantum_s` = control-loop period, 5 ms on this rig).
2. **Evaluate** — if throttle allows, compare current shifts to stored references:
   - **Angle shift:** distance from active target equilibrium (`|angle|` upright, `π−|angle|` inverted).
   - **Position shift:** `|position − target_position|`.
3. **Fire** (independent per axis): shift > dead band **and** ratio `current/reference` or its inverse ≥ `log_base` → update that axis's reference; fire if **either** axis spikes.
4. **On fire:** run inner controller (or PL chain including NN). **On skip:** hold `last_Q` (zero-order hold).

**Telemetry semantics** (state packet byte, bits 0–3; also CSV columns):

| `secloc_skipped_update` (bit 0) | `secloc_gate_skipped` (bit 1) | Meaning |
|---|---|---|
| 0 | 0 | **Update** — inner/PL ran |
| 1 | 1 | **Gate skip** — gate consulted, declined |
| 1 | 0 | **Throttle hold** — gate not consulted; not counted in skip statistics |
| — | bit 2 `secloc_pl_used` | Step computed by PL backend |
| — | bit 3 `secloc_pl_fault` | PL backend selected but absent/failed → Q=0 |

Terminal skip % counts only rows that are updates or gate skips (not throttle holds). Same logic in `chip_secloc_stats.py` and `secloc_gate.py`.

---

## Default SecLoc profile (Secloc2026)

| Item | Value | Where |
|---|---|---|
| Gate | `log_base=1.05`, `ref_period_ticks=4`, `dead_ang=dead_pos=0.001` | `config_secloc.yml` + `secloc_defaults.h` |
| PC inner | `neural-imitator`, `USE_SECLOC=True` | `globals.py` |
| Chip inner | `SECLOC_INNER_NNC` (C NN in `NC_C/`) | `secloc_defaults.h` |
| Backend | `SECLOC_BACKEND_PL_SHADOW` | `secloc_defaults.h` |
| IO / apply | 5 ms polling, 20 ms apply window (= 4 ticks) | `globals.py` |
| Derivative window | `TIMESTEPS_FOR_DERIVATIVE=4` (4 × 5 ms = 20 ms) | `globals.py` |
| Hardware angle filter | trimmed mean, window 63, trim 7 (firmware boot default; PC override off) | `goniometer_zynq.h` / `globals.py` |
| Deployed NN | `Dense-7IN-32H1-32H2-1OUT-8` | `config_controllers.yml`; VHDL + `NC_C/` + `nn_marshal_config.h` |

**PL_SHADOW:** control Q from PL; SW gate stepped in parallel; `secloc_shadow_mismatch_count` must stay 0 (same float32 code). Promote to `SECLOC_BACKEND_PL` once verified.

**No SW fallback:** with PL backend selected, a missing chain or failed MMIO → Q=0, bit 3 set, `secloc_pl_fault_count` incremented.

---

## SecLoc parameters — edit without rebuild

### Gate profile (all three sites)

```yaml
# Driver/CartPoleSimulation/Control_Toolkit_ASF/config_secloc.yml
default:
  log_base: 1.05
  ref_period_ticks: 4      # control-loop iterations; 0/1 = consult every iteration
  dead_ang: 0.001
  dead_pos: 0.001
  poll_stats_window_s: 5.0
```

One edit reaches all three domains, via three routes:

- **PC gate:** file watcher reloads on save (`secloc_gate.py`).
- **PS gate:** driver pushes same values via `CMD_SET_SECLOC_CONFIG` (0xD3, 4×float32/int32 pack).
- **PL gate:** PS forwards them to the `secloc_shell` sticky registers on config change — no rebuild.
- **Standalone chip (no PC attached):** PS boots from the mirror in `Firmware/Src/General/secloc_defaults.h` (`SECLOC_DEFAULT_*`) — keeping that in sync requires a firmware rebuild.

Keep `ref_period_ticks` aligned with `CONTROLLER_APPLY_WINDOW_MS / POLLING_PERIOD_MS` when using PC split control (currently 4).

### Execution backend (chip only, rebuild firmware)

`Firmware/Src/General/secloc_defaults.h` → `SECLOC_DEFAULT_BACKEND` — decides **where Q is computed in `u` mode** (PC path is unaffected):

| Macro | Q computed on | Gate stepped on |
|---|---|---|
| `SECLOC_BACKEND_SW` | PS (inner via `secloc_inner_evaluate`) | PS |
| `SECLOC_BACKEND_PL` | PL chain | PL only |
| `SECLOC_BACKEND_PL_SHADOW` | PL chain | PL + PS (compare) |

Runtime API: `secloc_set_backend()` — no serial command; change default + rebuild for experiments.

### Inner controller

**PC (`k`):** `globals.py` → `CONTROLLER_NAME` (`mpc`, `neural-imitator`, `pid`, `lqr`, …). SecLoc wrapper is always `controller_secloc`; inner loaded via `set_inner_controller_name()`. Inner yaml: `config_controllers.yml` entry for that name.

**Chip (`u`):** `secloc_defaults.h` → `SECLOC_DEFAULT_INNER_CONTROLLER`:

```c
SECLOC_INNER_NNC   // NC_C/network.c — default
SECLOC_INNER_LQR
SECLOC_INNER_PID
```

SW-path NN input order differs from SecLoc spec; remapped in `secloc_nnc_evaluate()` (`secloc_controller.c`).

**PC ctypes parity with firmware:** `config_controllers.yml` → `secloc-c` entry compiles `secloc_controller.c` + `secloc_logic.c` on the host.

---

## C parameters: overwritten by the PC driver vs firmware-truth

Not every value in the firmware sources is authoritative. Three classes:

### Overwritten at driver connect — C value matters **only when the chip runs standalone** (no `control.py`)

The driver pushes these from the PC at startup (and re-pushes as noted); editing them in C changes nothing while the driver is attached.

| C value (boot default) | Where in C | Overwritten from | When |
|---|---|---|---|
| Gate `log_base`, `ref_period_ticks`, `dead_ang`, `dead_pos` (`SECLOC_DEFAULT_*`) | `secloc_defaults.h` | `config_secloc.yml` via `CMD_SET_SECLOC_CONFIG` | At connect + on every yaml save |
| Gate tick size (`SECLOC_DEFAULT_TIME_QUANTUM_S`) | `secloc_defaults.h` | Derived from `POLLING_PERIOD_MS` | At connect + on control-config push |
| `POLLING_PERIOD_MS`, `CONTROL_SYNC`, `ANGLE_AVERAGE_LEN`, `TIMESTEPS_FOR_DERIVATIVE`, `correct_motor_dynamics`, `ANGLE_HANGING` | `parameters.c` boot values | `Driver/globals.py` via `CMD_SET_CONTROL_CONFIG` | At connect, after calibration, after `b` |
| `target_position`, `target_equilibrium` | `control.c` init values | Keyboard / dance / protocols via `CMD_SET_TARGET_*` | Continuously while connected |

For standalone-chip experiments, mirror your yaml/`globals.py` values into `secloc_defaults.h` / `parameters.c` and rebuild — that is the only reason to touch those C defaults.

### Conditionally overwritten — firmware value rules **unless the PC explicitly overrides**

| C value | Where in C | PC override | Default behavior |
|---|---|---|---|
| PL angle filter (trimmed mean 63/7) | `goniometer_zynq.h` boot default | Only if `HARDWARE_ANGLE_FILTER_OVERRIDE=True` in `globals.py` (`CMD_SET_ANGLE_FILTER`) | Override is **off** → firmware default is what runs |
| On-chip PID gains | `hardware_pid.c` | `CMD_SET_PID_CONFIG` exists, but the driver's push (`set_firmware_parameters()`) is commented out in `PhysicalCartPoleDriver.setup()` | Firmware gains rule unless you re-enable/call the push manually |

### Never overwritten — genuine firmware-truth, rebuild to change

No serial command touches these; the C source is the single authority even with the driver connected.

| Value | Where in C |
|---|---|
| On-chip controller selection (`ON_CHIP_BOOT_CONTROLLER`, ids 0–6) | `control.c` |
| SecLoc inner controller (`SECLOC_DEFAULT_INNER_CONTROLLER`) | `secloc_defaults.h` |
| SecLoc backend (`SECLOC_DEFAULT_BACKEND`: SW / PL / PL_SHADOW) | `secloc_defaults.h` (runtime API `secloc_set_backend()` exists but has no serial command) |
| LQR gains | `lqr.c` (`LQR_DefaultGains`) — keep manually in sync with yaml `lqr:` Q/R |
| Legacy SecLoc+LQR gains and its fused gate constants (`log_base=1.05`, dead bands 0) | `secloc_lqr.c` |
| C NN weights | `NC_C/network_parameters.c` — must match TF model + PL bitstream |
| Gate algorithm | `secloc_logic.c` (+ PL HLS include + Python mirror) |
| Board/motor constants (PWM period, encoder range, board define) | `parameters.c` / `hardware_bridge.h` — must match the mirrors in `globals.py` |

Rule of thumb: **anything the PC can express in yaml/`globals.py` gets pushed and wins; anything structural (which controller, which backend, which weights, which algorithm) lives only in C.**

---

## Control modes and keyboard reference

PC control (`k`) and chip control (`u`) are mutually exclusive. Chip boots `ON_CHIP_BOOT_CONTROLLER = OnChipController_SECLOC` (`control.c`). Full bindings (`Driver/DriverFunctions/keyboard_controller.py`):

| Key | Effect |
|---|---|
| `h` / `?` | Print key list |
| `Shift+K` | Calibrate track centre (`CMD_CALIBRATE`) |
| `k` | PC control on/off (SecLoc wrapper + `CONTROLLER_NAME` inner) |
| `u` | Chip control on/off — **primary PL SecLoc path** |
| `5` | Print `CMD_GET_SECLOC_INFO`: `backend`, `pl_available`, `shadow_mismatches`, `pl_update_count`, `pl_nn_wait_cycles`, `pl_faults` |
| `l` | CSV recording start/stop |
| `Shift+L` | Time-limited recording (`TIME_LIMITED_RECORDING_LENGTH` = 1000 steps) |
| `;` | Toggle target equilibrium (upright ↔ hanging) |
| `[` / `]` | Target position − / + (10 encoder counts ≈ 0.9 mm per press, clipped to ±80% track) |
| `b` | Measure hanging angle → push new `ANGLE_HANGING` to chip |
| `=` / `-` | Finetune zero angle ±0.002 rad |
| `Shift+D` | Dance mode on/off (target follows `DANCE_PATH` in `globals.py`: square/sin, amplitude, period) |
| `m` / `n` | Cycle / start-stop experiment protocol (scripted target sequences) |
| `Shift+N` | Run hardware experiment from chip (`CMD_RUN_HARDWARE_EXPERIMENT`, offline buffers) |
| `9` / `0` | Artificial latency +1 / −1 ms |
| `6` / `7` / `8` | Live plotter: start server / save plot / reset |
| `j` | Joystick on/off |
| `ESC` | Graceful exit (motor 0, close serial) |

With `CONTROLLER_NAME='pid'` extra keys tune PID gains live (`1/2`, `q/w`, `a/s` = angle KP/KI/KD down/up; `3/4`, `e/r`, `d/f` = position; `p` print, `Shift+S` save JSON). Note `Shift+L` then triggers both PID-load and time-limited recording.

---

## Controllers catalog

### PC (Python, selected via `CONTROLLER_NAME` in `Driver/globals.py`)

Auto-discovered from `Control_Toolkit_ASF/Controllers/` and `Control_Toolkit/Controllers/`. With `USE_SECLOC=True` the driver wraps the chosen controller in the modular `secloc` gate wrapper.

| Name | Description | SecLoc relation |
|---|---|---|
| `neural-imitator` | TF neural policy (`nn_evaluator_mode: 'C'` = generated `network.c` for on-chip math parity) | Default inner, `USE_SECLOC=True` |
| `mpc` | MPC; optimizer from `OPTIMIZER_NAME` (`rpgd-c` parallel, `rpgd` single-thread + pin `CONTROL_CPU_AFFINITY`; also cem-*, mppi, gradient-tf, …) | Supported inner (split loop) |
| `pid` | Cascaded angle/position PID, live keyboard tuning | Inner via wrapper |
| `lqr` | Continuous-time LQR; `Q`/`R` hot-reload from yaml | Inner via wrapper |
| `secloc` | Modular gate wrapper (this is what `USE_SECLOC=True` on the rig and the GUI's **Use SecLoc** checkbox load); not selectable directly — always needs an inner | Gate itself |
| `secloc-lqr` | **Legacy monolithic** gate+LQR fused; PC twin of on-chip `secloc_lqr.c`. Run with `USE_SECLOC=False` (gate built in) | Legacy baseline |
| `secloc-do-mpc-discrete` | **Legacy monolithic** gate+do-mpc, event-driven re-optimization with per-step `dt` tvp. `USE_SECLOC=False` | Legacy baseline |
| `secloc-c` | On-chip SecLoc C sources compiled via ctypes on the PC (decision parity); forces `SECLOC_BACKEND_SW` since no PL exists on the host | Parity harness |
| `c` | Generic ctypes wrapper for any firmware `ControllerOps` (`controller_file`, `ops_name` in yaml) | Utility |
| `do-mpc`, `do-mpc-discrete`, `mppi-cartpole`, `difflg`, `embedded`, `remote`, `manual-stabilization` | Non-SecLoc controllers / research extras | — |

Hyper-parameters per controller: `Driver/CartPoleSimulation/Control_Toolkit_ASF/config_controllers.yml` (entry name = controller name).

### On chip (`Firmware/Src/CartPoleFirmware/control.c`, compile-time `ON_CHIP_BOOT_CONTROLLER`)

| ID | Macro | Controller |
|---|---|---|
| 0 | `OnChipController_PID` | Cascaded PID (`hardware_pid.c`) |
| 1 | `OnChipController_NeuralImitator` | FPGA neural imitator binding (`Zynq/neural_imitator.c`) |
| 2 | `OnChipController_PID_position` | Position-only PID |
| 3 | `OnChipController_LQR` | LQR (`lqr.c`, gains synced with yaml `lqr:`) |
| 4 | `OnChipController_neural_controller_C` | Pure-C NN (`NC_C/network.c`) |
| 5 | `OnChipController_SECLOC` | **Modular SecLoc + inner (boot default)** |
| 6 | `OnChipController_SECLOC_LQR` | Legacy monolithic SecLoc+LQR (`secloc_lqr.c`, fixed float gains) |

No runtime controller-index command exists; changing the on-chip controller = edit `ON_CHIP_BOOT_CONTROLLER` + rebuild.

---

## Rig timing and signal conditioning

All pushed from `Driver/globals.py` at startup via `CMD_SET_CONTROL_CONFIG` (re-sent after calibration and `b`). Restart `control.py` after editing.

| Parameter (`globals.py`) | Default | Affects | Meaning |
|---|---|---|---|
| `POLLING_PERIOD_MS` | 5 (SecLoc profile; auto-selected per `CONTROLLER_NAME`) | **PC + PS + PL** | PS control-loop and PC IO-thread period; also the gate's tick size (`time_quantum`), so it rescales `ref_period_ticks` in all three gate sites |
| `CONTROLLER_APPLY_WINDOW_MS` | 20 | PC | Split loop: trigger → apply latency; must be multiple of polling period. Irrelevant in `u` mode |
| `CONTROL_SYNC` | True | PS | Motor command applied at next timeslot (deterministic latency) |
| `TIMESTEPS_FOR_DERIVATIVE` | 4 | PS (PC mirrors it for its own state view) | angleD/positionD finite-difference window in polling periods (PS clamps 1–20); larger = smoother, more lag. Feeds **both** PC and chip controllers, since derivatives are computed on the PS |
| `ANGLE_AVG_LENGTH` | 1 | PS | Rapid successive ADC reads averaged per sample (max 32); 1 = off, the PL filter does the smoothing |
| `CORRECT_MOTOR_DYNAMICS` | True (False for `pid`) | PC + PS (applied wherever Q→PWM runs: PC in `k` mode, PS in `u` mode) | Friction/gain compensation (`MOTOR_CORRECTION` tuples per motor/controller) |
| `SERIAL_PORT_NUMBER`, `SERIAL_BAUD` | 0, 230400 | PC | Index into detected `ttyUSB*/ttyACM*`; baud fixed by firmware |
| `CONTROL_CPU_AFFINITY`, `LOOP_CPU_AFFINITY` | `""`, `"3"` | PC | TF-compute / IO-thread core pinning (match to optimizer) |

### Sensor filtering pipeline (all stages, in signal order)

The angle signal passes through up to five conditioning stages before a controller (or the gate) sees it. Each is independently configurable:

| # | Stage | Domain | Knob | Default | Change via |
|---|---|---|---|---|---|
| 1 | **PL angle filter** on raw XADC samples (~2.2 µs each) | PL | `filter_mode` + `window` (1–64) + `trim` | trimmed mean 63/7 | `HARDWARE_ANGLE_FILTER_*` in `globals.py` (restart; only if `OVERRIDE=True`, else firmware default rules) |
| 2 | **PS median over quick ADC reads** per control step | PS | `ANGLE_AVG_LENGTH` (1–32; `ClassicMedianFilter`) | 1 (off — stage 1 does the smoothing) | `globals.py` (restart, pushed via `CMD_SET_CONTROL_CONFIG`) |
| 3 | **Dead-zone freeze/extrapolation** — while the pole crosses the potentiometer gap the FPGA flags contaminated samples; PS holds the last stable derivative and extrapolates the angle (`treat_deadangle_with_derivative`) | PL flag + PS logic | `HW_DZ_MAX_EXTRAPOLATION_MS=500`, settling tolerance/cap | on (automatic) | Compile-time constants in `angle_processing.c` (firmware rebuild); contaminated steps show up in the `invalid_steps` CSV column |
| 4 | **Finite-difference derivative** for angleD/positionD | PS | `TIMESTEPS_FOR_DERIVATIVE` (1–20 polling periods) | 4 with SecLoc @ 5 ms (= 20 ms window), else 1 | `globals.py` (restart) |
| 5a | **PS derivative median** after stage 4 | PS | `ANGLE_D_BUFFER_SIZE` / `POSITION_D_BUFFER_SIZE` | 1 (off) | Compile-time in `angle_processing.c` (firmware rebuild) |
| 5b | **PC derivative median** on received state | PC | `ANGLE_D_MEDIAN_LEN` / `POSITION_D_MEDIAN_LEN` | 1 (off) | `globals.py` (restart; `incoming_data_processor.py`) — affects PC controllers/gate only, not the chip |

Everything the gate and the controllers consume is downstream of stages 1–4 (both PC and chip paths), so filter changes shift the noise floor that `dead_ang`/`dead_pos` and `log_base` operate on. Stage 5b applies only to the PC control path.

#### Stage 1 details — `CMD_SET_ANGLE_FILTER` (PL)

| `filter_mode` | Name | Meaning |
|---|---|---|
| 0 | raw | Passthrough |
| 1 | median | Rolling-window median |
| 2 | trimmed mean | Sort window, drop `trim_count` from each end, average rest (`trim=0` → plain mean) |

Implementation `Firmware/Src/Zynq/goniometer_zynq.c`; boot default in `goniometer_zynq.h`. To override from the PC set in `globals.py`: `HARDWARE_ANGLE_FILTER_OVERRIDE=True` + `_WINDOW` / `_TRIM` / `_MODE` (sent once at startup; recorded in CSV header). Requirement: `2*trim < window`.

---

## Target position and equilibrium

| Method | How |
|---|---|
| Keyboard | `[` / `]` position, `;` equilibrium flip |
| Dance mode | `Shift+D`; path/amplitude/period: `DANCE_*` in `globals.py` |
| Experiment protocols | `m` select, `n` run — scripted target sequences (`DriverFunctions/ExperimentProtocols/`) |
| Zedboard switches | Only if `USE_TARGET_SWITCHES=true` (`parameters.c`; default false): switch 1 = ±0.09 m position jumps every 1000 iterations, switch 2 = equilibrium. Any `CMD_SET_TARGET_POSITION` from the PC disables switch control |

Targets are forwarded to the chip (`CMD_SET_TARGET_POSITION/EQUILIBRIUM`), so they steer both PC and chip control; the gate's position criterion is relative to `target_position`, so target steps are the canonical disturbance experiment.

---

## PC SecLoc control loop (split IO / compute)

When `USE_SECLOC=True` and inner is `mpc` or `neural-imitator`:

- **IO thread** (5 ms): `controller_secloc.should_trigger()` — cheap gate only.
- **Main thread:** `compute_step()` — inner controller (TF).
- **Apply:** fresh Q on deadline tick = `CONTROLLER_APPLY_WINDOW_MS` after trigger; ZOH between.

Implementation: `Driver/DriverFunctions/split_control_loop.py`. CSV: `controller_update_applied`, `split_control_busy`, `secloc_skipped_update`, `secloc_gate_skipped`.

Chip control (`u`) has no split — full `SECLOC_Evaluate` every firmware loop iteration. Other inners (`pid`, `lqr`, legacy `secloc-*`) run synchronously in the IO loop.

---

## SecLoc in the simulation GUI (no rig)

`python Driver/CartPoleSimulation/run_cartpole_gui.py` runs the simulator-only GUI. SecLoc there works via a checkbox, not a controller entry:

- Pick an inner controller in the **Controller** radio list (`lqr`, `mpc`, `neural-imitator`, …), then tick **Use SecLoc** below the list. This wraps the selection in the modular gate wrapper (`controller_secloc`) — the same thing `USE_SECLOC=True` loads on the rig. Untick to return to the plain controller.
- The `secloc` wrapper does **not** appear in the radio list; it is not a standalone controller (it needs an inner). The checkbox is the only way to enable it in the GUI.
- The checkbox is disabled for `manual-stabilization` and for controllers with SecLoc already built in (`secloc-c`, `secloc-lqr`, `secloc-do-mpc-discrete`) — wrapping those would double-gate. It is also locked while an experiment is running, like the controller radio buttons.
- Gate params come from `config_secloc.yml` as everywhere else (file watcher reloads on save). The gate's `ref_period_ticks` tick in the GUI is the simulation's `controller_update_interval` (`config_gui.yml`), not `POLLING_PERIOD_MS`.
- To start the GUI with the gate on, set `use_secloc_init: true` in `Driver/CartPoleSimulation/config_gui.yml` (default false).

`secloc-c` in the GUI compiles the on-chip C sources (`secloc_controller.c` + gate + inner controllers, including `secloc_controller_pl.c` for the PL-backend symbols) into a shared library via ctypes. Because the firmware boot default backend is `PL_SHADOW` and no PL exists on the PC, the wrapper forces `SECLOC_BACKEND_SW` at load — otherwise every step would fault to zero force by the no-fallback policy. Result: gate + C inner (default `SECLOC_INNER_NNC`) with on-chip math parity, in simulation.

---

## Recording

- **`l`** toggles CSV → `Driver/ExperimentRecordings/CPP_<controller>[_<optimizer>]_<date>_<time>.csv`; **`Shift+L`** records exactly `TIME_LIMITED_RECORDING_LENGTH` steps.
- **Header metadata:** git revision, controller/optimizer, `dt`, full physical-parameter dump, **SecLoc block** (`log_base`, `ref_period_ticks`, `dead_ang`, `dead_pos`, on-chip controller label), CPU affinity, angle-filter override. The analysis tools parse this header, so gate params never have to be tracked by hand.
- **Columns (always):** `time`, `deltaTimeMs`, `time_chip`, `angle[_raw|_cos|_sin]`, `angleD[_raw]`, `position[_raw]`, `positionD`, `target_position`, `target_equilibrium`, `Q`, `actualMotorSave`, `latency`, `firmware/controller_latency_violations`, `pythonLatency`, `controller_steptime`, `invalid_steps`, …
- **SecLoc columns:** `secloc_skipped_update`, `secloc_gate_skipped` (both paths); `secloc_pl_used`, `secloc_pl_fault` (chip path); `controller_update_applied`, `split_control_busy` (PC split loop).

---

## Analysis and plotting tools

Two tools, both run from the repo root; output PNGs go to `tests/output/`.

### `tests/plot_secloc_experiment.py` — the four-panel experiment figure

Runs against a recording in `Driver/ExperimentRecordings/`; `--latest` picks the most recent, or pass an explicit CSV path.

```bash
python tests/plot_secloc_experiment.py --latest              # newest recording
python tests/plot_secloc_experiment.py path/to/CPP_....csv
# options: --window-s 0.5 (rolling-average window for all % curves)
#          --stab-angle 0.1 (|angle| threshold for upright-hold shading)
#          --chip (force on-chip semantics; auto-detected from header)
#          --output-dir DIR (default tests/output/)
```

Writes `<csv-stem>_secloc_timeseries.png` — four stacked panels over experiment time, with upright-stabilization phases shaded in all panels:

| Panel | Content |
|---|---|
| 1 | **Angle** vs target angle |
| 2 | **Position** vs target position |
| 3 | Share of decisions where the raw **sensor reading was bit-identical** to the previous decision (angle / position separately) — rises during quiet holds (quantization plateaus) |
| 4 | **Skipped control updates**, split by what changed between consecutive decisions: nothing / only angle / only position / both (stacked areas summing to the replayed total); overlaid with the logged hardware gate skip rate (dashed) and the **coasting time** — share of wall time spent on the held plan (dotted) |

Internally it re-runs the real `SeclocGate` over the logged states (float64, exactly as the PC driver ran it) and validates the replayed decisions against the logged telemetry, so the figure doubles as a consistency check. Works for both PC-gate and on-chip recordings (chip mode auto-detected); a "how to read this figure" caption with the run's log_base and timing is embedded in the PNG.

### `tests/test_secloc_log_base_sweep.py` — simulated log_base sweep vs noise (no rig, no recording)

Simulates PID stabilization at 5 ms polls under increasing measurement noise (until PID loses the pole), replays each recorded trajectory through the gate over a `log_base` grid, and plots **skip % vs log_base with one curve per noise level**.

```bash
python tests/test_secloc_log_base_sweep.py               # float sensors
python tests/test_secloc_log_base_sweep.py --quant       # ADC/encoder-quantized sensors (hardware-like)
python tests/test_secloc_log_base_sweep.py --plot-only   # replot from cached sweep data (fast)
python tests/test_secloc_log_base_sweep.py --recompute   # ignore cache, rerun full sweep
```

Use it to pick a `log_base` for a target skip rate before occupying the hardware, and to see how much noise inflates event rates. The quantized variant additionally separates true gate skips from ADC-plateau artifacts (skip|changed vs skip|flat curves). Sweep data is cached under `tests/output/secloc_sweep_cache/`.

---

## What reloads live vs what needs restart/rebuild

| Change | Affects | Mechanism | Effort |
|---|---|---|---|
| Gate params (`config_secloc.yml`) | PC + PS + PL | File watcher (PC) + auto push to PS → PL sticky registers | Save file |
| LQR `Q`/`R` (`config_controllers.yml`) | PC | File watcher in `controller_lqr` | Save file |
| PID gains (PC controller) | PC | Keyboard keys | Live |
| Target position / equilibrium / latency / zero-angle | PC + PS (targets forwarded to chip; gate inputs everywhere) | Keyboard | Live |
| `CONTROLLER_NAME`, `USE_SECLOC`, `OPTIMIZER_NAME`, apply window, serial, affinity | PC | Read at startup | Restart `control.py` |
| Polling period, derivative window, `ANGLE_AVG_LENGTH`, `CONTROL_SYNC` | PC + PS (pushed via `CMD_SET_CONTROL_CONFIG`) | Read at startup | Restart `control.py` |
| Angle filter override (`HARDWARE_ANGLE_FILTER_*`) | PL | Read at startup, sent via `CMD_SET_ANGLE_FILTER` | Restart `control.py` |
| Other `config_controllers.yml` entries (NN name, MPC horizon, secloc-lqr gate params, …) | PC | Read at controller construction | Restart `control.py` |
| Chip inner controller, backend, standalone gate defaults, boot controller id | PS (backend selects PS-vs-PL execution) | `secloc_defaults.h` / `control.c` | Firmware rebuild |
| Gate algorithm | PC (`secloc_logic.py`) + PS (`secloc_logic.c`) + PL (same C included in HLS) | Edit all mirrors | Firmware **and** FPGA rebuild (+ Python mirror edit) |
| NN weights | PC (TF model) + PS (`NC_C/`) + PL (VHDL + `nn_marshal_config.h`) | Three artifacts must match | yaml swap (PC) / firmware rebuild (PS) / bitstream swap (PL) |

---

## FPGA touch points (SecLoc only)

| Change | Path |
|---|---|
| Gate algorithm | `Firmware/Src/General/secloc_logic.c` (+ Python mirror); PL includes same file |
| PL gate + ZOH + NN watchdog | `FPGA/CustomIPs/secloc_gate_hls/secloc_gate.cpp` |
| PS register front-end | `FPGA/CustomIPs/secloc_shell_hls/`, `Firmware/Src/Zynq/secloc_frontend_link.c` |
| NN normalize/quantize | `FPGA/CustomIPs/nn_marshal_hls/` + `nn_marshal_config.h` (per network) |
| Swap NN in bitstream | `FPGA/VivadoProjects/swap_nn_and_build.sh [network_dir]` — re-synth **marshal only** |
| Build all three IPs | `FPGA/CustomIPs/build_secloc_chain.sh` |
| Block design | `FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_Zedboard.tcl` |

Chip C weights (`Firmware/Src/General/NC_C/`) must match deployed VHDL + marshal header.

Deeper PL notes: `Docs/SecLoc_PL_Integration_Report.md`, `FPGA/CustomIPs/README_secloc_chain.md`.

---

## SecLoc file index

| Purpose | Path |
|---|---|
| Gate profile (runtime) | `Driver/CartPoleSimulation/Control_Toolkit_ASF/config_secloc.yml` |
| Gate logic (single source) | `Firmware/Src/General/secloc_logic.c` |
| PC wrapper | `Driver/CartPoleSimulation/Control_Toolkit_ASF/Controllers/controller_secloc.py` |
| PC gate plumbing | `Driver/CartPoleSimulation/Control_Toolkit_ASF/Controllers/secloc_gate.py` |
| Legacy monolithic (PC) | `.../Controllers/controller_secloc_lqr.py`, `controller_secloc_do_mpc_discrete.py` |
| Legacy monolithic (chip) | `Firmware/Src/General/secloc_lqr.c` (`SECLOC_LQR_Ops`, id 6) |
| Chip defaults | `Firmware/Src/General/secloc_defaults.h` |
| Chip SecLoc + inner dispatch | `Firmware/Src/General/secloc_controller.c` |
| PL backend / no-fallback policy | `Firmware/Src/General/secloc_controller_pl.c`, `.h` |
| Inner enum | `Firmware/Src/General/secloc_inner.h` |
| Serial SecLoc cmds | `Driver/DriverFunctions/interface.py` (`CMD_SET_SECLOC_CONFIG`, `CMD_GET_SECLOC_INFO`) |
| Chip telemetry aggregation | `Driver/DriverFunctions/chip_secloc_stats.py` |
| Analysis / plotting | `tests/plot_secloc_experiment.py` (four-panel figure), `tests/test_secloc_log_base_sweep.py` (simulated noise sweep) |
| PC settings | `Driver/globals.py` (`USE_SECLOC`, `CONTROLLER_NAME`, timing, filter, serial) |
| GUI settings (simulation) | `Driver/CartPoleSimulation/config_gui.yml` (`use_secloc_init`, `controller_update_interval`) |

---

## Troubleshooting (SecLoc)

| Symptom | Likely cause |
|---|---|
| `PL-FAULT`, cart dead | PL chain absent in bitstream or MMIO timeout; check `5`. Use `SECLOC_BACKEND_SW` to isolate gate on CPU. |
| `shadow_mismatches > 0` | SW/PL gate divergence — should not occur; check param sync (`CMD_SET_SECLOC_CONFIG` vs yaml). |
| Skip rate ≠ expectation | Distinguish gate skips vs throttle holds in CSV; panel 4 of `plot_secloc_experiment.py` shows the replayed-vs-logged breakdown. |
| PC mpc unstable / late | `compute_step` overruns apply window; increase `CONTROLLER_APPLY_WINDOW_MS` or use chip PL path. |
| Gate edits ignored on chip without PC | Edit `secloc_defaults.h` and rebuild firmware. |
| Noisy angleD triggering spurious gate events | Increase `TIMESTEPS_FOR_DERIVATIVE`, angle-filter window, or dead bands. |
| No serial connection | Check `SERIAL_PORT_NUMBER` in `globals.py`; Zynq enumerates as Digilent USB (`ttyUSB*`). |
| `plot_secloc_experiment.py` rejects a CSV | Old-format recording (missing `Secloc ref_period_ticks` header or telemetry columns); re-record on this branch. |
