# Neural Control Tuning Log — Physical Cartpole

Goal: get the `neural-imitator` controller (Dense nets only) to do **swing-up + stabilization** on the long pole (`L = 0.395`).

Pipeline facts (constant across runs):
- Controller: `neural-imitator`, CHIP `ZYNQ`, MOTOR `POLOLU`.
- Net input order (7): `angleD, angle_cos, angle_sin, position, positionD, target_equilibrium, target_position`.
- Sensor scaling done in `Driver/DriverFunctions/incoming_data_processor.py`; derivatives are dt-corrected (divided by measured loop time).
- Firmware `ANGLE_360_DEG_IN_ADC_UNITS = 4049.44` matches `Driver/globals.py` (consistent).
- Angle convention: upright = 0 rad, hanging = ±π.

Knobs we change (where):
- Net: `Driver/CartPoleSimulation/Control_Toolkit_ASF/config_controllers.yml` → `neural-imitator: PATH_TO_MODELS + net_name`.
- Motor calibration: `Driver/globals.py` → `MOTOR_CORRECTION_POLOLU` (PC-control path; firmware `MOTOR_CORRECTION` is on-chip only).
- Control rate: `Driver/globals.py` → `CONTROL_PERIOD_MS` (neural-imitator branch).
- Input quantization: `config_controllers.yml` → `input_precision`.

Motor-calibration reproducibility (RESOLVED 2026-06-19):
- **Problem found:** the in-use driver value `(0.5701800, …)` ("July 2024 paper") was NOT regenerable from any recording in the repo — an orphaned legacy number. Driver also disagreed with firmware. Raw `CPP_step_response.csv` was gitignored, with two divergent copies → not reproducible.
- **Fix:** recorded a fresh bidirectional step response (`CPP_step_response-1.csv`), made it the canonical `Driver/DataAnalysis/MotorAndCartFriction/CPP_step_response.csv` (force-add past .gitignore), and adopted the **force-based** calibration as canonical.
- **Canonical value (force-fit):** `MOTOR_CORRECTION_POLOLU = (0.5116974, 0.0178784, 0.0280385)` — reproducible via `python MotorCalibration.py --force-fit` (u_max_target=2.62 N, effective_mass=0.317 kg, PWM=10000). Same tuple goes in `globals.py` (ZYNQ) AND firmware `parameters.c` (ZYNQ). Re-test pending.
- Why force method: anchors `Q=1` to the training force scale (2.62 N) — physically correct for a force-trained net — and self-checks `Q=±1` stays in motor's linear range. Velocity method anchors to an arbitrary v_sat and yields noise-sensitive friction offsets.
- New-motor workflow: record step response → `MotorCalibration.py --force-fit` → paste tuple into globals.py + parameters.c → reflash → test.

Older calibration values (legacy / not canonical):
- July 2024 paper (orphaned):   `(0.5701800, 0.0361973, 0.0272124)`
- "ready for exp" commit 912febd3: `(0.6216901, 0.0750750, 0.0549491)`
- velocity-method on fresh data:  `(0.5906047, -0.0055199, 0.0091346)`

---

## Experiment runs

| # | Recording (CPP_neural-imitator_*) | Net | Motor calib | dt (ms) | input_precision | Swing-up | Longest balance | Notes |
|---|---|---|---|---|---|---|---|---|
| R0 | 2026-06-19_11-55-00 | (pre-session, idle) | 1.77 N era | 5 | — | no | 0 s | idle/old, not a real attempt |
| R1 | 2026-06-19_13-04-41 | Dense-7IN-32H1-32H2-1OUT-1 (paper) | July2024 0.5702 | 5 | ap_fixed<12,2> | yes (assisted) | 4.86 s | under-damped, |Q|>1 frequent |
| R2 | 2026-06-19_13-13-36 | Dense-7IN-32H1-32H2-1OUT-0 (CPS-2023) | 0.6216901 | 5 | ap_fixed<12,2> | yes | 1.80 s | strong force, still under-damped |
| **R3** | **2026-06-19_13-28-11** | **Dense-7IN-32H1-32H2-1OUT-1 (paper)** | **July2024 0.5702** | **10** | **ap_fixed<12,2>** | **NO (hand needed)** | **33.37 s (83% up)** | **BEST stabilization. dt 5→10 fixed damping. Net can't pump (swing-up data filtered out in training).** |

### Key learnings
- **Motor force was NOT the core issue** (R2 had strong force, still failed). Earlier 1.77 N value was a regression but not the root cause.
- **Control rate is decisive for stabilization:** 5 ms → noisy derivatives → under-damped (R1/R2 ≤5 s). 10 ms → clean derivatives → 33 s balance (R3).
- **Paper net = great stabilizer, poor swing-up** by design: IROS24 training data filtered `|angleD|<20, |positionD|<1`, removing swing-up motion.
- Signs of all feedback terms are correct (not a direction bug). Inputs are mostly within training range.

### CURRENT BEST KNOWN-GOOD (ACCEPTED) ✅
- Net: `Dense-7IN-32H1-32H2-1OUT-8` at `PATH_TO_MODELS: ./CartPoleSimulation/SI_Toolkit_ASF/Experiments` (output `Q_calculated`)
- `MOTOR_CORRECTION_POLOLU = (0.5116974, 0.0178784, 0.0280385)` (force-fit, reproducible; replaced legacy 0.5701800 — re-test pending)
- `CONTROL_PERIOD_MS = 10`
- `TIMESTEPS_FOR_DERIVATIVE = 1` in BOTH `globals.py` and firmware `parameters.c` (matched)
- `input_precision: ap_fixed<12,2>`, `nn_evaluator_mode: normal`
- Result: reliable swing-up + stabilization + disturbance rejection. Acceptable. (`-1OUT-8` angleD range ±21.2 also tolerates the kick-spin and the N=1 derivative noise.)
- Note: angleD (firmware) and positionD (driver) windows MUST use the same `TIMESTEPS_FOR_DERIVATIVE`, else the two velocities get inconsistent noise/lag.

### Earlier best (stabilization-only)
- `Dense-7IN-32H1-32H2-1OUT-1` (paper), July2024 calib, 10 ms: 33 s balance but no autonomous swing-up.

---

## Derivative-noise finding (root cause of the dt effect)
Measured from recordings near upright:
- 5 ms: `angleD` quantum 0.310 rad/s, jitter ~0.8–1.0 rad/s (≈ signal magnitude → kills damping).
- 10 ms: `angleD` quantum 0.155 rad/s, jitter ~0.27 rad/s.
- Theory: `angleD` quantum = (2π/4049.44)/(N·dt). So **N=2 @ 5 ms = N=1 @ 10 ms = 0.155 rad/s**, but keeps 5 ms control bandwidth.
Conclusion: the dt change helped only *indirectly* via the derivative window. Decouple them: keep dt=5 ms, set `TIMESTEPS_FOR_DERIVATIVE = 2`.

`TIMESTEPS_FOR_DERIVATIVE` must be set in BOTH:
- `Firmware/Src/CartPoleFirmware/parameters.c` (computes `angleD` on-chip) → **requires reflash**.
- `Driver/globals.py` (computes `positionD` in driver + history buffers).

## R4 — derivative fix validated (dt=10 ms + TIMESTEPS_FOR_DERIVATIVE=2)
| # | Recording | Net | calib | dt | N (deriv) | Result |
|---|---|---|---|---|---|---|
| R4 | 2026-06-19_13-51-14 | Dense-7IN-32H1-32H2-1OUT-0 (CPS-2023) | July2024 0.5702 | 10 ms | 2 | **Swings up reliably + rejects disturbances.** But hard kick from top → over-the-top SPIN it can't recover. |

- Confirmed: `TIMESTEPS_FOR_DERIVATIVE=2` (firmware reflashed + driver) gives clean derivatives → swing-up + disturbance rejection both work. Big milestone.
- Spin analysis: during spin `|angleD|` reaches **17.5 rad/s**; motor NOT saturated, cart NOT at track end, Q weak/ineffective. Cause: spin exceeds CPS-2023 net's `angleD` training max (**14.8**) → out-of-distribution → net can't de-spin.

### Dense nets by angleD training range (drop-in 7-input only)
- CPS-2023 `…-1OUT-0`: ±14.8 (spins out)
- paper `…-1OUT-1`: ±20.1 (covers spin, but poor swing-up)
- **`…-1OUT-8` (Experiment-14): ±21.2 (covers spin, swing-up era)** ← trying next
- Wider nets (±40–156) are dynamics models / short-pole / 8-input (need Q_applied_-1) → NOT drop-in.

## Next experiments
- [ ] **E-spin (current setup):** net `Dense-7IN-32H1-32H2-1OUT-8`, July2024 calib, dt=10 ms, N=2. Hypothesis: wider angleD range (±21.2) lets it act during the kick-spin while keeping swing-up + balance.
- [ ] If still spins: the de-spin (energy-removal from sustained rotation) regime may be uncovered by all narrow Dense nets → options: (a) adapt pipeline to feed `Q_applied_-1` and use the ±40.8 8-input nets, (b) add a guard that triggers re-swing-up when |angleD| beyond training range, (c) accept hard-kick spin as edge case.
- [ ] Sweep N=3 (smoother) only if balance needs it.
