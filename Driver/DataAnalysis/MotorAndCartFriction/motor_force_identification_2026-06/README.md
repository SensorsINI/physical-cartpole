# Motor force / cart-dynamics identification — investigation log (June 2026)

**Status: RESOLVED OFFLINE (June 2026); one hardware retest pending.** See section 9 for
the resolution. In short: the earlier `u_max=13` (EMF shuttle) that broke RPGD is **wrong**;
forward-prediction validation against the real plant identifies the honest set
`m_cart=2.82`, `u_max=21.7`, `M_fric=36.8`, `v_max=0.59` (acceleration gain 7.70, equal to
the only authority ever confirmed to drive the hardware). The only configuration *previously*
confirmed to swing up was the old `m_cart=0.230`, `u_max=1.77`, `M_fric=3.22` — physically
wrong in absolute magnitude but with the same gain. This folder archives the data, analysis
scripts, and reasoning. Sections 1–8 below are the original investigation log (kept for the
record); section 9 is the offline resolution.

This document is meant to be self-contained: a reader who has never seen the chat should be
able to understand the problem, what we measured, what we concluded, what is still wrong,
and what to try next.

---

## 1. The problem we set out to solve

The physical cartpole is controlled by RPGD (an MPC optimizer). In **simulation**
(plant == model) RPGD swings up and balances cleanly. On **hardware**, RPGD only works
with one specific, hand-tuned parameter set:

| parameter | "working" (hand-tuned) | meaning |
|-----------|------------------------|---------|
| `m_cart`  | 0.230 kg | effective translational mass |
| `u_max`   | 1.77 N   | force at command `Q = 1` |
| `M_fric`  | 3.22 N/(m/s) | viscous + back-EMF damping |

These numbers are **physically wrong in absolute magnitude** (the cart clearly does not
weigh 230 g once you include the reflected rotor inertia), yet they work. The goal was NOT
to retune RPGD — it was to understand *why* honest, independently-measured parameters
break a controller that simulation says should work, and to produce an honest model that
still drives the hardware. The user was explicit: "whole effort was just to make it work
as in simulation … We did something wrong I think. We must find it."

### The key invariant

RPGD's behaviour is governed almost entirely by the **command-to-acceleration gain**

```
accel / Q  =  u_max / m_cart        [m/s^2 per unit Q]
```

For the working set this is `1.77 / 0.230 = 7.70 m/s^2`. Any honest parameter set must
reproduce roughly this normalized gain, or RPGD will systematically under- or over-drive
the cart. This single ratio turned out to be the thread that ties every experiment together.

---

## 2. Why the first "honest" attempt failed

The original honest values came from a bare-cart **step response** calibration:
`m_cart=2.82`, `u_max=25.8`, `M_fric=42.4`.

* Normalized gain: `25.8 / 2.82 = 9.15 m/s^2` — ~19% higher than the working 7.70.
* On hardware this made RPGD command **gentler** Q (it thinks each unit of Q buys more
  acceleration than it does), so it under-drove the cart and failed to swing up.

The user recorded the failure (`CPP_mpc__2026-06-20_21-19-16.csv`): a very late, almost
accidental swing-up. Reverting to the working set restored a clean swing-up
(`CPP_mpc__2026-06-20_21-02-10.csv`).

### Root cause of the over-estimated `u_max`: collinearity

In an ordinary step response you hold `Q` constant and let `v` ramp up. So within each
step `Q` and `v` are **collinear** (correlation ≈ 0.76). The joint regression

```
M·a = u_max·Q − M_fric·v − C·sign(v)
```

cannot separate the `u_max·Q` term from the `−M_fric·v` term when `Q` and `v` move
together — it trades force gain against damping and lands on an inflated `u_max` (25.8 N).
This is the central methodological error we were chasing out of the calibration.

---

## 3. Effective mass (this part is solid)

`m_cart` is not the weighable cart mass — it is the **effective translational inertia**
(cart body + belt + pulleys + motor rotor inertia reflected through the 19:1 gearbox and
~16 mm pulley). It cannot be measured on a scale, so we identified it by **adding known
masses** and watching how the step-response acceleration drops:

* `CPP_step_response-2.csv` — bare cart, no pole, no added mass (baseline)
* `CPP_step_response-5.csv` — +227 g
* `CPP_step_response-9.csv` — +691 g

For a fixed force, `a = F / (m_cart + m_added)`, so `1/a` is linear in `m_added` and the
intercept gives `m_cart`. This yields **`m_cart ≈ 2.82 kg`** consistently. See
`../estimate_effective_mass.py` and the effective-mass section of `../README.md`.

The pole (86.46 g) and its hinge (14.12 g) were physically removed for all ID runs so the
recorded acceleration is purely the longitudinal motor + cart-friction dynamics. The
hinge sits near the rotation axis (negligible rotational inertia) and is treated as part
of the cart mass.

**Pole back-reaction check:** with the honest `m_cart`, the pole's reaction force on the
cart is ~3% of the drive force (it was a misleading 13–69% under the tiny 0.230 kg mass).
So decoupling the longitudinal ID from the pole is justified.

---

## 4. The three identification experiments

To beat the collinearity we built three new on-hardware protocols. All run **pole-removed,
no added mass**, all log the true applied PWM (`actualMotorSave`) so the fit uses measured
power regardless of the `Q→PWM` mapping, and all re-run the firmware position calibration
between blocks because the encoder zero drifts over a long run.

### 4a. EMF / force-velocity shuttle — `emf_identification_experiment.py`
Decorrelates `Q` and `v` by letting the cart **accelerate across the full track** (reaching
swing-up speeds ~1.3 m/s) while the power **magnitude is randomized** and **coast (p=0)
segments** are injected. The turn at each wall is done with a **known open-loop reverse
power** — force and velocity then have opposite signs, the maximally decorrelated regime.
Boundary protection is **stopping-distance based** (`v²/2a`), not fixed look-ahead.
Analysis: `../estimate_emf.py`.

* `CPP_emf_identification-1.csv` — first run. Had a **direction-desync bug** (the `dir`
  excitation flag could disagree with actual travel, so the brake watched the wrong wall →
  boundary excursions). Kept for the record.
* `CPP_emf_identification-2.csv` — after decoupling boundary protection from `dir` (brake
  now depends on actual `(position, velocity)` toward whichever wall is being approached).
  This is the clean run. `corr(|PWM|,|v|)` dropped to ~0.40.

Result of the shuttle fit: `u_max ≈ 13 N`. But the shuttle **never dwells at steady high
speed**, so it under-constrains `M_fric` (it returned ~16, too low).

### 4b. Step-response terminal (saturation) velocity — `plot_saturation_velocity.py`
The cleanest single measurement of the **damping ratio**: at the end of each constant-`Q`
step the cart provably reaches steady state (`a ≈ 0`), so `u_max·Q = M_fric·v_terminal`,
i.e. `v_terminal = (u_max/M_fric)·Q`. From `CPP_step_response-2.csv` this is **perfectly
linear** up to the safe PWM limit, with slope `k/M_fric = 9.42e-5 m/s per PWM count`,
giving terminal speed at `Q=1` of **~0.59 m/s**. This confirms `Q=1` is still in the
linear (non-saturated) regime — one of the user's explicit questions.
Plot: `data/CPP_step_response-2_vsat_vs_Q.png`.

Combining `u_max=13` with this ratio gives `M_fric ≈ 22`, hence the current yml values.
The EMF "terminal velocities" that appeared *above* this line were transient coast-downs,
not true steady state — see the separated transient/terminal panels in
`data/CPP_motor_id_summary.png`.

### 4c. Velocity-conditioned pulse test — `pulse_identification_experiment.py`
Designed to nail the **absolute force gain** `u_max` without collinearity: bring the bare
cart to a target velocity, then apply a short (140 ms) known-power pulse and read the
**instantaneous acceleration** from the early pulse window (after actuation latency, before
`v` drifts). Targets `v ∈ {0, ±0.2, ±0.4, ±0.6} m/s`, pulse powers `±{0.35,0.60,0.85}` of
full PWM. Includes `MAX_PREP_ATTEMPTS` / skip logic so it can't get stuck retrying an
unreachable target near a boundary. Analysis: `../estimate_pulse_identification.py`.

* `CPP_pulse_identification-1.csv`.

Result: **`u_max ≈ 21.6 N`, `M_fric ≈ 36.5 N/(m/s)`** (latency-dependent, see below).
Crucially the normalized gain `21.6 / 2.82 = 7.66 m/s²` **almost exactly matches the
working set's 7.70 m/s²** — strong independent evidence that the pulse number is the most
trustworthy `u_max`, and that the working RPGD was implicitly using ~7.7 m/s².

---

## 5. Summary of estimates

| method | `u_max` [N] | `M_fric` [N/(m/s)] | `accel/Q = u_max/m_cart` [m/s²] | note |
|--------|-------------|--------------------|---------------------------------|------|
| working hand-tuned (m=0.230) | 1.77 | 3.22 | **7.70** | only config confirmed to work |
| step response (collinear)    | 25.8 | 42.4 | 9.15 | `u_max` inflated by Q–v collinearity |
| EMF shuttle (decorrelated)   | ~13  | ~16  | 4.6  | `M_fric` under-constrained (no steady high-v) |
| step terminal-velocity ratio | —    | (→22 given u=13) | — | clean damping ratio `k/M_fric` |
| **pulse (instantaneous a)**  | **~21.6** | **~36.5** | **7.66** | matches working normalized gain |
| **current yml (does NOT work)** | 13.0 | 22.0 | 4.6 | EMF `u_max` + terminal-ratio `M_fric` |

**The decisive observation:** the only sets whose normalized gain `u_max/m_cart` lands
near **7.7 m/s²** are the working hand-tuned set and the **pulse** fit. The current yml
(`13/2.82 = 4.6`) is ~40% too weak — exactly the "motor seems too weak now" symptom the
user reported. The yml should very likely be `u_max ≈ 21.5`, `M_fric ≈ 36`, `v_max ≈ 0.59`,
**not** the `13 / 22` it currently holds. This change was identified but **not yet
validated on hardware** — hence the unresolved status.

---

## 6. Remaining uncertainty: actuation latency

The pulse `u_max` is sensitive to **how much of the early pulse window is discarded** as
command/actuation latency. Sweeping the assumed latency moves `u_max` from ~14 N (long
latency, more early samples kept that are still ramping) to ~29 N (very short latency).
The ~21.6 N figure uses a mid latency that also reproduces the working normalized gain, but
the latency itself has not been measured directly. **This is the single biggest open
question** and the reason we cannot yet commit a final `u_max`.

---

## 7. Plan looking forward

1. **Measure actuation latency directly.** From a dead stop, command a step and find the
   delay between the command timestamp and the first sample with `|a|` above the noise
   floor. Do this for several power levels. This pins down which slice of the pulse window
   is valid and collapses the 14–29 N range to a single `u_max`.
2. **Adopt the pulse-consistent set and validate on hardware.** Set
   `u_max ≈ 21.5`, `M_fric ≈ 36`, `m_cart = 2.82`, `v_max ≈ 0.59` (normalized gain ≈ 7.7,
   matching the working RPGD), and record a swing-up. This is the most likely-to-work
   honest set and should be tried first.
3. **If swing-up still fails**, the residual mismatch is probably not in `u_max/m_cart`
   but in (a) the missing **back-EMF / `v_max`** term not being applied in the model used by
   RPGD (`v_max` is noted in the yml as "not implemented in model, but needed for MPC"), or
   (b) **MOTOR_CORRECTION / PWM saturation** at high Q. Verify the model actually consumes
   `v_max`, and confirm `Q=1` maps below `MOTOR_FULL_SCALE_SAFE`.
4. **Re-run pulse + EMF with a measured latency** and re-fit; the EMF shuttle and pulse
   should then agree on `u_max`. If they do, that is the final, defensible number.
5. Only as a last resort, **re-tune the RPGD cost** for the honest model (more aggressive
   control weighting) — but the whole point was to avoid retuning, so do this only after
   the parameter set is provably consistent.

---

## 8. File inventory

### Data (`data/`)
| file | what it is |
|------|------------|
| `CPP_step_response-2.csv` | bare cart, no pole, no mass — baseline (mass-ID + terminal velocity) |
| `CPP_step_response-5.csv` | +227 g — effective-mass identification |
| `CPP_step_response-9.csv` | +691 g — effective-mass cross-check |
| `CPP_emf_identification-1.csv` | first EMF shuttle (direction-desync bug; for record) |
| `CPP_emf_identification-2.csv` | clean decorrelated EMF shuttle |
| `CPP_pulse_identification-1.csv` | velocity-conditioned pulse test |
| `CPP_emf_identification-2_emf_fit.png` | EMF fit diagnostic plot |
| `CPP_motor_id_summary.png` | combined summary (EMF transients vs. step terminal points) |
| `CPP_step_response-2_vsat_vs_Q.png` | terminal velocity vs. Q (linearity of Q=1) |

### New experiment protocols (`Driver/DriverFunctions/ExperimentProtocols/`)
| file | purpose |
|------|---------|
| `emf_identification_experiment.py` | full-track decorrelated force-velocity shuttle |
| `pulse_identification_experiment.py` | velocity-conditioned instantaneous-force pulses |
| `experiment_protocols_selector.py` | registers the two protocols above (modified) |
| `step_response_experiment.py` | added recalibration between series (modified) |

### New / modified analysis scripts (`Driver/DataAnalysis/MotorAndCartFriction/`)
| file | purpose |
|------|---------|
| `estimate_effective_mass.py` | effective mass from added-mass step responses |
| `estimate_emf.py` | force-velocity / back-EMF fit from the EMF shuttle |
| `plot_saturation_velocity.py` | terminal velocity vs. Q (damping ratio, linearity) |
| `estimate_pulse_identification.py` | constrained & free `u_max`/`M_fric` from pulses |
| `MotorCalibration.py` | added mass-ID helpers (modified) |
| `README.md` | effective-mass section added (modified) |
| `validate_forward_prediction.py` | **NEW** forward-prediction arbiter using `predictor_ODE` (Step 2/3/4b) |
| `refit_motor_forward.py` | **NEW** forward velocity-trajectory motor re-fit (Step 4) |

### Parameter state at time of writing (NOT committed — live working files)
* `Driver/CartPoleSimulation/cartpole_physical_parameters.yml` (submodule):
  `m_cart=2.82`, `u_max=13.0`, `M_fric=22.0`, `v_max=0.59`, `J_fric=5.0e-5`
  — **does not work on hardware** (normalized gain 4.6 vs. needed ~7.7).
* `Driver/globals.py`: `MOTOR_CORRECTION_POLOLU = (0.6216901, 0.0750750, 0.0549491)`,
  `MOTOR_FULL_SCALE_SAFE = int(0.95 * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES)`.
* Confirmed-working RPGD parameters (to restore the hardware to a usable state):
  `m_cart=0.230`, `u_max=1.77`, `M_fric=3.22`.

---

## 9. RESOLUTION (June 2026) — offline forward-prediction validation

The contradiction in section 5 is **resolved offline** by using the project's own
`predictor_ODE` (the exact analytical model RPGD integrates) as an *arbiter*: feed each
recording its true initial state and recorded `Q`, roll the model forward, and measure
multi-step prediction error against the recorded motion. The model that best reproduces
the **real plant** is the honest one — independent of whether a controller "worked". Two
new scripts implement this:

* `validate_forward_prediction.py` — the arbiter (Steps 2, 3, 4b).
* `refit_motor_forward.py` — a forward velocity-trajectory re-fit of the motor law (Step 4).

### 9.1 Which parameter set reproduces the real cart? (arbiter)

Multi-step (500 ms) forward-prediction RMSE on the **real working swing-up**
`CPP_mpc__2026-06-20_21-02-10.csv` (pole attached):

| set | m_cart / u_max / M_fric | position RMSE [m] | positionD RMSE | angle RMSE [rad] |
|-----|-------------------------|-------------------|----------------|------------------|
| A old hand-tuned | 0.230 / 1.77 / 3.22 | 0.0059 | 0.0294 | 0.0472 |
| B EMF (current yml) | 2.82 / 13.0 / 22.0 | 0.0017 | 0.0082 | 0.0455 |
| **C pulse / invariant** | **2.82 / 21.7 / 36.8** | **0.0009** | **0.0049** | **0.0447** |
| D forward-fit | 2.82 / 19.0 / 29.6 | 0.0011 | 0.0053 | 0.0448 |

The heavy-mass sets (C, D) predict the real cart **~6x** better than the old light-mass set
A and **~2x** better than the EMF set B that is currently in the yml. **B (u_max=13) is
clearly inferior** — settling the question. The same ordering holds on the pole-removed
step and pulse recordings. Note A has the *same* acceleration gain as C (7.70 m/s²) yet
predicts far worse: the gain is not enough — the **absolute mass** (pole back-reaction)
matters, which is exactly what forward prediction exposes and a "working" controller hides.

### 9.2 The pole's effect on the cart really is small (Step 3)

Magnitude-weighted pole back-reaction as a fraction of the cart's horizontal acceleration,
on actively-driven samples of the swing-up:

| set | back-reaction on driven cart accel | abs. trajectory change over 500 ms |
|-----|-----------------------------------|------------------------------------|
| A (m=0.230) | 36.6% | mean 1.40 mm, max 60.6 mm |
| B (m=2.82) | 5.5% | mean 0.18 mm, max 9.1 mm |
| C (m=2.82) | 3.8% | mean 0.13 mm, max 5.9 mm |

With the honest heavy mass the pole changes the cart acceleration by only **~4%** (single-mm
over half a second), confirming section 6's ~3% estimate **through the actual predictor**.
With the old 0.230 kg mass it is **~37%** (cm-scale) and cannot be dropped — which is also
why set A mispredicts the real cart despite the right gain. Consequence: with the honest
mass the cart may be modeled as a standalone 2nd-order system (useful for embedded predictors).

### 9.3 Why the EMF `u_max = 13` was wrong, and what the motor force really is (Step 4)

`refit_motor_forward.py` fixes the robust `m_cart = 2.82` and fits `(force_gain, M_fric,
Coulomb)` by minimizing a **multi-step forward rollout of the velocity**, jointly across the
step + pulse + EMF recordings. This fixes all three defects of the old EMF fit:

1. **No second differentiation.** It uses `positionD` directly (one derivative). The old
   `estimate_emf.py` differentiates `positionD` again to get acceleration — doubling noise.
2. **Collinearity broken by joining recordings.** The step run alone has `corr(Q,v)=0.77`;
   the pulse (0.50) and EMF shuttle (0.31) cover the rest of the `(command, velocity)` plane,
   so the joint fit can separate force gain from damping.
3. **M_fric properly constrained.** The step dwell pins the damping that the EMF shuttle
   alone (no steady high-speed dwell) left under-determined and biased low (→ M_fric≈16,
   dragging u_max down to 13).

Result (R² = 0.92 on multi-step velocity, **both domains agree**):

| domain | u_max [N] | M_fric [N/(m/s)] | v_term(Q=1) | gain u_max/m_cart |
|--------|-----------|------------------|-------------|-------------------|
| Q-domain (model law `u=u_max·Q`) | 19.0 | 29.6 | 0.64 | 6.7 |
| PWM-domain (`u_max=k·gain·PWM`)   | 19.4 | 31.1 | — | 6.9 |

For contrast, the **old one-step acceleration regression** (the discredited method) still
scatters wildly across recordings — step 18 N, pulse 17 N, EMF 14 N, with `M_fric` of
27 / 3.5 / 16 — exactly the "inconclusive, contradictory" picture. The forward-fit removes
that scatter.

### 9.4 Verdict and recommended parameters

Every trustworthy line of evidence now agrees on a **heavy effective mass and a motor force
in the ~19–22 N band**:

| evidence | u_max | gain | note |
|----------|-------|------|------|
| working hand-tuned (gain only) | 1.77 (m=0.230) | 7.70 | proven authority, wrong absolutes |
| pulse / 3-invariant (set C) | 21.7 | 7.70 | won the arbiter; v_term=0.59 exactly |
| forward velocity-fit (set D) | 19.0 | 6.7 | most rigorous offline, full Q-range |
| swing-up velocity optimum | ~22 | ~7.8 | direct on the real plant |
| ~~EMF shuttle (current yml)~~ | ~~13~~ | ~~4.6~~ | **discarded** — see 9.3 |

**Recommended set (NOT yet active in the yml — pending one hardware retest):**
`m_cart = 2.82`, `u_max = 21.7`, `M_fric = 36.8`, `v_max = 0.59`. This is the arbiter winner
(set C) and is internally consistent on all three clean invariants: effective mass 2.82
(added-mass ratio), terminal-velocity ratio `u_max/M_fric = 0.59` (step saturation), and
acceleration gain `u_max/m_cart = 7.70` (the exact authority of the only set ever confirmed
to drive the hardware). The EMF `u_max = 13` is ~40% too weak (gain 4.6) and is the cause of
the "motor seems too weak now" symptom.

> NOTE: to keep the hardware in a known-working state, the live
> `cartpole_physical_parameters.yml` is intentionally left at the **proven working set**
> (`m_cart=0.230`, `u_max=1.77`, `M_fric=3.22`, same gain 7.70). To run the hardware retest,
> set the recommended values above; `validate_forward_prediction.py` reproduces the ranking
> and `refit_motor_forward.py` re-derives the numbers.

### 9.5 Single remaining gate (deferred, offline-first)

The only residual uncertainty is **actuation latency**, which places the absolute force
within the 19–22 N band (the forward-fit's 19 vs the pulse/arbiter's ~21.7). To close it:

1. Measure actuation latency directly (command a step from rest; time to first
   above-noise acceleration, several power levels).
2. One confirming hardware swing-up with the recommended set. Because its gain (7.70) equals
   the proven-working gain and the pole back-reaction is now only ~4%, it should behave at
   least as well as the old set — unlike last week's broken 25.8 N (gain 9.15) and 13 N
   (gain 4.6) attempts. The previous working set is preserved in the yml comments for
   instant rollback.
