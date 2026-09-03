# Motor Calibration and Cart Friction

The script MotorCalibration.py allows for
motor calibration so that cart appears
to have no sliding friction
and the relationship between acceleration and normed input is linear up to rolling friction: \
dv/dt = a * Q - b * v \
It uses actual motor input (motor_input) for its calculations. \
You need to update the calculated values manually in the driver of physical cartpole!

The script CheckMotorCalibration.py
let you check if the calibration done with MotorCalibration.py
brought intended results.
It uses normed control input Q.

GetForceAndFriction.py helps to determine the values related to motor -
parameter mapping from normed motor input Q to physical force acting on the cart
and parameter of rolling friction of the cart - needed for the simulation.

All above files require a measurement of cart accelerating in both directions
with piecewise constant motor command (to achieve saturation velocity).
Record a bidirectional step response with `python Driver/control.py`:
press **`m`** until the terminal shows `step-response`, then **`n`** to run it.
See [Docs/pc-driver.md](../../../Docs/pc-driver.md#experiment-protocols).

As a matter of example we provide two csv recordings - 
Original.csv and Pololu.csv -
taken with cartpoles we work while creating this files.
With these files the scripts above are tested to work without known problems.
Replace them with a measurement taken with your cartpole.

## Canonical procedure for neural control (force-fit)

For the neural-imitator controller the canonical calibration is the **force-based** fit,
because it anchors Q=1 to the force scale the network was trained with (u_max_target):

1. Record a fresh bidirectional step response (step_response_experiment) and save it as
   `CPP_step_response.csv` in this folder (the committed, reproducible recording).
2. Run: `python MotorCalibration.py --force-fit`
   (uses u_max_target and effective_mass set near the top of MotorCalibration.py;
   PWM must match the chip: 10000 Zynq / 7200 STM).
3. Paste the printed `MOTOR_CORRECTION` tuple into BOTH:
   - `Driver/globals.py` → `MOTOR_CORRECTION_POLOLU` (PC neural control), and
   - `Firmware/Src/CartPoleFirmware/parameters.c` → `MOTOR_CORRECTION` (on-chip; requires reflash).
4. The force-fit also prints a linear-range check; `Q=±1` should report `OK`.

After a motor swap, repeat steps 1–3 to get values that make the network work again.
`effective_mass` is the cart+pole+drivetrain effective mass assumption; keep it consistent
with the simulation/training, since it sets the absolute gain.

double_regression.py is a helper file used by MotorCalibration.py.
It is a linear regression algorithm which fits two lines to two datasets,
while imposing that the two lines are required to have the same slope; 
they are only allowed to have different intercept.
The proposed algorithm finds two such lines
which _together_ minimize the _total_ MSE of _both_ datasets.
This kind of regression is needed while fitting the lines to 
data containing sliding friction, which is the case for our cartpoles.

---

# Effective cart mass: why it is ~2.8 kg, not 0.23 kg (and how we measured it)

> Identified 20.06.2026 on the ZYNQ + Pololu #4751 cart. Reproduce with
> `python estimate_effective_mass.py` (uses the two committed recordings
> `CPP_massID_baseline.csv` and `CPP_massID_loaded_327p73g.csv`).

This section explains, from first principles, a subtle but important fact about
this machine: **the number that belongs in `m_cart` of the cart-pole equations is
not the weight of the cart body (~0.23 kg). It is an *effective* translational mass
of ~2.82 kg**, because the spinning motor rotor "feels" like extra mass to the cart.
Below we (1) build the intuition, (2) derive the formula, (3) show how to measure it
cleanly, (4) cross-check it analytically, and (5) explain why it is legitimate to use
it directly as the cart mass.

## 1. The puzzle

The cart body, weighed on a scale, is ~0.23 kg. Yet every dynamic measurement says
the cart behaves as if it were **~12x heavier**. For a long time this was hidden,
because the controllers were (accidentally) tuned in *acceleration space* — see §6.

## 2. Where the hidden mass comes from: reflected rotor inertia

The drive pulley sits on the **gearbox output shaft**. The motor armature (rotor)
spins `G = 19x` faster than that shaft, and the pulley turns rotation into translation
through its radius `r ~= 16 mm`. Energy bookkeeping tells us how a rotating part adds
to the cart's *linear* inertia.

If the cart moves at speed `v`, the pulley turns at `omega_pulley = v / r`, and the
rotor turns at `omega_rotor = G * v / r`. The rotor's kinetic energy is

```
KE_rotor = 1/2 * J_rotor * omega_rotor^2
         = 1/2 * J_rotor * (G/r)^2 * v^2
         = 1/2 * m_reflected * v^2 ,   with   m_reflected = J_rotor * G^2 / r^2 .
```

So a rotating inertia `J_rotor` *adds* a translational mass `m_reflected = J_rotor*G^2/r^2`
to the cart. The amplification factor is enormous here because `r` is small:

```
G^2 / r^2 = 19^2 / 0.016^2 ≈ 1.41e6  (1/m^2)
```

A perfectly ordinary armature inertia `J_rotor ~ 2e-6 kg·m^2` therefore reflects to
`~2.8 kg` at the cart. The same logic applies (with `omega = v/r`, no `G`) to the two
belt pulleys, but for a disk `I = 1/2 m r^2` gives `m_reflected = 1/2 m` — only ~24 g
each. The belt's own mass (it all moves at speed `v`) adds in full but is small.

```
M_eff ≈ m_cart_body (0.23) + belt + pulleys (~0.05) + J_rotor*G^2/r^2 (~2.5)  ≈ 2.8 kg
```

## 3. Measuring it cleanly (the matched-(u,v) ratio)

Direct system identification of `M` from a single run is hard: in a step response the
motor command `u` and the velocity `v` are strongly correlated (a bigger push → a
higher top speed), so a regression cannot cleanly separate the force term `k*u` from
the damping term `b*v` (collinearity). The trick is to **never rely on that split for
the mass**. Instead we run the *same* drive twice — once bare, once with a precisely
known extra mass `dm` — and take a ratio.

Per-sample longitudinal dynamics in each run (Newton's law for the cart):

```
M * a = k*u - b*v - c*sign(v)        =>      a = (k/M)*u + (-b/M)*v + (-c/M)*sign(v)
```

Fit `alpha = k/M` (the coefficient on `u`) in each run with a plain least-squares
regression that includes `v` and `sign(v)` as nuisance regressors. The motor constant
`k` is a property of the motor and does not change between the two back-to-back runs,
so:

```
alpha_base / alpha_load = (k/M_base) / (k/M_load) = M_load / M_base
```

and since `M_load = M_base + dm`:

```
M_base = dm / (alpha_base/alpha_load - 1) ,     M_load = M_base + dm .
```

Why this is robust (and why it succeeded where everything else failed):
- The **collinearity is identical in both runs** (same drive, and the saturation speed
  is mass-independent), so it cancels exactly in the ratio.
- Back-EMF, viscous and Coulomb friction sit in the nuisance terms `b, c` — they do not
  enter `alpha`.
- Even the *increase* of friction caused by the added mass only shifts `c` (the
  `sign(v)` term), not `alpha`.

Procedure that produced the committed data:
1. Record a dense **bidirectional** step response with the bare cart (no pole),
   8 sweeps, re-zeroing the encoder between sweeps (`step_response_experiment.py`
   with `NUM_REPEATS=8`, `RECALIBRATE_BETWEEN_SERIES=True`). → `CPP_massID_baseline.csv`
2. Place a known mass on the cart (here `dm = 327.73 g`) and repeat with **identical
   settings**. → `CPP_massID_loaded_327p73g.csv`
3. `python estimate_effective_mass.py`

Result (≈20 000 driven samples per run, 4000-sample bootstrap):

```
alpha_base/alpha_load = 1.116
M_base = 2.82 kg   (95% CI [2.43, 3.36]),   M_load = 3.14 kg
```

The measured ratio `1.116` matches the ideal `(2.82+0.327)/2.82 = 1.116` essentially
exactly — a strong internal consistency check.

## 4. Analytic cross-check

Invert the result: subtracting the body+belt+pulleys (~0.33 kg) leaves
`m_reflected ≈ 2.5 kg`, which implies

```
J_rotor = m_reflected * r^2 / G^2 = 2.5 * 0.016^2 / 19^2 ≈ 1.8e-6 kg·m^2
```

— a textbook armature inertia for a motor this size. Independently, the no-load speed
(530 rpm output = 55.5 rad/s) times `r=0.016 m` gives `0.89 m/s`, matching the
observed top speed — confirming `r ≈ 15–16 mm`. Pololu does not publish `J_rotor`, so
the dynamic measurement (§3) is the primary number and this is the sanity bracket.

## 5. Why this effective mass is the correct `m_cart`

In the cart-pole equations the cart's translational inertia multiplies `x_ddot`.
A rigidly-coupled rotating part contributes to that same `x_ddot` term exactly like
added cart mass (this is what the energy derivation in §2 proves — its kinetic energy
is `1/2 m_reflected v^2`, indistinguishable from translational mass). Therefore the
effective mass can be substituted directly for `m_cart`, **and it must be**, because
the controllers that predict the cart's motion need the inertia the cart actually has.

The honest model parameters derived from the same data (see Method 2 in the script,
restricted to the linear PWM range; the force/friction split carries ~±15%
collinearity uncertainty, unlike the mass which is robust):

| parameter | old (effective, ~12x off) | identified honest value |
|---|---|---|
| `m_cart` | 0.230 kg | **2.82 kg** (CI [2.43, 3.36]) |
| `u_max`  | 1.77 N   | **25.8 N** (≈ half the 52 N geared stall, H-bridge limited) |
| `M_fric` | 3.22 N/(m/s) | **42.4 N/(m/s)** (mostly reflected motor back-EMF/gearbox damping) |

These are written into `Driver/CartPoleSimulation/cartpole_physical_parameters.yml`.

## 6. Bonus: why the old (wrong) mass still worked, and the pole back-reaction

The pole obeys `theta_ddot = (g·sin θ − x_ddot·cos θ)/L_eff`, which contains **no cart
mass** — the pole only responds to the cart *acceleration*. The cart's acceleration is
`≈ u_max·Q / m_cart`, so only the **ratio** `u_max/m_cart` matters for the dominant
behavior. The old model had both `u_max` and `m_cart` wrong by the same ~12x factor, so
the ratio (the acceleration gain) was accidentally right and the motor calibration
absorbed the rest. It only worked "barely" because the *absolute* mass governs the
pole→cart back-reaction (the `m_pole·L·theta_ddot·cos θ` coupling), whose relative size
is `m_pole/(m_cart+m_pole)`:

- old model: `0.087/0.317 ≈ 27 %` (pole strongly shoves the cart)
- honest model: `0.087/2.9 ≈ 3 %` (heavy effective cart barely notices the pole)

Numerically, dropping the back-reaction changes the predicted cart acceleration by only
~3 % on average (≤ ~11 % in the most violent centrifugal part of a swing-up) with the
honest mass — *smaller than the ±17 % uncertainty on the mass itself*. So with the
correct effective mass the cart and pole equations may be **decoupled** (cart = a
simple force-driven second-order system, pole = a pendulum driven by the cart's
acceleration) with negligible loss of fidelity — useful for embedded/compute-bound
predictors. (With the old 0.23 kg mass the same approximation would have caused a
13–69 % error, which is why it could never be dropped before.)