# Calibration

Three things matter for control: track center, hanging / zero angle, and the
motor map. Friction identification is optional for a first working controller;
see [Driver/DataAnalysis/MotorAndCartFriction/README.md](../Driver/DataAnalysis/MotorAndCartFriction/README.md).

Day-to-day keys and buttons: **`K`** / **BTN5** (track center), **`b`** / **BTN0**
(hanging), and the [operating guide](operating.md).

## Track center

Not stored. Recalibrate after every power cycle (`K` on the PC, **BTN5** on Zybo).
The cart drives to both ends and stops in the middle. If it stalls near center,
calibration speed in firmware may be too low for friction.

`K` / BTN5 never change `ANGLE_HANGING`.

## Motor type (ORIGINAL vs POLOLU)

Calibration also detects encoder sign and sets `MOTOR`. That value is hardcoded
**independently** in firmware (`parameters.c`) and in `globals.py`.

* Starting the Python driver sends the PC motor type to the chip and overwrites
  firmware RAM.
* Calibration from the PC updates both sides for that session.
* Calibration from a board button updates firmware RAM only.
* After reset, each side reloads its compile-time / file default.

In the lab, motor type also selects which robot’s hanging constants apply
(`ANGLE_HANGING_POLOLU` vs `ANGLE_HANGING_ORIGINAL`).

## Motor power

Each motor (and wear) changes the Q → acceleration map. Record a bidirectional
step response (`m` until the step-response protocol is selected, then `n`),
then follow
[Driver/DataAnalysis/MotorAndCartFriction/README.md](../Driver/DataAnalysis/MotorAndCartFriction/README.md).
Paste `MOTOR_CORRECTION` into `globals.py` **and**
[parameters.c](../Firmware/Src/CartPoleFirmware/parameters.c) (reflash for on-chip).
Show-mux profiles currently share the LSTM/RPGD map
`{0.5733488, 0.0257380, 0.0258429}`.

## Angle: dead zone

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

## Full circle (`ANGLE_360_DEG_IN_ADC_UNITS`)

The ADC is 12-bit, but the dead zone means a physical turn is more than 4096
counts. Measure hanging vs upright on the side that does not include the wrap;
the circle is twice that difference. Script:
[Driver/DataAnalysis/AngleUpDown](../Driver/DataAnalysis/AngleUpDown).

Firmware **does** define `ANGLE_360_DEG_IN_ADC_UNITS` in `parameters.c`. It
must match `globals.py`. Current Zybo Development values (new analog chain,
2026-09-03): hanging **3273.353**, circle **4068.73**. STM and Zybo numbers
differ (different robots and/or the 3.3 V → 1 V divider).

## Zero angle

Hang the pole, then **BTN0** or **`b`**. Gently balance upright: the reported
angle should oscillate around 0. Fine-tune with `=` / `-`, or edit
`ANGLE_HANGING_*` and restart.

## Checklist (first working controller)

1. Dead zone at horizontal; pot clamp tight.
2. **`K`** or **BTN5** — track center.
3. **BTN0** or **`b`** — hanging capture.
4. Balance upright; angle reads ~0 rad (`=` / `-` if needed).
5. Optional: step-response motor map → paste into `globals.py` and `parameters.c`.
