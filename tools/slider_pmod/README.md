# Zybo JB slider → CartPoleFirmware `target_position`

This is the calibration path for the **main** Zybo `CartPoleFirmware`, not the standalone Slider_test app.

The physical Pmod slider on **JB** is read by PmodAD1 (`0x83C30000`). With `USE_EXTERNAL_INTERFACE` defined in `Firmware/Src/CartPoleFirmware/hardware_bridge.h`, every control cycle sets

```
target_position = get_normed_slider_state() * SliderTargetHalfLength  /* ±0.12 m */
```

in `Firmware/Src/CartPoleFirmware/control.c`. That **overwrites** a PC `CMD_SET_TARGET_POSITION` while the flag is on. The driver shows the chip target. Leave on-chip control off while checking the map.

## Map

`Firmware/Src/Zynq/external_interface.c` (must match `slider_curve.py`):

1. Decode ADC: `(ch0 >> 1) & 0xFFF`.
2. Affine: `normed = 2 * (adc - SLIDER_ADC_LEFT) / (SLIDER_ADC_RIGHT - SLIDER_ADC_LEFT) - 1`.
   Zero is the electrical mid `(LEFT+RIGHT)/2`.

Measured rails (2026-09-01): the slider saturates the 12-bit ADC (**0 … 4095**). Target 0 is ADC 2047.5.

| Slider | Normed | `target_position` |
|---|---|---|
| Left stop (min ADC) | −1 | −0.12 m |
| Electrical mid | 0 | 0 |
| Right stop (max ADC) | +1 | +0.12 m |

RGB: green if target > 0, blue if < 0.

## Check the map

Close Vitis (it steals JTAG) and the cartpole GUI (it holds the PS UART).

```bash
tools/slider_pmod/program_cartpole_slider.sh
python3 tools/slider_pmod/measure_slider_linear.py
```

UART is **230400** on the Digilent FTDI **interface 1** (usually `/dev/ttyUSB1`). Park still, then press Enter at LEFT and RIGHT. Expect about −0.12 m and +0.12 m.

Live print: `python3 tools/slider_pmod/measure_slider_linear.py --watch`

Optional affinity sweeps (Slider_test bitstream, 115200):

```bash
python3 tools/slider_pmod/log_slider_curve.py --out tools/slider_pmod/data/YYYY-MM-DD
python3 tools/slider_pmod/plot_slider_curve.py tools/slider_pmod/data/YYYY-MM-DD
```

## Bitstream

PmodAD1 on the cartpole design is clocked at **100 MHz**. SPI counts must be **40 / 120 / 1000 / 800** (same pin timing as 20 @ 50 MHz). Counts of 20 @ 100 MHz break AD7476 framing (`0x3000` / `0x31FF`).

Set that on the **BD cell**, not only in Verilog defaults:

```bash
# live secloc project
vivado -mode batch -source FPGA/VivadoProjects/set_pmodad1_timing_and_build.tcl
```

The Zybo recreate script `FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_12_09_2025.tcl` writes the same `CONFIG.AD1_CLOCKS_*`.

## Pins

Same JB mapping as Slider_test: `cs=Y7`, `sclk=W6`, `ad1_d0=Y6`, `ad1_d1=V6`.
